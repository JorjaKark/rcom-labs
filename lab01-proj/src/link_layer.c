// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


// ----- Frame constants -----
#define FLAG 0x7E

// Address field (commands from Tx or replies from Rx = 0x03)
#define A_TX_CMD_OR_RX_REPLY 0x03
// Address field (commands from Rx or replies from Tx = 0x01)
#define A_RX_CMD_OR_TX_REPLY 0x01

// Control field values (S/U frames)
#define C_SET  0x03
#define C_UA   0x07
#define C_DISC 0x0B
#define C_RR0  0xAA
#define C_RR1  0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55

typedef enum { ST_START, ST_FLAG, ST_A, ST_C, ST_BCC_OK, ST_END } SuState;

static void su_reset(SuState *st) { *st = ST_START; }

static int su_feed(SuState *st, unsigned char byte,
                   unsigned char expectedA, unsigned char expectedC) {
    static unsigned char A, C;
    switch (*st) {
        case ST_START:
            if (byte == FLAG) *st = ST_FLAG;
            break;
        case ST_FLAG:
            if (byte == expectedA) { A = byte; *st = ST_A; }
            else if (byte != FLAG) *st = ST_START;
            break;
        case ST_A:
            if (byte == expectedC) { C = byte; *st = ST_C; }
            else if (byte == FLAG) *st = ST_FLAG;
            else *st = ST_START;
            break;
        case ST_C:
            if (byte == (unsigned char)(A ^ C)) *st = ST_BCC_OK;
            else if (byte == FLAG) *st = ST_FLAG;
            else *st = ST_START;
            break;
        case ST_BCC_OK:
            if (byte == FLAG) { *st = ST_END; return 1; }
            else *st = ST_START;
            break;
        case ST_END: break;
    }
    return 0;
}

// --- Alarm state ---
static volatile sig_atomic_t alarmEnabled = 0;
static volatile sig_atomic_t alarmCount = 0;

// Alarm handler
static void alarmHandler(int signal) {
    (void)signal;
    alarmEnabled = 0;   // timer is no longer active
    alarmCount++;       // count a timeout (will trigger retransmission)
}

// Build FLAG A C BCC1 FLAG into out[5]
static int build_supervision(unsigned char A, unsigned char C, unsigned char out[5]) {
    out[0] = FLAG;
    out[1] = A;
    out[2] = C;
    out[3] = (unsigned char)(A ^ C); // BCC1
    out[4] = FLAG;
    return 5;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // 1) Open and configure the serial port
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
        return -1;

    // 2) Branch by role
    if (connectionParameters.role == LlTx) {
        // -------- TRANSMITTER: send SET; wait for UA with timeout/retries --------

        // 2a) Install SIGALRM handler (your pattern)
        struct sigaction act = {0};
        act.sa_handler = alarmHandler;
        sigemptyset(&act.sa_mask);
        if (sigaction(SIGALRM, &act, NULL) == -1) {
            closeSerialPort();
            return -1;
        }

        // 2b) Build the SET frame (A=0x03, C=SET)
        unsigned char setFrame[5];
        build_supervision(A_TX_CMD_OR_RX_REPLY, C_SET, setFrame);

        // 2c) Prepare FSM to detect UA and control variables
        SuState st;
        su_reset(&st);

        unsigned char byte = 0;
        int attempts = 0;              // number of timeouts occurred (retransmissions)
        alarmEnabled = 0;              // no active timer yet
        alarmCount  = 0;

        // 2d) Main open loop:
        //     - If no timer is active, (re)send SET and arm the timer (alarmEnabled=1)
        //     - Read bytes and feed the FSM until a full UA is recognized
        //     - If timer fires (alarmEnabled cleared by handler), retransmit and count attempt
        while (attempts < connectionParameters.nRetransmissions) {

            if (!alarmEnabled) {
                // (Re)send SET and start timer
                if (writeBytesSerialPort(setFrame, 5) != 5) {
                    closeSerialPort();
                    return -1;
                }
                alarm(connectionParameters.timeout);  // arm timer (seconds)
                alarmEnabled = 1;
            }

            // Blocking read (up to the VTIME in serial_port.c); SIGALRM interrupts it on timeout
            int r = readByteSerialPort(&byte);
            if (r == 1) {
                // Feed FSM looking for UA (A=0x03, C=UA)
                if (su_feed(&st, byte, A_TX_CMD_OR_RX_REPLY, C_UA)) {
                    alarm(0);                 // cancel timer
                    alarmEnabled = 0;
                    return 0;                 // OPEN success
                }
                continue; // keep assembling UA
            }

            // If we get here: either no byte (r==0) or read interrupted; check if timer fired
            if (!alarmEnabled) {
                // Timer expired -> retransmit
                attempts++;
                su_reset(&st);                // reset FSM for a clean UA
                if (attempts >= connectionParameters.nRetransmissions) {
                    closeSerialPort();
                    return -1;                // OPEN failed after max retries
                }
                // Loop continues -> will resend SET and arm timer again
            }
            // else: some read issue not caused by timer; loop again and keep reading
        }

        // Shouldn't reach here (loop returns on success/failure)
        closeSerialPort();
        return -1;

    } else {
        // -------- RECEIVER: wait for SET; reply with UA --------

        SuState st;
        su_reset(&st);

        unsigned char byte = 0;

        // 2e) Read until we receive a full valid SET (A=0x03, C=SET)
        while (1) {
            int r = readByteSerialPort(&byte);  // blocking (VTIME-based)
            if (r < 0) {                        // read error
                closeSerialPort();
                return -1;
            }
            if (r == 0) continue;               // nothing this slice; keep waiting

            if (su_feed(&st, byte, A_TX_CMD_OR_RX_REPLY, C_SET)) {
                break;                          // got a full SET
            }
        }

        // 2f) Send UA (A=0x03, C=UA)
        unsigned char ua[5];
        build_supervision(A_TX_CMD_OR_RX_REPLY, C_UA, ua);
        if (writeBytesSerialPort(ua, 5) != 5) {
            closeSerialPort();
            return -1;
        }

        return 0; // OPEN success
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
