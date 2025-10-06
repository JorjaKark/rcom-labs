// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// ----- Frame constants -----
#define FLAG 0x7E

// Address field (commands from Tx or replies from Rx = 0x03)
#define A_TX_CMD_OR_RX_REPLY 0x03
// Address field (commands from Rx or replies from Tx = 0x01)
#define A_RX_CMD_OR_TX_REPLY 0x01

// Control field values (S/U frames)
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55

typedef enum { ST_START, ST_FLAG, ST_A, ST_C, ST_BCC_OK, ST_END } SuState;

static void su_reset(SuState *st) { *st = ST_START; }

static int su_feed(SuState *st, unsigned char byte, unsigned char expectedA,
                   unsigned char expectedC) {
  static unsigned char A, C;
  switch (*st) {
  case ST_START:
    if (byte == FLAG)
      *st = ST_FLAG;
    break;
  case ST_FLAG:
    if (byte == expectedA) {
      A = byte;
      *st = ST_A;
    } else if (byte != FLAG)
      *st = ST_START;
    break;
  case ST_A:
    if (byte == expectedC) {
      C = byte;
      *st = ST_C;
    } else if (byte == FLAG)
      *st = ST_FLAG;
    else
      *st = ST_START;
    break;
  case ST_C:
    if (byte == (unsigned char)(A ^ C))
      *st = ST_BCC_OK;
    else if (byte == FLAG)
      *st = ST_FLAG;
    else
      *st = ST_START;
    break;
  case ST_BCC_OK:
    if (byte == FLAG) {
      *st = ST_END;
      return 1;
    } else
      *st = ST_START;
    break;
  case ST_END:
    break;
  }
  return 0;
}

// --- Alarm state ---
static volatile sig_atomic_t alarmEnabled = 0;
static volatile sig_atomic_t alarmCount = 0;

// Alarm handler
static void alarmHandler(int signal) {
  (void)signal;
  alarmEnabled = 0; // timer is no longer active
  alarmCount++;     // count a timeout (will trigger retransmission)
}

// Running sequence state (Stop-and-Wait)
static unsigned char g_tx_ns;      // transmitter next sequence number (0/1)
static unsigned char g_rx_expected; // receiver expected Ns (0/1)

// Build FLAG A C BCC1 FLAG into out[5]
static int build_supervision(unsigned char A, unsigned char C,
                             unsigned char out[5]) {
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
int llopen(LinkLayer connectionParameters) {
  // 1) Open and configure the serial port
  if (openSerialPort(connectionParameters.serialPort,
                     connectionParameters.baudRate) < 0)
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
    int attempts = 0; // number of timeouts occurred (retransmissions)
    alarmEnabled = 0; // no active timer yet
    alarmCount = 0;

    // 2d) Main open loop:
    //     - If no timer is active, (re)send SET and arm the timer
    //     (alarmEnabled=1)
    //     - Read bytes and feed the FSM until a full UA is recognized
    //     - If timer fires (alarmEnabled cleared by handler), retransmit and
    //     count attempt
    while (attempts < connectionParameters.nRetransmissions) {

      if (!alarmEnabled) {
        // (Re)send SET and start timer
        if (writeBytesSerialPort(setFrame, 5) != 5) {
          closeSerialPort();
          return -1;
        }
        alarm(connectionParameters.timeout); // arm timer (seconds)
        alarmEnabled = 1;
      }

      // Blocking read (up to the VTIME in serial_port.c); SIGALRM interrupts it
      // on timeout
      int r = readByteSerialPort(&byte);
      if (r == 1) {
        // Feed FSM looking for UA (A=0x03, C=UA)
        if (su_feed(&st, byte, A_TX_CMD_OR_RX_REPLY, C_UA)) {
          alarm(0); // cancel timer
          alarmEnabled = 0;

          g_tx_ns = 0;
          g_rx_expected = 0;

          return 0; // OPEN success
        }
        continue; // keep assembling UA
      }

      // If we get here: either no byte (r==0) or read interrupted; check if
      // timer fired
      if (!alarmEnabled) {
        // Timer expired -> retransmit
        attempts++;
        su_reset(&st); // reset FSM for a clean UA
        if (attempts >= connectionParameters.nRetransmissions) {
          closeSerialPort();
          return -1; // OPEN failed after max retries
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
      int r = readByteSerialPort(&byte); // blocking (VTIME-based)
      if (r < 0) {                       // read error
        closeSerialPort();
        return -1;
      }
      if (r == 0)
        continue; // nothing this slice; keep waiting

      if (su_feed(&st, byte, A_TX_CMD_OR_RX_REPLY, C_SET)) {
        break; // got a full SET
      }
    }

    // 2f) Send UA (A=0x03, C=UA)
    unsigned char ua[5];
    build_supervision(A_TX_CMD_OR_RX_REPLY, C_UA, ua);
    if (writeBytesSerialPort(ua, 5) != 5) {
      closeSerialPort();
      return -1;
    }

    // Reset Stop-and-Wait state for data phase
    g_tx_ns = 0;
    g_rx_expected = 0;

    return 0; // OPEN success
  }
}

// ======================= M3 helpers =======================

// Byte-stuffing constants
#define ESC 0x7D
#define ESC_XOR 0x20

// Reasonable upper bounds
#define MAX_IFRAME_DATA MAX_PAYLOAD_SIZE // maximum data bytes in an I-frame
#define MAX_STUFFED(x)                                                         \
  (2 * (x) + 16) // worst-case: every byte needs escaping + header slack

// I-frame Control values (Ns)
#define C_I0 0x00
#define C_I1 0x80
// Running sequence state (Stop-and-Wait)
static unsigned char g_tx_ns = 0; // transmitter next sequence number (0/1)
static unsigned char g_rx_expected =
    0; // receiver expected Ns (0/1)// BCC2: XOR of all data bytes
static unsigned char bcc2_calc(const unsigned char *buf, int len) {
  unsigned char x = 0x00;
  for (int i = 0; i < len; ++i)
    x ^= buf[i];
  return x;
}

// Stuff a byte if needed (0x7E or 0x7D); return #bytes written (1 or 2)
static int stuff_byte(unsigned char b, unsigned char *out) {
  if (b == FLAG || b == ESC) {
    out[0] = ESC;
    out[1] = b ^ ESC_XOR;
    return 2;
  }
  out[0] = b;
  return 1;
}
// Stuff a buffer; return new length
static int stuff_bytes(const unsigned char *in, int n, unsigned char *out) {
  int j = 0;
  for (int i = 0; i < n; ++i)
    j += stuff_byte(in[i], out + j);
  return j;
}

// Destuff a buffer; return new length or -1 on malformed escape
static int destuff_bytes(const unsigned char *in, int n, unsigned char *out) {
  int j = 0;
  for (int i = 0; i < n; ++i) {
    if (in[i] == ESC) {
      if (i + 1 >= n)
        return -1;
      out[j++] = in[++i] ^ ESC_XOR;
    } else {
      out[j++] = in[i];
    }
  }
  return j;
}

// Build an Information frame:
//   FLAG | A | C | BCC1 | [stuffed DATA...] | [stuffed BCC2] | FLAG
// We stuff only DATA and BCC2 (header bytes are fixed and safe).
// Returns frame length or -1.
static int build_iframe(unsigned char A, unsigned char C,
                        const unsigned char *data, int len, unsigned char *out,
                        int outCap) {
  if (len < 0 || len > MAX_IFRAME_DATA)
    return -1;

  // Minimum required capacity check (very safe upper bound)
  int worst = 1 + 3 + (2 * len) + 2 +
              1; // FLAG + A,C,BCC1 + stuffed data + stuffed bcc2(max 2) + FLAG
  if (outCap < worst)
    return -1;

  int pos = 0;
  out[pos++] = FLAG;
  out[pos++] = A;
  out[pos++] = C;
  out[pos++] = (unsigned char)(A ^ C);

  // Stuff DATA
  if (len > 0) {
    pos += stuff_bytes(data, len, out + pos);
  }

  // Stuff BCC2
  unsigned char b2 = bcc2_calc(data, len);
  pos += stuff_byte(b2, out + pos);

  out[pos++] = FLAG;
  return pos;
}

// Read a single supervision (S/U) frame and return its Control (C) byte.
// Waits for: FLAG A C BCC1 FLAG, validates A and BCC1, returns 1 and writes
// *Cout. Blocks (no timer here; timers arrive in M4). Returns -1 on read or
// format error.
static int read_supervision_any(unsigned char expectedA, unsigned char *Cout) {
  enum { ST_S, ST_F, ST_A, ST_C, ST_B, ST_END } st = ST_S;
  unsigned char A = 0, C = 0, B = 0, b = 0;

  while (1) {
    int r = readByteSerialPort(&b);
    if (r < 0)
      return -1;
    if (r == 0)
      continue;

    switch (st) {
    case ST_S:
      if (b == FLAG)
        st = ST_F;
      break;
    case ST_F:
      if (b == expectedA) {
        A = b;
        st = ST_A;
      } else if (b != FLAG)
        st = ST_S;
      break;
    case ST_A:
      C = b;
      st = ST_C;
      break;
    case ST_C:
      B = b;
      st = ST_B;
      break;
    case ST_B:
      if (b == FLAG) {
        if (B == (unsigned char)(A ^ C)) {
          if (Cout)
            *Cout = C;
          return 1;
        }
      }
      st = ST_S; // restart if header invalid or wrong closing
      break;
    default:
      break;
    }
  }
}

// Read one Information frame into dataOut. Validates header + BCC2.
// Returns payload length (>=0) and sets *Cout to I-frame C (0x00 or 0x80).
// Blocks until a full I frame; returns -1 on format/BCC errors (caller may
// REJ).
static int read_iframe(unsigned char expectedA, unsigned char *Cout,
                       unsigned char *dataOut, int maxLen) {
  // 1) Wait for opening FLAG
  unsigned char b = 0;
  while (1) {
    int r = readByteSerialPort(&b);
    if (r < 0)
      return -1;
    if (r == 0)
      continue;
    if (b == FLAG)
      break;
  }

  // 2) Collect bytes until closing FLAG
  unsigned char body[MAX_STUFFED(MAX_IFRAME_DATA) + 8];
  int blen = 0;
  while (1) {
    int r = readByteSerialPort(&b);
    if (r < 0)
      return -1;
    if (r == 0)
      continue;
    if (b == FLAG)
      break; // closing flag reached
    if (blen >= (int)sizeof(body))
      return -1; // overflow
    body[blen++] = b;
  }

  // body = [A][C][BCC1][stuffed DATA... + stuffed BCC2]
  if (blen < 4)
    return -1;
  unsigned char A = body[0];
  unsigned char C = body[1];
  unsigned char B1 = body[2];
  if (A != expectedA)
    return -1;
  if (B1 != (unsigned char)(A ^ C))
    return -1;

  // Destuff payload+BCC2 portion
  int stuffedLen = blen - 3;
  if (stuffedLen < 1) { // must at least contain BCC2
    if (Cout)
      *Cout = C;
    return 0; // empty payload case (allowed)
  }

  unsigned char buf[MAX_STUFFED(MAX_IFRAME_DATA) + 8];
  int dst = destuff_bytes(body + 3, stuffedLen, buf);
  if (dst < 1)
    return -1; // must include BCC2

  int payloadLen = dst - 1;
  unsigned char calc = bcc2_calc(buf, payloadLen);
  unsigned char b2 = buf[payloadLen];
  if (calc != b2)
    return -1; // BCC2 mismatch

  if (payloadLen > maxLen)
    return -1;

  if (payloadLen > 0)
    memcpy(dataOut, buf, payloadLen);
  if (Cout)
    *Cout = C;
  return payloadLen;
}
// Send RR(Nr)
static int send_rr(unsigned char Nr) {
  unsigned char c = (Nr == 0) ? C_RR0 : C_RR1;
  unsigned char f[5];
  build_supervision(A_TX_CMD_OR_RX_REPLY, c, f);
  return (writeBytesSerialPort(f, 5) == 5) ? 0 : -1;
}

// Send REJ(Nr)
static int send_rej(unsigned char Nr) {
  unsigned char c = (Nr == 0) ? C_REJ0 : C_REJ1;
  unsigned char f[5];
  build_supervision(A_TX_CMD_OR_RX_REPLY, c, f);
  return (writeBytesSerialPort(f, 5) == 5) ? 0 : -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
  if (bufSize < 0 || bufSize > MAX_IFRAME_DATA)
    return -1;

  // 1) Choose C by Ns (0x00 for Ns=0, 0x80 for Ns=1)
  unsigned char C = (g_tx_ns == 0) ? C_I0 : C_I1;

  // 2) Build I-frame
  unsigned char frame[MAX_STUFFED(MAX_IFRAME_DATA) + 16];
  int flen =
      build_iframe(A_TX_CMD_OR_RX_REPLY, C, buf, bufSize, frame, sizeof(frame));
  if (flen < 0)
    return -1;

  // 3) Send I-frame
  if (writeBytesSerialPort(frame, flen) != flen)
    return -1;

  // 4) Wait for RR/REJ (no timer here; M4 adds timeout and retransmissions)
  while (1) {
    unsigned char c = 0;
    int ok = read_supervision_any(A_TX_CMD_OR_RX_REPLY, &c);
    if (ok < 0)
      return -1; // read error

    // Accept only RR/REJ controls
    if (c == C_RR0 || c == C_RR1) {
      unsigned char nr = (c == C_RR0) ? 0 : 1;
      // In Stop-and-Wait, RR(Nr) should acknowledge Ns, so Nr == Ns^1
      if (nr == (unsigned char)(g_tx_ns ^ 1)) {
        g_tx_ns ^= 1; // advance sequence on valid ACK
        return bufSize;
      }
      // Otherwise ignore spurious RR and keep waiting
    } else if (c == C_REJ0 || c == C_REJ1) {
      // Receiver signaled error: leave retransmission to M4
      return -1;
    } else {
      // Ignore unrelated supervision frames (e.g., DISC during close, etc.)
    }
  }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////

int llread(unsigned char *packet) {
  while (1) {
    unsigned char c = 0;
    int n = read_iframe(A_TX_CMD_OR_RX_REPLY, &c, packet, MAX_IFRAME_DATA);
    if (n < 0) {
      // Format/BCC2 error → REJ(expected)
      if (send_rej(g_rx_expected) < 0)
        return -1;
      continue; // keep waiting for a good I-frame
    }

    // We received a valid I-frame; extract Ns from C (0x80 bit)
    unsigned char ns = (c == C_I1) ? 1 : 0;

    if (ns == g_rx_expected) {
      // New frame in sequence → deliver and ACK with RR(expected^1)
      if (send_rr(g_rx_expected ^ 1) < 0)
        return -1;
      g_rx_expected ^= 1;
      return n; // deliver 'n' bytes now in 'packet'
    } else {
      // Duplicate frame → re-ACK with current expected (no data delivered)
      if (send_rr(g_rx_expected) < 0)
        return -1;
      // Don’t return data (duplicate). Loop to wait for the next frame.
      // If you prefer to signal duplicate to upper layer, you could return 0.
    }
  }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose() {
  // TODO: Implement this function

  return 0;
}
