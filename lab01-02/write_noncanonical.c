// Example of how to write to the serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]
// Extended with UA waiting + timeout + retransmissions using alarm

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BAUDRATE 38400
#define BUF_SIZE 256

#define FLAG 0x7E
#define A_SENDER 0x03
#define C_SET 0x03
#define C_UA 0x07

int fd = -1;           // File descriptor for open serial port
struct termios oldtio; // Serial port settings to restore on closing

int alarmEnabled = FALSE;
int alarmCount = 0;
const int MAX_RETRANSMISSIONS = 3;

// Alarm handler
void alarmHandler(int signal) {

    (void)signal;
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d: retransmitting SET\n", alarmCount);
}

int openSerialPort(const char *serialPort, int baudRate);
int closeSerialPort();
int readByteSerialPort(unsigned char *byte);
int writeBytesSerialPort(const unsigned char *bytes, int nBytes);

// ---------------------------------------------------
// MAIN
// ---------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS0\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    const char *serialPort = argv[1];

    if (openSerialPort(serialPort, BAUDRATE) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }

    printf("Serial port %s opened\n", serialPort);

    // Configure alarm handler
    struct sigaction act = {0};
    act.sa_handler = alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }

    // Build SET frame
    unsigned char setFrame[5] = {FLAG, A_SENDER, C_SET, A_SENDER ^ C_SET, FLAG};

    unsigned char buf[5];
    int n = 0;

    while (alarmCount < MAX_RETRANSMISSIONS)
    {
        if (alarmEnabled == FALSE)
        {
            // Send SET frame
            writeBytesSerialPort(setFrame, 5);
            printf("SET frame sent\n");

            // Enable alarm
            alarm(3);
            alarmEnabled = TRUE;
        }

        int r = readByteSerialPort(&buf[n]);
        if (r > 0)
        {
            n += r;
            if (n == 5)
            {
                // Check if UA
                if (buf[0] == FLAG && buf[1] == 0x01 &&
                    buf[2] == C_UA && buf[3] == (buf[1] ^ buf[2]) &&
                    buf[4] == FLAG)
                {
                    alarm(0); // disable alarm
                    printf("UA frame received: ");
                    for (int i = 0; i < 5; i++)
                        printf("0x%02X ", buf[i]);
                    printf("\n");
                    break;
                }
                n = 0; // reset buffer if not UA
            }
        }
    }

    if (alarmCount >= MAX_RETRANSMISSIONS)
    {
        printf("Connection failed: UA not received\n");
    }

    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    printf("Serial port %s closed\n", serialPort);

    return 0;
}

// ---------------------------------------------------
// SERIAL PORT LIBRARY IMPLEMENTATION (unchanged)
// ---------------------------------------------------
int openSerialPort(const char *serialPort, int baudRate) {
    int oflags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    fd = open(serialPort, oflags);
    if (fd < 0) {
        perror(serialPort);
        return -1;
    }

    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

#define CASE_BAUDRATE(baudrate) \
    case baudrate:              \
        br = B##baudrate;       \
        break;

    tcflag_t br;
    switch (baudRate) {
        CASE_BAUDRATE(1200);
        CASE_BAUDRATE(1800);
        CASE_BAUDRATE(2400);
        CASE_BAUDRATE(4800);
        CASE_BAUDRATE(9600);
        CASE_BAUDRATE(19200);
        CASE_BAUDRATE(38400);
        CASE_BAUDRATE(57600);
        CASE_BAUDRATE(115200);
    default:
        fprintf(stderr, "Unsupported baud rate\n");
        return -1;
    }
#undef CASE_BAUDRATE

    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = br | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    oflags ^= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, oflags) == -1) {
        perror("fcntl");
        close(fd);
        return -1;
    }

    return fd;
}

int closeSerialPort() {
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        return -1;
    }
    return close(fd);
}

int readByteSerialPort(unsigned char *byte) {
    return read(fd, byte, 1);
}

int writeBytesSerialPort(const unsigned char *bytes, int nBytes) {
    return write(fd, bytes, nBytes);
}
