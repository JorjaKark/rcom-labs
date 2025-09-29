#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>

#define FLAG 0x7E
#define A    0x03   // Address: sender → receiver
#define C_SET 0x03  // Control field: SET
#define C_UA  0x07  // Control field: UA

#define BAUDRATE B38400
#define MAX_RETRIES 3
#define TIMEOUT 3

int fd;
struct termios oldtio;
int alarmEnabled = 0;
int retransmissions = 0;
int uaReceived = 0;

void alarmHandler(int sig) {
    (void)sig; // suppress unused parameter warning
    alarmEnabled = 0;
    retransmissions++;
    printf("Timeout #%d: retransmitting SET\n", retransmissions);
}


void sendSET() {
    unsigned char setFrame[5] = {FLAG, A, C_SET, A ^ C_SET, FLAG};
    write(fd, setFrame, 5);
    printf("SET sent\n");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <SerialPort>\n", argv[0]);
        exit(1);
    }

    // Open serial port
    fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (fd < 0) { perror(argv[1]); exit(1); }

    struct termios newtio;
    tcgetattr(fd, &oldtio);
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    // Setup alarm handler
    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = alarmHandler;
    sigaction(SIGALRM, &act, NULL);

    // Try to send SET until UA is received or max retries
    while (!uaReceived && retransmissions < MAX_RETRIES) {
        if (!alarmEnabled) {
            sendSET();
            alarm(TIMEOUT);
            alarmEnabled = 1;
        }

        unsigned char buf[5];
        int n = read(fd, buf, 5);
        if (n == 5 && buf[0] == FLAG && buf[1] == 0x01 && buf[2] == C_UA &&
            buf[3] == (buf[1] ^ buf[2]) && buf[4] == FLAG) {
            printf("UA received!\n");
            uaReceived = 1;
            alarm(0); // cancel alarm
        }
    }

    if (!uaReceived)
        printf("Connection failed after %d retries\n", MAX_RETRIES);

    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    return 0;
}
