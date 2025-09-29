#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>

#define FLAG 0x7E
#define A    0x03 
#define C_SET 0x03
#define C_UA  0x07

#define BAUDRATE B38400

typedef enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP } State;

int fd;
struct termios oldtio;

void sendUA() {
    unsigned char uaFrame[5] = {FLAG, 0x01, C_UA, 0x01 ^ C_UA, FLAG};
    write(fd, uaFrame, 5);
    printf("UA sent\n");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <SerialPort>\n", argv[0]);
        exit(1);
    }

    fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (fd < 0) { perror(argv[1]); exit(1); }

    struct termios newtio;
    tcgetattr(fd, &oldtio);
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    State state = START;
    unsigned char byte;
    while (state != STOP) {
        if (read(fd, &byte, 1) > 0) {
            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte == C_SET) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == (A ^ C_SET)) state = BCC_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG) state = STOP;
                    else state = START;
                    break;
                default: state = START;
            }
        }
    }

    printf("SET frame received!\n");
    sendUA();

    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    return 0;
}
