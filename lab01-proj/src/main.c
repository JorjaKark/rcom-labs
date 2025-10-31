// Main file of the serial port project.
// DO NOT CHANGE THIS FILE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>  // <-- added for timing

#include "application_layer.h"

#define N_TRIES 3
#define TIMEOUT 4

// Helper to get current time in seconds (with microsecond precision)
static double now_seconds()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// Arguments:
//   $1: /dev/ttySxx
//   $2: baud rate
//   $3: tx | rx
//   $4: filename
int main(int argc, char *argv[])
{
    if (argc < 5)
    {
        printf("Usage: %s /dev/ttySxx baudrate tx|rx filename\n", argv[0]);
        exit(1);
    }

    const char *serialPort = argv[1];
    const int baudrate = atoi(argv[2]);
    const char *role = argv[3];
    const char *filename = argv[4];

    // Validate baud rate
    switch (baudrate)
    {
    case 1200:
    case 1800:
    case 2400:
    case 4800:
    case 9600:
    case 19200:
    case 38400:
    case 57600:
    case 115200:
        break;
    default:
        printf("Unsupported baud rate (must be one of 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200)\n");
        exit(2);
    }

    // Validate role
    if (strcmp("tx", role) != 0 && strcmp("rx", role) != 0)
    {
        printf("ERROR: Role must be \"tx\" or \"rx\"\n");
        exit(3);
    }

    printf("Starting link-layer protocol application\n"
           "  - Serial port: %s\n"
           "  - Role: %s\n"
           "  - Baudrate: %d\n"
           "  - Number of tries: %d\n"
           "  - Timeout: %d\n"
           "  - Filename: %s\n",
           serialPort,
           role,
           baudrate,
           N_TRIES,
           TIMEOUT,
           filename);

    // --- Measure total transfer time ---
    double t_start = now_seconds();

    applicationLayer(serialPort, role, baudrate, N_TRIES, TIMEOUT, filename);

    double t_end = now_seconds();
    double elapsed = t_end - t_start;

    printf("Baudrate (C): %d bit/s\n", baudrate);
    printf("Total transfer time: %.6f seconds\n", elapsed);
    // -----------------------------------

    return 0;
}
