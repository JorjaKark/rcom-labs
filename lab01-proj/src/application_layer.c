#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// ---- Application layer control ----
#define C_START 0x01
#define C_DATA 0x02
#define C_END 0x03

// ---- TLV types ----
#define T_FILESIZE 0x00
#define T_FILENAME 0x01

// ---- Build CONTROL packet ----
static int build_control_packet(unsigned char controlType,
                                const char *fileName,
                                unsigned long fileSize,
                                unsigned char *outBuffer,
                                int outCapacity)
{
    if (!fileName || !outBuffer)
        return -1;

    // Encode file size as big endian
    unsigned char fileSizeBytes[8];
    int fileSizeLength = 0;

    unsigned long remaining = fileSize;
    do
    {
        fileSizeBytes[7 - fileSizeLength] = (unsigned char)(remaining & 0xFF);
        remaining >>= 8;
        fileSizeLength++;
    } while (remaining != 0 && fileSizeLength < 8);

    const unsigned char *fileSizePtr = &fileSizeBytes[8 - fileSizeLength];

    int fileNameLength = (int)strlen(fileName);
    int totalLength = 1 + (1 + 1 + fileSizeLength) + (1 + 1 + fileNameLength);
    if (totalLength > outCapacity)
        return -1;

    int pos = 0;
    outBuffer[pos++] = controlType;

    outBuffer[pos++] = T_FILESIZE;
    outBuffer[pos++] = (unsigned char)fileSizeLength;
    memcpy(outBuffer + pos, fileSizePtr, fileSizeLength);
    pos += fileSizeLength;

    outBuffer[pos++] = T_FILENAME;
    outBuffer[pos++] = (unsigned char)fileNameLength;
    memcpy(outBuffer + pos, fileName, fileNameLength);
    pos += fileNameLength;

    return pos;
}

// ---- Parse CONTROL packet ----
static int parse_control_packet(const unsigned char *packet, int packetLength,
                                char *outFileName, size_t fileNameCapacity,
                                unsigned long *outFileSize)
{
    if (!packet || packetLength < 1)
        return -1;
    int pos = 1; // skip control byte C

    unsigned long parsedFileSize = 0;
    int hasFileSize = 0;
    int hasFileName = 0;

    while (pos + 2 <= packetLength)
    {
        unsigned char tag = packet[pos++];
        unsigned char length = packet[pos++];
        if (pos + length > packetLength)
            return -1;

        if (tag == T_FILESIZE)
        {
            if (length == 0 || length > 8)
                return -1;
            unsigned long size = 0;
            for (int i = 0; i < length; ++i)
                size = (size << 8) | packet[pos + i];
            parsedFileSize = size;
            hasFileSize = 1;
        }
        else if (tag == T_FILENAME)
        {
            if (length == 0 || (size_t)length >= fileNameCapacity)
                return -1;
            memcpy(outFileName, packet + pos, length);
            outFileName[length] = '\0';
            hasFileName = 1;
        }
        pos += length;
    }

    if (!hasFileSize || !hasFileName)
        return -1;
    if (outFileSize)
        *outFileSize = parsedFileSize;
    return 0;
}

// ---- Build DATA packet ----
static int build_data_packet(unsigned char sequenceNumber,
                             const unsigned char *data,
                             int dataLength,
                             unsigned char *outBuffer,
                             int outCapacity)
{
    if (!outBuffer || dataLength < 0 || dataLength > MAX_PAYLOAD_SIZE)
        return -1;
    int requiredLength = 1 + 1 + 2 + dataLength;
    if (requiredLength > outCapacity)
        return -1;

    outBuffer[0] = C_DATA;
    outBuffer[1] = sequenceNumber;
    outBuffer[2] = (unsigned char)(dataLength & 0xFF);        // L2
    outBuffer[3] = (unsigned char)((dataLength >> 8) & 0xFF); // L1
    if (dataLength > 0 && data)
        memcpy(outBuffer + 4, data, dataLength);
    return requiredLength;
}

// ---- Parse DATA packet ----
static int parse_data_packet(const unsigned char *packet,
                             int packetLength,
                             unsigned char *sequenceOut)
{
    if (!packet || packetLength < 4)
        return -1;
    if (packet[0] != C_DATA)
        return -1;
    if (sequenceOut)
        *sequenceOut = packet[1];
    int payloadLength = (int)packet[2] | ((int)packet[3] << 8);
    if (payloadLength < 0 || (4 + payloadLength) > packetLength)
        return -1;
    return payloadLength;
}

// ---- Main application layer logic ----
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filePath)
{
    // ---- Configure link layer ----
    LinkLayer link = {0};
    snprintf(link.serialPort, sizeof(link.serialPort), "%s", serialPort);

    if (strcmp(role, "tx") == 0)
    {
        link.role = LlTx;
    }
    else
    {
        link.role = LlRx;
    }

    link.baudRate = baudRate;
    link.nRetransmissions = nTries;
    link.timeout = timeout;

    // ---- Open link ----
    if (llopen(link) < 0)
    {
        fprintf(stderr, "[APP] llopen failed\n");
        return;
    }

    // ---- Transmitter ----
    if (link.role == LlTx)
    {
        FILE *inputFile = fopen(filePath, "rb");
        if (!inputFile)
        {
            perror("[APP] fopen input");
            llclose();
            return;
        }

        if (fseek(inputFile, 0, SEEK_END) != 0)
        {
            perror("[APP] fseek");
            fclose(inputFile);
            llclose();
            return;
        }

        long inputFileLength = ftell(inputFile);
        if (inputFileLength < 0)
        {
            perror("[APP] ftell");
            fclose(inputFile);
            llclose();
            return;
        }

        rewind(inputFile);
        unsigned long fileSize = (unsigned long)inputFileLength;

        // Extract base file name
        const char *baseName = strrchr(filePath, '/');
        if (baseName != NULL)
        {
            baseName = baseName + 1;
        }
        else
        {
            baseName = filePath;
        }

        // ---- Send START packet ----
        unsigned char packetBuffer[512];
        int packetLength = build_control_packet(C_START, baseName, fileSize, packetBuffer, sizeof(packetBuffer));
        if (packetLength < 0)
        {
            fprintf(stderr, "[APP] build START failed\n");
            fclose(inputFile);
            llclose();
            return;
        }

        if (llwrite(packetBuffer, packetLength) < 0)
        {
            fprintf(stderr, "[APP] llwrite START failed\n");
            fclose(inputFile);
            llclose();
            return;
        }

        // ---- Send DATA packets ----
        unsigned char sequenceNumber = 0;
        unsigned char dataBuffer[MAX_PAYLOAD_SIZE];
        unsigned char dataPacket[MAX_PAYLOAD_SIZE + 8];

        while (!feof(inputFile))
        {
            size_t bytesRead = fread(dataBuffer, 1, MAX_PAYLOAD_SIZE, inputFile);
            if (ferror(inputFile))
            {
                perror("[APP] fread");
                fclose(inputFile);
                llclose();
                return;
            }

            if (bytesRead == 0)
                break;

            int dataPacketLength = build_data_packet(sequenceNumber, dataBuffer, (int)bytesRead, dataPacket, sizeof(dataPacket));
            if (dataPacketLength < 0)
            {
                fprintf(stderr, "[APP] build DATA failed\n");
                fclose(inputFile);
                llclose();
                return;
            }

            if (llwrite(dataPacket, dataPacketLength) < 0)
            {
                fprintf(stderr, "[APP] llwrite DATA failed (seq=%u)\n", sequenceNumber);
                fclose(inputFile);
                llclose();
                return;
            }

            sequenceNumber = (unsigned char)((sequenceNumber + 1) & 0xFF);
        }

        // ---- Send END packet ----
        packetLength = build_control_packet(C_END, baseName, fileSize, packetBuffer, sizeof(packetBuffer));
        if (packetLength < 0)
        {
            fprintf(stderr, "[APP] build END failed\n");
            fclose(inputFile);
            llclose();
            return;
        }

        if (llwrite(packetBuffer, packetLength) < 0)
        {
            fprintf(stderr, "[APP] llwrite END failed\n");
            fclose(inputFile);
            llclose();
            return;
        }

        fclose(inputFile);

        // ---- Close link ----
        if (llclose() < 0)
        {
            fprintf(stderr, "[APP] llclose failed\n");
            return;
        }

        printf("[APP] Transmission complete: sent %lu bytes\n", fileSize);
    }

    // ---- Receiver ----
    else
    {
        unsigned char packet[MAX_PAYLOAD_SIZE + 16];
        int packetLength = llread(packet);

        if (packetLength <= 0 || packet[0] != C_START)
        {
            fprintf(stderr, "[APP] Expected START, got len=%d C=0x%02X\n",
                    packetLength, packetLength > 0 ? packet[0] : 0x00);
            llclose();
            return;
        }

        char senderFileName[256];
        unsigned long announcedFileSize = 0;
        if (parse_control_packet(packet, packetLength, senderFileName, sizeof(senderFileName), &announcedFileSize) < 0)
        {
            fprintf(stderr, "[APP] parse START failed\n");
            llclose();
            return;
        }

        FILE *outputFile = fopen(filePath, "wb");
        if (!outputFile)
        {
            perror("[APP] fopen output");
            llclose();
            return;
        }

        unsigned char expectedSequence = 0;
        unsigned long totalReceivedBytes = 0;

        // ---- Receive DATA packets until END ----
        while (1)
        {
            packetLength = llread(packet);
            if (packetLength < 0)
            {
                fprintf(stderr, "[APP] llread error\n");
                fclose(outputFile);
                llclose();
                return;
            }

            if (packetLength == 0)
                continue;

            unsigned char controlType = packet[0];

            if (controlType == C_DATA)
            {
                unsigned char sequenceNumber = 0;
                int dataLength = parse_data_packet(packet, packetLength, &sequenceNumber);
                if (dataLength < 0)
                {
                    fprintf(stderr, "[APP] bad DATA packet\n");
                    fclose(outputFile);
                    llclose();
                    return;
                }

                if (sequenceNumber == expectedSequence)
                {
                    if (dataLength > 0)
                    {
                        if ((int)fwrite(packet + 4, 1, dataLength, outputFile) != dataLength)
                        {
                            perror("[APP] fwrite");
                            fclose(outputFile);
                            llclose();
                            return;
                        }
                        totalReceivedBytes += (unsigned long)dataLength;
                    }
                    expectedSequence = (unsigned char)((expectedSequence + 1) & 0xFF);
                }
            }
            else if (controlType == C_END)
            {
                // ---- Consistency check ----
                char endFileName[256];
                unsigned long endFileSize = 0;
                if (parse_control_packet(packet, packetLength, endFileName, sizeof(endFileName), &endFileSize) == 0)
                {
                    if (endFileSize != announcedFileSize)
                    {
                        fprintf(stderr, "[APP] Warning: size mismatch START=%lu END=%lu\n",
                                announcedFileSize, endFileSize);
                    }
                }
                break;
            }
        }

        fclose(outputFile);

        // ---- Close link ----
        if (llclose() < 0)
        {
            fprintf(stderr, "[APP] llclose failed\n");
            return;
        }

        printf("[APP] Reception complete: announced %lu bytes, wrote %lu bytes\n",
               announcedFileSize, totalReceivedBytes);
    }
}
