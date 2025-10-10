#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// ---- App-layer control codes ----
#define C_DATA  0x01
#define C_START 0x02
#define C_END   0x03

// ---- TLV types for START/END ----
#define T_FILESIZE 0x00
#define T_FILENAME 0x01

// Build START/END packet: C, [T0=filesize L0 V0], [T1=filename L1 V1]
static int build_control_packet(unsigned char C,
                                const char *filename,
                                unsigned long filesize,
                                unsigned char *out,
                                int outCap)
{
    if (!filename || !out) return -1;

    // Encode filesize big-endian, minimal bytes
    unsigned char sizebuf[8];
    int nsize = 0;
    {
        unsigned long s = filesize;
        do {
            sizebuf[7 - nsize] = (unsigned char)(s & 0xFF);
            s >>= 8;
            nsize++;
        } while (s != 0 && nsize < 8);
    }
    const unsigned char *sizeptr = &sizebuf[8 - nsize];

    int nameLen = (int)strlen(filename);
    int need = 1 /*C*/ + (1+1+nsize) + (1+1+nameLen);
    if (need > outCap) return -1;

    int p = 0;
    out[p++] = C;

    out[p++] = T_FILESIZE;
    out[p++] = (unsigned char)nsize;
    memcpy(out + p, sizeptr, nsize);
    p += nsize;

    out[p++] = T_FILENAME;
    out[p++] = (unsigned char)nameLen;
    memcpy(out + p, filename, nameLen);
    p += nameLen;

    return p;
}

// Parse START/END → fills outName (with '\0') and outSize. Returns 0 on success.
static int parse_control_packet(const unsigned char *pkt, int len,
                                char *outName, size_t nameCap,
                                unsigned long *outSize)
{
    if (!pkt || len < 1) return -1;
    int p = 1; // skip C

    unsigned long filesize = 0;
    int gotSize = 0, gotName = 0;

    while (p + 2 <= len) {
        unsigned char T = pkt[p++];
        unsigned char L = pkt[p++];
        if (p + L > len) return -1;

        if (T == T_FILESIZE) {
            if (L == 0 || L > 8) return -1;
            unsigned long s = 0;
            for (int i = 0; i < L; ++i) s = (s << 8) | pkt[p + i];
            filesize = s;
            gotSize = 1;
        } else if (T == T_FILENAME) {
            if (L == 0 || (size_t)L >= nameCap) return -1;
            memcpy(outName, pkt + p, L);
            outName[L] = '\0';
            gotName = 1;
        }
        p += L;
    }

    if (!gotSize || !gotName) return -1;
    if (outSize) *outSize = filesize;
    return 0;
}

// Build DATA packet: [C=0x01][N][L2][L1][DATA...]
static int build_data_packet(unsigned char seq,
                             const unsigned char *data, int len,
                             unsigned char *out, int outCap)
{
    if (!out || len < 0 || len > MAX_PAYLOAD_SIZE) return -1;
    int need = 1 + 1 + 2 + len;
    if (need > outCap) return -1;

    out[0] = C_DATA;
    out[1] = seq;
    out[2] = (unsigned char)(len & 0xFF);        // L2
    out[3] = (unsigned char)((len >> 8) & 0xFF); // L1
    if (len > 0 && data) memcpy(out + 4, data, len);
    return need;
}

// Parse DATA packet; returns payload length (>=0) or -1. Writes seq to *seqOut.
static int parse_data_packet(const unsigned char *pkt, int len, unsigned char *seqOut)
{
    if (!pkt || len < 4) return -1;
    if (pkt[0] != C_DATA) return -1;
    if (seqOut) *seqOut = pkt[1];
    int L = (int)pkt[2] | ((int)pkt[3] << 8); // L2 + 256*L1
    if (L < 0 || (4 + L) > len) return -1;
    return L;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // 0) Build link-layer config
    LinkLayer ll = {0};
    snprintf(ll.serialPort, sizeof(ll.serialPort), "%s", serialPort);
    ll.role             = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    ll.baudRate         = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout          = timeout;

    // 1) Open link
    if (llopen(ll) < 0) {
        fprintf(stderr, "[APP] llopen failed\n");
        return;
    }

    if (ll.role == LlTx) {
        // ================== TRANSMITTER ==================

        // 2) Open input file and get size
        FILE *in = fopen(filename, "rb");
        if (!in) { perror("[APP] fopen input"); llclose(); return; }
        if (fseek(in, 0, SEEK_END) != 0) { perror("[APP] fseek"); fclose(in); llclose(); return; }
        long fszL = ftell(in);
        if (fszL < 0) { perror("[APP] ftell"); fclose(in); llclose(); return; }
        rewind(in);

        unsigned long fsz = (unsigned long)fszL;

        // Use only the base name (optional; START includes the original name)
        const char *base = strrchr(filename, '/');
        base = base ? base + 1 : filename;

        // 3) Send START
        unsigned char pkt[512];
        int plen = build_control_packet(C_START, base, fsz, pkt, sizeof(pkt));
        if (plen < 0) {
            fprintf(stderr, "[APP] build START failed\n");
            fclose(in); llclose(); return;
        }
        if (llwrite(pkt, plen) < 0) {
            fprintf(stderr, "[APP] llwrite START failed\n");
            fclose(in); llclose(); return;
        }

        // 4) Send DATA chunks
        unsigned char seq = 0;
        unsigned char dbuf[MAX_PAYLOAD_SIZE];
        unsigned char dpkt[MAX_PAYLOAD_SIZE + 8];

        while (!feof(in)) {
            size_t n = fread(dbuf, 1, MAX_PAYLOAD_SIZE, in);
            if (ferror(in)) { perror("[APP] fread"); fclose(in); llclose(); return; }
            if (n == 0) break;

            int dlen = build_data_packet(seq, dbuf, (int)n, dpkt, sizeof(dpkt));
            if (dlen < 0) {
                fprintf(stderr, "[APP] build DATA failed\n");
                fclose(in); llclose(); return;
            }

            if (llwrite(dpkt, dlen) < 0) {
                fprintf(stderr, "[APP] llwrite DATA failed (seq=%u)\n", seq);
                fclose(in); llclose(); return;
            }

            seq = (unsigned char)((seq + 1) & 0xFF);
        }

        // 5) Send END
        plen = build_control_packet(C_END, base, fsz, pkt, sizeof(pkt));
        if (plen < 0) {
            fprintf(stderr, "[APP] build END failed\n");
            fclose(in); llclose(); return;
        }
        if (llwrite(pkt, plen) < 0) {
            fprintf(stderr, "[APP] llwrite END failed\n");
            fclose(in); llclose(); return;
        }

        fclose(in);

        // 6) Close link
        if (llclose() < 0) {
            fprintf(stderr, "[APP] llclose failed\n");
            return;
        }

        printf("[APP] Transmission complete: sent %lu bytes\n", fsz);
    }
    else {
        // =================== RECEIVER ====================

        // 2) Expect START
        unsigned char pkt[MAX_PAYLOAD_SIZE + 16];
        int n = llread(pkt);
        if (n <= 0 || pkt[0] != C_START) {
            fprintf(stderr, "[APP] Expected START, got len=%d C=0x%02X\n", n, n>0?pkt[0]:0x00);
            llclose();
            return;
        }

        char txName[256];
        unsigned long txSize = 0;
        if (parse_control_packet(pkt, n, txName, sizeof(txName), &txSize) < 0) {
            fprintf(stderr, "[APP] parse START failed\n");
            llclose(); return;
        }

        // 3) Open output file (use the filename argument given to this program)
        FILE *out = fopen(filename, "wb");
        if (!out) { perror("[APP] fopen output"); llclose(); return; }

        // 4) Receive DATA until END
        unsigned char expectedSeq = 0;
        unsigned long received = 0;

        while(1) {
            n = llread(pkt);
            if (n < 0) { fprintf(stderr, "[APP] llread error\n"); fclose(out); llclose(); return; }
            if (n == 0) { continue; } // duplicate I-frame scenario (re-ACKed at link-layer)

            unsigned char C = pkt[0];

            if (C == C_DATA) {
                unsigned char seq = 0;
                int dlen = parse_data_packet(pkt, n, &seq);
                if (dlen < 0) { fprintf(stderr, "[APP] bad DATA packet\n"); fclose(out); llclose(); return; }

                // DATA bytes start at pkt[4]
                if (seq == expectedSeq) {
                    if (dlen > 0) {
                        if ((int)fwrite(pkt + 4, 1, dlen, out) != dlen) {
                            perror("[APP] fwrite");
                            fclose(out); llclose(); return;
                        }
                        received += (unsigned long)dlen;
                    }
                    expectedSeq = (unsigned char)((expectedSeq + 1) & 0xFF);
                } else {
                    // Duplicate DATA (already delivered); ignore at app-layer.
                }
            }
            else if (C == C_END) {
                // Optional consistency check
                char endName[256];
                unsigned long endSize = 0;
                if (parse_control_packet(pkt, n, endName, sizeof(endName), &endSize) == 0) {
                    if (endSize != txSize) {
                        fprintf(stderr, "[APP] Warning: size mismatch START=%lu END=%lu\n", txSize, endSize);
                    }
                    // You may also compare 'txName' vs 'endName'
                }
                break; // done
            }
            else if (C == C_START) {
                fprintf(stderr, "[APP] Unexpected START during transfer\n");
                fclose(out); llclose(); return;
            }
            else {
                fprintf(stderr, "[APP] Unknown app packet C=0x%02X\n", C);
            }
        }

        fclose(out);

        // 5) Close link
        if (llclose() < 0) {
            fprintf(stderr, "[APP] llclose failed\n");
            return;
        }

        printf("[APP] Reception complete: announced %lu bytes, wrote %lu bytes\n",
               txSize, received);
    }
}
