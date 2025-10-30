#include "link_layer.h"
#include "serial_port.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define _POSIX_SOURCE 1

// ---- Constants ----
#define FLAG 0x7E

#define A_TX_CMD_OR_RX_REPLY 0x03
#define A_RX_CMD_OR_TX_REPLY 0x01

#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55

#define C_I0 0x00
#define C_I1 0x80

// ---- Byte stuffing ----
#define ESC 0x7D
#define ESC_XOR 0x20

#define MAX_IFRAME_DATA (MAX_PAYLOAD_SIZE + 8)
#define MAX_STUFFED(x) (2 * (x) + 16)

// ---- Link context ----
static int g_linkRole = 0;
static int g_timeoutSeconds = 0;
static int g_maxRetransmissions = 0;

static unsigned char g_txNextNs = 0;   // 0/1
static unsigned char g_rxExpected = 0; // 0/1

// ---- S/U FSM ----
typedef enum
{
  ST_START,
  ST_FLAG,
  ST_A,
  ST_C,
  ST_BCC_OK,
  ST_END
} SuState;

static void su_reset_state(SuState *state) { *state = ST_START; }

static int su_fsm_feed(SuState *state,
                       unsigned char incomingByte,
                       unsigned char expectedA,
                       unsigned char expectedC)
{
  static unsigned char lastA;
  static unsigned char lastC;

  switch (*state)
  {
  case ST_START:
    if (incomingByte == FLAG)
      *state = ST_FLAG;
    break;

  case ST_FLAG:
    if (incomingByte == expectedA)
    {
      lastA = incomingByte;
      *state = ST_A;
    }
    else if (incomingByte != FLAG)
    {
      *state = ST_START;
    }
    break;

  case ST_A:
    if (incomingByte == expectedC)
    {
      lastC = incomingByte;
      *state = ST_C;
    }
    else if (incomingByte == FLAG)
    {
      *state = ST_FLAG;
    }
    else
    {
      *state = ST_START;
    }
    break;

  case ST_C:
    if (incomingByte == (unsigned char)(lastA ^ lastC))
    {
      *state = ST_BCC_OK;
    }
    else if (incomingByte == FLAG)
    {
      *state = ST_FLAG;
    }
    else
    {
      *state = ST_START;
    }
    break;

  case ST_BCC_OK:
    if (incomingByte == FLAG)
    {
      *state = ST_END;
      return 1;
    }
    else
    {
      *state = ST_START;
    }
    break;

  case ST_END:
    break;
  }
  return 0;
}

// ---- Alarm / timeout ----
static volatile sig_atomic_t g_alarmEnabled = 0;
static volatile sig_atomic_t g_alarmCount = 0;

static void alarmHandler(int signal)
{
  (void)signal;
  g_alarmEnabled = 0;
  g_alarmCount++;
}

// ---- BCC2 / stuffing ----
static unsigned char compute_bcc2(const unsigned char *buf, int len)
{
  unsigned char x = 0x00;
  for (int i = 0; i < len; ++i)
    x ^= buf[i];
  return x;
}

static int stuff_byte(unsigned char in, unsigned char *out)
{
  if (in == FLAG || in == ESC)
  {
    out[0] = ESC;
    out[1] = (unsigned char)(in ^ ESC_XOR);
    return 2;
  }
  out[0] = in;
  return 1;
}

static int stuff_bytes(const unsigned char *in, int n, unsigned char *out)
{
  int outLen = 0;
  for (int i = 0; i < n; ++i)
    outLen += stuff_byte(in[i], out + outLen);
  return outLen;
}

static int destuff_bytes(const unsigned char *in, int n, unsigned char *out)
{
  int outLen = 0;
  for (int i = 0; i < n; ++i)
  {
    if (in[i] == ESC)
    {
      if (i + 1 >= n)
        return -1;
      out[outLen++] = (unsigned char)(in[++i] ^ ESC_XOR);
    }
    else
    {
      out[outLen++] = in[i];
    }
  }
  return outLen;
}

// ---- Build S/U: FLAG | A | C | A^C | FLAG ----
static int build_supervision_frame(unsigned char address,
                                   unsigned char control,
                                   unsigned char out[5])
{
  out[0] = FLAG;
  out[1] = address;
  out[2] = control;
  out[3] = (unsigned char)(address ^ control);
  out[4] = FLAG;
  return 5;
}

// ---- Build I-frame: FLAG | A | C | A^C | stuffed(DATA) | stuffed(BCC2) | FLAG ----
static int build_iframe(unsigned char address,
                        unsigned char control,
                        const unsigned char *data,
                        int dataLen,
                        unsigned char *out,
                        int outCapacity)
{
  if (dataLen < 0 || dataLen > MAX_IFRAME_DATA)
    return -1;

  int worstCase = 1 + 3 + (2 * dataLen) + 2 + 1;
  if (outCapacity < worstCase)
    return -1;

  int pos = 0;
  out[pos++] = FLAG;
  out[pos++] = address;
  out[pos++] = control;
  out[pos++] = (unsigned char)(address ^ control);

  if (dataLen > 0)
    pos += stuff_bytes(data, dataLen, out + pos);

  unsigned char bcc2 = compute_bcc2(data, dataLen);
  pos += stuff_byte(bcc2, out + pos);

  out[pos++] = FLAG;
  return pos;
}

// ---- Read any S/U ----
static int read_supervision_frame_any(unsigned char expectedAddress,
                                      unsigned char *controlOut)
{
  enum
  {
    ST_S,
    ST_F,
    ST_A,
    ST_C,
    ST_B
  } state = ST_S;
  unsigned char A = 0, C = 0, B = 0, byte = 0;

  for (;;)
  {
    int r = readByteSerialPort(&byte);
    if (r < 0)
    {
      fprintf(stderr, "read_supervision_frame_any: read error\n");
      return -1;
    }
    if (r == 0)
      continue;

    switch (state)
    {
    case ST_S:
      if (byte == FLAG)
        state = ST_F;
      break;

    case ST_F:
      if (byte == expectedAddress)
      {
        A = byte;
        state = ST_A;
      }
      else if (byte != FLAG)
      {
        state = ST_S;
      }
      break;

    case ST_A:
      C = byte;
      state = ST_C;
      break;

    case ST_C:
      B = byte;
      state = ST_B;
      break;

    case ST_B:
      if (byte == FLAG)
      {
        if (B == (unsigned char)(A ^ C))
        {
          if (controlOut)
            *controlOut = C;

          return 1;
        }
        else
        {
          fprintf(stderr, "read_supervision_frame_any: bad BCC1 (got 0x%02X expected 0x%02X)\n",
                  B, (unsigned char)(A ^ C));
        }
      }
      state = ST_S;
      break;
    }
  }
}

// ---- Read I-frame ----
static int read_iframe(unsigned char expectedAddress,
                       unsigned char *controlOut,
                       unsigned char *dataOut,
                       int maxDataLen)
{
  unsigned char byte = 0;
  while (1)
  {
    int r = readByteSerialPort(&byte);
    if (r < 0)
      return -1;
    if (r == 0)
      continue;
    if (byte == FLAG)
      break;
  }

  unsigned char body[MAX_STUFFED(MAX_IFRAME_DATA) + 8];
  int bodyLen = 0;

  while (1)
  {
    int r = readByteSerialPort(&byte);
    if (r < 0)
      return -1;
    if (r == 0)
      continue;
    if (byte == FLAG)
      break;
    if (bodyLen >= (int)sizeof(body))
      return -1;
    body[bodyLen++] = byte;
  }

  if (bodyLen < 4)
    return -1;

  unsigned char A = body[0];
  unsigned char C = body[1];
  unsigned char B1 = body[2];

  if (A != expectedAddress)
    return -1;
  if (B1 != (unsigned char)(A ^ C))
    return -1;

  int stuffedLen = bodyLen - 3;
  if (stuffedLen < 1)
  {
    if (controlOut)
      *controlOut = C;
    return 0;
  }

  unsigned char destuffed[MAX_STUFFED(MAX_IFRAME_DATA) + 8];
  int destuffedLen = destuff_bytes(body + 3, stuffedLen, destuffed);
  if (destuffedLen < 1)
    return -1;

  int payloadLen = destuffedLen - 1;
  unsigned char computedBcc2 = compute_bcc2(destuffed, payloadLen);
  unsigned char bcc2 = destuffed[payloadLen];

  if (computedBcc2 != bcc2)
    return -1;
  if (payloadLen > maxDataLen)
    return -1;

  if (payloadLen > 0)
    memcpy(dataOut, destuffed, payloadLen);
  if (controlOut)
    *controlOut = C;

  return payloadLen;
}

// ---- Helpers (RR/REJ) ----
static int send_rr(unsigned char nrExpected)
{
  unsigned char control;
  if (nrExpected == 0)
  {
    control = C_RR0;
  }
  else
  {
    control = C_RR1;
  }

  unsigned char frame[5];
  build_supervision_frame(A_RX_CMD_OR_TX_REPLY, control, frame);
  int wrote = writeBytesSerialPort(frame, 5);
  if (wrote == 5)
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

static int send_rej(unsigned char nrExpected)
{
  unsigned char control;
  if (nrExpected == 0)
  {
    control = C_REJ0;
  }
  else
  {
    control = C_REJ1;
  }

  unsigned char frame[5];
  build_supervision_frame(A_RX_CMD_OR_TX_REPLY, control, frame);
  int wrote = writeBytesSerialPort(frame, 5);

  fprintf(stderr, "llread: sent REJ (expected sequence %u)\n", (unsigned)nrExpected);

  if (wrote == 5)
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

// ---- llopen ----
int llopen(LinkLayer connectionParameters)
{
  if (openSerialPort(connectionParameters.serialPort,
                     connectionParameters.baudRate) < 0)
    return -1;

  g_linkRole = connectionParameters.role;
  g_timeoutSeconds = connectionParameters.timeout;
  g_maxRetransmissions = connectionParameters.nRetransmissions;

  if (connectionParameters.role == LlTx)
  {
    struct sigaction act = {0};
    act.sa_handler = alarmHandler;
    sigemptyset(&act.sa_mask);
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
      closeSerialPort();
      return -1;
    }

    unsigned char setFrame[5];
    build_supervision_frame(A_TX_CMD_OR_RX_REPLY, C_SET, setFrame);

    SuState suState;
    su_reset_state(&suState);

    unsigned char incomingByte = 0;
    int attempts = 0;
    g_alarmEnabled = 0;
    g_alarmCount = 0;

    while (attempts < connectionParameters.nRetransmissions)
    {
      if (!g_alarmEnabled)
      {
        if (writeBytesSerialPort(setFrame, 5) != 5)
        {
          closeSerialPort();
          return -1;
        }
        alarm(connectionParameters.timeout);
        g_alarmEnabled = 1;
      }

      int r = readByteSerialPort(&incomingByte);
      if (r == 1)
      {
        if (su_fsm_feed(&suState, incomingByte, A_TX_CMD_OR_RX_REPLY, C_UA))
        {
          alarm(0);
          g_alarmEnabled = 0;
          g_txNextNs = 0;
          g_rxExpected = 0;

          fprintf(stderr, "llopen: connection established\n");

          return 0;
        }
        continue;
      }

      if (!g_alarmEnabled)
      {
        attempts++;
        su_reset_state(&suState);

        fprintf(stderr, "llopen: timeout waiting for UA, retrying (attempt %d/%d)\n",
                attempts, connectionParameters.nRetransmissions);

        if (attempts >= connectionParameters.nRetransmissions)
        {
          closeSerialPort();
          return -1;
        }
      }
    }

    closeSerialPort();
    return -1;
  }
  else
  {
    SuState suState;
    su_reset_state(&suState);

    unsigned char incomingByte = 0;

    while (1)
    {
      int r = readByteSerialPort(&incomingByte);
      if (r < 0)
      {
        closeSerialPort();
        return -1;
      }
      if (r == 0)
        continue;
      if (su_fsm_feed(&suState, incomingByte, A_TX_CMD_OR_RX_REPLY, C_SET))
        break;
    }

    unsigned char ua[5];
    build_supervision_frame(A_TX_CMD_OR_RX_REPLY, C_UA, ua);
    if (writeBytesSerialPort(ua, 5) != 5)
    {
      closeSerialPort();
      return -1;
    }

    g_txNextNs = 0;
    g_rxExpected = 0;

    return 0;
  }
}

// ---- llwrite ----
int llwrite(const unsigned char *appPayload, int appPayloadSize)
{
  if (appPayloadSize < 0 || appPayloadSize > MAX_IFRAME_DATA)
  {
    fprintf(stderr, "llwrite: invalid payload size = %d\n", appPayloadSize);
    return -1;
  }

  struct sigaction act = {0};
  act.sa_handler = alarmHandler;
  sigemptyset(&act.sa_mask);
  if (sigaction(SIGALRM, &act, NULL) == -1)
  {
    perror("llwrite: sigaction failed");
    return -1;
  }

  unsigned char control;
  if (g_txNextNs == 0)
  {
    control = C_I0;
  }
  else
  {
    control = C_I1;
  }

  unsigned char txFrame[MAX_STUFFED(MAX_IFRAME_DATA) + 16];
  int txLen = build_iframe(A_TX_CMD_OR_RX_REPLY, control,
                           appPayload, appPayloadSize,
                           txFrame, sizeof(txFrame));
  if (txLen < 0)
  {
    fprintf(stderr, "llwrite: build_iframe failed (Ns=%u)\n", (unsigned)g_txNextNs);
    return -1;
  }

  int attempts = 0;
  g_alarmEnabled = 0;
  g_alarmCount = 0;

  while (attempts < g_maxRetransmissions)
  {
    if (!g_alarmEnabled)
    {
      int written = writeBytesSerialPort(txFrame, txLen);
      if (written != txLen)
      {
        fprintf(stderr, "llwrite: writeBytesSerialPort wrote %d/%d bytes\n", written, txLen);
        return -1;
      }
      alarm(g_timeoutSeconds);
      g_alarmEnabled = 1;
    }

    unsigned char suControl = 0;
    int ok = read_supervision_frame_any(A_RX_CMD_OR_TX_REPLY, &suControl);

    if (ok == 1)
    {
      if (suControl == C_RR0 || suControl == C_RR1)
      {
        unsigned char nr;
        if (suControl == C_RR1)
        {
          nr = 1;
        }
        else
        {
          nr = 0;
        }

        if (nr == (unsigned char)(g_txNextNs ^ 1))
        {
          alarm(0);
          g_alarmEnabled = 0;
          g_txNextNs ^= 1;
          return appPayloadSize;
        }
        else
        {
          fprintf(stderr, "llwrite: received RR for wrong frame (Nr=%u expected=%u)\n",
                  (unsigned)nr, (unsigned)(g_txNextNs ^ 1));
          continue;
        }
      }
      else if (suControl == C_REJ0 || suControl == C_REJ1)
      {
        attempts++;
        alarm(0);
        g_alarmEnabled = 0;
        fprintf(stderr, "llwrite: got REJ, retransmitting (attempt %d/%d)\n",
                attempts, g_maxRetransmissions);
        continue;
      }
      else if (suControl == C_DISC)
      {
        alarm(0);
        g_alarmEnabled = 0;
        fprintf(stderr, "llwrite: received DISC during transfer\n");
        return -1;
      }
      else
      {
        continue;
      }
    }

    if (!g_alarmEnabled)
    {
      attempts++;
      fprintf(stderr, "llwrite: timeout, retransmitting (attempt %d/%d)\n",
              attempts, g_maxRetransmissions);
    }
    else
    {
      attempts++;
      alarm(0);
      g_alarmEnabled = 0;
      fprintf(stderr, "llwrite: read error, retransmitting (attempt %d/%d)\n",
              attempts, g_maxRetransmissions);
    }
  }

  alarm(0);
  g_alarmEnabled = 0;
  fprintf(stderr, "llwrite: failed after %d attempts\n", g_maxRetransmissions);
  return -1;
}

// ---- llread ----
int llread(unsigned char *appPacketOut)
{
  for (;;)
  {
    unsigned char control = 0;
    int payloadLen = read_iframe(A_TX_CMD_OR_RX_REPLY, &control,
                                 appPacketOut, MAX_IFRAME_DATA);
    if (payloadLen < 0)
    {
      if (send_rej(g_rxExpected) < 0)
        return -1;
      continue;
    }

    unsigned char ns;
    if (control == C_I1)
    {
      ns = 1;
    }
    else
    {
      ns = 0;
    }

    if (ns == g_rxExpected)
    {
      if (send_rr(g_rxExpected ^ 1) < 0)
        return -1;
      g_rxExpected ^= 1;
      return payloadLen;
    }
    else
    {
      if (send_rr(g_rxExpected) < 0)
        return -1;
    }
  }
}

// ---- llclose ----
int llclose()
{
  struct sigaction act = {0};
  act.sa_handler = alarmHandler;
  sigemptyset(&act.sa_mask);
  if (sigaction(SIGALRM, &act, NULL) == -1)
  {
    closeSerialPort();
    return -1;
  }

  if (g_linkRole == LlTx)
  {
    unsigned char discTx[5];
    build_supervision_frame(A_TX_CMD_OR_RX_REPLY, C_DISC, discTx);

    int attempts = 0;
    g_alarmEnabled = 0;
    g_alarmCount = 0;

    while (attempts < g_maxRetransmissions)
    {
      if (!g_alarmEnabled)
      {
        if (writeBytesSerialPort(discTx, 5) != 5)
        {
          closeSerialPort();
          return -1;
        }
        alarm(g_timeoutSeconds);
        g_alarmEnabled = 1;
      }

      unsigned char control = 0;
      int ok = read_supervision_frame_any(A_RX_CMD_OR_TX_REPLY, &control);
      if (ok == 1)
      {
        if (control == C_DISC)
        {
          alarm(0);
          g_alarmEnabled = 0;

          unsigned char uaTx[5];
          build_supervision_frame(A_RX_CMD_OR_TX_REPLY, C_UA, uaTx);
          if (writeBytesSerialPort(uaTx, 5) != 5)
          {
            closeSerialPort();
            return -1;
          }
          closeSerialPort();

          fprintf(stderr, "llclose: link closed\n");

          return 0;
        }
        continue;
      }

      if (!g_alarmEnabled)
      {
        attempts++;
      }
      else
      {
        attempts++;
        alarm(0);
        g_alarmEnabled = 0;
      }
    }

    alarm(0);
    g_alarmEnabled = 0;
    closeSerialPort();
    return -1;
  }
  else
  {
    while (1)
    {
      unsigned char control = 0;
      int ok = read_supervision_frame_any(A_TX_CMD_OR_RX_REPLY, &control);
      if (ok < 0)
      {
        closeSerialPort();
        return -1;
      }
      if (ok == 1 && control == C_DISC)
        break;
    }

    unsigned char discRx[5];
    build_supervision_frame(A_RX_CMD_OR_TX_REPLY, C_DISC, discRx);
    if (writeBytesSerialPort(discRx, 5) != 5)
    {
      closeSerialPort();
      return -1;
    }

    while (1)
    {
      unsigned char control = 0;
      int ok = read_supervision_frame_any(A_RX_CMD_OR_TX_REPLY, &control);
      if (ok < 0)
      {
        closeSerialPort();
        return -1;
      }
      if (ok == 1 && control == C_UA)
        break;
    }

    closeSerialPort();

    fprintf(stderr, "llclose: link closed\n");

    return 0;
  }
}
