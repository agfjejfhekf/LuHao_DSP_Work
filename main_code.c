#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

/*
 * F28335 + TLV320AIC23 G.711 audio experiment.
 *
 * Change gCodecMode in the CCS Expressions window:
 *   0: audio bypass
 *   1: G.711 A-law encode/decode
 *   2: G.711 mu-law encode/decode
 *   3: G.721 32 kbit/s ADPCM encode/decode
 *   4: G.722 64 kbit/s wideband ADPCM encode/decode
 *   5: G.723 24 kbit/s ADPCM encode/decode
 *   6: G.723 40 kbit/s ADPCM encode/decode
 */

#define AUDIOEN                 (*(volatile Uint16 *)0x180002)
#define AIC23_I2C_ADDRESS       0x001A

#define CODEC_MODE_BYPASS       0U
#define CODEC_MODE_ALAW         1U
#define CODEC_MODE_ULAW         2U
#define CODEC_MODE_G721         3U
#define CODEC_MODE_G722         4U
#define CODEC_MODE_G723_24      5U
#define CODEC_MODE_G723_40      6U

#define AUDIO_BUFFER_LENGTH     1024U
#define ULAW_BIAS               0x84L
#define ULAW_CLIP               32635L

volatile Uint16 gCodecMode = CODEC_MODE_ALAW;
volatile Uint16 gEncodedBuffer[AUDIO_BUFFER_LENGTH];
volatile Uint16 gBufferIndex = 0U;

typedef struct
{
    long yl, yu, dms, dml, ap;
    long a[2], b[6], pk[2], dq[6], sr[2], td;
} G721State;

G721State gG721EncoderLeft;
G721State gG721EncoderRight;
G721State gG721DecoderLeft;
G721State gG721DecoderRight;

typedef struct
{
    int nb, det, s, sz, r;
    int p[2], a[2], b[6], d[7];
} G722Band;

typedef struct
{
    int x[12], y[12], ptr;
    G722Band band[2];
} G722State;

G722State gG722EncoderLeft;
G722State gG722EncoderRight;
G722State gG722DecoderLeft;
G722State gG722DecoderRight;

G721State gG723EncoderLeft;
G721State gG723EncoderRight;
G721State gG723DecoderLeft;
G721State gG723DecoderRight;

static int gG722InputLeft[2];
static int gG722InputRight[2];
static int gG722QueuedLeft;
static int gG722QueuedRight;
static Uint16 gG722Phase;

static void I2CA_Init(void);
static Uint16 AIC23Write(Uint16 address, Uint16 data);
static void Delay(Uint16 time);
static void AIC23Init(void);
static Uint16 G711EncodeStereo(int left, int right, Uint16 mode);
static unsigned char LinearToALaw(int pcm);
static int ALawToLinear(unsigned char code);
static unsigned char LinearToULaw(int pcm);
static int ULawToLinear(unsigned char code);
static Uint16 FindSegment(unsigned long value,
                          const unsigned long *table,
                          Uint16 tableSize);
static void G721_Init(G721State *state);
static unsigned int G721_Encode(int sample, G721State *state);
static int G721_Decode(unsigned int code, G721State *state);
static void G721_ResetAll(void);
static unsigned int G723_Encode(int sample, G721State *state, Uint16 bits);
static int G723_Decode(unsigned int code, G721State *state, Uint16 bits);
static void G723_ResetAll(void);
static void G722_Init(G722State *state);
static unsigned int G722_EncodePair(int sample0, int sample1,
                                    G722State *state);
static void G722_DecodeByte(unsigned int code, int *sample0, int *sample1,
                            G722State *state);
static void G722_ResetAll(void);
static void SetCodecSampleRate(Uint16 mode);

void main(void)
{
    int left;
    int right;
    Uint16 packedCode;
    unsigned char leftCode;
    unsigned char rightCode;
    Uint16 previousMode;
    int decoded0;
    int decoded1;
    Uint16 g723Bits;

    InitSysCtrl();
    InitXintf16Gpio();
    InitMcbspaGpio();
    InitI2CGpio();
    AUDIOEN = 0U;

    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    I2CA_Init();
    AIC23Init();
    InitMcbspa();
    G721_ResetAll();
    G722_ResetAll();
    G723_ResetAll();
    SetCodecSampleRate(gCodecMode);
    previousMode = gCodecMode;

    for (;;)
    {
        left = (int)McbspaRegs.DRR1.all;
        right = (int)McbspaRegs.DRR2.all;

        if (previousMode != gCodecMode)
        {
            SetCodecSampleRate(gCodecMode);
            if (gCodecMode == CODEC_MODE_G721)
            {
                G721_ResetAll();
            }
            else if (gCodecMode == CODEC_MODE_G722)
            {
                G722_ResetAll();
            }
            else if ((gCodecMode == CODEC_MODE_G723_24) ||
                     (gCodecMode == CODEC_MODE_G723_40))
            {
                G723_ResetAll();
            }
            previousMode = gCodecMode;
        }

        if (gCodecMode == CODEC_MODE_ALAW)
        {
            packedCode = G711EncodeStereo(left, right, CODEC_MODE_ALAW);
            leftCode = (unsigned char)((packedCode >> 8) & 0x00FFU);
            rightCode = (unsigned char)(packedCode & 0x00FFU);
            left = ALawToLinear(leftCode);
            right = ALawToLinear(rightCode);
            gEncodedBuffer[gBufferIndex] = packedCode;
        }
        else if (gCodecMode == CODEC_MODE_ULAW)
        {
            packedCode = G711EncodeStereo(left, right, CODEC_MODE_ULAW);
            leftCode = (unsigned char)((packedCode >> 8) & 0x00FFU);
            rightCode = (unsigned char)(packedCode & 0x00FFU);
            left = ULawToLinear(leftCode);
            right = ULawToLinear(rightCode);
            gEncodedBuffer[gBufferIndex] = packedCode;
        }
        else if (gCodecMode == CODEC_MODE_G721)
        {
            leftCode = (unsigned char)
                G721_Encode(left, &gG721EncoderLeft);
            rightCode = (unsigned char)
                G721_Encode(right, &gG721EncoderRight);
            packedCode = (Uint16)((((Uint16)leftCode & 0x000FU) << 4) |
                                  ((Uint16)rightCode & 0x000FU));
            left = G721_Decode(leftCode, &gG721DecoderLeft);
            right = G721_Decode(rightCode, &gG721DecoderRight);
            gEncodedBuffer[gBufferIndex] = packedCode;
        }
        else if (gCodecMode == CODEC_MODE_G722)
        {
            gG722InputLeft[gG722Phase] = left;
            gG722InputRight[gG722Phase] = right;
            if (gG722Phase == 0U)
            {
                left = gG722QueuedLeft;
                right = gG722QueuedRight;
                gEncodedBuffer[gBufferIndex] = 0U;
                gG722Phase = 1U;
            }
            else
            {
                leftCode = (unsigned char)
                    G722_EncodePair(gG722InputLeft[0], gG722InputLeft[1],
                                    &gG722EncoderLeft);
                rightCode = (unsigned char)
                    G722_EncodePair(gG722InputRight[0], gG722InputRight[1],
                                    &gG722EncoderRight);
                G722_DecodeByte(leftCode, &decoded0, &decoded1,
                                &gG722DecoderLeft);
                left = decoded0;
                gG722QueuedLeft = decoded1;
                G722_DecodeByte(rightCode, &decoded0, &decoded1,
                                &gG722DecoderRight);
                right = decoded0;
                gG722QueuedRight = decoded1;
                packedCode = (Uint16)((((Uint16)leftCode) << 8) |
                                      (Uint16)rightCode);
                gEncodedBuffer[gBufferIndex] = packedCode;
                gG722Phase = 0U;
            }
        }
        else if ((gCodecMode == CODEC_MODE_G723_24) ||
                 (gCodecMode == CODEC_MODE_G723_40))
        {
            g723Bits = (gCodecMode == CODEC_MODE_G723_24) ? 3U : 5U;
            leftCode = (unsigned char)
                G723_Encode(left, &gG723EncoderLeft, g723Bits);
            rightCode = (unsigned char)
                G723_Encode(right, &gG723EncoderRight, g723Bits);
            packedCode = (Uint16)((((Uint16)leftCode) << 8) |
                                  (Uint16)rightCode);
            left = G723_Decode(leftCode, &gG723DecoderLeft, g723Bits);
            right = G723_Decode(rightCode, &gG723DecoderRight, g723Bits);
            gEncodedBuffer[gBufferIndex] = packedCode;
        }
        else
        {
            gEncodedBuffer[gBufferIndex] = 0U;
        }

        McbspaRegs.DXR1.all = (Uint16)left;
        McbspaRegs.DXR2.all = (Uint16)right;

        gBufferIndex++;
        if (gBufferIndex >= AUDIO_BUFFER_LENGTH)
        {
            gBufferIndex = 0U;
        }
    }
}

static void I2CA_Init(void)
{
    I2caRegs.I2CSAR = AIC23_I2C_ADDRESS;

#if (CPU_FRQ_150MHZ)
    I2caRegs.I2CPSC.all = 14;
#endif
#if (CPU_FRQ_100MHZ)
    I2caRegs.I2CPSC.all = 9;
#endif

    I2caRegs.I2CCLKL = 100;
    I2caRegs.I2CCLKH = 100;
    I2caRegs.I2CIER.all = 0x0024;
    I2caRegs.I2CMDR.all = 0x0420;
    I2caRegs.I2CFFTX.all = 0x6000;
    I2caRegs.I2CFFRX.all = 0x2040;
}

static Uint16 AIC23Write(Uint16 address, Uint16 data)
{
    Uint32 timeout;

    timeout = 100000UL;
    while ((I2caRegs.I2CMDR.bit.STP == 1U) && (timeout > 0UL))
    {
        timeout--;
    }
    if (timeout == 0UL)
    {
        return I2C_STP_NOT_READY_ERROR;
    }

    timeout = 100000UL;
    while ((I2caRegs.I2CSTR.bit.BB == 1U) && (timeout > 0UL))
    {
        timeout--;
    }
    if (timeout == 0UL)
    {
        return I2C_BUS_BUSY_ERROR;
    }

    I2caRegs.I2CSAR = AIC23_I2C_ADDRESS;
    I2caRegs.I2CCNT = 2;
    I2caRegs.I2CDXR = address;
    I2caRegs.I2CDXR = data;
    I2caRegs.I2CMDR.all = 0x6E20;

    return I2C_SUCCESS;
}

static void Delay(Uint16 time)
{
    volatile Uint16 i;
    volatile Uint16 j;

    for (i = 0U; i < time; i++)
    {
        for (j = 0U; j < 1024U; j++)
        {
            asm(" NOP");
        }
    }
}

static void AIC23Init(void)
{
    (void)AIC23Write(0x00, 0x00);
    Delay(100);
    (void)AIC23Write(0x02, 0x00);
    Delay(100);
    (void)AIC23Write(0x04, 0x7F);
    Delay(100);
    (void)AIC23Write(0x06, 0x7F);
    Delay(100);
    (void)AIC23Write(0x08, 0x14);
    Delay(100);
    (void)AIC23Write(0x0A, 0x00);
    Delay(100);
    (void)AIC23Write(0x0C, 0x00);
    Delay(100);
    (void)AIC23Write(0x0E, 0x43);
    Delay(100);
    (void)AIC23Write(0x10, 0x23);
    Delay(100);
    (void)AIC23Write(0x12, 0x01);
    Delay(100);
}

static Uint16 G711EncodeStereo(int left, int right, Uint16 mode)
{
    unsigned char leftCode;
    unsigned char rightCode;

    if (mode == CODEC_MODE_ULAW)
    {
        leftCode = LinearToULaw(left);
        rightCode = LinearToULaw(right);
    }
    else
    {
        leftCode = LinearToALaw(left);
        rightCode = LinearToALaw(right);
    }

    return (Uint16)((((Uint16)leftCode & 0x00FFU) << 8) |
                    ((Uint16)rightCode & 0x00FFU));
}

static Uint16 FindSegment(unsigned long value,
                          const unsigned long *table,
                          Uint16 tableSize)
{
    Uint16 i;

    for (i = 0U; i < tableSize; i++)
    {
        if (value <= table[i])
        {
            return i;
        }
    }
    return tableSize;
}

static unsigned char LinearToALaw(int pcm)
{
    static const unsigned long segmentEnd[8] =
    {
        0x001FUL, 0x003FUL, 0x007FUL, 0x00FFUL,
        0x01FFUL, 0x03FFUL, 0x07FFUL, 0x0FFFUL
    };
    long sample;
    unsigned long magnitude;
    Uint16 mask;
    Uint16 segment;
    Uint16 code;

    sample = (long)pcm;
    sample >>= 3;

    if (sample >= 0L)
    {
        mask = 0x00D5U;
        magnitude = (unsigned long)sample;
    }
    else
    {
        mask = 0x0055U;
        magnitude = (unsigned long)(-sample - 1L);
    }

    segment = FindSegment(magnitude, segmentEnd, 8U);
    if (segment >= 8U)
    {
        return (unsigned char)(0x007FU ^ mask);
    }

    code = (Uint16)(segment << 4);
    if (segment < 2U)
    {
        code |= (Uint16)((magnitude >> 1) & 0x000FUL);
    }
    else
    {
        code |= (Uint16)((magnitude >> segment) & 0x000FUL);
    }

    return (unsigned char)((code ^ mask) & 0x00FFU);
}

static int ALawToLinear(unsigned char inputCode)
{
    Uint16 code;
    Uint16 segment;
    long sample;

    code = ((Uint16)inputCode & 0x00FFU) ^ 0x0055U;
    sample = (long)((code & 0x000FU) << 4);
    segment = (code & 0x0070U) >> 4;

    if (segment == 0U)
    {
        sample += 8L;
    }
    else if (segment == 1U)
    {
        sample += 0x108L;
    }
    else
    {
        sample += 0x108L;
        sample <<= (segment - 1U);
    }

    return (code & 0x0080U) ? (int)sample : (int)(-sample);
}

static unsigned char LinearToULaw(int pcm)
{
    static const unsigned long segmentEnd[8] =
    {
        0x00FFUL, 0x01FFUL, 0x03FFUL, 0x07FFUL,
        0x0FFFUL, 0x1FFFUL, 0x3FFFUL, 0x7FFFUL
    };
    long sample;
    unsigned long magnitude;
    Uint16 mask;
    Uint16 segment;
    Uint16 code;

    sample = (long)pcm;
    if (sample < 0L)
    {
        magnitude = (unsigned long)(-sample);
        mask = 0x007FU;
    }
    else
    {
        magnitude = (unsigned long)sample;
        mask = 0x00FFU;
    }

    if (magnitude > ULAW_CLIP)
    {
        magnitude = ULAW_CLIP;
    }
    magnitude += ULAW_BIAS;

    segment = FindSegment(magnitude, segmentEnd, 8U);
    code = (Uint16)((segment << 4) |
                    ((magnitude >> (segment + 3U)) & 0x000FUL));

    return (unsigned char)((code ^ mask) & 0x00FFU);
}

static int ULawToLinear(unsigned char inputCode)
{
    Uint16 code;
    long sample;

    code = (~(Uint16)inputCode) & 0x00FFU;
    sample = (long)(((code & 0x000FU) << 3) + ULAW_BIAS);
    sample <<= ((code & 0x0070U) >> 4);

    if (code & 0x0080U)
    {
        return (int)(ULAW_BIAS - sample);
    }
    return (int)(sample - ULAW_BIAS);
}

/* G.721 32 kbit/s ADPCM implementation. */
static const long g721Power2[15] =
{1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384};
static const long g721Qtab[7] = {-124,80,178,246,300,349,400};
static const long g721Dqln[16] =
{-2048,4,135,213,273,323,373,425,425,373,323,273,213,135,4,-2048};
static const long g721Wi[16] =
{-12,18,41,64,112,198,355,1122,1122,355,198,112,64,41,18,-12};
static const long g721Fi[16] =
{0,0,0,512,512,512,1536,3584,3584,1536,512,512,512,0,0,0};

static long G721Abs(long x)
{
    return (x < 0L) ? -x : x;
}

static long G721Quan(long value, const long *table, int size)
{
    int i;
    for (i = 0; (i < size) && (value >= table[i]); i++) {}
    return (long)i;
}

static long G721Fmult(long an, long srn)
{
    long magnitude, exponent, mantissa;
    long resultExponent, resultMantissa, result;

    magnitude = (an > 0L) ? an : ((-an) & 0x1FFFL);
    exponent = G721Quan(magnitude, g721Power2, 15) - 6L;
    mantissa = (magnitude == 0L) ? 32L :
               ((exponent >= 0L) ?
                (magnitude >> exponent) : (magnitude << -exponent));
    resultExponent = exponent + ((srn >> 6) & 15L) - 13L;
    resultMantissa = (mantissa * (srn & 63L)) >> 4;
    result = (resultExponent >= 0L) ?
             ((resultMantissa << resultExponent) & 0x7FFFL) :
             (resultMantissa >> -resultExponent);
    return ((an ^ srn) < 0L) ? -result : result;
}

static long G721PredictZero(G721State *state)
{
    int i;
    long estimate;

    estimate = G721Fmult(state->b[0] >> 2, state->dq[0]);
    for (i = 1; i < 6; i++)
    {
        estimate += G721Fmult(state->b[i] >> 2, state->dq[i]);
    }
    return estimate;
}

static long G721PredictPole(G721State *state)
{
    return G721Fmult(state->a[1] >> 2, state->sr[1]) +
           G721Fmult(state->a[0] >> 2, state->sr[0]);
}

static long G721StepSize(G721State *state)
{
    long y, difference, coefficient;

    if (state->ap >= 256L)
    {
        return state->yu;
    }
    y = state->yl >> 6;
    difference = state->yu - y;
    coefficient = state->ap >> 2;
    if (difference > 0L)
    {
        y += (difference * coefficient) >> 6;
    }
    else if (difference < 0L)
    {
        y += (difference * coefficient + 63L) >> 6;
    }
    return y;
}

static long G721Quantize(long difference, long step)
{
    long magnitude, exponent, mantissa, logMagnitude, code;

    magnitude = G721Abs(difference);
    exponent = G721Quan(magnitude >> 1, g721Power2, 15);
    mantissa = ((magnitude << 7) >> exponent) & 127L;
    logMagnitude = (exponent << 7) + mantissa - (step >> 2);
    code = G721Quan(logMagnitude, g721Qtab, 7);
    if (difference < 0L)
    {
        return 15L - code;
    }
    return (code == 0L) ? 15L : code;
}

static long G721Reconstruct(long sign, long dqln, long step)
{
    long logDifference, exponent, mantissa, difference;

    logDifference = dqln + (step >> 2);
    if (logDifference < 0L)
    {
        return sign ? -32768L : 0L;
    }
    exponent = (logDifference >> 7) & 15L;
    mantissa = 128L + (logDifference & 127L);
    difference = (mantissa << 7) >> (14L - exponent);
    return sign ? difference - 32768L : difference;
}

static void G72xUpdate(Uint16 codeBits, long step, long wi, long fi,
                       long dq, long sr, long dqsez, G721State *state)
{
    int i;
    long magnitude, exponent, a2p, a1Limit, signDifference, fa1;
    long transition, ylInteger, ylFraction, threshold1, threshold2;
    long dqThreshold, currentSign;

    currentSign = (dqsez < 0L);
    magnitude = dq & 0x7FFFL;
    ylInteger = state->yl >> 15;
    ylFraction = (state->yl >> 10) & 31L;
    threshold1 = (32L + ylFraction) << ylInteger;
    threshold2 = (ylInteger > 9L) ? (31L << 10) : threshold1;
    dqThreshold = (threshold2 + (threshold2 >> 1)) >> 1;
    transition = ((state->td != 0L) && (magnitude > dqThreshold));

    state->yu = step + ((wi - step) >> 5);
    if (state->yu < 544L) state->yu = 544L;
    else if (state->yu > 5120L) state->yu = 5120L;
    state->yl += state->yu + ((-state->yl) >> 6);

    a2p = 0L;
    if (transition)
    {
        state->a[0] = state->a[1] = 0L;
        for (i = 0; i < 6; i++) state->b[i] = 0L;
    }
    else
    {
        signDifference = currentSign ^ state->pk[0];
        a2p = state->a[1] - (state->a[1] >> 7);
        if (dqsez != 0L)
        {
            fa1 = signDifference ? state->a[0] : -state->a[0];
            if (fa1 < -8191L) a2p -= 256L;
            else if (fa1 > 8191L) a2p += 255L;
            else a2p += fa1 >> 5;

            if (currentSign ^ state->pk[1])
            {
                if (a2p <= -12160L) a2p = -12288L;
                else if (a2p >= 12416L) a2p = 12288L;
                else a2p -= 128L;
            }
            else if (a2p <= -12416L) a2p = -12288L;
            else if (a2p >= 12160L) a2p = 12288L;
            else a2p += 128L;
        }
        state->a[1] = a2p;
        state->a[0] -= state->a[0] >> 8;
        if (dqsez != 0L)
        {
            state->a[0] += signDifference ? -192L : 192L;
        }
        a1Limit = 15360L - a2p;
        if (state->a[0] < -a1Limit) state->a[0] = -a1Limit;
        else if (state->a[0] > a1Limit) state->a[0] = a1Limit;

        for (i = 0; i < 6; i++)
        {
            state->b[i] -= state->b[i] >>
                           ((codeBits == 5U) ? 9 : 8);
            if (dq & 0x7FFFL)
            {
                state->b[i] += ((dq ^ state->dq[i]) >= 0L) ?
                               128L : -128L;
            }
        }
    }

    for (i = 5; i > 0; i--) state->dq[i] = state->dq[i - 1];
    if (magnitude == 0L)
    {
        state->dq[0] = (dq >= 0L) ? 0x20L : 0xFC20L;
    }
    else
    {
        exponent = G721Quan(magnitude, g721Power2, 15);
        state->dq[0] = (exponent << 6) +
                       ((magnitude << 6) >> exponent);
        if (dq < 0L) state->dq[0] -= 0x400L;
    }

    state->sr[1] = state->sr[0];
    if (sr == 0L)
    {
        state->sr[0] = 0x20L;
    }
    else if (sr > 0L)
    {
        exponent = G721Quan(sr, g721Power2, 15);
        state->sr[0] = (exponent << 6) + ((sr << 6) >> exponent);
    }
    else if (sr > -32768L)
    {
        magnitude = -sr;
        exponent = G721Quan(magnitude, g721Power2, 15);
        state->sr[0] = (exponent << 6) +
                       ((magnitude << 6) >> exponent) - 0x400L;
    }
    else
    {
        state->sr[0] = 0xFC20L;
    }

    state->pk[1] = state->pk[0];
    state->pk[0] = currentSign;
    if (transition) state->td = 0L;
    else state->td = (a2p < -11776L);
    state->dms += (fi - state->dms) >> 5;
    state->dml += ((fi << 2) - state->dml) >> 7;
    if (transition || (step < 1536L) || state->td ||
        (G721Abs((state->dms << 2) - state->dml) >=
         (state->dml >> 3)))
    {
        state->ap += (512L - state->ap) >> 4;
    }
    else
    {
        state->ap += (-state->ap) >> 4;
    }
}

static void G721_Init(G721State *state)
{
    int i;

    state->yl = 34816L;
    state->yu = 544L;
    state->dms = state->dml = state->ap = 0L;
    for (i = 0; i < 2; i++)
    {
        state->a[i] = state->pk[i] = 0L;
        state->sr[i] = 32L;
    }
    for (i = 0; i < 6; i++)
    {
        state->b[i] = 0L;
        state->dq[i] = 32L;
    }
    state->td = 0L;
}

static unsigned int G721_Encode(int sample, G721State *state)
{
    long zeroEstimate, poleZeroEstimate, estimate, difference;
    long step, code, reconstructedDifference, reconstructedSignal;

    zeroEstimate = G721PredictZero(state);
    poleZeroEstimate = zeroEstimate >> 1;
    estimate = (zeroEstimate + G721PredictPole(state)) >> 1;
    difference = ((long)sample >> 2) - estimate;
    step = G721StepSize(state);
    code = G721Quantize(difference, step);
    reconstructedDifference =
        G721Reconstruct(code & 8L, g721Dqln[code], step);
    reconstructedSignal = (reconstructedDifference < 0L) ?
        estimate - (reconstructedDifference & 0x3FFFL) :
        estimate + reconstructedDifference;
    G72xUpdate(4U, step, g721Wi[code] << 5, g721Fi[code],
               reconstructedDifference, reconstructedSignal,
               reconstructedSignal + poleZeroEstimate - estimate, state);
    return (unsigned int)(code & 15L);
}

static int G721_Decode(unsigned int input, G721State *state)
{
    long code, zeroEstimate, poleZeroEstimate, estimate, step;
    long reconstructedDifference, reconstructedSignal, output;

    code = input & 15U;
    zeroEstimate = G721PredictZero(state);
    poleZeroEstimate = zeroEstimate >> 1;
    estimate = (zeroEstimate + G721PredictPole(state)) >> 1;
    step = G721StepSize(state);
    reconstructedDifference =
        G721Reconstruct(code & 8L, g721Dqln[code], step);
    reconstructedSignal = (reconstructedDifference < 0L) ?
        estimate - (reconstructedDifference & 0x3FFFL) :
        estimate + reconstructedDifference;
    G72xUpdate(4U, step, g721Wi[code] << 5, g721Fi[code],
               reconstructedDifference, reconstructedSignal,
               reconstructedSignal - estimate + poleZeroEstimate, state);
    output = reconstructedSignal << 2;
    if (output > 32767L) output = 32767L;
    else if (output < -32768L) output = -32768L;
    return (int)output;
}

static void G721_ResetAll(void)
{
    G721_Init(&gG721EncoderLeft);
    G721_Init(&gG721EncoderRight);
    G721_Init(&gG721DecoderLeft);
    G721_Init(&gG721DecoderRight);
}

static const long g723Dqln24[8] =
{-2048,135,273,373,373,273,135,-2048};
static const long g723Wi24[8] =
{-128,960,4384,18624,18624,4384,960,-128};
static const long g723Fi24[8] =
{0,512,1024,3584,3584,1024,512,0};
static const long g723Qtab24[3] = {8,218,331};

static const long g723Dqln40[32] =
{-2048,-66,28,104,169,224,274,318,358,395,429,459,488,514,539,566,
 566,539,514,488,459,429,395,358,318,274,224,169,104,28,-66,-2048};
static const long g723Wi40[32] =
{448,448,768,1248,1280,1312,1856,3200,4512,5728,7008,8960,11456,
 14080,16928,22272,22272,16928,14080,11456,8960,7008,5728,4512,
 3200,1856,1312,1280,1248,768,448,448};
static const long g723Fi40[32] =
{0,0,0,0,0,512,512,512,512,512,1024,1536,2048,2560,3072,3072,
 3072,3072,2560,2048,1536,1024,512,512,512,512,512,0,0,0,0,0};
static const long g723Qtab40[15] =
{-122,-16,68,139,198,250,298,339,378,413,445,475,502,528,553};

static long G723Quantize(long difference, long step,
                         const long *table, int size)
{
    long magnitude, exponent, mantissa, normalizedLog, code;

    magnitude = G721Abs(difference);
    exponent = G721Quan(magnitude >> 1, g721Power2, 15);
    mantissa = ((magnitude << 7) >> exponent) & 127L;
    normalizedLog = (exponent << 7) + mantissa - (step >> 2);
    code = G721Quan(normalizedLog, table, size);
    if (difference < 0L)
    {
        return ((long)(size << 1) + 1L - code);
    }
    if (code == 0L)
    {
        return (long)(size << 1) + 1L;
    }
    return code;
}

static unsigned int G723_Encode(int sample, G721State *state, Uint16 bits)
{
    const long *dqln;
    const long *wi;
    const long *fi;
    const long *qtable;
    int qsize;
    long signMask;
    long zeroEstimate, poleZeroEstimate, estimate, difference;
    long step, code, reconstructedDifference, reconstructedSignal;

    if (bits == 3U)
    {
        dqln = g723Dqln24;
        wi = g723Wi24;
        fi = g723Fi24;
        qtable = g723Qtab24;
        qsize = 3;
        signMask = 4L;
    }
    else
    {
        dqln = g723Dqln40;
        wi = g723Wi40;
        fi = g723Fi40;
        qtable = g723Qtab40;
        qsize = 15;
        signMask = 16L;
    }

    zeroEstimate = G721PredictZero(state);
    poleZeroEstimate = zeroEstimate >> 1;
    estimate = (zeroEstimate + G721PredictPole(state)) >> 1;
    difference = ((long)sample >> 2) - estimate;
    step = G721StepSize(state);
    code = G723Quantize(difference, step, qtable, qsize);
    reconstructedDifference =
        G721Reconstruct(code & signMask, dqln[code], step);
    reconstructedSignal = (reconstructedDifference < 0L) ?
        estimate - (reconstructedDifference &
                    ((bits == 5U) ? 0x7FFFL : 0x3FFFL)) :
        estimate + reconstructedDifference;
    G72xUpdate(bits, step, wi[code], fi[code],
               reconstructedDifference, reconstructedSignal,
               reconstructedSignal + poleZeroEstimate - estimate, state);
    return (unsigned int)(code & ((1U << bits) - 1U));
}

static int G723_Decode(unsigned int input, G721State *state, Uint16 bits)
{
    const long *dqln;
    const long *wi;
    const long *fi;
    long signMask;
    long code, zeroEstimate, poleZeroEstimate, estimate, step;
    long reconstructedDifference, reconstructedSignal, output;

    if (bits == 3U)
    {
        dqln = g723Dqln24;
        wi = g723Wi24;
        fi = g723Fi24;
        signMask = 4L;
        code = input & 7U;
    }
    else
    {
        dqln = g723Dqln40;
        wi = g723Wi40;
        fi = g723Fi40;
        signMask = 16L;
        code = input & 31U;
    }

    zeroEstimate = G721PredictZero(state);
    poleZeroEstimate = zeroEstimate >> 1;
    estimate = (zeroEstimate + G721PredictPole(state)) >> 1;
    step = G721StepSize(state);
    reconstructedDifference =
        G721Reconstruct(code & signMask, dqln[code], step);
    reconstructedSignal = (reconstructedDifference < 0L) ?
        estimate - (reconstructedDifference &
                    ((bits == 5U) ? 0x7FFFL : 0x3FFFL)) :
        estimate + reconstructedDifference;
    G72xUpdate(bits, step, wi[code], fi[code],
               reconstructedDifference, reconstructedSignal,
               reconstructedSignal - estimate + poleZeroEstimate, state);
    output = reconstructedSignal << 2;
    if (output > 32767L) output = 32767L;
    else if (output < -32768L) output = -32768L;
    return (int)output;
}

static void G723_ResetAll(void)
{
    G721_Init(&gG723EncoderLeft);
    G721_Init(&gG723EncoderRight);
    G721_Init(&gG723DecoderLeft);
    G721_Init(&gG723DecoderRight);
}

static const int g722QmfF[12] =
{3,-11,12,32,-210,951,3876,-805,362,-156,53,-11};
static const int g722QmfR[12] =
{-11,53,-156,362,-805,3876,951,-210,32,12,-11,3};
static const int g722Qm2[4] = {-7408,-1616,7408,1616};
static const int g722Qm4[16] =
{0,-20456,-12896,-8968,-6288,-4240,-2584,-1200,
 20456,12896,8968,6288,4240,2584,1200,0};
static const int g722Qm6[64] =
{-136,-136,-136,-136,-24808,-21904,-19008,-16704,
 -14984,-13512,-12280,-11192,-10232,-9360,-8576,-7856,
 -7192,-6576,-6000,-5456,-4944,-4464,-4008,-3576,
 -3168,-2776,-2400,-2032,-1688,-1360,-1040,-728,
 24808,21904,19008,16704,14984,13512,12280,11192,
 10232,9360,8576,7856,7192,6576,6000,5456,
 4944,4464,4008,3576,3168,2776,2400,2032,
 1688,1360,1040,728,432,136,-432,-136};
static const int g722Q6[32] =
{0,35,72,110,150,190,233,276,323,370,422,473,530,587,650,714,
 786,858,940,1023,1121,1219,1339,1458,1612,1765,1980,2195,
 2557,2919,0,0};
static const int g722Ilb[32] =
{2048,2093,2139,2186,2233,2282,2332,2383,2435,2489,2543,2599,
 2656,2714,2774,2834,2896,2960,3025,3091,3158,3228,3298,3371,
 3444,3520,3597,3676,3756,3838,3922,4008};
static const int g722Iln[32] =
{0,63,62,31,30,29,28,27,26,25,24,23,22,21,20,19,
 18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,0};
static const int g722Ilp[32] =
{0,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,
 46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,0};
static const int g722Ihn[3] = {0,1,0};
static const int g722Ihp[3] = {0,3,2};
static const int g722Wl[8] = {-60,-30,58,172,334,538,1198,3042};
static const int g722Rl42[16] =
{0,7,6,5,4,3,2,1,7,6,5,4,3,2,1,0};
static const int g722Wh[3] = {0,-214,798};
static const int g722Rh2[4] = {2,1,2,1};

static int G722Abs(int x)
{
    return (x < 0) ? -x : x;
}

static int G722Sat16(long x)
{
    if (x > 32767L) return 32767;
    if (x < -32768L) return -32768;
    return (int)x;
}

static int G722Sat15(long x)
{
    if (x > 16383L) return 16383;
    if (x < -16384L) return -16384;
    return (int)x;
}

static int G722Add(int a, int b)
{
    return G722Sat16((long)a + b);
}

static int G722Sub(int a, int b)
{
    return G722Sat16((long)a - b);
}

static long G722Dot(const int *values, const int *coefficients, int pointer)
{
    int i;
    long sum;

    sum = 0L;
    for (i = 0; i < 12; i++)
    {
        sum += (long)values[pointer] * coefficients[i];
        pointer++;
        if (pointer >= 12) pointer = 0;
    }
    return sum;
}

static void G722Block4(G722Band *state, int dx)
{
    int wd1, wd2, wd3, sp, r, p, ap0, ap1, i;
    long wd32, sz;

    r = G722Add(state->s, dx);
    p = G722Add(state->sz, dx);
    wd1 = G722Sat16((long)state->a[0] << 2);
    wd32 = ((p ^ state->p[0]) & 0x8000) ? wd1 : -wd1;
    if (wd32 > 32767L) wd32 = 32767L;
    wd3 = (int)((((p ^ state->p[1]) & 0x8000) ? -128L : 128L) +
                (wd32 >> 7) +
                (((long)state->a[1] * 32512L) >> 15));
    if (G722Abs(wd3) > 12288) wd3 = (wd3 < 0) ? -12288 : 12288;
    ap1 = wd3;

    wd1 = ((p ^ state->p[0]) & 0x8000) ? -192 : 192;
    wd2 = (int)(((long)state->a[0] * 32640L) >> 15);
    ap0 = G722Add(wd1, wd2);
    wd3 = G722Sub(15360, ap1);
    if (G722Abs(ap0) > wd3) ap0 = (ap0 < 0) ? -wd3 : wd3;

    wd1 = G722Add(r, r);
    wd1 = (int)(((long)ap0 * wd1) >> 15);
    wd2 = G722Add(state->r, state->r);
    wd2 = (int)(((long)ap1 * wd2) >> 15);
    sp = G722Add(wd1, wd2);
    state->r = r;
    state->a[1] = ap1;
    state->a[0] = ap0;
    state->p[1] = state->p[0];
    state->p[0] = p;

    wd1 = (dx == 0) ? 0 : 128;
    state->d[0] = dx;
    sz = 0L;
    for (i = 5; i >= 0; i--)
    {
        wd2 = ((state->d[i + 1] ^ dx) & 0x8000) ? -wd1 : wd1;
        wd3 = (int)(((long)state->b[i] * 32640L) >> 15);
        state->b[i] = G722Add(wd2, wd3);
        wd3 = G722Add(state->d[i], state->d[i]);
        sz += ((long)state->b[i] * wd3) >> 15;
        state->d[i + 1] = state->d[i];
    }
    state->sz = G722Sat16(sz);
    state->s = G722Add(sp, state->sz);
}

static void G722UpdateLow(G722Band *band, int index, int difference)
{
    int wd1, wd2, wd3;

    wd2 = g722Rl42[index];
    wd1 = (int)(((long)band->nb * 127L) >> 7) + g722Wl[wd2];
    if (wd1 < 0) wd1 = 0;
    else if (wd1 > 18432) wd1 = 18432;
    band->nb = wd1;
    wd1 = (band->nb >> 6) & 31;
    wd2 = 8 - (band->nb >> 11);
    wd3 = (wd2 < 0) ? (g722Ilb[wd1] << -wd2) :
                      (g722Ilb[wd1] >> wd2);
    band->det = wd3 << 2;
    G722Block4(band, difference);
}

static void G722UpdateHigh(G722Band *band, int index, int difference)
{
    int wd1, wd2, wd3;

    wd2 = g722Rh2[index];
    wd1 = (int)(((long)band->nb * 127L) >> 7) + g722Wh[wd2];
    if (wd1 < 0) wd1 = 0;
    else if (wd1 > 22528) wd1 = 22528;
    band->nb = wd1;
    wd1 = (band->nb >> 6) & 31;
    wd2 = 10 - (band->nb >> 11);
    wd3 = (wd2 < 0) ? (g722Ilb[wd1] << -wd2) :
                      (g722Ilb[wd1] >> wd2);
    band->det = wd3 << 2;
    G722Block4(band, difference);
}

static void G722_Init(G722State *state)
{
    int i, band;

    state->ptr = 0;
    for (i = 0; i < 12; i++)
    {
        state->x[i] = 0;
        state->y[i] = 0;
    }
    for (band = 0; band < 2; band++)
    {
        state->band[band].nb = 0;
        state->band[band].s = 0;
        state->band[band].sz = 0;
        state->band[band].r = 0;
        for (i = 0; i < 2; i++)
        {
            state->band[band].p[i] = 0;
            state->band[band].a[i] = 0;
        }
        for (i = 0; i < 6; i++) state->band[band].b[i] = 0;
        for (i = 0; i < 7; i++) state->band[band].d[i] = 0;
    }
    state->band[0].det = 32;
    state->band[1].det = 8;
}

static unsigned int G722_EncodePair(int sample0, int sample1,
                                    G722State *state)
{
    long oddSum, evenSum;
    int low, high, error, magnitude, threshold, i;
    int lowCode, lowIndex, lowDifference;
    int highMagnitudeIndex, highCode, highDifference;

    state->x[state->ptr] = sample0;
    state->y[state->ptr] = sample1;
    state->ptr++;
    if (state->ptr >= 12) state->ptr = 0;
    oddSum = G722Dot(state->x, g722QmfF, state->ptr);
    evenSum = G722Dot(state->y, g722QmfR, state->ptr);
    low = (int)((evenSum + oddSum) >> 14);
    high = (int)((evenSum - oddSum) >> 14);

    error = G722Sub(low, state->band[0].s);
    magnitude = (error >= 0) ? error : ~error;
    for (i = 1; i < 30; i++)
    {
        threshold = (int)(((long)g722Q6[i] *
                           state->band[0].det) >> 12);
        if (magnitude < threshold) break;
    }
    lowCode = (error < 0) ? g722Iln[i] : g722Ilp[i];
    lowIndex = lowCode >> 2;
    lowDifference = (int)(((long)state->band[0].det *
                           g722Qm4[lowIndex]) >> 15);
    G722UpdateLow(&state->band[0], lowIndex, lowDifference);

    error = G722Sub(high, state->band[1].s);
    magnitude = (error >= 0) ? error : ~error;
    threshold = (int)((564L * state->band[1].det) >> 12);
    highMagnitudeIndex = (magnitude >= threshold) ? 2 : 1;
    highCode = (error < 0) ? g722Ihn[highMagnitudeIndex] :
                             g722Ihp[highMagnitudeIndex];
    highDifference = (int)(((long)state->band[1].det *
                            g722Qm2[highCode]) >> 15);
    G722UpdateHigh(&state->band[1], highCode, highDifference);
    return (unsigned int)(((highCode << 6) | lowCode) & 0xFF);
}

static void G722_DecodeByte(unsigned int input, int *sample0, int *sample1,
                            G722State *state)
{
    int code, lowCode, highCode, lowIndex;
    int lowDifference, highDifference, low, high;
    long output0, output1;

    code = input & 0xFFU;
    lowCode = code & 0x3F;
    highCode = (code >> 6) & 3;
    low = G722Sat15(state->band[0].s +
          (((long)state->band[0].det * g722Qm6[lowCode]) >> 15));
    lowIndex = lowCode >> 2;
    lowDifference = (int)(((long)state->band[0].det *
                           g722Qm4[lowIndex]) >> 15);
    G722UpdateLow(&state->band[0], lowIndex, lowDifference);

    highDifference = (int)(((long)state->band[1].det *
                            g722Qm2[highCode]) >> 15);
    high = G722Sat15(state->band[1].s + highDifference);
    G722UpdateHigh(&state->band[1], highCode, highDifference);

    state->x[state->ptr] = low + high;
    state->y[state->ptr] = low - high;
    state->ptr++;
    if (state->ptr >= 12) state->ptr = 0;
    output0 = G722Dot(state->y, g722QmfR, state->ptr) >> 11;
    output1 = G722Dot(state->x, g722QmfF, state->ptr) >> 11;
    *sample0 = G722Sat16(output0);
    *sample1 = G722Sat16(output1);
}

static void G722_ResetAll(void)
{
    G722_Init(&gG722EncoderLeft);
    G722_Init(&gG722EncoderRight);
    G722_Init(&gG722DecoderLeft);
    G722_Init(&gG722DecoderRight);
    gG722InputLeft[0] = gG722InputLeft[1] = 0;
    gG722InputRight[0] = gG722InputRight[1] = 0;
    gG722QueuedLeft = 0;
    gG722QueuedRight = 0;
    gG722Phase = 0U;
}

static void SetCodecSampleRate(Uint16 mode)
{
    (void)AIC23Write(0x12, 0x00);
    Delay(10);
    if (mode == CODEC_MODE_G722)
    {
        (void)AIC23Write(0x10, 0x4D);
    }
    else
    {
        (void)AIC23Write(0x10, 0x23);
    }
    Delay(10);
    (void)AIC23Write(0x12, 0x01);
    Delay(10);
}

/* End of file. */
