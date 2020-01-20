/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include "userrevfx.h"
#include "buffer_ops.h"
#include "LCWCommon.h"

#define LCW_DELAY_BUFFER_DEC(p) ( ((p)->pointer - 1) & (p)->mask )
#define LCW_DELAY_BUFFER_LUT(p, i) ( (p)->buffer[((p)->pointer + (i)) & (p)->mask] )

typedef struct {
    int32_t *buffer;
    uint32_t size;
    uint32_t mask;
    int32_t pointer;
    int32_t gain;
} LCWDelayBuffer;

#define LCW_REVERB_GAIN_TABLE_SIZE (64 + 1)

#define AP1_DELAY (337) // = 48000 * 0.007
#define AP2_DELAY (241) // = 48000 * 0.005
#define AP3_DELAY (109) // = 48000 * 0.0023
#define AP4_DELAY (53)  // = 48000 * 0.0017

#define LCW_REVERB_COMB_SIZE (1<<14)
#define LCW_REVERB_COMB_MAX (6)
#define LCW_REVERB_COMB_BUFFER_TOTAL (LCW_REVERB_COMB_SIZE * LCW_REVERB_COMB_MAX)

#define LCW_REVERB_AP_SIZE (1<<12)
#define LCW_REVERB_AP_MAX (6)
#define LCW_REVERB_AP_BUFFER_TOTAL (LCW_REVERB_AP_SIZE * LCW_REVERB_AP_MAX)

static __sdram int32_t s_reverb_ram_comb_buffer[LCW_REVERB_COMB_BUFFER_TOTAL];
static __sdram int32_t s_reverb_ram_ap_buffer[LCW_REVERB_AP_BUFFER_TOTAL];

static LCWDelayBuffer revCombBuffers[LCW_REVERB_COMB_MAX];
static LCWDelayBuffer revApBuffers[LCW_REVERB_AP_MAX];

static float s_inputGain;
static float s_mix;
static float s_depth;
static float s_time;

// q12
static const uint16_t gainTable[LCW_REVERB_GAIN_TABLE_SIZE][LCW_REVERB_COMB_MAX] = {
  { 0x000, 0x000, 0x000, 0x000 }, // [ 0]
// 1361, 2099, 2341, 4253, 4816, 5382
// 3119, 3691, 3967
  { 0xAFF, 0x8F9, 0x864, 0x4F4, 0x43E, 0x3A1 }, // [ 1] 0.522
  { 0xB2C, 0x932, 0x8A0, 0x535, 0x47D, 0x3DE }, // [ 2] 0.545
  { 0xB58, 0x96A, 0x8DB, 0x576, 0x4BD, 0x41B }, // [ 3] 0.569
  { 0xB82, 0x9A0, 0x914, 0x5B7, 0x4FD, 0x459 }, // [ 4] 0.595
  { 0xBAC, 0x9D6, 0x94D, 0x5F8, 0x53E, 0x499 }, // [ 5] 0.621
  { 0xBD4, 0xA0B, 0x984, 0x63A, 0x57F, 0x4D8 }, // [ 6] 0.648
  { 0xBFB, 0xA3E, 0x9BA, 0x67B, 0x5C0, 0x519 }, // [ 7] 0.677
  { 0xC21, 0xA70, 0x9F0, 0x6BC, 0x601, 0x55A }, // [ 8] 0.707
  { 0xC46, 0xAA1, 0xA23, 0x6FC, 0x642, 0x59B }, // [ 9] 0.738
  { 0xC69, 0xAD0, 0xA56, 0x73C, 0x683, 0x5DC }, // [10] 0.771
  { 0xC8C, 0xAFF, 0xA88, 0x77B, 0x6C4, 0x61D }, // [11] 0.805
  { 0xCAD, 0xB2C, 0xAB8, 0x7BA, 0x704, 0x65F }, // [12] 0.841
  { 0xCCD, 0xB58, 0xAE7, 0x7F8, 0x744, 0x6A0 }, // [13] 0.878
  { 0xCEC, 0xB82, 0xB15, 0x835, 0x784, 0x6E0 }, // [14] 0.917
  { 0xD0A, 0xBAC, 0xB41, 0x872, 0x7C2, 0x720 }, // [15] 0.958
  { 0xD27, 0xBD4, 0xB6C, 0x8AD, 0x800, 0x760 }, // [16] 1.000
  { 0xD43, 0xBFB, 0xB97, 0x8E7, 0x83D, 0x79F }, // [17] 1.044
  { 0xD5F, 0xC21, 0xBBF, 0x921, 0x879, 0x7DD }, // [18] 1.091
  { 0xD79, 0xC46, 0xBE7, 0x959, 0x8B5, 0x81B }, // [19] 1.139
  { 0xD92, 0xC69, 0xC0D, 0x990, 0x8EF, 0x857 }, // [20] 1.189
  { 0xDAA, 0xC8C, 0xC33, 0x9C6, 0x928, 0x893 }, // [21] 1.242
  { 0xDC2, 0xCAD, 0xC57, 0x9FB, 0x960, 0x8CE }, // [22] 1.297
  { 0xDD8, 0xCCD, 0xC7A, 0xA2F, 0x997, 0x908 }, // [23] 1.354
  { 0xDEE, 0xCEC, 0xC9C, 0xA61, 0x9CD, 0x941 }, // [24] 1.414
  { 0xE03, 0xD0A, 0xCBD, 0xA92, 0xA02, 0x978 }, // [25] 1.477
  { 0xE17, 0xD27, 0xCDC, 0xAC2, 0xA35, 0x9AF }, // [26] 1.542
  { 0xE2B, 0xD43, 0xCFB, 0xAF1, 0xA68, 0x9E4 }, // [27] 1.610
  { 0xE3E, 0xD5F, 0xD18, 0xB1E, 0xA99, 0xA18 }, // [28] 1.682
  { 0xE50, 0xD79, 0xD35, 0xB4B, 0xAC8, 0xA4B }, // [29] 1.756
  { 0xE61, 0xD92, 0xD51, 0xB76, 0xAF7, 0xA7D }, // [30] 1.834
  { 0xE72, 0xDAA, 0xD6B, 0xBA0, 0xB24, 0xAAE }, // [31] 1.915
  { 0xE82, 0xDC2, 0xD85, 0xBC8, 0xB50, 0xADD }, // [32] 2.000
  { 0xE91, 0xDD8, 0xD9E, 0xBF0, 0xB7B, 0xB0B }, // [33] 2.089
  { 0xEA0, 0xDEE, 0xDB6, 0xC16, 0xBA5, 0xB38 }, // [34] 2.181
  { 0xEAE, 0xE03, 0xDCD, 0xC3B, 0xBCD, 0xB63 }, // [35] 2.278
  { 0xEBC, 0xE17, 0xDE3, 0xC5F, 0xBF5, 0xB8E }, // [36] 2.378
  { 0xEC9, 0xE2B, 0xDF8, 0xC81, 0xC1B, 0xBB7 }, // [37] 2.484
  { 0xED6, 0xE3E, 0xE0D, 0xCA3, 0xC40, 0xBDF }, // [38] 2.594
  { 0xEE2, 0xE50, 0xE21, 0xCC4, 0xC63, 0xC05 }, // [39] 2.709
  { 0xEEE, 0xE61, 0xE34, 0xCE3, 0xC86, 0xC2B }, // [40] 2.828
  { 0xEF9, 0xE72, 0xE46, 0xD01, 0xCA7, 0xC4F }, // [41] 2.954
  { 0xF04, 0xE82, 0xE58, 0xD1F, 0xCC8, 0xC72 }, // [42] 3.084
  { 0xF0E, 0xE91, 0xE69, 0xD3B, 0xCE7, 0xC95 }, // [43] 3.221
  { 0xF18, 0xEA0, 0xE7A, 0xD57, 0xD05, 0xCB6 }, // [44] 3.364
  { 0xF22, 0xEAE, 0xE89, 0xD71, 0xD23, 0xCD5 }, // [45] 3.513
  { 0xF2B, 0xEBC, 0xE99, 0xD8B, 0xD3F, 0xCF4 }, // [46] 3.668
  { 0xF34, 0xEC9, 0xEA7, 0xDA3, 0xD5A, 0xD12 }, // [47] 3.830
  { 0xF3C, 0xED6, 0xEB5, 0xDBB, 0xD74, 0xD2F }, // [48] 4.000
  { 0xF44, 0xEE2, 0xEC3, 0xDD2, 0xD8E, 0xD4B }, // [49] 4.177
  { 0xF4C, 0xEEE, 0xED0, 0xDE8, 0xDA6, 0xD66 }, // [50] 4.362
  { 0xF54, 0xEF9, 0xEDC, 0xDFD, 0xDBE, 0xD80 }, // [51] 4.555
  { 0xF5B, 0xF04, 0xEE8, 0xE11, 0xDD5, 0xD99 }, // [52] 4.757
  { 0xF62, 0xF0E, 0xEF3, 0xE25, 0xDEB, 0xDB1 }, // [53] 4.967
  { 0xF68, 0xF18, 0xEFE, 0xE38, 0xE00, 0xDC8 }, // [54] 5.187
  { 0xF6F, 0xF22, 0xF09, 0xE4A, 0xE14, 0xDDE }, // [55] 5.417
  { 0xF75, 0xF2B, 0xF13, 0xE5C, 0xE28, 0xDF4 }, // [56] 5.657
  { 0xF7A, 0xF34, 0xF1D, 0xE6D, 0xE3B, 0xE09 }, // [57] 5.907
  { 0xF80, 0xF3C, 0xF26, 0xE7D, 0xE4D, 0xE1D }, // [58] 6.169
  { 0xF85, 0xF44, 0xF2F, 0xE8D, 0xE5E, 0xE30 }, // [59] 6.442
  { 0xF8A, 0xF4C, 0xF38, 0xE9C, 0xE6F, 0xE43 }, // [60] 6.727
  { 0xF8F, 0xF54, 0xF40, 0xEAA, 0xE7F, 0xE54 }, // [61] 7.025
  { 0xF94, 0xF5B, 0xF48, 0xEB8, 0xE8F, 0xE66 }, // [62] 7.336
  { 0xF99, 0xF62, 0xF50, 0xEC5, 0xE9E, 0xE76 }, // [63] 7.661
  { 0xF9D, 0xF68, 0xF57, 0xED2, 0xEAC, 0xE86 }  // [64] 8.000
};

__fast_inline float softlimiter(float c, float x)
{
  float xf = si_fabsf(x);
  if ( xf < c ) {
    return x;
  }
  else {
    return si_copysignf( c + fx_softclipf(c, xf - c), x );
  }
}

void REVFX_INIT(uint32_t platform, uint32_t api)
{
  for (int32_t i=0; i<LCW_REVERB_COMB_MAX; i++) {
    LCWDelayBuffer *buf = &(revCombBuffers[i]);
    buf->buffer = &(s_reverb_ram_comb_buffer[LCW_REVERB_COMB_SIZE * i]);
    buf->size = LCW_REVERB_COMB_SIZE;
    buf->mask = LCW_REVERB_COMB_SIZE - 1;
    buf->pointer = 0;
    buf->gain = LCW_SQ15_16( 0.7 ) >> 4;
  }

  for (int32_t i=0; i<LCW_REVERB_AP_MAX; i++) {
    LCWDelayBuffer *buf = &(revApBuffers[i]);
    buf->buffer = &(s_reverb_ram_ap_buffer[LCW_REVERB_AP_SIZE * i]);
    buf->size = LCW_REVERB_AP_SIZE;
    buf->mask = LCW_REVERB_AP_SIZE - 1;
    buf->pointer = 0;
    buf->gain = LCW_SQ15_16( 0.7 ) >> 4;
  }

  s_mix = 0.5f;
  s_depth = 0.f;
  s_time = 0.f;
  s_inputGain = 0.f;
}

void REVFX_PROCESS(float *xn, uint32_t frames)
{
  float * __restrict x = xn;
  const float * x_e = x + 2*frames;

  const float dry = 1.f - s_mix;
  const float wet = s_mix;
  const int32_t depth = (int32_t)(s_depth * 0x400);

  int32_t time = (int32_t)((LCW_REVERB_GAIN_TABLE_SIZE - 1) * s_time);

  for (int32_t i=0; i<3; i++) {
    revCombBuffers[i].gain = gainTable[1][i];
  }
  for (int32_t i=3; i<LCW_REVERB_COMB_MAX; i++) {
    revCombBuffers[i].gain = gainTable[time][i];
  }

  LCWDelayBuffer *comb = &(revCombBuffers[0]);
  LCWDelayBuffer *ap = &revApBuffers[0];

  const int32_t preDelay[] = { 1361, 2099, 2341, 4457, 6880, 7689 };
  const int32_t apDelay[] = { AP1_DELAY, AP2_DELAY, AP3_DELAY, AP4_DELAY };
  for (; x != x_e; ) {
    float xL = *x;
    // float xR = *(x + 1);
    int32_t inL = (int32_t)( s_inputGain * xL * (1 << 20) );

    /* in -> comb1[] */
    int64_t combSum1 = 0;
    int64_t combSum[3] = { 0, 0, 0 };
    const int32_t posi[3] = { 3119, 3691, 3967 };
    for (int32_t j=3; j<LCW_REVERB_COMB_MAX; j++) {
      LCWDelayBuffer *p = comb + j;
      int32_t z = LCW_DELAY_BUFFER_LUT(p, preDelay[j]);
      combSum1 += z;

      for (int32_t k=0; k<3; k++) {
        combSum[k] += LCW_DELAY_BUFFER_LUT(p, posi[k]);
      }
      
      p->pointer = LCW_DELAY_BUFFER_DEC(p);
      p->buffer[p->pointer] =
        (int32_t)(inL + (((int64_t)z * p->gain) >> 12));
    }

    /* comb2[] -> comb1[] */
    int64_t combSum2 = 0;
    for (int32_t j=0; j<3; j++) {
      int64_t in = (combSum1 + combSum[j]) >> 2;
      LCWDelayBuffer *p = comb + j;
      int32_t z = LCW_DELAY_BUFFER_LUT(p, preDelay[j]);
      combSum2 += z;
      p->pointer = LCW_DELAY_BUFFER_DEC(p);
      p->buffer[p->pointer] =
        (int32_t)(in + (((int64_t)z * p->gain) >> 12));
    }

    /* comb2[] -> AP */
    int64_t out = (combSum2 * depth) >> 12;
    for (int32_t j=0; j<4; j++) {
      LCWDelayBuffer *p = ap + j;

      int32_t z = LCW_DELAY_BUFFER_LUT(p, apDelay[j]);
      int64_t in = out + (((int64_t)z * p->gain) >> 12);
      out = (int64_t)z - ((in * p->gain) >> 12);

      p->pointer = LCW_DELAY_BUFFER_DEC(p);
      p->buffer[p->pointer] = in;
    }
 
    float outL = out / (float)(1 << 20);
    float yL = softlimiter( 0.1f, (dry * xL) + (wet * outL) );

    *(x++) = yL;
    *(x++) = yL;

    if ( s_inputGain < 0.99998f ) {
      s_inputGain += ( (1.f - s_inputGain) * 0.0625f );
    }
    else { s_inputGain = 1.f; }
  }
}

void REVFX_RESUME(void)
{
  buf_clr_u32(
    (uint32_t * __restrict__)s_reverb_ram_comb_buffer,
    LCW_REVERB_COMB_BUFFER_TOTAL );
  buf_clr_u32(
    (uint32_t * __restrict__)s_reverb_ram_ap_buffer,
    LCW_REVERB_AP_BUFFER_TOTAL );
  s_inputGain = 0.f;
}

void REVFX_PARAM(uint8_t index, int32_t value)
{
  const float valf = q31_to_f32(value);
  switch (index) {
  case k_user_revfx_param_time:
    s_time = clip01f(valf);
    break;
  case k_user_revfx_param_depth:
    s_depth = clip01f(valf);
    break;
  case k_user_revfx_param_shift_depth:
    // Rescale to add notch around 0.5f
    s_mix = (valf <= 0.49f) ? 1.02040816326530612244f * valf : (valf >= 0.51f) ? 0.5f + 1.02f * (valf-0.51f) : 0.5f;
    break;
  default:
    break;
  }
}
