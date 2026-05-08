/*
 * anchor_table.h - GENERATO AUTOMATICAMENTE da generate_and_build_all.py
 * Ancore richieste: 108 e 113..154 (senza 120)
 */
#ifndef ANCHOR_TABLE_H
#define ANCHOR_TABLE_H

#define NUM_ANCHORS 42
#define ANCHOR_STEP_UUS 2000
#define TOTAL_WINDOW 87000
#define PER_ANCHOR_TIMEOUT 1800

typedef struct {
  uint8_t addr[2];
  uint8_t slot;
} anchor_entry_t;

static const anchor_entry_t anchor_table[NUM_ANCHORS] = {
  {{0x5A, 0x33},  0},  /* Node 108 | std=0.054m | (132.88,  29.14) m */
  {{0x85, 0x30},  1},  /* Node 113 | std=0.035m | (140.33,  29.23) m */
  {{0x42, 0xB0},  2},  /* Node 114 | std=0.046m | (145.66,  27.62) m */
  {{0x42, 0x9C},  3},  /* Node 115 | std=0.035m | (145.73,  29.25) m */
  {{0x42, 0xD5},  4},  /* Node 116 | std=0.051m | (157.38,  27.62) m */
  {{0x42, 0xB5},  5},  /* Node 117 | std=0.029m | (157.46,  29.24) m */
  {{0x5D, 0x36},  6},  /* Node 118 | std=0.095m | (171.15,  27.62) m */
  {{0x5B, 0xBA},  7},  /* Node 119 | std=0.048m | (171.22,  29.25) m */
  {{0x42, 0x3A},  8},  /* Node 121 | std=0.078m | (178.42,  29.25) m */
  {{0x5D, 0x14},  9},  /* Node 122 | std=0.048m | (183.83,  29.25) m */
  {{0x85, 0x23}, 10},  /* Node 123 | std=0.038m | (185.79,  29.01) m */
  {{0x84, 0xB6}, 11},  /* Node 124 | std=0.041m | (185.79,  26.61) m */
  {{0x5A, 0x1B}, 12},  /* Node 125 | std=0.044m | (188.85,  21.31) m */
  {{0x5B, 0x2A}, 13},  /* Node 126 | std=0.041m | (184.22,  21.26) m */
  {{0x59, 0x0A}, 14},  /* Node 127 | std=0.047m | (188.86,  17.69) m */
  {{0x41, 0xF1}, 15},  /* Node 128 | std=0.047m | (184.21,  17.68) m */
  {{0x17, 0x0B}, 16},  /* Node 129 | std=0.245m | (174.95,  17.86) m */
  {{0x57, 0x25}, 17},  /* Node 130 | std=0.192m | (174.97,  15.46) m */
  {{0x5B, 0xAE}, 18},  /* Node 131 | std=0.047m | (184.22,  15.89) m */
  {{0x58, 0x2E}, 19},  /* Node 132 | std=0.038m | (188.86,  15.92) m */
  {{0x58, 0x1A}, 20},  /* Node 133 | std=0.033m | (188.85,   8.42) m */
  {{0x5C, 0x98}, 21},  /* Node 134 | std=0.041m | (186.20,   8.24) m */
  {{0x58, 0x26}, 22},  /* Node 135 | std=0.058m | (184.21,   8.40) m */
  {{0x5C, 0x8F}, 23},  /* Node 136 | std=0.060m | (185.80,   2.44) m */
  {{0x5C, 0x93}, 24},  /* Node 137 | std=0.051m | (185.56,   0.53) m */
  {{0x85, 0xB5}, 25},  /* Node 138 | std=0.040m | (183.76,   0.54) m */
  {{0x8E, 0x88}, 26},  /* Node 139 | std=0.060m | (177.15,   0.55) m */
  {{0x5A, 0x2F}, 27},  /* Node 140 | std=0.048m | (177.24,   2.16) m */
  {{0x5A, 0x23}, 28},  /* Node 141 | std=0.044m | (169.36,   0.55) m */
  {{0x1A, 0x00}, 29},  /* Node 142 | std=0.048m | (169.44,   2.17) m */
  {{0x11, 0x15}, 30},  /* Node 143 | std=0.054m | (156.44,   0.56) m */
  {{0x59, 0x8F}, 31},  /* Node 144 | std=0.042m | (156.53,   2.19) m */
  {{0x5D, 0x3B}, 32},  /* Node 145 | std=0.245m | (142.05,   0.55) m */
  {{0x5B, 0xA6}, 33},  /* Node 146 | std=0.120m | (142.13,   2.18) m */
  {{0x85, 0x1B}, 34},  /* Node 147 | std=0.067m | (134.86,   0.46) m */
  {{0x5B, 0x3B}, 35},  /* Node 148 | std=0.038m | (133.06,   0.45) m */
  {{0x57, 0xBA}, 36},  /* Node 149 | std=0.043m | (132.88,   2.50) m */
  {{0x5C, 0xA4}, 37},  /* Node 150 | std=0.043m | (132.89,   8.50) m */
  {{0x57, 0x9E}, 38},  /* Node 151 | std=0.147m | (134.51,   8.43) m */
  {{0x58, 0x1E}, 39},  /* Node 152 | std=0.439m | (132.89,  20.74) m */
  {{0x58, 0x9D}, 40},  /* Node 153 | std=0.081m | (134.51,  20.67) m */
  {{0x5D, 0x0C}, 41},  /* Node 154 | std=0.034m | (132.87,  26.73) m */
};

static inline uint32_t get_resp_delay_from_table(linkaddr_t addr)
{
  uint8_t i;
  for(i = 0; i < NUM_ANCHORS; i++) {
    if(anchor_table[i].addr[0] == addr.u8[0] &&
       anchor_table[i].addr[1] == addr.u8[1]) {
      return (uint32_t)(anchor_table[i].slot + 1) * ANCHOR_STEP_UUS;
    }
  }
  return ANCHOR_STEP_UUS;
}

#endif /* ANCHOR_TABLE_H */
