#ifndef RNG_SUPPORT_H
#define RNG_SUPPORT_H
/*---------------------------------------------------------------------------*/
#include "deca_regs.h"
#include "core/net/linkaddr.h"
/*---------------------------------------------------------------------------*/
#define RX_WAIT_FLAGS (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)
#define NO_RX_TIMEOUT 0
#define RNG_TS_LEN 8
#define DWT_TX_BITMASK (0xFFFFFFFE00UL)
#define DWT_VALUES 1099511627776
#define ANTENNA_DELAY 16455
#define SPEED_OF_LIGHT 299702547
#define BUF_LEN 30
#define CRC_LEN 2

/* Quality thresholds used by rng-init-all.c */
#define FP_RX_DIFF_MIN   (-40.0f)
#define MIN_PREAM_COUNT   80
/*---------------------------------------------------------------------------*/

typedef struct {
  uint8_t fctrl[2];
  uint8_t seqn;
  uint8_t pan_id[2];
  uint8_t dst[2];
  uint8_t src[2];
} __attribute__ ((__packed__)) ieee154_hdr_t;

typedef struct {
  ieee154_hdr_t hdr;
} __attribute__ ((__packed__)) sstwr_init_msg_t;

typedef struct {
  ieee154_hdr_t hdr;
  uint8_t  init_rx_ts[RNG_TS_LEN];
  uint8_t  resp_tx_ts[RNG_TS_LEN];
  uint16_t fp_amp1;
  uint16_t fp_amp2;
  uint16_t fp_amp3;
  uint16_t rx_pream;
  uint16_t cir_max;
  uint16_t std_noise;
} __attribute__ ((__packed__)) sstwr_resp_msg_t;

uint64_t get_rx_timestamp(void);
uint64_t get_tx_timestamp(void);
uint64_t predict_tx_timestamp(uint64_t tx_ts);
void resp_msg_set_timestamp(uint8_t *ts_field, uint64_t ts);
void resp_msg_get_timestamp(uint8_t *ts_field, uint64_t *ts);

void fill_ieee_hdr(ieee154_hdr_t *hdr, linkaddr_t *src, linkaddr_t *dst, uint8_t seqn);
void radio_init();
uint8_t start_tx(void *data, uint8_t len, uint8_t mode, uint16_t rx_to, uint64_t tx_time);
void start_rx(uint16_t rx_to);
uint8_t wait_rx();
uint8_t read_rx_data(void* pkt, uint32_t pkt_size);
void radio_reset();
/*---------------------------------------------------------------------------*/
#endif /* RNG_SUPPORT_H */
