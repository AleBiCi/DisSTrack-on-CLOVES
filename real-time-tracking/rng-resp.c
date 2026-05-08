/*
 * rng-resp.c - Firmware responder per tracking realtime.
 *
 * Versione piu' vicina all'esempio multi-ranging:
 * il responder attende una richiesta unicast e risponde con un delay fisso.
 * In questo modo ogni transazione tag-ancora resta isolata e non ci sono
 * collisioni tra risposte di ancore diverse.
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
#include "net/netstack.h"
#include <stdio.h>
#include "dw1000.h"
#include "core/net/linkaddr.h"
#include "rng-support.h"

PROCESS(ranging_resp_process, "Ranging responder process");
AUTOSTART_PROCESSES(&ranging_resp_process);

#define UUS_TO_DWT_TIME (65536)
#define RESP_TX_DELAY_UUS 2500

PROCESS_THREAD(ranging_resp_process, ev, data)
{
  static sstwr_init_msg_t init_msg;
  static sstwr_resp_msg_t resp_msg;
  static uint8_t ret;

  PROCESS_BEGIN();

  printf("I am %02x:%02x\n",
    linkaddr_node_addr.u8[0],
    linkaddr_node_addr.u8[1]);

  radio_init();

  while(1) {
    dwt_rxdiag_t diag;
    linkaddr_t init_src, init_dst;
    uint64_t init_rx_ts, resp_tx_ts, predicted_tx_ts;

    start_rx(NO_RX_TIMEOUT);

    ret = wait_rx();
    if(!ret) {
      radio_reset();
      continue;
    }

    dwt_readdiagnostics(&diag);

    ret = read_rx_data(&init_msg, sizeof(init_msg));
    if(!ret) {
      radio_reset();
      continue;
    }

    init_src.u8[0] = init_msg.hdr.src[1];
    init_src.u8[1] = init_msg.hdr.src[0];
    init_dst.u8[0] = init_msg.hdr.dst[1];
    init_dst.u8[1] = init_msg.hdr.dst[0];

    if(!linkaddr_cmp(&init_dst, &linkaddr_node_addr)) {
      continue;
    }

    init_rx_ts = get_rx_timestamp();
    resp_tx_ts = (init_rx_ts + (uint64_t)RESP_TX_DELAY_UUS * UUS_TO_DWT_TIME) % DWT_VALUES;

    fill_ieee_hdr(&resp_msg.hdr, &linkaddr_node_addr, &init_src, init_msg.hdr.seqn);
    predicted_tx_ts = predict_tx_timestamp(resp_tx_ts);
    resp_msg_set_timestamp(&resp_msg.init_rx_ts[0], init_rx_ts);
    resp_msg_set_timestamp(&resp_msg.resp_tx_ts[0], predicted_tx_ts);

    resp_msg.fp_amp1   = diag.firstPathAmp1;
    resp_msg.fp_amp2   = diag.firstPathAmp2;
    resp_msg.fp_amp3   = diag.firstPathAmp3;
    resp_msg.rx_pream  = diag.rxPreamCount;
    resp_msg.cir_max   = diag.maxGrowthCIR;
    resp_msg.std_noise = diag.stdNoise;

    ret = start_tx(&resp_msg, sizeof(resp_msg), DWT_START_TX_DELAYED, 0, resp_tx_ts);
    if(!ret) {
      printf("[%u] RESP fail %02x:%02x\n",
        init_msg.hdr.seqn,
        resp_msg.hdr.dst[1], resp_msg.hdr.dst[0]);
    }
  }

  PROCESS_END();
}
