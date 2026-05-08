x/*
 * rng-init-all.c — Tag realtime con quality filtering.
 *
 * Invia INIT broadcast, raccoglie le risposte staggered delle ancore.
 * Per ogni risposta:
 *   1. Calcola la distanza SS-TWR
 *   2. Calcola FP power e RX power dalle metriche DW1000
 *   3. Se FP-RX > soglia E preamble count > soglia -> misura valida (LOS)
 *      Altrimenti -> scarta (NLOS / multipath / segnale debole)
 *   4. Raccoglie tutte le risposte del round e stampa una sola misura per ancora
 *
 * Formato output seriale:
 *   RANGING MEAS [round] [tag->ancora] distanza_mm QUAL fp_rx_diff_x10 pream_count FLAG LOS/NLOS
 */

#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
#include "net/netstack.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dw1000.h"
#include "core/net/linkaddr.h"
#include "rng-support.h"

PROCESS(ranging_process, "Realtime tracking tag");
AUTOSTART_PROCESSES(&ranging_process);

#define RANGING_INTERVAL (CLOCK_SECOND / 2)

#include "anchor_table.h"

static const linkaddr_t broadcast_addr = {{0xff, 0xff}};

typedef struct {
    linkaddr_t src;
    uint32_t dist_mm;
    int16_t qual_x10;
    uint16_t pream;
    uint8_t is_los;
    uint8_t used;
} round_measure_t;

static float compute_fp_rx_diff(uint16_t fp1, uint16_t fp2, uint16_t fp3,
                                  uint16_t pream, uint16_t cir_max)
{
    float n2, fp_val, fp_power, rx_power;
    if(pream == 0) return -99.0f;
    n2     = (float)pream * (float)pream;
    fp_val = (float)fp1*(float)fp1 + (float)fp2*(float)fp2 + (float)fp3*(float)fp3;
    if(fp_val < 1.0f) return -99.0f;
    fp_power = 10.0f * log10f(fp_val / n2);
    rx_power = 10.0f * log10f((float)cir_max * 131072.0f / n2);
    return fp_power - rx_power;
}

static int
find_measurement(round_measure_t *measures, uint8_t count, const linkaddr_t *src)
{
  uint8_t i;
  for(i = 0; i < count; i++) {
    if(measures[i].used && linkaddr_cmp(&measures[i].src, src)) {
      return i;
    }
  }
  return -1;
}

static uint8_t
should_replace_measurement(const round_measure_t *current,
                           uint8_t is_los,
                           int16_t qual_x10,
                           uint16_t pream)
{
  if(is_los != current->is_los) {
    return is_los > current->is_los;
  }
  if(qual_x10 != current->qual_x10) {
    return qual_x10 > current->qual_x10;
  }
  return pream > current->pream;
}

PROCESS_THREAD(ranging_process, ev, data)
{
  static struct etimer et;
  static sstwr_init_msg_t init_msg;
  static sstwr_resp_msg_t resp_msg;
  static round_measure_t round_measures[NUM_ANCHORS];
  static uint8_t seqn = 0;
  static uint8_t ret;

  PROCESS_BEGIN();

  printf("I am %02x:%02x\n",
    linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

  radio_init();

  while(1) {
    uint8_t i, n_timeout = 0, n_dup = 0, n_meas = 0, n_los = 0, n_nlos = 0;
    memset(round_measures, 0, sizeof(round_measures));

    seqn++;
    etimer_set(&et, RANGING_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && etimer_expired(&et));

    fill_ieee_hdr(&init_msg.hdr, &linkaddr_node_addr, &broadcast_addr, seqn);
    ret = start_tx(&init_msg, sizeof(init_msg),
                   DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
                   TOTAL_WINDOW, 0);
    if(!ret) { radio_reset(); printf("[%u] fail (TX err)\n", seqn); continue; }

    for(i = 0; i < NUM_ANCHORS; i++) {
      if(i > 0) {
        start_rx(PER_ANCHOR_TIMEOUT);
      }
      ret = wait_rx();
      if(!ret) { n_timeout++; continue; }

      ret = read_rx_data(&resp_msg, sizeof(resp_msg));
      if(!ret) { radio_reset(); continue; }

      linkaddr_t resp_src, resp_dst;
      resp_src.u8[0] = resp_msg.hdr.src[1]; resp_src.u8[1] = resp_msg.hdr.src[0];
      resp_dst.u8[0] = resp_msg.hdr.dst[1]; resp_dst.u8[1] = resp_msg.hdr.dst[0];
      if(!linkaddr_cmp(&resp_dst, &linkaddr_node_addr)) continue;
      if(resp_msg.hdr.seqn != seqn) continue;

      uint64_t init_tx_ts = get_tx_timestamp();
      uint64_t resp_rx_ts = get_rx_timestamp();
      uint64_t init_rx_ts, resp_tx_ts;
      resp_msg_get_timestamp(&resp_msg.init_rx_ts[0], &init_rx_ts);
      resp_msg_get_timestamp(&resp_msg.resp_tx_ts[0], &resp_tx_ts);

      int64_t t_one = (int64_t)((resp_rx_ts - init_tx_ts) % DWT_VALUES);
      int64_t t_two = (int64_t)((resp_tx_ts - init_rx_ts) % DWT_VALUES);
      double tof = ((t_one - t_two) / 2.0) * DWT_TIME_UNITS;
      double dist_m = tof * SPEED_OF_LIGHT;
      // 1. Hard Gating a livello di sensore (scarta l'impossibile)
      if (dist_m < -0.10 || dist_m > 20.0) {
      // Puoi stampare un log di debug qui se ti è utile
        continue; // Oppure usa il codice di errore previsto dalla tua funzione
      }

      // 2. Clamping per le fluttuazioni (salva i calcoli validi)
      // Se c'è un lievissimo underflow dovuto al rumore, forziamo a zero
      if (dist_m < 0.0) {
        dist_m = 0.0;
      }

      // Ora il cast è sicuro e non farà underflow
      uint32_t dist_mm = (uint32_t)lround(dist_m * 1000.0);

      /* Quality filtering */
      float fp_rx_diff = compute_fp_rx_diff(
          resp_msg.fp_amp1, resp_msg.fp_amp2, resp_msg.fp_amp3,
          resp_msg.rx_pream, resp_msg.cir_max);
      int16_t fp_rx_int = (int16_t)lroundf(fp_rx_diff * 10.0f);
      uint8_t is_los = (resp_msg.rx_pream >= MIN_PREAM_COUNT &&
                        fp_rx_diff >= FP_RX_DIFF_MIN);
      int meas_idx = find_measurement(round_measures, n_meas, &resp_src);

      if(meas_idx < 0) {
        if(n_meas >= NUM_ANCHORS) {
          continue;
        }
        meas_idx = n_meas++;
      } else {
        n_dup++;
      }

      if(round_measures[meas_idx].used &&
         !should_replace_measurement(&round_measures[meas_idx],
                                     is_los,
                                     fp_rx_int,
                                     resp_msg.rx_pream)) {
        continue;
      }

      round_measures[meas_idx].src = resp_src;
      round_measures[meas_idx].dist_mm = dist_mm;
      round_measures[meas_idx].qual_x10 = fp_rx_int;
      round_measures[meas_idx].pream = resp_msg.rx_pream;
      round_measures[meas_idx].is_los = is_los;
      round_measures[meas_idx].used = 1;
    }

    for(i = 0; i < n_meas; i++) {
      if(!round_measures[i].used) {
        continue;
      }

      if(round_measures[i].is_los) {
        n_los++;
      } else {
        n_nlos++;
      }

      printf("RANGING MEAS [%u] [%02x:%02x->%02x:%02x] %lu mm QUAL %d %u FLAG %s\n",
        seqn,
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        round_measures[i].src.u8[0], round_measures[i].src.u8[1],
        round_measures[i].dist_mm,
        round_measures[i].qual_x10,
        round_measures[i].pream,
        round_measures[i].is_los ? "LOS" : "NLOS");
    }

    printf("[%u] round: %u meas | %u LOS | %u NLOS | %u timeout | %u dup\n",
      seqn, n_meas, n_los, n_nlos, n_timeout, n_dup);
  }

  PROCESS_END();
}
