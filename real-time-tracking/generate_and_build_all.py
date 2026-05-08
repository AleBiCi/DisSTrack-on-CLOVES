# -*- coding: utf-8 -*-
"""
Genera i file di supporto per il tracking realtime e compila i binari.

Scelta importante:
- NON sovrascrive rng-init-all.c
- NON sovrascrive rng-resp.c

Il motivo e' che questi due file contengono logica applicativa custom
(quality filtering e protocollo RESP esteso) che deve restare intatta.

Il generatore si occupa invece di:
- anchor_table.h
- rng-support.h
- rng-support.c
- compilazione e copia dei .bin
"""

import csv
import os
import re
import shutil
import subprocess
import sys

CSV_FILE = "DEPT_evb1000_map.csv"
VARIANCE_FILE = "variance_per_node.csv"
BINS_DIR = "bins"

ANCHOR_STEP_UUS = 2000
PER_ANCHOR_TIMEOUT_UUS = 6000
WINDOW_MARGIN_UUS = 3000
TRACKING_ANCHOR_IDS = [108] + list(range(113, 120)) + list(range(121, 155))

VALID_NODES = {
    1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
    100, 101, 104, 105, 106, 107, 108, 109, 110, 111, 113, 114, 115,
    116, 117, 118, 119, 121, 122, 123, 124, 125, 126, 127, 128, 129,
    130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142,
    143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154,
}


def parse_map(path):
    nodes = []
    coord_re = re.compile(r"\[([0-9.]+),\s*([0-9.]+)\]")
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["Zone"].strip() != "DEPT":
                continue
            node_id = int(row["NodeId"])
            if node_id not in VALID_NODES:
                continue
            match = coord_re.search(row["Coordinates"])
            if not match:
                continue
            short = row["evb1000"].strip()[-5:].lower()
            nodes.append({
                "id": node_id,
                "short": short,
                "hi": short[:2].upper(),
                "lo": short[3:].upper(),
                "x": float(match.group(1)),
                "y": float(match.group(2)),
            })
    nodes.sort(key=lambda n: n["id"])
    return nodes


def load_variances(path):
    if not os.path.exists(path):
        return {}
    variances = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            node_id = int(row["node_id"])
            variances[node_id] = {
                "std_m": float(row["pooled_std_m"]),
                "bias_mm": float(row["mean_bias_mm"]) if row["mean_bias_mm"] else None,
            }
    return variances


def select_tracking_anchors(nodes, variances):
    selected = []
    for node in nodes:
        if node["id"] not in TRACKING_ANCHOR_IDS:
            continue
        variance = variances.get(node["id"])
        enriched = dict(node)
        enriched["std_m"] = variance["std_m"] if variance else 0.0
        enriched["bias_mm"] = variance["bias_mm"] if variance else None
        selected.append(enriched)
    selected.sort(key=lambda n: TRACKING_ANCHOR_IDS.index(n["id"]))
    return selected


def render_anchor_table(nodes):
    num_anchors = len(nodes)
    total_window = (num_anchors * ANCHOR_STEP_UUS) + WINDOW_MARGIN_UUS
    rows = []
    for slot, node in enumerate(nodes):
        rows.append(
            "  {{0x%(hi)s, 0x%(lo)s}, %(slot)2d},  /* Node %(id)3d | std=%(std).3fm | (%(x)6.2f, %(y)6.2f) m */"
            % {
                "hi": node["hi"],
                "lo": node["lo"],
                "slot": slot,
                "id": node["id"],
                "std": node["std_m"],
                "x": node["x"],
                "y": node["y"],
            }
        )

    return """\
/*
 * anchor_table.h - GENERATO AUTOMATICAMENTE da generate_and_build_all.py
 * Ancore richieste: 108 e 113..154 (senza 120)
 */
#ifndef ANCHOR_TABLE_H
#define ANCHOR_TABLE_H

#define NUM_ANCHORS {num_anchors}
#define ANCHOR_STEP_UUS {step}
#define TOTAL_WINDOW {total_window}
#define PER_ANCHOR_TIMEOUT {per_timeout}

typedef struct {{
  uint8_t addr[2];
  uint8_t slot;
}} anchor_entry_t;

static const anchor_entry_t anchor_table[NUM_ANCHORS] = {{
{rows}
}};

static inline uint32_t get_resp_delay_from_table(linkaddr_t addr)
{{
  uint8_t i;
  for(i = 0; i < NUM_ANCHORS; i++) {{
    if(anchor_table[i].addr[0] == addr.u8[0] &&
       anchor_table[i].addr[1] == addr.u8[1]) {{
      return (uint32_t)(anchor_table[i].slot + 1) * ANCHOR_STEP_UUS;
    }}
  }}
  return ANCHOR_STEP_UUS;
}}

#endif /* ANCHOR_TABLE_H */
""".format(
        num_anchors=num_anchors,
        step=ANCHOR_STEP_UUS,
        total_window=total_window,
        per_timeout=PER_ANCHOR_TIMEOUT_UUS,
        rows="\n".join(rows),
    )


def render_rng_support_h():
    return """\
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
#define FP_RX_DIFF_MIN   (-6.0f)
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
"""


def render_rng_support_c():
    return """\
#include "dw1000.h"
#include "net/netstack.h"
#include "dev/watchdog.h"
#include "rng-support.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
uint64_t
get_rx_timestamp(void)
{
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for(i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}
/*---------------------------------------------------------------------------*/
uint64_t
get_tx_timestamp(void)
{
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  int i;
  dwt_readtxtimestamp(ts_tab);
  for(i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}
/*---------------------------------------------------------------------------*/
uint64_t
predict_tx_timestamp(uint64_t tx_ts)
{
  return (tx_ts & DWT_TX_BITMASK) + ANTENNA_DELAY;
}
/*---------------------------------------------------------------------------*/
void
resp_msg_set_timestamp(uint8_t *ts_field, uint64_t ts)
{
  int i;
  for(i = 0; i < RNG_TS_LEN; i++) {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}
/*---------------------------------------------------------------------------*/
void
resp_msg_get_timestamp(uint8_t *ts_field, uint64_t *ts)
{
  int i;
  *ts = 0;
  for(i = 0; i < RNG_TS_LEN; i++) {
    *ts |= (uint64_t)(ts_field[i]) << (i * 8);
  }
}
/*---------------------------------------------------------------------------*/
void
fill_ieee_hdr(ieee154_hdr_t *hdr, linkaddr_t *src, linkaddr_t *dst, uint8_t seqn)
{
  hdr->fctrl[0] = 0x41;
  hdr->fctrl[1] = 0x88;
  hdr->seqn = seqn;
  hdr->pan_id[0] = IEEE802154_PANID & 0xff;
  hdr->pan_id[1] = IEEE802154_PANID >> 8;
  if(src != NULL) {
    hdr->src[0] = src->u8[1];
    hdr->src[1] = src->u8[0];
  }
  if(dst != NULL) {
    hdr->dst[0] = dst->u8[1];
    hdr->dst[1] = dst->u8[0];
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
start_tx(void *data, uint8_t len, uint8_t mode, uint16_t rx_to, uint64_t tx_time)
{
  dwt_forcetrxoff();
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS | RX_WAIT_FLAGS);

  if (mode & DWT_RESPONSE_EXPECTED) {
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(rx_to);
  }

  if (mode & DWT_START_TX_DELAYED) {
    dwt_setdelayedtrxtime((uint32_t)((tx_time & DWT_TX_BITMASK) >> 8));
  } else {
    dwt_setdelayedtrxtime(0);
  }

  if(dwt_writetxdata(len + CRC_LEN, (uint8_t*)data, 0) == DWT_SUCCESS) {
    dwt_writetxfctrl(len + CRC_LEN, 0, 1);
    dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_TXCLKS_125M);

    if(dwt_starttx(mode) == DWT_SUCCESS) {
      while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
        watchdog_periodic();
      }
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
      return 1;
    }
  }

  PRINTF("TX failed. Status: %lx\\n", dwt_read32bitreg(SYS_STATUS_ID));
  return 0;
}
/*---------------------------------------------------------------------------*/
void
radio_init(void)
{
  dwt_forcetrxoff();
  dwt_setcallbacks(0, 0, 0, 0);
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO |
    DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_ARFE, 0);
  dwt_setrxantennadelay(ANTENNA_DELAY);
  dwt_settxantennadelay(ANTENNA_DELAY);
  dwt_enableframefilter(DWT_FF_NOTYPE_EN);
}
/*---------------------------------------------------------------------------*/
void
start_rx(uint16_t rx_to)
{
  /* Clear stale RX flags before enabling the next slot, but keep turnaround light. */
  dwt_write32bitreg(SYS_STATUS_ID, RX_WAIT_FLAGS);
  dwt_setrxtimeout(rx_to);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
/*---------------------------------------------------------------------------*/
uint8_t
wait_rx(void)
{
  uint32_t status_reg;
  while(!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & RX_WAIT_FLAGS)) {
    watchdog_periodic();
  }
  if (status_reg & SYS_STATUS_RXFCG) {
    return 1;
  }

  dwt_write32bitreg(SYS_STATUS_ID, status_reg & RX_WAIT_FLAGS);
  if(status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
    dwt_rxreset();
  }
  PRINTF("RX failed. Status: %lx\\n", status_reg);
  return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
read_rx_data(void* pkt, uint32_t pkt_size)
{
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
  if (frame_len == pkt_size + CRC_LEN) {
    dwt_readrxdata((uint8_t *)pkt, pkt_size, 0);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    return 1;
  }

  dwt_write32bitreg(SYS_STATUS_ID, RX_WAIT_FLAGS);
  return 0;
}
/*---------------------------------------------------------------------------*/
void
radio_reset(void)
{
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS | RX_WAIT_FLAGS);
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  dwt_rxreset();
  dwt_forcetrxoff();
}
/*---------------------------------------------------------------------------*/
"""


def write_file(path, content):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(content)


def build_binary(target_name):
    result = subprocess.call(f"make TARGET=evb1000 {target_name}", shell=True)
    if result != 0:
        raise RuntimeError(f"compilazione fallita per {target_name}")

    for ext in ("bin", "evb1000"):
        candidate = f"{target_name}.{ext}"
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(f"binario non trovato per {target_name}")


def main():
    nodes = parse_map(CSV_FILE)
    variances = load_variances(VARIANCE_FILE)
    selected = select_tracking_anchors(nodes, variances)

    if not selected:
        print("[error] nessuna ancora selezionata")
        sys.exit(1)

    print(f"[select] ancore scelte per tracking: {len(selected)}")
    print(f"[select] ids: {TRACKING_ANCHOR_IDS}")

    write_file("anchor_table.h", render_anchor_table(selected))
    write_file("rng-support.h", render_rng_support_h())
    write_file("rng-support.c", render_rng_support_c())
    os.makedirs(BINS_DIR, exist_ok=True)

    try:
        tag_bin = build_binary("rng-init-all")
        resp_bin = build_binary("rng-resp")
    except RuntimeError as exc:
        print(f"[error] {exc}")
        sys.exit(1)

    shutil.copy(tag_bin, os.path.join(BINS_DIR, "rng-init-all.bin"))
    shutil.copy(resp_bin, os.path.join(BINS_DIR, "rng-resp.bin"))

    print("[output] scritto anchor_table.h")
    print("[output] scritto rng-support.h")
    print("[output] scritto rng-support.c")
    print("[done] pronto per il flash del tag e delle ancore")


if __name__ == "__main__":
    main()
