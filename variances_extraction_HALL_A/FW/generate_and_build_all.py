# -*- coding: utf-8 -*-
import os, re, shutil, subprocess, sys, csv, math

CSV_FILE      = "HALL-A_evb1000_map.csv"
NUM_NEIGHBORS = 5
BUILD_BASE    = "build_nodes"
BINS_DIR      = "bins"

VALID_NODES = {
    50, 51, 52, 53, 54, 55, 56, 57, 58,
    61, 62, 63, 64, 65, 70, 71, 72, 73,
    74, 75, 76, 77
}

def parse_csv(path):
    nodes = []
    coord_re = re.compile(r'\[([0-9.]+),\s*([0-9.]+)\]')
    with open(path) as f:
        for row in csv.DictReader(f):
            if row['Zone'].strip() != 'HALL-A':
                continue
            nid = int(row['NodeId'])
            if nid not in VALID_NODES:
                continue
            m = coord_re.search(row['Coordinates'])
            if not m:
                continue
            short = row['evb1000'].strip()[-5:].lower()
            nodes.append({
                'id':    nid,
                'x':     float(m.group(1)),
                'y':     float(m.group(2)),
                'short': short,
                'hi':    short[:2].upper(),
                'lo':    short[3:].upper(),
            })
    return nodes

def find_neighbors(nodes, idx, n):
    ref = nodes[idx]
    dists = []
    for i, nd in enumerate(nodes):
        if i == idx:
            continue
        d = math.sqrt((nd['x']-ref['x'])**2 + (nd['y']-ref['y'])**2)
        dists.append((d, i))
    dists.sort()
    return [nodes[i] for _, i in dists[:n]]

def generate_c(node, neighbors):
    resp_lines = "\n".join(
        "  {{{{0x{hi}, 0x{lo}}}}},  /* Node {id} */".format(**r)
        for r in neighbors
    )
    return """\
#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
#include "net/netstack.h"
#include <stdio.h>
#include "dw1000.h"
#include "core/net/linkaddr.h"
#include "rng-support.h"

PROCESS(ranging_process, "Ranging process");
AUTOSTART_PROCESSES(&ranging_process);

#define RANGING_INTERVAL (CLOCK_SECOND / 2)
#define RANGING_TIMEOUT  (5000)
#define NUM_DEST {n}

static linkaddr_t resp_list[NUM_DEST] = {{
{resp_lines}
}};
static linkaddr_t resp;

PROCESS_THREAD(ranging_process, ev, data)
{{
  static struct etimer et;
  static sstwr_init_msg_t init_msg;
  static sstwr_resp_msg_t resp_msg;
  static uint8_t seqn = 0;
  static uint8_t ret;

  PROCESS_BEGIN();
  printf("I am %02x:%02x\\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
  radio_init();

  while(1) {{
    seqn++;
    linkaddr_copy(&resp, &resp_list[seqn % NUM_DEST]);
    etimer_set(&et, RANGING_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && etimer_expired(&et));
    printf("[%u] ranging with %02x:%02x ...\\n", seqn, resp.u8[0], resp.u8[1]);
    fill_ieee_hdr(&init_msg.hdr, &linkaddr_node_addr, &resp, seqn);
    ret = start_tx(&init_msg, sizeof(init_msg),
                   DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED,
                   RANGING_TIMEOUT, 0);
    if (!ret) {{ radio_reset(); printf("[%u] fail TX\\n", seqn); continue; }}
    ret = wait_rx();
    if (!ret) {{ radio_reset(); printf("[%u] fail RX\\n", seqn); continue; }}
    ret = read_rx_data(&resp_msg, sizeof(resp_msg));
    if (!ret) {{ radio_reset(); printf("[%u] fail size\\n", seqn); continue; }}
    linkaddr_t resp_src, resp_dst;
    resp_src.u8[0] = resp_msg.hdr.src[1]; resp_src.u8[1] = resp_msg.hdr.src[0];
    resp_dst.u8[0] = resp_msg.hdr.dst[1]; resp_dst.u8[1] = resp_msg.hdr.dst[0];
    if(linkaddr_cmp(&resp_src, &resp) && linkaddr_cmp(&resp_dst, &linkaddr_node_addr)) {{
      uint64_t init_tx_ts = get_tx_timestamp();
      uint64_t resp_rx_ts = get_rx_timestamp();
      uint64_t init_rx_ts, resp_tx_ts;
      resp_msg_get_timestamp(&resp_msg.init_rx_ts[0], &init_rx_ts);
      resp_msg_get_timestamp(&resp_msg.resp_tx_ts[0], &resp_tx_ts);
      int64_t t_one = (int64_t)((resp_rx_ts - init_tx_ts) % DWT_VALUES);
      int64_t t_two = (int64_t)((resp_tx_ts - init_rx_ts) % DWT_VALUES);
      double tof = ((t_one - t_two) / 2.0) * DWT_TIME_UNITS;
      uint64_t dist_mm = (uint64_t)(tof * SPEED_OF_LIGHT * 1000);
      printf("RANGING OK [%02x:%02x->%02x:%02x] %llu mm\\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        resp.u8[0], resp.u8[1], dist_mm);
    }} else {{
      printf("[%u] fail addr\\n", seqn);
    }}
  }}
  PROCESS_END();
}}
""".format(n=len(neighbors), resp_lines=resp_lines)

def main():
	print("[main] CSV_FILES={}".format(CSV_FILE))
	nodes = parse_csv(CSV_FILE)
	print("[main] {} nodi validi".format(len(nodes)))
	os.makedirs(BINS_DIR)
	common = ["rng-resp.c","rng-support.c","rng-support.h","project-conf.h","Makefile"]
	missing = [f for f in common if not os.path.exists(f)]
	if missing:
		print("[ERROR] File mancanti: {}".format(missing))
		sys.exit(1)
	success, failed = 0, []
	total = len(nodes)
	for idx, node in enumerate(nodes):
		nid     = node['id']
		bin_out = os.path.join(BINS_DIR, "rng-init-node{:03d}.bin".format(nid))
		if os.path.exists(bin_out):
			print("[{:3d}/{}] Node {:3d} - skip".format(idx+1, total, nid))
			success += 1
			continue
		neighbors = find_neighbors(nodes, idx, NUM_NEIGHBORS)
		print("\n[{:3d}/{}] Node {:3d} -> vicini: {}".format(
		      idx+1, total, nid, [r['id'] for r in neighbors]))
		build_dir = os.path.join(BUILD_BASE, "node_{:03d}".format(nid))
		os.makedirs(build_dir)
		with open(os.path.join(build_dir, "rng-init.c"), "w") as f:
			f.write(generate_c(node, neighbors))
		for f in common:
			shutil.copy(f, build_dir)
		res = subprocess.call("make TARGET=evb1000 rng-init", shell=True, cwd=build_dir)
		if res != 0:
			print("  [ERROR] compilazione fallita")
			failed.append(nid)
			continue
		copied = False
		for ext in ['bin', 'evb1000']:
			src = os.path.join(build_dir, "rng-init.{}".format(ext))
			if os.path.exists(src):
				shutil.copy(src, bin_out)
				print("  OK -> {}".format(bin_out))
				success += 1
				copied = True
				break
		if not copied:
			failed.append(nid)
	for ext in ['evb1000','bin']:
		src = "rng-resp.{}".format(ext)
		if os.path.exists(src):
			shutil.copy(src, os.path.join(BINS_DIR, "rng-resp.bin"))
			break
	print("\nOK: {}/{}  Falliti: {}".format(success, total, failed))
	print("Binari in: {}/".format(BINS_DIR))

if __name__ == "__main__":
	main()
