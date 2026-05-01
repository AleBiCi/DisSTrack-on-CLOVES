#!/usr/bin/env python3
"""
compute_variances.py
--------------------
Legge tutti i file serial.X.log dentro logs_varianza/job_XXXXX/
e calcola la varianza per ogni coppia (initiator → responder).

Output:
  - variance_per_pair.csv    → varianza per ogni coppia (init, resp)
  - variance_per_node.csv    → std dev pooled per ogni nodo initiator
                               (questo è il noise_std da usare nell'EKF)

Uso:
  python compute_variances.py
  python compute_variances.py --logs-dir logs_varianza --csv DEPT_evb1000_map.csv
"""

import argparse
import glob
import os
import re
from collections import defaultdict

import numpy as np
import pandas as pd

LOGS_DIR = "logs_varianza_v2"
CSV_FILE = "HALL-A_evb1000_map.csv"

PATTERN = re.compile(
    r'RANGING OK \[([0-9a-f]{2}:[0-9a-f]{2})->([0-9a-f]{2}:[0-9a-f]{2})\]\s+(\d+)\s+mm',
    re.IGNORECASE
)


def load_map(csv_path):
    df = pd.read_csv(csv_path)
    df['short'] = df['evb1000'].astype(str).str.strip().str[-5:].str.lower()
    df['NodeId'] = df['NodeId'].astype(int)

    coord_re = re.compile(r'\[([0-9.]+),\s*([0-9.]+)\]')
    xs, ys = [], []
    for val in df['Coordinates']:
        m = coord_re.search(str(val))
        xs.append(float(m.group(1)) if m else None)
        ys.append(float(m.group(2)) if m else None)
    df['x'] = xs
    df['y'] = ys

    addr_to_id = dict(zip(df['short'], df['NodeId']))
    id_to_xy   = {int(row.NodeId): (row.x, row.y)
                  for row in df.dropna(subset=['x','y']).itertuples()}
    return addr_to_id, id_to_xy


def parse_log(path, addr_to_id):
    """Legge un file .log e ritorna lista di (init_id, resp_id, dist_mm)."""
    results = []
    try:
        with open(path, 'r', errors='ignore') as f:
            for line in f:
                m = PATTERN.search(line)
                if m:
                    init_addr = m.group(1).lower()
                    resp_addr = m.group(2).lower()
                    dist_mm   = int(m.group(3))
                    init_id   = addr_to_id.get(init_addr)
                    resp_id   = addr_to_id.get(resp_addr)
                    if init_id and resp_id:
                        results.append((init_id, resp_id, dist_mm))
    except Exception as e:
        print(f"  [warn] errore leggendo {path}: {e}")
    return results


def main(logs_dir, csv_path):
    addr_to_id, id_to_xy = load_map(csv_path)
    print(f"[map] {len(addr_to_id)} nodi caricati dal CSV")

    # Raccoglie tutte le misure: (init_id, resp_id) → [dist_mm, ...]
    all_meas = defaultdict(list)
    n_files  = 0
    n_lines  = 0

    # Cerca tutti i serial.X.log in qualsiasi sottocartella di logs_dir
    pattern = os.path.join(logs_dir, "**", "serial.*.log")
    log_files = glob.glob(pattern, recursive=True)

    if not log_files:
        # Prova anche job.log e test.log
        for pat in ["**/job.log", "**/test.log", "**/serial.log"]:
            log_files += glob.glob(os.path.join(logs_dir, pat), recursive=True)

    print(f"[scan] trovati {len(log_files)} file log in {logs_dir}/")

    for log_path in sorted(log_files):
        meas = parse_log(log_path, addr_to_id)
        if meas:
            for init_id, resp_id, dist_mm in meas:
                all_meas[(init_id, resp_id)].append(dist_mm)
            n_lines += len(meas)
            n_files += 1

    print(f"[parse] {n_files} file con dati, {n_lines} misure RANGING OK totali")
    print(f"[parse] {len(all_meas)} coppie (init→resp) distinte")

    if not all_meas:
        print("[ERROR] Nessuna misura trovata. Controlla il formato dei log.")
        return

    # ── Tabella per coppia ────────────────────────────────────────────────────
    def true_dist(init_id, resp_id):
        pi = id_to_xy.get(init_id)
        pr = id_to_xy.get(resp_id)
        if pi and pr:
            return np.sqrt((pi[0]-pr[0])**2 + (pi[1]-pr[1])**2) * 1000
        return None

    pair_rows = []
    for (init_id, resp_id), dists in sorted(all_meas.items()):
        dists  = np.array(dists)
        td     = true_dist(init_id, resp_id)
        pair_rows.append({
            'init_id':       init_id,
            'resp_id':       resp_id,
            'n_samples':     len(dists),
            'mean_mm':       round(float(np.mean(dists)), 2),
            'std_mm':        round(float(np.std(dists)),  2),
            'var_mm2':       round(float(np.var(dists)),  2),
            'min_mm':        int(np.min(dists)),
            'max_mm':        int(np.max(dists)),
            'true_dist_mm':  round(td, 1) if td else None,
            'bias_mm':       round(float(np.mean(dists)) - td, 1) if td else None,
        })

    pair_df = pd.DataFrame(pair_rows)
    pair_df.to_csv("variance_per_pair.csv", index=False)
    print(f"\n[output] variance_per_pair.csv  ({len(pair_df)} coppie)")

    # ── Tabella per nodo initiator ────────────────────────────────────────────
    node_rows = []
    for init_id, group in pair_df.groupby('init_id'):
        # Pooled std dev = sqrt(media delle varianze per coppia)
        pooled_std_mm = float(np.sqrt(group['var_mm2'].mean()))
        pooled_std_m  = pooled_std_mm / 1000.0

        node_rows.append({
            'node_id':        init_id,
            'n_pairs':        len(group),
            'n_samples_tot':  int(group['n_samples'].sum()),
            'pooled_std_mm':  round(pooled_std_mm, 3),
            'pooled_std_m':   round(pooled_std_m,  6),
            'mean_bias_mm':   round(float(group['bias_mm'].mean()), 2)
                              if group['bias_mm'].notna().any() else None,
        })

    node_df = pd.DataFrame(node_rows).sort_values('node_id')
    node_df.to_csv("variance_per_node.csv", index=False)
    print(f"[output] variance_per_node.csv  ({len(node_df)} nodi)")

    # ── Stampa riepilogo ──────────────────────────────────────────────────────
    print("\n" + "="*55)
    print("  RIEPILOGO STD DEV PER NODO (noise_std per EKF)")
    print("="*55)
    print(f"  {'Node':>6}  {'Coppie':>6}  {'Campioni':>8}  "
          f"{'Std[mm]':>8}  {'Std[m]':>8}")
    print("  " + "-"*51)
    for _, r in node_df.iterrows():
        print(f"  {int(r.node_id):>6}  {int(r.n_pairs):>6}  "
              f"{int(r.n_samples_tot):>8}  "
              f"{r.pooled_std_mm:>8.2f}  {r.pooled_std_m:>8.4f}")

    global_std_mm = float(np.sqrt(pair_df['var_mm2'].mean()))
    global_std_m  = global_std_mm / 1000.0
    print("  " + "-"*51)
    print(f"\n  Std Dev GLOBALE: {global_std_mm:.2f} mm  ({global_std_m:.4f} m)")
    print(f"\n  → Usa 'pooled_std_m' per ogni nodo in ekf.py")
    print(f"  → Oppure usa {global_std_m:.4f} come valore unico globale")
    print("="*55)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs-dir', default=LOGS_DIR)
    ap.add_argument('--csv',      default=CSV_FILE)
    args = ap.parse_args()
    main(args.logs_dir, args.csv)