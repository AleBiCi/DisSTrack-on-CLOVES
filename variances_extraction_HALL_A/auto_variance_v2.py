#!/usr/bin/env python3
"""
auto_variance_v2.py
-------------------
Come auto_variance.py ma usa un binario specifico per ogni nodo
(con solo i 5 vicini nella lista) invece di rng-init-all.bin.

I binari devono essere in bins/rng-init-node{NNN}.bin
(generati da generate_and_build_all.py nella VM).

Uso:
  python auto_variance_v2.py               # sottomette tutti i job
  python auto_variance_v2.py --dry-run     # test senza sottomettere
  python auto_variance_v2.py --start-from 10
  python auto_variance_v2.py --download    # scarica i risultati
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
import csv as csvlib

import numpy as np
import pandas as pd

# ── Config ────────────────────────────────────────────────────────────────────
CSV_FILE      = "HALL-A_evb1000_map.csv"
NUM_NEIGHBORS = 5
JOB_DURATION  = 120
JOBS_LOG      = "submitted_jobs_v2.csv"
RESULTS_DIR   = "logs_varianza_v2"
BINS_DIR      = "bins"
CLOVES_CLIENT = os.path.join("cloves-client", "iot_testbed_client.py")

VALID_NODES = {
    70, 71, 72, 73, 74, 75, 76, 77
}
# ─────────────────────────────────────────────────────────────────────────────


def parse_csv(path):
    df = pd.read_csv(path)
    df = df[df['Zone'].astype(str).str.strip() == 'HALL-A'].copy()
    df['NodeId'] = df['NodeId'].astype(int)
    df = df[df['NodeId'].isin(VALID_NODES)].reset_index(drop=True)

    coord_re = re.compile(r'\[([0-9.]+),\s*([0-9.]+)\]')
    xs, ys = [], []
    for val in df['Coordinates']:
        m = coord_re.search(str(val))
        xs.append(float(m.group(1)) if m else np.nan)
        ys.append(float(m.group(2)) if m else np.nan)
    df['x'] = xs
    df['y'] = ys
    return df.dropna(subset=['x', 'y']).reset_index(drop=True)


def find_neighbors(df, idx, n):
    pos   = df[['x', 'y']].values
    dists = np.linalg.norm(pos - pos[idx], axis=1)
    dists[idx] = np.inf
    return df.iloc[np.argsort(dists)[:n]]['NodeId'].astype(int).tolist()


def submit_job(node_id, resp_ids, init_bin, resp_bin, dry_run=False):
    experiment = {
        "island":     "HALL-A",
        "start_time": "asap",
        "duration":   JOB_DURATION,
        "logs":       2,
        "binaries": [
            {
                "hardware": "evb1000",
                "bin_file": os.path.basename(init_bin),
                "targets":  [node_id]
            },
            {
                "hardware": "evb1000",
                "bin_file": "rng-resp.bin",
                "targets":  resp_ids
            }
        ]
    }

    # Crea cartella temporanea per il job con i file necessari
    job_dir = f"_job_tmp_{node_id}"
    os.makedirs(job_dir, exist_ok=True)
    shutil.copy(init_bin, os.path.join(job_dir, os.path.basename(init_bin)))
    shutil.copy(resp_bin, os.path.join(job_dir, "rng-resp.bin"))

    exp_path = os.path.join(job_dir, "experiment.json")
    with open(exp_path, "w") as f:
        json.dump(experiment, f, indent=3)

    if dry_run:
        shutil.rmtree(job_dir)
        return "DRY_RUN"

    client = os.path.abspath(CLOVES_CLIENT)
    result = subprocess.run(
    f'python "{client}" schedule "experiment.json"',
    shell=True, capture_output=True, text=True,
    cwd=job_dir
    )
    output = result.stdout + result.stderr
    m = re.search(r"'job_id':\s*(\d+)", output)
    job_id = m.group(1) if m else "UNKNOWN"

    if job_id == "UNKNOWN":
        print(f"  [warn] {output.strip()[:500]}")

    shutil.rmtree(job_dir)
    return job_id


def download_all():
    if not os.path.exists(JOBS_LOG):
        print(f"[download] {JOBS_LOG} non trovato.")
        return

    os.makedirs(RESULTS_DIR, exist_ok=True)
    client = os.path.abspath(CLOVES_CLIENT)

    with open(JOBS_LOG) as f:
        reader = csvlib.DictReader(f)
        for row in reader:
            job_id  = row.get('job_id', '').strip()
            node_id = row.get('node_id', '').strip()
            if job_id in ('', 'UNKNOWN', 'DRY_RUN', 'SKIPPED', 'NO_BIN'):
                continue

            out_dir = os.path.join(RESULTS_DIR, f"node_{int(node_id):03d}")
            os.makedirs(out_dir, exist_ok=True)

            print(f"[download] node_{int(node_id):03d}  job_id={job_id}")
            result = subprocess.run(
                f'python "{client}" download -u {job_id}',
                shell=True, capture_output=True, text=True,
                cwd=out_dir
            )
            print(f"  {(result.stdout + result.stderr).strip()[:150]}")

    print(f"\n[download] Completato. Log in {RESULTS_DIR}/")


def main():
    global JOB_DURATION

    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run',    action='store_true')
    ap.add_argument('--download',   action='store_true')
    ap.add_argument('--start-from', type=int, default=0)
    ap.add_argument('--csv',        default=CSV_FILE)
    ap.add_argument('--duration',   type=int, default=JOB_DURATION)
    args = ap.parse_args()

    JOB_DURATION = args.duration

    if args.download:
        download_all()
        return

    # Controlla che rng-resp.bin esista
    resp_bin = "rng-resp.bin"
    if not os.path.exists(resp_bin) and not args.dry_run:
        print(f"[ERRORE] {resp_bin} non trovato.")
        sys.exit(1)

    # Controlla che la cartella bins/ esista
    if not os.path.isdir(BINS_DIR) and not args.dry_run:
        print(f"[ERRORE] cartella '{BINS_DIR}/' non trovata.")
        print(f"  Genera i binari con generate_and_build_all.py nella VM")
        sys.exit(1)

    df    = parse_csv(args.csv)
    total = len(df)
    print(f"[main] {total} nodi HALL-A validi")
    print(f"[main] Durata job: {JOB_DURATION}s | Vicini: {NUM_NEIGHBORS}")
    if args.dry_run:
        print("[main] MODALITA' DRY-RUN\n")

    log_rows = []

    for idx, row in df.iterrows():
        if idx < args.start_from:
            log_rows.append({'node_id': int(row['NodeId']),
                             'job_id': 'SKIPPED', 'responders': ''})
            continue

        node_id   = int(row['NodeId'])
        neighbors = find_neighbors(df, idx, NUM_NEIGHBORS)
        init_bin  = os.path.join(BINS_DIR, f"rng-init-node{node_id:03d}.bin")

        print(f"\n[{idx+1:3d}/{total}] Node {node_id:3d} "
              f"({row['x']:.1f}, {row['y']:.1f})  →  resp: {neighbors}")

        # Controlla che il binario esista
        if not os.path.exists(init_bin) and not args.dry_run:
            print(f"  [warn] {init_bin} non trovato — skip")
            log_rows.append({'node_id': node_id, 'job_id': 'NO_BIN',
                             'responders': str(neighbors)})
            continue

        job_id = submit_job(node_id, neighbors, init_bin, resp_bin,
                            dry_run=args.dry_run)
        print(f"  job_id = {job_id}")

        log_rows.append({'node_id': node_id, 'job_id': job_id,
                         'responders': str(neighbors)})
        pd.DataFrame(log_rows).to_csv(JOBS_LOG, index=False)

        if not args.dry_run and job_id != "UNKNOWN":
            time.sleep(3)

    submitted = sum(1 for r in log_rows
                    if r['job_id'] not in ('SKIPPED','UNKNOWN','DRY_RUN','NO_BIN'))
    print(f"\n{'='*50}")
    print(f"  Job sottomessi: {submitted}/{total}")
    print(f"  Log: {JOBS_LOG}")
    print(f"\n  Quando finiti:")
    print(f"    python auto_variance_v2.py --download")
    print(f"    python compute_variances.py --logs-dir {RESULTS_DIR}")
    print(f"{'='*50}")


if __name__ == "__main__":
    main()
