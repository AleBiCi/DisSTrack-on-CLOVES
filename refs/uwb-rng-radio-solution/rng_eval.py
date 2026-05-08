#!/usr/bin/env python3
import os
import sys
import re
import pandas as pd

def distance(a, b):
    return (
        ((a[0] - b[0]) ** 2) +
        ((a[1] - b[1]) ** 2)
    ) ** 0.5

def parse_file(log_file, csv_file="DEPT_evb1000_map.csv"):

    # Dataframe from deployment csv
    deployment_df = pd.read_csv(csv_file)

    # Obtain short addresses (last 5 characters) and add them to the dataframe
    deployment_df['evb1000Short'] = deployment_df['evb1000'].apply(lambda x: x[-5:])

    # Create the id_by_addr and the coord_by_id dictionary
    id_by_addr = dict(zip(deployment_df['evb1000Short'], deployment_df["NodeId"]))
    coord_strings = deployment_df['Coordinates'].apply(lambda x: x.translate(str.maketrans('', '', '[] ')))
    coordinates = []
    for c in coord_strings:
        try:
            coordinates.append(list(map(float, c.split(','))))
        except ValueError:
            coordinates.append(None)
    print(coordinates)
    coord_by_id = dict(zip(deployment_df["NodeId"], coordinates))

    # Regular expressions to match
    # Ex.: RANGING OK [11:0c->19:15] 169 mm
    regex_rng = re.compile(r".*RANGING OK \[(?P<init>\w\w:\w\w)->(?P<resp>\w\w:\w\w)\] (?P<dist>\d+) mm.*\n")

    # Open log and read line by line
    with open(log_file, 'r') as f:
        for line in f:

            # Match transmissions strings
            m = regex_rng.match(line)
            if m:

                # Get dictionary of matched groups
                d = m.groupdict()

                # Retrieve IDs of nodes and the measured distance
                init = id_by_addr[d['init']]
                resp = id_by_addr[d['resp']]
                dist = int(d['dist'])

                # Retrieve coordinates
                init_coords = coord_by_id[init]
                resp_coords = coord_by_id[resp]

                # Compute the error
                true_dist = int(distance(init_coords, resp_coords) * 1000) # mm
                err = int(dist - true_dist)
                print(f"Ranging error [{init}->{resp}] {err} (dist {dist} true dist {true_dist})")


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print("Error: Missing log file.")
        sys.exit(1)

    # Get the log path and check that it exists
    log_file = sys.argv[1]
    if not os.path.isfile(log_file) or not os.path.exists(log_file):
        print("Error: Log file not found.")
        sys.exit(1)

    # Parse file
    parse_file(log_file)
