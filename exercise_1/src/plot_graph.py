import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os.path
import csv
import numpy as np
import math

def numberOfRows(csvFile:str):
    count = 0
    with open(csvFile, "rt") as f:
        reader = csv.reader(f)
        count = sum(1 for row in reader)
    return count

def checkNameInCSV(name:str, csvFile:str):
    pos = 0
    found = False
    with open(csvFile, "rt") as f:
        reader = csv.reader(f, delimiter=",")
        for row in reader:
            if name in row:
                found = True
                pos = row.index(name)
                break
    return (found, pos)

def checkFileExists(filePath: str):
    return os.path.isfile(filePath)

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    parser.add_argument("-s", type=float, required=True, help="The sample data rate in seconds.")
    parser.add_argument("-y", type=str, nargs="+", required=True, help="The signal to plot.")
    parser.add_argument("-ng", action="store_false", help="No GUI. Save PNG file with same csv_file_name.")
    parser.add_argument("-pk", action="store_true", help="Find the time to get from 0 to 100km/h.")
    parser.add_argument("-ns", action="store_false", help="Do not save result to file.")
    return parser.parse_args()

def main():
    args = parseArguments()
    sampleRate = args.s
    prop_name = "vd_double_track/x_vec/v_x_mps"
    max_v = 27.78
    steady_state = 0.95

    if not checkFileExists(args.csv_file):
        print("file does not exist")
        exit(0)

    for name in args.y:
        if not checkNameInCSV(name, args.csv_file)[0]:
            print(f"[{name}] not found")
            exit(0)
    
    # The time to get from 0 to 95% of max speed
    if not checkNameInCSV(prop_name, args.csv_file)[0]:
        print("param not found")
        exit(0)
    
    startTime = 0
    finalTime = (numberOfRows(args.csv_file) - 1)*sampleRate # first row is header
    time_axis = np.arange(startTime, finalTime, sampleRate).tolist()
    data = pd.read_csv(args.csv_file)
    nPlots = math.ceil(math.sqrt(len(args.y)))
    fig, ax = plt.subplots(nPlots, nPlots, sharex="row")
    ax_idx_row = 0
    ax_idx_col = 0
    for name in args.y:
        ax[ax_idx_row, ax_idx_col].plot(time_axis, data[name])
        ax[ax_idx_row, ax_idx_col].set_title(name)
        ax_idx_col += 1
        if (ax_idx_col == nPlots):
            ax_idx_col = 0
            ax_idx_row += 1
    fig.tight_layout()

    if args.ng:
        plt.show()
    elif args.ns:
        plt.savefig(args.csv_file+".png")
    
    if args.pk:
        count = 0
        for v in data[prop_name]:
            if v == 0:
                continue
            elif (v > 0) and (v < steady_state*max_v):
                count += 1
            else:
                break
        print(f"{args.csv_file}:\ntime to 95% of max_v[{steady_state*max_v}]: {count*sampleRate}s")
            

if __name__ == "__main__":
    main()