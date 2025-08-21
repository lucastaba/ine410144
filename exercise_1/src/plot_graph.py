import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os.path
import csv
import numpy as np

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
    parser.add_argument("-s", type=float, required=True)
    parser.add_argument("-y", type=str, required=True)
    return parser.parse_args()

# data = pd.read_csv(filename)

def main():
    args = parseArguments()
    sampleRate = args.s

    if not checkFileExists(args.csv_file):
        print("file does not exist")
        exit(0)

    if not checkNameInCSV(args.y, args.csv_file)[0]:
        print(f"[{args.y}] not found")
        exit(0)
    
    startTime = 0
    finalTime = (numberOfRows(args.csv_file) - 1)*sampleRate # first row is header
    time_axis = np.arange(startTime, finalTime, sampleRate).tolist()
    data = pd.read_csv(args.csv_file)
    fig, ax = plt.subplots()
    ax.plot(time_axis, data[args.y])
    plt.show()

if __name__ == "__main__":
    main()