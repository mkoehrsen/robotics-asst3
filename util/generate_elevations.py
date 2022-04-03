#!/usr/bin/env python3
"""
Utility to generate an elevation map to use in the sim. Run this manually
and then copy the output to the "height" field of the ElevationGrid.
"""

import argparse
import random
from functools import reduce

MAX_HEIGHT=1
MAX_DIFF=.25

def random_elev(nbr1, nbr2):
    # Stay within range (0, 1). Bias
    # downwards if neighbors are above .5
    # and upwards otherwise
    nbr_min = min(nbr1, nbr2)
    nbr_max = max(nbr1, nbr2)

    if nbr_min > MAX_HEIGHT - MAX_DIFF:
        elev_range = (MAX_HEIGHT - MAX_DIFF, MAX_HEIGHT)
    elif nbr_max < MAX_DIFF:
        elev_range = (0.0, MAX_DIFF)
    elif nbr_max < MAX_HEIGHT/2:
        elev_range = (nbr_max - MAX_DIFF, nbr_max + MAX_DIFF)
    else:
        elev_range = (nbr_min - MAX_DIFF, nbr_min + MAX_DIFF)

    return random.uniform(*elev_range)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("xdim", type=int)
    parser.add_argument("ydim", type=int)
    args = parser.parse_args()

    elevations = [
        [0 for x in range(args.xdim)]
        for y in range(args.ydim)
    ]

    for x in range(args.xdim):
        for y in range(args.ydim):
            if x in (0, args.xdim-1) or y in (0, args.ydim-1):
                elevations[y][x] = 2
            else:
                elevations[y][x] = random_elev(elevations[y-1][x], elevations[y][x-1])
    
    print(reduce(lambda x1, x2: x1+x2, elevations, []))

main()