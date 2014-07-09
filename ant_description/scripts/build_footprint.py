#!/usr/bin/env python

import math
import yaml
import argparse

class CircularFootprint:
    def __init__(self, radius, points):
        self._radius = radius
        self._points = points

        if self._points < 2:
            raise Exception('Footprint points must be > 1')

        self._theta_inc = 2*math.pi/(self._points - 1)

        self._footprint = []

    def build(self):
        theta = 0.0
        for _ in range(0, self._points):
            xi = self._radius * math.cos(theta)
            yi = self._radius * math.sin(theta)
            self._footprint.append([xi, yi])

            theta += self._theta_inc

    def save(self, filename):
        footprint_param = {}
        footprint_param['footprint'] = self._footprint
        param_str = yaml.dump(footprint_param)

        try:
            f= open(filename, 'w', 0)
        except IOError:
            raise Exception('Failed to create output file')

        f.write(param_str)

        f.close()

def main():
    parser = argparse.ArgumentParser(description='Circular footprint builder.')
    parser.add_argument('radius', metavar='radius', type=float, help='Footprint radius')
    parser.add_argument('points', metavar='points', type=int, help='Footprint points')
    parser.add_argument('filename', metavar='filename', type=str, help='Output filename')

    args = parser.parse_args()

    cf = CircularFootprint(args.radius, args.points)
    cf.build()
    cf.save(args.filename)

if __name__ == "__main__":
    main()
