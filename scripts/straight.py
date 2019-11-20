#!/usr/bin/env python
import math


def wall(f, sx,sy,sz,ex,ey,ez,stepSize):
    dx = ex-sx
    dy = ey-sy
    dz = ez-sz
    length = math.sqrt(dx*dx+dy*dy)
    for lId in range(int(length/stepSize)):
        for zId in range(int(dz/stepSize)):
            x = sx + dx / length * lId * stepSize
            y = sy + dy / length * lId * stepSize
            z = sz + zId * stepSize
            f.write('{} {} {}\n'.format(x, y , z))

def line(f, sx,sy,sz,ex,ey,ez,stepSize):
    dx = ex-sx
    dy = ey-sy
    dz = ez-sz
    length = math.sqrt(dx*dx+dy*dy+dz*dz)
    for lId in range(int(length/stepSize)):
            x = sx + dx / length * lId * stepSize
            y = sy + dy / length * lId * stepSize
            z = sz + dz / length * lId * stepSize
            f.write('{} {} {}\n'.format(x, y , z))

length = 2000
height = 10
wallDistance = 20
startEndMargin = 20
featureSpace = 0.5
viewpointSpace = 2

structure = open("straight_structure.xyz", "w")
wall(structure,    0,0,           0,    length,0,           height,    featureSpace)
wall(structure,    0,wallDistance,0,    length,wallDistance,height,    featureSpace)
structure.close()

odometry = open("straight_odometry.xyz", "w")
line(odometry, startEndMargin,wallDistance/2,height/2, length-startEndMargin,wallDistance/2,height/2, viewpointSpace)
odometry.close()
