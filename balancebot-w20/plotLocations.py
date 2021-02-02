# -*- coding: utf-8 -*-
"""
Created on Fri Jan 31 12:05:08 2020

@author: niraj
"""

import matplotlib.pyplot as plt
import numpy as np

plt.figure(1)
refx = np.concatenate((np.arange(0,1,.01), np.ones(100), np.arange(1,0,-.01), np.zeros(100)))
refy = np.concatenate((np.zeros(100), np.arange(0,1,.01), np.ones(100), np.arange(1,0,-.01)))
plt.plot(refx, refy, 'r')

xvals = []
yvals = []

firstLine = True
with open("location.csv") as inFile:
    for line in inFile:
        if firstLine:
            firstLine = False
            continue
        sl = line.split()
        xvals.append(float(sl[0].strip(',')))
        yvals.append(float(sl[1].strip(',')))

plt.xlim(-.1,1.1)
plt.ylim(-.1,1.1)
pxvals = np.array(xvals)
pyvals = np.array(yvals)
plt.plot(pxvals, pyvals, 'b')
plt.show()
