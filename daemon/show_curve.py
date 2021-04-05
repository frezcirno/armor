#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import toml
import sys
import os

dir_name = sys.argv[1]

f = open(os.path.join(dir_name, "../config.toml"))
parsed_toml = toml.load(f)
f.close()

line_y = parsed_toml['curve']['dy']
line_x = parsed_toml['curve']['dx']

line_y = sorted(line_y, key=lambda x: x[0])
line_x = sorted(line_x, key=lambda x: x[0])

line_y = np.array(line_y).transpose()
line_x = np.array(line_x).transpose()
dis_data = np.linspace(0, 10000, 100)

figure = plt.figure(dpi=75, figsize=(8, 6))
ax = figure.add_subplot(211)
ax.scatter(line_y[0], line_y[1])
abc = np.polyfit(line_y[0], line_y[1], 2)
ax.plot(dis_data, abc[0] * dis_data ** 2 + abc[1] * dis_data + abc[2], label='Quadratic')
ax.plot(line_y[0], line_y[1], 'r', label='Linear')
ax.plot([0, line_y[0][0]],
        [line_y[1][0] - (line_y[1][1] - line_y[1][0]) / (line_y[0][1] - line_y[0][0]) * line_y[0][0], line_y[1][0]],
        'r')

ax.plot([10000, line_y[0][-1]],
        [line_y[1][-1] + (line_y[1][-2] - line_y[1][-1]) / (line_y[0][-2] - line_y[0][-1]) * (10000-line_y[0][-1]), line_y[1][-1]],
        'r')
ax.legend()
ax.set_xlim([0, 10000])
ax.set_xlabel("z")
ax.set_ylabel("$\Delta$y")
ax.grid()

ax = figure.add_subplot(212)
ax.scatter(line_x[0], line_x[1])

abc = np.polyfit(line_x[0], line_x[1], 2)
ax.plot(dis_data, abc[0] * dis_data ** 2 + abc[1] * dis_data + abc[2], label='Quadratic')

ax.plot(line_x[0], line_x[1], 'r', label='Linear')
ax.plot([0, line_x[0][0]],
        [line_x[1][0] - (line_x[1][1] - line_x[1][0]) / (line_x[0][1] - line_x[0][0]) * line_x[0][0], line_x[1][0]],
        'r')

ax.plot([10000, line_x[0][-1]],
        [line_x[1][-1] + (line_x[1][-2] - line_x[1][-1]) / (line_x[0][-2] - line_x[0][-1]) * 10000, line_x[1][-1]],
        'r')
ax.set_xlim([0, 10000])
ax.set_xlabel("z")
ax.set_ylabel("$\Delta$x")
ax.grid()
ax.legend()
plt.show()
