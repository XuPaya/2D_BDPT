import matplotlib.pyplot as plt
import numpy as np
h = 50
w = 89
heatmap = np.zeros((h, w), dtype = np.float32)
heatmap_boundary = np.zeros((h, w), dtype = np.float32)
maxabs = -1.0
with open('warpfield_boundary.txt') as f:
    lines = f.readlines()
    for i in np.arange(h):
        for j in np.arange(w):
            line = lines[i * w + j];
            strs = line.split(' ')
            heatmap[h - i - 1, j] = float(strs[0])
            if (maxabs < abs(heatmap[h - i - 1, j])):
                maxabs = abs(heatmap[h - i - 1, j])
with open('warpfield_interior.txt') as f:
    lines = f.readlines()
    for i in np.arange(h):
        for j in np.arange(w):
            line = lines[i * w + j];
            strs = line.split(' ')
            heatmap_boundary[h - i - 1, j] = heatmap[h-i-1, j] - float(strs[0])
            if (maxabs < abs(heatmap_boundary[h - i - 1, j])):
                maxabs = abs(heatmap_boundary[h - i - 1, j])

maxabs = 10
plt.imshow(heatmap, cmap = 'bwr', vmax = maxabs, vmin = -maxabs)
plt.show()
# plt.savefig("warpfield_boundary.png")
