import matplotlib.pyplot as plt
import numpy as np

pairs = []
with open('warp_out_unweighted.txt') as f:
    lines = f.readlines()
    for line in lines:
        strs = line.split(' ')
        pair = []
        for str in strs:
            if str is '-nan(ind)\\n':
                str = '0.0'
            try:
                pair.append(float(str))
            except ValueError:
                pair.append(0.0)
        pairs.append(pair)
points = np.array(pairs)
# plt.plot(points[505:530, 0], points[505:530, 1])
plt.plot(points[:, 0], points[:, 1])
# plt.plot(points[:, 0], points[:, 2])
# plt.plot(points[:, 0], points[:, 3])

x1, y1 = [0.5189, 0.5189], [0.0, 2.405]
x2, y2 = [-0.0269, -0.0269], [0.0, 1.711]
# x3, y3 = [-0.8165, -0.8165], [0.0, -1.835]
x4, y4 = [-0.03956, -0.03956], [0.0, -1.280]
plt.plot(x1, y1, x2, y2, x4, y4, marker = 'o', color = 'r')
# plt.plot(x2, y2, x4, y4, marker = 'o', color = 'r')

# plt.show()
plt.savefig('warp_unweighted.png')
