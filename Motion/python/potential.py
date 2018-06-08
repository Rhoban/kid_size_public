import math
import numpy as np
import matplotlib.pyplot as plt
from sympy import *

# Parameters for viewer
DELTA = 0.8
STEP = 0.05
TARGET = (-0.15, 0.06)
BALL = (0, 0.001)
START = (0.5, -0.1)
MODE = 'scan'
# MODE = 'walk'

# Symbols
x, y = symbols('x, y')
targetX, targetY = symbols('targetX, targetY')
ballX, ballY = symbols('ballX, ballY')
repulsion = symbols('repulsion')

# Distances
dist = sqrt((x-targetX)**2+(y-targetY)**2)
distAvoid = sqrt((x-ballX)**2+(y-ballY)**2)

# Near
score = dist
score *= 1/(distAvoid**(repulsion))

# Far
# score = dist
# score += 0.5/(distAvoid2**(0.3))

# Bad
# score = dist**2
# score += ((1/distAvoid)-(1/0.2))

# Printing code
gx = printing.jscode(score.diff(x).replace(x, 0).replace(y, 0))
gy = printing.jscode(score.diff(y).replace(x, 0).replace(y, 0))
gx = gx.replace('ballX', 'ball.x').replace('ballY', 'ball.y').replace('targetX', 'target.position.x').replace('targetY', 'target.position.y').replace('Math.pow', 'pow').replace('Math.sqrt', 'sqrt')
gy = gy.replace('ballX', 'ball.x').replace('ballY', 'ball.y').replace('targetX', 'target.position.x').replace('targetY', 'target.position.y').replace('Math.pow', 'pow').replace('Math.sqrt', 'sqrt')
print('X = '+gx+';')
print('Y = '+gy+';')

score = score.replace(repulsion, 0.75)

# Preparing equation
score = score.replace(targetX, TARGET[0])
score = score.replace(targetY, TARGET[1])
score = score.replace(ballX, BALL[0])
score = score.replace(ballY, BALL[1])
gx = score.diff(x)
gy = score.diff(y)

gc = 'np.array(['+str(gx)+','+str(gy)+'])'
gc = gc.replace('sqrt', 'math.sqrt').replace('tanh', 'math.tanh')

soa = []

# Computing arrows

if MODE == 'walk':
    x, y = np.array([START[0], START[1]])
    dist = 1
    N = 0
    while dist > STEP and N < 100:
        N += 1
        g = eval(gc)
        g = -g*STEP/np.linalg.norm(g)
        dist = math.sqrt((x-TARGET[0])**2 + (y-TARGET[1])**2)
        soa += [[x, y, g[0], g[1]]]
        x += g[0]
        y += g[1]
else:
    for x in np.arange(-DELTA, DELTA, STEP):
        for y in np.arange(-DELTA, DELTA, STEP):
            g = eval(gc)
            g = -g*STEP/np.linalg.norm(g)
            dx, dy = g
            soa += [[x, y, dx, dy]]

soa = np.array(soa)

# Drawing
X, Y, U, V = zip(*soa)
plt.figure()
ax = plt.gca()

ax.add_artist(plt.Circle(BALL, 0.0475, color='r'))
ax.add_artist(plt.Circle(TARGET, 0.01, color='g'))

ax.quiver(X, Y, U, V, angles='xy', scale_units='xy', scale=1)
ax.set_xlim([-DELTA, DELTA])
ax.set_ylim([-DELTA, DELTA])
plt.axes().set_aspect('equal', 'datalim')

plt.draw()
plt.grid()
plt.show()
