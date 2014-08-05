#!/usr/bin/python
# coding: utf-8
"""Closed Loop Control
"""

from tp1 import *
from numpy.linalg import inv, norm

q_ini = array([-pi/3,2*pi/3])
p_goal = array([0.5, 0.7])
r = Robot2R()
p_ini = r.termpos(q_ini)

# For every iteration the robot will try to get to p_goal
plt.figure()
plt.title(u"Closed Loop, Inverse with global fixed goal")
nIter = 20
plt.plot([p_ini[0], p_goal[0]], [p_ini[1], p_goal[1]], 'r')
q = q_ini
r.draw(q)
dpmax = 1.01*norm(p_goal - p_ini)/nIter

for i in range(nIter):
    # same operations as in Open-Loop but with a recalculation at every time
    dx=(p_goal-r.termpos(q))/(nIter-i)
    dq=dot(inv(r.termjac(q)),dx)
    q=q+dq
    r.draw(q)

# For every iteration the robot will try to get to p_current_goal
plt.figure()
nIter = 20
plt.title(u"Closed-Loop, Inverse with local dynamic goal")
traj = AffineTraj(p_ini, p_goal)
traj.draw()
q = q_ini
r.draw(q)
time = 0
for i in range(nIter):
    dx = (traj.pos(float(i+1)/nIter)-r.termpos(q))
    dq = dot(inv(r.termjac(q)), dx)
    q = q+dq
    r.draw(q)

# Idem Transposed instead of inverse
plt.figure()
plt.title(u"Closed-Loop, Transposed with local dynamic goal")
nIter = 50
traj = AffineTraj(p_ini, p_goal)
traj.draw()
q = q_ini
r.draw(q)
for i in range(nIter):
    # Error is going to increment since the jacobian matrix is not
    # symetric
    dx = (traj.pos(float(i+1)/nIter)-r.termpos(q))
    dq = dot(r.termjac(q).T, dx)
    q = q+dq
    if not mod(i, 5): # Draw 1 out of 5
        r.draw(q)
plt.show()
