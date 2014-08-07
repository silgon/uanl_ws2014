#!/usr/bin/python
# -*- coding: utf-8 -*-
"""Open Loop Control in simple system
"""

from tp1 import *
from numpy.linalg import inv

q_ini = array([-pi/3, 2*pi/3])
p_goal = array([0.5, 0.7])
r = Robot2R()
p_ini = r.termpos(q_ini)
plt.plot([p_ini[0], p_goal[0]], [p_ini[1], p_goal[1]], 'r')
q = q_ini
nIter = 20
dx=(p_goal-p_ini)/nIter
r.draw(q)
for i in range(nIter):
    #TODO
    r.draw(q)
    # dx=J*dq
    # then dq=inv(J)*dx
    dq = dot(inv(r.termjac(q)), dx)
    q = q+dq

plt.show()
