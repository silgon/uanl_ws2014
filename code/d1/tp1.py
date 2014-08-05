# -*- coding: utf-8 -*-
"""Class for Robot 2R
"""

from numpy import *
import matplotlib.pyplot as plt


class Robot2R():
    def __init__(self, lengths=(0.5,0.4)):
        self.lengths = array(lengths)

    @property
    def ndof(self):
        return 2

    def termpos(self, q):
        """End effector position"""
    	assert len(q) == 2
    	lengths = self.lengths
        return array([lengths[0]*cos(q[0]) + lengths[1]*cos(q[0]+q[1]),
                      lengths[0]*sin(q[0]) + lengths[1]*sin(q[0]+q[1])])

    def termjac(self, q):
        """End effector jacobian"""
        lengths = self.lengths
        return array([[-lengths[0]*sin(q[0]) - lengths[1]*sin(q[0]+q[1]),
                       -lengths[1]*sin(q[0]+q[1])],
                      [lengths[0]*cos(q[0]) + lengths[1]*cos(q[0]+q[1]),
                       lengths[1]*cos(q[0]+q[1])]])

    def _mgd(self, q):
        lengths = self.lengths
        return array([[0., 0.],
                      [lengths[0]*cos(q[0]),
                       lengths[0]*sin(q[0])],
                      [lengths[0]*cos(q[0]) + lengths[1]*cos(q[0]+q[1]),
                       lengths[0]*sin(q[0]) + lengths[1]*sin(q[0]+q[1])]])

    def draw(self, q, axes=None):
        points = self._mgd(q)
        if axes is None:
            axes = plt.gca()
            axes.axis('equal')
        axes.plot(points[:, 0], points[:, 1], 'ko-', label='robot')


class AffineTraj():
    def __init__(self, p_start, p_end):
        self.p_start = p_start
        self.p_end = p_end
        self.t_start = 0.
        self.t_end = 1.

    def draw(self):
        plt.plot([self.p_start[0], self.p_end[0]],
                 [self.p_start[1], self.p_end[1]], 'r')
        plt.axis('equal')

    def pos(self, time):
        if time <= self.t_start:
            return self.p_start
        elif time >= self.t_end:
            return self.p_end
        else:
            return ((self.t_end-time)* self.p_start +\
                    (time-self.t_start)* self.p_end)/(self.t_end-self.t_start)
