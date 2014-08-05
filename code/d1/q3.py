# -*- coding: utf-8 -*-
"""Manipulability Ellipsoid
"""

from tp1 import *
from numpy.linalg import svd

def draw_manipulability_ellipsoid(J, center=None):
    """Draw the manipulability ellipsoid of jacobian J.
    :param J: the jacobian matrix
    :type J: (2,2)-shaped array
    :param center: center of the ellipse
    :type center: (2,)-shaped array
    """

    if center is None:
        center = zeros(2)

    n = 36
    angle = linspace(0, 2*pi, n) 
    sphere = vstack([cos(angle), sin(angle)]).T
    ellipse = zeros((36, 2))
    for i in range(n):
        ellipse[i,:] = center + dot(J, sphere[i,:])
    plt.plot(ellipse[:,0], ellipse[:,1], 'g-')
    plt.axis('equal')


plt.figure()
r = Robot2R()
for q in [(-pi/3, 2*pi/3), (-pi/10, pi/5), (0., 0.)]:
    J = r.termjac(q)
    p = r.termpos(q)
    r.draw(q)
    (U, s, Vh) = svd(J)
    draw_manipulability_ellipsoid(J, p)
    # point for big axis
    majj = s[0]*U[0, 0]+p[0], s[0]*U[1, 0]+p[1]
    # point for big axis
    minn = s[1]*U[0, 1]+p[0], s[1]*U[1, 1]+p[1]
    plt.plot([p[0], majj[0]], [p[1], majj[1]])
    plt.plot([p[0], minn[0]], [p[1], minn[1]])

plt.show()
