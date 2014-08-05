# -*- coding: utf-8 -*-

from tp1 import *
from numpy.linalg import inv, pinv, svd

r = Robot2R()
q = array([0, pi])
dp = array([-1e-2, 1e-2])
J = r.termjac(q)
dq = {}


# Classical Inverse 
J_i = inv(J)
dq = dot(J_i, dp)
print "Inverse", dq
# Pseudo-inverse de Moore-Penrose
J_pi = pinv(J)
dq_pi = dot(J_pi, dp)
print "Pseudo-inverse", dq_pi

# DLS
lam = 20.
(U,s,Vh) = svd(J)
J_p = dot(dot(Vh, diag(s/(s**2+lam**2))), U.T)
dq = dot(J_p, dp)
q_f = q+dq
print "DLS", q_f

# Reduced SVD
(U,s,Vh) = svd(J)
J_p = dot(dot(Vh, diag(1./s)), U.T)
dq = dot(J_p, dp)
q_f = q+dq

print "SVD_R",q_f

