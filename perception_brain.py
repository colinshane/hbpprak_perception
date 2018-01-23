# -*- coding: utf-8 -*-

# pragma: no cover
__author__ = 'Benjamin Alt, Felix Schneider'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np

resolution = 15
n_motors = 4  # down, up, left, right

upper_half = sim.Population(resolution * (resolution // 2), cellclass=sim.IF_curr_exp())
lower_half = sim.Population(resolution * (resolution // 2), cellclass=sim.IF_curr_exp())
left_half = sim.Population(resolution * (resolution // 2), cellclass=sim.IF_curr_exp())
right_half = sim.Population(resolution * (resolution // 2), cellclass=sim.IF_curr_exp())

down = sim.Population(1, cellclass=sim.IF_curr_exp())
up = sim.Population(1, cellclass=sim.IF_curr_exp())
left = sim.Population(1, cellclass=sim.IF_curr_exp())
right = sim.Population(1, cellclass=sim.IF_curr_exp())

indices = np.arange(resolution * resolution).reshape((resolution, resolution))
w = np.linspace(0, 1, resolution // 2)
weights = np.tile(w, (1, resolution)).flatten().reshape((len(w) * resolution, 1))
w = np.linspace(1, 0, resolution // 2)
weights_inv = np.tile(w, (1, resolution)).flatten().reshape((len(w) * resolution, 1))

pro_down = sim.Projection(lower_half, down, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=weights))
pro_up = sim.Projection(upper_half, up, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=weights_inv))
pro_left = sim.Projection(left_half, left, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=weights_inv))
pro_right = sim.Projection(right_half, right, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=weights_inv))

print "**************************** DEBUG ***************************"
print "weights_inv:"
print weights_inv
print
print "weights:"
print weights
print "**************************** DEBUG ***************************"
