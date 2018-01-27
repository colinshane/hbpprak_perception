# -*- coding: utf-8 -*-

# pragma: no cover
__author__ = 'Benjamin Alt, Felix Schneider'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np


resolution = 21
n_motors = 4  # down, up, left, right

sensors = sim.Population(resolution * resolution, cellclass=sim.IF_curr_exp())
down, up, left, right = [sim.Population(1, cellclass=sim.IF_curr_exp()) for _ in range(n_motors)]

indices = np.arange(resolution * resolution).reshape((resolution, resolution))
w = np.linspace(0, 1, resolution // 2)
weights = np.tile(w, (1, resolution)).flatten().reshape((len(w) * resolution, 1))
w = np.linspace(1, 0, resolution // 2)
weights_inv = np.tile(w, (1, resolution)).flatten().reshape((len(w) * resolution, 1))

weights = np.ones(shape=(len(w) * resolution,1))

upper_half = sim.PopulationView(sensors, indices[:resolution // 2].flatten())
lower_half = sim.PopulationView(sensors, indices[resolution - resolution//2:].flatten())
left_half = sim.PopulationView(sensors, indices[:, :resolution // 2].flatten())
right_half = sim.PopulationView(sensors, indices[:, resolution - resolution//2:].flatten())

pro_down = sim.Projection(lower_half, down, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=weights))
pro_up = sim.Projection(upper_half, up, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=weights))
pro_left = sim.Projection(left_half, left, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=weights))
pro_right = sim.Projection(right_half, right, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=weights))

circuit = sensors + down + up + left + right
