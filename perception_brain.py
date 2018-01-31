# -*- coding: utf-8 -*-

# pragma: no cover
__author__ = 'Benjamin Alt, Felix Schneider'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np


resolution = 17
n_motors = 4  # down, up, left, right

sensors = sim.Population(resolution * resolution, cellclass=sim.IF_curr_exp())
down_stage_one, up_stage_one, left_stage_one, right_stage_one = [sim.Population(1, cellclass=sim.IF_curr_exp()) for _ in range(n_motors)]
down_stage_two, up_stage_two, left_stage_two, right_stage_two = [sim.Population(1, cellclass=sim.IF_curr_exp()) for _ in range(n_motors)]
center_stage_two = sim.Population(1, cellclass=sim.IF_curr_exp())

indices = np.arange(resolution * resolution).reshape((resolution, resolution))
w = np.linspace(0.5, 1, resolution // 2)
weights = np.tile(w, (1, resolution)).flatten().reshape((len(w) * resolution, 1))
w = np.linspace(1, 0.5, resolution // 2)
weights_inv = np.tile(w, (1, resolution)).flatten().reshape((len(w) * resolution, 1))

"""
# Create gaussian curve with mean 1 
x = np.linspace(-1,1,resolution // 2)
sigma, mu = 0.8, 0.0
weights = np.exp(-( (x-mu)**2 / ( 2.0 * sigma**2 )))
weights = np.tile(w, (1, resolution)).flatten().reshape((len(w) * resolution, 1))
"""

"""
# Barrier approach (square of active neurons around center)
length = (resolution // 2) * resolution
weights = np.zeros(shape=(length, 1))
barrier_idx = int(length * 0.66)
weights[barrier_idx,0] = 1
"""

ones = np.ones(shape=(len(w) * resolution,1))

upper_half = sim.PopulationView(sensors, indices[:resolution // 2].flatten())
lower_half = sim.PopulationView(sensors, indices[resolution - resolution//2:].flatten())
left_half = sim.PopulationView(sensors, indices[:, :resolution // 2].flatten())
right_half = sim.PopulationView(sensors, indices[:, resolution - resolution//2:].flatten())
center_nine = sim.PopulationView(sensors, indices[(resolution//2) - 1 : (resolution//2) + 2, (resolution//2) - 1 : (resolution//2) + 2].flatten())

pro_down_stage_one = sim.Projection(lower_half, down_stage_one, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_up_stage_one = sim.Projection(upper_half, up_stage_one, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_left_stage_one = sim.Projection(left_half, left_stage_one, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_right_stage_one = sim.Projection(right_half, right_stage_one, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))

pro_down_stage_two = sim.Projection(lower_half, down_stage_two, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_up_stage_two = sim.Projection(upper_half, up_stage_two, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_left_stage_two = sim.Projection(left_half, left_stage_two, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_right_stage_two = sim.Projection(right_half, right_stage_two, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_center_stage_two = sim.Projection(center_nine, center_stage_two, sim.AllToAllConnector(),
                        sim.StaticSynapse(weight=np.ones(shape=(9,1))))

circuit = sensors + down_stage_one + up_stage_one + left_stage_one + right_stage_one + down_stage_two + up_stage_two + left_stage_two + right_stage_two + center_stage_two
