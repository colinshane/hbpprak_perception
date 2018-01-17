# -*- coding: utf-8 -*-

# pragma: no cover
__author__ = 'Benjamin Alt'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np

n_sensors = 100
n_motors = 2

sensors = sim.Population(n_sensors, cellclass=sim.IF_curr_exp())
motors = sim.Population(n_motors, cellclass=sim.IF_curr_exp())
sim.Projection(sensors, motors, sim.AllToAllConnector(),
               sim.StaticSynapse(weight=1.))

circuit = sensors + motors
