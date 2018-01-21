# This specifies that the neurons of the motor population
# should be monitored. You can see them in the spike train widget
@nrp.NeuronMonitor(nrp.brain.left_half, nrp.spike_recorder)
def monitor_population(t):
    return True
