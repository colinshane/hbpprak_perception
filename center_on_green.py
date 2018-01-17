# Imported Python Transfer Function
import numpy as np
@nrp.MapSpikeSink("motors", nrp.brain.motors, nrp.leaky_integrator_alpha)
@nrp.MapRobotPublisher('neck_yaw', Topic('/robot/neck_yaw/pos', std_msgs.msg.Float64))
@nrp.Neuron2Robot()
def center_on_green(t, motors, neck_yaw):
    pass
    #neck_yaw.send_message(std_msgs.msg.Float64(30. * motors.voltage))
