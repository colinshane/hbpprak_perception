# Imported Python Transfer Function

# Motors: down, up, left, right
import numpy as np
@nrp.MapSpikeSink("motors_down", nrp.brain.down, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_left", nrp.brain.left, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_up", nrp.brain.up, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_right", nrp.brain.right, nrp.leaky_integrator_alpha)
@nrp.MapRobotPublisher('neck_pitch', Topic('/robot/neck_pitch/vel', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('neck_yaw', Topic('/robot/neck_yaw/vel', std_msgs.msg.Float64))
@nrp.Neuron2Robot()
def center_on_green(t, motors_down, motors_left, motors_up, motors_right, neck_pitch, neck_yaw):
    #neck_pitch.send_message(std_msgs.msg.Float64(1.5*(motors_up.voltage - motors_down.voltage)))
    #neck_yaw.send_message(std_msgs.msg.Float64(1.5*(motors_left.voltage - motors_right.voltage)))
    pass
