# Imported Python Transfer Function

# Motors: down, up, left, right
import numpy as np
@nrp.MapSpikeSink("motors_down", nrp.brain.down, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_left", nrp.brain.left, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_up", nrp.brain.up, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_right", nrp.brain.right, nrp.leaky_integrator_alpha)
@nrp.MapRobotPublisher('eye_tilt', Topic('/robot/eye_tilt/vel', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_pan', Topic('/robot/left_eye_pan/vel', std_msgs.msg.Float64))
@nrp.Neuron2Robot()
def center_on_green(t, motors_down, motors_left, motors_up, motors_right, eye_tilt, eye_pan):
    scaling_factor = 3
    eye_tilt.send_message(std_msgs.msg.Float64(scaling_factor * (motors_up.voltage - motors_down.voltage)))
    eye_pan.send_message(std_msgs.msg.Float64(scaling_factor * ( motors_left.voltage - motors_right.voltage)))
