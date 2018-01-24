# Imported Python Transfer Function

# Motors: down, up, left, right
import numpy as np
@nrp.MapSpikeSink("motors_down", nrp.brain.down, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_left", nrp.brain.left, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_up", nrp.brain.up, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_right", nrp.brain.right, nrp.leaky_integrator_alpha)
@nrp.MapRobotPublisher('neck_pitch', Topic('/robot/neck_pitch/vel', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('neck_yaw', Topic('/robot/neck_yaw/vel', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('debug_left', Topic('/debug_left', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('debug_right', Topic('/debug_right', std_msgs.msg.Float64))
@nrp.Neuron2Robot()
def center_on_green(t, motors_down, motors_left, motors_up, motors_right, neck_pitch, neck_yaw, debug_left, debug_right):
    #scaling_factor = 0.001
    scaling_factor = 4
    if t > 5:
        pass
        #neck_pitch.send_message(std_msgs.msg.Float64(1.5*(motors_up.voltage - motors_down.voltage)))
        neck_yaw.send_message(std_msgs.msg.Float64(scaling_factor * (motors_left.voltage - motors_right.voltage)))
    debug_left.send_message(std_msgs.msg.Float64(scaling_factor * motors_left.voltage))
    debug_right.send_message(std_msgs.msg.Float64(scaling_factor * motors_right.voltage))
