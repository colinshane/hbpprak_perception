# Imported Python Transfer Function

# Motors: down, up, left, right
import numpy as np
@nrp.MapSpikeSink("motors_down_stage_one", nrp.brain.down_stage_one, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_left_stage_one", nrp.brain.left_stage_one, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_up_stage_one", nrp.brain.up_stage_one, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_right_stage_one", nrp.brain.right_stage_one, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_down_stage_two", nrp.brain.down_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_left_stage_two", nrp.brain.left_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_up_stage_two", nrp.brain.up_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_right_stage_two", nrp.brain.right_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_center_stage_two", nrp.brain.center_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapRobotPublisher('eye_tilt', Topic('/robot/eye_tilt/vel', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_pan', Topic('/robot/left_eye_pan/vel', std_msgs.msg.Float64))
@nrp.MapRobotSubscriber("shuffle_status_sub", Topic("/group_3/shuffling", std_msgs.msg.Bool))
@nrp.Neuron2Robot()
def center_on_green(t, motors_down_stage_one, motors_left_stage_one, motors_up_stage_one, motors_right_stage_one, 
                    motors_down_stage_two, motors_left_stage_two, motors_up_stage_two, motors_right_stage_two, motors_center_stage_two,
                    eye_tilt, eye_pan, shuffle_status_sub):

    stage_two = shuffle_status_sub.value.data if shuffle_status_sub.value is not None else False

    if not stage_two:
        scaling_factor = 3
        tilt = scaling_factor * (motors_up_stage_one.voltage - motors_down_stage_one.voltage)
        pan = scaling_factor * ( motors_left_stage_one.voltage - motors_right_stage_one.voltage)
    else:
        """
        scaling_factor = 3
        diff_tilt = motors_up_stage_two.voltage - motors_down_stage_two.voltage
        sign_tilt = -1 if diff_tilt < 0 else 1
        tilt = sign_tilt * scaling_factor * diff_tilt * diff_tilt
        diff_pan = motors_left_stage_two.voltage - motors_right_stage_two.voltage
        sign_pan = -1 if diff_pan < 0 else 1
        pan = sign_pan * scaling_factor * diff_pan * diff_pan
        """
        if motors_center_stage_two.voltage > 0.008:
            scaling_factor = 0
            clientLogger.info("Setting vel to 0")
        else:
            scaling_factor = 4
        tilt = scaling_factor * (motors_up_stage_two.voltage - motors_down_stage_two.voltage)
        pan = scaling_factor * ( motors_left_stage_two.voltage - motors_right_stage_two.voltage)
    
    eye_tilt.send_message(std_msgs.msg.Float64(tilt))
    eye_pan.send_message(std_msgs.msg.Float64(pan))
