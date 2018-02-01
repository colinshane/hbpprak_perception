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
@nrp.MapRobotPublisher('eye_tilt_pos', Topic('/robot/eye_tilt/pos', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_pan_pos', Topic('/robot/left_eye_pan/pos', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_tilt_vel', Topic('/robot/eye_tilt/vel', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_pan_vel', Topic('/robot/left_eye_pan/vel', std_msgs.msg.Float64))
@nrp.MapRobotSubscriber("eye_tilt_sub", Topic("/robot/eye_tilt/pos", std_msgs.msg.Float64))
@nrp.MapRobotSubscriber("eye_pan_sub", Topic("/robot/left_eye_pan/pos", std_msgs.msg.Float64))
@nrp.MapRobotSubscriber("shuffle_status_sub", Topic("/group_3/shuffling", std_msgs.msg.Bool))
@nrp.Neuron2Robot()
def center_on_green(t, motors_down_stage_one, motors_left_stage_one, motors_up_stage_one, motors_right_stage_one, 
                    motors_down_stage_two, motors_left_stage_two, motors_up_stage_two, motors_right_stage_two, motors_center_stage_two,
                    eye_tilt_pos, eye_pan_pos, eye_tilt_vel, eye_pan_vel, eye_tilt_sub, eye_pan_sub, shuffle_status_sub):

    stage_two = shuffle_status_sub.value.data if shuffle_status_sub.value is not None else False

    if not stage_two:
        # Stage one: Velocity-controlled motion to green ball
        scaling_factor = 3
        tilt = scaling_factor * (motors_up_stage_one.voltage - motors_down_stage_one.voltage)
        pan = scaling_factor * ( motors_left_stage_one.voltage - motors_right_stage_one.voltage)
        eye_tilt_vel.send_message(std_msgs.msg.Float64(tilt))
        eye_pan_vel.send_message(std_msgs.msg.Float64(pan))

    elif eye_tilt_sub.value is not None and eye_pan_sub.value is not None:
        clientLogger.info("Centering on red...")
        # Stage two: Position-controlled motion to red cup
        delta_theta = 0.02 # Radians per control period
        current_tilt = eye_tilt_sub.value.data
        current_pan = eye_pan_sub.value.data
        tilt = current_tilt + delta_theta * (motors_up_stage_two.voltage - motors_down_stage_two.voltage)
        pan = current_pan + delta_theta * ( motors_left_stage_two.voltage - motors_right_stage_two.voltage)
        if tilt - current_tilt #TODO
        eye_tilt_pos.send_message(std_msgs.msg.Float64(tilt))
        eye_pan_pos.send_message(std_msgs.msg.Float64(pan))
