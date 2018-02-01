import numpy as np
@nrp.MapSpikeSink("motors_down_stage_one", nrp.brain.down_stage_one, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_left_stage_one", nrp.brain.left_stage_one, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_up_stage_one", nrp.brain.up_stage_one, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_right_stage_one", nrp.brain.right_stage_one, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_down_stage_two", nrp.brain.down_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_left_stage_two", nrp.brain.left_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_up_stage_two", nrp.brain.up_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapSpikeSink("motors_right_stage_two", nrp.brain.right_stage_two, nrp.leaky_integrator_alpha)
@nrp.MapRobotPublisher('eye_tilt_pos', Topic('/robot/eye_tilt/pos', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_pan_pos', Topic('/robot/left_eye_pan/pos', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_tilt_vel', Topic('/robot/eye_tilt/vel', std_msgs.msg.Float64))
@nrp.MapRobotPublisher('eye_pan_vel', Topic('/robot/left_eye_pan/vel', std_msgs.msg.Float64))
@nrp.MapRobotSubscriber("joint_state_sub", Topic("/robot/joints", sensor_msgs.msg.JointState))
@nrp.MapRobotSubscriber("shuffle_status_sub", Topic("/group_3/shuffling", std_msgs.msg.Bool))
@nrp.Neuron2Robot()
def center_on_green(t, motors_down_stage_one, motors_left_stage_one, motors_up_stage_one, motors_right_stage_one, 
                    motors_down_stage_two, motors_left_stage_two, motors_up_stage_two, motors_right_stage_two,
                    eye_tilt_pos, eye_pan_pos, eye_tilt_vel, eye_pan_vel, joint_state_sub, shuffle_status_sub):

    stage_two = shuffle_status_sub.value.data if shuffle_status_sub.value is not None else False

    if not stage_two:
        # Stage one: Velocity-controlled motion to green ball
        scaling_factor = 3
        tilt = scaling_factor * (motors_up_stage_one.voltage - motors_down_stage_one.voltage)
        pan = scaling_factor * ( motors_left_stage_one.voltage - motors_right_stage_one.voltage)
        eye_tilt_vel.send_message(std_msgs.msg.Float64(tilt))
        eye_pan_vel.send_message(std_msgs.msg.Float64(pan))

    else:
        # Stage two: Position-controlled motion to red cup
        scaling_factor = 0.02
        joint_names = joint_state_sub.value.name
        joint_positions = joint_state_sub.value.position
        current_tilt = joint_positions[joint_names.index("eye_tilt")]
        current_pan = joint_positions[joint_names.index("left_eye_pan")]

        tilt = current_tilt + scaling_factor * (motors_up_stage_two.voltage - motors_down_stage_two.voltage)
        pan = current_pan + scaling_factor * ( motors_left_stage_two.voltage - motors_right_stage_two.voltage)

        eye_tilt_pos.send_message(std_msgs.msg.Float64(tilt))
        eye_pan_pos.send_message(std_msgs.msg.Float64(pan))
