import random
import functools
import threading
import os

import rospy
from gazebo_msgs.srv import GetModelState, SetModelState, \
                            SetModelStateRequest, \
                            DeleteModel, DeleteModelRequest, \
                            SpawnEntity, SpawnEntityRequest
from std_srvs.srv import Trigger, TriggerResponse

from hbp_nrp_excontrol.logs import clientLogger

import thimblerigger_config as tc


def simple_trigger_callback(func):
    """
    Simple decorator to wrap a function that takes no arguments (but self)
    and returns a boolean into a ROS Trigger Service compatible function.

    :param func: The function to turn into a trigger service callback.
    :returns The same function, but it does not throw an error if it receives
             userdata and returns a TriggerResponse instead of a boolean.
             The message of the trigger response will contain
             the called function's name.
    """
    execution_state_msg = {True: "{} finished successfully.".format(func.__name__),
                       False: "{} failed.".format(func.__name__)}

    @functools.wraps(func)
    def as_trigger_callback(self, *args, **kwargs):
        result = func(self)
        return TriggerResponse(result, execution_state_msg[result])

    return as_trigger_callback


class Thimblerigger(object):

    def __init__(self, num_mugs=3, num_shuffles=1, mug_radius=0.1, mug_height=0.15, seed=None):
        """
        Thimblerigger is a game challenge for the NRP simulator.
        There are some mugs on the ground.
        The robot is shown a ball under one of the mugs.
        Then, the mugs are shuffled.
        In the end, the robot has to identify which mug the ball is under.

        :param num_mugs: Number of mugs to spawn.
        :param num_shuffles: Number of permutations to go through when shuffling once.
        :param mug_radius: Radius for one mug.
        :param mug_height: Height of one mug.
        All movements will scale to fit the radius and height parameters.
        :param seed: Seed for the random number generator controlling which mug the ball is under
                     and what shuffling permutations are generated.
        """

        # SDFs to spawn objects
        self.mug_sdf, self.mug_from_mesh = self._load_mug_model()
        self.ball_sdf = tc.ball_sdf_xml

        # Random number generator
        self.rnd = random.Random()
        self.rnd.seed(seed)

        # Mug naming
        self._mug_prefix = "mug"
        self._ball_name = "ball"
        self.mug_order = ["{}{}".format(self._mug_prefix, mug)
                          for mug in range(num_mugs)]

        # Size and animation parameters
        self.num_shuffles = num_shuffles
        self.mug_height = mug_height
        self.mug_radius = mug_radius
        self.lift_height = 2 * self.mug_height
        self.shuffle_displacement = 4 * self.mug_radius
        self.ball_radius = 0.5 * self.mug_radius

        # Get ROS proxies
        self._move_proxy = rospy.ServiceProxy('gazebo/set_model_state',
                                              SetModelState,
                                              persistent=True)
        self._model_state_proxy =  rospy.ServiceProxy('gazebo/get_model_state',
                                                      GetModelState,
                                                      persistent=True)
        self._spawn_proxy = rospy.ServiceProxy('gazebo/spawn_sdf_entity',
                                               SpawnEntity,
                                               persistent=True)
        self._despawn_proxy = rospy.ServiceProxy("gazebo/delete_model",
                                                 DeleteModel,
                                                 persistent=True)

        # Resettable values
        self._ball_visible = None
        self._ball_spawned = None
        self.mug_with_ball = None

        # Spawn mugs and setup resettable values
        self._setup()

        # Create services to control the thimblerigger
        self._offer_services()


    def _offer_services(self):
        """
        Start some ros services for different actions of the thimblerigger.
        The actions are:
        - Resetting the state of the thimblerigger
        - Showing the ball under the mug
        - Hiding the ball under the mug
        - Shuffling

        :returns None.
        """
        reset_service = rospy.Service(tc.thimblerigger_reset_service, Trigger, self.reset)
        show_correct_service = rospy.Service(tc.thimblerigger_show_correct_service, Trigger, self.show_mug_with_ball)
        hide_correct_service = rospy.Service(tc.thimblerigger_hide_correct_service, Trigger, self.hide_ball_under_mug)
        shuffle_service = rospy.Service(tc.thimblerigger_shuffle_service, Trigger, self.shuffle)

    def _setup(self):
        """
        Sets up some internal parameters and spawns the mugs.
        Don't call this from externally.

        :returns None.
        """
        self.reset()
        self._spawn_mugs()

    def _spawn_mugs(self):
        """
        Spawns the mugs in the gazebo environment.
        Will align them along the global x-axis.

        :returns True.
        """
        clientLogger.info("Mug sdf: {}".format(self.mug_sdf))
        clientLogger.info("Spawning {} mugs.".format(len(self.mug_order)))
        for i, mug_name in enumerate(self.mug_order):
            msg = SpawnEntityRequest()
            msg.entity_name = mug_name
            if self.mug_from_mesh:
                msg.entity_xml = self.mug_sdf
                msg.initial_pose.position.x = i * self.shuffle_displacement
                msg.initial_pose.position.y = 0
                msg.initial_pose.position.z = 0.205
                msg.initial_pose.orientation.x = 1
                msg.initial_pose.orientation.y = 0
                msg.initial_pose.orientation.z = 0
                msg.initial_pose.orientation.w = 0
            else:  # Fallback in case 3D model could not be loaded
                msg.entity_xml = self.mug_sdf.format(mug_name=mug_name,
                                                     radius=self.mug_radius,
                                                     length=self.mug_height)
                msg.initial_pose.position.x = i * self.shuffle_displacement
                msg.initial_pose.position.y = 0
                msg.initial_pose.position.z = self.mug_height / 2
            msg.reference_frame = "world"
            self._spawn_proxy(msg)
        return True

    @simple_trigger_callback
    def reset(self):
        """
        Resets the state of the thimblerigger.
        Specifically, it:
        - Hides the ball
        - Chooses a new mug for the ball to be under

        :returns True.
        """
        clientLogger.info("(Re)setting thimblerigger experiment.")
        self._despawn_ball()
        self._hide_ball()

        for mug_name in self.mug_order:
            msg = DeleteModelRequest()
            msg.model_name = mug_name
            self._despawn_proxy(msg)

        self._spawn_mugs()
        self.choose_mug_for_ball()

        return True

    def choose_mug_for_ball(self):
        """
        Chooses a new mug for the ball to be under.

        :returns None.
        """
        clientLogger.info("Choosing random mug for ball.")
        self.mug_with_ball = random.choice(self.mug_order)

    @simple_trigger_callback
    def show_mug_with_ball(self):
        """
        Lifts the mug under which the ball is located up and shows the ball.

        :returns True, if the mug was lifted and the ball is visible.
        """
        clientLogger.info("Showing which mug contains the ball.")
        self._spawn_ball()
        self._show_ball()
        return self._ball_spawned and self._ball_visible

    @simple_trigger_callback
    def hide_ball_under_mug(self):
        """
        Lowers the mug under which the ball is located so the ball is invisible.

        :return True, if the mug was lowered and the ball is invisible.
        """
        clientLogger.info("Hiding ball under mug again.")
        self._hide_ball()
        self._despawn_ball()
        return not self._ball_spawned and not self._ball_visible

    def _spawn_ball(self):
        """
        Spawns the ball under the mug under which it should be, if it
        is not already there.

        :returns None.
        """
        if not self._ball_spawned:
            under_mug_pose = self._model_state_proxy(self.mug_with_ball, 'world')
            msg = SpawnEntityRequest()
            msg.entity_name = self._ball_name
            msg.entity_xml = self.ball_sdf.format(ball_name=msg.entity_name,
                                                  radius=self.ball_radius)
            msg.initial_pose.position = under_mug_pose.pose.position
            msg.initial_pose.position.z = self.ball_radius
            msg.reference_frame = "world"
            self._spawn_proxy(msg)
            self._ball_spawned = True

    def _despawn_ball(self):
        """
        Despawns the ball, if it is there.

        :returns None.
        """
        if self._ball_spawned:
            msg = DeleteModelRequest()
            msg.model_name = self._ball_name
            self._despawn_proxy(msg)
            self._ball_spawned = False

    def _show_ball(self):
        """
        Lifts the mug under which the ball is, if it is not already lifted.

        :returns None.
        """
        if not self._ball_visible:
            self._move_continuously(model_name=self.mug_with_ball,
                                    dz=self.lift_height, smoothness=100)
            self._ball_visible = True

    def _hide_ball(self):
        """
        Lowers the mug under which the ball is, if it is lifted.

        :returns None.
        """
        if self._ball_visible:
            self._move_continuously(model_name=self.mug_with_ball,
                                    dz=-self.lift_height, smoothness=100)
            self._ball_visible = False

    @simple_trigger_callback
    def shuffle(self):
        clientLogger.info("Shuffeling mugs {} times now.".format(self.num_shuffles))

        for _ in range(self.num_shuffles):
            self._shuffle_once(displacement=self.shuffle_displacement)

        return True

    def _shuffle_once(self, displacement):
        """
        Shuffles the mugs in the scene randomly.
        The algorithm works as follows:
        - Generate a random permutation of the current mug order
        - Compute the permutation cycles of the generated permutation
        - For all cycles:
            + Displace a mug to one side
            + Displace the mug at its goal pose, if not already displaced
            + Move into the goal pose

        :returns True.
        """

        # Compute a new permutation
        new_order = random.sample(self.mug_order, len(self.mug_order))

        # Find the cycles in the permutation
        cycles = find_cycles(self.mug_order, new_order)

        def displace(mug, direction):
            """
            Small helper function to move a mug out of the line of mugs.
            The direction indicates whether it should move left or right
            to avoid collisions.
            """
            self._move_continuously(mug, dy=direction * self.shuffle_displacement)
            return direction * -1

        def move_into(mug, pose_to):
            """
            Small helper function to move a mug from the displaced state
            to its goal state.
            """
            current_pose = self._model_state_proxy(mug, 'world')
            dx = pose_to.pose.position.x - current_pose.pose.position.x
            dy = pose_to.pose.position.y - current_pose.pose.position.y
            self._move_continuously(mug, dx=dx)
            self._move_continuously(mug, dy=dy)

        for cycle in cycles:
            # Map to check which mug is already displaced
            displaced = {self.mug_order[idx]: False for idx in cycle}
            direction = 1

            move_infos = []
            # Compute where which mug needs to go
            for i, mug_idx in enumerate(cycle):
                goto_idx = cycle[(i + 1) % len(cycle)]
                goto_mug = self.mug_order[goto_idx]
                pose_to = self._model_state_proxy(goto_mug, 'world')
                move_infos.append((self.mug_order[mug_idx], goto_mug, pose_to))

            # Move all the mugs to the correct position
            for mug_name, goto_mug, pose_to in move_infos:
                if not displaced[mug_name]:
                    direction = displace(mug_name, direction)
                    displaced[mug_name] = True
                if not displaced[goto_mug]:
                    direction = displace(goto_mug, direction)
                    displaced[goto_mug] = True

                move_into(mug_name, pose_to)

        # Update the current order of the mugs
        self.mug_order = new_order
        return True

    def _move_continuously(self, model_name, dx=0., dy=0., dz=0., smoothness=50):
        """
        Moves a gazebo object around in a smooth way in global coordinates.

        :param model_name: The name of the model to move around.
        :param dx, dy, dz: Displacement along the (x,y,z)-axes.
        :param smoothness: How smooth the movement should be.
                           The higher the smoother, the lower the faster.

        :returns None.
        """

        # Smoothness higher than 1 might cause some slight position errors
        # due to floating point errors
        smoothness = max(1, smoothness)
        for i in range(smoothness):
            model_state = self._model_state_proxy(model_name, 'world')
            msg = SetModelStateRequest()
            msg.model_state.model_name = model_name
            msg.model_state.pose = model_state.pose
            msg.model_state.pose.position.x += float(dx) / smoothness
            msg.model_state.pose.position.y += float(dy) / smoothness
            msg.model_state.pose.position.z += float(dz) / smoothness
            msg.model_state.twist = model_state.twist
            msg.model_state.scale = model_state.scale
            msg.model_state.reference_frame = 'world'
            self._move_proxy(msg)

    def _load_mug_model(self):
        clientLogger.info("Loading mug model...")
        gazebo_model_dirs = os.environ["GAZEBO_MODEL_PATH"]
        for model_dir in gazebo_model_dirs.split(":"):
            clientLogger.info("Checking model directory {}".format(model_dir))
            model_dir_contents = os.listdir(model_dir)
            if tc.mug_model_symlink_name in model_dir_contents:
                clientLogger.info("Found symlink {}".format(tc.mug_model_symlink_name))
                sdf_filename = os.path.join(model_dir, tc.mug_model_symlink_name, "model.sdf")
                with open(sdf_filename) as mug_model_file:
                    return mug_model_file.read(), True
        return tc.mug_sdf_xml, False

def find_cycles(a, b):
    """
    Finds the cycles of a permutation needed to move
    from permutation a to permutation b.

    :param a: A list containing some elements
    :param b: A list containing the same elements as a,
              possibly in a diffferent order.

    :returns the list of cycles to go from permutation a to permutation b.
             Only cycles of at least length 2 are added.
    """
    assert sorted(a) == sorted(b)
    mapping = {i: b.index(a[i]) for i in range(len(a))}
    cycles = []
    while len(mapping) > 0:
        cycle = []
        i = next(iter(mapping))
        while i in mapping:
            nxt = mapping.pop(i)
            cycle.append(nxt)
            i = nxt
        if len(cycle) > 1:
            cycles.append(cycle)
    return cycles
