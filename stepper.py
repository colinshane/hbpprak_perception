import multiprocessing
import os, sys
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import time

sys.path.append(os.path.join(os.path.dirname(__file__)))
from hbp_nrp_excontrol.logs import clientLogger
from thimblerigger_server import ThimbleriggerChallengeServer

import thimblerigger_config as tc

def run_challenge():
    """
    Step through the stages of the challenge.
    """

    #start_service = tc.thimblerigger_start_service
    #stop_service = tc.thimblerigger_stop_service
    step_service = tc.thimblerigger_step_service

    #rospy.wait_for_service(start_service)
    #rospy.wait_for_service(stop_service)
    rospy.wait_for_service(step_service)

    #clientLogger.info("Making start challenge service call...")
    #start_client = rospy.ServiceProxy(start_service, Trigger)
    #try:
        #res = start_client()
    #except rospy.ServiceException as exc:
        #clientLogger.info("Service did not process request: " + str(exc))

    step_client = rospy.ServiceProxy(step_service, Trigger)
    try:
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        time.sleep(3)
        res = step_client() # Do one "step" in the challenge
        
    except rospy.ServiceException as exc:
        clientLogger.info("Service did not process request: " + str(exc))


class Stepper(object):
    """
    Dummy object for spawning a separate process for stepping through the challenge.
    This is only useful for debugging the step mode. Later, calling the step
    service will be done by the team solving the challenge.
    """
    def __init__(self):
        self.process = None

    def run_async(self):
        clientLogger.info("WARNING: using auto-stepper! Only use this in debug mode!")
        self.process = multiprocessing.Process(target=run_challenge)
        self.process.start()
