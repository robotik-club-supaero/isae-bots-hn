# -*- coding: utf-8 -*-
#     ____                                                  
#    / ___| _   _ _ __   __ _  ___ _ __ ___                 
#    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
#     ___) | |_| | |_) | (_| |  __/ | | (_) |               
#    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
#   ____       _ |_|       _   _ _       ____ _       _     
#  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
#  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
#  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
#  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 

# pyright: reportMissingImports=false

#################################################################
#                                                               #
#                           IMPORTS                             #
#                                                               #
#################################################################

import os, sys, inspect
import time
import smach

from std_msgs.msg      import Empty
from geometry_msgs.msg import Quaternion

# import les states de la SM
from an_sm_states.sm_park import park
from an_sm_states.sm_pickup_plants import pickupPlant
from an_sm_states.sm_pickup_pots import pickupPot
from an_sm_states.sm_deposit_pots import depositPot
from an_sm_states.sm_waiting import waiting

from an_const import *
from an_utils import *
from an_comm import enable_comm, repartitor_pub, disp_pub, stop_teensy_pub

from geometry_msgs.msg import Quaternion, Pose2D

#NOTE to import from parent directory
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
from strat_const import ACTIONS_LIST, ACTION_TRANSITIONS, ActionScore, Action

#################################################################
#                                                               #
#                       SM STATE : SETUP                        #
#                                                               #
#################################################################

class Setup(smach.State):
    """
    STATE MACHINE: setup the SM
    """

    def __init__(self):
        smach.State.__init__(	self, 	
                                outcomes=['start', 'preempted'],
                                input_keys=USERDATA_VAR_LIST,
                                output_keys=USERDATA_VAR_LIST)

    def execute(self, userdata):
        ##############################
        ## VARIABLES INITIALIZATION ##
        ##############################
        
        ## Game param variables
        userdata.start = False
        userdata.color = 0
        userdata.score = [ActionScore.SCORE_INIT.value]
        userdata.nb_actions_done = [0]
        userdata.park = [0] 
        
        
        ## Data about the match
        # userdata.deposit_area = [-1] # Coordonnées de là où on dépose les gâteaux
        # userdata.take_cakes_area = [-1] 	# Coordonnées de la pile de gâteaux qui nous intéressent.
        # userdata.take_cherries_area = [-1] 	# Coordonnées du rack de cerises qui nous intéressent.
        # userdata.pucks_taken = [0]	# Pemet de savoir combien on transporte de palets pour pouvoir savoir comment on récupère les autres
        # userdata.cherries_loaded = [0] # Permet de savoir si le robot transporte des cerises ou non. 0: Non ; 1: Oui 
        # userdata.stage_to_go = [0]
        # userdata.stage_to_deposit = [0]

        ## Callback of subscribers
        userdata.cb_depl = [DspCallback.PENDING]  # result of displacement action. CHECK an_const to see details on cb_depl
        userdata.robot_pos = Pose2D(x=-1, y=-1, theta=-1)  # current position of the robot
        userdata.cb_arm = [-1]    # state of the arm
        userdata.cb_doors = [-1]	# state of the doors
        userdata.cb_clamp = [-1] 	# state of the clamp
        userdata.cb_elevator = [-1] # state of the elevator

        ## Game infos variables
        userdata.next_action = [Action.PENDING]  # action en cours (avec arguments eventuels)
        userdata.next_move = Quaternion(x=-1, y=-1, z=-1, w=-1)
        userdata.error_reaction = [-1]
        userdata.nb_errors = [0]
        userdata.open_clamp = False
        userdata.open_doors = False
        userdata.elevator_zero = True

        ## Enable pubs and subs in pr_an_comm.py
        time.sleep(0.01)
        enable_comm()

        ##############################
        ## WAITING FOR START SIGNAL ##
        ##############################
        log_info('Waiting for match to start ...')

        while not userdata.start:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            time.sleep(0.01)
        
        log_info('Starting match !')
        return 'start'

#################################################################
#                                                               #
#                    SM STATE : REPARTITOR                      #
#                                                               #
#################################################################

class Repartitor(smach.State):
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self):
        smach.State.__init__(	self, 	
                                outcomes=ACTIONS_LIST,
                                input_keys=['nb_actions_done', 'next_action', 'pucks_taken'],
                                output_keys=['nb_actions_done', 'next_action'])

    def execute(self, userdata):
        log_info('[Repartitor] Requesting next action ...')
        repartitor_pub.publish(Empty()) # demande nextAction au DN

        userdata.next_action[0] = Action.PENDING # reset variable prochaine action
        while userdata.next_action[0] == Action.PENDING: # en attente de reponse du DN
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            time.sleep(0.01)

        userdata.nb_actions_done[0] = 0  		     	  # reinitialisation nb etapes
        return ACTIONS_LIST[userdata.next_action[0].value]   # lancement prochaine action  
        
#################################################################
#                                                               #
#                        SM STATE : END                         #
#                                                               #
#################################################################

class End(smach.State):
    """
    STATE MACHINE : Dispatch actions between sm substates.
    """

    def __init__(self):
        smach.State.__init__(	self, 	
                                 outcomes=['end','preempted'],
                                input_keys=[''],
                                output_keys=[''])

    def execute(self, userdata):
        log_info('[End] Killing state machine ...')

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ###########################
        ## STOP RUNNING PROGRAMS ##
        ###########################
        disp_pub.publish(Quaternion(x=0,y=0,z=0,w=-1))			# arrêt PF : w = -1
        stop_teensy_pub.publish(Quaternion(x=0,y=0,z=0,w=2))	# arrêt BR (code w=2 pour le BN)
        return 'end'

#################################################################
#                                                               #
#                        INITIALIZATION                         #
#                                                               #
#################################################################

def init_sm(sm):
    """
    Init state machine with its substates
    """

    with sm:
        # Primary States
        smach.StateMachine.add('SETUP', 
                                 Setup(),
                                transitions={'preempted':'END','start':'REPARTITOR'})
        smach.StateMachine.add('REPARTITOR', 
                                 Repartitor(),
                                transitions=ACTION_TRANSITIONS)
        smach.StateMachine.add('END', 
                                 End(),
                                transitions={'end':'exit all','preempted':'exit preempted'})

        # Specific Action States
        smach.StateMachine.add('PICKUPPLANT', pickupPlant,
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'exit preempted'})
        smach.StateMachine.add('PICKUPPOT', pickupPot,
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'exit preempted'})
        smach.StateMachine.add('DEPOSITPOT', depositPot,
                        transitions={'success':'REPARTITOR','fail':'REPARTITOR','preempted':'exit preempted'})
  
        # Other States
        smach.StateMachine.add('PARK', park,
                                transitions={'preempted':'END','end':'REPARTITOR','fail':'REPARTITOR'})
        smach.StateMachine.add('WAITING', waiting,
                                transitions={'preempted':'END','end':'REPARTITOR'})