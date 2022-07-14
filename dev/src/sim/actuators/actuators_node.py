#!/usr/bin/env python
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

import math
import time
import rospy
import random
from std_msgs.msg      import Int16
from geometry_msgs.msg import Pose2D


#################################################################
#                                                               #
#                            UTILS                              #
#                                                               #
#################################################################

_NODENAME_ = "[SIM/ACU]"

## Constants PR

ARM_DEPLOY_TIME = 1
ARM_RETRACT_TIME = 1

READ_TIME = 0.5
READ_FAIL_PROB = 0  # entre 0 et 1

GRAB_TIME = 1
DROP_TIME = 1

## Constants GR
ARM_TAKING = 1
ARM_DEPOSING = 1

#############################

#######################################################################
## DATA to put in .ini file in section [Simu]
EXCAVATION_SQUARE_POS_X = 1850
EXCAVATION_SQUARE_POS_Y = [667.5, 852.5, 1037.5, 1222.5, 1407.5, 1592.5, 1777.5, 1962.5, 2147.5, 2332.5]

Y_ARM_OFFSET = 40

x_threshold = 50
y_threshold = 30
c_threshold = 1.
#######################################################################

HOME = 0
AWAY = 1
NONE = 2


## ACTUATOR Node ######################################################
class ActuatorNode():

	"""Node used during simulations to send callbacks instead of the actuators."""

	def __init__(self):		

		#### Communication - pubs & subs ####
		# Sub a /color pour s'initialiser au set de la couleur
		self.sub_color = rospy.Subscriber("/color", Int16, self.update_color)
		self.sub_pos = rospy.Subscriber("/current_position", Pose2D, self.update_position)
		
		# Simule la reponse du BN sur les resistances
		self.res_sub = rospy.Subscriber('/res_request', Int16, self.res_response)
		self.res_pub = rospy.Publisher("/res_feedback", Int16, queue_size=10, latch=True) 

		# Simule la reponse du BN sur les sorties de bras
		self.arm_sub = rospy.Subscriber('/arm_request', Int16, self.arm_response)
		self.arm_pub = rospy.Publisher("/arm_feedback", Int16, queue_size=10, latch=True)  

		# Simule la reponse du BN sur le grab de la statuette
		self.grab_statue_sub = rospy.Subscriber("/grab_statue_request", Int16, self.grab_statue_response)
		self.grab_statue_pub = rospy.Publisher("/grab_statue_feedback", Int16, queue_size=10)

		self.grab_replic_sub = rospy.Subscriber("/drop_replic_request", Int16, self.grab_replic_response)
		self.grab_replic_pub = rospy.Publisher("/drop_replic_feedback", Int16, queue_size=10)

		self.drop_statue_sub = rospy.Subscriber("/drop_statue_request", Int16, self.drop_statue_response)
		self.drop_statue_pub = rospy.Publisher("/drop_statue_feedback", Int16, queue_size=10)

		# Comm avec l'interface de simulation
		self.square_layout_pub = rospy.Publisher("/simu/squareLayout", Int16, queue_size=10, latch=True)
		self.square_info_pub = rospy.Publisher("/simu/squareInfo", Int16, queue_size=10, latch=True)
        
        ###################
        #                 #
        #       GR        #
        #                 #
        ###################

		self.arm_taking_sub = rospy.Subscriber('/arm_order', Int16, self.take_sample_response)
		self.arm_taking_pub = rospy.Publisher('/arm_feedback', Int16, queue_size=10, latch=True)
		self.arm_deposing_sub = rospy.Subscriber('/arm_order', Int16, self.deposit_sample_response)
		self.arm_deposing_pub = rospy.Publisher('arm_feedback', Int16, queue_size=10, latch=True)

		#### Variables ####

		self.color = 0  							# par défaut
		self.curr_pos = Pose2D(x=0, y=0, theta=0)	# par defaut

		self.info_square = [-1] * 7  				# init undefined
		self.curr_square = 0


		# TODO : en constantes
		self.excavationSquarePos_x = 1870  # TODO : a paramétrer
		self.excavationSquarePos_y = [667.5, 852.5, 1037.5, 1222.5, 1407.5, 1592.5, 1777.5, 1962.5, 2147.5, 2332.5]


		# Publication continue s'il y a besoin
		# while(True):
		# 	time.sleep(0.01)


	###################################################################
	# CALLBACKS
	###################################################################

	def update_color(self, msg):
		"""Callback de couleur."""
		self.color = msg.data

		# Generation aleatoire des carres de fouille
		loginfo(NODE_NAME+"Random generation of excavation squares.")
		self.setRandomSquareLayout()

	def update_position(self, msg):
		"""Callback de position."""
		self.curr_pos = msg


	def setRandomSquareLayout(self):
		"""Generate random distribution of excav squares."""
		
		#easySquaresLayouts = ( (2, c, c), (c, c, 2 ) )
		#hardSquaresLayouts = ( (c, 1-c, 1-c, c), (1-c, c, c, 1-c) )
		pc_1 = [0, 0, 2, 1, 0, 0, 1, 2, 1, 1]
		pc_2 = [2, 0, 0, 0, 1, 1, 0, 1, 1, 2]
		pc_3 = [0, 0, 2, 0, 1, 1, 0, 2, 1, 1]
		pc_4 = [2, 0, 0, 1, 0, 0, 1, 1, 1, 2]
		pc = (pc_1, pc_2, pc_3, pc_4)

		randNb = randrange(4)

		for k in range(7):  # les 7 premiers carrés en commençant par le côté de la couleur
			if not self.color in [HOME,AWAY]:
				raise ValueError
			if self.color == HOME: 
				self.info_square[k] = pc[randNb][k]
			if self.color == AWAY: 
				self.info_square[k] = pc[randNb][-k - 1]
			
		self.square_layout_pub.publish(randNb)  # publication de la layout pour l'interface
		loginfo(NODE_NAME+"Generated distribution n° {} : {}".format(randNb, self.info_square))


	def findCurrentSquare(self):
		"""Finds the square being probed."""

		x_R, y_R, cap = self.curr_pos.x, self.curr_pos.y, self.curr_pos.theta
		
		for k in range(10):
			y_S = EXCAVATION_SQUARE_POS_Y[k] - Y_ARM_OFFSET
			x_S = EXCAVATION_SQUARE_POS_X
			c_S = (1 - 2*self.color) * pi/2

			# Test de localisation dans un carré pour x et y et avec un
			# theta proche de pi/2 si color = 0 et proche de -pi/2 si color = 1
			if abs(y_S-y_R) < y_threshold and abs(x_S-x_R) < x_threshold and abs(cap - c_S) < c_threshold:
				if self.color == HOME:
					self.curr_square = k
				else:
					self.curr_square = 9-k
				return True
		return False


	def res_response(self, msg):
		'''Square info : 0:home | 1:away | 2:cross'''

		# Sleep during estimated reading time of resistance
		sleep(READ_TIME)

		if not self.findCurrentSquare():
			logwarn(NODE_NAME+"Erreur simulée : trop loin pour lire la résistance d'un carré")
			return

		if random() < READ_FAIL_PROB:  # cas d'échec de lecture de résistance
			logwarn(NODE_NAME+"Erreur simulée : erreur de lecture de la résistance du carré {}".format(self.curr_square))
			return
		
		loginfo(NODE_NAME+"Succès simulé pour la lecture de résistance du carré  {}".format(self.curr_square))
		self.res_pub.publish(self.info_square[self.curr_square])
		self.curr_square += 1  # on passe au carré suivant


	def arm_response(self, msg):
		'''Lire et basculer doivent être indépendants car on doit pouvoir lire sans basculer et basculer sans lire'''

		#action = msg.data
		action = 1  # TODO : a voir les ordres des bras
		
		if action == 1:  # déployer le bras
			sleep(ARM_DEPLOY_TIME)

			if not self.findCurrentSquare():
				logwarn(NODE_NAME+"Erreur simulée : trop loin pour basculer un carré")
				# TODO : réponse négative du BN
				return

			self.arm_pub.publish(0)
			self.square_info_pub.publish(10*self.curr_square + 1)
			loginfo(NODE_NAME+"Réponse simulée : abaissement du carré {}".format(self.curr_square))

		elif action == 0:  # rentrer le bras, pas de problème ici a priori # TODO : et s'il y a quand même un problème mdr
			sleep(ARM_RETRACT_TIME)
			self.arm_pub.publish(0)
		
		else:
			rospy.logerr(NODE_NAME+"Action du bras non prise en charge")

	def grab_statue_response(self, msg):
		sleep(GRAB_TIME)
		self.grab_statue_pub.publish(data=1)
		loginfo(NODE_NAME+"Réponse simulée : grab de la statuette.")

	def grab_replic_response(self, msg):
		sleep(GRAB_TIME)
		self.grab_replic_pub.publish(data=1)
		loginfo(NODE_NAME+"Réponse simulée : grab de la replique.")

	def drop_statue_response(self, msg):
		sleep(DROP_TIME)
		self.drop_statue_pub.publish(data=1)
		loginfo(NODE_NAME+"Réponse simulée : drop de la statuette")

        ###################
        #                 #
        #       GR        #
        #                 #
        ###################

	def take_sample_response(self, msg):
		if msg.data in [11, 21, 31, 13, 43]:
			sleep(ARM_TAKING)
			self.arm_taking_pub.publish(data=0)
			loginfo(NODE_NAME_GR+"Réponse simulée : sample récupéré")
    
	def deposit_sample_response(self, msg):
		if msg.data in [12, 14, 22, 24, 32, 34, 42, 44]:
			sleep(ARM_DEPOSING)
			self.arm_deposing_pub.publish(data=0)
			loginfo(NODE_NAME_GR+"Réponse simulée : sample déposée")


#################################################################
#																#
# 							Main 								#
#																#
#################################################################
def main():
	rospy.init_node("pr_bn_response")
	node = ActuatorNode()
	loginfo(NODE_NAME+"Initializing Actuator Node.")
	loginfo(NODE_NAME+"Simulation of BN responses.")
	rospy.spin()


if __name__ == '__main__':
	main()
