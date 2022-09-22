# -*- coding: utf-8 -*-

'''
Modem class of the GUI node
It contains all the logic and behaviors
'''


class Model():

    def __init__(self):
        

        #### BUTTONS AND STUFF ####

        self.verticalOrientation = True



        #### MATCH VARIABLES ####
        self.matchState = False
        self.side = None

        self.robot_positions = [None]*4  # 4 is the maximum number of robots
        self.nbRobots = 2


        print("Initialized Model")











    def set_robot_pos(self, id, x, y, theta):
        self.robot_positions[0] = x
        self.robot_positions[1] = y
        self.robot_positions[2] = theta