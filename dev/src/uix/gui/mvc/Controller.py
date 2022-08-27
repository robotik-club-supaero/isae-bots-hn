# -*- coding: utf-8 -*-

'''
Controller class of the GUI node
It links the model, the view and the ROS node together
'''

class Controller():

    def __init__(self, model, view):
        

        self.model = model
        self.view = view


    def set_match_state(self, matchState):
        if matchState == self.model.matchState:
            self.view.blinkLED('matchLED', 'r')
            return

        self.model.matchState = matchState
        self.view.setLED('matchLED', 'g')


    def set_side(self, side):
        if side == self.model.side:
            self.view.blinkLED('sideLED', 'r')
            return

        self.model.side = side
        self.view.setLED('sideLED', 'g')
