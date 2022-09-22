# -*- coding: utf-8 -*-

'''
Controller class of the GUI node
It links the model, the view and the ROS node together
'''


class Controller():

    def __init__(self, model, view):
        

        self.model = model
        self.view = view



    def test(self):
        print("hello")


    def set_match_state(self, matchState):
        if matchState == self.model.matchState:
            self.view.blinkLED('matchLED', 'r')
            return

        self.model.matchState = matchState
        self.view.setLED('matchLED', 'g')
        self.view.refreshNeeded()


    def set_side(self, side):
        if side == self.model.side:
            self.view.blinkLED('sideLED', 'r')
            return

        self.model.side = side
        self.view.setLED('sideLED', 'g')
        self.view.refreshNeeded()


    def set_robot_pos(self, id, x, y, theta):
        pos_r = self.model.robot_positions[id]

        if pos_r is None:
            print(f"ERROR : robot with id {id} has no initialized position")
            return

        if (x, y, theta) == (pos_r[0], pos_r[1], pos_r[2]):
            return
            
        self.model.set_robot_pos(id, x, y, theta)
        self.view.update_robot_pos(id, x, y, theta)
        self.view.refreshNeeded()