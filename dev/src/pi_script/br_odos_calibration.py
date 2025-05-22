#! /usr/bin/python
# -*- coding: utf-8 -*-

import sys
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from br_messages.msg import OdosCount, DisplacementOrder

from numpy import mean

class CalibrationNode(Node):
    def __init__(self):
        super().__init__("odos_calib")

        self.odos_count = OdosCount()
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)    
        self.odos_sub = self.create_subscription(OdosCount, "/br/odosCount", self.odos_callback, qos_profile)

    def odos_callback(self, msg):
        self.odos_count = msg

    def odosLigneDroite(self):
        
        fileLigneDroite = open("logLigneDroite.log", "a")

        LR = []
        Units = [] 

        continueInput = 'y'
        keepInput = 'y'

        print("Calibrating LR")

        while continueInput != 'n':

            input("Press enter to start ")
            Li = self.odos_count.left
            Ri = self.odos_count.right
            
            input("Press enter to stop ")
            Lf = self.odos_count.left
            Rf = self.odos_count.right

            dL = Lf - Li
            dR = Rf - Ri + 1

            d = float(input("Enter distance (mm) : "))

            keepInput = input("Keep this try ? [y/n] ")
            if(keepInput != 'n'):
                LR.append(dL/dR)
                Units.append(dL/d)
                fileLigneDroite.write(str((d,Li,Ri,Lf,Rf)) + "\n")

            print("LR : " + str(LR) + ", avg = " + str(mean(LR)))
            print("Units : " + str(Units) + ", avg = " + str(mean(Units)))
            
            continueInput = input("Continue ? [y/n]")

        return mean(LR), mean(Units) 

    def odosRotation(self, LR = 1.):

        fileRotation = open("logRotation.log", "a")

        Ecarts = []

        continueInput = 'y'
        keepInput = 'y'

        print("Calibrating rotation")

        while continueInput != 'n':

            input("Press enter to start ")
            Li = self.odos_count.left
            Ri = self.odos_count.right

            input("Press enter to stop the measure ")
            Lf = self.odos_count.left
            Rf = self.odos_count.right

            dL = Lf - Li
            dR = (Rf - Ri)*LR + 1

            d = float(input("Enter distance (rad) : "))

            keepInput = input("Keep this try ? [y/n] ")
            if(keepInput != 'n'):
                Ecarts.append((dL-dR)/d)
                fileRotation.write(str((d,Li,Ri,Lf,Rf)) + "\n")

            print("Ecarts : " + str(Ecarts) + ", avg = " + str(mean(Ecarts)))
            
            continueInput = input("Continue ? [y/n] ")
            
        return mean(Ecarts)

#################################################################
#                                                               #
#                             Main                              #
#                                                               #
#################################################################

def main():
    rclpy.init(args=sys.argv)

    node = CalibrationNode()
    try:
        th = Thread(target = lambda: rclpy.spin(node))
        th.start()

        (LR,E) = node.odosLigneDroite()
        Ecarts = node.odosRotation(LR)
        print(LR)
        print(E)
        print(Ecarts)

    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
