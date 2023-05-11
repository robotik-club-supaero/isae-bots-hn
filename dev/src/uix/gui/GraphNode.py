#!/usr/bin/env python
# -*- coding: utf-8 -*-
# plot en live le contenu du topic /logTotale

import os
import roslib
import rospy

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from std_msgs.msg import Float32MultiArray

from math import fmod, pi, cos, sin

from threading import RLock

from signal import signal, SIGINT
from sys import exit

from enum import Enum

# class D(Enum):
#     current_time = 0
#     robotPosX = 1
#     robotPosY = 2
#     robotPosTheta = 3
#     goalPointPosX = 4
#     goalPointPosY = 5
#     goalPointPosTheta = 6
#     trajectoryS = 7
#     goalSpeedLinear = 8
#     goalSpeedAngular = 9
#     asservErrorX = 10
#     asservErrorY = 11
#     commandV = 12
#     commandOmega = 13
#     commandeMotorR = 14
#     commandeMotorL = 15
#     rampSpeed = 16
#     rampState = 17
#     BrState = 18

D = (
    'current_time',
    'robotPosX',
    'robotPosY',
    'robotPosTheta',
    'goalPointPosX',
    'goalPointPosY',
    'goalPointPosTheta',
    'trajectoryS',
    'goalSpeedLinear',
    'goalSpeedAngular',
    'asservErrorX',
    'asservErrorY',
    'commandV',
    'commandOmega',
    'commandeMotorR',
    'commandeMotorL',
    'rampSpeed',
    'rampState',
    'BrState'
)

def handler(signal_received, frame):
    # Handle any cleanup here
    rospy.loginfo("Received ctrl-c signal, abording graphes ...")
    rospy.signal_shutdown(signal_received)
    exit(0)

class GraphNode(pg.GraphicsWindow):


    NBPOINTS = 800
    nbDonnees = 0
    index = 0
    LENTAB = 19  # number of fields ?
    tab = [[] for i in range(LENTAB)]
    lastTime = 0
    isStopped = False

    alpha = 0.15 # asserv parameter

    
    def keyPressEvent(self,e):
        if e.key() == QtCore.Qt.Key_Space:
             self.isStopped = not self.isStopped
    def resetTab(self):
        rospy.loginfo("Reset de l'affichage des graphes")
        self.nbDonnees = 0 # on vide les tableaux
        for i in range(self.LENTAB):
            self.tab[i] = []
        self.setRanges()
    
    def update_donnees(self, msg):
        with self.verrou :
            if self.lastTime > msg.data[0] : # si retour dans le temps ,la base roulante a redemarre
                self.resetTab()
            self.lastTime = msg.data[0]
            if self.nbDonnees < self.NBPOINTS :  # tant que l'on a pas assez de données on ne shift pas
                for i in range(self.LENTAB):
                    self.tab[i].append(msg.data[i])
            else :
                for i in range(self.LENTAB):
                    self.tab[i][:-1] = self.tab[i][1:]   # quand on a assez de données on shift en supprimant la donnée la plus ancienne
                    self.tab[i][-1] = msg.data[i]

            # self.tab[D.index('robotPositionTheta')][-1] = fmod(msg.data[D.robotPosTheta],360)
            # self.tab[15][-1] *= 100  # to see the errors
            # self.tab[16][-1] *= 100
            self.nbDonnees += 1
        
#fonction callback qui update les listes de données (pyqtgraph replot tout automatiquement)
    def refreshgui(self):
        with self.verrou :
            if self.isStopped:
                return
            if len(self.tab[0]) <= 0 :
                return
            for i in self.graphs:
                if i == 0:  # States
                    self.curves[0].setData(self.tab[D.index('current_time')], self.tab[D.index('BrState')])  #stateAsserv #assigne les données aux abscisses
                    self.curves[1].setData(self.tab[D.index('current_time')], self.tab[D.index('rampState')])

                if i == 1:  # Ramp Goal speed
                    self.curves[2].setData(self.tab[D.index('current_time')], self.tab[D.index('rampSpeed')])

                if i == 2:  # HN Goal Speeds
                    self.curves[3].setData(self.tab[D.index('current_time')], self.tab[D.index('goalSpeedLinear')])
                    self.curves[4].setData(self.tab[D.index('current_time')], self.tab[D.index('goalSpeedAngular')])

                if i == 3:  # Trajectory s
                    self.curves[5].setData(self.tab[D.index('current_time')], self.tab[D.index('trajectoryS')])

                if i == 4:  # Point positions
                    self.curves[6].setData(self.tab[D.index('robotPosX')], self.tab[D.index('robotPosY')])
                    self.curves[7].setData(
                        [
                            self.tab[D.index('robotPosX')][j] + self.alpha*cos(self.tab[D.index('robotPosTheta')][j])
                            for j in range(len(self.tab[D.index('robotPosX')]))
                        ],
                        [
                            self.tab[D.index('robotPosY')][j] + self.alpha*cos(self.tab[D.index('robotPosTheta')][j])
                            for j in range(len(self.tab[D.index('robotPosY')]))
                        ]
                        )
                    self.curves[8].setData(self.tab[D.index('goalPointPosX')], self.tab[D.index('goalPointPosY')])
                    self.curves[9].setData(
                        [
                            self.tab[D.index('goalPointPosX')][j] + self.alpha*cos(self.tab[D.index('goalPointPosTheta')][j])
                            for j in range(len(self.tab[D.index('goalPointPosX')]))
                        ],
                        [
                            self.tab[D.index('goalPointPosY')][j] + self.alpha*cos(self.tab[D.index('goalPointPosTheta')][j])
                            for j in range(len(self.tab[D.index('goalPointPosY')]))
                        ]
                        )

                if i == 5:  # Errors
                    self.curves[10].setData(self.tab[D.index('current_time')], self.tab[D.index('asservErrorX')])
                    self.curves[11].setData(self.tab[D.index('current_time')], self.tab[D.index('asservErrorY')])

                if i == 6:  # Asserv commands
                    self.curves[12].setData(self.tab[D.index('current_time')], self.tab[D.index('commandV')])
                    self.curves[13].setData(self.tab[D.index('current_time')], self.tab[D.index('commandOmega')])

                if i == 7:  # Motor commands
                    self.curves[14].setData(self.tab[D.index('current_time')], self.tab[D.index('commandeMotorR')])
                    self.curves[15].setData(self.tab[D.index('current_time')], self.tab[D.index('commandeMotorL')])


    def setRanges(self):
        for i in self.graphs:   #initialisation de tous les graphs
            self.axes[i].getViewBox().enableAutoRange(self.axes[i].getViewBox().XAxis,enable=True)
            if i == 0:
                self.axes[i].setYRange(-1.1,10.1)
            if i == 1:
                self.axes[i].setYRange(-1.1,10.1)
            if i == 2:
                self.axes[i].setYRange(-1.1,10.1)
            if i == 3:
                self.axes[i].setYRange(-0.1,1.1)
            if i == 7:
                self.axes[i].setYRange(-10,360)
            else :  
                self.axes[i].getViewBox().enableAutoRange(self.axes[i].getViewBox().YAxis,enable=True)   

    
    def __init__(self):

        rospy.init_node('GraphNode')
        rospy.loginfo("Initialisation de l'affichage des graphes")

        self.graphs = [k for k in range(8)] #TODO set
        self.app =  QtGui.QApplication([])
        super(pg.GraphicsWindow,self).__init__()
        super(pg.GraphicsWindow,self).setWindowTitle('GraphNode')

        self.axes = [None for i in range(8)] #TODO set nb
        self.curves = [None for i in range(16)] #TODO set nb



        self.axes[0] = self.addPlot(title="BR and Ramp States")
        self.axes[0].addLegend()
        self.axes[0].showGrid(x = True, y = True)
        self.curves[0] = self.axes[0].plot([],[], pen='b', name="BR")
        self.curves[1] = self.axes[0].plot([],[], pen='r', name="Ramp")


        self.axes[1] = self.addPlot(title="Ramp goal speed")
        self.axes[1].showGrid(x = True, y = True)
        self.curves[2] = self.axes[1].plot([],[], pen='r')


        self.axes[2] = self.addPlot(title = "HN goal Speeds")
        self.axes[2].addLegend()
        self.axes[2].showGrid(x = True, y = True)
        self.curves[3] = self.axes[2].plot([],[], pen='b', name="Linear")
        self.curves[4] = self.axes[2].plot([],[], pen='r', name="Angular")


        self.axes[3] = self.addPlot(title="Trajectory s")
        self.curves[5] = self.axes[3].plot([],[])
        self.axes[3].showGrid(x = True, y = True)


        self.nextRow()


        self.axes[4] = self.addPlot(title = "Point positions")
        self.axes[4].addLegend()
        self.axes[4].showGrid(x = True, y = True)
        self.curves[6] = self.axes[4].plot([],[], pen=None, symbol='x', symbolBrush = 'b', symbolPen =None, symbolSize = 10, name = "Robot", )
        self.curves[7] = self.axes[4].plot([],[], pen=None, symbol='x', symbolBrush = 'r', symbolPen =None, symbolSize = 10, name = "Asserv Point")
        self.curves[8] = self.axes[4].plot([],[], pen=None, symbol='o', symbolBrush = 'g', symbolPen =None, symbolSize = 5, name = "Goal Point")
        self.curves[9] = self.axes[4].plot([],[], pen=None, symbol='o', symbolBrush = 'r', symbolPen =None, symbolSize = 5, name = "Asserv Goal")
        self.axes[4].setAspectLocked(True)


        self.axes[5] = self.addPlot(title = "Errors")
        self.axes[5].addLegend()
        self.axes[5].showGrid(x = True, y = True)
        self.curves[10] = self.axes[5].plot([],[], pen = 'b', name = "Error X")
        self.curves[11] = self.axes[5].plot([],[], pen = 'r', name = "Error Y")


        self.axes[6] = self.addPlot(title = "Asserv commands")
        self.axes[6].addLegend()
        self.axes[6].showGrid(x = True, y = True)
        self.curves[12] = self.axes[6].plot([],[], pen = 'b', name = "Command V")
        self.curves[13] = self.axes[6].plot([],[], pen = 'r', name = "Command omega")


        self.axes[7] = self.addPlot(title = "Motor commands")
        self.axes[7].addLegend()
        self.axes[7].showGrid(x = True, y = True)
        self.curves[14] = self.axes[7].plot([],[], pen = 'b', name = "Command R")
        self.curves[15] = self.axes[7].plot([],[], pen = 'r', name = "Command L")


        self.setRanges()
        # initialisation des suscribers
        self.sub_data=rospy.Subscriber("logTotaleArray", Float32MultiArray, self.update_donnees)
        rospy.loginfo("Fin d'initialisation de l'affichage des graphes")
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refreshgui)
        self.timer.setInterval(10)
        self.timer.start(10)
        self.verrou = RLock()
        self.show()
        ## verrou pour multi-thread : le callback ROS s'execute dans un thread différent de l'interface QT, on met un verrou pour éviter que self.tab soit lu et écrit en même temps

       
if __name__ == '__main__':
    node = GraphNode()
    signal(SIGINT, handler)
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        node.app.exec_()
