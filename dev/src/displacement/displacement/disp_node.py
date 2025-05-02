#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false
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


import sys
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Int16, Float32MultiArray, String, Empty

from br_messages.msg import Position, Point, DisplacementOrder, Command
from message.msg import SensorObstacleList, DisplacementRequest

from config import StratConfig, COLOR
from config.qos import default_profile, latch_profile, br_position_topic_profile

from .disp_manager import DisplacementManager
from .pathfinder import PathFinder, USE_REGULAR_GRID
from .disp_consts import BR_Callback, DspCallback, SIMULATION, AVOIDANCE_UPDATE_INTERVAL, PUBLISH_GRID_INTERVAL

class DisplacementNode(Node):

    def __init__(self):
        super().__init__("DSP")

        self.color = 0

        self.match_ended = False
        
        self._last_update = 0
        self.manager = DisplacementManager(self, self.get_logger())

        self._setup_config()
        if not self.config.enable_static_obstacles:
            self.get_logger().warn("Static obstacles disabled! -- Make sure to enable before a match!")

        self.init_pos = self.config.default_init_zone_index

        # Comm Teensy
        self.pub_teensy_go_to = self.create_publisher(DisplacementOrder, '/br/goTo', latch_profile)
        self.pub_teensy_cmd = self.create_publisher(Command, '/br/command', latch_profile)
        self.pub_teensy_stop = self.create_publisher(Empty, '/br/stop', latch_profile)
        self.pub_teensy_speed = self.create_publisher(Int16, "/br/setSpeed", latch_profile)
        self.pub_teensy_reset = self.create_publisher(Position, '/br/resetPosition', latch_profile)

        self.msg_go_to = DisplacementOrder()
        
        # General subscriptions
        self.color_sub = self.create_subscription(Int16, '/game/color', self.cb_color, default_profile)
        self.init_pos_sub = self.create_subscription(Int16, '/game/init_pos', self.cb_init_pos, default_profile)
        self.end_sub = self.create_subscription(Int16, '/game/end', self.callback_end, default_profile)

        self.sub_teensy = self.create_subscription(Int16,  "/br/callbacks", self.callback_teensy, default_profile) 
        self.sub_pos = self.create_subscription(Position,  "/br/currentPosition", self.callback_position, br_position_topic_profile) 
   
        # Comm Lidar/Sonar
        self.sub_lidar = self.create_subscription(SensorObstacleList, "/sensors/obstaclesLidar", self.callback_lidar, default_profile)
        self.sub_lidar = self.create_subscription(SensorObstacleList, "/sensors/obstaclesSonar", self.callback_sonar, default_profile)
   
        # Comm Strat
        self.pub_strat = self.create_publisher(Int16, "/dsp/callback/next_move", latch_profile)
        self.sub_strat = self.create_subscription(DisplacementRequest, "/dsp/order/next_move", self.callback_strat, default_profile)

        # Obstacles
        self.sub_delete = self.create_subscription(String, "/removeObs", self.callback_delete_obs, default_profile)

        ############################
        #### Pour la Simulation ####
        ############################

        # Comm Simulation            
        if SIMULATION:
            self.pub_grid = self.create_publisher(Float32MultiArray, "/simu/nodegrid", latch_profile)
            self.last_grid_update = 0

        self._setup_init_pos()

        self.get_logger().info("Displacement node initialized")

    def sendPathCommand(self, path, backward, final_theta, allow_curve=True):        
        msg = self.msg_go_to

        msg.path = [Point(x=point.x, y=point.y) for point in path]
        msg.kind = 0
        if backward:
            msg.kind |= DisplacementOrder.REVERSE
        if final_theta is not None:
            msg.kind |= DisplacementOrder.FINAL_ORIENTATION
            msg.theta = final_theta
        if allow_curve:
            msg.kind |= DisplacementOrder.ALLOW_CURVE

        self.pub_teensy_go_to.publish(msg)

    def sendOrientationCommand(self, theta):
        self.msg_go_to.path = []
        self.msg_go_to.kind = DisplacementOrder.FINAL_ORIENTATION
        self.msg_go_to.theta = theta

        self.pub_teensy_go_to.publish(self.msg_go_to)

    def sendStopCommand(self):
        self.pub_teensy_stop.publish(Empty())

    def sendSpeedCommand(self, linear, angular):
        msg = Command()
        msg.linear = linear
        msg.angular = angular

        self.pub_teensy_cmd.publish(msg)

    def sendSetSpeed(self, speed_factor):
        msg = Int16()
        msg.data = speed_factor

        self.pub_teensy_speed.publish(msg)

    def reportPathNotFound(self):
        self.pub_strat.publish(Int16(data=DspCallback.PATH_NOT_FOUND))

    def reportBlocked(self):
        self.pub_strat.publish(Int16(data=DspCallback.DEST_BLOCKED))

    def exit(self):
        self.manager.cancelDisplacement()

    def cb_color(self, msg):
        """
        Callback function from topic /game/color.
        """
        if msg.data not in [0,1]:
            self.get_logger().error(f"Wrong value of color given ({msg.data})...")
            return
        
        self.color = msg.data
        self.get_logger().info("Received color : {}".format(COLOR[self.color]))
        self._setup_config()
        self._setup_init_pos()

    def cb_init_pos(self, msg):
        """
        Callback function from topic /game/init_pos
        """
        if msg.data not in range(self.config.init_zone_count):
            self.get_logger().error(f"Wrong value of init pos given ({msg.data})...")
            return

        self.init_pos = msg.data
        self.get_logger().info("Received init pos : {}".format(msg.data))
        self._setup_init_pos()

    def _setup_config(self):
        self.config = StratConfig(self.color)
        self.manager.setStaticObstacles(self.config.static_obstacles())

    def _setup_init_pos(self):
        """Update la position de départ du robot."""
        x, y, z = self.config.init_zones[self.init_pos]

        self.reset_position(x, y, z)

    def callback_position(self, msg):
        """Update la position actuelle du robot.""" 
        self.manager.setRobotPosition(msg)
        self._update_manager()

    def callback_end(self, msg):
        if msg.data == 1:
            self.manager.cancelDisplacement()
            self.matchEnded = True

    def callback_delete_obs(self, msg):
        self.manager.getPathFinder().remove_obstacle(msg.data)

    def callback_lidar(self, msg):
        """Traitement des msg reçus du lidar etc."""
        self.manager.setLidarObstacles(msg)
        self._update_manager()

    def callback_sonar(self, msg):
        self.manager.setSonarObstacles(msg)
        self._update_manager()

    def _update_manager(self, force=False):
        if force or time.time() - self._last_update > AVOIDANCE_UPDATE_INTERVAL:
            self._last_update = time.time()
            self.manager.update()
            self.publish_grid()

    def callback_teensy(self, msg):
        """Traitement des msg reçus de la teensy."""
        if msg.data == BR_Callback.ERROR_ASSERV:
            self.get_logger().error("BR asserv error")
            self.manager.cancelDisplacement()
            self.pub_strat.publish(Int16(data=DspCallback.ERROR_ASSERV))

        elif msg.data == BR_Callback.OK_ORDER:
            self.manager.cancelDisplacement()
            self.pub_strat.publish(Int16(data=DspCallback.SUCCESS))
        
    def callback_strat(self, msg):
        """Traitement des commandes de la strat."""
        if msg.kind == 0:
            self.manager.cancelDisplacement()
        
        elif msg.kind & DisplacementRequest.MOVE != 0:
            x, y = msg.x, msg.y
            if msg.kind & DisplacementRequest.ORIENTATION != 0:
                theta = msg.theta
            else:
                theta = None
            
            self.get_logger().info(f"Displacement order towards ({x}, {y})")

            backward = msg.kind & DisplacementRequest.BACKWARD != 0
            straight_only = msg.kind & DisplacementRequest.STRAIGHT_ONLY != 0
            self.manager.requestDisplacementTo(Point(x=x, y=y), backward, theta, straight_only)

        elif msg.kind & DisplacementRequest.ORIENTATION != 0:
            self.get_logger().info(f"Orientation order towards theta={msg.theta}")
            self.manager.requestRotation(msg.theta)

        else:
            self.get_logger().error(f"Unrecognized displacement request kind: {msg.kind}")
            self.pub_strat.publish(Int16(data=DspCallback.NOT_RECOGNIZED))

    def reset_position(self, x, y, theta):
        self.manager.cancelDisplacement()

        msg = Position()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        self.pub_teensy_reset.publish(msg)

        self.publish_grid(force=True)

    def publish_grid(self, grid=None, force=False):
        """Publish grid to the interfaceNode."""

        # If we use a regular grid instead of a visibility graph, don't re-publish the grid because it has not changed
        if SIMULATION and (force or not USE_REGULAR_GRID) and time.time() - self.last_grid_update > PUBLISH_GRID_INTERVAL:
            self.last_grid_update = time.time()

            if grid is None: grid = self.manager.getPathFinder().get_grid()
            node_coords = []
            for n in range(len(grid)):
                node_coords.extend(grid[n])
            
            msg = Float32MultiArray()
            msg.data = node_coords
            self.pub_grid.publish(msg)
            self.last_grid_update = time.time()

    def destroy_node(self):
        self.manager.cancelDisplacement()
        super().destroy_node()

#################################################################
#																#
# 							MAIN PROG 							#
#																#
#################################################################

def main():
    rclpy.init(args=sys.argv)
    node = DisplacementNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        node.get_logger().warning("Node forced to terminate")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


#################################################################
if __name__ == '__main__':
    main()