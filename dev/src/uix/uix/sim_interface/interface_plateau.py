#!/usr/bin/env python3
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
#
# pyright: reportMissingImports=false

import os
import sys
import threading
from threading import RLock, Thread
import time
import random
from math import cos, sin, atan2
from enum import IntEnum

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, Int16, Float32MultiArray, Empty

import tkinter as tk
from PIL import Image, ImageTk

from message.msg import SensorObstacleList, SensorObstacle, CircleObstacle
from message_utils.geometry import make_absolute

from br_trajectories import getTrajectoryCurves, Point2D, Position2D
from br_messages.msg import Position, Point, DisplacementOrder
from config import RobotConfig, NaiveStratConfig
from config.qos import default_profile, latch_profile, br_position_topic_profile

from .interface_const import *

ScreenUnits = float
PhysicalUnits = float

def circle_segment_collide(center, radius, segment):
    OA = segment[0] - center
    OB = segment[1] - center
    if np.dot(OA, segment[0] - segment[1]) > 0 and np.dot(OB, segment[1] - segment[0]) > 0:
        prod = np.abs(OA[0] * OB[1] - OA[1] * OB[0])
        dist = prod / np.linalg.norm(segment[1] - segment[0])
        return dist < radius
    else:
        return np.minimum(np.linalg.norm(OA), np.linalg.norm(OB)) < radius

class ScaledCanvas:

    _width: PhysicalUnits
    _height: PhysicalUnits

    _border_left: ScreenUnits
    _border_right: ScreenUnits
    _border_top: ScreenUnits
    _border_bottom: ScreenUnits

    def __init__(self, window, width, height, scale, **kwargs):
        """
        Arguments
        ----
        window (tk.Tk)
            parent window
        width, height (float, physical units)
            the physical size of the area to be drawn
        scale (float, pixel-per-physical-unit)
            the scale factor to convert between pixels and physical units

        Additional keyword arguments
        ----
        border_left, border_top, border_right, border_bottom (float, pixels)
            optional padding (in screen PIXELS) to add around the drawing area


        The actual size of the canvas on screen depends on the physical size, the borders and the scale factor.
        """
        self._width = width
        self._height = height
        self._scale = scale
        for border in ["border_left", "border_top", "border_right", "border_bottom"]:
            self.__setattr__("_" + border,  kwargs.get(border, 0))

        self._canvas = tk.Canvas(window, width=self.outer_width, height=self.outer_height,
                                 borderwidth=0, highlightthickness=0)
        self.pack(expand=True)

    @property
    def client_width(self) -> PhysicalUnits:
        return self._width

    @property
    def outer_width(self) -> ScreenUnits:
        return self.client_width * self._scale + self._border_left + self._border_right

    @property
    def client_height(self) -> PhysicalUnits:
        return self._height

    @property
    def outer_height(self) -> ScreenUnits:
        return self.client_height * self._scale + self._border_top + self._border_bottom

    @property
    def scale(self):
        return self._scale

    def setScale(self, scale):
        self._scale = scale

    def physical_to_screen_units(self, coords, in_place=False):
        """
        Converts physical 2D-coordinates to the corresponding location on screen

        Arguments:
        ----
        coords (numpy array of dim [..., 2])
            Can be multi-dimensional for batch conversions
        in_place (bool, optional)
            If True, 'coords' is modified in-place and is not copied (the change is visible to the caller).
            Otherwise, the conversion happens in a newly allocated array.
            Defaults to False.
        """
        if not in_place:
            coords = coords.copy()
        coords[..., 0] = coords[..., 0] * self._scale + self._border_left
        coords[..., 1] = (self._height - coords[..., 1]) * \
            self._scale + self._border_top
        return coords

    def screen_to_physical_units(self, coords, in_place=False):
        """Inverse of physical_to_screen_units"""
        if not in_place:
            coords = coords.copy()
        coords[..., 0] = (coords[..., 0] - self._border_left) / self._scale
        coords[..., 1] = self._height - \
            (coords[..., 1] - self._border_top) / self._scale
        return coords

    def pack(self, *args, **kwargs):
        """See tk.Canvas.pack"""
        self._canvas.pack(*args, **kwargs)

    def bind(self, *args, **kwargs):
        """See tk.Canvas.bind  

        Warning: coordinates in callbacks will be in screen units (PIXELS) and offset by the top/left borders"""
        # TODO provide better interface
        self._canvas.bind(*args, **kwargs)

    def delete(self, *args):
        """See tk.Canvas.delete"""
        self._canvas.delete(*args)

    @staticmethod
    def _rotate(center, point, rotation):
        """Arguments
        ----
        center (numpy array of dim [..., 2])
            2D rotation center. Can be multi-dimensional for batch rotation
        point (numpy array of dim [..., 2] broadcastable with center)
            2D points to rotate
        rotation (float)
            rotation angle (in radians)"""
        point -= center
        x = np.cos(rotation) * point[..., 0] - np.sin(rotation) * point[..., 1]
        point[..., 1] = np.sin(rotation) * point[..., 0] + \
            np.cos(rotation) * point[..., 1]
        point[..., 0] = x
        point += center

    # def draw_regular_polygon(self, center, radius, num_edges, rotation=0, rotation_center=None, **kwargs):
    #     """ TO BE TESTED
    #     Arguments:
    #     ----
    #     center (np array of dim 2, physical units)
    #         batches are not supported here
    #     radius (float, physical units)
    #         distance between the center and the edges of the polygon
    #     num_edges (int)
    #     rotation (float, optional, default to 0)
    #         rotation angle with respect to the default configuration (in radians).
    #     rotation_center (np array of dim 2, optional)
    #         Ignored if 'rotation' = 0. If omitted, defaults to 'center'. 
    #     """
    #     if rotation_center is not None:
    #         kwargs["rotation"] = rotation
    #         kwargs["rotation_center"] = rotation_center
    #         rotation = 0

    #     angles = np.pi / num_edges + 2 * \
    #         np.arange(num_edges) * np.pi / num_edges + rotation
    #     edges = np.zeros((num_edges, 2))
    #     edges[:, 0] = radius * np.cos(angles)
    #     edges[:, 1] = radius * np.sin(angles)
    #     edges += center
    #     self.draw_polygon(edges, **kwargs)

    def draw_rectangle(self, center, width_, height, **kwargs):
        """
        Arguments: 
        ---
        center (numpy array of size 2)
            batches are not supported here
        width_, height (float, physical units)

        Additional keyword arguments will be given as they are to 'tk.Canvas.create_polygon'. All values used
        in those arguments should be in PIXELS.
        """
        edges = np.ones((4, 2)) * center

        dX = width_ / 2
        dY = height / 2

        edges[0] += [-dX, -dY]
        edges[1] += [dX, -dY]
        edges[2] += [dX, dY]
        edges[3] += [-dX, dY]

        self.draw_polygon(edges, **kwargs)

    def draw_line(self, start, end, rotation=0, **kwargs):
        """
        Prints a line between 'start' and 'end' (in physical units).
        'start' and 'end' should be numpy arrays of dim 2 (batches are not supported).
        'rotation' is an optional rotation angle around 'start'

        Additional keyword arguments will be given as they are to 'tk.Canvas.create_line'. All values used
        in those arguments should be in PIXELS.
        """
        if rotation != 0:
            end = end.copy()
            ScaledCanvas._rotate(start, end, rotation)
        start = self.physical_to_screen_units(start)
        end = self.physical_to_screen_units(end)
        self._canvas.create_line(*start.tolist(), *end.tolist(), **kwargs)

    def draw_curve(self, points, **kwargs):
        points = self.physical_to_screen_units(points)
        self._canvas.create_line(*points.flatten().tolist(), **kwargs)

    def draw_polygon(self, edges, rotation=0, rotation_center=None, **kwargs):
        """
        Arguments:
        ----
        edges (numpy array of dim [N, 2], in physical units)
            should be ordered either clockwise or anti-clockwise 
        rotation (float, optional, default to 0)
            optional rotation angle (in radians) for the polygon
        rotation_center (numpy array of dim 2, in physical units, optional)
            if omitted, defaults to the barycenter of the polygon (there is an overhead to compute it)

        Additional keyword arguments will be given as they are to 'tk.Canvas.create_polygon'. All values used
        in those arguments should be in PIXELS.
        """
        if rotation != 0:
            if rotation_center is None:
                rotation_center = edges.sum(axis=0) / edges.shape[0]
            ScaledCanvas._rotate(rotation_center, edges, rotation)

        edges = self.physical_to_screen_units(edges)
        self._canvas.create_polygon(*edges.flatten().tolist(), **kwargs)

    def draw_oval(self, center, r1, r2=None, **kwargs):
        """
        Arguments:
        ----
        center (numpy array of dim 2, in physical units)
            batches are not supported here
        r1 (float, in physical units)
            horizontal radius of the oval
        r2 (float, in physical units, optional)
            vertical radius of the oval. If omitted, defaults to 'r1'
        If 'r1' = 'r2', this draws a circle.
        The oval cannot be rotated.

        Additional keyword arguments will be given as they are to 'tk.Canvas.create_oval'. All values used
        in those arguments should be in PIXELS.
        """
        center = self.physical_to_screen_units(center).tolist()
        if r2 is None:
            r2 = r1
        r1 = r1 * self._scale
        r2 = r2 * self._scale

        self._canvas.create_oval(
            center[0] - r1, center[1] - r2, center[0] + r1, center[1] + r2, **kwargs)

    def draw_image(self, image, x, y, **kwargs):
        """
        Arguments:
        ----
        image (tk.PhotoImage)
            Caveat: the caller is responsible for keeping a reference to the image as long as it is shown
        x, y (float, in screen units (PIXELS))
            location of the top left-hand corner of the image, in PIXELS. Should not be offset by the border.

        Additional keyword arguments will be given as they are to 'tk.Canvas.create_image'. All values used
        in those arguments should be in PIXELS.
        """
        self._canvas.create_image(x + self._border_left, y + self._border_top, image=image, **kwargs)
        self._canvas.update()


class Drawable:
    """This class must be inherited"""

    def __init__(self):
        self.needsRedraw = True

    def invalidate(self):
        """Invalidates the object. This notifies the object that its appearance has changed
        and it should redraw itself, no rendering is done until 'redraw' is called."""
        self.needsRedraw = True

    def _draw(self, canvas):
        """Must be overriden - instructs the object to draw itself on the canvas."""
        raise NotImplementedError("This function must be overriden")

    def redraw(self, canvas, force=False):
        """Asks the object to redraw itself on the canvas.

        Arguments
        ----
        canvas (ScaledCanvas)
        force (bool, optional, default to False)
            if False, the object will redraw only if it has been invalidated"""
        if force or self.needsRedraw:
            self._draw(canvas)
            self.needsRedraw = False


class DoorState(IntEnum):
    CLOSED = 0
    OPEN = 1
    BLOCKED = 2

class ArmState(IntEnum):
    EXTENDED = 1
    RETRACTED = 2
    BLOCKED = 3

class Robot(Drawable):

    _width: PhysicalUnits
    _height: PhysicalUnits

    def __init__(self, width, height):
        super().__init__()
        self._width = width
        self._height = height
        self._location = np.zeros(2)
        self._rotation = 0
        self._doorState = DoorState.CLOSED
        self._leftArmState = ArmState.RETRACTED
        self._rightArmState = ArmState.RETRACTED
        self._plants = []

    @staticmethod
    def load(config):      
        return Robot(config.robot_length, config.robot_width)

    def _draw(self, canvas):
        canvas.delete("robot_doors")
        if self._doorState == DoorState.OPEN:
            canvas.draw_line(self._location, (self._location + [
                             self._width, self._height / 2]), rotation=self._rotation, fill="black", width=7, tag="robot_doors")
            canvas.draw_line(self._location, (self._location + [
                             self._width, -self._height / 2]), rotation=self._rotation, fill="black", width=7, tag="robot_doors")

        canvas.delete("robot_arm")      
        if self._leftArmState == ArmState.EXTENDED:
            canvas.draw_line(self._location, (self._location + [0, self._height]), rotation=self._rotation, fill="blue", width=5, tag="robot_arm")
        if self._rightArmState == ArmState.EXTENDED:
            canvas.draw_line(self._location, (self._location - [0, self._height]), rotation=self._rotation, fill="blue", width=5, tag="robot_arm")

        canvas.delete("robot")
        canvas.draw_rectangle(self._location, self._width, self._height,
                              rotation=self._rotation, fill="blue", tag="robot")

        canvas.delete("robot_direction")
        canvas.draw_line(self._location, (self._location + [self._width/2, 0]),
                         rotation=self._rotation, fill='pink', width=5, tag="robot_direction")

    def setLocation(self, location, rotation):
        self._location[:] = location
        self._rotation = rotation
        self.invalidate()

    @property
    def location(self):
        return self._location

    @property
    def rotation(self):
        return self._rotation

    def setDoorState(self, new_state):
        self._doorState = DoorState(new_state)
        self.invalidate()

    @property
    def doorsOpen(self):
        return self._doorState == DoorState.OPEN

    def setLeftArmState(self, new_state):
        self._leftArmState = ArmState(new_state)
        self.invalidate()

    def setRightArmState(self, new_state):
        self._rightArmState = ArmState(new_state)
        self.invalidate()

    @property
    def doorLine(self):
        line = np.zeros((2,2))
        line[0] = self.location + [self._width / 2, self._height / 2]
        line[1] = self.location + [self._width / 2, -self._height / 2]
        ScaledCanvas._rotate(self.location, line, self._rotation)
        return line

    @property
    def plants(self):
        return len(self._plants)

    def carryPlant(self, plant):
        if self.plants < PLANT_CAPACITY:
            self._plants.append(plant)
        else:
            raise ValueError("Robot cannot carry more plants")

    def potPlants(self):
        for plant in self._plants:
            plant.markInPot()

    def releasePlants(self):        
        a, b = self.doorLine
        pos = (a + b) / 2 + [2 * random.random() - 1, 2 * random.random() - 1]

        while self.plants > 0:
            plant = self._plants.pop()
            plant.setLocation(pos)
            yield plant

class Pot(Drawable):

    def __init__(self, id, location):
        super().__init__()
        self._id = id
        self._location = np.zeros(2)
        self.setLocation(location)

    def _draw(self, canvas):
        self.clear(canvas)
        canvas.draw_oval(self._location, POT_RADIUS, fill="brown", tag=f"pot_{self._id}")

    def clear(self, canvas):
        canvas.delete(f"pot_{self._id}")

    def setLocation(self, new_location):
        self._location[:] = new_location

    @property
    def location(self):
        return self._location

class Plant(Drawable):

    def __init__(self, id, location):
        super().__init__()
        self._id = id
        self._location = np.zeros(2)
        self.setLocation(location)
        self._in_pot = False

    def _draw(self, canvas):
        self.clear(canvas)
        canvas.draw_oval(self._location, PLANT_RADIUS, fill="green" if not self._in_pot else "chartreuse", tag=f"plant_{self._id}")

    def clear(self, canvas):
        canvas.delete(f"plant_{self._id}")

    @property
    def inPot(self):
        return self._in_pot

    def markInPot(self):
        self._in_pot = True

    def setLocation(self, new_location):
        self._location[:] = new_location

    @property
    def location(self):
        return self._location

class Order(Drawable):
    """Marker for the position where the robot was manually asked to go"""

    def __init__(self, color, tag):
        super().__init__()
        self._location = np.zeros(2)
        self._color = color
        self._tag = tag
        self._visible = False

    def _draw(self, canvas):
        canvas.delete(self._tag)
        if self._visible:
            canvas.draw_oval(self._location, 10/canvas.scale,
                             fill=self._color, tag=self._tag)

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, location):
        self._location[:] = location
        self.invalidate()

    def show(self):
        self._visible = True
        self.invalidate()

    def hide(self):
        self._visible = False
        self.invalidate()


class Obstacles(Drawable):

    def __init__(self, color, tag, plot_radius=OBSTACLE_PLOT_RADIUS):
        super().__init__()
        self._color = color
        self._tag = tag
        self._radius = plot_radius
        self._obstacles = []

    def clear(self):
        self._obstacles.clear()
        self.invalidate()

    def addObstacle(self, info):
        x0, y0, _dist, dx, dy = info

        location = np.array([x0, y0])
        delta = np.array([dx, dy])

        self._obstacles.append((location, delta))
        self.invalidate()

    @property
    def plot_radius(self):
        return self._radius

    @plot_radius.setter
    def plot_radius(self, radius):
        self._radius = radius
        self.invalidate()

    def _draw(self, canvas):
        canvas.delete(self._tag)

        for location, delta in self._obstacles:
            canvas.draw_oval(
                location, self._radius, fill=self._color, tag=self._tag)
            canvas.draw_line(
                location, location + RATIOV * delta, fill='green', tag=self._tag)


class Path(Drawable):

    def __init__(self):
        super().__init__()
        self._startPos = np.zeros(3)
        self._path = []
        self._finalCap = None
        self._allowCurve = True

    def replace(self, new_path, finalCap, allowCurve):
        self._path = new_path
        self._finalCap = finalCap
        self._allowCurve = allowCurve
        self.invalidate()

    def clear(self):
        self._path.clear()

    def setStartPos(self, new_pos, rotation):
        self._startPos[:2] = new_pos
        self._startPos[2] = rotation
        self.invalidate()

    def _draw(self, canvas):
        canvas.delete("path_circle")
        canvas.delete("path_line")

        path = self._path
        if len(path) == 0:
            return
        
        # 1er point
        self._create_path_circle(canvas, self._startPos[:2])
        
        # reste des points
        for point in path:
            self._create_path_circle(canvas, np.array([point.x, point.y]))
     
        if self._allowCurve:
            startPos = Position2D(Point2D(*self._startPos[:2]) / 1000., self._startPos[2])
            curves = getTrajectoryCurves(startPos, self._finalCap, [Point2D(point.x, point.y) / 1000. for point in path])
            for curve in curves:
                self._create_path_curve(canvas, curve)
        else:
            node1 = np.zeros(2)
            node2 = np.zeros(2)

            node1[:] = self._startPos[:2]
            for point in path:
                node2[:] = [point.x, point.y]
                self._create_path_line(canvas, node1, node2)
                node1[:] = node2    

        # fleche du dernier cap
        if self._finalCap is not None:
            self._create_path_arrow(canvas, path[-1], self._finalCap)

    def _create_path_circle(self, canvas, node):
        canvas.draw_oval(node, PATHCIRCLEWIDTH / canvas.scale,
                         fill='yellow', tag='path_circle')

    def _create_path_line(self, canvas, node1, node2):
        canvas.draw_line(node1, node2, fill='yellow',
                         width=PATHWIDTH, tag="path_line")

    def _create_path_curve(self, canvas, curve):
        points = np.zeros((10, 2))
        for i in range(points.shape[0]):
            pt = curve.at(i / (points.shape[0] - 1)) 
            points[i] = [pt.x, pt.y]
        canvas.draw_curve(points * 1000., fill="yellow", width=PATHWIDTH, tag="path_line")

    def _create_path_arrow(self, canvas, lastNode, finalCap):
        factor = PATHARROWLENGTH / canvas.scale
        lastNode = np.array([lastNode.x, lastNode.y])
        cache = np.array([cos(finalCap), sin(finalCap)])

        canvas.draw_line(lastNode, lastNode + cache * factor, fill='purple', arrow=tk.LAST, arrowshape=(
            8, 10, 6), width=PATHWIDTH*2/3, tag="path_line")


class Grid(Drawable):

    def __init__(self):
        super().__init__()
        self._grid = []

    def setGrid(self, grid):
        grid = list(grid)
        if grid == self._grid:
            return
        self._grid = grid
        self.invalidate()

    def _draw(self, canvas):
        canvas.delete("grid_circle")

        node = np.zeros(2)
        for i in range(len(self._grid) // 2):
            node[:] = self._grid[2*i:2*i+2]
            canvas.draw_oval(node, 5, fill='grey', tag='grid_circle')

class ControlWindow(tk.Toplevel):
    def __init__(self, node):
        super().__init__(node._fenetre)
        self.title('InterfaceNode - Contrôles')

        self._pubIdle = node.create_publisher(Bool, "/br/idle", latch_profile)
        self._pubColor = node.create_publisher(Int16, "/game/color", latch_profile)
        self._pubStart = node.create_publisher(Int16, "/game/start", latch_profile)
        self._pubInitPos = node.create_publisher(Int16, "/game/init_pos", latch_profile)
        self._pubStrat = node.create_publisher(Int16, "/game/strat", latch_profile)

        config = NaiveStratConfig()
        self._num_init_pos = config.init_zone_count
        self._num_strat = len(config.strat_names)

        self._init_pos = 0
        self._strat = 0

        idle_bt = tk.Button(self, text="BR idle", command=lambda: self._pubIdle.publish(Bool(data=False)))
        idle_bt.grid(column=0, row=0)

        ready_bt = tk.Button(self, text="BR ready", command=lambda: self._pubIdle.publish(Bool(data=True)))
        ready_bt.grid(column=1, row=0)

        home_bt = tk.Button(self, text="Color HOME", command=lambda: self._pubColor.publish(Int16(data=0)))
        home_bt.grid(column=0, row=1)

        away_bt = tk.Button(self, text="Color AWAY", command=lambda: self._pubColor.publish(Int16(data=1)))
        away_bt.grid(column=1, row=1)

        init_pos_bt = tk.Button(self, text="Change init pos", command=self._set_init_pos)
        init_pos_bt.grid(column=0, row=2)
    
        strat_bt = tk.Button(self, text="Change strat", command=self._set_strat)
        strat_bt.grid(column=1, row=2)

        start_bt = tk.Button(self, text="Start match", command=lambda: self._pubStart.publish(Int16(data=1)))
        start_bt.grid(column=0, row=3)

    def _set_init_pos(self):
        self._init_pos += 1
        self._pubInitPos.publish(Int16(data=self._init_pos % self._num_init_pos))

    def _set_strat(self):
        self._strat += 1
        self._pubStrat.publish(Int16(data=self._strat % self._num_strat))

class InterfaceNode(Node):

    def __init__(self, refresh_interval=10):
        super().__init__('InterfaceNode')
        self.get_logger().info("Initialisation de l'interface")

        self._fenetre = tk.Tk()
        self._fenetre.title('InterfaceNodePlateauOnly')
        self.refresh_interval = refresh_interval
        self._force_redraw = False

        self._imageHeight = PREFERRED_WINDOW_HEIGHT
        self._canvas = ScaledCanvas(
            self._fenetre, TABLE_WIDTH, TABLE_HEIGHT, PREFERRED_WINDOW_HEIGHT/TABLE_HEIGHT,
            border_left = BORDER_LEFT, border_right = BORDER_RIGHT,
            border_top = BORDER_TOP, border_bottom = BORDER_BOTTOM)

        self._canvas.bind('<Button-3>', self.noteClickPosition)
        self._canvas.bind('<ButtonRelease-3>', self.sendOrder)
        self._canvas.bind("<Configure>", self._resize)

        self.image_original = Image.open(os.path.join(os.path.dirname(__file__), "Background_Interface.gif"))
        self._canvas.pack()

        self._robot = Robot.load(RobotConfig())
        self._clickMarker = Order(color="yellow", tag="clickPosition")
        self._orderMarker = Order(color="purple", tag="order")
        self._sonarObstacles = Obstacles("purple", "sonars")
        self._lidarObstacles = Obstacles("red", "lidar")
        self._robotObstacle = Obstacles("yellow", tag="opponent") # Dummy obstacle added by the disp node 

        self._path = Path()
        self._grid = Grid()

        self._plants = []
        for cluster in [[698.5,1001.5], [1298.5,1001.5], [698.5,2001.5], [1298.5,2001.5], [498.5,1503], [1498.5,1503]]:            
            x, y = cluster
            for i in range(6):
                dx = (CLUSTER_RADIUS - PLANT_RADIUS) * cos(2*np.pi*i/6)
                dy = (CLUSTER_RADIUS - PLANT_RADIUS) * sin(2*np.pi*i/6)
                self._plants.append(Plant(len(self._plants), [x+dx, y+dy]))
        
        self._pots = []
        for cluster in [[ 612.5, 35], [1387.5, 35], [1965, 1000], [612.5, 2965], [1387.5, 2965], [1965, 2000]]:
            self._pots.append(Pot(len(self._pots), cluster))

        self.lock = RLock()

        # initialisation des suscribers
        self._subStart = self.create_subscription(Int16, "/game/color", self.updateColor, default_profile)

        self._subGrid = self.create_subscription(
            Float32MultiArray, "/simu/nodegrid", self.updateGrid, default_profile)

        self._subPos = self.create_subscription(
            Position, "/br/currentPosition", self.updateRobotPosition, br_position_topic_profile)
        self._subDoors = self.create_subscription(
            Int16, "/act/callback/doors", self.updateRobotDoorState, default_profile)
        self._subLeftArm = self.create_subscription(
            Int16, "/act/callback/left_arm", self.updateRobotLeftArmState, default_profile)
        self._subRightArm = self.create_subscription(
            Int16, "/act/callback/right_arm", self.updateRobotRightArmState, default_profile)
        self._subObstacles = self.create_subscription(
            SensorObstacleList, "/sensors/obstaclesLidar", self.updateObstaclesLidar, default_profile)
        self._subObstacles = self.create_subscription(
            SensorObstacleList, "/sensors/obstaclesSonar", self.updateObstaclesSonar, default_profile)
        self._subRobotObstacle = self.create_subscription(
            CircleObstacle, "/simu/robotObstacle", self.updateRobotObstacle, default_profile)

        self._subOrder = self.create_subscription(
            DisplacementOrder, "/br/goTo", self.updateOrder, default_profile)

        self._deposit_sub = self.create_subscription(Empty, '/simu/deposit_end', self.potsDepositEnd, default_profile)

        self._pubOrder = self.create_publisher(DisplacementOrder, "/br/goTo", default_profile)

        self._controlWindow = ControlWindow(self)
        self.get_logger().info("Fin d'initialisation de l'interface")
        self._fenetre.after(refresh_interval, self.refresh)
        

    def invalidate_all(self):
        self._force_redraw = True

    def refresh(self, force=False):
        with self.lock:  # verrouille l'acces memoire pendant l'affichage

            force = force or self._force_redraw

            self._grid.redraw(self._canvas, force=force)

            self._path.redraw(self._canvas, force=force)

            self._robotObstacle.redraw(self._canvas, force=force)
            self._lidarObstacles.redraw(self._canvas, force=force)
            self._sonarObstacles.redraw(self._canvas, force=force)

            self._robot.redraw(self._canvas, force=force)
            self._clickMarker.redraw(self._canvas, force=force)
            self._orderMarker.redraw(self._canvas, force=force)

            for plant in self._plants:
                plant.redraw(self._canvas, force=force)
            for pot in self._pots:
                pot.redraw(self._canvas, force=force)

            self._force_redraw = False

            if self._robot.doorsOpen:
                doorLine = self._robot.doorLine
                i = 0
                while self._robot.plants < PLANT_CAPACITY and i < len(self._plants):
                    plant = self._plants[i]
                    if not plant.inPot and circle_segment_collide(plant.location, PLANT_RADIUS, doorLine):
                        self._robot.carryPlant(plant)
                        plant.clear(self._canvas)
                        self._plants.pop(i)
                        continue
                    i += 1

                if self._robot.plants:
                    i = 0
                    while i < len(self._pots):
                        pot = self._pots[i]
                        if circle_segment_collide(pot.location, POT_RADIUS, doorLine):
                            self._robot.potPlants()
                            pot.clear(self._canvas)
                            self._pots.pop(i)
                            continue
                        i += 1

        self._fenetre.after(self.refresh_interval, self.refresh)

    def _resize(self, event):
        window_width = event.width - BORDER_LEFT-BORDER_RIGHT 
        window_height = event.height - BORDER_TOP-BORDER_BOTTOM

        scale = min(window_height / TABLE_HEIGHT, window_width / TABLE_WIDTH)

        self._canvas.setScale(scale)
        self._redrawBackground(scale * TABLE_HEIGHT)

    def _redrawBackground(self, window_height):
        image = self.image_original

        w, h = image.size
        if window_height != h:
            image = image.resize((int(w * window_height / h), int(window_height)))

        self.image = ImageTk.PhotoImage(image)

        self._canvas.delete("background")
        self._canvas.draw_image(
            self.image, 0, 0, anchor="nw", tag="background")
        
        self._force_redraw = True

    def noteClickPosition(self, evt):
        x, y = self._canvas.screen_to_physical_units(np.array([evt.x, evt.y]))
        self._clickMarker.location = [x, y]
        self._clickMarker.show()

    def sendOrder(self, evt):
        x, y = self._canvas.screen_to_physical_units(np.array([evt.x, evt.y]))
        self._clickMarker.hide()
        self.get_logger().info("click received at " + str(x) + "," + str(y))

        x0, y0 = self._clickMarker.location.tolist()
        
        msg = DisplacementOrder()
        msg.path = [Point(x=x0, y=y0)]
        msg.theta = atan2(y-y0, x-x0)
        msg.kind = DisplacementOrder.ALLOW_CURVE | DisplacementOrder.FINAL_ORIENTATION

        self._pubOrder.publish(msg)

    def updateOrder(self, msg):
        with self.lock:
            if len(msg.path) > 0:
                self._orderMarker.location = [msg.path[-1].x, msg.path[-1].y]
                self._orderMarker.show()
                 
            self._path.setStartPos(self._robot.location, self._robot.rotation)

            final_cap = msg.theta if msg.kind & DisplacementOrder.FINAL_ORIENTATION != 0 else None
            self._path.replace(msg.path, final_cap, msg.kind & DisplacementOrder.ALLOW_CURVE != 0)

    def updateRobotPosition(self, msg):
        '''Se fait appeler par un subscriber si c'est avec la simulation 1 robot'''
        with self.lock:
            self._robot.setLocation([msg.x, msg.y], msg.theta)

    def updateRobotDoorState(self, msg):
        with self.lock:
            self._robot.setDoorState(msg.data)
    
    def updateRobotLeftArmState(self, msg):
        with self.lock:
            self._robot.setLeftArmState(msg.data)
    
    def updateRobotRightArmState(self, msg):
        with self.lock:
            self._robot.setRightArmState(msg.data)

    def updateObstaclesLidar(self, msg):
        with self.lock:
            self._updateObstacles(self._lidarObstacles, msg)
    
    def updateObstaclesSonar(self, msg):
        with self.lock:
            self._updateObstacles(self._sonarObstacles, msg)

    def _updateObstacles(self, lst, msg): 
        robot_pos = Position(x=self._robot.location[0], y=self._robot.location[1], theta=self._robot.rotation)

        lst.clear()
        for obs in msg.obstacles:
            x_abs, y_abs = make_absolute(robot_pos, obs)               
            lst.addObstacle((x_abs, y_abs, 0, 0, 0))

    def updateRobotObstacle(self, msg):
        with self.lock:
            self._robotObstacle.clear()
            self._robotObstacle.addObstacle([msg.x, msg.y, 0, 0, 0])
            self._robotObstacle.plot_radius = msg.radius

    def updateColor(self, msg):
        with self.lock:
            # self.color = msg.data # -- DOESN'T SEEM TO BE USED
            self._path.clear()
            self._lidarObstacles.clear()

    def updateGrid(self, msg):
        with self.lock:
            self._grid.setGrid(msg.data)

    def potsDepositEnd(self, _msg):
        with self.lock:
            for plant in self._robot.releasePlants():
                plant.invalidate()
                self._plants.append(plant)   

    def _spin(self):
        try:
            rclpy.spin(self)
        except (ExternalShutdownException, KeyboardInterrupt):
            self.get_logger().warning("Node forced to terminate")

    def mainloop(self, n=0):
        thread = Thread(target=self._spin)
        try:
            thread.start()
            self._fenetre.mainloop(n)           
        finally:
            thread.join()

def main():
    rclpy.init(args=sys.argv)
    
    node = InterfaceNode()
    try:
        node.mainloop()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()