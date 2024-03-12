#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy

from geometry_msgs.msg import Pose2D, Quaternion
from std_msgs.msg import Int16, Int16MultiArray, Float32MultiArray

from threading import RLock

from math import cos, sin, atan2
import numpy as np

import tkinter as tk
import configparser as ConfigParser

from signal import signal, SIGINT
from sys import exit

from enum import Enum


def handler(signal_received, frame):
    # Handle any cleanup here
    rospy.loginfo("Received ctrl-c signal, closing interface ...")
    rospy.signal_shutdown(signal_received)
    exit(0)


ScreenUnits = float
PhysicalUnits = float


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


class DoorState(Enum):
    CLOSED = 0
    OPEN = 1
    BLOCKED = 2

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

    @staticmethod
    def load(file):
        reader = ConfigParser.ConfigParser(inline_comment_prefixes=";")
        reader.read(os.path.join(os.path.dirname(__file__), file))

        height = int(reader.get("Robot", "robot_larg"))
        width = int(reader.get("Robot", "robot_long"))

        return Robot(width, height)

    def _draw(self, canvas):
        canvas.delete("robot_doors")      
        # TODO: test + implement DoorState.BLOCKED
        if self._doorState == DoorState.OPEN:
            canvas.draw_line(self._location, (self._location + [
                             self._width, self._height / 2]), rotation=self._rotation, fill="black", width=7, tag="robot_doors")
            canvas.draw_line(self._location, (self._location + [
                             self._width, -self._height / 2]), rotation=self._rotation, fill="black", width=7, tag="robot_doors")

        canvas.delete("robot")
        canvas.draw_rectangle(self._location, self._width, self._height,
                              rotation=self._rotation, fill="red", tag="robot")

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

    def setDoorState(self, new_state):
        self._doorState = DoorState(new_state)
        self.invalidate()


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
            canvas.draw_oval(self._location, 10/InterfaceNode.SCALE,
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

    def __init__(self, color, tag):
        super().__init__()
        self._color = color
        self._tag = tag
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

    def _draw(self, canvas):
        canvas.delete(self._tag)

        for location, delta in self._obstacles:
            canvas.draw_oval(
                location, InterfaceNode.OBSTACLE_PLOT_RADIUS, fill=self._color, tag=self._tag)
            canvas.draw_line(
                location, locaton + InterfaceNode.RATIOV * delta, fill='green', tag=self._tag)


class Path(Drawable):

    PATHWIDTH = 8  # (px)
    PATHARROWLENGTH = 150  # (mm)

    def __init__(self):
        super().__init__()
        self.PATHCIRCLEWIDTH = 25 / InterfaceNode.SCALE  # (mm) - 25 px
        self._startPos = np.zeros(2)
        self._path = []

    def replace(self, new_path):
        self._path = list(new_path)
        self._finalCap = self._path.pop()
        self.invalidate()

    def clear(self):
        self._path.clear()

    def setStartPos(self, new_pos):
        self._startPos[:] = new_pos
        self.invalidate()

    def _draw(self, canvas):
        canvas.delete("path_circle")
        canvas.delete("path_line")

        path = self._path
        l = len(path)//2
        if l == 0:
            return
        print(f"Longueur du path : {2*l}")

        # 1er deplacement
        Path._create_path_circle(canvas, self._startPos)
        Path._create_path_line(canvas, self._startPos,
                               np.array([path[0], path[1]]))

        node1 = np.zeros(2)
        node2 = np.zeros(2)
        # reste des deplacements
        for k in range(l):
            node1[:] = path[2*k:2*k+2]
            if k < l-1:
                node2[:] = path[2*k+2:2*k+4]
                self._create_path_line(canvas, node1, node2)
            self._create_path_circle(canvas, node1)

        # fleche du dernier cap
        node1[:] = path[2*l-2:2*l]
        self._create_path_arrow(canvas, node1, self._finalCap, cache=node2)

    def _create_path_circle(self, canvas, node):
        canvas.draw_oval(node, self.PATHCIRCLEWIDTH,
                         fill='yellow', tag='path_circle')

    def _create_path_line(self, canvas, node1, node2):
        canvas.draw_line(node1, node2, fill='yellow',
                         width=self.PATHWIDTH, tag="path_line")

    def _create_path_arrow(self, canvas, lastNode, finalCap, cache=None):
        if cache is None:
            cache = np.zeros(2)
        cache[:] = [cos(finalCap), sin(finalCap)]
        cache *= self.PATHARROWLENGTH
        cache += lastNode

        canvas.draw_line(lastNode, cache, fill='purple', arrow=tk.LAST, arrowshape=(
            8, 10, 6), width=self.PATHWIDTH*2/3, tag="path_line")


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
            node[:] = msg[2*i:2*i+2]
            canvas.draw_oval(node, 5, fill='grey', tag='grid_circle')

class InterfaceNode:

    BORDER_LEFT = 32  # marge à gauche de l'interface (px)
    BORDER_RIGHT = 32  # marge à droite de l'interface (px)
    BORDER_TOP = 0  # marge en haut de l'interface (px)
    BORDER_BOTTOM = 0  # marge en bas de l'interface (px)

    # SCALE must match the size of 'Background_interface.gif'
    SCALE = 975/3000  # conversion mm en pixels (3m <-> 975px)
    TABLE_HEIGHT = 3000  # (mm)
    TABLE_WIDTH = 2000  # (mm)

    OBSTACLE_PLOT_RADIUS = 180 / SCALE  # (mm) - 180 px
    RATIOV = 1

    def __init__(self, refresh_interval=10):
        rospy.init_node('InterfaceNode')
        rospy.loginfo("Initialisation de l'interface")

        self._fenetre = tk.Tk()
        self._fenetre.title('InterfaceNodePlateauOnly')
        self.refresh_interval = refresh_interval
        self._force_redraw = False

        self._canvas = ScaledCanvas(
            self._fenetre, InterfaceNode.TABLE_WIDTH, InterfaceNode.TABLE_HEIGHT, InterfaceNode.SCALE,
            border_left=InterfaceNode.BORDER_LEFT, border_right=InterfaceNode.BORDER_RIGHT,
            border_top=InterfaceNode.BORDER_TOP, border_bottom=InterfaceNode.BORDER_BOTTOM)

        self._canvas.bind('<Button-3>', self.noteClickPosition)
        self._canvas.bind('<ButtonRelease-3>', self.sendOrder)

        self.image = tk.PhotoImage(file=os.path.join(
            os.path.dirname(__file__), "Background_Interface.gif"))
        self._canvas.draw_image(
            self.image, 0, 0, anchor="nw")
        self._canvas.pack()

        self._robot = Robot.load("../pr_start.ini")
        self._clickMarker = Order(color="yellow", tag="clickPosition")
        self._orderMarker = Order(color="purple", tag="order")
        self._sonarObstacles = Obstacles("blue", "sonars")
        self._lidarObstacles = Obstacles("red", "lidar")
        self._path = Path()
        self._grid = Grid()

        self.lock = RLock()

        # initialisation des suscribers
        self._subStart = rospy.Subscriber("/color", Int16, self.updateColor)

        self._subGrid = rospy.Subscriber(
            "/simu/nodegrid", Float32MultiArray, self.updateGrid)

        self._subPos = rospy.Subscriber(
            "/current_position", Pose2D, self.updateRobotPosition)
        self._subDoors = rospy.Subscriber(
            "/act/callback/doors", Int16, self.updateRobotDoorState)
        self._subObstacles = rospy.Subscriber(
            "/obstaclesInfo", Int16MultiArray, self.updateObstacles)
        self._subPath = rospy.Subscriber(
            "/simu/current_path", Float32MultiArray, self.updatePath)

        self._subOrder = rospy.Subscriber(
            "/nextPositionTeensy", Quaternion, self.updateOrder)

        self._pubOrder = rospy.Publisher(
            "/nextPositionTeensy", Quaternion, queue_size=10, latch=False)

        rospy.loginfo("Fin d'initialisation de l'interface")
        self._fenetre.after(refresh_interval, self.refresh)

    def invalidate_all(self):
        self._force_redraw = True

    def refresh(self, force=False):
        with self.lock:  # verrouille l'acces memoire pendant l'affichage

            force = self._force_redraw

            self._grid.redraw(self._canvas, force=force)

            self._path.redraw(self._canvas, force=force)

            self._lidarObstacles.redraw(self._canvas, force=force)
            self._sonarObstacles.redraw(self._canvas, force=force)

            self._robot.redraw(self._canvas, force=force)
            self._clickMarker.redraw(self._canvas, force=force)
            self._orderMarker.redraw(self._canvas, force=force)

            # TODO: Plot excavation squares
            # Corresponding section from old_gui/interface_plateau.py [lines 377-418] was commented out
            # and was not imported

            self._force_redraw = False

        self._fenetre.after(self.refresh_interval, self.refresh)

    def noteClickPosition(self, evt):
        x, y = self._canvas.screen_to_physical_units(np.array([evt.x, evt.y]))
        self._clickMarker.location = [x, y]
        self._clickMarker.show()

    def sendOrder(self, evt):
        x, y = self._canvas.screen_to_physical_units(np.array([evt.x, evt.y]))
        self._clickMarker.hide()
        rospy.loginfo("click received at " + str(x) + "," + str(y))

        x0, y0 = self._clickMarker.location.tolist()
        self._pubOrder.publish(Quaternion(x0, y0, atan2(y-y0, x-x0), 0))

    def updateOrder(self, msg):
        with self.lock:
            if msg.w in [0, 1, 5, 6, 7, 8, 10, 11]:  # What is this?
                self._orderMarker.location = [msg.x, msg.y]
                self._orderMarker.show()

    def updateRobotPosition(self, msg):
        '''Se fait appeler par un subscriber si c'est avec la simulation 1 robot'''
        with self.lock:
            self._robot.setLocation([msg.x, msg.y], msg.theta)
            self._path.setStartPos(self._robot.location)

    def updateRobotDoorState(self, msg):
        with self.lock:
            print(self._robot._doorState)
            self._robot.setDoorState(msg.data)
            print(self._robot._doorState)

    def updateObstacles(self, msg):
        with self.lock:
            if msg.data[0]:
                obstacles = self._sonarObstacles
            else:
                obstacles = self._lidarObstacles

            obstacles.clear()

            for i in range((msg.layout.dim[0]).size):
                obstacles.addObstacles([msg.data[5*i + j + 1]
                                       for j in range(5)])

    def updatePath(self, msg):
        with self.lock:
            if msg.data is None:
                return
            self._path.replace(msg.data)

    def updateColor(self, msg):
        with self.lock:
            # self.color = msg.data # -- DOESN'T SEEM TO BE USED
            self._path.clear()
            self._lidarObstacles.clear()

    def updateGrid(self, msg):
        with self.lock:
            self._grid.setGrid(msg.data)

    def mainloop(self, n=0):
        self._fenetre.mainloop(n)

if __name__ == '__main__':
    node = InterfaceNode()
    signal(SIGINT, handler)
    node.mainloop()
