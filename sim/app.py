from email.mime import base
from mayavi import mlab
from mayavi.tools.figure import figure
from tvtk.api import tvtk
import numpy as np
import random

from referenceframe import ReferenceFrame
from helpermath import *

def plotLine(figure, points, color=(1, 1, 1)):
    """
    Plot line using mayavi and tvtk.
    Args:
        figure (mlab.figure): Mayavi figure for plot.
        points (list): List of points defining trajectory.
        color (tuple): Tuple defining path color, i.e (0, 0, 0).
    """
    line = tvtk.LineSource(points=points) # Can modify line using line.trait_set(points=data)
    line_mapper = tvtk.PolyDataMapper(input_connection=line.output_port)
    p = tvtk.Property(line_width=2, color=color)
    line_actor = tvtk.Actor(mapper=line_mapper, property=p)
    figure.scene.add_actor(line_actor)

def getEndPoint(start_point, length, a0, a1):
    z = start_point[2] + length * np.sin(np.deg2rad(a1))
    base = length * np.cos(np.deg2rad(a1))
    x = start_point[0] + base * np.cos(np.deg2rad(a0))
    y = start_point[1] + base * np.sin(np.deg2rad(a0))
    return [x, y, z]

class Assembly:
    def __init__(self, arms, origin=np.array([0.0, 0.0, 0.0])):
        self.arms = arms # Parent-Child relationship contained in this list
        self.origin = origin
        self.reference_frame = ReferenceFrame()
        self.set_parent_reference_frames()
        self.set_universal_reference_frames()
        self.position_arms()

    def set_parent_reference_frames(self):
        for i in range(1, len(self.arms)):
            self.arms[i].set_parent_reference_frame(self.arms[i-1].reference_frame)
    
    def set_universal_reference_frames(self):
        for arm in self.arms:
            arm.set_universal_reference_frame(self.reference_frame)

    def get_arm_points(self, i):
        if i == 0:
            start_point = self.origin
            end_point = start_point + Quaternion(arms[i].attitude).rotate(np.array([self.arms[i].length, 0, 0]))
        else:
            start_point = self.arms[i-1].end_point
            end_point = start_point + Quaternion(arms[i].attitude).rotate(np.array([self.arms[i].length, 0, 0]))
        return start_point, end_point
    
    def position_arms(self):
        for i in range(0, len(self.arms)):
            start_point, end_point = self.get_arm_points(i)
            self.arms[i].set_start_point(start_point)
            self.arms[i].set_end_point(end_point)
    
    def plot(self, figure):
        self.reference_frame.plot(figure, self.origin)
        for arm in arms:
            arm.plot(figure)

class Arm:
    def __init__(self, length, attitude=np.array([1.0, 0.0, 0.0, 0.0])):
        self.length = length
        self.attitude = attitude
        self.reference_frame = ReferenceFrame()
        self.parent_reference_frame = None
        self.universal_reference_frame = None
    
    def set_start_point(self, start_point):
        self.start_point = start_point
    
    def set_end_point(self, end_point):
        self.end_point = end_point
    
    def set_parent_reference_frame(self, reference_frame):
        self.parent_reference_frame = reference_frame
    
    def set_universal_reference_frame(self, reference_frame):
        self.universal_reference_frame = reference_frame
    
    def set_attitude(self, attitude, local=False):  ### WORKING IN QUATERNIONS ###
        """
        Set current attitude vector.
        [qw, qx, qy, qz]
        Args:
            attitude (Quaternion): Attitude vector to set.
            local (bool): If True, attitude is relative to parent_reference_frame. Else
                          attitude is relative to universal_reference_frame.
        """
        if local: # Convert local (parent_reference_frame) attitude to universal attitude
            # Step 1: Update reference_frame
            i, j, k = self.parent_reference_frame.getIJK()
            self.reference_frame.setIJK(i, j, k)
            R = referenceFrames2rotationMatrix(self.parent_reference_frame, self.universal_reference_frame)
            quaternion = Quaternion(matrix=R)
            attitude = quaternion.rotate(attitude) # Rotate attitude vector so it is in universal_reference_frame
            self.reference_frame.rotate(attitude) # Rotate reference_frame
            # Step 2: Update attitude state
            R = referenceFrames2rotationMatrix(self.reference_frame, self.universal_reference_frame)
            attitude = Quaternion(matrix=R)
        else:
            self.reference_frame.setIJK(np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]))
            self.reference_frame.rotate(attitude)
        self.attitude = np.array(list(attitude))
    
    def plot(self, figure):
        plotLine(figure, np.array([self.start_point, self.end_point]))
        self.reference_frame.plot(figure, self.start_point, scale_factor=0.1)

def get_beta(target_point):
    beta = np.arctan(target_point[1] / target_point[0])
    return beta

def get_base_length(target_point):
    base_length = np.sqrt(target_point[0]**2 + target_point[1]**2)
    return base_length

def get_alpha_1(target_point, base_length):
    alpha_1 = np.arctan(target_point[2] / base_length)
    return alpha_1

def get_alpha_2(target_point, base_length, length):
    isosceles_base = np.sqrt(base_length**2 + target_point[2]**2)
    alpha_2 = np.arccos((isosceles_base / 2) / length)
    return alpha_2

def get_theta_phi(alpha_1, alpha_2):
    theta = alpha_1 + alpha_2
    phi = alpha_1 - alpha_2
    return theta, phi

length = 1.0
arms = [Arm(length), Arm(length)]
assembly = Assembly(arms)

target_point = np.array([random.uniform(0, np.sqrt(2)), random.uniform(0, np.sqrt(2)), random.uniform(0, np.sqrt(2))])

beta = get_beta(target_point)
base_length = get_base_length(target_point)
alpha_1 = get_alpha_1(target_point, base_length)
alpha_2 = get_alpha_2(target_point, base_length, length)
theta, phi = get_theta_phi(alpha_1, alpha_2)

assembly.arms[0].set_attitude(euler2quaternion([0, -theta, beta]))
assembly.arms[1].set_attitude(euler2quaternion([0, -phi, beta]), local=False)

#assembly.arms[0].set_attitude(euler2quaternion([0, np.deg2rad(-60), np.deg2rad(20)]))
#assembly.arms[1].set_attitude(euler2quaternion([0, np.deg2rad(60), 0]), local=True)

assembly.position_arms()

figure = figure()
assembly.plot(figure)

mlab.points3d(target_point[0], target_point[1], target_point[2], resolution=16, scale_factor=0.1, color=(1, 0, 1))
mlab.show()

print(assembly.arms[-1].end_point)
print(np.rad2deg(beta))
print(base_length)
print(alpha_1)
print(alpha_2)
print(np.rad2deg(theta))
print(np.rad2deg(phi))
'''
l0 = 1
l1 = l0
l2 = 0.5 * l0

a0 = 20
p0 = [0, 0, 0]
p1 = getEndPoint(p0, l0, a0, 70)
p2 = getEndPoint(p1, l1, a0, -30)
p3 = getEndPoint(p2, l2, a0, 17)

fig = figure()
rf0 = ReferenceFrame()
rf1 = ReferenceFrame()
rf2 = ReferenceFrame()
plotLine(fig, np.array([p0, p1]))
plotLine(fig, np.array([p1, p2]))
plotLine(fig, np.array([p2, p3]))
rf0.plot(fig, p0, 0.25)
rf1.plot(fig, p1, 0.25)
rf2.plot(fig, p2, 0.25)
mlab.show()

print(p1)
print(p2)
print(p3)
print(np.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2))
print(np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2))
print(np.sqrt((p3[0]-p2[0])**2 + (p3[1]-p2[1])**2 + (p3[2]-p2[2])**2))
'''