from mayavi import mlab
from mayavi.tools.figure import figure
from tvtk.api import tvtk
import numpy as np

from referenceframe import ReferenceFrame

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
#mlab.plot3d(x, y, z, tube_radius=1, colormap='Spectral')
#mlab.show()