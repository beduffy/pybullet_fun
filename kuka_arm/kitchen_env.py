"""
What we did:
todo
1. Depth -> Pointcloud (near far plane and all that stuff)
2.
"""

import sys
import math
import time
from collections import Counter
import pickle

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from matplotlib import colors
from matplotlib.colors import ListedColormap, LinearSegmentedColormap
from matplotlib.font_manager import FontProperties
import pybullet as p
from tqdm import tqdm
import scipy.io
import scipy.stats

from kitchen_utils import rand_cmap  # i hate pycharm
# from .kitchen_utils import rand_cmap


default_quat_orientation = p.getQuaternionFromEuler([0, 0, 0])
quat_orientation = p.getQuaternionFromEuler([0, 0, 3.14 / 2])

all_objects = [
    {'urdf_fn': 'beer_bottle.urdf', 'basePosition': [3.4, 1.5, 1.1], 'dynamic': True},
    {'urdf_fn': 'bowl.urdf', 'basePosition': [3.4, 1.5, 1.1], 'dynamic': True},
    {'urdf_fn': 'plate.urdf', 'basePosition': [3.4, 1, 1.1], 'dynamic': True},
    {'urdf_fn': 'glass.urdf', 'basePosition': [3.5, 1.3, 1.1], 'dynamic': True},
    {'urdf_fn': 'knife.urdf', 'basePosition': [3.6, 1.7, 1.1], 'dynamic': True, 'globalScaling': 0.01},
    {'urdf_fn': 'spoon.urdf', 'basePosition': [3.5, 1.2, 1.1], 'dynamic': True, 'globalScaling': 0.01},
    {'urdf_fn': 'fork.urdf', 'basePosition': [3.4, 1.65, 1.1], 'dynamic': True, 'globalScaling': 1.0},
    {'urdf_fn': 'tissue_box.urdf', 'basePosition': [3.5, 0.7, 1.1], 'dynamic': True, 'globalScaling': 0.016},
    {'urdf_fn': 'tissue_box.urdf', 'basePosition': [3.5, 6, 1.5], 'dynamic': True, 'globalScaling': 0.016},
    {'urdf_fn': 'banana.urdf', 'basePosition': [3.6, 2.3, 1.1], 'dynamic': True},
    {'urdf_fn': 'crisps_chips.urdf', 'basePosition': [3.5, 2.4, 1.1], 'dynamic': True, 'globalScaling': 0.01},
    {'urdf_fn': 'crisps_chips.urdf', 'basePosition': [3.45, 6.35, 1.5], 'dynamic': True, 'globalScaling': 0.01},
    {'urdf_fn': 'TeaCup.urdf', 'basePosition': [3.4, 2, 1.1], 'dynamic': True, 'baseOrientation': quat_orientation, 'globalScaling': 0.06, 'flags': p.URDF_USE_MATERIAL_COLORS_FROM_MTL},
    {'urdf_fn': 'TeaCup.urdf', 'basePosition': [3.6, 5.8, 1.55], 'dynamic': True, 'baseOrientation': quat_orientation, 'globalScaling': 0.06, 'flags': p.URDF_USE_MATERIAL_COLORS_FROM_MTL},
    {'urdf_fn': 'TeaCup.urdf', 'basePosition': [1.5, 5.8, 1.5], 'dynamic': True, 'baseOrientation': quat_orientation, 'globalScaling': 0.06, 'flags': p.URDF_USE_MATERIAL_COLORS_FROM_MTL},
    {'urdf_fn': 'sofa.urdf', 'basePosition': [0.5, 6, 1], 'dynamic': True, 'baseOrientation': quat_orientation},
    {'urdf_fn': 'table.urdf', 'basePosition': [1.5, 6, 1], 'dynamic': True, 'baseOrientation': quat_orientation},
    {'urdf_fn': 'table.urdf', 'basePosition': [3.5, 6, 1], 'dynamic': True, 'baseOrientation': quat_orientation, 'globalScaling': 1.5},
    {'urdf_fn': 'trash_can.urdf', 'basePosition': [3.5, 9, 1], 'dynamic': True, 'baseOrientation': quat_orientation},
    {'urdf_fn': 'trash_can.urdf', 'basePosition': [3.5, 4, 1], 'dynamic': True, 'baseOrientation': quat_orientation},
    {'urdf_fn': 'bed.urdf', 'basePosition': [1.1, 7.8, 1], 'dynamic': True, 'baseOrientation': quat_orientation, 'globalScaling': 1.7},
    {'urdf_fn': 'nordic_floor_lamp_obj.urdf', 'basePosition': [1.5, 9, 1.3], 'dynamic': True, 'baseOrientation': quat_orientation, 'globalScaling': 0.01},
    {'urdf_fn': 'kitchen_walls.urdf', 'dynamic': False},  # todo some link segmentation is black because https://github.com/bulletphysics/bullet3/issues/1631
    {'urdf_fn': 'kitchen.urdf', 'basePosition': [0.1, 0.15, 0.1], 'dynamic': False}
]

def load_all_urdfs():
    p.loadURDF("plane.urdf")

    all_obj_names_counter = Counter()
    for idx, object_spec in enumerate(all_objects):
        print('Loading urdf object with specification: {}'.format(object_spec))
        obj_id = p.loadURDF(object_spec['urdf_fn'], basePosition=object_spec.get('basePosition', [0, 0, 0]),
                            globalScaling=object_spec.get('globalScaling', 1.0),
                            baseOrientation=object_spec.get('baseOrientation', default_quat_orientation),
                            flags=object_spec.get('flags', 0)
                            )
        all_objects[idx]['obj_id'] = obj_id
        canonical_name = object_spec['urdf_fn'][:-5]
        all_obj_names_counter.update([canonical_name])
        all_objects[idx]['obj_name'] = '{}_{}'.format(canonical_name,
                                                      all_obj_names_counter['canonical_name'])


    all_obj_ids = [obj['obj_id'] for obj in all_objects]
    obj_id_to_obj_name = {obj['obj_id']: obj['obj_name'] for obj in all_objects}
    obj_name_to_obj_id = {obj['obj_name']: obj['obj_id'] for obj in all_objects}
    print(obj_id_to_obj_name)
    return obj_id_to_obj_name, obj_name_to_obj_id

# todo attempt to make kitchen static while still keeping cupboards openable
# kitchen = p.loadURDF("kitchen.urdf")
# # p.loadURDF("IAI_kitchen.urdf", basePosition=[0, 0, 0.2])
# numJoints = p.getNumJoints(kitchen)
# for jointIndex in range(numJoints):
#     #p.resetJointState(kitchen, jointIndex, jointPositions[jointIndex])
#     print(p.getJointInfo(kitchen, jointIndex))
#     print(p.getDynamicsInfo(kitchen, jointIndex))
#     # p.changeDynamics(kitchen, jointIndex, mass=1000000) # try mass 0 for the heavy shit in the file
#     # print(p.getDynamicsInfo(kitchen, jointIndex))
#

def getRayFromTo(mouseX, mouseY):
    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
    camPos = [camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
              camTarget[2] - dist * camForward[2]]
    farPlane = 10000
    rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]),
                  (camTarget[2] - camPos[2])]
    lenFwd = math.sqrt(
        rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] + rayForward[2] * rayForward[
            2])
    invLen = farPlane * 1. / lenFwd
    rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
    rayFrom = camPos
    oneOverWidth = float(1) / float(width)
    oneOverHeight = float(1) / float(height)

    dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
    dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
    rayToCenter = [rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1],
                   rayFrom[2] + rayForward[2]]
    ortho = [
        - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0],
        - 0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1],
        - 0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]]

    rayTo = [rayFrom[0] + rayForward[0] + ortho[0],
             rayFrom[1] + rayForward[1] + ortho[1],
             rayFrom[2] + rayForward[2] + ortho[2]]
    lenOrtho = math.sqrt(ortho[0] * ortho[0] + ortho[1] * ortho[1] + ortho[2] * ortho[2])
    alpha = math.atan(lenOrtho / farPlane)
    return rayFrom, rayTo, alpha

def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    swapped = False  # swap start and end points if necessary and store swap state
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    ystep = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points

def create_point_cloud_and_occupancy_grid():
    start_of_point_cloud_calculation = time.time()
    print('Beginning point cloud and occupancy grid creation')

    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, \
                        pitch, yaw, dist, camTarget = p.getDebugVisualizerCamera()
    camPos = [camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
              camTarget[2] - dist * camForward[2]]
    farPlane = 10000
    rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
    lenFwd = math.sqrt(
        rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] + rayForward[2] * rayForward[2])
    oneOverWidth = float(1) / float(width)
    oneOverHeight = float(1) / float(height)
    dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
    dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]

    lendHor = math.sqrt(dHor[0] * dHor[0] + dHor[1] * dHor[1] + dHor[2] * dHor[2])
    lendVer = math.sqrt(dVer[0] * dVer[0] + dVer[1] * dVer[1] + dVer[2] * dVer[2])

    cornersX = [0, width, width, 0]
    cornersY = [0, 0, height, height]
    corners3D = []

    imgW = int(width / 10)  # todo why do we do this? try not doing this. Is this the gap in ray tracing?
    imgH = int(height / 10)

    img = p.getCameraImage(imgW, imgH, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # img = p.getCameraImage(imgW, imgH)
    rgbBuffer = img[2]
    depthBuffer = img[3]
    segBuffer = img[4]
    print("rgbBuffer.shape=", rgbBuffer.shape)
    print("depthBuffer.shape=", depthBuffer.shape)
    print("segBuffer.shape=", segBuffer.shape)

    # disable rendering temporary makes adding objects faster
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
    # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  # if you want to visualise the balls/points appearing but makes it very slow
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1, 1, 1, 1], radius=0.03)
    collisionShapeId = -1  # p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="duck_vhacd.obj", collisionFramePosition=shift,meshScale=meshScale)

    for i in range(4):
        w = cornersX[i]
        h = cornersY[i]
        rayFrom, rayTo, _ = getRayFromTo(w, h)
        rf = np.array(rayFrom)
        rt = np.array(rayTo)
        vec = rt - rf
        l = np.sqrt(np.dot(vec, vec))
        newTo = (0.01 / l) * vec + rf
        # print("len vec=",np.sqrt(np.dot(vec,vec)))

        p.addUserDebugLine(rayFrom, newTo, [1, 0, 0])
        corners3D.append(newTo)
    count = 0

    step_amt = 3
    # step_amt = 1
    stepX, stepY = step_amt, step_amt
    all_point_locations = []
    all_point_seg_objs = []
    for w in range(0, imgW, stepX):
        for h in range(0, imgH, stepY):
            count += 1
            if ((count % 100) == 0):
                print(count, "out of ", imgW * imgH / (stepX * stepY))
            rayFrom, rayTo, alpha = getRayFromTo(w * (width / imgW), h * (height / imgH))
            rf = np.array(rayFrom)
            rt = np.array(rayTo)
            vec = rt - rf
            l = np.sqrt(np.dot(vec, vec))
            depthImg = float(depthBuffer[h, w])
            far = 1000.
            near = 0.01
            depth = far * near / (far - (far - near) * depthImg)
            depth /= math.cos(alpha)
            newTo = (depth / l) * vec + rf

            # todo we aren't shooting a ray out of each pixel obviously? So what are we doing? Floating point approximation on mouseX?
            # todo euclidean distance isn't taking into account rayFrom?!?!?!?!?!!
            if math.sqrt(newTo[0] ** 2 + newTo[1] ** 2) < 30: # and newTo[2] < 0.2001:  # magic number for now  # todo could put floor check here
                all_point_locations.append(newTo)
                all_point_seg_objs.append(segBuffer[h, w])

                # Place balls/points with colour of pixel  # todo add boolean for this?
                # p.addUserDebugLine(rayFrom, newTo, [1, 0, 0])
                # mb = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collisionShapeId,
                #                        baseVisualShapeIndex=visualShapeId, basePosition=newTo,
                #                        useMaximalCoordinates=True)
                # color = rgbBuffer[h, w]
                # color = [color[0] / 255., color[1] / 255., color[2] / 255., 1]
                # p.changeVisualShape(mb, -1, rgbaColor=color)

    # trapezoid camera view pinhole frustum thingy
    p.addUserDebugLine(corners3D[0], corners3D[1], [1, 0, 0])
    p.addUserDebugLine(corners3D[1], corners3D[2], [1, 0, 0])
    p.addUserDebugLine(corners3D[2], corners3D[3], [1, 0, 0])
    p.addUserDebugLine(corners3D[3], corners3D[0], [1, 0, 0])
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    print("Ready to create grid")

    calculate_single_viewpoint_occupancy_grid(all_point_locations, all_point_seg_objs, camPos,
                                              camTarget, segBuffer)
    print('Time taken to create pointcloud and occupancy grid: {}'.format(
        time.time() - start_of_point_cloud_calculation))

def calculate_single_viewpoint_occupancy_grid(all_point_locations, all_point_seg_objs, camPos,
                                              camTarget, segBuffer):

    # todo find floor plane here. todo  and bucket and find out that a large percentage of the smallest values is most likely the floor value
    # todo below should be another function
    # import pdb;pdb.set_trace()
    grid_size = 0.1
    floor_height = 0.2001  # floor width was 0.2

    #
    # todo don't magic number floor plane? How could I calculate? Find a number of points on the same plane?
    all_point_x_y_locations_above_floor = [(point_loc[0], point_loc[1], point_seg) for
                                           point_loc, point_seg in
                                           zip(all_point_locations, all_point_seg_objs)
                                           if point_loc[2] > floor_height]
    # all_point_x_y_locations_above_floor = [(point_loc[0], point_loc[1], point_seg) for
    #                                        point_loc, point_seg in zip(all_point_locations, all_point_seg_objs)]
    # zs = [xyz[2] for xyz in all_point_locations]; zs = sorted(zs)  # todo why is there variation if it's perfect?
    # bins = np.linspace(zs[0], zs[-1], 4) # todo how many bins though?
    # digitized = np.digitize(zs, bins)  # todo take lowest bin, round it up?

    # loop through each point and set occupancy grid to 1 where it is closest
    all_x = [point[0] for point in all_point_x_y_locations_above_floor]
    all_y = [point[1] for point in all_point_x_y_locations_above_floor]
    border = 1
    x_range = np.arange(math.floor(min(all_x) - border), math.ceil(max(all_x) + border), 0.1)
    y_range = np.arange(math.floor(min(all_y) - border), math.ceil(max(all_y) + border), 0.1)

    # todo above is wrong. All xticks forever are wrong. arange is also ill-advised
    # todo clean higher level functions which creates grid from point_cloud and

    # todo if viewpoint is above house why do we still get balls below floor height outside the house?
    grid = np.zeros((y_range.shape[0], x_range.shape[0]))
    grid_object_class = np.zeros(
        (y_range.shape[0], x_range.shape[0]))  # todo variable shape. todo do both for easy viz
    grid_list_object_class = [[list() for x in range(x_range.shape[0])] for y in
                              range(y_range.shape[0])]

    for point in all_point_x_y_locations_above_floor:
        occupy_closest_grid(point, grid, grid_object_class, grid_list_object_class, x_range,
                            y_range, grid_size)

    # draw all figure plots
    camForward_xy = (camTarget[0] - camPos[0]), (camTarget[1] - camPos[1])
    append_pose_and_beam_measurements(camPos, camForward_xy,
                                      all_point_x_y_locations_above_floor)

    draw_figures(x_range, y_range, all_point_x_y_locations_above_floor, camPos, camForward_xy,
                 grid,
                 grid_object_class, segBuffer)

    # p.resetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=yaw, cameraPitch=pitch,  # todo attempt to move camera
    #                                     cameraTargetPosition=camTarget)  # failed attempt at putting camera to old position. hmm

def occupy_closest_grid(point, grid, grid_object_class, grid_list_object_class, x_range, y_range,
                        grid_size):
    point_seg_class = point[2]
    x, y = round(point[0], 1), round(point[1],
                                     1)  # round to nearest 10cm2 # todo don't hardcode the round?
    grid_x, grid_y = int(round((x - x_range[0]) / grid_size)), int(
        round((y - y_range[0]) / grid_size))
    grid[grid.shape[0] - grid_y, grid_x] = 1
    grid_object_class[grid.shape[0] - grid_y, grid_x] = point_seg_class
    grid_list_object_class[grid.shape[0] - grid_y][grid_x].append(
        point_seg_class)  # todo could double check if it properly works

def append_pose_and_beam_measurements(camPos, camForward_xy, all_point_x_y_locations_above_floor):
    # beam_measurements. each has: (range, angle of beam)
    beam_measurements = []
    for point in all_point_x_y_locations_above_floor:
        # atan2 takes two arguments and uses the sign information to calculate correct quadrant for the angle
        range_of_beam = np.sqrt(np.sum((np.array((camPos[0], camPos[1])) -
                                        np.array((point[0], point[1]))) ** 2))
        angle_of_beam = math.atan2(point[1] - camPos[1], point[0] - camPos[0])  # todo we subtract later, should we do now? todo confirm right here
        beam_measurements.append((range_of_beam, angle_of_beam, point[0], point[1], point[2]))
    beam_measurements = np.array(beam_measurements)

    # state/pose. each has: (x, y, theta (direction we are facing))
    pose = np.array([(camPos[0], camPos[1], math.atan2(camForward_xy[1], camForward_xy[0]))])

    all_beam_measurements.append(beam_measurements)  # todo make not red
    all_poses.append(pose)

def draw_figures(x_range, y_range, all_point_x_y_locations_above_floor, camPos, camForward_xy, grid, grid_object_class, segBuffer):
    fig, (ax, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 10))

    ax.scatter([x[0] for x in all_point_x_y_locations_above_floor],
               [x[1] for x in all_point_x_y_locations_above_floor], s=1.5)
    ax.scatter(camPos[0], camPos[1], color='r')
    for point in all_point_x_y_locations_above_floor:
        ax.plot([camPos[0], point[0]], [camPos[1], point[1]], c='r', linewidth=0.2)

    # todo make line segment. too long. Todo find point 3-5 along camForward_xy axis?
    ax.plot([camPos[0], camForward_xy[0]], [camPos[1], camForward_xy[1]], c='blue', linewidth=2.0)
    ax.set_xticks(x_range)
    ax.set_yticks(y_range)
    ax.grid(True)
    ax.set_title('Point Cloud 2D')
    plt.xticks(rotation=80)

    fig.canvas.draw()
    ax2.set_xticks(range(x_range.shape[0]))
    ax2.set_yticks(range(y_range.shape[0]))
    ax2.set_xticklabels([round(x, 1) for x in x_range])
    ax2.set_yticklabels([round(y, 1) for y in y_range])
    plt.xticks(rotation=80)
    ax2.set_title('Occupancy Grid')
    cam_pos_grid_x, cam_pos_grid_y = int((camPos[0] + abs(x_range[0])) * 10), \
                                     int((camPos[1] + abs(y_range[0])) * 10)  # todo fix/change
    ax2.scatter(cam_pos_grid_x, grid.shape[0] - cam_pos_grid_y, color='r')
    ax2.imshow(grid, interpolation='none', cmap='gray')

    fig.canvas.draw()
    ax3.set_xticks(range(x_range.shape[0]))
    ax3.set_yticks(range(y_range.shape[0]))
    ax3.set_xticklabels([round(x, 1) for x in x_range])
    ax3.set_yticklabels([round(y, 1) for y in y_range])
    ax3.set_title('Object Map')
    plt.xticks(rotation=80)
    num_unique_objects = len(np.unique(grid_object_class))

    print('{} unique objects in segmentation buffer'.format(num_unique_objects))
    uniq_ints_in_seg_buffer = np.unique(segBuffer)
    mapping_to_positive_ints = {x: i for i, x in enumerate(uniq_ints_in_seg_buffer)}
    # mapping_from_positive_ints_to_orig = {i: x for i, x in enumerate(uniq_ints_in_seg_buffer)}
    # Make kitchen_walls_and_floor object the 0th index so tmp swap needed
    key_with_0_value = list(mapping_to_positive_ints.keys())[list(mapping_to_positive_ints.values()).index(0)]
    mapping_to_positive_ints[key_with_0_value] = mapping_to_positive_ints[obj_name_to_obj_id['kitchen_walls_0']]
    mapping_to_positive_ints[obj_name_to_obj_id['kitchen_walls_0']] = 0

    grid_object_class = grid_object_class.astype('int64')
    grid_object_class_mapped_positive_ints = np.copy(grid_object_class)
    for key in mapping_to_positive_ints:
        grid_object_class_mapped_positive_ints[grid_object_class == key] = \
            mapping_to_positive_ints[key]
    ax3.imshow(grid_object_class_mapped_positive_ints, interpolation='none', cmap=new_cmap)
    fontP = FontProperties()
    fontP.set_size('small')
    # reverse mapping. Colour should be the value of mapping_to_positive_ints[key]
    # Label should be object name using original key
    legend_elements = [Patch(facecolor=new_cmap(positive_val),
                       label=obj_id_to_obj_name[key])
                       for key, positive_val in mapping_to_positive_ints.items()
                       if key in obj_id_to_obj_name]
    box = ax3.get_position()
    ax3.set_position([box.x0, box.y0, box.width * 0.8, box.height])
    ax3.legend(loc='center left', handles=legend_elements, prop=fontP,  bbox_to_anchor=(1, 0.5))
    #
    # print("obj_name_to_obj_id['kitchen_0']: {}".format(obj_name_to_obj_id['kitchen_0']))
    # print('mapping_to_positive_ints[obj_name_to_obj_id[\'kitchen_0\']: ', mapping_to_positive_ints[obj_name_to_obj_id['kitchen_0']])
    # positive_val = mapping_to_positive_ints[obj_name_to_obj_id['kitchen_0']]
    # c = new_cmap(positive_val)
    #
    # print('new_cmap(positive_val): rgb({},{},{})'.format(int(c[0] * 255), int(c[1] * 255), int(c[2] * 255)))
    # plt.figure()
    # plt.imshow(segBuffer == obj_name_to_obj_id['kitchen_0'])
    # plt.figure()
    # arr = np.array([[0, 1, 2, 3, 4, 5, 7], [1, 1, 1, 1, 1, 1, 1]])
    # plt.imshow(arr, interpolation='none', cmap=new_cmap)
    #
    # # plt.figure()
    #
    #
    # fig, (ax4) = plt.subplots(1, 1, figsize=(5, 5))
    # import pdb;
    # pdb.set_trace()
    # # ax4.pcolor(grid_object_class_mapped_positive_ints, cmap=new_cmap)
    # my_cmap = plt.get_cmap('Paired', 40)
    # # my_cmap = ListedColormap(['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w'] * 10)
    # ax4.pcolor(grid_object_class_mapped_positive_ints, cmap=my_cmap)
    # ax4.set_title('the one')
    # fontP = FontProperties()
    # fontP.set_size('small')
    # # reverse mapping. Colour should be the value of mapping_to_positive_ints[key]
    # # Label should be object name using original key
    # legend_elements = [Patch(facecolor=my_cmap(positive_val),
    #                          label=obj_id_to_obj_name[key])
    #                    for key, positive_val in mapping_to_positive_ints.items()
    #                    if key in obj_id_to_obj_name]
    # box = ax4.get_position()
    # ax4.set_position([box.x0, box.y0, box.width * 0.8, box.height])
    # ax4.legend(loc='center left', handles=legend_elements, prop=fontP, bbox_to_anchor=(1, 0.5))
    #
    # # todo try pcolormap here!!!! https://matplotlib.org/3.1.0/gallery/images_contours_and_fields/pcolor_demo.html#sphx-glr-gallery-images-contours-and-fields-pcolor-demo-py
    # # todo or here: https://stackoverflow.com/questions/52566969/python-mapping-a-2d-array-to-a-grid-with-pyplot
    # # todo also try cmap blues or something. But we want the first to be black

    # import pdb; pdb.set_trace()
    plt.show()


class Map():
    """
    Adapted from: https://gist.github.com/superjax/33151f018407244cb61402e094099c1d
    """
    def __init__(self, xsize, ysize, grid_size, x_range, y_range, min_point_grid_x, min_point_grid_y,
                 measurement_model='laser_width_inverse_range_sensor_model'):
        # self.border = 2
        # self.border = 5  # todo messes up a few things. Ever needed?
        self.x_range = x_range
        self.y_range = y_range
        self.min_point_grid_x, self.min_point_grid_y = min_point_grid_x, min_point_grid_y
        self.border = 0
        self.xsize = xsize + self.border  # Add extra cells for the borders
        self.ysize = ysize + self.border
        self.grid_size = grid_size # save this off for future use
        self.log_prob_map = np.zeros((self.ysize, self.xsize)) # set all to zero

        self.alpha = 1.0 # The assumed thickness of obstacles
        self.beta = 5.0 * np.pi / 180.0 # The assumed width of the laser beam
        self.z_max = 150.0 # The max reading from the laser

        # Pre-allocate the x and y positions of all grid positions into a 3D tensor
        # (pre-allocation = faster)
        self.grid_position_m = np.array([np.tile((np.arange(self.ysize) * self.grid_size)[:, None].T, (self.xsize, 1)),
                                         np.tile((np.arange(self.xsize) * self.grid_size)[:, None], (1, self.ysize))])
        self.grid_position_m = np.swapaxes(self.grid_position_m, 1, 2)
        # shape of above: (2, ysize, xsize) because later we find out the positions for each x and y
        # coordinate of each grid cell relative to the agent's pose

        # Log-Probabilities to add or remove from the map
        self.l_occ = np.log(0.65/0.35)
        self.l_free = np.log(0.35/0.65)

        # self.measurement_model = measurement_model
        # self.measurement_model = 'bresenham_ray_tracing'
        self.measurement_model = 'my_algorithm'

    def update_map(self, pose, z):
        dx = self.grid_position_m.copy() # A tensor of coordinates of all cells
        dx[1, :, :] -= pose[0]  # A matrix of all the x coordinates of the cell
        dx[0, :, :] -= pose[1]  # A matrix of all the y coordinates of the cell
        # pose[2] is theta in table 9.2 chapter 9 of probabilistic robotics
        # theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2] # matrix of all bearings from robot to cell
        theta_to_grid = np.arctan2(dx[0, :, :], dx[1, :, :]) # matrix of all bearings from robot to cell

        # todo. removing "- pose[2]" made it horizontal as it should be? Why should or shouldn't this be done?
        # Wrap to +pi / - pi
        theta_to_grid[theta_to_grid > np.pi] -= 2. * np.pi
        theta_to_grid[theta_to_grid < -np.pi] += 2. * np.pi

        dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot

        # For each laser beam
        for z_i in z:
            r = z_i[0] # range measured
            b = z_i[1] # bearing measured
            # z_i[2:5] # xyz of point in pointcloud

            if self.measurement_model == 'laser_width_inverse_range_sensor_model':
                # Check table 9.2 in probabilistic robotics book
                # todo this still has issues with looking good
                # Calculate which cells are measured free or occupied, so we know which cells to update
                # Doing it this way is like a billion times faster than looping through each cell (because vectorized numpy is the only way to numpy)
                free_mask = (np.abs(theta_to_grid - b) <= self.beta / 2.0) & (dist_to_grid < (r - self.alpha / 2.0))  # todo play around with this and fully understand
                occ_mask = (np.abs(theta_to_grid - b) <= self.beta / 2.0) & (np.abs(dist_to_grid - r) <= self.alpha / 2.0)  # todo don't remove close things
                # Adjust the cells appropriately
                # this doesn't calculate the full frustum. It does it per laser.
                self.log_prob_map[occ_mask] += self.l_occ
                self.log_prob_map[free_mask] += self.l_free
            elif self.measurement_model == 'bresenham_ray_tracing':  # bresenham ray tracing line measurement model
                # https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
                # Use actual x, y location of point instead of range and bearing to calculate free-space
                free_mask, occ_mask = [], []
                # todo do we need to add x_range[0] and flip y-axis here? First flip y-axis later. Nah only in visualisation
                point_x_grid_pos = int(round(z_i[2].item() / self.grid_size)) + self.min_point_grid_x
                point_y_grid_pos = int(round(z_i[3].item() / self.grid_size)) - self.min_point_grid_y
                pose_x_grid_pos = int(round(pose[0].item() / self.grid_size)) + self.min_point_grid_x  # todo why was it: centix = int(round(-minx / xyreso))
                pose_y_grid_pos = int(round(pose[1].item() / self.grid_size)) - self.min_point_grid_y

                # the issue with bresenham. The world isn't 2D e.g. looking over tables.
                # todo if lasers overlap e.g. over the table and onto the table with same theta, then the last one or furthest one will override. World isn't 2D.
                laser_beams = bresenham((pose_x_grid_pos, pose_y_grid_pos),
                                        (point_x_grid_pos, point_y_grid_pos))  # line from the lidar to the occupied point
                for laser_beam in laser_beams:
                    free_mask.append([laser_beam[0], laser_beam[1]])
                occ_mask.append([point_x_grid_pos, point_y_grid_pos])  # todo others around it?
                occ_mask.append([point_x_grid_pos + 1, point_y_grid_pos])  # todo good idea?
                occ_mask.append([point_x_grid_pos, point_y_grid_pos + 1])
                occ_mask.append([point_x_grid_pos + 1, point_y_grid_pos + 1])

                self.log_prob_map[[xy[1] for xy in occ_mask], [xy[0] for xy in occ_mask]] += self.l_occ
                self.log_prob_map[[xy[1] for xy in free_mask], [xy[0] for xy in free_mask]] += self.l_free

                # todo try flood fill as well? Not a bad idea
            elif self.measurement_model == 'my_algorithm':
                # For each point position, add log odds occupied

                # todo need to add the old border (not new) to every grid position!!!
                # point_x_grid_pos = int(self.x_range[0]) + int(round(z_i[2].item() / self.grid_size))  # todo rounding errors? always floor?
                # point_y_grid_pos = int(self.y_range[0]) + int(round(z_i[3].item() / self.grid_size))  # todo division of number below 1 by a number below 1 causes the number to go high??!
                point_x_grid_pos = int(round(z_i[2].item() / self.grid_size)) + self.min_point_grid_x
                point_y_grid_pos = int(round(z_i[3].item() / self.grid_size)) - self.min_point_grid_y  # todo looks much better but -2 is the bottom instead of 0
                # todo why subtract min_point_y vs add min point x?
                # todo seems a bit off with int(self.x_range[0]) +
                # todo do we subtract y_range[0]?
                # self.log_prob_map[point_x_grid_pos, point_y_grid_pos] += self.l_occ
                self.log_prob_map[point_y_grid_pos, point_x_grid_pos] += self.l_occ  # todo flip?!?!? Why again did we stop flipping earlier?

                # todo grey means don't know. black means occupied. white means free. Should be done somewhere, 0.5 starting value in probability map?


if __name__ == '__main__':
    start_time = time.time()

    run_simulation = True
    if run_simulation:
        cid = p.connect(p.SHARED_MEMORY)
        if (cid < 0):
            p.connect(p.GUI)

        p.resetSimulation()
        p.setGravity(0, 0, -9.8)

        # # todo find better way of debugging maps and organising code in general. Instead of commenting this out all the time e.g. different file, user input
        obj_id_to_obj_name, obj_name_to_obj_id = load_all_urdfs()
        print('Time taken to load all objects and begin simulation: {:.2f}'.format(
            time.time() - start_time))

        # # new_cmap = rand_cmap(100, type='bright', first_color_black=True, last_color_black=False, verbose=True)
        new_cmap = rand_cmap(100, type='bright', first_color_black=True, last_color_black=False, verbose=False)
        segLinkIndex = False
        all_beam_measurements = []
        all_poses = []
        print('Beginning physics simulation loop')
        for i in range(10000000):
            keys = p.getKeyboardEvents()

            if ord('d') in keys:
                state = keys[ord('d')]
                if (state & p.KEY_WAS_RELEASED):
                    segLinkIndex = 1 - segLinkIndex
                    # print("segLinkIndex=",segLinkIndex)
                    print(segLinkIndex)
            if ord('p') in keys:
                state = keys[ord('p')]
                if (state & p.KEY_WAS_RELEASED):
                    create_point_cloud_and_occupancy_grid()
            if ord('q') in keys:
                state = keys[ord('q')]
                if (state & p.KEY_WAS_RELEASED):
                    if all_beam_measurements:
                        with open('beam_measurements_and_pose.pkl', 'wb') as f:
                            pickle.dump(
                                {'beam_measurements': all_beam_measurements,
                                 'pose': np.concatenate(all_poses)}, f)
                    break

            flags = 0
            if (segLinkIndex):
                flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX

            p.stepSimulation()
            # _, _, rgb, depth, seg = p.getCameraImage(320, 200, flags=flags)  # uncomment for updating every frame but slow

            time.sleep(1 / 240)

    # todo move camera to right place at start. meh, was hard to do but shouldn't be

    # load data from multiple measurements taken from pressing p key.
    # with open('beam_measurements_and_pose_two_viewpoints_with_jpg.pkl', 'rb') as f:
    # with open('beam_measurements_and_pose_5_good_with_last_in_2nd_room.pkl', 'rb') as f:
    with open('beam_measurements_and_pose.pkl', 'rb') as f:
        data = pickle.load(f)
        # state/pose each has: (x, y, theta (direction we are facing))
        # measurements each has: (range, angle of beam)
        state = data['pose']
        measurements = data['beam_measurements']
        grid_size = 0.1

        # get all point cloud xy positions from all measurements and calculate size of big map
        all_point_xy_concat = [m[2:4] for viewpoint in measurements for m in viewpoint]
        point_xs = [z_i[0].item() for z_i in all_point_xy_concat]
        point_ys = [z_i[1].item() for z_i in all_point_xy_concat]
        min_point_x, max_point_x = min(point_xs), max(point_xs)
        min_point_y, max_point_y = min(point_ys), max(point_ys)
        point_grid_xs = [int(round(z_i[0].item() / grid_size)) for z_i in all_point_xy_concat]
        point_grid_ys = [int(round(z_i[1].item() / grid_size)) for z_i in all_point_xy_concat]
        min_point_grid_x, max_point_grid_x = min(point_grid_xs), max(point_grid_xs)
        min_point_grid_y, max_point_grid_y = min(point_grid_ys), max(point_grid_ys)
        border = 5  # todo try bigger border and check why it doesn't work as I want it
        x_range = np.arange(min_point_grid_x - border, max_point_grid_x + border)  # x_range isn't float anymore, so why arange todo. Also how did this shift happen?
        y_range = np.arange(min_point_grid_x - border, max_point_grid_y + border + 5)  # todo remove magic number? for blue circle hmm
        # todo how to get read world values for x and y ticks from this?
        print('Min point x: ', min_point_x, 'max point x: ', max_point_x)
        print('Min point y: ', min_point_y, 'max_point y: ', max_point_y)
        print('Min point grid x: ', min_point_grid_x, 'Max_point grid x: ', max_point_grid_x)
        print('Min point grid y: ', min_point_grid_y, 'max_point grid y: ', max_point_grid_y)
        print('x_range: {}. y_range: {}'.format(len(x_range), len(y_range)))
        # todo could store the old grid too and use that
        # todo keep flipping and analysing the new grid. Confirm angles correct, confirm points correct, confirm x and y axis, borders (not 30) etc

    # Define the parameters for the map.  (Commented out is a 60x60m map with grid size 0.1x0.1m)
    # map = Map(int(60 / grid_size), int(60 / grid_size), grid_size)
    map = Map(int(len(x_range)), int(len(y_range)), grid_size, x_range, y_range,
              min_point_grid_x, min_point_grid_y)

    # mismatch between real 3d space conversion to 2d top down space to gridspace
    # todo understand what can stay in the same space and what conversions need to be done visualisation
    # todo what I want on a high level:
    # todo: 1. the circle of pose correct so I can debug better. Work with only 1 image first. Is correct but will it stay correct?
    # todo: 2. the right map size with axes and proper conversion from 2D top down space to grid space. Half done but axes...
    # todo: 3. aggregate two maps correctly. Kinda there.

    # todo. Wait every single one of my axes in all the charts are wrong. All I ever wanted though was for 0, 0 to be the corner of the house?
    # todo end goal would be to animate 3-20+ viewpoints and slowly build up a map
    # todo create pybullet maze procedurally and use it to test mapping. First test multiple viewpoints in our current house of course

    # plt.ion() # enable real-time plotting  # todo might wanna keep for animation and gif
    # plt.figure(1) # create a plot  # todo remove 1?
    # import pdb; pdb.set_trace()
    for i in tqdm(range(len(state))):
        map.update_map(state[i, :], measurements[i])
        # todo why is there stuff at the bottom of -2.0?

        fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10, 10))
        # Real-Time Plotting
        # (comment out these next lines to make it run super fast, matplotlib is painfully slow)
        pose = state[i, :]
        # cam_pos_grid_x, cam_pos_grid_y = int(round(pose[0] / grid_size)), \
        #                                  int(round(pose[1] / grid_size))
        cam_pos_grid_x, cam_pos_grid_y = int(round(pose[0] / grid_size)) + min_point_grid_x, \
                                         int(round(pose[1] / grid_size)) - min_point_grid_y  # todo x is slightly to the right
        # todo border will also affect this! Removed for now. Add + border above.

        circle_x_pos, circle_y_pos = cam_pos_grid_x + map.border, cam_pos_grid_y + map.border
        print('pose[0:2]: {}. circle_x_pos: {} circle_y_pos: {}'.format(pose[0:2], circle_x_pos,
                                                                        circle_y_pos))
        circle = plt.Circle((circle_x_pos, circle_y_pos),
                            radius=1.5, fc='blue')  # todo try without imshow and xticks below (how does imshow affect it anyway?)
        ax.add_patch(circle)
        # todo move/fix arrow
        # arrow = pose[0:2] + np.array([3.5, 0]).dot(np.array([[np.cos(pose[2]), np.sin(pose[2])],
        # arrow = np.array([cam_pos_grid_x, cam_pos_grid_y]) + np.array([3.5, 0]).dot(np.array([[np.cos(pose[2]), np.sin(pose[2])],
        # arrow = np.array([cam_pos_grid_x, cam_pos_grid_y]) + np.array([0.5, 0]).dot(np.array([[np.cos(pose[2]), np.sin(pose[2])],
        #                                                      [-np.sin(pose[2]), np.cos(pose[2])]]))  # todo inverted inverted rotations?
        # # ax.plot([pose[1], arrow[1]], [pose[0], arrow[0]])  # todo why inverted?
        # ax.plot([cam_pos_grid_y, arrow[1]], [cam_pos_grid_x, arrow[0]])  # todo why inverted?
        # ax.plot(, c='r')  # todo don't have camForward or camTarget here. So how will I get arrow?

        probability_map = 1.0 - 1. / (1. + np.exp(map.log_prob_map))
        # probability_map = probability_map.T  # transpose x, y -> y, x.  # todo needed for inverse beam with measurement model or not?
        ax.imshow(probability_map, 'Greys', origin='lower')  # todo how does origin lower affect the circle? not at all? Wait it does affect it!
        ax.set_title('Probability map: {}'.format(i + 1))
        # todo in my original grid y-axis went from top to bottom -2.0 to 4.9 but it covered everything. Why is there a gap?
        # ax.set_xticks(range(x_range.shape[0]), [round(x, 1) for x in x_range])  # todo y ticks are inverted?
        # ax.set_yticks(range(y_range.shape[0]), [round(y, 1) for y in y_range])  # todo x + y ticks are all wrong. 0, 0 is not the corner of the map
        # ax.set_xticks([round(x, 1) for x in x_range])
        # ax.set_yticks([round(y, 1) for y in y_range])
        # import pdb;pdb.set_trace()
        ax.set_xticks(range(x_range.shape[0]))
        ax.set_yticks(range(y_range.shape[0]))
        ax.set_xticklabels([round(x, 1) for x in x_range])
        ax.set_yticklabels([round(y, 1) for y in y_range])  # todo how to do real world ticks rather than grid ticks
        plt.xticks(rotation=90)
        # todo don't show every one since they're jumbled  # ax.set_xticks(ax.get_xticks()[::2]) # or # for label in ax.get_xticklabels()[::2]: label.set_visible(False)
        # todo y_ticks are messed up

        thresholded_map = np.ones(probability_map.shape)
        thresholded_map[probability_map > 0.7] = 0
        ax2.imshow(thresholded_map, cmap='gray', origin='lower')
        circle2 = plt.Circle((circle_x_pos, circle_y_pos),
                             radius=1.5, fc='blue')
        ax2.add_patch(circle2)
        ax2.set_title('Thresholded map: {}'.format(i + 1))
        ax2.set_xticks(range(x_range.shape[0]), [round(x, 1) for x in x_range])
        ax2.set_yticks(range(y_range.shape[0]), [round(y, 1) for y in y_range])
        plt.xticks(rotation=90)
        plt.pause(0.005)
        # break

    # Final Plotting
    # plt.figure()
    # plt.ioff()
    # plt.clf()
    # plt.title('Final figure')
    # plt.imshow(np.swapaxes(1.0 - 1./(1.+np.exp(map.log_prob_map)), 0, 1), 'Greys') # This is probability  # todo keep eventually if we do animation instead
    plt.show()
