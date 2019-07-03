import math
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

import pybullet as p

start_time = time.time()

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

quat_orientation = p.getQuaternionFromEuler([0, 0, 3.14 / 2])

# Dynamic objects
beer_bottle = p.loadURDF("beer_bottle.urdf", basePosition=[3.4, 1.5, 1.1])
bowl = p.loadURDF("bowl.urdf", basePosition=[3.4, 1.5, 1.1])
plate = p.loadURDF("plate.urdf", basePosition=[3.4, 1, 1.1])
glass = p.loadURDF("glass.urdf", basePosition=[3.5, 1.3, 1.1])
knife = p.loadURDF("knife.urdf", basePosition=[3.6, 1.7, 1.1], globalScaling=0.01)
spoon = p.loadURDF("spoon.urdf", basePosition=[3.5, 1.2, 1.1], globalScaling=0.01)
fork = p.loadURDF("fork.urdf", basePosition=[3.4, 1.65, 1.1], globalScaling=1.0)
tissue_box = p.loadURDF("tissue_box.urdf", basePosition=[3.5, 0.7, 1.1], globalScaling=0.016)
tissue_box2 = p.loadURDF("tissue_box.urdf", basePosition=[3.5, 6, 1.5], globalScaling=0.016)
banana = p.loadURDF("banana.urdf", basePosition=[3.6, 2.3, 1.1], globalScaling=1.0)
crisps_chips = p.loadURDF("crisps_chips.urdf", basePosition=[3.5, 2.4, 1.1], globalScaling=0.01)
crisps_chips2 = p.loadURDF("crisps_chips.urdf", basePosition=[3.45, 6.35, 1.5], globalScaling=0.01)
teacup_model1 = p.loadURDF('TeaCup.urdf', basePosition=[3.4, 2, 1.1], baseOrientation=quat_orientation,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)
teacup_model2 = p.loadURDF('TeaCup.urdf', basePosition=[3.6, 5.8, 1.55], baseOrientation=quat_orientation,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)
teacup_model3 = p.loadURDF('TeaCup.urdf', basePosition=[1.5, 5.8, 1.5], baseOrientation=quat_orientation,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL, globalScaling=0.1)

# Mostly static
sofa = p.loadURDF("sofa.urdf", basePosition=[0.5, 6, 1], baseOrientation=quat_orientation)
table = p.loadURDF("table.urdf", basePosition=[1.5, 6, 1], baseOrientation=quat_orientation)
table2 = p.loadURDF("table.urdf", basePosition=[3.5, 6, 1], globalScaling=1.5,
                   baseOrientation=quat_orientation)
trash_can1 = p.loadURDF("trash_can.urdf", basePosition=[3.5, 9, 1], baseOrientation=quat_orientation)
trash_can2 = p.loadURDF("trash_can.urdf", basePosition=[3.5, 4, 1], baseOrientation=quat_orientation)
bed = p.loadURDF("bed.urdf", basePosition=[1.1, 7.8, 1], globalScaling=1.7, baseOrientation=quat_orientation)
lamp = p.loadURDF("nordic_floor_lamp_obj.urdf", basePosition=[1.5, 9, 1.3], globalScaling=0.01,
                 baseOrientation=quat_orientation)


walls = p.loadURDF("kitchen_walls.urdf")
kitchen = p.loadURDF("kitchen.urdf", basePosition=[0.2, 0.2, 0.1])

# todo attempt to make kitchen static while still keeping cupboards openable
# kitchen = p.loadURDF("kitchen.urdf")
# # p.loadURDF("IAI_kitchen.urdf", basePosition=[0, 0, 0.2])
# numJoints = p.getNumJoints(kitchen)
# for jointIndex in range(numJoints):
#     #p.resetJointState(kitchen, jointIndex, jointPositions[jointIndex])
#     print(p.getJointInfo(kitchen, jointIndex))
#     print(p.getDynamicsInfo(kitchen, jointIndex))
#     # p.changeDynamics(kitchen, jointIndex, mass=1000000)
#     # print(p.getDynamicsInfo(kitchen, jointIndex))
#

print('Time taken to load all objects and begin simulation: {:.2f}'.format(time.time() - start_time))

##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################
##########################

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

def create_point_cloud_and_occupancy_grid():
    start_of_point_cloud_calculation = time.time()
    print('Beginning point cloud and occupancy grid creation')
    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
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

    imgW = int(width / 10)
    imgH = int(height / 10)

    img = p.getCameraImage(imgW, imgH, renderer=p.ER_BULLET_HARDWARE_OPENGL)
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
    # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  # todo remove? slow
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
    step_amt = 1
    stepX = step_amt
    stepY = step_amt
    all_ball_locations = []
    all_ball_seg_objs = []
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
            if math.sqrt(newTo[0] ** 2 + newTo[1] ** 2) < 30:  # magic number for now
                all_ball_locations.append(newTo)
                all_ball_seg_objs.append(segBuffer[h, w])
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
    print("ready\n")

    # zeros = np.zeros_like(seg)
    # zeros[seg == 11] = 1
    # plt.imshow(zeros)
    # plt.show()
    # depth[seg == 11]
    # np.linspace(-15, 15)

    # todo:
    # 1. Plot x, y locations of all balls, and plot location of bla
    # 2. Plot grid with linspace or other

    floor_height = 0.201  # floor width was 0.2
    all_ball_x_y_locations_above_floor = [(ball_loc[0], ball_loc[1], ball_seg) for ball_loc, ball_seg in
                                          zip(all_ball_locations, all_ball_seg_objs)
                                          if ball_loc[2] > floor_height]

    fig = plt.figure()
    ax = fig.gca()

    ax.scatter([x[0] for x in all_ball_x_y_locations_above_floor],
               [x[1] for x in all_ball_x_y_locations_above_floor], s=1.5)
    ax.scatter(camPos[0], camPos[1], color='r')
    x_range = np.arange(math.floor(ax.get_xlim()[0]), math.ceil(ax.get_xlim()[1]), 0.1)
    y_range = np.arange(math.floor(ax.get_ylim()[0]), math.ceil(ax.get_ylim()[1]), 0.1)
    ax.set_xticks(x_range)
    ax.set_yticks(y_range)
    ax.grid(True)
    plt.xticks(rotation=80)
    print('Time taken to create pointcloud and occupancy grid: {}'.format(
        time.time() - start_of_point_cloud_calculation))

    # loop through each point and set occupancy grid to 1 where it is closest

    def set_closest_occupancy_grid(ball):
        ball_seg_class = ball[2]
        x, y = round(ball[0], 1), round(ball[1], 1)
        grid_x, grid_y = int((x + abs(x_range[0])) * 10), int((y + abs(y_range[0])) * 10)
        grid[grid.shape[0] - grid_y, grid_x] = 1
        grid_object_class[grid.shape[0] - grid_y, grid_x] = ball_seg_class

    grid = np.zeros((y_range.shape[0], x_range.shape[0]))
    grid_object_class = np.zeros((y_range.shape[0], x_range.shape[0])) ## todo variable shape. todo do both for easy viz
    grid_list_object_class = [[] * x_range.shape[0]]

    for ball in all_ball_x_y_locations_above_floor:
        set_closest_occupancy_grid(ball)

    # import pdb;
    # pdb.set_trace()
    fig = plt.figure()
    ax2 = fig.gca()
    fig.canvas.draw()
    ax2.set_xticks(range(x_range.shape[0]))
    ax2.set_yticks(range(y_range.shape[0]))
    ax2.set_xticklabels([round(x, 1) for x in x_range])
    ax2.set_yticklabels([round(y, 1) for y in y_range])
    # ax2.grid(True)
    plt.xticks(rotation=80)
    ax2.imshow(grid, interpolation='none', cmap='gray')

    fig = plt.figure()
    ax3 = fig.gca()
    fig.canvas.draw()
    ax3.set_xticks(range(x_range.shape[0]))
    ax3.set_yticks(range(y_range.shape[0]))
    ax3.set_xticklabels([round(x, 1) for x in x_range])
    ax3.set_yticklabels([round(y, 1) for y in y_range])
    plt.xticks(rotation=80)
    # cmap = colors.ListedColormap(['k', 'b', 'y', 'g', 'r'] * 50)  # todo hack
    # map for -2097151976 -> 0 so colour map works
    # def get_cmap(n, name='hsv'):
    #     '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    #     RGB color; the keyword argument name must be a standard mpl colormap name.'''
    #     return plt.cm.get_cmap(name, n)
    #
    # num_unique_objects = len(np.unique(grid_object_class))
    # import pdb;pdb.set_trace()

    # #
    # from itertools import cycle
    # cycol = cycle('bgrcmk')
    # #

    # cm = get_cmap(num_unique_objects)# todo make 0 black and the rest random colouirs
    # cmap = [[0, 0, 0]] + get_cmap(num_unique_objects)
    # print('{} unique objects in segmentation buffer'.format(num_unique_objects))
    # mapping_to_positive_ints = {x: i for i, x in enumerate(np.unique(seg))}
    # grid_object_class_mapped_positive_ints = np.copy(grid_object_class)
    # for key in mapping_to_positive_ints:
    #     grid_object_class_mapped_positive_ints[grid_object_class == key] = \
    #         mapping_to_positive_ints[key]
    # ax3.imshow(grid_object_class_mapped_positive_ints, interpolation='none', cmap=cmap)  # c = next(cycol)


    # todo create terrain map

    plt.show()

time.sleep(4)
segLinkIndex = False
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

    flags = 0
    if (segLinkIndex):
        flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX

    p.stepSimulation()
    # _, _, rgb, depth, seg = p.getCameraImage(320, 200, flags=flags)

    # import pdb;pdb.set_trace()
    time.sleep(1/ 240)
