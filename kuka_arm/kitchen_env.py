import math
import time
from collections import Counter

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from matplotlib import colors
from matplotlib.font_manager import FontProperties
import pybullet as p

start_time = time.time()

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
    p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

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

def rand_cmap(nlabels, type='bright', first_color_black=True, last_color_black=False, verbose=False):
    """
    Creates a random colormap to be used together with matplotlib. Useful for segmentation tasks
    :param nlabels: Number of labels (size of colormap)
    :param type: 'bright' for strong colors, 'soft' for pastel colors
    :param first_color_black: Option to use first color as black, True or False
    :param last_color_black: Option to use last color as black, True or False
    :param verbose: Prints the number of labels and shows the colormap. True or False
    :return: colormap for matplotlib
    """
    from matplotlib.colors import LinearSegmentedColormap
    import colorsys

    if type not in ('bright', 'soft'):
        print ('Please choose "bright" or "soft" for type')
        return

    if verbose:
        print('Number of labels: ' + str(nlabels))

    # Generate color map for bright colors, based on hsv
    if type == 'bright':
        randHSVcolors = [(np.random.uniform(low=0.0, high=1),
                          np.random.uniform(low=0.2, high=1),
                          np.random.uniform(low=0.9, high=1)) for i in range(nlabels)]

        # Convert HSV list to RGB
        randRGBcolors = []
        for HSVcolor in randHSVcolors:
            randRGBcolors.append(colorsys.hsv_to_rgb(HSVcolor[0], HSVcolor[1], HSVcolor[2]))

        if first_color_black:
            randRGBcolors[0] = [0, 0, 0]

        if last_color_black:
            randRGBcolors[-1] = [0, 0, 0]

        random_colormap = LinearSegmentedColormap.from_list('new_map', randRGBcolors, N=nlabels)

    # Generate soft pastel colors, by limiting the RGB spectrum
    if type == 'soft':
        low = 0.6
        high = 0.95
        randRGBcolors = [(np.random.uniform(low=low, high=high),
                          np.random.uniform(low=low, high=high),
                          np.random.uniform(low=low, high=high)) for i in range(nlabels)]

        if first_color_black:
            randRGBcolors[0] = [0, 0, 0]

        if last_color_black:
            randRGBcolors[-1] = [0, 0, 0]
        random_colormap = LinearSegmentedColormap.from_list('new_map', randRGBcolors, N=nlabels)

    # Display colorbar
    if verbose:
        from matplotlib import colors, colorbar
        from matplotlib import pyplot as plt
        fig, ax = plt.subplots(1, 1, figsize=(15, 0.5))

        bounds = np.linspace(0, nlabels, nlabels + 1)
        norm = colors.BoundaryNorm(bounds, nlabels)

        cb = colorbar.ColorbarBase(ax, cmap=random_colormap, norm=norm, spacing='proportional', ticks=None,
                                   boundaries=bounds, format='%1i', orientation=u'horizontal')

    return random_colormap

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
    # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  # if you want to visualise the balls appearing but makes it very slow
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
            if math.sqrt(newTo[0] ** 2 + newTo[1] ** 2) < 30:  # magic number for now  # todo could put floor check here
                all_ball_locations.append(newTo)
                all_ball_seg_objs.append(segBuffer[h, w])

                # Place balls with colour of pixel
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

    floor_height = 0.2001  # floor width was 0.2
    all_ball_x_y_locations_above_floor = [(ball_loc[0], ball_loc[1], ball_seg) for ball_loc, ball_seg in
                                          zip(all_ball_locations, all_ball_seg_objs)
                                          if ball_loc[2] > floor_height]

    fig, (ax, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 10))

    ax.scatter([x[0] for x in all_ball_x_y_locations_above_floor],
               [x[1] for x in all_ball_x_y_locations_above_floor], s=1.5)
    ax.scatter(camPos[0], camPos[1], color='r')
    x_range = np.arange(math.floor(ax.get_xlim()[0]), math.ceil(ax.get_xlim()[1]), 0.1)
    y_range = np.arange(math.floor(ax.get_ylim()[0]), math.ceil(ax.get_ylim()[1]), 0.1)
    ax.set_xticks(x_range)
    ax.set_yticks(y_range)
    ax.grid(True)
    ax.set_title('Point Cloud 2D')
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
        grid_list_object_class[grid.shape[0] - grid_y][grid_x].append(ball_seg_class)  # todo could double check if it properly works

    grid = np.zeros((y_range.shape[0], x_range.shape[0]))
    grid_object_class = np.zeros((y_range.shape[0], x_range.shape[0])) ## todo variable shape. todo do both for easy viz
    grid_list_object_class = [[list() for x in range(x_range.shape[0])] for y in
                              range(y_range.shape[0])]

    for ball in all_ball_x_y_locations_above_floor:
        set_closest_occupancy_grid(ball)

    fig.canvas.draw()
    ax2.set_xticks(range(x_range.shape[0]))
    ax2.set_yticks(range(y_range.shape[0]))
    ax2.set_xticklabels([round(x, 1) for x in x_range])
    ax2.set_yticklabels([round(y, 1) for y in y_range])
    plt.xticks(rotation=80)
    ax2.set_title('Occupancy Grid')
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

    print("obj_name_to_obj_id['kitchen_0']: {}".format(obj_name_to_obj_id['kitchen_0']))
    print('mapping_to_positive_ints[obj_name_to_obj_id[\'kitchen_0\']: ', mapping_to_positive_ints[obj_name_to_obj_id['kitchen_0']])
    positive_val = mapping_to_positive_ints[obj_name_to_obj_id['kitchen_0']]
    c = new_cmap(positive_val)

    print('new_cmap(positive_val): rgb({},{},{})'.format(int(c[0] * 255), int(c[1] * 255), int(c[2] * 255)))
    plt.figure()
    plt.imshow(segBuffer == obj_name_to_obj_id['kitchen_0'])
    plt.figure()
    arr = np.array([[0, 1, 2, 3, 4, 5, 7], [1, 1, 1, 1, 1, 1, 1]])
    plt.imshow(arr, interpolation='none', cmap=new_cmap)

    import pdb; pdb.set_trace()
    plt.show()


# time.sleep(4)
new_cmap = rand_cmap(100, type='bright', first_color_black=True, last_color_black=False, verbose=True)
segLinkIndex = False
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

    flags = 0
    if (segLinkIndex):
        flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX

    p.stepSimulation()
    # _, _, rgb, depth, seg = p.getCameraImage(320, 200, flags=flags)  # uncomment for updating every frame but slow

    # import pdb;pdb.set_trace()
    time.sleep(1/ 240)
