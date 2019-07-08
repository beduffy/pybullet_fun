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


def load_all_urdfs():
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

def create_point_cloud_and_occupancy_grid():
    start_of_point_cloud_calculation = time.time()
    print('Beginning point cloud and occupancy grid creation')
    width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, pitch, yaw, dist, camTarget = p.getDebugVisualizerCamera()
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
    stepX, stepY = step_amt, step_amt
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
    print("Ready to create grid")

    floor_height = 0.2001  # floor width was 0.2
    all_ball_x_y_locations_above_floor = [(ball_loc[0], ball_loc[1], ball_seg) for ball_loc, ball_seg in
                                          zip(all_ball_locations, all_ball_seg_objs)
                                          if ball_loc[2] > floor_height]




    # loop through each point and set occupancy grid to 1 where it is closest
    # x_range = np.arange(math.floor(ax.get_xlim()[0]), math.ceil(ax.get_xlim()[1]), 0.1)
    # y_range = np.arange(math.floor(ax.get_ylim()[0]), math.ceil(ax.get_ylim()[1]), 0.1)
    all_x = [ball[0] for ball in all_ball_x_y_locations_above_floor]
    all_y = [ball[1] for ball in all_ball_x_y_locations_above_floor]
    global x_range, y_range
    x_range = np.arange(math.floor(min(all_x) - 1), math.ceil(max(all_x)), 0.1)
    y_range = np.arange(math.floor(min(all_y) - 1), math.ceil(max(all_y)), 0.1)

    def set_closest_occupancy_grid(ball):
        ball_seg_class = ball[2]
        x, y = round(ball[0], 1), round(ball[1], 1)
        grid_x, grid_y = int((x + abs(x_range[0])) * 10), int((y + abs(y_range[0])) * 10)
        grid[grid.shape[0] - grid_y, grid_x] = 1  # todo crashed in another room. fix. #IndexError: index 50 is out of bounds for axis 1 with size 50
        grid_object_class[grid.shape[0] - grid_y, grid_x] = ball_seg_class
        grid_list_object_class[grid.shape[0] - grid_y][grid_x].append(ball_seg_class)  # todo could double check if it properly works

    grid = np.zeros((y_range.shape[0], x_range.shape[0]))
    grid_object_class = np.zeros((y_range.shape[0], x_range.shape[0])) ## todo variable shape. todo do both for easy viz
    grid_list_object_class = [[list() for x in range(x_range.shape[0])] for y in
                              range(y_range.shape[0])]

    for ball in all_ball_x_y_locations_above_floor:
        set_closest_occupancy_grid(ball)

    # draw all figure plots
    camForward_xy = (camTarget[0] - camPos[0]), (camTarget[1] - camPos[1])
    append_pose_and_beam_measurements(camPos, camForward_xy, all_ball_x_y_locations_above_floor,
                                      x_range, y_range)

    draw_figures(x_range, y_range, all_ball_x_y_locations_above_floor, camPos, camForward_xy, grid,
                 grid_object_class, segBuffer)
    print('Time taken to create pointcloud and occupancy grid: {}'.format(
        time.time() - start_of_point_cloud_calculation))


    # p.resetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=yaw, cameraPitch=pitch,
    #                                     cameraTargetPosition=camTarget)  # failed attempt at putting camera to old position. hmm

def append_pose_and_beam_measurements(camPos, camForward_xy, all_ball_x_y_locations_above_floor,
                                      x_range, y_range):
    # beam_measurements. each has: (range, angle of beam)
    beam_measurements = []
    for ball in all_ball_x_y_locations_above_floor:  # todo variable number of balls. Do I need to do fixed lidar like thing or just list it?
        # atan2 takes two arguments and uses the sign information to calculate correct quadrant for the angle
        range_of_beam = np.sqrt(np.sum((np.array((camPos[0], camPos[1])) -
                                        np.array((ball[0], ball[1]))) ** 2))
        angle_of_beam = math.atan2(ball[1] - camPos[1], ball[0] - camPos[0])  # todo we subtract later, should we do now? todo confirm right here
        beam_measurements.append((range_of_beam, angle_of_beam))
    beam_measurements = np.array(beam_measurements).reshape(1, len(beam_measurements),
                                                            len(beam_measurements[0]))

    import pdb;
    pdb.set_trace()
    # state/pose. each has: (x, y, theta (direction we are facing))
    # pose = (camPos[0], camPos[1], math.atan2(camForward_xy[1] - camPos[1], camForward_xy[0] - camPos[0])) ## todo remove CamPos?!?!?!?!?! nah because camForward has it already
    # pose = np.array([(camPos[0], camPos[1], math.atan2(camPos[1], camPos[0]))])
    pose = np.array([(camPos[0], camPos[1], math.atan2(camForward_xy[1], camForward_xy[0]))])

    all_beam_measurements.append(beam_measurements)
    all_poses.append(pose)
    # with open('beam_measurements_and_pose.pkl', 'wb') as f:
    #     pickle.dump(
    #         {'beam_measurements': beam_measurements, 'pose': pose, 'grid_xrange': len(x_range),
    #          'grid_yrange': len(y_range)}, f)

def draw_figures(x_range, y_range, all_ball_x_y_locations_above_floor, camPos, camForward_xy, grid, grid_object_class, segBuffer):
    fig, (ax, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 10))

    ax.scatter([x[0] for x in all_ball_x_y_locations_above_floor],
               [x[1] for x in all_ball_x_y_locations_above_floor], s=1.5)
    ax.scatter(camPos[0], camPos[1], color='r')
    for ball in all_ball_x_y_locations_above_floor:
        ax.plot([camPos[0], ball[0]], [camPos[1], ball[1]], c='r', linewidth=0.2)

    # import pdb;    pdb.set_trace()
    ax.plot([camPos[0], camForward_xy[0]], [camPos[1], camForward_xy[1]], c='blue', linewidth=2.0)  # todo make line segment. too long.
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
    ax2.imshow(grid, interpolation='none', cmap='gray')

    # fig.canvas.draw()
    # ax3.set_xticks(range(x_range.shape[0]))
    # ax3.set_yticks(range(y_range.shape[0]))
    # ax3.set_xticklabels([round(x, 1) for x in x_range])
    # ax3.set_yticklabels([round(y, 1) for y in y_range])
    # ax3.set_title('Object Map')
    # plt.xticks(rotation=80)
    # num_unique_objects = len(np.unique(grid_object_class))
    #
    # print('{} unique objects in segmentation buffer'.format(num_unique_objects))
    # uniq_ints_in_seg_buffer = np.unique(segBuffer)
    # mapping_to_positive_ints = {x: i for i, x in enumerate(uniq_ints_in_seg_buffer)}
    # # mapping_from_positive_ints_to_orig = {i: x for i, x in enumerate(uniq_ints_in_seg_buffer)}
    # # Make kitchen_walls_and_floor object the 0th index so tmp swap needed
    # key_with_0_value = list(mapping_to_positive_ints.keys())[list(mapping_to_positive_ints.values()).index(0)]
    # mapping_to_positive_ints[key_with_0_value] = mapping_to_positive_ints[obj_name_to_obj_id['kitchen_walls_0']]
    # mapping_to_positive_ints[obj_name_to_obj_id['kitchen_walls_0']] = 0
    #
    # grid_object_class = grid_object_class.astype('int64')
    # grid_object_class_mapped_positive_ints = np.copy(grid_object_class)
    # for key in mapping_to_positive_ints:
    #     grid_object_class_mapped_positive_ints[grid_object_class == key] = \
    #         mapping_to_positive_ints[key]
    # ax3.imshow(grid_object_class_mapped_positive_ints, interpolation='none', cmap=new_cmap)
    # fontP = FontProperties()
    # fontP.set_size('small')
    # # reverse mapping. Colour should be the value of mapping_to_positive_ints[key]
    # # Label should be object name using original key
    # legend_elements = [Patch(facecolor=new_cmap(positive_val),
    #                    label=obj_id_to_obj_name[key])
    #                    for key, positive_val in mapping_to_positive_ints.items()
    #                    if key in obj_id_to_obj_name]
    # box = ax3.get_position()
    # ax3.set_position([box.x0, box.y0, box.width * 0.8, box.height])
    # ax3.legend(loc='center left', handles=legend_elements, prop=fontP,  bbox_to_anchor=(1, 0.5))
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
    # # todo draw the lasers/lines to the balls in 2d map because it looks cool

    # import pdb; pdb.set_trace()
    plt.show()


class Map():
    """
    Adapted from: https://gist.github.com/superjax/33151f018407244cb61402e094099c1d
    """
    def __init__(self, xsize, ysize, grid_size):
        self.xsize = xsize + 2 # Add extra cells for the borders
        self.ysize = ysize + 2
        # self.xsize = xsize + 30  # Add extra cells for the borders # todo good idea or not?
        # self.ysize = ysize + 30  # looks better without
        self.grid_size = grid_size # save this off for future use
        self.log_prob_map = np.zeros((self.xsize, self.ysize)) # set all to zero

        self.alpha = 1.0 # The assumed thickness of obstacles
        self.beta = 5.0 * np.pi / 180.0 # The assumed width of the laser beam
        self.z_max = 150.0 # The max reading from the laser

        # Pre-allocate the x and y positions of all grid positions into a 3D tensor
        # (pre-allocation = faster)
        self.grid_position_m = np.array([np.tile(np.arange(0, self.xsize * self.grid_size, self.grid_size)[:,None], (1, self.ysize)),
                                         np.tile(np.arange(0, self.ysize * self.grid_size, self.grid_size)[:,None].T, (self.xsize, 1))])
        # shape: (2, 102, 102) # todo why?
        # Log-Probabilities to add or remove from the map
        self.l_occ = np.log(0.65/0.35)
        self.l_free = np.log(0.35/0.65)

    def update_map(self, pose, z):
        dx = self.grid_position_m.copy() # A tensor of coordinates of all cells
        dx[0, :, :] -= pose[0] # A matrix of all the x coordinates of the cell
        dx[1, :, :] -= pose[1] # A matrix of all the y coordinates of the cell
        # pose[2] must be theta in table 9.2 chapter 9 of probabilistic robotics
        # theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2] # matrix of all bearings from robot to cell
        theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) # matrix of all bearings from robot to cell

        # todo is the bug above or below? above. removing - pose[2] made it horizantal as it should be?
        # Wrap to +pi / - pi
        theta_to_grid[theta_to_grid > np.pi] -= 2. * np.pi
        theta_to_grid[theta_to_grid < -np.pi] += 2. * np.pi

        dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot

        # For each laser beam
        for z_i in z:
            r = z_i[0] # range measured
            b = z_i[1] # bearing measured

            # Calculate which cells are measured free or occupied, so we know which cells to update
            # Doing it this way is like a billion times faster than looping through each cell (because vectorized numpy is the only way to numpy)
            free_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (dist_to_grid < (r - self.alpha / 2.0))
            occ_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (np.abs(dist_to_grid - r) <= self.alpha / 2.0)

            # Adjust the cells appropriately
            self.log_prob_map[occ_mask] += self.l_occ
            self.log_prob_map[free_mask] += self.l_free
            # todo try the above with my original grid and find another way to do free and occ mask? e.g. bresenham


if __name__ == '__main__':
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

    # obj_id_to_obj_name, obj_name_to_obj_id = load_all_urdfs()
    # print('Time taken to load all objects and begin simulation: {:.2f}'.format(
    #     time.time() - start_time))
    #
    # # new_cmap = rand_cmap(100, type='bright', first_color_black=True, last_color_black=False, verbose=True)
    # new_cmap = rand_cmap(100, type='bright', first_color_black=True, last_color_black=False, verbose=False)
    # segLinkIndex = False
    # all_beam_measurements = []
    # all_poses = []
    # print('Beginning physics simulation loop')
    # for i in range(10000000):
    #     keys = p.getKeyboardEvents()
    #
    #     if ord('d') in keys:
    #         state = keys[ord('d')]
    #         if (state & p.KEY_WAS_RELEASED):
    #             segLinkIndex = 1 - segLinkIndex
    #             # print("segLinkIndex=",segLinkIndex)
    #             print(segLinkIndex)
    #     if ord('p') in keys:
    #         state = keys[ord('p')]
    #         if (state & p.KEY_WAS_RELEASED):
    #             create_point_cloud_and_occupancy_grid()
    #     if ord('q') in keys:
    #         state = keys[ord('q')]
    #         if (state & p.KEY_WAS_RELEASED):
    #             # todo ValueError: all the input array dimensions except for the concatenation axis must match exactly
    #             # fixes: list sequence or fixed number of lasers. fixed sequence it is.
    #             if all_beam_measurements:
    #                 with open('beam_measurements_and_pose.pkl', 'wb') as f:
    #                     pickle.dump(
    #                         {'beam_measurements': all_beam_measurements,
    #                         # 'beam_measurements': np.concatenate(all_beam_measurements),
    #                          'pose': np.concatenate(all_poses),
    #                          'grid_xrange': len(x_range),
    #                          'grid_yrange': len(y_range)}, f)
    #             break
    #
    #     flags = 0
    #     if (segLinkIndex):
    #         flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
    #
    #     p.stepSimulation()
    #     # _, _, rgb, depth, seg = p.getCameraImage(320, 200, flags=flags)  # uncomment for updating every frame but slow
    #
    #     # import pdb;pdb.set_trace()
    #     time.sleep(1 / 240)

    # todo move camera to right place at start. meh, was hard to do but shouldn't be
    # TODO take 1-3 lidar point cloud pictures automatically. And even save them and load and don't even run pybullet????
    # todo do 1st image first

    # load matlab generated data (located at http://jamessjackson.com/files/index.php/s/sdKzy9nnqaVlKUe)
    with open('beam_measurements_and_pose.pkl', 'rb') as f:
        data = pickle.load(f)
        # state/pose each has: (x, y, theta (direction we are facing))
        # measurements each has: (range, angle of beam)
        state = data['pose']
        measurements = data['beam_measurements']
        # todo could store the old grid too and use that
        # todo keep flipping and analysing the new grid. Confirm angles correct, confirm points correct, confirm x and y axis, borders (not 30) etc

    # Define the parameters for the map.  (This is a 100x100m map with grid size 1x1m)
    # grid_size = 1.0
    # map = Map(int(100/grid_size), int(100/grid_size), grid_size)
    grid_size = 0.1
    # map = Map(int(60 / grid_size), int(60 / grid_size), grid_size)
    map = Map(int(data['grid_xrange']), int(data['grid_yrange']), grid_size)

    plt.ion() # enable real-time plotting
    plt.figure(1) # create a plot
    # for i in tqdm(range(len(state.T))):  # transpose, so columns of state are (x, y, theta)
    import pdb;pdb.set_trace()
    for i in tqdm(range(len(state))):
        # map.update_map(state[:, i], measurements[:, :, i].T) # update the map
        # map.update_map(state[i, :], measurements[i, :]) # update the map
        map.update_map(state[i, :], measurements[i][0]) # update the map  # todo instead of [0] do:

        # Real-Time Plotting
        # (comment out these next lines to make it run super fast, matplotlib is painfully slow)
        plt.clf()
        # pose = state[:, i]
        pose = state[i, :]
        circle = plt.Circle((pose[1], pose[0]), radius=1.5, fc='blue')
        plt.gca().add_patch(circle)
        arrow = pose[0:2] + np.array([3.5, 0]).dot(np.array([[np.cos(pose[2]), np.sin(pose[2])],
                                                             [-np.sin(pose[2]), np.cos(pose[2])]]))  # todo what is this?
        plt.plot([pose[1], arrow[1]], [pose[0], arrow[0]])  # todo why inverted?
        # todo why is the arrow pointing the right way but everything else is in the wrong place?
        # todo should y-axis be inverted?
        # todo flip axes as well!?!?!?
        # todo why so big?
        # todo try thresholded version

        probability_map = 1.0 - 1./(1. + np.exp(map.log_prob_map))
        # probability_map = np.swapaxes(probability_map, 0, 1)  # todo. was done because my original grid was y, x
        # probability_map = np.flip(probability_map, 1)  # todo is this even correct?
        plt.imshow(probability_map, 'Greys')

        plt.figure()
        thresholded_map = np.zeros(probability_map.shape)
        thresholded_map[probability_map > 0.7] = 1
        plt.imshow(thresholded_map)
        plt.pause(0.005)

    # import pdb;pdb.set_trace()
    # Final Plotting
    plt.ioff()
    plt.clf()
    plt.imshow(1.0 - 1./(1.+np.exp(map.log_prob_map)), 'Greys') # This is probability
    plt.imshow(map.log_prob_map, 'Greys') # log probabilities (looks really cool)
    plt.show()



    sys.exit()

    ####################
    ####################
    ####################
    ####################
    ####################
    ####################
    ####################
    # lidar_to_grid_map.py
    EXTEND_AREA = 1.0
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


    def calc_grid_map_config(ox, oy, xyreso):
        """
        Calculates the size, and the maximum distances according to the measurement center
        """
        # import pdb;pdb.set_trace()
        minx = round(min(ox) - EXTEND_AREA / 2.0)  # todo why divide by 2?
        miny = round(min(oy) - EXTEND_AREA / 2.0)
        maxx = round(max(ox) + EXTEND_AREA / 2.0)
        maxy = round(max(oy) + EXTEND_AREA / 2.0)
        xw = int(round((maxx - minx) / xyreso))
        yw = int(round((maxy - miny) / xyreso))
        print("The grid map is ", xw, "x", yw, ".")
        return minx, miny, maxx, maxy, xw, yw


    # def generate_ray_casting_grid_map(ox, oy, ix, iy, xyreso, breshen=True):  # attempt to get center
    def generate_ray_casting_grid_map(ox, oy, xyreso, breshen=True):
        """
        The breshen boolean tells if it's computed with bresenham ray casting (True) or with flood fill (False)
        """
        minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(ox, oy, xyreso)
        pmap = np.ones(
            (xw, yw)) / 2  # default 0.5 -- [[0.5 for i in range(yw)] for i in range(xw)]
        centix = int(round(-minx / xyreso))  # center x coordinate of the grid map
        centiy = int(round(-miny / xyreso))  # center y coordinate of the grid map
        # occupancy grid computed with bresenham ray casting
        if breshen:
            for (x, y) in zip(ox, oy):
                ix = int(round((x - minx) / xyreso))  # x coordinate of the occupied area
                iy = int(round((y - miny) / xyreso))  # y coordinate of the occupied area
                # ix = int(round((ix - minx) / xyreso))  # attempo
                # iy = int(round((iy - miny) / xyreso))
                laser_beams = bresenham((centix, centiy),
                                        (ix, iy))  # line form the lidar to the occupied point
                for laser_beam in laser_beams:
                    pmap[laser_beam[0]][laser_beam[1]] = 0.0  # free area 0.0
                pmap[ix][iy] = 1.0  # occupied area 1.0
                pmap[ix + 1][iy] = 1.0  # extend the occupied area
                pmap[ix][iy + 1] = 1.0  # extend the occupied area
                pmap[ix + 1][iy + 1] = 1.0  # extend the occupied area
        # # occupancy grid computed with with flood fill
        # else:
        #     pmap = init_floodfill((centix, centiy), (ox, oy), (xw, yw), (minx, miny), xyreso)
        #     flood_fill((centix, centiy), pmap)
        #     pmap = np.array(pmap, dtype=np.float)
        #     for (x, y) in zip(ox, oy):
        #         ix = int(round((x - minx) / xyreso))
        #         iy = int(round((y - miny) / xyreso))
        #         pmap[ix][iy] = 1.0  # occupied area 1.0
        #         pmap[ix + 1][iy] = 1.0  # extend the occupied area
        #         pmap[ix][iy + 1] = 1.0  # extend the occupied area
        #         pmap[ix + 1][iy + 1] = 1.0  # extend the occupied area
        return pmap, minx, maxx, miny, maxy, xyreso

    print(__file__, "start")
    xyreso = 0.02  # x-y grid resolution
    # ang, dist = file_read("lidar01.csv")
    import pdb;pdb.set_trace()
    dist = measurements[0, :, 0]
    ang = measurements[0, :, 1]
    ox = np.sin(ang) * dist  # todo why inverted y and x?
    oy = np.cos(ang) * dist
    pmap, minx, maxx, miny, maxy, xyreso = generate_ray_casting_grid_map(ox, oy, state[0, 0],
                                                                         state[0, 1], xyreso, True)
    xyres = np.array(pmap).shape
    plt.figure(1, figsize=(10, 4))
    plt.subplot(122)
    plt.imshow(pmap, cmap="PiYG_r")  # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
    plt.clim(-0.4, 1.4)
    plt.gca().set_xticks(np.arange(-.5, xyres[1], 1), minor=True)
    plt.gca().set_yticks(np.arange(-.5, xyres[0], 1), minor=True)
    plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
    plt.colorbar()
    plt.subplot(121)
    plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
    plt.axis("equal")
    plt.plot(0.0, 0.0, "ob")
    plt.gca().set_aspect("equal", "box")
    bottom, top = plt.ylim()  # return the current ylim
    plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
    plt.grid(True)
    plt.show()
