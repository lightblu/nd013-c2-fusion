# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Process the point-cloud and prepare it for object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import cv2
import numpy as np
import torch

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2, label_pb2

# object detection tools and helper functions
import misc.objdet_tools as tools
import zlib
import open3d as o3d
import math


# visualize lidar point-cloud
def show_pcl(pcl):

    ####### ID_S1_EX2 START #######     
    #######
    print("student task ID_S1_EX2")

    # step 1 : initialize open3d with key callback and create window

    # Some more keys for some additional action, though most is already featured in the viewer itself,
    # press 'h' to get a list of mouse and keyboard commands!
    def key_callback(visualizer):
        show_pcl.is_rendering = False

    def esc_callback(v):
        v.destroy_window()
        sys.exit(0)

    # We use our function object to not spoil global namespace and have something like a function-local static in C/C++
    if 'vis' not in show_pcl.__dict__:
        show_pcl.is_rendering = True
        show_pcl.vis = o3d.visualization.VisualizerWithKeyCallback()
        # Keys: https://www.glfw.org/docs/latest/group__keys.html
        show_pcl.vis.register_key_callback(262, key_callback) # right
        show_pcl.vis.register_key_callback(32, key_callback) # space
        show_pcl.vis.register_key_callback(256, esc_callback) # esc
        show_pcl.vis.create_window()
        # step 2 : create instance of open3d point-cloud class
        show_pcl.pcd = o3d.geometry.PointCloud()

    # step 3 : set points in pcd instance by converting the point-cloud into 3d vectors (using open3d function Vector3dVector)
    show_pcl.pcd.points =  o3d.utility.Vector3dVector(pcl[:, :3])

    # step 4 : for the first frame, add the pcd instance to visualization using add_geometry; for all other frames, use update_geometry instead
    if show_pcl.is_rendering == True: # This is only true in the first setup round
        show_pcl.vis.add_geometry(show_pcl.pcd)
    else:
        show_pcl.vis.update_geometry(show_pcl.pcd)
    
    # step 5 : visualize point cloud and keep window open until right-arrow is pressed (key-code 262)
    show_pcl.is_rendering = True  # to be able to return from here, we cannot use .run() but need to di
    while show_pcl.is_rendering:  # it "manually" with poll_events, update_renderer and our own loop guard
        show_pcl.vis.poll_events()
        show_pcl.vis.update_renderer()
        #vis.run()

    #######
    ####### ID_S1_EX2 END #######     


# visualize range image
def show_range_image(frame, lidar_name, crop_to_forward=True):

    ####### ID_S1_EX1 START #######     
    #######
    print("student task ID_S1_EX1")

    # step 1 : extract lidar data and range image for the roof-mounted lidar

    # find the lidar image given by lidar_name
    lidar =  next(li for li in frame.lasers if li.name == lidar_name)

    # extract range image as described in "The Lidar Sensor - Step 12"
    if len(lidar.ri_return1.range_image_compressed) > 0: # use first response
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)

    # ri now has a shape of (64, 2650, 4):
    # - 64 lines top to bottom
    # - 2650 cols which are 360 degrees covered in 0.1358 degree steps
    # - 4 channels: range, intensity, ?, ?

    # Unmentioned step but in rubric: crop to 90 degree field of view
    # Reusing approach from "The Lidar Sensor - Step 15": Center of the image corresponds to the positive x-axis,
    # but we correct the center according to the azimuth from the as shown in "The Lidar Sensor - Step 18".
    # Note: As we know the correction is small from the center, we can save us bounds checking / more complex correction here
    # (but sure would need to handle it better in reality for generic datasets!)
    if crop_to_forward:
        laser_cal = next(lc for lc in frame.context.laser_calibrations if lc.name == lidar_name)
        extrinsic = np.array(laser_cal.extrinsic.transform).reshape(4,4)
        az_correction = math.atan2(extrinsic[1,0], extrinsic[0,0])
        # Calculate what we need to substract from the center for correction
        index_correction = int(az_correction / (360 / ri.shape[1]))
        center = int(ri.shape[1]/2) - index_correction
        deg45 = int(ri.shape[1] / 8)
        ri = ri[:,center-deg45:center+deg45]

    # step 2 : extract the range and the intensity channel from the range image
    range_channel = ri[:,:,0]
    intensity_channel = ri[:,:,1]
    
    # step 3 : set values <0 to zero
    # Note: Could have clipped ri before to not duplicate, but following instructions!
    range_clipped = np.clip(range_channel, 0, np.Infinity)
    intensity_clipped = np.clip(intensity_channel, 0, np.Infinity)
    
    # step 4 : map the range channel onto an 8-bit scale and make sure that the full range of values is appropriately considered
    range_mapped = np.interp(range_clipped, (range_clipped.min(), range_clipped.max()), (0, 255))
    
    # step 5 : map the intensity channel onto an 8-bit scale and normalize with the difference between the 1- and 99-percentile to mitigate the influence of outliers
    # Note: Amazing to see that the max is 22016 while the 99 percentile is at 0.5 for the first image!
    p01, p99 = np.percentile(intensity_clipped, [1, 99])
    intensity_mapped = np.interp(intensity_clipped, (p01, p99), (0, 255))
    
    # step 6 : stack the range and intensity image vertically using np.vstack and convert the result to an unsigned 8-bit integer
    img_range_intensity = np.vstack((range_mapped, intensity_mapped)).astype(np.uint8)
    #######
    ####### ID_S1_EX1 END #######     
    
    return img_range_intensity

def quick_show(img):
    """Just for quick testing/debuggin."""
    while (1):
        cv2.imshow('test', img)
        if cv2.waitKey(10) & 0xFF == 27:
            break
    cv2.destroyAllWindows()

# create birds-eye view of lidar data
def bev_from_pcl(lidar_pcl, configs):

    # remove lidar points outside detection area and with too low reflectivity
    mask = np.where((lidar_pcl[:, 0] >= configs.lim_x[0]) & (lidar_pcl[:, 0] <= configs.lim_x[1]) &
                    (lidar_pcl[:, 1] >= configs.lim_y[0]) & (lidar_pcl[:, 1] <= configs.lim_y[1]) &
                    (lidar_pcl[:, 2] >= configs.lim_z[0]) & (lidar_pcl[:, 2] <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[mask]
    
    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]  

    # convert sensor coordinates to bev-map coordinates (center is bottom-middle)
    ####### ID_S2_EX1 START #######     
    #######
    print("student task ID_S2_EX1")

    ## step 1 :  compute bev-map discretization by dividing x-range by the bev-image height (see configs)
    ## step 2 : create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates
    # step 3 : perform the same operation as in step 2 for the y-coordinates but make sure that no negative bev-coordinates occur

    # Just using the nice np.interp function
    lidar_pcl_cpy = lidar_pcl.copy()
    lidar_pcl_cpy[:, 0] = np.interp(lidar_pcl[:, 0], (configs.lim_x[0], configs.lim_x[1]), (0, configs.bev_width)).astype(int)
    lidar_pcl_cpy[:, 1] = np.interp(lidar_pcl[:, 1], (configs.lim_y[0], configs.lim_y[1]), (0, configs.bev_height)).astype(int)

    # step 4 : visualize point-cloud using the function show_pcl from a previous task
    # Temporariy only in this function for debuggin! Note, due to not converting z this will look quite 3d-flattened already ;)
    #show_pcl(lidar_pcl_cpy)
    ####### ID_S2_EX1 END #######     
    
    # Compute intensity layer of the BEV map
    ####### ID_S2_EX2 START #######     
    #######
    print("student task ID_S2_EX2")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    intensity_map = np.zeros([configs.bev_width + 1, configs.bev_height + 1])

    # step 2 : re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then -z (use numpy.lexsort)
    lidar_pcl_sorted = lidar_pcl_cpy[ np.lexsort((-lidar_pcl_cpy[:,2], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0]))]


    ## step 3 : extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    ##          also, store the number of points per x,y-cell in a variable named "counts" for use in the next task

    _, idx, counts = np.unique(lidar_pcl_sorted[:, 0:2], axis=0, return_index=True, return_counts=True)
    lidar_top_pcl = lidar_pcl_sorted[idx]

    ## step 4 : assign the intensity value of each unique entry in lidar_top_pcl to the intensity map 
    ##          make sure that the intensity is scaled in such a way that objects of interest (e.g. vehicles) are clearly visible    
    ##          also, make sure that the influence of outliers is mitigated by normalizing intensity on the difference between the max. and min. value within the point cloud
    p01, p99 = np.percentile(lidar_top_pcl[:,3], [1, 99])
    intensity_map[lidar_top_pcl[:, 0].astype(int), lidar_top_pcl[:, 1].astype(int)] = np.interp(lidar_top_pcl[:,3], (p01, p99), (0, 1))

    ## step 5 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    #quick_show(intensity_map)

    #######
    ####### ID_S2_EX2 END ####### 


    # Compute height layer of the BEV map
    ####### ID_S2_EX3 START #######     
    #######
    print("student task ID_S2_EX3")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map
    height_map = np.zeros([configs.bev_width + 1, configs.bev_height + 1])

    ## step 2 : assign the height value of each unique entry in lidar_top_pcl to the height map 
    ##          make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    ##          use the lidar_pcl_top data structure from the previous task to access the pixels of the height_map

    # remember shift of heights at the beginning!
    height_map[lidar_top_pcl[:, 0].astype(int), lidar_top_pcl[:, 1].astype(int)] = np.interp(lidar_top_pcl[:,2], (0, configs.lim_z[1] - configs.lim_z[0]), (0,1))

    ## step 3 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background
    #quick_show(height_map)

    #######
    ####### ID_S2_EX3 END #######       

    # Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    #_, _, counts = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64)) 
    density_map[np.int_(lidar_top_pcl[:, 0]), np.int_(lidar_top_pcl[:, 1])] = normalizedCounts
    #quick_show(density_map)
        
    # assemble 3-channel bev-map from individual maps
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]  # r_map
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]  # g_map
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]  # b_map

    # expand dimension of bev_map before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)  # create tensor from birds-eye view
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps


