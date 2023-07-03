#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import time
import glob
import cv2
import rospy
import config_utils
import tensorflow.compat.v1 as tf
from contact_graspnet_ros.srv import GraspGroup, GraspGroupResponse
from contact_graspnet_ros.msg import GraspPose
from cv_bridge import CvBridge, CvBridgeError
from data import regularize_pc_point_count, depth2pc, load_available_input_data
from contact_grasp_estimator import GraspEstimator
from visualization_utils import visualize_grasps, show_image

tf.disable_eager_execution()
physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(BASE_DIR))


class GraspnetNode():
    def __init__(self) -> None:
        """
        Predict 6-DoF grasp distribution for given model and input data

        :param global_config: config.yaml from checkpoint directory
        :param checkpoint_dir: checkpoint directory
        :param K: Camera Matrix with intrinsics to convert depth to point cloud
        :param local_regions: Crop 3D local regions around given segments. 
        :param skip_border_objects: When extracting local_regions, ignore segments at depth map boundary.
        :param filter_grasps: Filter and assign grasp contacts according to segmap.
        :param segmap_id: only return grasps from specified segmap_id.
        :param z_range: crop point cloud at a minimum/maximum z distance from camera to filter out outlier points. Default: [0.2, 1.8] m
        :param forward_passes: Number of forward passes to run on each point cloud. Default: 1
        """
        self.ckpt_dir = rospy.get_param(
            "~ckpt_dir",
            os.path.dirname(os.path.abspath(__file__)).rstrip("contact_graspnet") + "checkpoints/scene_test_2048_bs3_hor_sigma_001")
        self.z_range = rospy.get_param("~z_range", [0.2, 1.1])
        self.local_regions = rospy.get_param("~local_regions", False)
        self.filter_grasps = rospy.get_param("~filter_grasps", True)
        self.skip_border_objects = rospy.get_param("~skip_border_objects", False)
        self.visulize = rospy.get_param("~visulize", True)
        self.forward_passes = rospy.get_param("~forward_passes", 5)
        self.arg_configs = rospy.get_param("~arg_configs", [])

        global_config = config_utils.load_config(self.ckpt_dir, batch_size=self.forward_passes, arg_configs=self.arg_configs)
        print(str(global_config))
        # Build the model
        self.grasp_estimator = GraspEstimator(global_config)
        self.grasp_estimator.build_network()

        # Add ops to save and restore all the variables.
        saver = tf.train.Saver(save_relative_paths=True)

        # Create a session
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        self.sess = tf.Session(config=config)

        # Load weights
        self.grasp_estimator.load_weights(self.sess, saver, self.ckpt_dir, mode='test')

        self.bridge = CvBridge()
        self.grasppoint_srv = rospy.Service('contact_graspnet/get_grasp_result', GraspGroup, self.handle_grasp)
        self.main_vis = False
        self.tmp_img = ()
        self.tmp_grasp = ()

    def handle_grasp(self, request):
        try:
            image_rgb = self.bridge.imgmsg_to_cv2(request.rgb, "rgb8")
            segmap = self.bridge.imgmsg_to_cv2(request.seg, "passthrough")
            cam_K = np.array(request.K).reshape(3, 3)
            image_depth = self.bridge.imgmsg_to_cv2(request.depth, "passthrough").copy()
            image_depth[np.isnan(image_depth)] = 0.
            segmap_id = request.segmap_id

            return self.inference(image_rgb, image_depth, segmap, cam_K, segmap_id)
        except CvBridgeError as e:
            rospy.logerr(e)

    def inference(self, rgb, depth, segmap, cam_K, segmap_id):
        begin = time.time()
        rospy.loginfo('Converting depth to point cloud(s)...')
        pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(
            depth, cam_K, segmap=segmap, rgb=rgb,
            skip_border_objects=self.skip_border_objects, z_range=self.z_range)

        rospy.loginfo('Generating Grasps...')
        pred_grasps_cam, scores, contact_pts, _ = self.grasp_estimator.predict_scene_grasps(
            self.sess, pc_full, pc_segments=pc_segments,
            local_regions=self.local_regions, filter_grasps=self.filter_grasps, forward_passes=self.forward_passes)

        reponse = GraspGroupResponse()
        target_id = segmap_id if (self.local_regions or self.filter_grasps) else -1
        for k in pred_grasps_cam:
            if k != target_id:
                continue
            for pose_cam, score, contact_pt in zip(pred_grasps_cam[k], scores[k], contact_pts[k]):
                grasp_pose = GraspPose()
                grasp_pose.pred_grasps_cam = pose_cam.flatten()
                grasp_pose.score = score
                grasp_pose.contact_pt = contact_pt
                reponse.grasp_poses.append(grasp_pose)

        # Visualize results
        if self.visulize:
            self.main_vis = True
            self.tmp_img = (rgb, segmap)
            self.tmp_grasp = (pc_full, pred_grasps_cam, scores, True, pc_colors)

        print("inference time: ", time.time() - begin)

        return reponse


if __name__ == "__main__":
    rospy.init_node('grasp_pose_estimator', anonymous=True, disable_signals=True)
    rospy.loginfo("Init grasp estimation node")
    graspnet_node = GraspnetNode()
    rate = rospy.Rate(10)
    try:
        while True:
            if graspnet_node.main_vis:
                visualize_grasps(*graspnet_node.tmp_grasp)
                show_image(*graspnet_node.tmp_img)
                graspnet_node.main_vis = False
                graspnet_node.tmp_img = ()
                graspnet_node.tmp_grasp = ()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down...')
