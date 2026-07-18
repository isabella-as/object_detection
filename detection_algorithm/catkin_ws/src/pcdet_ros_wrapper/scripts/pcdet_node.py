#!/usr/bin/env python3

import os
import sys
import torch
import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.models import build_detector
from pcdet.utils import common_utils

class OpenPCDetROSNode:
    def __init__(self):
        rospy.init_node('pcdet_node', anonymous=True)
        
        cfg_file = "/home_station/OpenPCDet/tools/cfgs/kitti_models/pointpillar.yaml"
        ckpt_file = "/home_station/OpenPCDet/tools/pointpillar_7728.pth"
        
        cfg_from_yaml_file(cfg_file, cfg)
        self.logger = common_utils.create_logger()
        
        self.model = build_detector(cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=None)
        self.model.load_params_from_file(filename=ckpt_file, logger=self.logger, to_cpu=False)
        self.model.cuda()
        self.model.eval()
        
        self.class_names = cfg.CLASS_NAMES

        self.sub_pointcloud = rospy.Subscriber('/kitti/velo/pointcloud', PointCloud2, self.lidar_callback, queue_size=1)
        self.pub_detections = rospy.Publisher('/detection_boxes_3d', MarkerArray, queue_size=1)
        
        rospy.loginfo("--- OpenPCDet PointPillars ROS 1 Node Online (MarkerArray Mode) ---")

    def lidar_callback(self, msg):
        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points_list.append([p[0], p[1], p[2], p[3]])
            
        if len(points_list) == 0:
            return
            
        points_np = np.array(points_list, dtype=np.float32)

        with torch.no_grad():
            input_dict = {
                'points': torch.from_numpy(points_np).cuda(),
                'frame_id': torch.tensor([0])
            }
            
            ones = torch.zeros((input_dict['points'].shape[0], 1), device=input_dict['points'].device)
            input_dict['points'] = torch.cat((ones, input_dict['points']), dim=1)
            
            pred_dicts, _ = self.model.forward({'batch_dict': input_dict})
            
        marker_array = MarkerArray()
        boxes = pred_dicts[0]['pred_boxes'].cpu().numpy()
        scores = pred_dicts[0]['pred_scores'].cpu().numpy()

        for i in range(len(boxes)):
            if scores[i] < 0.4:
                continue
                
            marker = Marker()
            marker.header = msg.header
            marker.ns = "pcdet_boxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(boxes[i][0])
            marker.pose.position.y = float(boxes[i][1])
            marker.pose.position.z = float(boxes[i][2])
            
            marker.scale.x = float(boxes[i][3])
            marker.scale.y = float(boxes[i][4])
            marker.scale.z = float(boxes[i][5])
            
            q = Quaternion()
            q.z = np.sin(boxes[i][6] / 2.0)
            q.w = np.cos(boxes[i][6] / 2.0)
            marker.pose.orientation = q
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            
            marker.lifetime = rospy.Duration(0.12)
            marker_array.markers.append(marker)

        self.pub_detections.publish(marker_array)

if __name__ == '__main__':
    try:
        node = OpenPCDetROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
