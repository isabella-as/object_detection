#!/usr/bin/env python3

import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class KittiPublisherContinuous(Node):
    def __init__(self):
        super().__init__('kitti_publisher_continuous')
        
        # Exact pathways mapped inside your bridge container volume
        self.dataset_dir = "/home_station/my_kitti_data/training/velodyne"
        self.val_txt_path = "/home_station/my_kitti_data/ImageSets/val.txt"
        
        self.pub_cloud = self.create_publisher(PointCloud2, '/kitti/velo/pointcloud', 10)
        
        # Load validation frame sequence list
        if not os.path.exists(self.val_txt_path):
            self.get_logger().error(f"Missing ImageSets layout index file: {self.val_txt_path}")
            return
            
        with open(self.val_txt_path, 'r') as f:
            self.frame_ids = [line.strip() for line in f.readlines() if line.strip()]
            
        self.get_logger().info(f"Loaded {len(self.frame_ids)} sequences from val.txt. Starting streaming loop...")
        self.current_idx = 0
        
        # Publish at 10 Hz (0.1 seconds per frame to mimic real LiDAR rates)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.current_idx >= len(self.frame_ids):
            self.get_logger().info("End of val split reached. Resetting stream loop to beginning!")
            self.current_idx = 0
            
        frame_id = self.frame_ids[self.current_idx]
        bin_path = os.path.join(self.dataset_dir, f"{frame_id}.bin")
        
        if not os.path.exists(bin_path):
            self.get_logger().warn(f"Pointcloud binary missing, skipping frame: {bin_path}")
            self.current_idx += 1
            return

        # Load binary KITTI scan [X, Y, Z, Intensity]
        scan = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
        
        # Create ROS2 PointCloud2 message skeleton
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "velo_link"
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Pack raw array float bytes efficiently
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(scan)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = scan.tobytes()
        
        self.pub_cloud.publish(msg)
        self.get_logger().info(f"Published continuous sequence frame: {frame_id}")
        self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = KittiPublisherContinuous()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
