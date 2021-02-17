#!/usr/bin/env python

import rospy
import threading
import numpy as np
import time
import message_filters
import cv2
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import NavSatFix
from tf import transformations
import requests

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

from realm_msgs.msg import Frame
from cv_bridge import CvBridge, CvBridgeError

ADDRESS = "http://127.0.0.1:3002/predictions/vehicle_orientation"


def pose_to_pq(msg):
  p = np.array([msg.position.x, msg.position.y, msg.position.z])
  q = np.array([msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w])
  return p, q

def get_scaled_params(pinhole_msg, new_size):
  ny, nx = new_size
  xscale = nx / float(pinhole_msg.width.data)
  yscale = ny / float(pinhole_msg.height.data)
  fx = pinhole_msg.fx.data * xscale
  fy = pinhole_msg.fy.data * yscale
  cx = pinhole_msg.cx.data * xscale
  cy = pinhole_msg.cy.data * yscale

  return fx, fy, cx, cy



def detect_vehicle(frame_msg, img_msg, depth_msg, pose_msg):
  cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
  imencoded = cv2.imencode(".jpg", cv_img)[1]
  response = requests.post(ADDRESS, data=imencoded.tostring(), headers={'content_type': 'image/jpeg'}, timeout=5)

  if response.status_code == 200:
    res = response.json()
    if res['boxes']:
      print(res)
    for box in res['boxes']:
      px, py = np.rint(box[0:2]).astype(int)
      depth_np = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

      p, q = pose_to_pq(pose_msg.pose)
      Tc2w = transformations.quaternion_matrix(q)
      Tc2w[0:3,3] = p

      fx, fy, cx, cy = get_scaled_params(frame_msg.camera_model, cv_img.shape[0:2])

      z = depth_np[py, px]
      point = np.array([[(px - cx) * z/fx, (py - cy) * z/fy, z, 1]]).T
      point_world = np.dot(Tc2w, point)

      mark_pub.publish(create_marker(*point_world[0:3]))

    if res['boxes']:
      cv_img = draw_rotated_rect(res['boxes'], cv_img)
  img_msg = bridge.cv2_to_imgmsg(cv_img, "bgr8")
  img_pub.publish(img_msg)


def draw_rotated_rect(boxes, img):
  for rotated_box in boxes:
    cnt_x, cnt_y, w, h, angle = rotated_box
    theta = angle * np.pi / 180.0
    c = np.cos(theta)
    s = np.sin(theta)
    rect = [(-w / 2, h / 2), (-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2)]
    # x: left->right ; y: top->down
    rotated_rect = [(s * yy + c * xx + cnt_x, c * yy - s * xx + cnt_y) for (xx, yy) in rect]
    rotated_rect = np.rint(rotated_rect).astype(int)
    for k in range(4):
        j = (k + 1) % 4
        img = cv2.line(img,
            (rotated_rect[k][0], rotated_rect[k][1]),
            (rotated_rect[j][0], rotated_rect[j][1]),
            (0,255,0),
            1
        )

  return img


def callback_frame_depth(frame_msg, img_msg, depth_msg, pose_msg):
  post_thread = threading.Thread(target=detect_vehicle, args=(frame_msg, img_msg, depth_msg, pose_msg,))
  post_thread.start()

marker_id = 0
def create_marker(x, y, z):
  global marker_id
  marker = Marker()
  marker.header.frame_id = "utm"
  marker.header.stamp = rospy.get_rostime()
  marker.ns = "vehicle"
  marker.id = marker_id
  marker.type = Marker.CUBE
  marker.action = Marker.ADD
  marker.pose.position.x = x
  marker.pose.position.y = y
  marker.pose.position.z = z
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0
  marker.scale.x = 5
  marker.scale.y = 5
  marker.scale.z = 5
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 0.0
  marker.color.b = 0.0  
  marker.lifetime = rospy.Duration(0)

  marker_id += 1

  return marker


if __name__ == '__main__':
  rospy.init_node('vehicle_detection', anonymous=False)
  rospy.loginfo("Starting vehicle_detection node")

  bridge = CvBridge()

  img_pub = rospy.Publisher("vehicle_detection", Image, queue_size=5)
  mark_pub = rospy.Publisher("vehicle_marker", Marker, queue_size=5)

  frame_sub = message_filters.Subscriber("/realm/alexa/densification/frame", Frame)
  depth_sub = message_filters.Subscriber("/realm/alexa/densification/depth", Image)
  img_sub = message_filters.Subscriber("/realm/alexa/densification/img", Image)
  pose_sub = message_filters.Subscriber("/realm/alexa/densification/pose/utm", PoseStamped)

  ts = message_filters.ApproximateTimeSynchronizer([frame_sub, img_sub, depth_sub, pose_sub], 10, 5)
  ts.registerCallback(callback_frame_depth)

  # video_thread = threading.Thread(target=monitor_video)
  # video_thread.daemon = True
  # video_thread.start()
  rospy.spin()

    