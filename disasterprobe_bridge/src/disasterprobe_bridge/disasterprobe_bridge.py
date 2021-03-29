#!/usr/bin/env python

import rospy
import sys
import cv2
import threading
import socketio
import numpy as np
import time
from std_msgs.msg import Header
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge, CvBridgeError

FPS = 20

capture = None
should_run = True
sio = socketio.Client()

def utc_to_rostime(utc_timestamp):
  sec = utc_timestamp/1000. # millis to seconds
  return rospy.Time.from_sec(sec)

def crop_center(img,cropx,cropy):
  y,x,_ = img.shape
  startx = x//2-(cropx//2)
  starty = y//2-(cropy//2)    
  return img[starty:starty+cropy,startx:startx+cropx]


""" Socketio Events
"""
@sio.event
def connect():
  rospy.loginfo("Connected to socketIO server")

@sio.event
def photoEvent(data):
  timestamp = data['timestamp']
  timeSync.photo_msg(timestamp)

@sio.event
def UAVState(data):
  timestamp = data["timestamp"]
  header = Header(stamp=utc_to_rostime(timestamp))

  heading = Float64(data=data["attitude"]["yaw"])
  heading_pub.publish(heading)

  location = data["aircraftLocation"]
  if location["available"] == "false":
    rospy.logwarn_throttle(20, "GNSS not available.")
  else:
    gnss_msg = NavSatFix(header=header)
    gnss_msg.latitude = location["latitude"]
    gnss_msg.longitude = location["longitude"]
    gnss_msg.altitude = location["altitude"]

    rel_alt = Float64(data=location['altitude'])

    gnss_pub.publish(gnss_msg)
    rel_alt_pub.publish(rel_alt)

@sio.event
def endMission(data):
  global should_run
  should_run = False

  
"""
Video monitoring function
"""
def monitor_video():
  fps_rate = rospy.Rate(FPS)

  while True:
    capture = cv2.VideoCapture(cv_input)
    fps = capture.get(5)
  
    # Check that we actually have a valid video source
    if fps == 0.0:
      rospy.logwarn("Video source %s not found, retrying..." % cv_input)
      capture.release()
      rospy.sleep(5)
    else:
      rospy.loginfo("VideoCapture connected")
      break

  bridge = CvBridge()

  while True:
    ret, frame = capture.read()
    if not ret:
      continue

    if resize_video:
      frame = crop_center(frame, resize_width, resize_height)
      
    gray = cv2.cvtColor(crop_center(frame, 100, 100), cv2.COLOR_BGR2GRAY)
    if np.average(gray) < 20:
      timeSync.black_frame_detected()
    else:
      try:
        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")

        # compensate video timestamp for streaming delay
        corrected_time = rospy.Time.now() - timeSync.get_delay()
        image_msg.header = Header(stamp=corrected_time)
        image_pub.publish(image_msg)
      except CvBridgeError, err:
          rospy.logerr(err)

    # fps_rate.sleep()

class TimeSync:
  max_delay = rospy.Duration(5)
  device_delay = rospy.Duration(1) 
  prev_device_time = None
  prev_photo_call = rospy.Time()

  def photo_msg(self, timestamp):
    now = rospy.Time.now()

    if now - self.prev_photo_call < rospy.Duration(2):
      return
    self.prev_photo_call = now

    if (self.prev_device_time == None or now - self.prev_device_time > self.max_delay):
      self.prev_device_time = utc_to_rostime(timestamp)

  def black_frame_detected(self):
    now = rospy.Time.now()
    if (self.prev_device_time != None and now - self.prev_device_time < self.max_delay):
      self.device_delay = rospy.Time.now() - self.prev_device_time

      rospy.loginfo("Device delay: %.3f" % self.device_delay.to_sec())
      self.prev_device_time = None

  def get_delay(self):
    return self.device_delay # difference between ros and device time


def cleanup():
  rospy.loginfo("Shutting down node")
  if capture is not None:
    capture.release()
                  
if __name__ == '__main__':
  rospy.init_node('disasterprobe_bridge', anonymous=False)
  rospy.loginfo("Starting disasterprobe_bridge")
  rospy.on_shutdown(cleanup)
  timeSync = TimeSync()


  # Get params
  cv_input = rospy.get_param("~video_input")
  resize_video = rospy.get_param("~resize_video", False)
  resize_width = rospy.get_param("~resize_width", 0)
  resize_height = rospy.get_param("~resize_height", 0)

  # Outputs
  image_pub = rospy.Publisher("image", Image, queue_size=5)
  gnss_pub = rospy.Publisher("gnss", NavSatFix, queue_size=5)
  heading_pub = rospy.Publisher("heading", Float64, queue_size=5)
  rel_alt_pub = rospy.Publisher("rel_alt", Float64, queue_size=5)

  video_thread = threading.Thread(target=monitor_video)
  video_thread.daemon = True
  video_thread.start()


  sio.connect("http://localhost:3004", headers={'clienttype': 'ROS'})

  while should_run:
    time.sleep(1)