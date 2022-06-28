import sys
import rosbag
import time
import subprocess
import yaml
import rospy
import os
import argparse
import math
from shutil import move

timeshift = 0.14033

def status(length, percent):
  sys.stdout.write('\x1B[2K') # Erase entire current line
  sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
  progress = "Progress: ["
  for i in range(0, length):
    if i < length * percent:
      progress += '='
    else:
      progress += ' '
  progress += "] " + str(round(percent * 100.0, 2)) + "%"
  sys.stdout.write(progress)
  sys.stdout.flush()


def main(args):
  parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
  parser.add_argument('bagfile', nargs=1, help='input bag file')
  parser.add_argument('--max-offset', nargs=1, help='max time offset (sec) to correct.', default='60', type=float)
  args = parser.parse_args()
    
  # Get bag duration  
  
  bagfile = args.bagfile[0]
  
  info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagfile], stdout=subprocess.PIPE).communicate()[0])
  duration = info_dict['duration']
  start_time = info_dict['start']
  
  orig = os.path.splitext(bagfile)[0] + ".orig.bag"
  
  move(bagfile, orig)
  
  with rosbag.Bag(bagfile, 'w') as outbag:
      
    last_time = time.time()
    for topic, msg, t in rosbag.Bag(orig).read_messages():
        
      if time.time() - last_time > .1:
          percent = (t.to_sec() - start_time) / duration
          status(40, percent)
          last_time = time.time()

      if topic == "/velodyne_points": 
        outbag.write(topic, msg, msg.header.stamp)# if diff < args.max_offset else t)
      elif topic == '/econ_cam_0/image_raw' or topic == '/econ_cam_0/camera_info':
        msg.header.stamp = msg.header.stamp - rospy.Duration(timeshift)
        outbag.write(topic, msg, msg.header.stamp)         

  status(40, 1)
  print("\ndone")

if __name__ == "__main__":
  main(sys.argv[1:])  
