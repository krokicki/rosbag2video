#!/usr/bin/env python

"""
rosbag2video.py
Create annotated videos from ROS bags
Based on rosbag2video.py by Maximilian Laiacker 2016
"""

import roslib
roslib.load_manifest('rosbag')
import rosbag
import sys, getopt
import os
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import shlex, subprocess


def filter_image_msgs(topic, datatype, md5sum, msg_def, header):
    print topic
    if datatype=="sensor_msgs/CompressedImage":
        if (opt_topic != "" and opt_topic == topic) or opt_topic == "":
            print "############# USING ######################"
            print "Using topic %s with datatype %s" % (topic,datatype)
            return True;
    if datatype=="theora_image_transport/Packet":
        if (opt_topic != "" and opt_topic == topic) or opt_topic == "":
            print "Using topic %s with datatype %s" % (topic,datatype)
            print 'Theora format is not supported'
            return False;
    if datatype=="sensor_msgs/Image":
        if (opt_topic != "" and opt_topic == topic) or opt_topic == "":
            print "Using topic %s with datatype %s" % (topic,datatype)
            return True;
    return False;


def process_frame(topic, msg, t, pix_fmt=None):
    global out_file, t_first, t_video, t_file, cv_image
    #print "Process frame ",t
    if len(msg.data)>0:
        if not topic in t_first:
            t_first[topic] = t
            t_video[topic] = 0
            t_file[topic] = 0
        t_file[topic] = (t-t_first[topic]).to_sec()
        # Catch up to the current place in the bag by repeating the same frame
        while t_video[topic] < t_file[topic]:
            #print "  t_video[topic]=",t_video[topic]," < t_file[topic]=",t_file[topic]
            if not topic in p_avconv:
                if pix_fmt:
                    size = str(msg.width)+"x"+str(msg.height)
                    p_avconv[topic] = subprocess.Popen(['avconv','-r',str(opt_fps),'-an','-f','rawvideo','-s',size,'-pix_fmt', pix_fmt,'-i','-',out_file],stdin=subprocess.PIPE)
                else:
                    p_avconv[topic] = subprocess.Popen(['avconv','-r',str(opt_fps),'-an','-c','mjpeg','-f','mjpeg','-i','-',out_file],stdin=subprocess.PIPE)

            p_avconv[topic].stdin.write(msg.data)
            t_video[topic] += 1.0/opt_fps
        if opt_display_images:
            cv2.imshow(topic, cv_image)
            key=cv2.waitKey(1)
            if key==1048603:
                exit(1);


def parse_jpeg(topic, msg, t):
    global cv_image
    if msg.format.find("jpeg")!=-1:
        #print "Parsing Jpeg"
        if msg.format.find("8")!=-1 and (msg.format.find("rgb")!=-1 or msg.format.find("bgr")!=-1):
            if opt_display_images:
                np_arr = np.fromstring(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        elif msg.format.find("mono8")!=-1 :
            if opt_display_images:
                np_arr = np.fromstring(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        else:
            print 'Unsupported format:', msg.format
            exit(1)

    process_frame(topic, msg, t)


def parse_mono(topic, msg, t):
    global cv_image
    #print "Parsing Mono"
    pix_fmt=""
    if msg.encoding.find("mono8")!=-1 :
        pix_fmt = "gray"
        if opt_display_images:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    elif msg.encoding.find("bgr8")!=-1 :
        pix_fmt = "bgr24"
        if opt_display_images:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    elif msg.encoding.find("rgb8")!=-1 :
        pix_fmt = "rgb24"
        if opt_display_images:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    else:
        print 'Unsupported encoding:', msg.encoding
        exit(1)

    process_frame(topic, msg, t, pix_fmt=pix_fmt)


# Main program

opt_fps = 25.0
opt_out_file = ""
opt_fourcc = "XVID"
opt_topic = ""
opt_files = []
opt_display_images = False;

def print_help():
    print
    print 'rosbag2video.py [--fps 25] [-o outputfile] [-s (show video)] [-t topic] bagfile1 [bagfile2] ...'
    print
    print 'converts image sequence(s) in ros bag file(s) to video file(s) with fixed frame rate using avconv'
    print 'avconv needs to be installed! (sudo apt-get install libav-tools)'
    print 'if no output file (-o) is given the filename \'<topic>.mp4\' is used and default output codec is h264'
    print 'multiple image topics are supported only when -o option is _not_ used'
    print 'avconv will guess the format according to given extension'
    print 'compressed and raw image messages are supported with mono8 and bgr8/rgb8'
    print 'Maximilian Laiacker 2016'

if len(sys.argv) < 2:
    print 'Please specify ros bag file(s)'
    print 'For example:'
    print_help()
    exit(1)
else :
   try:
      opts, opt_files = getopt.getopt(sys.argv[1:],"hsr:o:c:t:",["fps=","ofile=","codec=","topic="])
   except getopt.GetoptError:
      print_help()
      sys.exit(2)
   for opt, arg in opts:
      if opt == '-h':
         print_help()
         sys.exit()
      elif opt == '-s':
          opt_display_images = True
      elif opt in ("-r", "--fps"):
         opt_fps = float(arg)
      elif opt in ("-o", "--ofile"):
         opt_out_file = arg
      elif opt in ("-c", "--codec"):
         opt_fourcc = arg
      elif opt in ("-t", "--topic"):
         opt_topic = arg
      else:
          print "opz:", opt,'arg:', arg

out_file = None
if opt_out_file=="":
    out_file = str(topic).replace("/", "")+".mp4"
else:
    out_file = opt_out_file

if (opt_fps<=0):
    opt_fps = 1
print "using ",opt_fps," FPS"
t_first = {}
t_file = {}
t_video = {}
cv_image = []

p_avconv = {}
bridge = CvBridge()
for files in range(0,len(opt_files)):
    # First arg is the bag to look at
    bagfile = opt_files[files]
    print "Reading %s" % bagfile
    # Go through the bag file
    bag = rosbag.Bag(bagfile)
    topics = bag.read_messages(connection_filter=filter_image_msgs)
    print topics
    for topic, msg, t in topics:
        #print topic, 'at', str(t)#,'msg=', str(msg)
        try:
            parse_jpeg(topic, msg, t)
        except AttributeError:
            try:
                parse_mono(topic, msg, t)
            except AttributeError:
                # maybe theora packet
                # theora not supported
                pass

    bag.close();
