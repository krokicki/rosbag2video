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

image_topic = None

def filter_image_msgs(topic, datatype, md5sum, msg_def, header):
    global image_topic
    print "%s (%s)" % (topic, datatype)
    if datatype=="sensor_msgs/CompressedImage":
        if (opt_topic != "" and opt_topic == topic) or opt_topic == "":
            image_topic = topic
            print "############# USING ######################"
            print "Using topic %s with datatype %s" % (topic,datatype)
            return True
    if datatype=="theora_image_transport/Packet":
        if (opt_topic != "" and opt_topic == topic) or opt_topic == "":
            print "Using topic %s with datatype %s" % (topic,datatype)
            print 'Theora format is not supported'
            return False
    if datatype=="sensor_msgs/Image":
        if (opt_topic != "" and opt_topic == topic) or opt_topic == "":
            image_topic = topic
            print "Using topic %s with datatype %s" % (topic,datatype)
            return True
    if topic=="/mavros/global_position/global":
        return True
    if topic=="/mavros/battery":
        return True
    return False

frame = 0
def process_frame(topic, msg, t, pix_fmt=None):
    global out_file, t_first, t_video, t_file, cv_image, frame, telemetry

    height, width, channels = cv_image.shape 
    if not topic in t_first:
        t_first[topic] = t
        t_video[topic] = 0
        t_file[topic] = 0
    t_file[topic] = (t-t_first[topic]).to_sec()
    # Catch up to the current place in the bag by repeating the same frame

    dup = 0
    while t_video[topic] < t_file[topic]:
        #print "  t_video[topic]=",t_video[topic]," < t_file[topic]=",t_file[topic]
        if not topic in p_avconv:
            if pix_fmt:
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                p_avconv[topic] = cv2.VideoWriter('%s'%(out_file), fourcc, opt_fps, (width, height))
                #size = str(msg.width)+"x"+str(msg.height)
                #p_avconv[topic] = subprocess.Popen(['avconv','-r',str(opt_fps),'-an','-f','rawvideo','-s',size,'-pix_fmt', pix_fmt,'-i','-',out_file],stdin=subprocess.PIPE)
            else:
                p_avconv[topic] = subprocess.Popen(['avconv','-r',str(opt_fps),'-an','-c','mjpeg','-f','mjpeg','-i','-',out_file],stdin=subprocess.PIPE)

        frame = frame + 1
        img = cv_image
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        xoffset = 10
        yoffset = 80
        linespacing = 15
        row = 0

        def printText(text):
            cv2.putText(img,text,(xoffset,yoffset+linespacing*printText.row),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255),1,cv2.LINE_AA)
            printText.row += 1
        printText.row = 0

        printText("Frame %d"%frame)
        for attr in ("Latitude", "Longitude", "Altitude", "Battery Voltage", "Battery Pct"):
            if attr in telemetry:
                printText("%s: %s" % (attr,telemetry[attr]))

        if dup>2:
            DUP_MSG = "Lost Connection"
            DUP_FONT = cv2.FONT_HERSHEY_DUPLEX
            DUP_FONT_SIZE = 2.0
            DUP_FONT_THICK = 2
            retval, baseline = cv2.getTextSize(DUP_MSG, DUP_FONT, DUP_FONT_SIZE, DUP_FONT_THICK) 
            x = (width - retval[0]) / 2
            y = (height - retval[1]) / 2
            cv2.putText(img,DUP_MSG,(x,y),cv2.FONT_HERSHEY_DUPLEX,DUP_FONT_SIZE,(255,255,0),DUP_FONT_THICK,cv2.LINE_AA)
        dup += 1                

        #img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
        p_avconv[topic].write(img)
        #p_avconv[topic].stdin.write(msg.data)
        t_video[topic] += 1.0/opt_fps
    
        if opt_display_images:
            cv2.imshow(topic, img)
            key=cv2.waitKey(1)
            if key==1048603:
                exit(1);


def parse_jpeg(topic, msg, t):
    global cv_image
    if msg.format.find("jpeg")!=-1:
        #print "Parsing Jpeg"
        if msg.format.find("8")!=-1 and (msg.format.find("rgb")!=-1 or msg.format.find("bgr")!=-1):
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        elif msg.format.find("mono8")!=-1 :
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
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    elif msg.encoding.find("bgr8")!=-1 :
        pix_fmt = "bgr24"
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    elif msg.encoding.find("rgb8")!=-1 :
        pix_fmt = "rgb24"
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

telemetry = {}

for files in range(0,len(opt_files)):
    # First arg is the bag to look at
    bagfile = opt_files[files]
    print "Reading %s" % bagfile
    # Go through the bag file
    bag = rosbag.Bag(bagfile)
    topics = bag.read_messages(connection_filter=filter_image_msgs)
    for topic, msg, t in topics:
        #print topic, 'at', str(t)#,'msg=', str(msg)
        if topic=="/mavros/global_position/global":
            telemetry["Latitude"] = getattr(msg, "latitude")
            telemetry["Longitude"] = getattr(msg, "longitude")
            telemetry["Altitude"] = getattr(msg, "altitude")
        if topic=="/mavros/battery":
            telemetry["Battery Voltage"] = getattr(msg, "voltage")
            telemetry["Battery Pct"] = getattr(msg, "percentage")

        if topic==image_topic:
            try:
                parse_jpeg(topic, msg, t)
            except AttributeError:
                try:
                    parse_mono(topic, msg, t)
                except AttributeError:
                    # maybe theora packet
                    # theora not supported
                    pass

    print "Closing bag"
    bag.close();
    print "Releasing output files"
    for topic in p_avconv:
        p_avconv[topic].release()


