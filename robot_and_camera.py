import pyrealsense2 as rs
import numpy as np
import cv2 as cv

from  pycreate2 import Create2
import time

port = "COM3"  # where is your serial port?
bot = Create2(port)
bot.start()
bot.safe()

#-------------------------------------------------------------------
pipeline = rs.pipeline() # Create a pipeline

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# We will be removing the background of objects more than
clipping_distance_in_meters = 2 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
align_to = rs.stream.color
align = rs.align(align_to)

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frqames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_250)

        # Initialize the detector parameters using default values
        parameters =  cv.aruco.DetectorParameters_create()

        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(color_image, dictionary, parameters=parameters)

        if np.all(markerIds is not None):  # If there are markers found by detector
            #for i in range(0, len(markerIds)):

            corner1 = markerCorners[0][0]
            x = int(corner1[0][0])
            y = int(corner1[0][1])
            
            dist = depth_image[x][y] * depth_scale
        
            text = '{:.0f} cm'.format(dist*100)
            cv.putText(color_image,text, 
                (x,y), 
                cv.FONT_HERSHEY_SIMPLEX, 
                0.5, (255,0,0), 2)
        
            if dist < 120:
                bot.drive_direct(50, 50)
                time.sleep(1)
                print("Distance= " + str(dist) + " cm. Moving!")
            else:
                bot.drive_stop()
                print("stopped")
    
        # Remove background - Set pixels further than clipping_distance to grey
        #grey_color = 153
        #depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        #bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        #put images next to each other
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))

        cv.namedWindow('Align Example', cv.WINDOW_NORMAL)
        cv.imshow('Align Example', images)
        key = cv.waitKey(1)
        
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv.destroyAllWindows()
            bot.close()
            break
finally:
    pipeline.stop()