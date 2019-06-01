# import the necessary packages

from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import bluetooth
import argparse
import imutils
import time
import cv2
import numpy as np
import os.path as osp
import os
import threading
import serial
import math
from hpd import HPD


default_xAngle = 60
delimiter = "/"
RESIZE_RATIO = 1.4 
SKIP_FRAMES = 3 
JAVARA_TERMINATE = False
hpd = None
default_zAngle = 60

def main(args) :
    global JAVARA_TERMINATE
    inches = 0
    port = 1

    server_socket=bluetooth.BluetoothSocket( bluetooth.RFCOMM ) 
    server_socket.bind(("",port))
    server_socket.listen(1)

    client_socket,address = server_socket.accept()
    print("Accepted connection from ",address)

    while True:
        data = client_socket.recv(1024)
        print("Received: %s" % data)
        # MESSAGE FORMAT : s(start) or q(quit)|inches|
        
        data = data.decode('utf-8')
        
        if data[0] == "q":
            print ("JAVARA STOP !!")
            JAVARA_TERMINATE = True

        if data[0] == "s":
            print("JAVARA START !!")
            inches = data[1:]

            JAVARA_TERMINATE = False
            t1 = threading.Thread(target=tracking, args=(args, inches))
            t1.daemon = True
            t1.start()
        
        if data[0] == "Q" :
            break
 
    client_socket.close()
    server_socket.close()                      

def tracking(args, inches):
    global hpd
    global JAVARA_TERMINATE
    global no_detect_flag
    
    filename = args["input_file"]
    inches = 5.7
    default_zAngle = inches * 2.54 * 4 + 4
    
    img_num = 0


    if filename is None:
        isVideo = False
        
        # created a *threaded *video stream, allow the camera sensor to warmup,
        # and start the FPS counter
        print("[INFO] sampling THREADED frames from `picamera` module...")
        vs = PiVideoStream().start()
        #time.sleep(1.0)
        fps = FPS().start()
        
    else:
        isVideo = True
        cap = cv2.VideoCapture(filename)
        fps = cap.get(cv2.CAP_PROP_FPS)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        name, ext = osp.splitext(filename)
        out = cv2.VideoWriter(args["output_file"], fourcc, fps, (width, height))

    # Initialize head pose detection
    hpd = HPD(args["landmark_type"], args["landmark_predictor"])

    xy_arduino = serial.Serial('/dev/ttyUSB1', 115200)
    pm_arduino = serial.Serial('/dev/ttyUSB0',115200)
    first_arduino = serial.Serial('/dev/ttyACM0', 115200)
    
    time.sleep(0.5)

    cv2.namedWindow('frame2', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('frame2', 320, 240)

    
    while(vs.stopped == False):
        # Capture frame-by-frame
        # start_time = time.time()
        print('\rframe: %d' % fps._numFrames, end='')
        frame = vs.read()

        h, w, c = frame.shape
        new_h = (int)(h / RESIZE_RATIO)
        new_w = (int)(w / RESIZE_RATIO)
        frame_small = cv2.resize(frame, (new_w, new_h))
        frame_small2 = cv2.flip(frame_small, 1) #
        frame_small3 = cv2.flip(frame_small, 0) # 

        
        if isVideo:

            if frame is None:
                break
            else:
                out.write(frame)

        else:

            if (fps._numFrames % SKIP_FRAMES == 0):               
                frameOut, angles, tvec = hpd.processImage(frame_small3)
                if tvec==None:
                    print('\rframe2: %d' % fps._numFrames, end='')        
                    print(" There is no face detected\n")
                    
                    
                    # face detecting!!
                    fifth_angle = 0
                    
                    if no_detect_flag == 0:
                        fourth_angle = -10
                        no_detect_flag += 1
                    elif no_detect_flag == 1:
                        fourth_angle = 23
                        no_detect_flag += 1
                    elif no_detect_flag == 2:
                        fourth_angle = -10
                        no_detect_flag = 0 
                        
                    fourth_fifth_angle = str(int(fourth_angle)) + delimiter + str(int(fifth_angle))
                    fourth_fifth_angle = fourth_fifth_angle.encode('utf-8')
                    print('fourth_fifth_angle', fourth_fifth_angle)
                    pm_arduino.write(fourth_fifth_angle)
                    
                    time.sleep(2.5)
                    fps.update()
                    continue
                    
                else:
                    # tvec: transpose vector
                    # tx : left(+tx) & right(-tx)
                    # ty : up(+ty) & down(-ty)
                    # tz : far(bit tz value) & close(small tz value) :: -tz value
                    tx, ty, tz = tvec[:, 0]
                    rx, ry, rz = angles
                
                
                    ################
                    #### before ####
                    ################
                    # user up (+ty) ->    second_angle(-), fourth_angle(-)
                    # user down (-ty) ->    second_angle(+), fourth_angle(+)
                    
                    # user far (tz bigger than before) -> thrid_angle(+)
                    # user close (tz smaller than before) -> thrid_angle(-)
                    

                    ############
                    ## change ##
                    ############
                    # ty -> third, fourth angle
                    # tz -> second angle
                    
                    # user up (+ty) ->    third_angle(-), fourth_angle(+)
                    # user down (-ty) ->    third_angle(+), fourth_angle(-)
                    # user far (tz bigger than before) -> second_angle(-)
                    # user close (tz smaller than before) -> second_angle(+)
                
                    # user go left(tx) -> first_angle
                    # user go right(tx) -> first_angle
                    
                    '''
                    first_angle = str(int(-tx))
                    first_angle = first_angle.encode('utf-8')
                    first_arduino.write(first_angle)
                    
                    print("first_angle: ", first_angle)
                    
                    '''
                    third_angle = ty
                    second_angle = default_zAngle + tz
                    if abs(second_angle) <= 4:
                        second_angle = 0
                    
                        
                    second_third_angle = str(int(third_angle)) + delimiter + str(int(second_angle))
                    second_third_angle = second_third_angle.encode('utf-8')
                    xy_arduino.write(second_third_angle)
                    print('')
                    print("second_angle: ", second_third_angle)
                    
                    
                    
                    fourth_angle = -ty
                    fifth_angle = 0
                    
                    if second_angle >= 20 and second_angle < 25:
                        fourth_angle = 10
                    elif second_angle <= -20 and second_angle > -25:
                        fourth_angle = -10
                    elif second_angle >= 25 and second_angle < 30:
                        fourth_angle = 13
                    elif second_angle <= -25 and second_angle > -30:    
                        fourth_angle = -13
                    elif second_angle >= 30 and second_angle < 35:
                        fourth_angle = 16
                    elif second_angle <= -30 and second_angle > -35:    
                        fourth_angle = -16
                    elif second_angle >= 35:
                        fourth_angle = 19
                    elif second_angle <= -35:    
                        fourth_angle = -19
                        
                    
                    
                    fourth_fifth_angle = str(int(fourth_angle)) + delimiter + str(int(fifth_angle))
                    fourth_fifth_angle = fourth_fifth_angle.encode('utf-8')
                    
                    print('fourth_fifth_angle', fourth_fifth_angle)
                    pm_arduino.write(fourth_fifth_angle)
                    
                    
                    time.sleep(2)
                    img_num += 1
            
            else:
                pass


            # Display the resulting frame
            # elapsed_time = time.time() - start_time
            # cv2.putText(frameOut, "FPS: {:.2f}".format(1 / elapsed_time), ((int)(frameOut.shape[0]/2) - 100,(int)(frameOut.shape[1]/2) - 100),cv2.FONT_HERSHEY_DUPLEX, fontScale=0.6, color=(0,0,255), thickness = 2)
            cv2.imshow('frame2',frameOut)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

##        count += 1
        
        fps.update()

    # When everything done, release the capture
    # stop the timer and display FPS information
    fps.stop()
    print()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    
    # do a bit of cleanup  
    vs.stop()
    if isVideo: out.release()
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', metavar='FILE', dest='input_file', default=None, help='Input video. If not given, web camera will be used.')
    parser.add_argument('-o', metavar='FILE', dest='output_file', default=None, help='Output video.')
    parser.add_argument('-lt', metavar='N', dest='landmark_type', type=int, default=1, help='Landmark type.')
    parser.add_argument('-lp', metavar='FILE', dest='landmark_predictor',
                        default='model/shape_predictor_68_face_landmarks.dat', help="Landmark predictor data file.")
    parser.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
    parser.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
    args = vars(parser.parse_args())
    main(args)

