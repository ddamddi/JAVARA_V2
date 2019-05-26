# import the necessary packages
from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
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

        if data[0] == "q":
            print ("JAVARA STOP !!")
            JAVARA_TERMINATE = True   

        if data[0] == "s":
            print("JAVARA START !!")
            inches = data[1:]
    
            JAVARA_TERMINATE = False
            t1 = threading.Thread(target=tracking , args= args)
            t1.daemon = True
            t1.start()
        
        if data[0] == "Q" :
            break
 
    client_socket.close()
    server_socket.close()                      

def tracking(args):
    global JAVARA_TERMINATE
    global hpd
    filename = args["input_file"]  
    img_num = 0

    if filename is None:
        # created a *threaded *video stream, allow the camera sensor to warmup,
        # and start the FPS counter
        isVideo = False
        print("[INFO] sampling THREADED frames from `picamera` module...")
        vs = PiVideoStream().start()
        time.sleep(2.0)
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

    if hpd == None : 
        # Initialize head pose detection
        print("[INFO] Initialize Head Pose Detection model...")
        hpd = HPD(args["landmark_type"], args["landmark_predictor"])

    xy_arduino = serial.Serial('/dev/ttyUSB1', 9600)
    pm_arduino = serial.Serial('/dev/ttyUSB0',9600)
    first_arduino = serial.Serial('/dev/ttyUSB2', 9600)
    
    time.sleep(0.5)
    servo_angle = 0

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
                    
                    fps.update()
                    #count += 1
                    continue
                    
                else:
                    tx, ty, tz = tvec[:, 0]
                    rx, ry, rz = angles


                    th = math.radians(servo_angle)
                    # xy angle
                    # tz: : x angle, tx: y angle
                    temp_z = int(tz) + default_xAngle   #temp_z: x angle
                    
                    x_angle = math.sin(th)*temp_z + math.cos(th)*tx
                    y_angle = math.cos(th)*temp_z + math.sin(th)*tx
                    
                    temp_y = int(ty)
                    z_angle = str(temp_y)
                    
                    z_angle = z_angle.encode('utf-8')
                    ry = int(ry)
                    
                    if (abs(ry) <=15):
                        ry=0
                        
                    
                    print("\ntx: ",tx, "\nty: ", ty,"\ntz: ", tz)
                    
                    
                    print("\n\n\nry: ", ry)
                    
                    # user up (+y) ->    +third_angle
                    # user down (-y) ->  -third_angle
                    
                    # user up (+y) ->    -second_angle
                    # user down (-y) ->    +second_angle
                    
                    
                    second_angle = -ty
                    third_angle = ty
                    
                    second_third_angle = str(int(third_angle)) + delimiter + str(int(second_angle))
                    second_third_angle = second_third_angle.encode('utf-8')
                    xy_arduino.write(second_third_angle)
                    print("second_angle: ", second_third_angle)
                    
                    
                    
                    '''
                    first_angle = str(int(-tx))
                    first_angle = first_angle.encode('utf-8')
                    first_arduino.write(first_angle)
                    
                    print("first_angle: ", first_angle)
                    '''
                    
                    '''
                    fourth_angle = -ty
                    fifth_angle = 0
                    fourth_fifth_angle = str(int(fourth_angle)) + delimiter + str(int(fifth_angle))
                    fourth_fifth_angle = fourth_fifth_angle.encode('utf-8')
                    
                    print('fourth_fifth_angle', fourth_fifth_angle)
                    pm_arduino.write(fourth_fifth_angle)
                    '''
                    time.sleep(3)
                    # write XY angle
                    # xy_arduino.write(xy_angle)
                    
                    # write Z angle
                    # z_arduino.write(z_angle)
                    # servo_arduino.write(tempAngle)
                    
                    
                    
                    #cv2.imwrite('detected/%d.jpg' % img_num, frameOut)
                    img_num += 1
            
            else:
                pass


            # Display the resulting frame
            # elapsed_time = time.time() - start_time
            # cv2.putText(frameOut, "FPS: {:.2f}".format(1 / elapsed_time), ((int)(frameOut.shape[0]/2) - 100,(int)(frameOut.shape[1]/2) - 100),cv2.FONT_HERSHEY_DUPLEX, fontScale=0.6, color=(0,0,255), thickness = 2)
            cv2.imshow('frame2',frameOut)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if JAVARA_TERMINATE == TRUE:
                break

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


