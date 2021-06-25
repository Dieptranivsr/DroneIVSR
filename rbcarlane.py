from gpiozero import Motor
import RPi.GPIO as GPIO
import numpy as np
import cv2
import math
from time import sleep

def constrain(x, a, b):
    if ( x <= a):
        return a
    elif (x >= b):
        return b
    return x

### -------------------- set up mode Robot -------------------- ###
_right_front_wheel = Motor(forward = 4, backward = 14)
_left_front_wheel  = Motor(forward = 17, backward = 18)
_right_back_wheel = Motor(forward = 8, backward = 7)
_left_back_wheel  = Motor(forward = 9, backward = 10)

###--------------------function run Robot---------------------###

def _forward(_speed, _time):                   #forward
     _right_front_wheel.forward(_speed)
     _left_front_wheel.forward(_speed)
     _right_back_wheel.forward(_speed)
     _left_back_wheel.forward(_speed)
     sleep(_time)

def _backward(_speed, _time):                  #backward
     _right_front_wheel.backward(_speed)
     _left_front_wheel.backward(_speed)
     _right_back_wheel.backward(_speed)
     _left_back_wheel.backward(_speed)
     sleep(_time)

def _run(_left_speed, _right_speed, _time):                    # run forward >< backward
    left = abs(_left_speed)
    right = abs(_right_speed)
    if _left_speed >= 0 :
        _left_front_wheel.forward(left)
        _left_back_wheel.forward(left)
    else :
        _left_front_wheel.backward(left)
        _left_back_wheel.backward(left)

    if _right_speed >= 0 :
        _right_front_wheel.forward(right)
        _right_back_wheel.forward(right)
    else :
        _right_front_wheel.backward(right)
        _right_back_wheel.backward(right)
    sleep(_time)

def _turn_right(_speed, _time):                #turn right
     _left_front_wheel.forward(0.25)
     _right_front_wheel.backward(0.1)
     _right_back_wheel.forward(_speed)
     _left_back_wheel.forward(_speed)
     sleep(_time)

def _turn_left(_speed, _time):                 #turn left
     _right_front_wheel.forward(0.25)
     _left_front_wheel.backward(0.1)
     _right_back_wheel.forward(_speed)
     _left_back_wheel.forward(_speed)
     sleep(_time)

def _stop():
     _right_front_wheel.stop()
     _left_front_wheel.stop()
     _right_front_wheel.stop()
     _left_front_wheel.stop()

def pp(dis, para):
    forwardSpeed = 1 - (dis - 400)/120

    # Adjust the left and right speeds
    leftSpeed = constrain(forwardSpeed,-1,1)
    rightSpeed = constrain(forwardSpeed,-1,1)

    # And set the motor speeds
    _run(leftSpeed, rightSpeed, 0.05)

###------------------- image processing -----------------###
def make_coordinate(img,line_parameters):
    slope,intercept=line_parameters[0], line_parameters[1]
    y1=img.shape[0]
    y2=int(y1*(3/5))
    x1=int((y1-intercept)/slope)
    x2=int((y2-intercept)/slope)
    return np.array([x1,y1,x2,y2])

def average_slope_intercept(img,lines):
    left_fit=[]
    right_fit=[]

    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
        # returns slope 1st and y-intercept 2nd
        parameters = np.polyfit((x1,x2),(y1,y2),1)#degree 1
        slope=parameters[0]
        intercept=parameters[1]
        if slope < 0:
            left_fit.append((slope,intercept))
        else:
            right_fit.append((slope,intercept))
    left_fit_average=np.average(left_fit,axis=0)
    right_fit_average=np.average(right_fit,axis=0)
    left_line=make_coordinate(img, left_fit_average)
    right_line=make_coordinate(img, right_fit_average)
    return np.array([left_line, right_line])

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255 #only one color because it is a gray image
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_the_lines(img,lines):
    img=np.copy(img)
    line_tb = []
    dis = 0
    para = 0
    blank_img = np.zeros((img.shape[0],img.shape[1],3), dtype=np.uint8)
    for line in lines:
        x1,y1,x2,y2 = line[0], line[1], line[2], line[3]
        #line(image,pt1,pt2,color,thickness=None,lineType=None,shift=None)
        cv2.line(blank_img,(x1,y1),(x2,y2),(0,255,0),thickness=4)
        line_tb.append([x1,x2,y1,y2])
    t1,t2,t3,t4 = np.average(line_tb, axis = 0)
    #print(str(t1)+"-"+str(t2)+"-"+str(t3)+"-"+str(t4))
    tb1 = (t1+t2)/2
    tb2 = (t3+t4)/2

    cv2.line(blank_img,(int(t1),int(t3)), (int(t2),int(t4)), (255,255,0), thickness=4)
    cv2.circle(blank_img, (int(tb1), int(tb2)), 3, (0,0,255), 5)
    ang1 = 180*(1-math.atan2(t3-t4,t1-t2)/np.pi)
    dis = tb1-320
    img = cv2.addWeighted(img,0.8,blank_img,1,0.0)
    return img,dis,ang1

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)
    flag = 0
    while (cap.isOpened):
        ret, frame = cap.read()
        # grab raw NumPy array representing image - 3D array
        img = frame
        #e1 = cv2.getTickCount()
        try:
            # Find lanes in image region of interest and draw lines on them
            #image dimensions
            height = img.shape[0]
            width = img.shape[1]

            #Bottom half of image
            region_of_interest_vertices = [
                    (0, height),
                    (0, height/2),
                    (width, height/2),
                    (width, height)
            ]

            gray_img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
            #Gaussian Blur will remove noise in image
            #blur = cv2.GaussianBlur(gray_img, (5,5),0)
            #Canny(img,minThreshold,maxThreshold) 1:2 or 1:3
            canny_image = cv2.Canny(gray_img,50,150)  #150
            cropped_image = region_of_interest( canny_image,
                                                np.array([region_of_interest_vertices],
                                                np.int32),)
            lines = cv2.HoughLinesP(cropped_image,  #smaller rho/theta=more accurate longer processing time
                                    rho=6, #number of pixels #6  #1
                                    theta=np.pi/60, #/60   #/180
                                    threshold=160, #160      #20
                                    lines=np.array([]),
                                    minLineLength=40, #40   #20
                                    maxLineGap=25)  #25    #300
            averaged_lines = average_slope_intercept(img,lines)
            image_w_lines,distance,angle1=draw_the_lines(img,averaged_lines)
            #print("distance - "+str(int(distance))+" angle1 - "+str(int(angle1)))

            image_w_lines,distance,angle1=draw_the_lines(img,averaged_lines)

            #e2 = cv2. getTickCount()
            #t = (e2 - e1)/cv2.getTickFrequency()
            #print(t)
            #img1 = cv2.resize(image_w_lines,(360,270))
            #cv2.imshow('show',img1)

            if ( distance <= -100):
                _turn_right(0.25,0.05)
            elif (angle1 <= 83):
                _turn_right(0.25,0.05)
            elif (angle1 > 83 and angle1 < 98):
                _forward(0.2,0.05)
            elif ( distance >= 100 ):
                _turn_left(0.25,0.05)
            elif (angle1 >= 98):
                _turn_left(0.25,0.05)
        except:
            print("An exception occurred")
            _backward(0.2,0.05)
            #print("---------------------")
        #except cv2.error as e:
            # inspect error object
            #print(e)
        #img1 = cv2.resize(image_w_lines,(360,270))
        #cv2.imshow('show',img1)
        # Wait 1 ms and read key input
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
