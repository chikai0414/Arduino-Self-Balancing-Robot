import cv2
import numpy as np
import time
import serial

ser=serial.Serial('/dev/ttyUSB0',57600)
#time.sleep(1)
font = cv2.FONT_HERSHEY_COMPLEX
cap = cv2.VideoCapture(0)
cv2.namedWindow('frame')
counttri=0
countrec=0
countcir=0
print("est")
while True:
    ret,img=cap.read()


    #img=cv2.imread("test.jpg")
    #img=cv2.resize(img,(640,480),interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


    ret,thresh =cv2.threshold(gray,100,255,0)
    contours, hierarchy =cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        x,y,w,h=cv2.boundingRect(approx)
        if(cv2.isContourConvex(approx) and w<400 and h<400 and w>80 and h>80 ):
            cv2.drawContours(img,[approx],0,(255,0,0),-1)

            if len(approx) == 3:
                cv2.putText(img,"Tri",(x+10,y+10),font,1,(0))
                counttri=counttri+1
                countrec=0
                countcir=0
            elif 3<len(approx)<8 :
                cv2.putText(img,"Rec",(x+10,y+10),font,1,(0))
                countrec=countrec+1
                counttri=0
                countcir=0
            else:
                cv2.putText(img,"Cir",(x+10,y+10),font,1,(0))
                countcir=countcir+1
                counttri=0
                countrec=0
            roi =img[y:y+h,x:x+w]
            cv2.imshow('roi',roi)
            n=cv2.contourArea(approx)
            if counttri>5 and n>12000 :
                counttri=0
                countrec=0
                countcir=0
                ser.write('l')
                time.sleep(3)
            elif countrec>5 and n>30000:
                counttri=0
                countrec=0
                countcir=0
                ser.write('r')
                time.sleep(3)
            elif countcir>5 and n>30000:
                counttri=0
                countrec=0
                countcir=0
                ser.write('z')
                time.sleep(1000)
                exit()
            print(cv2.contourArea(approx))

    #cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break