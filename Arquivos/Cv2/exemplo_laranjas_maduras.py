import cv2
import numpy as np

cap = cv2.VideoCapture('cat.m4v')
#cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    print("Codigo de retorno", ret)

    # Our operations on the frame come here
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    v1 = np.array([50,50,50])
    v2 = np.array([60,255,255])

    selecao = cv2.inRange(hsv, v1, v2)

    selecao_rgb = cv2.cvtColor(selecao, cv2.COLOR_GRAY2BGR)

    dst = cv2.Canny(selecao, 50, 200)
    imagem_out =  cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)


    # HoughCircles - detects circles using the Hough Method. For an explanation of
    # param1 and param2 please see an explanation here http://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
    circles = None
    circles=cv2.HoughCircles(dst,cv2.HOUGH_GRADIENT,4,25,param1=50,param2=200,minRadius=5,maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            print(i)
            # draw the outer circle
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
            cv2.circle(imagem_out,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(imagem_out,(i[0],i[1]),2,(0,0,255),3)

    # Display the resulting frame
    cv2.imshow('frame',imagem_out)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()