#! /usr/bin/env python
# -*- coding:utf-8 -*-
# Sugerimos rodar com:
# roslaunch my_simulation formas.launch

from __future__ import print_function, division
import rospy

import numpy as np

from numpy import array, uint8

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import math


ranges = None
minv = 0
maxv = 10
centro=[240,320]
distancia = 100
nao_bateu = True
bridge = CvBridge()

# globals that keep blue and green filter
green_img = None
blue_img = None
# global that keeps last image
image = None

## 

goal = "red"  # se este código vai centralizar e se aproximar do azul ou do verde

###

# Código extraído do notebook  que está na pasta Q4, em
#  https://github.com/mirwox/p2/blob/solution/q4/Q4.ipynb

#EXERCICIO COMPLETO DA PROVA SUB 2020 


# Limiares obtidos via color picker
# NOTE QUE ESTES VALORES FORAM EXPANDIDOS EM RELAÇÂO AO NOTEBOOK
g1, g2 = (array([45, 80, 20], dtype=uint8), array([ 65, 255, 255], dtype=uint8))
b1, b2 = (array([105,  80,  20], dtype=uint8), array([125, 255, 255], dtype=uint8))
r1 = np.array([0, 50, 50])
r2 = np.array([8, 255, 255])

def mask(hsv, a1, a2):
    return cv2.inRange(hsv, a1, a2)


def auto_canny(image, sigma=0.5):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    #image = cv2.blur(image, ksize=(5,5)) # blur se necessário
    #cv2.imshow("filter", image)
    #cv2.waitKey(0)
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged


def maior_circulo(mask, color):
    """ Retorna os dados do maior círculo existente na imagem. E uma imagem com este círculo desenhado"""
    tem_circulo = False
    maior_centro = (0,0)
    maior_raio = 0
    
    bordas = auto_canny(mask)
    
    # acumulador levemente ajustado
    circles=cv2.HoughCircles(image=bordas,method=cv2.HOUGH_GRADIENT,dp=2.2,minDist=250,param1=50,param2=100,minRadius=30,maxRadius=250)
    
    
    bordas_bgr = cv2.cvtColor(bordas, cv2.COLOR_GRAY2RGB)

    output =  bordas_bgr

    if circles is not None:        
        circles = np.uint16(np.around(circles))
        
        for i in circles[0,:]:
            tem_circulo = True
            # draw the outer circle
            cv2.circle(output,(i[0],i[1]),i[2],color,2)
            # draw the center of the circle
            cv2.circle(output,(i[0],i[1]),2,color,3)
            if i[2] > maior_raio:
                maior_centro = (int(i[0]), int(i[1]))
                                
    return tem_circulo, maior_centro, maior_raio, output


def processa_circulos_controle(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    blue = mask(hsv, b1,b2)
    green = mask(hsv, g1, g2)

    red = cv2.inRange(hsv, r1, r2)
    red1 = np.array([172, 50, 50])
    red2 = np.array([180, 255, 255]) 
    red += cv2.inRange(hsv, red1, red2) 

    
    cor_b = (255,0,0)
    cor_g = (0,255,0)
    cor_r = (0,0,255)
    tem_blue, centro_b, raio_n, img_b = maior_circulo(blue, cor_b)
    tem_green, centro_g, raio_g, img_g = maior_circulo(green, cor_g)
    tem_red, centro_r, raio_r, img_r = maior_circulo(red, cor_r)
    
    delta_blue = centro_b[0] - 320 # quão afastado está do centro da tela de 640 x 480
    delta_green = centro_g[0] - 320
    delta_red = centro_r[0]-320 # quão afastado está do centro da tela de 640 x 480
    return tem_green, delta_green, img_g, tem_blue, delta_blue, img_b, tem_red, delta_red, img_r

# Variaveis de controle

circle_visible = False
circle_delta = 0


### Fim do trecho inserido no gabarito


def scaneou(dado):
    global leituras
    global minv
    global maxv
    global distancia
    global nao_bateu
    #print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    #rint("Leituras:")
    ranges = np.array(dado.ranges).round(decimals=2)
    minv = dado.range_min 
    maxv = dado.range_max
    for m in ranges[0: 5]:
        if minv < m < maxv:
            if m < distancia:
                distancia = m    
    for m in ranges[355: 360]:
        if minv < m < maxv:  
            if m < distancia:
                distancia = m  
    if dado.ranges[0] <=1:
        nao_bateu=False

    leituras = ranges.copy()              
 
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global circle_visible
    global circle_delta
    global blue_img
    global green_img
    global image
    global red_img
    global state
    global centro

    #print("frame")
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        image = cv_image.copy()
        tem_green, delta_green, img_g, tem_blue, delta_blue, img_b, tem_red, delta_red, img_r = processa_circulos_controle(cv_image)
    
        centro = (160, image.shape[0]/2)

        if goal == "blue":
            circle_visible = tem_blue
            circle_delta = delta_blue
                     
        if goal == "green":
            circle_visible = tem_green
            circle_delta = delta_green

        if goal=="red":
            circle_visible = tem_red
            circle_delta = delta_red
            if circle_delta != 320 and circle_delta!=-320:
                state="CENTRALIZANDO"
            

        blue_img = img_b.copy()
        green_img = img_g.copy()
        red_img = img_r.copy()



    except CvBridgeError as e:
        print('ex', e)

def draw_lidar(cv_image, leituras):
    bot = [256,256] # centro do robô
    escala = 50 # transforma 0.01m em 0.5 px

    raio_bot = int(0.1*escala)
    # Desenha o robot
    cv2.circle(cv_image,(bot[0],bot[1]),raio_bot,(255,0,0),1)

    for i in range(len(leituras)):
        rad = math.radians(i)
        dist = leituras[i]
        if minv < dist < maxv:
            xl = int(bot[0] + dist*math.cos(rad)*50)
            yl = int(bot[1] + dist*math.sin(rad)*50)
            cv2.circle(cv_image,(xl,yl),1,(0,255,0),2)
            


if __name__=="__main__":

    rospy.init_node("q4")

    topico_imagem = "/camera/rgb/image_raw/compressed"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    delta_tolerance = 5
    dist_tolerance = 0.1
    metro = 1.0


    rot = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
    frente = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
    
    PROCURANDO, CENTRALIZANDO, AVANCANDO, PARADO = 1,2,3,4

    state = PROCURANDO

    # Evitando bugs em algun setups
    velocidade_saida.publish(zero)
    rospy.sleep(1.0)

    dt = 0.05

    while not rospy.is_shutdown():


        branco_bgr = np.zeros(shape=[512, 512, 3], dtype=np.uint8)

        if leituras is not None:
            draw_lidar(branco_bgr, leituras)
            cv2.imshow("LIDAR",branco_bgr)
            cv2.waitKey(1)
        if image is not None:
            cv2.imshow("Camera ", image)
            cv2.waitKey(1) 
        if goal == "blue":
            if blue_img is not None:
                cv2.imshow("Blue ", blue_img)
                cv2.waitKey(1) 
        if goal == "green":
            if green_img is not None:
                cv2.imshow("Green ", green_img)
                cv2.waitKey(1) 
        if goal=='red':
            if red_img is not None:
                cv2.imshow("red ", red_img)
                cv2.waitKey(1) 
                velocidade_saida.publish(rot)
                if state == "CENTRALIZANDO":
                    print(circle_delta)
                    
                    if circle_delta> 0 + delta_tolerance:
                        print('esquerda')
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.05))
                        velocidade_saida.publish(vel)
                        rospy.sleep(0.05)
                    elif circle_delta < 0- delta_tolerance:
                        print('direita')
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.05))
                        velocidade_saida.publish(vel)
                        rospy.sleep(0.05)
                    if circle_delta - delta_tolerance< 0  < circle_delta + delta_tolerance:
                        state= "AVANCANDO" 
                        if nao_bateu:
                            vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.01)
                        else:
                            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                            velocidade_saida.publish(vel)
                            rospy.sleep(0.01)

                    
        
        else:
            velocidade_saida.publish(rot)  
        rospy.sleep(dt)