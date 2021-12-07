#! /usr/bin/env python
# -*- coding:utf-8 -*-
# Sugerimos rodar com:
# roslaunch turtlebot3_gazebo  turtlebot3_stage_4.launch

from __future__ import print_function, division
import rospy

import numpy as np

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math


ranges = None
minv = 0
maxv = 10

def scaneou(dado):
    global ranges
    global minv
    global maxv
    # Prints suprimidos para evitar ruido
    #print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    #print("Leituras:")
    ranges = np.array(dado.ranges).round(decimals=2)
    minv = dado.range_min 
    maxv = dado.range_max
 


def desenha(cv_image):
    """
        Use esta função como exemplo de como desenhar na tela
    """
    cv2.circle(cv_image,(256,256),64,(0,255,0),2)
    cv2.line(cv_image,(256,256),(400,400),(255,0,0),5)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(cv_image,'Boa sorte!',(0,50), font, 2,(255,255,255),2,cv2.LINE_AA)

def draw_lidar(cv_image, leituras):
    if leituras is None:
        return
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


def draw_hough(cv_image):
    img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # O trecho abaixo é copiado direto da aula 2, parte sobre Hough, colocando só ma checagem para None na primeira iteraćão
    lines = cv2.HoughLinesP(img_gray, 10, math.pi/180.0, 100, np.array([]), 45, 5)
    if lines is None:
        print("No lines found")
        return
    a,b,c = lines.shape
    for i in range(a):
        # Faz uma linha ligando o ponto inicial ao ponto final, com a cor vermelha (BGR)
        cv2.line(cv_image, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 1, cv2.LINE_AA)    


### Parte da resposta ####

### Funções trazidas do exemplo da aula 2
def center_of_contour(contorno):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(contorno)
    if M["m00"] > 0.001:
        # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (int(cX), int(cY))
    else:
        return (-1, -1)

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)

### Combinacao de códigos da aula 2
def acha_maior_contorno_centro(gray):
    """ Estamos trabalhando com BGR como cores
        Retorna uma imagem com os contornos desenhados e a coordenada do centro do maior contorno
    """
    contornos, arvore = cv2.findContours(gray.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(bgr, contornos, -1, [255, 0, 0], 1);
    
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    p = (0,0)
    
    maior = None
    maior_area = 0
    for c in contornos:
        area = cv2.contourArea(c)
        if area > maior_area:
            maior_area = area
            maior = c

    if maior is not None:
        p = center_of_contour(maior)      
        cv2.drawContours(bgr, [maior], -1, [0, 0, 255], 2);
        crosshair(bgr, p, 5, (0,255,0))
    
    return bgr, p
    


################
        
        
if __name__=="__main__":

    rospy.init_node("le_scan")

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
    giro = Twist(Vector3(0,0,0), Vector3(0,0,0.1))


    cv2.namedWindow("Saida")


    cont = 0

    velocidade_saida.publish(giro)
    
    while not rospy.is_shutdown():
        # Cria uma imagem 512 x 512

        branco_rgb = np.zeros(shape=[512, 512, 3], dtype=np.uint8)
        # Chama funćões de desenho
        draw_lidar(branco_rgb, ranges)

        verde = branco_rgb[:,:,1]
                
        cont_bgr, p = acha_maior_contorno_centro(verde)
        c = (256, 256) # centro da imagem
        crosshair(cont_bgr, c, 5, (0,0,255))
        
        import math

        deltay = p[1]-c[1]
        deltax = p[0]-c[0]
                
        ang = math.degrees(math.atan2(deltax,deltay)) # É deltax/deltay e não o contrário porque queremos o ângulo com a vertical
        # A funcao atan2 evita a divisao por zero decorrente de o denominador ser zero
        cv2.putText(cont_bgr, "angulo: %5.2f"%(ang), (384, 384), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255),1,cv2.LINE_AA)
        
        tol = 1.5 # Tolerância em graus
        if 180 - tol < abs(ang) < 180 + tol:
            velocidade_saida.publish(zero)
            cv2.putText(cont_bgr, "Terminou!", (200, 412), cv2.FONT_HERSHEY_PLAIN, 4, (0, 255, 255),2,cv2.LINE_AA)
            # rospy.sleep(0.5)
        else:
            velocidade_saida.publish(giro)
            
        
        # Imprime a imagem de saida
        cv2.imshow("Saida", cont_bgr)
        cv2.waitKey(1) # TRocamos o 0 por 40 para esperar 40 millisegundos
        rospy.sleep(0.01)