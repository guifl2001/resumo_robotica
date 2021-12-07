#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Sugerimos rodar com:
# roslaunch turtlebot3_gazebo  turtlebot3_empty_world.launch 


from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time
from tf import transformations


x = None
y = None
ang_z = None
local_z = None
velocidade_saida = None


contador = 0
pula = 50

def recebe_odometria(data):
    global x
    global y
    global contador
    global ang_z

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    radians = transformations.euler_from_quaternion(lista) #pega o angulo em radiandos
    ang_z = radians[2]     #pega o angulo retornado em radiandos em vez de graus 
    angulos = np.degrees(transformations.euler_from_quaternion(lista))    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1

def segmento(length, v):
    vel = Twist(Vector3(v,0,0), Vector3(0,0,0)) #velocidade para percorrer o segmento 
    delta_t = length/v # define o tempo para percorrer tal distancia 
    velocidade_saida.publish(vel)  
    rospy.sleep(delta_t)



def rotacao(ang_rad, w):
    global ang_z

    angulo_inicial = ang_z  #define o angulo inicial da odometria

    if angulo_inicial < 0.0: #verifica se o angulo eh negativo, se for, transforma para positivo 
        angulo_inicial =  angulo_inicial + 2*3.141592
    
    
    vel_ang = Twist(Vector3(0,0,0), Vector3(0,0,w))  #define a rotacao mas nao eh chamada ainda
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))     # velocidade zero
    
    local_z = ang_z  #necessitamos fazer uma copia e utiliza la para evitar conflito com atualizacao da odometria

    if local_z < 0.0:  #verifica se o angulo eh negativo, se for, transforma para positivo
        local_z= local_z + 2*3.1415

    while (local_z<angulo_inicial+ang_rad):
        print(" ang inicial", angulo_inicial)   #enquanto o angulo recebido pela odometria for menor que o angulo de cada rotacao
        print(" ang ", ang_z)   #enquanto o angulo recebido pela odometria for menor que o angulo de cada rotacao
        print(" ang target", angulo_inicial+ang_rad)   #enquanto o angulo recebido pela odometria for menor que o angulo de cada rotacao

        velocidade_saida.publish(vel_ang)   #roda o robo
        rospy.sleep(0.01)
        local_z = ang_z
        if local_z < 0.0:  #verifica se o angulo eh negativo, se for, transforma para positivo
            local_z= local_z + 2*3.141592
    
    velocidade_saida.publish(zero)          #se o angulo recebido da odometria for maior ou  igual, para o robo 
    rospy.sleep(0.1)



def desenha_poligono(n):
    v=0.25
    w=0.15

    angle_degrees = 360/n #divide o valor 360 pelo numero de lados do poligono
    angle_rad = math.radians(angle_degrees) #passa o angulo para radiandos

    for i in range(n):   #loop ateh chegar no numero de lados, vai percorrer e rodar cada lado
        segmento(1.2,v)
        rotacao(angle_rad,w)



if __name__=="__main__":

    rospy.init_node("q3")


    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)


    rospy.sleep(1.0) # contorna bugs de timing    

    while not rospy.is_shutdown():
        if ang_z is not None:
            desenha_poligono(5) #chama a funcao principal para desenhar o poligono
        rospy.sleep(0.5)    
