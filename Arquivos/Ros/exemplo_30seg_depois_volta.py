#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Solucao - demo em https://youtu.be/n4Ivbk-uFho

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
import sys


x = None
y = None

contador = 0
pula = 100

# Adicionado no gabarito
alfa = -1

def recebe_odometria(data):
    global x
    global y
    global alfa
    global contador

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos_rad = transformations.euler_from_quaternion(lista)
    angulos = np.degrees(angulos_rad)    

    alfa = angulos_rad[2] # mais facil se guardarmos alfa em radianos

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1

### Funcoes da solucao
import math 

max_linear = 0.2 

max_angular = math.pi/8

def calcula_angulo(alfa, x, y):
    beta = math.atan((y/ x))
    angulo_total = beta + math.pi - alfa 
    return angulo_total


def calcula_dist(x, y):
    hipotenusa = math.sqrt(pow(x,2) + pow(y,2))
    return hipotenusa


if __name__=="__main__":

    rospy.init_node("exemplo_odom")

    t0 = rospy.get_rostime()


    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )


    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)



    while not rospy.is_shutdown():
        print("t0", t0)
        if t0.nsecs == 0:
            t0 = rospy.get_rostime()
            print("waiting for timer")
            continue        
        t1 = rospy.get_rostime()
        elapsed = (t1 - t0)
        print("Passaram ", elapsed.secs, " segundos")
        rospy.sleep(1.0)


        limite = 30
        ### Fica fazendo ajustes sucessivos para ir cada vez mais perto da origem

        if elapsed.secs >= limite:
            print("Retonando a base de ",x, ",", y,", ", math.degrees(alfa))
            ang = calcula_angulo(alfa, x, y)
            dist = calcula_dist(x,y)
            vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,max_angular))
            vel_trans = Twist(Vector3(max_linear,0,0), Vector3(0,0,0))
            zero = Twist(Vector3(0,0,0), Vector3(0,0,0))

            sleep_rot = abs(ang/max_angular)
            sleep_trans = abs(dist/max_linear)

            print(vel_rot, "\n",  sleep_rot)
            velocidade_saida.publish(vel_rot)
            rospy.sleep(sleep_rot)

            print(vel_trans ,"\n", sleep_trans)
            velocidade_saida.publish(vel_trans)
            rospy.sleep(sleep_trans)
            print("Terminou um ciclo")
            velocidade_saida.publish(zero)