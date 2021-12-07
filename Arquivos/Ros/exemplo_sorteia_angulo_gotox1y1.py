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
import sys


def dist(x1,y1, x2,y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx**2 + dy**2)



x = None
y = None

contador = 0
pula = 50

def recebe_odometria(data):
    global x
    global y
    global contador

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1



if __name__=="__main__":

    rospy.init_node("exemplo_odom")

    t0 = rospy.get_rostime()


    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

    # Variaveis da solucao
    import random

    w = 0.3

    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))

    while not rospy.is_shutdown():
        rospy.sleep(1.0)  # sleep para contornar bugs
        # Resposta comeca abaixo

        # sorteia angulo

        alfa = random.uniform(0, 2*math.pi)
        t_rot = alfa/w
        print("Alfa: {} Tempo: {}".format(alfa, t_rot))

        # girar aquele angulo no sentido horario
        vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,-w))
        pub.publish(vel_rot)
        rospy.sleep(t_rot)
        # Acbou o tempo da rotacao
        pub.publish(zero)
        rospy.sleep(0.5)

        x0 = x
        y0 = y
        finished = False
        vel_trans = Twist(Vector3(0.2,0,0), Vector3(0,0,0))

        while not finished:
            pub.publish(vel_trans)
            rospy.sleep(0.05)
            if dist(x0, y0, x, y) >= 1.33:
                finished = True
    

        pub.publish(zero)

        # baseado na odometria, parar depois de andar 1.33m
        print("terminou")
        rospy.sleep(0.5)
        sys.exit(0)