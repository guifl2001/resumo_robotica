#! /usr/bin/env python
# -*- coding:utf-8 -*-

"""
O código para este exercício está em: `sub202/scripts/Q2.py`

Para rodar, recomendamos que faça:

    roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch

Depois:

    rosrun sim202 Q2.py
"""


from __future__ import division, print_function


import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf import transformations
import sys

v = 0.2  # Velocidade linear
w = 0.2  # Velocidade angular
nao_bateu = True

dist_frente = 10


def scaneou(dado):
    global dist_frente

    leituras = np.array(dado.ranges).round(decimals=2)
    dist_frente = leituras[0]


def andar(pub, v):
    vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
    pub.publish(vel)

def parar(pub):
    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
    pub.publish(vel)
    rospy.sleep(0.1)

def girar(pub,giro,w):
    tempo=giro/w
    vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
    pub.publish(vel)
    rospy.sleep(tempo)


if __name__ == "__main__":
    rospy.init_node("Q2")


    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    FRENTE = 1
    GIRANDO = 2
    TERMINOU=3

    state = "FRENTE"

    dist_goal = 1.0

    giro = math.radians(90)

    try:
        while not rospy.is_shutdown():
            if state =="FRENTE":
                andar(pub,v)
                if dist_frente < dist_goal:
                    state="GIRANDO"
                    dist_goal-=0.05
                    parar(pub)
            elif state =="GIRANDO":
                girar(pub,giro,w)
                giro-=math.radians(5)
                parar(pub)
                if dist_goal<=0.5 or giro<=math.radians(50):
                    state = "TERMINOU"
                    parar(pub)
                else:
                    state="FRENTE"
            elif state == "TERMINOU":
                parar(pub)

            rospy.sleep(0.05)
            


    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
