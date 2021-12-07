#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

import math

angulo = 400

def scaneou(dado):
	print("Faixa valida de leituras: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	print(np.array(dado.ranges).round(decimals=2))

	global angulo
	ranges = dado.ranges
	menor = 1000

	for i in range(len(dado.ranges)):
		if dado.range_min < ranges[i] < dado.range_max:
			if ranges[i] < menor:
				menor = ranges[i]
				angulo = i



	


if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	vel = 15

	parado = Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0))
	esquerda =  Twist(Vector3(0.0, 0, 0), Vector3(0, 0, math.radians(vel)))
	direita = Twist(Vector3(0.0, 0, 0), Vector3(0, 0, math.radians(-vel)))
	tol = 10

	while not rospy.is_shutdown():
		if tol < angulo <= 180:
			velocidade_saida.publish(esquerda)
		elif 180 < angulo < 360 - tol:
			velocidade_saida.publish(direita)
		else:
			velocidade_saida.publish(parado)
		rospy.sleep(0.1)