__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule


bridge = CvBridge()
nao_bateu = True
cv_image = None
media = []
centro = []
atraso = 1.5E9 

area = 0.0 


check_delay = False 

def scaneou(dado):
	global nao_bateu
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("---------------")
	print("Dist",dado.ranges[0])
	if dado.ranges[0] <= 0.25:
		nao_bateu=False

#VIDEO (LINK) DEMONSTRANDO FUNCIONAMENTO: https://youtu.be/FnpoicRNtR4


#PRECISA DOS ARQUIVOS VISAO MODULE E MNET PARA RODAR, CHECAR NO R20SIM DENTRO DO CAKIN_WS/SRC/P1_SIM


def roda_todo_frame(imagem):
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime 
	delay = lag.nsecs
	
	if delay > atraso and check_delay==True:
		
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		

		media, centro, maior_area =  cormodule.identifica_cor(cv_image) 
		
		depois = time.clock()

		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/camera/rgb/image_raw/compressed" 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	v=0.2
	tempo = 1.0/v
	tol = 25
	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
		
				if nao_bateu:
					if (media[0] > centro[0]+tol):  #se a media do objeto identificado esta a direita do centro, gire a direita
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))

					if (media[0] < centro[0]+tol):  #se a media do objeto identificado esta a esquerda do centro, gire a esquerda
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
				
					if (media[0]-tol < centro[0]<media[0]+tol):
						vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
				else:
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					print("PAROU!!")
					print("------------------")


			velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")