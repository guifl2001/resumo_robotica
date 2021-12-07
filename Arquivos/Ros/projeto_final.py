#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from pickle import PERSID
import rospy
import numpy as np
from math import atan2, pi
import cv2
from sensor_msgs.msg import Image, CompressedImage, LaserScan
import cv2.aruco as aruco
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from Garra import Garra
from tf.transformations import euler_from_quaternion

class Follower:

    def __init__(self, target):
        
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_sub = rospy.Subscriber('/camera/image/compressed',
                                            CompressedImage, 
                                            self.image_callback, 
                                            queue_size=4, 
                                            buff_size = 2**24)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                            Twist, 
                                            queue_size=1)
        
        self.laser_subscriber = rospy.Subscriber('/scan',
                                                LaserScan, 
			                                    self.laser_callback)
        self.odom_subscriber = rospy.Subscriber("/odom", 
                                                Odometry, 
                                                self.odom_callback)
                                                
        
        self.twist = Twist()
        self.laser_msg = LaserScan()
        self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        self.direcao = ["Inicio"]
        self.PERCORRENDO = 'Percorrendo'
        self.CENTRALIZANDO = 'Centralizando'
        self.ATROPELANDO = 'Atropelando'
        self.PEGANDO = 'Pegando'
        self.RETORNANDO = 'Retornando'
        self.PROCURANDO = 'Procurando'
        self.INDO = "Indo"
        self.estado = self.PERCORRENDO
        self.bateu = False
        self.comeco = True
        self.tentou = False
        self.garra = Garra()
        self.x = 0
        self.y = 0
        self.z = 0
        self.target = target
        self.looking_id = None
        self.centro_aruco = [0, 0]
        self.cx_inteiro = -1
        self.cx = -1
        self.cy = -1
        self.h = -1
        self.w = -1
        self.COLORS = {
            'orange': ((0, 200, 200), (8, 255, 255)),
            'blue': ((80, 50, 50), (94, 255, 255)),
            'green': ((50, 100, 100), (75, 255, 255))
        }

        self.lastError = 0
        self.max_vel_linear = 0.2
        self.max_vel_angular = 2.0
        self.hertz = 250
        self.rate = rospy.Rate(self.hertz)



    def retorna_pista(self):
        """
        Método para fazer o robô retornar para o ultimo ponto que esteve na pista.

        Retorno:
            concluído (boolean): booleano que indica se terminou de se reposicionar na pista. 
        """
        chegou = self.dist((self.x, self.y), (self.ultima_posicao)) < 0.2
        posicionou = abs(self.ultimo_angulo - self.theta) < 0.4

        if not chegou:
            inc_x = self.ultima_posicao[0] - self.x
            inc_y = self.ultima_posicao[1] - self.y

            angle_to_goal = atan2(inc_y, inc_x)

            err = angle_to_goal - self.theta
            
            
            err = abs(err)

            print("ERRO: ", err)

            if err > 0.5:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.1
            elif err < -0.5:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.1
            else:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
        else:
            erro = self.ultimo_angulo - self.theta

            print("---------------------------")
            print(f"ERRO: {erro}")
            print("---------------------------")

            if erro >= pi:
                erro -= pi*2
            elif erro < -pi:
                erro += 2*pi
            
            if erro > 0.4:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.2
            elif erro < -0.4:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.2
        return chegou and posicionou

    def vai_pra_ponto(self, ponto):
        """
        Método para fazer o robô ir para um ponto especifico (x, y).

        Parametros:
            ponto (tuple): ponto (x, y) no plano que o robo deve ir.

        Retorno:
            concluído (boolean): booleano que indica se terminou de ir pro ponto.
        """
        chegou = self.dist((self.x, self.y), (ponto[0], ponto[1])) < 0.2

        if not chegou:
            inc_x = ponto[0] - self.x
            inc_y = ponto[1] - self.y

            angle_to_goal = atan2(inc_y, inc_x)

            err = angle_to_goal - self.theta
            
            
            err = abs(err)

            print("ERRO: ", err)

            if err > 0.5:
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.1
            elif err < -0.5:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.1
            else:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
        else:
            self.comeco = True
            return True



    def odom_callback(self, dado):
        """
        Grava nas variáveis x,y,z a posição extraída da odometria
        Atenção: *não coincidem* com o x,y,z locais do drone
        """
        self.x = dado.pose.pose.position.x
        self.y = dado.pose.pose.position.y
        self.z = dado.pose.pose.position.z

        rot_q = dado.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        


    def laser_callback(self, msg):
        """
        Função para o callback do laser, atualiza o atributu laser_msg a partir do sensor.
        """
        self.laser_msg = msg
        self.menor_laser = msg.range_min

    def get_laser(self, pos):
        """
        Função para obter a distancia dada pelo laser em certo ângulo do robô.

        Parametros:
            pos (float): ângulo do robô que a distacia será retornada.
        Retorno:
            distância (float | int): Distancia captada pelo lase no ângulo passado. 
        """
        return self.laser_msg.ranges[pos]


    def segmenta_cor_target(self, hsv):
        """
        Segmenta na imagem a cor passada no target, retorna a mascara.

        Parametros:
            hsv (img): Imagem em hsv que será segmentada.
        Retorno:
            mask (img): Mascara da imagem com a cor segmentada.
        """
        cor = self.target[0]
        min_cor, max_cor = self.COLORS[cor]
        mask = cv2.inRange(hsv, min_cor, max_cor)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones([3, 3]))


    def detecta_aruco(self, gray, frame):
        """
        Função usada para detecção dos arucos da pista.
        
        Parametros:
            gray (img): imagem em gray scale.
            frame (img): Imagem original para ser desenhado o contorno dos arucos e seus ids.
        Retorno:
            centro_aruco (List[int]): ponto (x, y) do centro do aruco.
        
        """
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict) #, parameters=parameters)
        try:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                print(f'ID: {ids[i]}')
                
                for c in corners[i]:
        
                    if ids[i] == self.target[1] or ids[i] == 200 or ids[i] == 100 or ids[i] == 50:
                            self.looking_id = ids[i]
                            centro_aruco_x = (c[0][0] + c[1][0])//2
                            centro_aruco_y = (c[0][1] + c[1][1])//2
                            self.tamanho_aruco = ((c[0][0] - c[1][0])**2 + (c[0][1] - c[1][1])**2)**0.5
                            return [centro_aruco_x, centro_aruco_y]
            return [0, 0]
        except TypeError as erro:
            return [0, 0]


    
    def centraliza_objeto(self, centro_objeto_x, direcao=None):
        """
        Centraliza no objeto passado.

        Parametros:
            centro_objeto_x (float | int): centro no eixo x do objeto a ser centralizado.
            direcao (String): direção que pode ser passada opcionalmente para maior controle da velocidade.
        """
        err = centro_objeto_x - self.w/2
        self.twist.angular.z = -float(err) / 200

        if direcao == 'Esquerda2':
            self.twist.angular.z = 0.6



    def dist(self, p1, p2):
        """
        Calcula distância euclidiana entre dois pontos.

        Parametros:
            p1 (list[int]): ponto (x, y) no plano.
            p2 (list[int]): ponto (x, y) no plano.
        Retorno:
            distancia (float | int) : distância dos pontos.  
        """
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5


    def image_callback(self, msg):
        """
        Método que implementa a visão do robô pela pista, segmenta os componentes de interesse.

        Parametros:
            msg : imagem da camera do robo.
        """
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
            # Para o robo físico
            # cv_image = cv2.flip(cv_image, -1)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Segmenta a cor amarela - para simulacao
            lower_yellow = np.array([22, 50, 50],dtype=np.uint8)
            upper_yellow = np.array([36, 255, 255],dtype=np.uint8)
            # Para o robo fisico
            # lower_yellow = np.array([15, 50, 50],dtype=np.uint8)
            # upper_yellow = np.array([28, 255, 255],dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask_inteira = mask.copy()
            mask_inteira = cv2.morphologyEx(mask_inteira, cv2.MORPH_OPEN, np.ones([3, 3]))

            # Cria a mascara do target
            mask_target = self.segmenta_cor_target(hsv)
            self.centro_aruco = self.detecta_aruco(gray, cv_image)

            cv2.putText(cv_image, f"Estado = {self.estado}", (30, 30), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 255))

            # Corta a imagem, para melhor orientar o robo
            h, w, d = cv_image.shape
            search_top = 3*h//4
            search_bot = 3*h//4 + 20
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            self.w = w
            self.h = h

            M_target = cv2.moments(mask_target)
            
            if M_target['m00'] > 0 and self.looking_id == self.target[1]:
                self.creeper_cx = int(M_target['m10']/M_target['m00'])

                cv2.putText(cv_image, "O creeper target esta visivel", (90, 90), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0))
                if not self.bateu:
                    if self.estado == self.PERCORRENDO:
                        self.ultima_posicao = (self.x, self.y)
                        self.ultimo_angulo = self.theta
                        if self.target[1] == 13:
                            self.ultimo_angulo = self.direcao_inicial
                    if self.target[1] == 21 or self.target == ('blue', 12) or self.target == ('blue', 51) and self.estado != self.PROCURANDO:
                        pass
                    elif self.estado == self.CENTRALIZANDO:
                        pass
                    else:
                        self.estado = self.ATROPELANDO



            M = cv2.moments(mask)
            self.M_inteiro = cv2.moments(mask_inteira)
            
            # Centro de massa da mascara
            self.esta_na_pista = M['m00'] > 0
            if M['m00'] > 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
                cv2.circle(cv_image, (self.cx, self.cy), 10, (0,0,255), -1)

            cv2.imshow("window", cv_image)
            cv2.imshow("Mask", mask_target)
            # cv2.imshow('Mascara', mask_inteira)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)
    
    def control(self):
        """Metodo para o controle do robo pela pista, controla velocidade e estados."""
        # Calcula o erro em relacao ao centro de massa da mascara
        err = self.cx - self.w/2
        # ------controle P simples------
    
        self.twist.linear.x = 0.3
        self.twist.angular.z = -float(err) / 100

        # Condicao para virar nos arucos
        # Primeira curva: esquerda
        if self.dist((0.043225, 0.793308), (self.x, self.y)) < 0.3 and self.direcao[-1] != "Esquerda1":
            self.estado = self.CENTRALIZANDO
            self.direcao.append("Esquerda1")
            
        # Segunda curva: direita
        elif self.dist((-0.43, -2.15), (self.x, self.y)) < 0.15 and self.direcao[-1] != "Direita1" and self.direcao[-1] != "Esquerda2":
            self.estado = self.CENTRALIZANDO
            self.direcao.append("Direita1")
        
        # Ultima curva: Esquerda
        elif self.dist((0.212119, 1.093690), (self.x, self.y)) < 0.3 and self.direcao[-1] != "Esquerda2":
            if self.estado != self.ATROPELANDO:
                self.estado = self.CENTRALIZANDO
            self.direcao.append("Esquerda2")

        # Para os creepers nao visiveis no trajeto
        if self.target[1] == 13 and self.dist((5, -0.440162), (self.x, self.y)) < 0.3 and not self.bateu and not self.tentou:
            self.estado = self.PROCURANDO
        if self.target == ("blue", 12) and self.dist((0.0, 0.0), (self.x, self.y)) < 0.3 and not self.bateu and not self.tentou and self.direcao[-1] in ("Esquerda2", "Esquerda1"):
            self.estado = self.PROCURANDO
        if self.target[1] == 21 and self.target[0] == 'green' and self.dist((-4.917459, -1.297056), (self.x, self.y)) < 0.3 and not self.bateu and not self.tentou:
            self.estado = self.PROCURANDO
        if self.target[1] == 21 and self.target[0] == 'orange' and self.dist((-1.497257, -2.879799), (self.x, self.y)) < 0.3 and not self.bateu and not self.tentou:
            self.estado = self.PROCURANDO
        if self.target == ("blue", 51) and self.dist((3.571676, 1.970857), (self.x, self.y)) < 0.4 and not self.bateu and not self.tentou:
            self.estado = self.INDO
        
        # Para ir para o creeper azul, que esta bem longe
        if self.estado == self.INDO:
            if self.comeco:
                self.ultima_posicao = (self.x, self.y)
                self.ultimo_angulo = self.theta
                self.comeco = False
            chegou = self.vai_pra_ponto((3.767621, 3.772054))
            if chegou:
                self.estado = self.PROCURANDO

        # Para procurar creepers nao visiveis no trajeto
        if self.estado == self.PROCURANDO:
            self.twist.linear.x = 0.0
            if self.comeco:
                self.direcao_inicial = self.theta
                if self.target != ("blue", 51):
                    self.ultima_posicao = (self.x, self.y)
                    self.ultimo_angulo = self.theta
                if self.target[1] == 21:
                    self.twist.angular.z = -0.1
                else:
                    self.twist.angular.z = 0.1
                rospy.sleep(2)
                self.comeco = False
            
            if abs(self.theta - self.direcao_inicial) > 0.1:
                self.twist.angular.z = 0.1
            else:
                self.tentou = True

         

        # Faz curva
        if self.estado == self.CENTRALIZANDO:
            print("-------------------------")
            print("ESTADO = CENTRALIZANDO")
            print("Tamanho do aruco", self.tamanho_aruco)
            print("-------------------------")

 
            if self.direcao[-1] == 'Esquerda2':
                if self.tamanho_aruco > 98:
                    self.estado = self.PERCORRENDO
                self.centro_aruco[0] = self.w * 0.3

            err = self.centro_aruco[0] - self.w / 2


            if abs(err) > 2 and self.tamanho_aruco < 100:
                if self.direcao[-1] == 'Direita1':
                    self.centro_aruco[0] = self.w * 0.7
                
                self.centraliza_objeto(self.centro_aruco[0], direcao=self.direcao[-1])
            else:
                self.estado = self.PERCORRENDO
        
        # Atropela o creeper 
        if self.estado == self.ATROPELANDO:
            self.centraliza_objeto(self.creeper_cx)

            media = self.get_laser(0)


            if 2 < media <= 3:
                self.twist.linear.x = 0.15
            elif 1 < media <= 2:
                self.twist.linear.x = 0.12
            elif 0.5 < media <= 1:
                self.twist.linear.x = 0.10
            elif 0.30 < media <= 0.5:
                self.twist.linear.x = 0.08
            elif media <= 0.30:
                self.twist.linear.x = 0.0
                if abs(self.creeper_cx - self.w / 2) <= 2:
                    self.bateu = True
                    self.estado = self.PEGANDO
                else:
                    err = self.creeper_cx - self.w / 2
                    self.twist.angular.z = -float(err) / 200
                
        
        # Pega o creeper com a garra
        if self.estado == self.PEGANDO:
            # Primeiro centraliza perfeitamente, e depois vai em direcao ao creeper.

            # Deve estar a uma distancia de aprox 0.35
            self.garra.abre_garra()
            self.garra.para_frente()
            rospy.sleep(1.0)
            # Avanca com a garra aberta e centralizado
            media = self.get_laser(0)

            self.twist.linear.x = 0.025

            tamanho_garra = 0.35
            tempo = (media - tamanho_garra) / 0.025
            rospy.sleep(tempo)
            self.twist.linear.x = 0.0
            rospy.sleep(0.5)
            self.garra.fecha_garra()
            rospy.sleep(2.0)
            self.garra.para_cima()
            self.estado = self.RETORNANDO


        # Retorna pra pista
        if self.estado == self.RETORNANDO:
            voltou = self.retorna_pista()
            if voltou:
                self.estado = self.PERCORRENDO
            
   
        # Publica velocidade
        self.cmd_vel_pub.publish(self.twist)
        # rospy.loginfo("linear: %f angular: %f", self.twist.linear.x, self.twist.angular.z)
        self.rate.sleep()

# Main loop
if __name__=="__main__":
    rospy.init_node('follower')
    follower = Follower(('green', 52))

    while not rospy.is_shutdown():
        follower.control()

# END ALL