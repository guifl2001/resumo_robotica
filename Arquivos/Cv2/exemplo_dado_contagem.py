#!/usr/bin/python
# -*- coding: utf-8 -*-

# GABARITO
# Veja rodando aqui https://youtu.be/mwyIJKPt5E8

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
# Baixe e salve na mesma pasta que este arquivo
# https://github.com/Insper/robot20/raw/master/media/dados.mp4
video = "dados.mp4"

font = cv2.FONT_HERSHEY_SIMPLEX 

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

def texto(img, a, p):
    """Escreve na img RGB dada a string a na posição definida pela tupla p"""
    cv2.putText(img, str(a), p, font,1,(0,255,255),2,cv2.LINE_AA)

def solucao_1(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    bordas = auto_canny(gray)
    bordas_bgr = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
    circles=cv2.HoughCircles(image=bordas,method=cv2.HOUGH_GRADIENT,dp=2.5,minDist=10,param1=200,param2=100,minRadius=5,maxRadius=50)
    bordas_rgb = cv2.cvtColor(bordas, cv2.COLOR_GRAY2RGB)
    output = bordas_bgr

    if circles is not None:        
        circles = np.uint16(np.around(circles))
        print(circles.shape)
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(output,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(output,(i[0],i[1]),2,(0,0,255),3)
        texto(output, "DADO: {}".format(circles.shape[1]), (50,50))
    cv2.imshow("Com circulos", bordas_bgr)


def solucao_2(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    menor = [0, 0, 200]
    maior = [180, 37, 255]
    menor = np.array(menor, dtype=np.uint8)
    maior = np.array(maior, dtype=np.uint8)
    mask = cv2.inRange(hsv, menor, maior)
    #cv2.imshow("Mask sol. 2",mask)
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    cv2.drawContours(mask_bgr, contornos, -1, [0, 0, 255], 3);
    n = len(contornos)
    texto(mask_bgr, "Dados {}".format(n), (50,50) )
    cv2.imshow("contornos", mask_bgr)


if __name__ == "__main__":

    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)
    cap.set(cv2.CAP_PROP_FPS, 3)


    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            #print("Codigo de retorno FALSO - problema para capturar o frame")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
            #sys.exit(0)

        # Our operations on the frame come here
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 2 jeitos diferentes de fazer
        solucao_1(frame)
        solucao_2(frame)
        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
        cv2.imshow('imagem', frame)

        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
