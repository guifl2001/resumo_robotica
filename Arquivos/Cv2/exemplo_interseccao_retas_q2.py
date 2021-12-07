#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np
import math

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
video = "../../robot20/media/lines.mp4"

def check_exists_size(name, size):
    """
        Função para diagnosticar se os arquivos estão com problemas
    """
    if os.path.isfile(name):
        stat = os.stat(name)
        print("Informações do arquivo ", name, "\n", stat)
        if stat.st_size !=size:
            print("Tamanho errado para o arquivo ", name, " Abortando ")
            mensagem_falta_arquivos()
            sys.exit(0)
    else:
        print("Arquivo ", name, " não encontrado. Abortando!")
        mensagem_falta_arquivos()
        sys.exit(0)

def mensagem_falta_arquivos():
    msg = """ls
    ls

    Tente apagar os arquivos em robot20/ros/exemplos_python/scripts:
         MobileNetSSD_deploy.prototxt.txt
         MobileNetSSD_deploy.caffemodel
    Depois
        No diretório robot20/ros/exemplos_python/scripts fazer:
        git checkout MobileNetSSD_deploy.prototxt.txt
        Depois ainda: 
        git lfs pull 

        No diretório No diretório robot20/media

        Fazer:
        git lfs pull

        Ou então baixe os arquivos manualmente nos links:
        https://github.com/Insper/robot20/tree/master/ros/exemplos_python/scripts
        e
        https://github.com/Insper/robot20/tree/master/media
    """
    print(msg)

def processa_imagem(frame):
    img = frame.copy()
    img2 = frame.copy()



    red = img[:,:,2]
    blue = img2[:,:,1]

    newFrame = cv2.cvtColor(blue, cv2.COLOR_GRAY2BGR)

    lim, red_lim = cv2.threshold(red, 230,255, cv2.THRESH_BINARY)
    mask_red = red_lim

    lim2, blue_lim = cv2.threshold(blue, 230, 255,cv2.THRESH_BINARY)
    mask_blue = blue_lim


    lines_red = cv2.HoughLinesP(mask_red, 10, math.pi/180, 100, np.array([]),5,5)
    a,b,c = lines_red.shape

    lines_blue = cv2.HoughLinesP(mask_blue, 10, math.pi/180, 100, np.array([]),5,5)
    a2,b2,c2 = lines_blue.shape

    m = []
    m1 = []
    m2 = []

    slope=0
    slope2=0
    x_int  = 0
    h1=0
    h2=0 

    for i in range(a):
        if (lines_red[i][0][2]-lines_red[i][0][0]) !=0:
            slope = ((float(lines_red[i][0][3])-float(lines_red[i][0][1]))/(float(lines_red[i][0][2])-float(lines_red[i][0][0])))
            h1 = ((float(lines_red[i][0][1]))-slope*float(lines_red[i][0][0]))
            cv2.line(newFrame, (lines_red[i][0][0],lines_red[i][0][1]), (lines_red[i][0][2], lines_red[i][0][3]), (0,0,255), 3, cv2.LINE_AA)
            
        else:
            slope=100000

    for i in range(a2):
        if (lines_blue[i][0][2]-lines_blue[i][0][0]) !=0:
            slope2 = ((float(lines_blue[i][0][3])-float(lines_blue[i][0][1]))/(float(lines_blue[i][0][2])-float(lines_blue[i][0][0])))
            h2 = ((float(lines_blue[i][0][1]))-slope*float(lines_blue[i][0][0]))
            cv2.line(newFrame, (lines_blue[i][0][0],lines_blue[i][0][1]), (lines_blue[i][0][2], lines_blue[i][0][3]), (255,0,0), 3, cv2.LINE_AA)
            

        else:
            slope2=100000


    if (slope-slope2) !=0:
        x_int = int((h2-h1)/abs(slope-slope2))
     
    y_int = abs(int(slope*x_int+h1))

    print(x_int,y_int)

    if x_int in range(50,300) and y_int in range(100,300):
        cv2.circle(newFrame, (x_int,y_int), 10, (0,255,0), -1)
    cv2.imshow('newfrmae', newFrame)






if __name__ == "__main__":


    # Checando se os arquivos necessários existem
    check_exists_size(video, 942014)

    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)

    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")


    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            #print("Codigo de retorno FALSO - problema para capturar o frame")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
            #sys.exit(0)


        processa_imagem(frame)



        cv2.imshow('imagem', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


