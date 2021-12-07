#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

# GABARITO. Este código pode ser visto em 
# https://youtu.be/pEW6QEdeqSU
# Se você for meu aluno, pode ver um vídeo explicando a construção desta solução em:
# https://web.microsoftstream.com/video/157cc69f-8814-4af7-94f6-7b660d70d8dd
#

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
# Se ainda não tiver baixe o vídeo em https://github.com/Insper/robot20/raw/master/media/cartas.mp4
video = "cartas.mp4"

def conta_contornos(mask, title):
    """Recebe uma imagem binaria e conta quantos contornos"""
    contornos, arvore = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)     
    desenha_contorno(mask, contornos, title)
    return len(contornos)

def desenha_contorno(mask, contornos, title): 
    mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(mask_rgb, contornos, -1, [255, 0, 0], 5);
    cv2.imshow(title, mask_rgb)

def processa(img):
    # recortar
    # pontos 
    # x,y (437, 170) ate (832, 565) - limites da área do naipe. Pegamos no Gimp
    cut = img[170:565, 437:832, :]
    

    reg = cut[:,:,2] # A imagem é BGR. 2 é o canal vermelho


    # segmentar vermelho e preto 

    # limiariza vermelho
    lim, reg_limiar = cv2.threshold(reg, 240, 255, cv2.THRESH_BINARY)
    #cv2.imshow("R limiar", reg_limiar)
    mask_red = reg_limiar
    # Este blur foi necessário porque devido à compactação MP4
    # havia falhas nas bordas dos contornos vermelhos
    # Poderíamos também ter usado operações morfológicas
    # de erosão ou fechamento conforme 
    # https://github.com/Insper/robot202/blob/master/aula02/aula02_Exemplos_Adicionais.ipynb
    mask_red = cv2.blur(mask_red, (3,3))


    gray = cv2.cvtColor(cut, cv2.COLOR_BGR2GRAY)
    lim, gray_limiar  = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
    
    # BITWISE not
    # Precisamos inverter a imagem!
    # Onde está preto precisa ficar branco. 
    # Isso é feito com o Bitwise not
    # Pode ser feito com o operador ~
    # cv2.imshow("GRay limiar", ~gray_limiar)
    # Podemos ainda usar 255 - matriz inteira
    # ou ainda podemos usar np.invert(mask_black)
    mask_black = 255-gray_limiar
    #cv2.imshow("GRay limiar", mask_black)

    # contar quantas figuras tem
    blacks = conta_contornos(mask_black, "BLACK")
    reds = conta_contornos(mask_red, "RED")

    font = cv2.FONT_HERSHEY_SIMPLEX

    if reds > blacks:        
        cv2.putText(img,'{} de Ouros'.format(reds),(20,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    else:
        cv2.putText(img,'{} de Paus'.format(blacks),(20,50), font, 1,(255,255,255),2,cv2.LINE_AA)





if __name__ == "__main__":



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

        # print(frame.shape) para saber o tam da imagem

        processa(frame)
        cv2.imshow('imagem', frame)

        # Lembre-se que *sempre* precisa ter um waitKey na sequência de imshow
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()