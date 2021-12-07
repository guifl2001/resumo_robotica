#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np
from numpy import array,uint8

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
model = "../../robot20/ros/exemplos_python/scripts/MobileNetSSD_deploy.caffemodel"
proto = "../../robot20/ros/exemplos_python/scripts/MobileNetSSD_deploy.prototxt.txt"
# video = "../../robot20/media/dogs_table.mp4"
# video = "../../robot20/media/dogs_table_black.mp4"
# video = "../../robot20/media/animacao_bulldogs.mp4"
# video = "../../robot20/media/animacao_bulldogs2.mp4"
video = "../../robot20/media/dogs_chairs.mp4"

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
    msg = """
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

def detect(frame):
    """
        Recebe - uma imagem colorida
        Devolve: objeto encontrado
    """
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence


        if confidence > 0.5:
            # extract the index of the class label from the `detections`,
            # then compute the (x, y)-coordinates of the bounding box for
            # the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # display the prediction
            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            print("[INFO] {}".format(label))
            cv2.rectangle(image, (startX, startY), (endX, endY),
                COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(image, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))

    # show the output image
    return image, results


cy = 800
cx = 0
dogs=[]

def conta_pixels(mask,ponto1,ponto2,txt_color):
    x1,y1 = ponto1
    x2,y2 = ponto2

    font = cv2.FONT_HERSHEY_SIMPLEX

    submask = mask[y1:y2,x1:x2]

    pixels = np.sum(submask)/255
    rgb_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

    return pixels, rgb_mask



def detecta_emcima (dog,chair,frame):
    dog_p1 = dog[2]
    dog_p2=dog[3]
    cx = (dog_p1[0]+dog_p2[0])/2
    cadeira_p1 = chair[2]
    cadeira_p2=chair[3]

    font = cv2.FONT_HERSHEY_SIMPLEX

    if cadeira_p1[0]<cx<cadeira_p2[0] and min(dog_p1[1], dog_p2[1])< max(cadeira_p1[1],cadeira_p2[1]):
        cv2.putText(frame, "CAO EM CIMA DA CADEIRA", (50,50),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        cv2.rectangle(frame, (t[2][0]+10, t[2][1]+10), (t[3][0]+10, t[3][1]+10),
        (0,0,255), 2)
    return frame


yellow = [array([19,50,45], dtype=uint8), array([29,255,255], dtype=uint8)]
magenta = [array([130,80,70], dtype=uint8), array([139,255,255],dtype=uint8)]
dog_magenta = None 
dog_yellow = None 
chair = None 


if __name__ == "__main__":


    # Checando se os arquivos necessários existem
    check_exists_size(proto, 29353)
    check_exists_size(model, 23147564)
    check_exists_size(video, 1313000)

    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)

    # cria a rede neural
    net = cv2.dnn.readNetFromCaffe(proto, model)

    CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
        "sofa", "train", "tvmonitor"]   

    CONFIDENCE = 0.7
    COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))


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

        saida, resultados = detect(frame)

        for t in resultados:
            if t[0] =="dog" or t[0]=="person":
                dogs.append(t)
            if t[0]=="chair":
                chair = t
            
        if len(dogs)>=1:
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask_yellow = cv2.inRange(frame_hsv,yellow[0],yellow[1])
            mask_magenta = cv2.inRange(frame_hsv,magenta[0],magenta[1])

            for d in dogs:
                qt_am,rgb_am = conta_pixels(mask_yellow.copy(), d[2], d[3], "am")
                qt_mg,rgb_mg = conta_pixels(mask_magenta.copy(), d[2],d[3], "mg")

                if qt_am> 50:
                    dog_yellow = d
                if qt_mg>50:
                    dog_magenta = d
                    
            if chair is not None:
                if dog_magenta is not None:
                    detecta_emcima(dog_magenta,chair,frame)
                #if dog_yellow is not None:   #DESCOMENTAR SE QUISER VER A DETETCAO COM OS DOIS CACHORROS EM CIMA DA CADEIRA
                    #detecta_emcima(dog_yellow,chair,frame)
            
            

            
        





        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
        cv2.imshow('imagem', frame)
        cv2.imshow('magenta', mask_magenta)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


