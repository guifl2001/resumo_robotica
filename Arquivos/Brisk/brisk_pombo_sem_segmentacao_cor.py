#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
video = "logomarca.mp4"


###  Início do código vindo do notebook Q2.ipynb

def make_list(pts):
    """Transforma uma representação de 4 pontos em uma matriz usada 
        pelo cv2 perspectiveTransform em uma lista de tuplas (x,y)        
    """
    # Remova os comentários para entender como os pontos são armazenados na cv2
    # print("Points:\n", pts) # Para salientar como encontrar os pontos
    # print("dst.shape: ", pts.shape)
    l = []
    for p in pts:
        x = p[0][0]
        y = p[0][1]
        l.append((x,y))
    return l
    

def find_box_corners(pts):        
    """ Versao mais didatica """
    l = make_list(pts)
    print("l: ", l)
    x_coords = [p[0] for p in l]
    y_coords = [p[1] for p in l]
    x_min = int(min(x_coords))
    x_max = int(max(x_coords))
    y_min = int(min(y_coords))
    y_max = int(max(y_coords))       
    return ((x_min,y_min),(x_max,y_max))

def find_box_corners2(pts):
    """Encontra os cantos da bounding box
       Versao mais eficiente de finx_box_corners
    """
    return ((min(pts[:,:,0]), min(pts[:,:,1])), (max(pts[:,:,0]), max(pts[:,:,1])))


def find_homography_draw_box(kp1, kp2, img_cena, good):
    
    out = img_cena.copy()
    
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)


    # Tenta achar uma trasformacao composta de rotacao, translacao e escala que situe uma imagem na outra
    # Esta transformação é chamada de homografia 
    # Para saber mais veja 
    # https://docs.opencv.org/3.4/d9/dab/tutorial_homography.html
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    matchesMask = mask.ravel().tolist()


    
    h,w = img_original.shape
    # Um retângulo com as dimensões da imagem original
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

    # Transforma os pontos do retângulo para onde estao na imagem destino usando a homografia encontrada
    dst = cv2.perspectiveTransform(pts,M)
   
    corners = find_box_corners(dst)
        

    # Desenha um contorno em vermelho ao redor de onde o objeto foi encontrado
    img2b = cv2.polylines(out,[np.int32(dst)],True,(255,255,0),5, cv2.LINE_AA)
    
    return img2b, corners


# Número mínimo de pontos correspondentes
MIN_MATCH_COUNT = 10


original_bgr =cv2.imread("pomba_gray.png")
# Versões RGB das imagens, para plot
original_rgb = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2RGB)
# Versões grayscale para feature matching
img_original = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2GRAY)

framed = None

# Cria o detector BRISK
brisk = cv2.BRISK_create()

# Encontra os pontos únicos (keypoints) nas duas imagems
kp1, des1 = brisk.detectAndCompute(img_original ,None)

# Configura o algoritmo de casamento de features que vê *como* o objeto que deve ser encontrado aparece na imagem
bf = cv2.BFMatcher(cv2.NORM_HAMMING)    

found = False

# Essa parte de baixo deve ir para um loop
def match(des1, cena_bgr):      
    global found          
    """ Apenas fatoramos código que muda todo frame na parte de baixo """

    img_cena = cv2.cvtColor(cena_bgr, cv2.COLOR_BGR2GRAY)
    
    kp2, des2 = brisk.detectAndCompute(img_cena,None)


    # Tenta fazer a melhor comparacao usando o algoritmo
    matches = bf.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    framed = cena_bgr
    bbox=((0,0),(0,0)) # Caixa vazia de retorno padrão
    

    if len(good)>MIN_MATCH_COUNT:
        found= True
        # Separa os bons matches na origem e no destino
        print("Matches found")    
        framed, bbox = find_homography_draw_box(kp1, kp2, cena_bgr, good)
    else:
        print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
        found=False
    return framed, bbox, found


tolerancia = 0.25

def processa(frame):
    """Recebe um frame BGR"""
    # Procura o padrao
    rmatch, corners, found = match(des1, frame)

    area_found = area(corners[0], corners[1])
    
    if found:
        cv2.putText(frame, "Encontrado", (corners[0][0], corners[1][1]), cv2.FONT_HERSHEY_COMPLEX, 3, (0,0,255),2,cv2.LINE_AA)
    else:
        cv2.putText(frame, "Nao Encontrei", (100, 100), cv2.FONT_HERSHEY_COMPLEX, 3, (0,0,255),2,cv2.LINE_AA)


    return frame




def area(pt1, pt2):
    return abs((pt1[0] - pt2[0])*(pt1[1] - pt2[1]))

###


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

        # Chamada da função resposta
        saida = processa(frame)
            
        cv2.imshow('Saida', saida)
        cv2.moveWindow('Saida', 100,50)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


