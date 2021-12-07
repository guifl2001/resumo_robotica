def scaneou(dado):
    global nao_bateu
    global dist
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("---------------")
    print("Dist",dado.ranges[0])
    if dado.ranges[0] <= 0.45:
        nao_bateu=False

    



def roda_todo_frame(imagem):  #para ros + cv2 (vai rodar o frame de acordo com a filtragem em cormodule.py)
    #print("frame")
    global cv_image
    global media
    global centro

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    #print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        #print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv_image2 = cv_image.copy();
        # cv_image = cv2.flip(cv_image, -1) # Descomente se for robo real

        media, centro =  cormodule.identifica_cor(cv_image) #puxa arquivo cormodule.py (centro = centro da image) (media= centro do maior contronro da cor desejada) (maior_area= tamanaho da area d maior contorno)
        

        depois = time.clock()

        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)



def roda_todo_frame(imagem): #outras versao do roda todo frame ser ter que puxar o arquivo do cormodule.py
    global cv_img
    global c_objeto 

    print("frame")

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem,"bgr8")

        bgr = cv_image.copy()

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        mini = np.array([142,50,50])
        maxi = np.array([170,255,255])

        mask = cv2.inRange(hsv, mini, maxi)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cm = center_of_mass(mask)

        crosshair(mask_bgr, cm, 15, color=(0,0,255))

        cimg = (int(cv_image.shape[1]/2), int(cv_image.shape[0]/2))

        crosshair(mask_bgr, cimg, 25, color=(0,255,0))

        cv2.imshow("Mascara", mask_bgr)

        c_img = cimg
        c_objeto = cm 

        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex',e)




def center_of_mass(mask):
    M = cv2.moments(mask)

    if M["m00"]==0
        M["m00"]=1
    cX = int(M["m10"]/M["m00"])
    cY = int(M["m01"]/M["m00"])

    return [int(cX), int(cY)]


def crosshair(img, point, size, color):
    """Desenha um crosshair centrado em point
       point deve ser uma tuple (x,y)
       color eh uma tuple rgb uint8 """

    x,y = point
    cv2.line(img, (x - size, y), (x + size, y), color, 5)
    cv2.line(img, (x, y - size), (x, y + size), color, 5)


low = np.array([22,50,50])
high = np.array([36,255,255])
def filter_color(bgr, low, high):   #filtra a cor no range dado (cuidado pois nao tem limiarizacao nem blur apos)
    hsv=cv2.cvtColor(bgr, cvt.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    return mask

def processa(frame_bgr):  #para exercicio de lente e camera (olhar P2 2020 gabarito )
    """ Recebe um frame BGR e mede a distância em pixels entre os círculos verde e amarelo presentes na imagem"""    
    
    # Isolating the yellow circle
    ret, y = cv2.threshold(frame_bgr[:,:,2], 230, 255, cv2.THRESH_BINARY) # Veja https://github.com/mirwox/revisao2020
    f = frame_bgr.copy()
    
    # isolating the green circle
    g_in = f[:,:,1]&(~f[:,:,2])
    ret,g = cv2.threshold(g_in, 230, 255, cv2.THRESH_BINARY) 

    # creation of an image for display purposes
    or_img = y|g
    or_bgr = cv2.cvtColor(or_img, cv2.COLOR_GRAY2BGR)
    
    # Finding the contours
    centro_yellow,cont_yellow  = centro_maior_contorno(y)
    centro_green, cont_green = centro_maior_contorno(g)
    
    # Drawing the centers
    crosshair(or_bgr, centro_yellow, 5, (0,0,255))
    crosshair(or_bgr, centro_green, 5, (0,0,255))    
    
    # Drawing contours
    cv2.drawContours(or_bgr, [cont_yellow], -1, [0, 255, 255], 2);    
    cv2.drawContours(or_bgr, [cont_green], -1, [0, 255, 0], 2);        
    
    # Drawing labels
    font = cv2.FONT_HERSHEY_SIMPLEX    
    texto(or_bgr, "Amarelo", centro_yellow)
    texto(or_bgr, "Verde", centro_green)
    
    # Drawing a line between the 2 points
    cv2.line(or_bgr,centro_yellow,centro_green,(0,0,230), 1)
    
    # calculate measurements
    h = dist(centro_yellow,centro_green)
    texto(or_bgr, "h = {:2.1f}".format(h), middle(centro_yellow, centro_green))        
    return or_bgr


def processa_dist(frame_bgr, f, H):
    """ Recebe um frame BGR e mede a distância em pixels entre os círculos verde e amarelo presentes na imagem"""            
    
    # Isolating the yellow circle
    ret, y = cv2.threshold(frame_bgr[:,:,2], 230, 255, cv2.THRESH_BINARY) # Veja https://github.com/mirwox/revisao2020
    img = frame_bgr.copy()
    
    # isolating the green circle
    g_in = img[:,:,1]&(~img[:,:,2])
    ret,g = cv2.threshold(g_in, 230, 255, cv2.THRESH_BINARY) 

    # creation of an image for display purposes
    or_img = y|g
    or_bgr = cv2.cvtColor(or_img, cv2.COLOR_GRAY2BGR)
    
    # Finding the contours
    centro_yellow,cont_yellow  = centro_maior_contorno(y)
    centro_green, cont_green = centro_maior_contorno(g)
    
    # Drawing the centers
    crosshair(or_bgr, centro_yellow, 5, (0,0,255))
    crosshair(or_bgr, centro_green, 5, (0,0,255))    
    
    # Drawing contours
    cv2.drawContours(or_bgr, [cont_yellow], -1, [0, 255, 255], 2);    
    cv2.drawContours(or_bgr, [cont_green], -1, [0, 255, 0], 2);        
    
    # Drawing labels
    font = cv2.FONT_HERSHEY_SIMPLEX    
    texto(or_bgr, "Amarelo", centro_yellow)
    texto(or_bgr, "Vermelho", centro_green)
    
    # Drawing a line between the 2 points
    cv2.line(or_bgr,centro_yellow,centro_green,(0,0,230), 1)
    
    # calculate measurements
    h = dist(centro_yellow,centro_green)
    texto(or_bgr, "h = {:2.1f}".format(h), middle(centro_yellow, centro_green))        
    
    # Computing distance D
    D = f*H/h
    
    texto(or_bgr, "D={:2.3f}m".format(D), (100,100), thick=1, sz=3 )        
    
    
    
    return or_bgr



# AGORA ENTRAM AS FUNCOES DE HOUGHLINES E CONTORNO

def centro_maior_contorno(mask):
    """Retorna uma tupla com (x,y) do centro do maior contorno da imagem binária passada como entrada.
        Retorna também o contorno para plot
    """
    contornos, arvore = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    maior = -1.0
    maior_c = None
    for c in contornos:            
        a = cv2.contourArea(c) # área
        if a > maior:
            p = center_of_contour(c) # centro de massa
            maior = a
            maior_c = c    
    return p,maior_c


def auto_canny(image, sigma=0.5):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    #image = cv2.blur(image, ksize=(5,5)) # blur se necessário
    #cv2.imshow("filter", image)
    #cv2.waitKey(0)
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged


def conta_contornos(mask, title):
    """Recebe uma imagem binaria e conta quantos contornos"""
    contornos, arvore = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)     
    desenha_contorno(mask, contornos, title)
    return len(contornos)

def desenha_contorno(mask, contornos, title): 
    mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(mask_rgb, contornos, -1, [255, 0, 0], 5);
    cv2.imshow(title, mask_rgb)

def conta_pixels(mask,ponto1,ponto2,txt_color):
    x1,y1 = ponto1
    x2,y2 = ponto2

    font = cv2.FONT_HERSHEY_SIMPLEX

    submask = mask[y1:y2,x1:x2]

    pixels = np.sum(submask)/255
    rgb_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

    return pixels, rgb_mask

def find_extreme_points(image):
    # load the image, convert it to grayscale, and blur it slightly
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    # threshold the image, then perform a series of erosions +
    # dilations to remove any small regions of noise
    thresh = cv2.threshold(gray, 45, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=2)
    # find contours in thresholded image, then grab the largest
    # one
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)

    # determine the most extreme points along the contour
    extLeft = tuple(c[c[:, :, 0].argmin()][0])
    extRight = tuple(c[c[:, :, 0].argmax()][0])
    extTop = tuple(c[c[:, :, 1].argmin()][0])
    extBot = tuple(c[c[:, :, 1].argmax()][0])

    # draw the outline of the object, then draw each of the
    # extreme points, where the left-most is red, right-most
    # is green, top-most is blue, and bottom-most is teal
    cv2.drawContours(image, [c], -1, (0, 255, 255), 2)
    cv2.circle(image, extLeft, 8, (0, 0, 255), -1)
    cv2.circle(image, extRight, 8, (0, 255, 0), -1)
    cv2.circle(image, extTop, 8, (255, 0, 0), -1)
    cv2.circle(image, extBot, 8, (255, 255, 0), -1)
    # show the output image
    cv2.imshow("Image", image)
    cv2.waitKey(0)

    return extRight, extBot, extLeft, extTop


def find_box_points(mask):
   
    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contornos:
        area = cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        # convert all coordinates floating point values to int
        box = np.int0(box)
        # draw a green 'nghien' rectangle
        if area>500:
            cv2.drawContours(mask, [box], 0, (0, 255, 0),1)
            print([box])


zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
dire = Twist(Vector3(0,0,0), Vector3(0,0,-w))
esq = Twist(Vector3(0,0,0), Vector3(0,0,w))
frente = Twist(Vector3(v,0,0), Vector3(0,0,0))