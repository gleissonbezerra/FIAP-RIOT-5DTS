##################
##### Este código está em Micropython e é compativel com a PLACA ARDUINO NICLA VISION
#####
##### Deve ser utilizado na placa para capturar as imagens de APRIL TAGS e estimativa de posição global da câmera
##################

#ulab é uma lib que simula relativamente bem o NUMPY em Micropython e já está disponível no Firmware. Entretanto, não foram feitos testes exaustivos podendo apresentar diferenças.
from ulab import numpy as np
import json

import sensor, image, time


#configura os parâmtetros de streaming da câmera embarcada
sensor.reset()

#sensor.set_pixformat(sensor.GRAYSCALE)  # or sensor.RGB565
sensor.set_pixformat(sensor.RGB565)

sensor.set_framesize(sensor.QQVGA) #QVGA (160x120)
sensor.set_auto_gain(False) #Para facilitar a captura das TAGs
sensor.set_auto_whitebal(False) #Para facilitar a captura das TAGs

#tamanho do sensor em pixels
image_width = 160
image_heigth = 120

#Carrega o arquivo gerado pelo calibrador no PC
with open('nicla.json', 'r') as json_file:
    camera_data = json.load(json_file)

#carre a matriz intrísica da camera gerada pelo calibrador
mtx = np.array(camera_data["mtx"])
#a matriz de distorção não foi utilizada, mas está dispónivel no arquivo
#dist = np.array(camera_data["dist"])

#Carrega os valores específicos da matriz
f_x = mtx[0][0]
f_y = mtx[1][1]
c_x = mtx[0][2]
c_y = mtx[1][2]

print(f_x, f_y)
print(c_x, c_y)

##
# FUnção que calcula pontos 3D relativos ao centro 3D da TAG
##
def calculate_corners(alfa, tag_size, tag_center):

    height, width = tag_size, tag_size

    rotation_y = np.array([
        [np.cos(alfa), 0, -np.sin(alfa)],
        [0, 1, 0],
        [np.sin(alfa), 0, np.cos(alfa)]
    ])

    top_left = np.array([-width / 2, height / 2, 0])
    top_right = np.array([width / 2, height / 2, 0])
    bottom_left = np.array([-width / 2, -height / 2, 0])
    bottom_right = np.array([width / 2, -height / 2, 0])

    return np.dot(rotation_y, top_left) + tag_center, np.dot(rotation_y, top_right) + tag_center, np.dot(rotation_y, bottom_right) + tag_center, np.dot(rotation_y, bottom_left) + tag_center

##
# Função de conversão de escala em X para metros calibrada com base em medições feitas e nas informações da matriz intrínca. Testes não exaustivos.
##
def metersX(input):
    return (6 * input) /100.

##
# Função de conversão de escala em Z para metros calibrada com base em medições feitas e nas informações da matriz intrínca. Testes não exaustivos.
##
def metersZ(input):
    return (6.5 * input) /100.
##
# Função para transformcar a coordenadas locais da TAG em coordenadas global da camera
##
def pose(tx, ty, tz, alfa):

    xtx = np.cos( np.radians(90)-alfa) * tz - np.cos( -alfa) * tx #caluclo do x'
    ztx = np.sin( np.radians(90)-alfa) * tz - np.sin( -alfa) * tx #calculo do z'

    # Normalização do angulo
    angle = np.radians(270)-alfa
    if angle > np.radians(360):
        angle -=  np.radians(360)

    return (xtx, ty, ztx, angle) #y não é alterado pois é zero nessa implementação

##
# Início do código principal
##

#matriz com a coleção de TAGS em ordem de IDs
world_tags = []

world_tags.append(np.array([0./100., 0.0/100., 0./100.])) #somente uma foi utilizada

#matriz para guardar os vértices 3D da TAG
corner_positions = np.zeros((4,3))

while (True):

    #captura um frame. Necessário inverter nos dois eixos devido a posição da câmera no Robô
    img = sensor.snapshot().replace(vflip=True,hmirror=True)

    #Utiliza o detector de tag disponível pora Micropython no firmware passando para ele a matriz intrisica para as projeções corretas
    tags = img.find_apriltags(families=image.TAG36H11, fx=f_x, fy=f_y, cx=c_x, cy=c_y)

    #para cada tag localizada
    for tag in tags:

        # verifica se é um ID válido
        if tag.id() < len(world_tags):

            #recupera as coordenadas fixas da TAG no sistema global
            world_position_tag = world_tags[tag.id()]

            #normaliza o angulo da TAG relativo a camera
            alfa = tag.y_rotation() - np.radians(270)

            #estima os vértices 3D da TAG
            corner_positions[0], corner_positions[1], corner_positions[2], corner_positions[3] = calculate_corners(alfa, 5/100, world_position_tag)
            #print(corner_positions[0], corner_positions[1], corner_positions[2], corner_positions[3])

            #Estima a posição da câmera no sistema global utilizando o sistema local da TAG
            p = pose(tag.x_translation(), tag.y_translation(), -tag.z_translation(), alfa)

            x = p[0]
            y = p[1]
            z = p[2]

            #Exibe a estimativa da camera na escala final de metros e graus
            print(metersX(x), metersZ(z), np.degrees(p[3]))

            # Desenha elementos para visualizar as estimativas na tela
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))

            if len(tag.corners()) == 4:
                for c in range(len(tag.corners())):
                    if corner_positions[c][2] >= 0: #verifica a coordenada z de cada vértice 3D
                        color = (0,0,255)
                    else:
                        color = (255,0,0)
                    img.draw_circle(tag.corners()[c][0],tag.corners()[c][1],5,color)
