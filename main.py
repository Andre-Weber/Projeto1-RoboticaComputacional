#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError
import cormodule
import visao_module
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import argparse


ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
	help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="kcf",
	help="OpenCV object tracker type")
args = vars(ap.parse_args())


(major, minor) = cv2.__version__.split(".")[:2]

# if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
# function to create our object tracker
if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(args["tracker"].upper())

# otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
# approrpiate object tracker constructor:
else:
	# initialize a dictionary that maps strings to their corresponding
	# OpenCV object tracker implementations
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
	}

	# grab the appropriate object tracker using our dictionary of
	# OpenCV object tracker objects
	tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()




bridge = CvBridge()

cv_image = None
media = []
laser_frente = []
centro = []
atraso = 0.5E9 # 1 segundo e meio. Em nanossegundos
area = 0.0 # Variavel com a area do maior contorno
viu_objeto = False
centro_mnet = []
count_frame = 0

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 
resultados=[]

dados = 0

dados_bumper = None

fps = None
initBB = None

x1 = 0
x2 = 0
y1 = 0
y2 = 0

distancia_min = 0.2
perto_frente = False
perto_esq = False
perto_costas = False
perto_dir = False

#Selecao de categoria via rede neural
print("Lista de objetos: background, aeroplane, bicycle, bird, boat, bottle, bus, car, cat, chair, cow, diningtable, dog, horse, motorbike, person, pottedplant, sheep, sofa, train, tvmonitor")
objeto = raw_input("Escolha o objeto que o robô irá evitar: ")


#Função para funcionamento do bumper
def bumperzou(dado):
    global dados_bumper
    #print("Numero: ", dado.data)
    dados_bumper = dado.data


#Função para funcionamento do le_scan
def scaneou(dado):
	global distancia_min, perto_frente, perto_esq, perto_costas, perto_dir, laser_frente
	dados_scan = dado.ranges
	
	laser_frente_naolimpa = list(dados_scan[:46] + scan_ranges[315:])
	for e in laser_frente:
		if e != 0:
			laser_frente.append(e)

	if len(laser_frente) > 0:
        maisperto_frente = min(laser_frente)
        if maisperto_frente <= distancia_min:
            perto_frente = True

        else:
            perto_frente = False

    else:
		perto_frente = False

    laser_esq_naolimpa = list(dados_scan[45:136])
	for e in laser_esq_naolimpa:
		if e != 0:
			laser_esq.append(e)

	if len(laser_esq) > 0:
        maisperto_esq = min(laser_esq)
        if maisperto_esq <= distancia_min:
            perto_esq = True

        else:
            perto_esq = False

    else:
		perto_esq = False

    laser_costas_naolimpa = list(scan_ranges[136:226])
    for e in laser_costas_naolimpa:
		if e != 0:
			laser_costas.append(e)
	
	if len(laser_costas) > 0:
        maisperto_costas = min(laser_costas)

        if maisperto_costas <= distancia_min:
            perto_costas = True

        else:
            perto_costas = False

    else:
		perto_costas = False

    laser_dir_naolimpa = list(scan_ranges[226:316])
	for e in laser_dir_naolimpa:
		if e != 0:
			laser_dir.append(e)

	if len(laser_dir) > 0:
        maisperto_dir = min(laser_dir)
        if maisperto_dir <= distancia_min:
            perto_dir = True

        else:
            perto_dir = False

    else:
		perto_dir = False



#A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global viu_objeto
	global centro_mnet
	global resultados
	global count_frame
	global initBB
	global x1, x2, y1, y2
	global tracker


	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	#print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, area =  cormodule.identifica_cor(cv_image)
		centro_mnet, imagem, resultados =  visao_module.processa(cv_image)

		for r in resultados:
			if r[0] == objeto:
				viu_objeto = True
			else:
				viu_objeto = False

		

		#######
		#Código para funcionamento do tracking
		if count_frame < 5:
			if len(resultados) != 0:
				if viu_objeto:
					count_frame += 1
				else:
					count_frame = 0
		else:
			if len(resultados) !=0:
				x1 =  resultados[0][2][0]
	        	y1 = resultados[0][2][1]
	        	x2 = resultados[0][3][0]
	        	y2 = resultados[0][3][1]
	        	difx = x2 - x1
	        	dify = y2 - y1

        	initBB = (x1, y1, abs(difx), abs(dify))

        	tracker.init(cv_image, initBB)

        	fps = FPS().start()


        	if initBB is not None:
            # grab the new bounding box coordinates of the object
	            (success, box) = tracker.update(cv_image)

	            # check to see if the tracking was a success
	            if success:
	                (x, y, w, h) = [int(v) for v in box]
	                cv2.rectangle(cv_image, (x, y), (x + w, y + h),
	                    (0, 255, 0), 2)
	            # update the FPS counter
	            fps.update()
	            fps.stop()
	    #######


		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("main")

	topico_imagem = "/kamera"
	
	# Para renomear a *webcam*
	# 
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, bumperzou)


	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))


			#Caso identifique o objeto selecionado a ser evitado
			if viu_objeto:
				if len(resultados) !=0:
					
					vel_back = Twist(Vector3(-0.3,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel_back)
					rospy.sleep(0.1)
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					rospy.sleep(0.1)



			#Caso identifique a cor azul (água)
			elif len(media) != 0 and len(centro) != 0:
				print("Média dos azuis: {0}, {1}".format(media[0], media[1]))
				print("Centro dos azuis: {0}, {1}".format(centro[0], centro[1]))
				#print("testeeeeeeeeeeeeeeeeee", centro, media)
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))

				dif = media[0] - centro[1]

				if  dif > 60:
					vel_turn_right = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.5))
					velocidade_saida.publish(vel_turn_right)
					rospy.sleep(0.1)
				if dif < -60:
					vel_turn_left = Twist(Vector3(0.1,0,0), Vector3(0,0,0.5))
					velocidade_saida.publish(vel_turn_left)
					rospy.sleep(0.1)
				else:
					#Controle proporcional
					v = 0
					for d in laser_frente:
						v = 1/2*d
					vel_front = Twist(Vector3(v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel_front)
					rospy.sleep(0.1)




			vel_forw_survive = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
			vel_back_survive = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))
			vel_turn_right_survive = Twist(Vector3(0,0,0), Vector3(0,0,1))
			vel_turn_left_survive = Twist(Vector3(0,0,0), Vector3(0,0,-1))
			vel_escape_right = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.5))
			vel_escape_left = Twist(Vector3(0.1,0,0), Vector3(0,0,0.5))
			vel_stop = Twist(Vector3(0,0,0), Vector3(0,0,0))

			#Adicionando laser scan
			if perto_frente == True:
				velocidade_saida.publish(vel_back_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_stop)
				rospy.sleep(0.1)
				
				perto_frente = False

			if perto_esq == True:
				velocidade_saida.publish(vel_escape_right)
				rospy.sleep(2)
				velocidade_saida.publish(vel_stop)
				rospy.sleep(0.1)

				perto_esq = False

			if perto_costas == True:
				velocidade_saida.publish(vel_forw_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_stop)
				rospy.sleep(0.1)
				
				perto_costas = False

			if perto_dir == True:
				velocidade_saida.publish(vel_escape_left)
				rospy.sleep(2)
				velocidade_saida.publish(vel_stop)
				rospy.sleep(0.1)

				perto_dir = False

			#Adicionando bumper
			if dados_bumper == 1:
				velocidade_saida.publish(vel_back_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_turn_left_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_stop)
				rospy.sleep(2)
				
				dados_bumper = 0
			if dados_bumper == 2:
				velocidade_saida.publish(vel_back_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_turn_right_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_stop)
				rospy.sleep(2)
				
				dados_bumper = 0
			if dados_bumper == 3:
				velocidade_saida.publish(vel_forw_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_turn_left_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_stop)
				rospy.sleep(2)
				
				dados_bumper = 0
			if dados_bumper == 4:
				velocidade_saida.publish(vel_forw_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_turn_right_survive)
				rospy.sleep(2)
				velocidade_saida.publish(vel_stop)
				rospy.sleep(2)
				
				dados_bumper = 0
	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


