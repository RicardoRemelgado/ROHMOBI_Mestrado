import cv2
import numpy as np

capture = cv2.VideoCapture(1)
tabela = cv2.imread("hsv.png")
cv2.imshow("tabela", tabela)

def mapa(x):
   temp=tabela.copy()
   cv2.rectangle(temp, (int(MinH*4.2666), 0), (int(MaxH*4.2666), 100), (127, 127, 127), 2)
   cv2.imshow("tabela", temp)

cv2.createTrackbar('MinH','tabela',0,255, mapa)
cv2.createTrackbar('MaxH','tabela',0,255, mapa)
cv2.createTrackbar('MinS','tabela',0,255, mapa)
cv2.createTrackbar('MaxS','tabela',0,255, mapa)
cv2.createTrackbar('MinV','tabela',0,255, mapa)
cv2.createTrackbar('MaxV','tabela',0,255, mapa)



ret, frame = capture.read()
while (ret and cv2.waitKey(1)!=27):
   cv2.imshow('Frame', frame)

   hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   cv2.imshow("hsv", hsv)

   MinH = cv2.getTrackbarPos('MinH', 'tabela')
   MaxH = cv2.getTrackbarPos('MaxH', 'tabela')

   MinS = cv2.getTrackbarPos('MinS', 'tabela')
   MaxS = cv2.getTrackbarPos('MaxS', 'tabela')
   MinV = cv2.getTrackbarPos('MinV', 'tabela')
   MaxV = cv2.getTrackbarPos('MaxV', 'tabela')


   cor_min = np.array([MinH, MinS, MinV])
   cor_max = np.array([MaxH, MaxS, MaxV])



   filtered = cv2.inRange(hsv, cor_min, cor_max)
   cv2.imshow('Filtrada', filtered)
   kernel = np.ones((5, 5), np.uint8)
   erosao = cv2.erode(filtered, kernel)
   cv2.imshow("erosion", erosao)

   ret, frame = capture.read()
capture.release()
cv2.destroyAllWindows()




'''
# Modo de utilização para leitura de tempo
import time

t=time.perf_counter()
while(ret and cv2.waitKey(1)!=ord('q')):
  # processamento
  #
  duracao = time.perf_counter() - t
  t=time.perf_counter()
  print("diff=", duracao, "  fps=", 1/duracao)
'''
