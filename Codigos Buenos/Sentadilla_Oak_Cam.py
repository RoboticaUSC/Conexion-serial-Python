import cv2
import depthai as dai
import mediapipe as mp
import numpy as np
from math import acos, degrees

# Inicializar la cámara OAK-D por nodos
pipeline = dai.Pipeline()

cam = pipeline.createColorCamera()
cam.setPreviewSize(1280, 720)
cam.setInterleaved(False)
cam.setFps(30)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)

cam_xout = pipeline.createXLinkOut()
cam_xout.setStreamName("cam_out")
cam.preview.link(cam_xout.input)

## inicializar estimador de pose
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

arriba = False
abajo = False
sentadillas = 0

with mp_pose.Pose(min_detection_confidence = 0.8, min_tracking_confidence = 0.8) as pose:
    with dai.Device(pipeline) as device:
        cam_queue = device.getOutputQueue(name="cam_out", maxSize=1, blocking=False)
        # Obtener el output de la cámara OAK-D y visualizarlo en la ventana
        while True:
            # Obtener el siguiente frame de la cámara OAK-D
            in_frame = cam_queue.get()
            frame = in_frame.getCvFrame()
            frame = cv2.flip(frame, 1)# Rotar la imagen horizontalmente

            al, an, _ = frame.shape

            results = pose.process(frame)

            if results.pose_landmarks is not None:
                mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, mp_drawing.DrawingSpec(color = (128, 0 , 250), thickness = 2, circle_radius = 2), mp_drawing.DrawingSpec(color = (255, 255, 255), thickness = 2))
                #Extraccion de los puntos importantes en la piernas
                #Pierna Derecha
                x1 = int(results.pose_landmarks.landmark[24].x * an)
                y1 = int(results.pose_landmarks.landmark[24].y * al)

                x2 = int(results.pose_landmarks.landmark[26].x * an)
                y2 = int(results.pose_landmarks.landmark[26].y * al)

                x3 = int(results.pose_landmarks.landmark[28].x * an)
                y3 = int(results.pose_landmarks.landmark[28].y * al)
                #Pierna Izquierda
                x4 = int(results.pose_landmarks.landmark[23].x * an)
                y4 = int(results.pose_landmarks.landmark[23].y * al)

                x5 = int(results.pose_landmarks.landmark[25].x * an)
                y5 = int(results.pose_landmarks.landmark[25].y * al)

                x6 = int(results.pose_landmarks.landmark[27].x * an)
                y6 = int(results.pose_landmarks.landmark[27].y * al)

                p1R = np.array([x1, y1])
                p2R = np.array([x2, y2])
                p3R = np.array([x3, y3])

                p1L = np.array([x4, y4])
                p2L = np.array([x5, y5])
                p3L = np.array([x6, y6])

                l1R = np.linalg.norm(p2R - p3R)
                l2R = np.linalg.norm(p1R - p3R)
                l3R = np.linalg.norm(p1R - p2R)

                l1L = np.linalg.norm(p2L - p3L)
                l2L = np.linalg.norm(p1L - p3L)
                l3L = np.linalg.norm(p1L - p2L)

                #Calculo para contar sentadillas
                try:
                    angulo_R = degrees(acos((l1R**2 + l3R**2 - l2R**2)/(2*l1R*l3R)))
                    angulo_L = degrees(acos((l1L**2 + l3L**2 - l2L**2)/(2*l1L*l3L)))
                except ValueError:
                    angulo_R = 170
                    angulo_L = 170

                if angulo_R >= 160:
                    arriba = True
                if arriba == True and abajo == False and angulo_R <= 120 and angulo_L <= 110:
                    abajo = True
                if arriba == True and abajo == True and angulo_R >= 160 and angulo_L >= 160:
                    sentadillas += 1
                    arriba = False
                    abajo = False

                cv2.putText(frame, "Angulo pierna Derecha: " + str(angulo_R), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                cv2.putText(frame, "Angulo pierna Izquierda: " + str(angulo_L), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, "Numero de sentadillas " + str(sentadillas), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 128, 255), 2)

                #Graficamos puntos de interes pierna derecha
                cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 0), 20)
                cv2.line(frame, (x2, y2), (x3, y3), (255, 255, 0), 20)
                cv2.line(frame, (x1, y1), (x3, y3), (255, 255, 0), 5)
                
                #Graficamos puntos de interes pierna izquierda
                cv2.line(frame, (x4, y4), (x5, y5), (51, 114, 40), 20)
                cv2.line(frame, (x5, y5), (x6, y6), (51, 114, 40), 20)
                cv2.line(frame, (x4, y4), (x6, y6), (51, 114, 40), 5)

            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) == 27:
                break

cv2.destroyAllWindows()