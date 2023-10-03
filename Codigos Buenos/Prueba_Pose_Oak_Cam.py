import cv2
import depthai as dai
import mediapipe as mp
import numpy as np

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

with mp_pose.Pose(min_detection_confidence = 0.8, min_tracking_confidence = 0.8) as pose:
    with dai.Device(pipeline) as device:
        cam_queue = device.getOutputQueue(name="cam_out", maxSize=1, blocking=False)
        # Obtener el output de la cámara OAK-D y visualizarlo en la ventana
        while True:
            # Obtener el siguiente frame de la cámara OAK-D
            in_frame = cam_queue.get()
            frame = in_frame.getCvFrame()
            frame = cv2.flip(frame, 1)# Rotar la imagen horizontalmente

            #frame_RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = pose.process(frame)

            if results.pose_landmarks is not None:
                mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, mp_drawing.DrawingSpec(color = (128, 0 , 250), thickness = 2, circle_radius = 2), mp_drawing.DrawingSpec(color = (255, 255, 255), thickness = 2))

            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) == 27:
                break

cv2.destroyAllWindows()