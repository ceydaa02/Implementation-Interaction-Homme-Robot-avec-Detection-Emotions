# fer.py
import cv2
from deepface import DeepFace
from collections import Counter
import time
from controller import Motion
from datetime import datetime
import os

"""
def analyze_emotion(duration=2, camera_index=0):
    detector_backend = "opencv"
    align = False
    expand = 0

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise Exception("Impossible d’ouvrir la caméra. Essayez un autre index (0, 1, 2...)")

    emotions = []
    start_time = time.time()
    print("Caméra activée, démarrage de l’analyse des émotions...")

    while time.time() - start_time < duration:
        ret, frame = cap.read()
        if not ret:
            continue

        try:
            result = DeepFace.analyze(img_path=frame.copy(),
                                      actions=['emotion'],
                                      enforce_detection=False,
                                      detector_backend=detector_backend,
                                      align=align,
                                      expand_percentage=expand,
                                      silent=True)
            emotion = result[0]['dominant_emotion']
            emotions.append(emotion)
        except:
            continue

    cap.release()
    cv2.destroyAllWindows()

    if not emotions:
        print("Aucune émotion n’a pu être détectée.")
        return None, []

    most_common = Counter(emotions).most_common(1)[0][0]
    print(f"Émotion dominante détectée : {most_common}")

    return most_common
"""





def analyze_emotion(duration=2, camera_index=0):
    detector_backend = "opencv"
    align = False
    expand = 0

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise Exception("Impossible d’ouvrir la caméra. Essayez un autre index (0, 1, 2...)")

    emotions = []
    last_frame = None
    last_result = None

    start_time = time.time()
    print("Caméra activée, démarrage de l’analyse des émotions...")

    while time.time() - start_time < duration:
        ret, frame = cap.read()
        if not ret:
            continue

        try:
            result = DeepFace.analyze(img_path=frame.copy(),
                                      actions=['emotion'],
                                      enforce_detection=False,
                                      detector_backend=detector_backend,
                                      align=align,
                                      expand_percentage=expand,
                                      silent=True)
            emotion = result[0]['dominant_emotion']
            emotions.append(emotion)
            last_result = result[0]
            last_frame = frame.copy()
        except:
            continue

    cap.release()
    cv2.destroyAllWindows()

    if not emotions or last_frame is None:
        print("Aucune émotion n’a pu être détectée.")
        return None

    most_common = Counter(emotions).most_common(1)[0][0]
    print(f"Émotion dominante détectée : {most_common}")


    if last_result and 'region' in last_result:
        region = last_result['region']
        x, y, w, h = region["x"], region["y"], region["w"], region["h"]
        cv2.rectangle(last_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(last_frame, most_common, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (50, 255, 50), 2)
    cv2.putText(last_frame, f"Emotion: {most_common}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


    os.makedirs("output", exist_ok=True)
    filename = f"output/emotion_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    cv2.imwrite(filename, last_frame)

    return most_common






def execute_motions(robot, motion_names, TIME_STEP, NAO_PATH):
    for name in motion_names:
        print(f"[MOUVEMENT] Exécution du mouvement : {name}")
        motion = Motion(NAO_PATH + f"{name}.motion")
        motion.play()
        for _ in range(int(motion.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                break
        for _ in range(4):  
            if robot.step(TIME_STEP) == -1:
                break