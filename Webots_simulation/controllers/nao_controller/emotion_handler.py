# fer.py
import cv2
from deepface import DeepFace
from collections import Counter
import time
from controller import Motion


def analyze_emotion(duration=2, camera_index=0):
    detector_backend = "opencv"
    align = False
    expand = 0

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise Exception("Kamera açılamadı. Farklı bir indeks deneyin (0, 1, 2...)")

    emotions = []
    start_time = time.time()
    print("🎥 Duygu analizi başlatıldı...")

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
        print("❌ Duygu tespit edilemedi.")
        return None, []

    most_common = Counter(emotions).most_common(1)[0][0]
    print(f"🎯 En çok tekrar eden duygu: {most_common}")

    return most_common



def execute_motions(robot, motion_names, TIME_STEP, NAO_PATH):
    for name in motion_names:
        print(f"[MOTION] {name}")
        motion = Motion(NAO_PATH + f"{name}.motion")
        motion.play()
        for _ in range(int(motion.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                break
        for _ in range(4):  # dengeleme süresi
            if robot.step(TIME_STEP) == -1:
                break