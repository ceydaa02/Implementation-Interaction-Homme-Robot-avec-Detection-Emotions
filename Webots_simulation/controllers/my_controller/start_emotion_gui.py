from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
import cv2
from deepface import DeepFace
from collections import Counter
import sys
import time

class EmotionWorker(QThread):
    emotion_result = pyqtSignal(str)
    current_frame = pyqtSignal(object)

    def __init__(self, duration=2):
        super().__init__()
        self.duration = duration
        self.result = "None"

    def run(self):
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print("[ERREUR] Impossible d’ouvrir la caméra.")
            self.result = "CameraError"
            return
 

        emotions = []
        start_time = time.time()
        end_time = start_time + self.duration

        while time.time() < end_time:
            ret, frame = cap.read()
            if not ret:
                continue

            try:
                result = DeepFace.analyze(frame, actions=["emotion"], enforce_detection=False, silent=True)
                emotion = result[0]["dominant_emotion"]
                emotions.append(emotion)

                region = result[0].get("region")
                if region:
                    x, y, w, h = region["x"], region["y"], region["w"], region["h"]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            except:
                pass

            self.current_frame.emit(frame)

        cap.release()
        if emotions:
            self.result = Counter(emotions).most_common(1)[0][0]
        else:
            self.result = "None"
        print(f"[INFO] Émotion dominante détectée : {self.result}")
        self.emotion_result.emit(self.result)

class EmotionWindow(QWidget):
    def __init__(self, duration=2):
        super().__init__()
        self.setWindowTitle("Analyse des émotions (Live)")
        self.setGeometry(200, 200, 640, 480)
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        self.setLayout(layout)

    def update_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qt_image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def stop(self):
        if hasattr(self, "timer") and self.timer.isActive():
            self.timer.stop()
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()


def analyze_emotion(duration=2):
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    window = EmotionWindow(duration=duration)
    result_holder = {}

    def on_done(result):
        result_holder["emotion"] = result
        window.close()
        app.quit()

    thread = EmotionWorker(duration=duration)
    thread.current_frame.connect(window.update_frame)
    thread.emotion_result.connect(on_done)
    thread.start()

    window.show()
    app.exec_()

    return result_holder.get("emotion", "None")

def prewarm_emotion_engine():
    print("[INFO] Initialisation anticipée de la caméra et du modèle DeepFace...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("[ERREUR] Impossible d’ouvrir la caméra (préparation)")
        return
    for _ in range(5):
        cap.read()
        time.sleep(0.05)
    ret, frame = cap.read()
    if ret:
        try:
            DeepFace.analyze(frame, actions=["emotion"], enforce_detection=False, silent=True)
            print("[INFO] Modèle DeepFace chargé avec succès.")
        except Exception as e:
            print(f"[AVERTISSEMENT] Erreur lors de l’analyse initiale : {e}")
    cap.release()
