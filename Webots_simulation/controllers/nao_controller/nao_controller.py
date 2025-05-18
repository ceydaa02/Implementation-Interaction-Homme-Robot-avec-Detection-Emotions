from controller import Robot, Motion
import numpy as np
from emotion_handler import analyze_emotion, execute_motions
from human_scanner import find_closest_human, go_to_face_human, find_gap, go_away_face_human


TIME_STEP = 32
MOTION_PATH = "../../motions/"

robot = Robot()

detected_human = find_closest_human(robot, TIME_STEP, MOTION_PATH)
dominant_emotion = analyze_emotion(duration=2, camera_index=0)
print(f"[INFO] Dominant Emotion: {dominant_emotion}")

dominant_emotion = "sad"
if dominant_emotion == "sad":
    go_to_face_human(robot, detected_human, TIME_STEP, MOTION_PATH)


motion = Motion(MOTION_PATH + "/TurnLeft60.motion")
motion.play()

# Hareketin süresi kadar bekle
for _ in range(int(motion.getDuration() / TIME_STEP)):
    if robot.step(TIME_STEP) == -1:
        break
        

detected_human = find_closest_human(robot, TIME_STEP, MOTION_PATH)
if detected_human is not None:
    dominant_emotion = analyze_emotion(duration=2, camera_index=0)
    print(f"[INFO] Dominant Emotion: {dominant_emotion}")
else:
    find_gap(robot, TIME_STEP, MOTION_PATH)
    while (detected_human := find_closest_human(robot, TIME_STEP, MOTION_PATH)) is None:
        for i in range(3):
            motion = Motion(MOTION_PATH + "/Forwards50.motion")
            motion.play()
    
            for _ in range(int(motion.getDuration() / TIME_STEP)+5):
                if robot.step(TIME_STEP) == -1:
                    break


dominant_emotion = analyze_emotion(duration=2, camera_index=0)
print(f"[INFO] Dominant Emotion: {dominant_emotion}")

dominant_emotion = "fear"
if dominant_emotion == "fear":
    go_away_face_human(robot, TIME_STEP, MOTION_PATH)


detected_human = find_closest_human(robot, TIME_STEP, MOTION_PATH)
if detected_human is not None:
    dominant_emotion = analyze_emotion(duration=2, camera_index=0)
    print(f"[INFO] Dominant Emotion: {dominant_emotion}")
    dominant_emotion = "happy"
    if dominant_emotion == "happy":
        motion = Motion(MOTION_PATH + "/Handwave.motion")
        motion.play()

        for _ in range(int(motion.getDuration() / TIME_STEP)+5):
            if robot.step(TIME_STEP) == -1:
                break

        motion = Motion(MOTION_PATH + "/WipeForeHead.motion")
        motion.play()

        for _ in range(int(motion.getDuration() / TIME_STEP)+5):
            if robot.step(TIME_STEP) == -1:
                break