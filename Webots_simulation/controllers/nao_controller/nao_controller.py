from controller import Robot, Motion
import numpy as np
from human_scanner import find_closest_human, show_sad_reaction, find_gap, show_fear_reaction, show_happy_reaction
from start_emotion_gui import prewarm_emotion_engine, analyze_emotion

prewarm_emotion_engine()


TIME_STEP = 32
MOTION_PATH = "../../motions/"

robot = Robot()

control = 3
p=0

while(control):
    if not p:
        detected_human = find_closest_human(robot, TIME_STEP, MOTION_PATH)
    else:
        p=0

    if detected_human is not None:
        dominant_emotion = analyze_emotion(duration=5)
            
        if dominant_emotion == "sad" or dominant_emotion == "neutral":
            show_sad_reaction(robot, detected_human, TIME_STEP, MOTION_PATH)
            motion = Motion(MOTION_PATH + "/TurnLeft60.motion")
            motion.play()

            for _ in range(int(motion.getDuration() / TIME_STEP)):
                if robot.step(TIME_STEP) == -1:
                    break

        elif dominant_emotion == "fear" or dominant_emotion == "angry" or dominant_emotion == "disgust" or dominant_emotion == "surprise":
            show_fear_reaction(robot, TIME_STEP, MOTION_PATH)

        elif dominant_emotion == "happy":
            show_happy_reaction(robot, TIME_STEP, MOTION_PATH)

        control-=1


    else:
        find_gap(robot, TIME_STEP, MOTION_PATH)
        while (detected_human := find_closest_human(robot, TIME_STEP, MOTION_PATH)) is None:
            for i in range(3):
                motion = Motion(MOTION_PATH + "/Forwards50.motion")
                motion.play()
        
                for _ in range(int(motion.getDuration() / TIME_STEP)+5):
                    if robot.step(TIME_STEP) == -1:
                        break
        p=1