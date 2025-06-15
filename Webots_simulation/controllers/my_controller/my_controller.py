from controller import Robot, Motion
import numpy as np
from human_scanner import find_closest_human, show_sad_reaction, show_fear_reaction, show_happy_reaction
from start_emotion_gui import prewarm_emotion_engine, analyze_emotion

prewarm_emotion_engine()


TIME_STEP = 32
MOTION_PATH = "../../motions/"
EACH_STEP = 16
steps_forward = 0

robot = Robot()

control = 3
p=0



while(control):

    if not p:
        detected_human, steps_taken = find_closest_human(robot, TIME_STEP, MOTION_PATH, steps_forward, step=3)
        steps_forward += steps_taken

    else:
        p=0

    if detected_human is not None:

        dominant_emotion = analyze_emotion(duration=5)


        if dominant_emotion == "sad" or dominant_emotion == "neutral":
            show_sad_reaction(robot, detected_human, TIME_STEP, MOTION_PATH, EACH_STEP, control, steps_forward)

        elif dominant_emotion == "fear" or dominant_emotion == "angry" or dominant_emotion == "disgust" or dominant_emotion == "surprise":
            show_fear_reaction(robot, detected_human, TIME_STEP, MOTION_PATH, EACH_STEP, control, steps_forward)


        elif dominant_emotion == "happy":
            show_happy_reaction(robot, detected_human, TIME_STEP, MOTION_PATH, EACH_STEP, control, steps_forward)

        control-=1
        steps_forward = 0

    else:
        print("Aucun humain détecté, robot en mouvement")
        detected_human = None
        steps_taken = 0
        while detected_human is None:
            detected_human, steps_taken = find_closest_human(robot, TIME_STEP, MOTION_PATH, steps_forward)
            steps_forward += steps_taken
            if detected_human is not None:
                break

            for i in range(3):
                steps_forward += 1
                print("Pas effectués : ", steps_forward)
                motion = Motion(MOTION_PATH + "/Forwards50.motion")
                motion.play()

                for _ in range(int(motion.getDuration() / TIME_STEP) + 5):
                    if robot.step(TIME_STEP) == -1:
                        break

        p=1