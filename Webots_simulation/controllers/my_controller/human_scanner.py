from controller import Motion
import numpy as np
from ultralytics import YOLO
import math


def set_robot_default_pose(robot, head_yaw, l_shoulder_pitch, r_shoulder_pitch, TIME_STEP):
    
    head_yaw.setPosition(0.0)
    for _ in range(20):
        if robot.step(TIME_STEP) == -1:
            break

    l_shoulder_pitch.setPosition(1.5)
    r_shoulder_pitch.setPosition(1.5)
    for _ in range(20):
        if robot.step(TIME_STEP) == -1:
            break


def find_closest_human(robot, TIME_STEP, MOTION_PATH, steps_forward, step=6):
    camera = robot.getDevice("CameraTop")
    camera.enable(4 * TIME_STEP)
    camera_fov_rad = camera.getFov()
    camera_fov_deg = math.degrees(camera_fov_rad)

    lidar = robot.getDevice("lidar")
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()
    lidar_resolution = lidar.getHorizontalResolution()
    lidar_fov = lidar.getFov()

    head_yaw = robot.getDevice("HeadYaw")
    head_yaw.setPosition(float('inf'))
    head_yaw.setVelocity(1.0)

    l_shoulder_pitch = robot.getDevice("LShoulderPitch")
    r_shoulder_pitch = robot.getDevice("RShoulderPitch")

    walk_motion = Motion(MOTION_PATH + "Forwards50.motion")
    model = YOLO("yolov8s.pt")

    print("[INFO] Recherche de l’humain par balayage de la tête...")

    detected_human = None
    found = False
    head_scan_angles_deg = list(range(-60, 65, 10))

    for angle_deg in head_scan_angles_deg:
        if found:
            break

        angle_rad = math.radians(angle_deg)
        head_yaw.setPosition(angle_rad)
        for _ in range(30):
            if robot.step(TIME_STEP) == -1:
                break

        image = camera.getImage()
        if not image:
            continue

        width = camera.getWidth()
        height = camera.getHeight()
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))[:, :, :3]

        results = model.predict(img, verbose=False)
        lidar_values = lidar.getRangeImage()

        for result in results:
            if found:
                break
            for box in result.boxes:
                if found:
                    break
                if int(box.cls[0]) == 0:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    w = x2 - x1
                    h = y2 - y1
                    if w * h < 300:
                        continue

                    center_x = (x1 + x2) / 2
                    normalized_x = center_x / width

                    relative_angle = (0.5 - normalized_x) * camera_fov_rad
                    lidar_angle_rad = angle_rad + (normalized_x - 0.5) * camera_fov_rad
                    lidar_index = int(((lidar_angle_rad + lidar_fov / 2) / lidar_fov) * lidar_resolution)

                    if 0 <= lidar_index < len(lidar_values):
                        distance = lidar_values[lidar_index]
                        if distance == float('inf') or distance > 5:
                            continue

                        print("Humain détecté sur la " + ("DROITE" if relative_angle < 0 else "GAUCHE"))
                        print(f"[DÉTECTÉ] Tête: {math.degrees(angle_rad):.1f}° | Décalage visuel: {math.degrees(relative_angle):.1f}° | Distance: {distance:.2f} m")

                        detected_human = {
                            "distance": distance,
                            "turn_angle_rad": relative_angle + angle_rad,
                            "relative_angle_rad": relative_angle
                        }
                        found = True
                        break
    set_robot_default_pose(robot, head_yaw, l_shoulder_pitch, r_shoulder_pitch, TIME_STEP)
    if detected_human is None:
        print("[INFO] Aucun humain détecté.")
        return None, 0
    
    tmp_steps_forward = 0
    for i in range(step-steps_forward):
        tmp_steps_forward += 1
        print("Pas effectués : ", steps_forward+tmp_steps_forward)
        forward = Motion(MOTION_PATH + "Forwards50.motion")
        forward.play()
        for _ in range(int(forward.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return


    turn_steps = 0
    turn_angle_deg = math.degrees(detected_human["turn_angle_rad"])
    if abs(turn_angle_deg) >= 15: 
        print(f"[ACTION] Rotation du torse de {turn_angle_deg:.1f}°...")
        turn_steps = round(abs(turn_angle_deg) / 40)
        for _ in range(turn_steps):
            turn_motion = Motion(
                MOTION_PATH + ("TurnRight40.motion" if turn_angle_deg < 0 else "TurnLeft40.motion")
            )
            turn_motion.play()
            for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
                if robot.step(TIME_STEP) == -1:
                    break
            for _ in range(5):
                if robot.step(TIME_STEP) == -1:
                    break
    else:
        print(f"[ACTION] Déjà aligné à {turn_angle_deg:.1f}°, pas de rotation nécessaire.")

    

    return detected_human, tmp_steps_forward


"""
    for _ in range(side_steps):
        side_motion = Motion(MOTION_PATH + f"{side_motion_name}.motion")
        side_motion.play()
        for _ in range(int(side_motion.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return

    for _ in range(forward_steps):
        forward = Motion(MOTION_PATH + "Forwards50.motion")
        forward.play()
        for _ in range(int(forward.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return
"""


"""
if(control!=1):
        for _ in range(side_steps):
            side_motion = Motion(MOTION_PATH + f"{return_motion_name}.motion")
            side_motion.play()
            for _ in range(int(side_motion.getDuration() // TIME_STEP)):
                if robot.step(TIME_STEP) == -1:
                    return
                

        turn_steps = 0
        turn_angle_deg = math.degrees(detected_human["turn_angle_rad"])
        if abs(turn_angle_deg) >= 20: 
            turn_steps = round(abs(turn_angle_deg) / 40)
            for _ in range(turn_steps):
                turn_motion = Motion(
                    MOTION_PATH + ("TurnLeft40.motion" if turn_angle_deg < 0 else "TurnRight40.motion")
                )
                turn_motion.play()
                for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
                    if robot.step(TIME_STEP) == -1:
                        break
                for _ in range(5):
                    if robot.step(TIME_STEP) == -1:
                        break

            last_steps_needed = math.ceil(forward_steps * math.sin(math.radians(40)))*5
            for _ in range(last_steps_needed):
                side_motion = Motion(MOTION_PATH + f"{return_motion_name}.motion")
                side_motion.play()
                for _ in range(int(side_motion.getDuration() // TIME_STEP)):
                    if robot.step(TIME_STEP) == -1:
                        return
                    
            step_taken = math.ceil(forward_steps * math.sin(math.radians(50)))
            steps_forward += step_taken
            print(f"[INFO] Pas effectués : {steps_forward}")

            for _ in range(EACH_STEP - steps_forward):
                
                forward = Motion(MOTION_PATH + "Forwards50.motion")
                forward.play()
                for _ in range(int(forward.getDuration() // TIME_STEP)):
                    if robot.step(TIME_STEP) == -1:
                        return
                    

"""



def show_sad_reaction(robot, detected_human, TIME_STEP, MOTION_PATH, EACH_STEP, control, steps_forward):
    distance = detected_human["distance"]
    angle = detected_human["relative_angle_rad"]
    turn_angle_deg = math.degrees(detected_human["turn_angle_rad"])

    if abs(turn_angle_deg) >= 15:
        side_motion_name = "TurnLeft40" if turn_angle_deg > 0 else "TurnRight40"
        return_motion_name = "TurnRight40" if turn_angle_deg > 0 else "TurnLeft40"

        forward_distance = distance * math.cos(angle)
        forward_steps = round(forward_distance / 1)

        print(f"[PLAN] Avancer de {forward_steps} pas vers la {'gauche' if angle > 0 else 'droite'}.")

        turn_motion = Motion(MOTION_PATH + f"{side_motion_name}.motion")
        turn_motion.play()
        for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                break
        for _ in range(5):
            if robot.step(TIME_STEP) == -1:
                break

        for _ in range(forward_steps):
            forward = Motion(MOTION_PATH + "Forwards50.motion")
            forward.play()
            for _ in range(int(forward.getDuration() // TIME_STEP)):
                if robot.step(TIME_STEP) == -1:
                    return
                
        turn_steps = round(abs(turn_angle_deg) / 40)
        if turn_steps:        
            turn_motion = Motion(MOTION_PATH + f"{return_motion_name}.motion")
            turn_motion.play()
            for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
                if robot.step(TIME_STEP) == -1:
                    break
            for _ in range(5):
                if robot.step(TIME_STEP) == -1:
                    break


    forward = Motion(MOTION_PATH + "OpenArms.motion")
    forward.play()
    for _ in range(int(forward.getDuration() // TIME_STEP) + 60):
        if robot.step(TIME_STEP) == -1:
            return
        

    forward = Motion(MOTION_PATH + "CloseArms.motion")
    forward.play()
    for _ in range(int(forward.getDuration() // TIME_STEP) + 5):
        if robot.step(TIME_STEP) == -1:
            return


    if(control!=1):
        if abs(turn_angle_deg) >= 15:
            turn_steps = round(abs(turn_angle_deg) / 40)
            for i in range(3*turn_steps):
                turn_motion = Motion(MOTION_PATH + f"{return_motion_name}.motion")
                turn_motion.play()
                for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
                    if robot.step(TIME_STEP) == -1:
                        break
                for _ in range(5):
                    if robot.step(TIME_STEP) == -1:
                        break

            for _ in range(forward_steps):
                forward = Motion(MOTION_PATH + "Forwards50.motion")
                forward.play()
                for _ in range(int(forward.getDuration() // TIME_STEP)):
                    if robot.step(TIME_STEP) == -1:
                        return
                    
            # Turn back to the original position
            for i in range(2*turn_steps):
                turn_motion = Motion(MOTION_PATH + f"{side_motion_name}.motion")
                turn_motion.play()
                for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
                    if robot.step(TIME_STEP) == -1:
                        break
                for _ in range(5):
                    if robot.step(TIME_STEP) == -1:
                        break

                    
            step_taken = math.ceil(forward_steps * math.sin(math.radians(10)))*2
            steps_forward += step_taken
            print(f"[INFO] Pas effectués : {steps_forward}")

        for _ in range(EACH_STEP - steps_forward):
            
            forward = Motion(MOTION_PATH + "Forwards50.motion")
            forward.play()
            for _ in range(int(forward.getDuration() // TIME_STEP)):
                if robot.step(TIME_STEP) == -1:
                    return
                


def show_happy_reaction(robot, detected_human, TIME_STEP, MOTION_PATH, EACH_STEP, control, steps_forward):

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

    if(control!=1):
        turn_steps = 0
        turn_angle_deg = math.degrees(detected_human["turn_angle_rad"])
        if abs(turn_angle_deg) >= 15: 
            turn_steps = round(abs(turn_angle_deg) / 40)
            for _ in range(turn_steps):
                turn_motion = Motion(
                    MOTION_PATH + ("TurnLeft40.motion" if turn_angle_deg < 0 else "TurnRight40.motion")
                )
                turn_motion.play()
                for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
                    if robot.step(TIME_STEP) == -1:
                        break
                for _ in range(5):
                    if robot.step(TIME_STEP) == -1:
                        break

            for _ in range(EACH_STEP - steps_forward - 1):
                
                forward = Motion(MOTION_PATH + "Forwards50.motion")
                forward.play()
                for _ in range(int(forward.getDuration() // TIME_STEP)):
                    if robot.step(TIME_STEP) == -1:
                        return


def show_fear_reaction(robot, detected_human, TIME_STEP, MOTION_PATH, EACH_STEP, control, steps_forward):

    turn_angle_deg = math.degrees(detected_human["turn_angle_rad"])
    turn_steps = round(abs(turn_angle_deg) / 40)
    if turn_steps:
        turn_motion = Motion(MOTION_PATH + "TurnLeft40.motion")
        turn_motion.play()
        for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return
        

    turn_motion = Motion(MOTION_PATH + "TurnLeft40.motion")
    turn_motion.play()
    for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
        if robot.step(TIME_STEP) == -1:
            return
        
    for _ in range(3):
        forward = Motion(MOTION_PATH + "Forwards50.motion")
        forward.play()
        for _ in range(int(forward.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return
            
    turn_motion = Motion(MOTION_PATH + "TurnRight40.motion")
    turn_motion.play()
    for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
        if robot.step(TIME_STEP) == -1:
            return
        

    for _ in range(4):
        steps_forward += 1
        forward = Motion(MOTION_PATH + "Forwards50.motion")
        forward.play()
        for _ in range(int(forward.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return
        


    if(control!=1):
        turn_motion = Motion(MOTION_PATH + "TurnRight40.motion")
        turn_motion.play()
        for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return
            

        for _ in range(3):
            forward = Motion(MOTION_PATH + "Forwards50.motion")
            forward.play()
            for _ in range(int(forward.getDuration() // TIME_STEP)):
                if robot.step(TIME_STEP) == -1:
                    return
                
        turn_motion = Motion(MOTION_PATH + "TurnLeft40.motion")
        turn_motion.play()
        for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return       

        step_taken = math.ceil(3 * math.sin(math.radians(40)))*2
        steps_forward += step_taken
        print(f"[INFO] Pas effectués : {steps_forward}")


        for _ in range(EACH_STEP - steps_forward - 1): 
            forward = Motion(MOTION_PATH + "Forwards50.motion")
            forward.play()
            for _ in range(int(forward.getDuration() // TIME_STEP)):
                if robot.step(TIME_STEP) == -1:
                    return
