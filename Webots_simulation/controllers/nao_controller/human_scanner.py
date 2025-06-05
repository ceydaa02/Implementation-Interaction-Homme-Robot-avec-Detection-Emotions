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


def find_closest_human(robot, TIME_STEP, MOTION_PATH):
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
    model = YOLO("yolov8n.pt")

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

    if detected_human is None:
        print("[INFO] Aucun humain détecté.")
        return None
    
    set_robot_default_pose(robot, head_yaw, l_shoulder_pitch, r_shoulder_pitch, TIME_STEP)


    turn_angle_deg = math.degrees(detected_human["turn_angle_rad"])
    print(f"[ACTION] Rotation du torse de {turn_angle_deg:.1f}°...")
    if abs(turn_angle_deg) >= 30: 
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
        print(f"[ACTION] Rotation du torse de {turn_angle_deg:.1f}°...")

    desired_distance = 2.5 
    initial_distance = detected_human["distance"]
    approach_distance = max(0.0, initial_distance - desired_distance)
    print(f"[ACTION] Avancer d'un total de {approach_distance:.2f} m vers la cible...")

    step_distance = 0.5  #0.765
    steps_needed = max(1, round(approach_distance / step_distance))

    for step in range(steps_needed):
        print(f"[ÉTAPE] Avancement de l’étape {step+1}/{steps_needed}...")

        walk_motion = Motion(MOTION_PATH + "Forwards50.motion")

        walk_motion.play()
        for _ in range(int(walk_motion.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                break
    print(f"[INFO] Arrêt à {desired_distance} m de la cible.")
    return detected_human




def go_to_face_human(robot, detected_human, TIME_STEP, MOTION_PATH):
    distance = detected_human["distance"]
    angle = detected_human["relative_angle_rad"]

    side_distance = distance * math.sin(angle)
    forward_distance = distance * math.cos(angle)

    side_steps = round(abs(side_distance) / 0.03)
    forward_steps = round(forward_distance / 0.5)

    side_motion_name = "SideStepLeft" if angle > 0 else "SideStepRight"

    print(f"[PLAN] Avancer de {forward_steps} pas et se déplacer de {side_steps} pas vers la {'gauche' if angle > 0 else 'droite'}.")


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





def find_gap(robot, TIME_STEP, MOTION_PATH):

    lidar = robot.getDevice("lidar")
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()

    lidar_values = lidar.getRangeImage()
    lidar_fov = lidar.getFov()
    resolution = len(lidar_values)

    gap_start = None
    gap_end = None
    min_jump = 3.0  # duvardan boşluğa sıçrama

    for i in range(10, resolution - 10):
        left = lidar_values[i - 5]
        center = lidar_values[i]
        right = lidar_values[i + 5]

        # Boşluk başlangıcı (duvar → açık alan)
        if gap_start is None and left < 6 and center > (left + min_jump):
            gap_start = i
            continue

        # Boşluk bitişi (açık alan → duvar)
        if gap_start is not None and center < 6 and right < 6:
            gap_end = i
            break

    if gap_start is None or gap_end is None:
        print("[AVERTISSEMENT] Impossible de détecter le début ou la fin du passage.")
        return

    # Geçit ortası
    gap_index = (gap_start + gap_end) // 2

    # Açı hesapla
    angle_rad = ((gap_index / resolution) - 0.5) * lidar_fov
    angle_deg = math.degrees(angle_rad)
    print(f"[PASSAGE] Centre du passage détecté ! Angle : {angle_deg:.2f}°")

    # Dönüş
    if abs(angle_deg) > 20:
        turn_motion_file = "TurnLeft40.motion" if angle_deg > 0 else "TurnRight40.motion"
        turn_motion = Motion(f"{MOTION_PATH}/{turn_motion_file}")
        turn_motion.play()
        for _ in range(int(turn_motion.getDuration() / TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return
        print(f"[ACTION] Tourné vers l’angle de {angle_deg:.1f}°.")

    else:
        print("[INFO] Déjà aligné vers le passage.")



def go_away_face_human(robot, TIME_STEP, MOTION_PATH):
    turn_motion = Motion(MOTION_PATH + "TurnLeft60.motion")
    turn_motion.play()
    for _ in range(int(turn_motion.getDuration() // TIME_STEP)):
        if robot.step(TIME_STEP) == -1:
            return
        
    for _ in range(10):
        forward = Motion(MOTION_PATH + "Forwards50.motion")
        forward.play()
        for _ in range(int(forward.getDuration() // TIME_STEP)):
            if robot.step(TIME_STEP) == -1:
                return