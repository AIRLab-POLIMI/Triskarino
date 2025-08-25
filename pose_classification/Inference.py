#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import xgboost as xgb
from ultralytics import YOLO
import os
import pandas as pd
import math
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

# Load models
model = YOLO("/input/your/path/to/yolo11x-pose.pt")
xgboost_model_path = "/input/your/path/to/Pose_Classification_Final.xgb"
model_predic = xgb.Booster()
model_predic.load_model(xgboost_model_path)

model_filename = os.path.basename(xgboost_model_path)
model_name_display = os.path.splitext(model_filename)[0]
tracker_config = "bytetrack.yaml"

# Initialize CvBridge
bridge = CvBridge()


class EMASmoother:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.prev_smoothed = None  # Will hold previous smoothed keypoints

    def smooth(self, current_keypoints):
        """
        current_keypoints: List of keypoints [(x1, y1), (x2, y2), ...]
        Returns: Smoothed keypoints in same format
        """
        if self.prev_smoothed is None:
            # For the first frame, no previous smoothing; return as is
            self.prev_smoothed = current_keypoints
            return current_keypoints

        smoothed_keypoints = []
        for (x, y), (sx, sy) in zip(current_keypoints, self.prev_smoothed):
            # Apply EMA only on x and y; keep confidence as is or average if you want
            new_x = self.alpha * x + (1 - self.alpha) * sx
            new_y = self.alpha * y + (1 - self.alpha) * sy
            # Confidence: keep current or average

            smoothed_keypoints.append((new_x, new_y))

        self.prev_smoothed = smoothed_keypoints
        return smoothed_keypoints

smoother = EMASmoother(alpha=0.3)

def apply_posture_rules(keypoints, model_prediction, bbox, frame_width, frame_height):
    """
    Applies custom rules to override model predictions based on context and visibility.
    """

    x1, y1, x2, y2 = bbox
    box_area = (x2 - x1) * (y2 - y1)
    frame_area = frame_width * frame_height
    box_ratio = box_area / frame_area

    # Track visible keypoint indices
    visible_indices = [
        idx for idx, kp in enumerate(keypoints) if kp and kp[0] > 0 and kp[1] > 0
    ]
    num_visible = len(visible_indices)

    # Rule: Person is too close
    if box_ratio > 0.2 and num_visible < 7:
        knee_ankle_indices = {13, 14, 15, 16}
        upper_body_indices = {
            5,
            6,
            7,
            8,
            9,
            10,
            11,
            12,
        }  # Shoulders, elbows, wrists, hips

        if set(visible_indices).issubset(knee_ankle_indices) and visible_indices:
            return 0, "Too close"
        elif any(idx in upper_body_indices for idx in visible_indices):
            return 4, "Too close"
        else:
            return 5, "Too close"

    elif num_visible < 4:
        return 5, "Occluded/OutOfFrame"

    return model_prediction, None

def get_direction(prev_bbox, curr_bbox):
    (x1_prev, y1_prev, x2_prev, y2_prev) = prev_bbox
    (x1_curr, y1_curr, x2_curr, y2_curr) = curr_bbox

    width_prev = x2_prev - x1_prev
    height_prev = y2_prev - y1_prev
    width_curr = x2_curr - x1_curr
    height_curr = y2_curr - y1_curr

    center_x_prev = (x1_prev + x2_prev) / 2
    center_x_curr = (x1_curr + x2_curr) / 2

    delta_x = center_x_curr - center_x_prev
    delta_width = width_curr - width_prev
    delta_height = height_curr - height_prev

    print(f"Δx = {delta_x:.2f}, Δwidth = {delta_width:.2f}, Δheight = {delta_height:.2f}")

    # Thresholds
    image_width = 640
    image_height = 360
    movement_threshold = image_width * 0.02     # ~13 px
    scale_threshold_w = width_prev * 0.05       # 5%
    scale_threshold_h = height_prev * 0.05      # 5%

    min_height_threshold = image_height * 0.3

    horizontal_dir = ""
    depth_dir = ""

    # Horizontal movement
    if abs(delta_x) > movement_threshold:
        horizontal_dir = "Right" if delta_x > 0 else "Left"

    # Depth movement (Toward/Away)
    if height_prev > min_height_threshold:
        if abs(delta_height) > scale_threshold_h or abs(delta_width) > scale_threshold_w:
            avg_delta = (delta_height + delta_width) / 2
            depth_dir = "Towards" if avg_delta > 0 else "Away"
    else:
        if abs(delta_width) > scale_threshold_w:
            depth_dir = "Towards" if delta_width > 0 else "Away"

    # Combine
    if horizontal_dir and depth_dir:
        return f"{horizontal_dir} + {depth_dir}"
    elif horizontal_dir:
        return horizontal_dir
    elif depth_dir:
        return depth_dir
    else:
        return "Idle"

def calculate_limb_angle(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle_horizontal = math.degrees(math.atan2(dy, dx))  # Angle w.r.t horizontal
    return angle_horizontal


def calculate_joint_angle(p1, p2, p3):
    vector1 = np.array([p1[0] - p2[0], p1[1] - p2[1]])
    vector2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])

    magnitude1 = np.linalg.norm(vector1)
    magnitude2 = np.linalg.norm(vector2)

    if magnitude1 == 0 or magnitude2 == 0:
        return 0  # Avoid division by zero

    dot_product = np.dot(vector1, vector2)
    cos_theta = dot_product / (magnitude1 * magnitude2)
    angle = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    return np.degrees(angle)


# Prepare distance features
def avg_point(p1, p2):
    if p1 is not None and p2 is not None:
        return [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]
    elif p1 is not None:
        return p1
    elif p2 is not None:
        return p2
    else:
        return None


def euclidean(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def safe_y(p1, p2):
    return p1[1] - p2[1] if p1 and p2 else np.nan


def safe_x(p1, p2):
    return p1[0] - p2[0] if p1 and p2 else np.nan

# Body keypoints
body_keypoints = {
    0: "nose",
    1: "left_eye",
    2: "right_eye",
    3: "left_ear",
    4: "right_ear",
    5: "left_shoulder",
    6: "right_shoulder",
    7: "left_elbow",
    8: "right_elbow",
    9: "left_wrist",
    10: "right_wrist",
    11: "left_hip",
    12: "right_hip",
    13: "left_knee",
    14: "right_knee",
    15: "left_ankle",
    16: "right_ankle",
}

# Limb pairs to compute angles
limbs = {
    "Left Upper Arm": (5, 7),
    "Left Lower Arm": (7, 9),
    "Right Upper Arm": (6, 8),
    "Right Lower Arm": (8, 10),
    "Left Thigh": (11, 13),
    "Left Calf": (13, 15),
    "Right Thigh": (12, 14),
    "Right Calf": (14, 16),
    #"Shoulders": (5, 6),
    #"Left Side": (5, 11),
    #"Right Side": (6, 12),
    #"Hips": (11, 12),
    #"Left Head": (3, 5),
    #"Right Head": (4, 6),
    #"Left Eye": (3, 1),
    #"Right Eye": (4, 2),
    #"Eyes": (1, 2),
    #"Left Nose": (1, 0),
    #"Right Nose": (2, 0),
}

joints = {
    "Left Elbow": (5, 7, 9),  # Left Shoulder → Left Elbow → Left Wrist
    "Right Elbow": (6, 8, 10),  # Right Shoulder → Right Elbow → Right Wrist
    "Left Knee": (11, 13, 15),  # Left Hip → Left Knee → Left Ankle
    "Right Knee": (12, 14, 16),  # Right Hip → Right Knee → Right Ankle
    "Left Hip": (5, 11, 13),  # Left Shoulder → Left Hip → Left Knee
    "Right Hip": (6, 12, 14),  # Right Shoulder → Right Hip → Right Knee
    "Right Shoulder": (12, 6, 8),  # Right Hip → Right Shoulder → Right Elbow
    "Left Shoulder": (11, 5, 7),  # Left Hip → Left Shoulder → Left Elbow
}

keypoint_features = [
    "nose_y",
    #"left_eye_y",
    #"right_eye_y",
    #"left_ear_y",
    #"right_ear_y",
    "left_shoulder_y",
    "right_shoulder_y",
    "left_elbow_y",
    "right_elbow_y",
    "left_wrist_y",
    "right_wrist_y",
    "left_hip_y",
    "right_hip_y",
    "left_knee_y",
    "right_knee_y",
    "left_ankle_y",
    "right_ankle_y",
]

custom_feature_keys = [
    #"y_nose_ankles", 
    #"y_nose_knees", 
    "y_hips_knees", 
    "y_hips_ankles", 
    "y_knees_ankles",
    #"y_shoulders_knees", 
    #"y_shoulders_wrists", 
    #"x_ankles_hips", 
    #"x_ankles_knees",
    "x_knees_hips", 
    #"x_shoulders_hips", 
    #"x_shoulders_wrists"
]

euc_feature_keys = [
    "euc_nose_feet", "euc_l_shoulder_knee", "euc_r_shoulder_knee",
    "euc_l_ankle_hip", "euc_r_ankle_hip",
]

normalized_feature_keys = [
    #"y_nose_ankles_div_shoulder_width",
    #"y_nose_ankles_div_hip_width",
    #"y_nose_ankles_div_torso_avg",
    #"y_nose_knees_div_shoulder_width",
    #"y_nose_knees_div_hip_width",
    #"y_nose_knees_div_torso_avg",
    "y_hips_knees_div_shoulder_width",
    "y_hips_knees_div_hip_width",
    "y_hips_knees_div_torso_avg",
    "y_hips_ankles_div_shoulder_width",
    "y_hips_ankles_div_hip_width",
    "y_hips_ankles_div_torso_avg",
    "y_knees_ankles_div_shoulder_width",
    "y_knees_ankles_div_hip_width",
    "y_knees_ankles_div_torso_avg",
    #"y_shoulders_knees_div_shoulder_width",
    #"y_shoulders_knees_div_hip_width",
    #"y_shoulders_knees_div_torso_avg",
    #"y_shoulders_wrists_div_shoulder_width",
    #"y_shoulders_wrists_div_hip_width",
    #"y_shoulders_wrists_div_torso_avg",
    #"x_ankles_hips_div_shoulder_width",
    #"x_ankles_hips_div_hip_width",
    #"x_ankles_hips_div_torso_avg",
    #"x_ankles_knees_div_shoulder_width",
    #"x_ankles_knees_div_hip_width",
    #"x_ankles_knees_div_torso_avg",
    "x_knees_hips_div_shoulder_width",
    "x_knees_hips_div_hip_width",
    "x_knees_hips_div_torso_avg",
    #"x_shoulders_hips_div_shoulder_width",
    #"x_shoulders_hips_div_hip_width",
    #"x_shoulders_hips_div_torso_avg",
    #"x_shoulders_wrists_div_shoulder_width",
    #"x_shoulders_wrists_div_hip_width",
    #"x_shoulders_wrists_div_torso_avg",
]

# Class mapping
class_mapping = {
    0: "Standing",
    1: "Sitting",
    2: "SittingInChair",
    3: "Crawling",
    4: "OnTheGround",
    5: "Unknown",
}

keypoint_index_map = {v: k for k, v in body_keypoints.items()}
# Extract model input feature names
angle_feature_names = [
    name for name in model_predic.feature_names if "angle" in name.lower()
]

# Image callback function
frame_count = 0
touch_detected = False
move_detected = False
previous_bboxes = {}

def touch_callback(msg):
    global touch_detected
    touch_detected = msg.data

def move_callback(msg):
    global move_detected
    move_detected = msg.data

def image_callback(msg):
    global frame_count
    frame_count += 1
    global previous_bboxes

    if frame_count % 10 != 0:
        return

    try:
        # Convert the compressed image to OpenCV format
        image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Run YOLO + keypoint inference
        results_yolo = model.track(image, persist=True, tracker=tracker_config)
        image = results_yolo[0].plot()
        resized_image = cv2.resize(image, (1280, 720))
        frame_angles = {}
        frame_joint_angles = {}
        custom_features = [np.nan] * len(custom_feature_keys)
        euc_features = [np.nan] * len(euc_feature_keys)
        normalized_features = [np.nan] * len(normalized_feature_keys)
        h, w, _ = image.shape

        if results_yolo[0].boxes.id is not None:
            ids = results_yolo[0].boxes.id.cpu().numpy()
            boxes = results_yolo[0].boxes.xyxy.cpu().numpy()
            keypoints_list = results_yolo[0].keypoints.xyn.tolist()

            for i, person_id in enumerate(ids):
                x1, y1, x2, y2 = map(int, boxes[i])

                keypoints = keypoints_list[i]
                smoothed_keypoints = smoother.smooth(keypoints)
                bbox = (x1, y1, x2, y2)
                frame_width = resized_image.shape[1]
                frame_height = resized_image.shape[0]

                original_h, original_w = image.shape[:2]
                resized_w, resized_h = 1280, 720
                scale_x = resized_w / original_w
                scale_y = resized_h / original_h

                scaled_x1 = int(x1 * scale_x)
                scaled_x2 = int(x2 * scale_x)
                scaled_y1 = int(y1 * scale_y)
                scaled_y2 = int(y2 * scale_y)

                # Extract keypoints
                kp = {idx: smoothed_keypoints[idx] for idx in body_keypoints if smoothed_keypoints[idx] is not None and len(smoothed_keypoints[idx]) >= 2}
                normalized_kp = {
                    idx: (keypoint[0] / w, keypoint[1] / h)
                    for idx, keypoint in kp.items()
                }

                # Only compute direction if robot is stationary and at least 70% keypoints are valid
                valid_keypoints = [k for k in keypoints if k and k[0] > 0 and k[1] > 0]
                if not move_detected and len(valid_keypoints) >= 0.30 * len(body_keypoints):

                    prev_bbox = previous_bboxes.get(person_id, bbox)
                    direction = get_direction(prev_bbox, bbox)
                    previous_bboxes[person_id] = bbox  # Update for next frame

                    # Draw direction text at top of bounding box
                    text = f"Dir: {direction}"
                    text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
                    text_width, text_height = text_size
                    label_x = scaled_x1 + (scaled_x2 - scaled_x1) // 2 - text_width // 2
                    label_y = scaled_y1 + text_height + 10  # Add some padding below the top
                    cv2.putText(resized_image, text, (label_x, label_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 200, 0), 3)

                # Calculate reference distances for normalizationq
                if 5 in kp and 6 in kp:
                    shoulder_width = euclidean(normalized_kp[5], normalized_kp[6])
                else:
                    shoulder_width = np.nan

                if 11 in kp and 12 in kp:
                    hip_width = euclidean(normalized_kp[11], normalized_kp[12])
                else:
                    hip_width = np.nan

                torso_left = euclidean(normalized_kp[5], normalized_kp[11]) if 5 in kp and 11 in kp else np.nan
                torso_right = euclidean(normalized_kp[6], normalized_kp[12]) if 6 in kp and 12 in kp else np.nan

                # Calculate average torso length if both exist, otherwise use whichever exists
                if not np.isnan(torso_left) and not np.isnan(torso_right):
                    torso_avg = (torso_left + torso_right) / 2
                elif not np.isnan(torso_left):
                    torso_avg = torso_left
                elif not np.isnan(torso_right):
                    torso_avg = torso_right
                else:
                    torso_avg = np.nan

                nose = normalized_kp.get(0)
                hips = avg_point(normalized_kp.get(11), normalized_kp.get(12))
                knees = avg_point(normalized_kp.get(13), normalized_kp.get(14))
                ankles = avg_point(normalized_kp.get(15), normalized_kp.get(16))
                shoulders = avg_point(normalized_kp.get(5), normalized_kp.get(6))
                wrists = avg_point(normalized_kp.get(9), normalized_kp.get(10))

                # --- Y distances
                #custom_features[custom_feature_keys.index("y_nose_ankles")] = safe_y(nose, ankles)
                #custom_features[custom_feature_keys.index("y_nose_knees")] = safe_y(nose, knees)
                custom_features[custom_feature_keys.index("y_hips_knees")] = safe_y(hips, knees)
                custom_features[custom_feature_keys.index("y_hips_ankles")] = safe_y(hips, ankles)
                custom_features[custom_feature_keys.index("y_knees_ankles")] = safe_y(knees, ankles)
                #custom_features[custom_feature_keys.index("y_shoulders_knees")] = safe_y(shoulders, knees)
                #custom_features[custom_feature_keys.index("y_shoulders_wrists")] = safe_y(shoulders, wrists)

                # --- X distances
                #custom_features[custom_feature_keys.index("x_ankles_hips")] = safe_x(ankles, hips)
                #custom_features[custom_feature_keys.index("x_ankles_knees")] = safe_x(ankles, knees)
                custom_features[custom_feature_keys.index("x_knees_hips")] = safe_x(knees, hips)
                #custom_features[custom_feature_keys.index("x_shoulders_hips")] = safe_x(shoulders, hips)
                #custom_features[custom_feature_keys.index("x_shoulders_wrists")] = safe_x(shoulders, wrists)

                # Nose to average feet
                euc_features[euc_feature_keys.index("euc_nose_feet")] = euclidean(normalized_kp[0], avg_point(normalized_kp[15], normalized_kp[16])) if 0 in kp and (15 in kp or 16 in kp) else 0

                # Left shoulder to left knee
                euc_features[euc_feature_keys.index("euc_l_shoulder_knee")] = euclidean(normalized_kp[5], normalized_kp[13]) if 5 in kp and 13 in kp else 0

                # Right shoulder to right knee
                euc_features[euc_feature_keys.index("euc_r_shoulder_knee")] = euclidean(normalized_kp[6], normalized_kp[14]) if 6 in kp and 14 in kp else 0

                # Left ankle to left hip
                euc_features[euc_feature_keys.index("euc_l_ankle_hip")] = euclidean(normalized_kp[15], normalized_kp[11]) if 15 in kp and 11 in kp else 0

                # Right ankle to right hip
                euc_features[euc_feature_keys.index("euc_r_ankle_hip")] = euclidean(normalized_kp[16], normalized_kp[12]) if 16 in kp and 12 in kp else 0

                # Normalize distances
                normalizers = {
                    "shoulder_width": shoulder_width,
                    "hip_width": hip_width,
                    "torso_avg": torso_avg,
                }

                for i, feat_name in enumerate(custom_feature_keys):
                    value = custom_features[i]
                    for j, norm_name in enumerate(normalizers):
                        norm_value = locals().get(norm_name, np.nan)
                        idx = i * len(normalizers) + j
                        normalized_features[idx] = value / norm_value if not np.isnan(value) and not np.isnan(norm_value) and norm_value != 0 else 0

                for limb_name, (p1_idx, p2_idx) in limbs.items():
                    feature_name = f"{limb_name.replace(' ', '_')}_angle"
                    if feature_name in angle_feature_names:
                        p1 = (smoothed_keypoints[p1_idx][0], smoothed_keypoints[p1_idx][1])
                        p2 = (smoothed_keypoints[p2_idx][0], smoothed_keypoints[p2_idx][1])
                        angle_horizontal = calculate_limb_angle(p1, p2)
                        frame_angles[feature_name] = angle_horizontal
                    else:
                        frame_angles[feature_name] = 0  # Default if invalid

                # Calculate joint angles (e.g., elbow, knee, ...)
                for joint_name, (p1_idx, p2_idx, p3_idx) in joints.items():
                    feature_name = f"{joint_name.replace(' ', '_')}_angle"
                    if feature_name in angle_feature_names:
                        p1 = (smoothed_keypoints[p1_idx][0], smoothed_keypoints[p1_idx][1])
                        p2 = (smoothed_keypoints[p2_idx][0], smoothed_keypoints[p2_idx][1])
                        p3 = (smoothed_keypoints[p3_idx][0], smoothed_keypoints[p3_idx][1])
                        joint_angle = calculate_joint_angle(p1, p2, p3)
                        frame_joint_angles[feature_name] = joint_angle
                    else:
                        frame_joint_angles[feature_name] = 0  # Default if invalid

                    # Append current frame's data for the target person

                frame_keypoint_features = {}
                for feature in keypoint_features:
                    if feature.endswith("_y"):
                        name = feature.replace("_y", "")
                        idx = keypoint_index_map.get(name)
                        if idx is not None and idx < len(smoothed_keypoints):
                            value = smoothed_keypoints[idx][1] if smoothed_keypoints[idx][1] > 0 else 0
                        else:
                            value = 0
                        frame_keypoint_features[feature] = value

                frame_custom_features = dict(zip(normalized_feature_keys, normalized_features))
                frame_euc_features = dict(zip(euc_feature_keys, euc_features))
                all_features = {
                    **frame_keypoint_features,
                    **frame_angles,
                    **frame_joint_angles,
                    **frame_custom_features,
                    **frame_euc_features,
                }

                all_columns = (
                    list(frame_keypoint_features.keys()) +
                    list(frame_angles.keys()) +
                    list(frame_joint_angles.keys()) +
                    list(frame_custom_features.keys()) +
                    list(frame_euc_features.keys())
                )
                df_angles = pd.DataFrame([all_features], columns=all_columns)

                dmatrix = xgb.DMatrix(df_angles, missing=0)

                cut = model_predic.predict(dmatrix)
                model_prediction = np.argmax(cut[0])

                final_prediction, note = apply_posture_rules(keypoints, model_prediction, bbox, frame_width, frame_height)
                rule_triggered = (final_prediction != model_prediction and note)

                # Decide whether the prediction was overridden
                if final_prediction != model_prediction and note:
                    highest_label = class_mapping.get(final_prediction, f"Class {final_prediction}")
                    label_text = f"{highest_label} ({note})"
                else:
                    highest_label = class_mapping.get(model_prediction, f"Class {model_prediction}")
                    label_text = highest_label

                if cut is not None and len(cut) > 0 and cut[0].ndim == 1:
                    y_offset = 50

                    # Show person ID
                    cv2.putText(resized_image, f"Person {int(person_id)}:", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    y_offset += 25

                    # Show all class scores

                    for idx, prob in enumerate(cut[0]):
                        class_label = class_mapping.get(idx, f"Class {idx}")
                        text = f"{class_label}: {prob:.2f}"
                        cv2.putText(resized_image, text, (10, y_offset),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                        y_offset += 22

                    # Draw best guess class label at the bottom of the bounding box
                    text_size, _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    text_width, text_height = text_size

                    label_x = scaled_x1 + (scaled_x2 - scaled_x1) // 2 - text_width // 2
                    label_y = scaled_y2 - 10

                    cv2.putText(resized_image, label_text, (label_x, label_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Show
        (text_w, _), _ = cv2.getTextSize(model_name_display, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        x_position = resized_image.shape[1] - text_w - 100
        cv2.putText(resized_image, model_name_display, (x_position, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        if touch_detected:
            cv2.putText(resized_image, "Touched", (resized_image.shape[1] - 150, resized_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        if move_detected:
            cv2.putText(resized_image, "Moving", (10, resized_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("ROS Live Feed", resized_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            rospy.signal_shutdown("Quit by user")

    except CvBridgeError as e:
        print(e)


# ROS node initialization
rospy.init_node("pose_detection_node", anonymous=True)

# Subscribe to the image topic
rospy.Subscriber("/rpi_camera/image_raw/compressed", CompressedImage, image_callback)

rospy.Subscriber("/touch_event", Bool, touch_callback)
rospy.Subscriber("/move_event", Bool, move_callback)

# Spin to keep the program running and processing incoming messages
rospy.spin()

# Close OpenCV window when ROS shuts down
cv2.destroyAllWindows()
