import os
import cv2
import pandas as pd
import numpy as np


def is_valid_keypoint(x, y):
    return not (np.isnan(x) or np.isnan(y) or x == 0.0 or y == 0.0)


def create_image_with_keypoints(row, frame_width, frame_height):
    frame = np.ones((frame_height, frame_width, 3), dtype=np.uint8) * 255

    body_keypoints = [
        "nose", "left_eye", "right_eye", "left_ear", "right_ear",
        "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
        "left_wrist", "right_wrist", "left_hip", "right_hip",
        "left_knee", "right_knee", "left_ankle", "right_ankle"
    ]

    keypoints = {}
    for i in range(len(body_keypoints)):
        x, y = row[i * 2], row[i * 2 + 1]
        if is_valid_keypoint(x, y):
            keypoints[i] = (int(x * frame_width), int(y * frame_height))

    for x, y in keypoints.values():
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    connections = {
        "face": [(3, 5), (4, 6), (3, 1), (4, 2), (1, 2), (1, 0), (2, 0)],
        "body": [(5, 6), (11, 12), (5, 11), (6, 12)],
        "arms": [(5, 7), (7, 9), (6, 8), (8, 10)],
        "legs": [(11, 13), (13, 15), (12, 14), (14, 16)],
    }

    colors = {
        "face": (0, 255, 0),
        "body": (255, 0, 255),
        "arms": (255, 0, 0),
        "legs": (0, 165, 255),
    }

    for part, links in connections.items():
        color = colors[part]
        for p1, p2 in links:
            if p1 in keypoints and p2 in keypoints:
                x1, y1 = keypoints[p1]
                x2, y2 = keypoints[p2]
                cv2.line(frame, (x1, y1), (x2, y2), color, 2)

    return frame


def process_balanced_dataset(dataset_path, output_directory):
    
    import shutil

    if os.path.exists(output_directory):
        shutil.rmtree(output_directory)
    os.makedirs(output_directory)

    df = pd.read_csv(dataset_path)

    frame_width, frame_height = 640, 360

    # Create directories dynamically while iterating
    for idx, row in df.iterrows():
        video_id = str(row["video_id"])
        frame_number = str(row.get("frame_number", "unknown"))
        class_name = video_id.split("-")[0]

        class_folder = os.path.join(output_directory, class_name)
        os.makedirs(class_folder, exist_ok=True)

        image = create_image_with_keypoints(row, frame_width, frame_height)

        image_filename = os.path.join(class_folder, f"{video_id}_{frame_number}_{idx}.jpg")
        cv2.imwrite(image_filename, image)

        if idx % 100 == 0:
            print(f"Processed {idx}/{len(df)} images...")


    print("All images processed and saved.")


# Define paths
balanced_dataset_path = "/input/your/path/to/balanced_dataset.csv"
output_directory = "/input/your/saving/path/"

# Run the processor
process_balanced_dataset(balanced_dataset_path, output_directory)
