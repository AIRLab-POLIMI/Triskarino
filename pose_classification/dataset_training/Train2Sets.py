import xgboost as xgb
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.metrics import (
    confusion_matrix,
    ConfusionMatrixDisplay,
    accuracy_score,
    classification_report,
)
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

# Keypoint features (all _x and _y columns)
keypoint_features = [
    #'nose_x',
    "nose_y",
    #'left_eye_x',
    #"left_eye_y",
    #'right_eye_x',
    #"right_eye_y",
    #'left_ear_x',
    #"left_ear_y",
    #'right_ear_x',
    #"right_ear_y",
    #'left_shoulder_x',
    "left_shoulder_y",
    #'right_shoulder_x',
    "right_shoulder_y",
    #'left_elbow_x',
    "left_elbow_y",
    #'right_elbow_x',
    "right_elbow_y",
    #'left_wrist_x',
    "left_wrist_y",
    #'right_wrist_x',
    "right_wrist_y",
    #'left_hip_x',
    "left_hip_y",
    #'right_hip_x',
    "right_hip_y",
    #'left_knee_x',
    "left_knee_y",
    #'right_knee_x',
    "right_knee_y",
    #'left_ankle_x',
    "left_ankle_y",
    #'right_ankle_x',
    "right_ankle_y",
]

# Angle features
angle_features = [
    "Left_Upper_Arm_angle",
    "Left_Lower_Arm_angle",
    "Right_Upper_Arm_angle",
    "Right_Lower_Arm_angle",
    "Left_Thigh_angle",
    "Left_Calf_angle",
    "Right_Thigh_angle",
    "Right_Calf_angle",
    #"Shoulders_angle",
    #"Left_Side_angle",
    #"Right_Side_angle",
    #"Hips_angle",
    #"Left_Head_angle",
    #"Right_Head_angle",
    #"Left_Eye_angle",
    #"Right_Eye_angle",
    #"Eyes_angle",
    #"Left_Nose_angle",
    #"Right_Nose_angle",
    "Left_Elbow_angle",
    "Right_Elbow_angle",
    "Left_Knee_angle",
    "Right_Knee_angle",
    "Left_Hip_angle",
    "Right_Hip_angle",
    "Right_Shoulder_angle",
    "Left_Shoulder_angle",
]

custom_features = [
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

euc_features = [
    "euc_nose_feet",
    "euc_l_shoulder_knee",
    "euc_r_shoulder_knee",
    "euc_l_ankle_hip",
    "euc_r_ankle_hip",
]

# Load dataset
file_path = "/input/your/path/to/balanced_dataset.csv"
web_path = "/input/your/path/to/webimages_balanced.csv"


def print_class_distribution(df, set_name, total_class_counts):
    class_counts = df["class"].value_counts(normalize=True) * 100
    print(f"\nClass Distribution in {set_name} Set:")
    for class_name, percentage in class_counts.items():
        total_count_for_class = total_class_counts[class_name]
        class_count_in_set = df[df["class"] == class_name].shape[0]
        class_percentage_in_set = (class_count_in_set / total_count_for_class) * 100
        print(
            f"{class_name}: {percentage:.2f}% of the set ({class_percentage_in_set:.2f}% of total for this class) "
        )


def split_dataset_combined(primary_file_path, web_file_path, train_ratio=0.7, val_ratio=0.15, test_ratio=0.15):

    # Load both datasets
    df_primary = pd.read_csv(primary_file_path)
    df_web = pd.read_csv(web_file_path)

    # Extract class name from video_id
    df_primary["class"] = df_primary["video_id"].str.extract(r"(^[A-Za-z]+)")
    df_web["class"] = df_web["video_id"].str.extract(r"(^[A-Za-z]+)")

    # Map classes and drop rows from other classes
    class_map = {"Standing": 0, "Sitting": 1, "SittingInChair": 2, "Crawling": 3}
    df_primary["class"] = df_primary["class"].map(class_map)
    df_web["class"] = df_web["class"].map(class_map)

    df_primary = df_primary.dropna(subset=["class"])
    df_web = df_web.dropna(subset=["class"])

    # Keep only common columns (intersection)
    common_columns = list(set(df_primary.columns).intersection(set(df_web.columns)))
    df_primary = df_primary[common_columns]
    df_web = df_web[common_columns]

    # Combine datasets
    df_combined = pd.concat([df_primary, df_web], ignore_index=True)

    # Split based only on df_primary (to ensure stratification and no leakage from web set)
    video_groups = df_primary.groupby("video_id").first().reset_index()
    train_videos, temp_videos = train_test_split(
        video_groups,
        test_size=(1 - train_ratio),
        stratify=video_groups["class"],
        random_state=42,
    )
    val_videos, test_videos = train_test_split(
        temp_videos,
        test_size=(test_ratio / (test_ratio + val_ratio)),
        stratify=temp_videos["class"],
        random_state=42,
    )

    # Filter df_primary based on split
    train_set_primary = df_primary[
        df_primary["video_id"].isin(train_videos["video_id"])
    ]
    val_set = df_primary[df_primary["video_id"].isin(val_videos["video_id"])]
    test_set = df_primary[df_primary["video_id"].isin(test_videos["video_id"])]

    # Force all web samples into the training set
    train_set = pd.concat([train_set_primary, df_web], ignore_index=True)

    # Print split summary
    total_class_counts = df_combined["class"].value_counts()
    print(f"Final Split Percentages:")
    print(f"Training: {len(train_set) / len(df_combined):.2%}")
    print(f"Validation: {len(val_set) / len(df_combined):.2%}")
    print(f"Testing: {len(test_set) / len(df_combined):.2%}")
    print_class_distribution(train_set, "Training", total_class_counts)
    print_class_distribution(val_set, "Validation", total_class_counts)
    print_class_distribution(test_set, "Testing", total_class_counts)

    return train_set, val_set, test_set, df_combined


def plot_confusion_matrix_with_info(
    cm,
    class_names,
    train_set,
    val_set,
    test_set,
    total_class_counts,
    df,
    y_true,
    y_pred,
    model,
    model_filename,
):
    fig, ax = plt.subplots(figsize=(12, 8))

    disp = ConfusionMatrixDisplay(confusion_matrix=cm, display_labels=class_names)
    disp.plot(cmap=plt.cm.Blues, values_format="d", ax=ax)
    disp.im_.colorbar.remove()

    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="3%", pad=0.05)
    fig.colorbar(disp.im_, cax=cax)
    ax.set_title("Confusion Matrix")
    plt.subplots_adjust(left=-0.1, bottom=0.2)

    train_pct = len(train_set) / len(df) * 100
    val_pct = len(val_set) / len(df) * 100
    test_pct = len(test_set) / len(df) * 100
    accuracy = accuracy_score(y_true, y_pred)
    report = classification_report(y_true, y_pred, target_names=class_names)

    def format_class_distribution(df, set_name):
        class_counts = df["class"].value_counts(normalize=True) * 100
        info = f"{set_name} Set:\n"
        for class_index, percentage in class_counts.items():
            class_name = class_names[int(class_index)]
            total_count = total_class_counts[class_index]
            class_count = df[df["class"] == class_index].shape[0]
            class_percentage = (class_count / total_count) * 100
            info += f"{class_name}: {percentage:.2f}% of the set ({class_percentage:.2f}% of total)\n"
        return info

    info_text = f"Final Split Percentages:\nTraining: {train_pct:.2f}%\nValidation: {val_pct:.2f}%\nTesting: {test_pct:.2f}%\n\n"
    info_text += format_class_distribution(train_set, "Training") + "\n"
    info_text += format_class_distribution(val_set, "Validation") + "\n"
    info_text += format_class_distribution(test_set, "Testing") + "\n"
    info_text += f"\nAccuracy: {accuracy:.4f}\n\nClassification Report:\n{report}"

    plt.gcf().text(
        1.3,
        0.5,
        info_text,
        fontsize=10,
        verticalalignment="center",
        horizontalalignment="left",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
        transform=ax.transAxes,
    )

    param_summary = f"Model: {model_filename}\n"
    param_summary += f"Training Parameters:\n"
    param_summary += f"max_depth: {model.get_params()['max_depth']}, "
    param_summary += f"min_child_weight: {model.get_params()['min_child_weight']}, "
    param_summary += f"gamma: {model.get_params()['gamma']},\n"
    param_summary += f"subsample: {model.get_params()['subsample']}, "
    param_summary += f"colsample_bytree: {model.get_params()['colsample_bytree']},\n"
    param_summary += f"reg_lambda: {model.get_params()['reg_lambda']}, "
    param_summary += f"reg_alpha: {model.get_params()['reg_alpha']}, "
    param_summary += f"early_stopping_rounds: {model.get_params().get('early_stopping_rounds', 'N/A')}, "
    param_summary += f"n_estimators: {model.get_params()['n_estimators']}"

    plt.gcf().text(
        0.5,
        0.01,
        param_summary,
        fontsize=10,
        verticalalignment="bottom",
        horizontalalignment="center",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
        transform=fig.transFigure,
    )

    plt.show()

train_set, val_set, test_set, df = split_dataset_combined(file_path, web_path)

# Save video_id splits to CSVs (optional)
# Get unique video IDs per set

train_videos = train_set["video_id"].unique()
val_videos = val_set["video_id"].unique()
test_videos = test_set["video_id"].unique()
train_set_ids = set(train_videos)
val_set_ids = set(val_videos)
test_set_ids = set(test_videos)

assert train_set_ids.isdisjoint(val_set_ids), "Train/Validation sets share video_ids!"
assert train_set_ids.isdisjoint(test_set_ids), "Train/Test sets share video_ids!"
assert val_set_ids.isdisjoint(test_set_ids), "Validation/Test sets share video_ids!"
print("\n No overlap in video_ids between train, validation, and test sets.")

# Use angle and keypoint features for training
# angle_features = [col for col in train_set.columns if 'angle' in col.lower()]
# keypoint_features = [col for col in train_set.columns if any(suffix in col for suffix in ['knee_x', 'knee_y'])]
# all_features = angle_features + keypoint_features
selected_features = keypoint_features + angle_features + custom_features + euc_features

print(f"Using features: {selected_features}")  # Optional print to verify

X_train, y_train = train_set[selected_features], train_set["class"]
X_val, y_val = val_set[selected_features], val_set["class"]
X_test, y_test = test_set[selected_features], test_set["class"]

# Initialize the XGBoost model
model = xgb.XGBClassifier(
    objective="multi:softprob",
    num_class=4,
    missing=0,
    eval_metric="mlogloss",
    max_depth=15,
    min_child_weight=4,
    gamma=2.872006909021972,
    learning_rate=0.06848179260041194,
    subsample=0.8539723625743169,
    colsample_bytree=0.922258108562239,
    reg_lambda=1.9805162159731216,
    reg_alpha=0.7879072613990459,
    # early_stopping_rounds=20
)

# Train the model with validation set
model.fit(X_train, y_train, eval_set=[(X_val, y_val)], verbose=False)

# Make predictions on the test set
y_pred = model.predict(X_test)

# Define class names
class_names = ["Standing", "Sitting", "SittingInChair", "Crawling"]

# Calculate metrics
accuracy = accuracy_score(y_test, y_pred)
report = classification_report(y_test, y_pred, target_names=class_names)

# Print results
print(f"Accuracy: {accuracy}")
print("Classification Report:")
print(report)

model_filename = "Pose_Classification_Final.xgb"
# Save the trained model
model.save_model("/home/thierry/Desktop/Models/" + model_filename)

# Compute confusion matrix
cm = confusion_matrix(y_test, y_pred)

# Call the function to plot confusion matrix
plot_confusion_matrix_with_info(
    cm,
    class_names,
    train_set,
    val_set,
    test_set,
    df["class"].value_counts(),
    df,
    y_test,
    y_pred,
    model,
    model_filename,
)

# Plot tree
xgb.plot_tree(model, num_trees=0)
plt.show()

# Plot feature importance
xgb.plot_importance(model)
plt.show()
