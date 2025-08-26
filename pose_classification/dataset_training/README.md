# Dataset and Training Programs for XGBoost Model

This folder contains the datasets and programs used for training the XGBoost pose classification model.

---

## Datasets

### `combined_dataset.csv`
- Stores the **raw feature values** of the four classes:  
  - Standing  
  - Sitting  
  - SittingInChair  
  - Crawling  
- Contains unprocessed data (before any reconstruction, augmentation, or balancing).
- Each sample (row/entry) with the same `video_id` comes from the same clip/sequence and can be ordered using the `frame_number` column.
- `video_id` encodes:  
  - The **Class** (e.g., Standing)  
  - Metadata about its origin (e.g., `1-30`)  
  - The subject identifier (e.g., *Adult*, *Kid in Blue*, ...).  
- Features include:  
  - Keypoint coordinates (normalized between 0 and 1, with `0` values representing missing data)  
  - Angles  
  - Distances  
  - Some motion/bounding box-related features (tested but not used).  
- Samples with `-conv` flag (e.g., `video_id-sample-conv`) come from **Walking** and **Running** classes, which were converted into **Standing** samples to increase intra-class variety (since walking/running subjects are by default standing).

---

### `balanced_dataset.csv`
- Fully processed and ready-to-train dataset.  
- Processing steps include:  
  1. **Partial reconstruction**: Linear interpolation recovers small gaps of missing keypoints.  
  2. **Augmentation by vertical symmetry**: Each sample is mirrored, doubling dataset size and removing horizontal/positional bias.  
  3. **Balancing classes**:  
     - Overrepresented classes → **cluster sampling** removes highly similar samples until reaching the median threshold (promoting variety).  
     - Underrepresented classes → **oversampling** with slight perturbations (adding noise, shifting, rotating keypoints, recomputing features) until reaching the median threshold.  

---

### `webimage_balanced.csv`
- Another **processed, ready-to-train dataset**, built from stock images collected online.  
- Created to address **edge cases** that the original dataset lacked.  
  - Example: *All SittingInChair samples in the original set are front-facing and very similar.*  
  - Using web images, we can add variety (different viewing angles and poses) and combine with the original dataset during training.  

---

## Training Programs

### `Train.py`
- Trains an XGBoost model **on a single dataset** (e.g., `balanced_dataset.csv` or `webimage_balanced.csv`).

### `Train2Sets.py`
- Trains an XGBoost model **on both datasets combined** (original + web images).  
- Apart from handling multiple datasets, it works similarly to `Train.py`.  
- You can comment/uncomment specific features to include them in training.

⚠️ **Important:**  
If you comment/uncomment features during training, you **must also update the same features in `Inference.py`**.  
Otherwise, the trained model and the inference program will mismatch (wrong number/type of features), and the system will fail.

---

### `reconstructbalanced.py`
- Reconstructs the **keypoint images** of the samples in `balanced_dataset.csv`.  
- Useful for visual inspection: helps you see what the samples and classes actually look like instead of only working with abstract feature values in a spreadsheet.

---

## Setup

To run these programs, you need to install the dependencies listed at the top of the code.  
It is **strongly recommended** to install them inside a **conda/miniconda virtual environment**.

Also, make sure to edit the training scripts to:  
- Point to your dataset paths.  
- Define the path where you want to save the trained model.

---

For any information on these programs or datasets, you should be able to contact me at: thierry.jannin1@gmail.com  
Master Thesis Project realized at AIRLab (PoliMi) between January and July 2025
