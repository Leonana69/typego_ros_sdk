import cv2
import numpy as np

# left/right camera dims
DIM = (1280, 720)


cap = cv2.VideoCapture(2)
fourcc = cv2.VideoWriter_fourcc(*'MJPG')   # Try MJPG first
try:
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
except:
    print("Note: Could not set FOURCC directly, using default")

# --- Choose resolution ---
# Try common resolutions for stereo cameras
width = DIM[0] * 2
height = DIM[1]

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
if actual_width == width and actual_height == height:
    print(f"Successfully set resolution to {width}x{height}")
else:
    print(f"Could not set {width}x{height}, got {actual_width}x{actual_height}")
    exit()

# --- Optional: set FPS ---
cap.set(cv2.CAP_PROP_FPS, 30)

# --- StereoSGBM parameters ---
min_disp = 0
num_disp = 32  # must be divisible by 16
block_size = 5
stereo = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=block_size,
    P1=8 * 3 * block_size ** 2,
    P2=32 * 3 * block_size ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
)

while True:
    ret, frame = cap.read()
    if not ret:
       print("Failed to grab frame")
       break

    # Split side-by-side frame into left and right images
    left = frame[:, :DIM[0]]
    right = frame[:, DIM[0]:]

    # Convert to grayscale for stereo matching
    gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    # Compute disparity map (values are fixed-point with 4 fractional bits)
    disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

    # Normalize disparity to 0-255 for display
    disp_vis = cv2.normalize(disparity, None, alpha=0, beta=255,
                             norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

    cv2.imshow("Left", left)
    cv2.imshow("Right", right)
    cv2.imshow("Disparity", disp_color)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()