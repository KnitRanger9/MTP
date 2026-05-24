import torch
import torch.nn as nn
import torch.nn.functional as F
# from torch.utils.data import Dataset, DataLoader
import numpy as np
import time
import cv2
# import os
# from glob import glob
from skimage.metrics import structural_similarity as ssim
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.animation import FuncAnimation
import pyzed.sl as sl

# --- 1. SuperPoint Model Definition ---
class SuperPoint(nn.Module):
    """
    SuperPoint model implementation based on the paper.
    Includes the VGG-style backbone, detector head, and descriptor head.
    """
    def __init__(self):
        super(SuperPoint, self).__init__()
        self.relu = nn.ReLU(inplace=True)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        
        # VGG-style Backbone
        self.conv1a = nn.Conv2d(1, 64, kernel_size=3, stride=1, padding=1)
        self.conv1b = nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1)
        self.conv2a = nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1)
        self.conv2b = nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1)
        self.conv3a = nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1)
        self.conv3b = nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1)
        self.conv4a = nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1)
        self.conv4b = nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1)
        
        # Detector Head - Updated names to match pre-trained weights
        self.convPa = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1)
        self.convPb = nn.Conv2d(256, 65, kernel_size=1, stride=1, padding=0)
        
        # Descriptor Head - Updated names to match pre-trained weights
        self.convDa = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1)
        self.convDb = nn.Conv2d(256, 256, kernel_size=1, stride=1, padding=0)

    def forward(self, x):
        # Shared Encoder
        x = self.relu(self.conv1a(x))
        x = self.relu(self.conv1b(x))
        x = self.pool(x)
        x = self.relu(self.conv2a(x))
        x = self.relu(self.conv2b(x))
        x = self.pool(x)
        x = self.relu(self.conv3a(x))
        x = self.relu(self.conv3b(x))
        x = self.pool(x)
        x = self.relu(self.conv4a(x))
        x = self.relu(self.conv4b(x))
        
        # Detector Head
        det_out = self.relu(self.convPa(x))
        det_out = self.convPb(det_out)
        
        # Descriptor Head
        desc_out = self.relu(self.convDa(x))
        desc_out = self.convDb(desc_out)
        
        return det_out, desc_out

def nms_fast(in_corners, H, W, dist_thresh):
    """
    Optimized NMS implementation from the paper.
    It performs NMS on a single-channel confidence map.
    """
    grid = np.zeros((H, W)).astype(int)
    inds = np.zeros((H, W)).astype(int)
    
    # Sort by confidence and round to integer coords
    inds1 = np.argsort(-in_corners[2,:])
    corners = in_corners[:,inds1]
    rcorners = corners[:2,:].round().astype(int)
    
    # Check for out of bounds and existing points
    valid = (rcorners[0,:] >= 0) & (rcorners[0,:] < H) & \
            (rcorners[1,:] >= 0) & (rcorners[1,:] < W)
    rcorners = rcorners[:,valid]
    
    # Initialize the grid
    grid[rcorners[0,:], rcorners[1,:]] = 1
    inds[rcorners[0,:], rcorners[1,:]] = np.arange(rcorners.shape[1])

    # Pad the grid for simpler window comparisons
    pad = dist_thresh
    grid_pad = np.pad(grid, ((pad,pad),(pad,pad)), mode='constant')
    
    # Iterate through points and suppress neighbors
    count = 0
    for i in range(rcorners.shape[1]):
        pt = (rcorners[0,i], rcorners[1,i])
        if grid[pt] == 1:
            grid_pad[pt[0]+pad-dist_thresh:pt[0]+pad+dist_thresh+1, 
                     pt[1]+pad-dist_thresh:pt[1]+pad+dist_thresh+1] = 0
            grid_pad[pt[0]+pad, pt[1]+pad] = 1
            count += 1
            
    # Extract the surviving points
    keep_inds = np.where(grid_pad[pad:-pad,pad:-pad] == 1)
    keep_inds = inds[keep_inds]
    return corners[:, keep_inds]


def is_similar(prev_frame, curr_frame, threshold=0.7):
    score, _ = ssim(prev_frame, curr_frame, full=True)
    return score > threshold

def get_superpoint_keypoints(detector_output, H, W, conf_thresh=0.015, nms_dist=4, max_kps=1000):
    """
    Post-processes the detector output to get keypoints.
    This function implements the optimized softmax and NMS from the paper. 
    """
    # Print input shapes for debugging
    print(f"Input shapes - H: {H}, W: {W}")
    print(f"Detector output shape: {detector_output.shape}")
    
    # Optimized Softmax: Convert detector scores to probabilities 
    probs = F.softmax(detector_output, dim=1)
    print(f"After softmax shape: {probs.shape}")
    
    # Remove the "no interest point" dustbin channel 
    probs = probs[:, :-1, :, :]
    print(f"After removing dustbin shape: {probs.shape}")
    
    # Reshape to get a confidence map for each pixel 
    Hc, Wc = probs.shape[2], probs.shape[3]
    print(f"Feature map dimensions - Hc: {Hc}, Wc: {Wc}")
    
    heatmap = probs.permute(0, 2, 3, 1).reshape(-1, Hc, Wc, 8, 8)
    print(f"After first reshape shape: {heatmap.shape}")
    
    heatmap = heatmap.permute(0, 1, 3, 2, 4).reshape(-1, Hc*8, Wc*8)
    print(f"After second reshape shape: {heatmap.shape}")
    
    # Ensure the heatmap matches the input image size
    if heatmap.shape[1] != H or heatmap.shape[2] != W:
        print(f"Resizing heatmap from {heatmap.shape} to ({H}, {W})")
        heatmap = F.interpolate(heatmap.unsqueeze(1), size=(H, W), mode='bilinear', align_corners=False).squeeze(1)
        print(f"After interpolation shape: {heatmap.shape}")
    
    # NMS
    pts = torch.where(heatmap[0] > conf_thresh)
    if len(pts[0]) == 0:
        print("No points found above confidence threshold")
        return np.empty((3,0)), np.empty((0,256))
        
    corners = torch.stack(pts).T.float() # (y, x) -> (row, col)
    print(f"Found {len(corners)} points above threshold")
    
    # Add confidence values
    confidences = heatmap[0, corners[:,0].long(), corners[:,1].long()]
    corners = torch.cat((corners.T.flip(0), confidences.unsqueeze(0)), dim=0)

    # NMS to get final points
    nms_corners = nms_fast(corners.cpu().numpy(), H, W, nms_dist)
    print(f"After NMS: {nms_corners.shape[1]} points")
    
    # Ranking Optimization: Use a heap or simple sorting for top-k 
    # Here we simply sort and take the top k
    if nms_corners.shape[1] > max_kps:
        sorted_indices = np.argsort(-nms_corners[2,:])
        nms_corners = nms_corners[:, sorted_indices[:max_kps]]
        print(f"After top-k selection: {nms_corners.shape[1]} points")
        
    keypoints = nms_corners[:2, :].T # Format as (x, y)
    
    return keypoints, nms_corners

def get_superpoint_descriptors(descriptors_output, keypoints, H, W):
    """
    Extracts descriptors for the given keypoints. 
    Implements deferred normalization as per the paper's optimization. 
    """
    # Interpolate descriptor map to original image size
    desc_map = F.interpolate(descriptors_output, size=(H, W), mode='bilinear', align_corners=False)
    
    # Normalize keypoints to be in [-1, 1] range for grid_sample
    # kps_tensor = torch.from_numpy(keypoints).float()
    kps_tensor = torch.from_numpy(keypoints).float()
    kps_norm = kps_tensor.clone()
    kps_norm[:, 0] = (kps_norm[:, 0] / (W - 1)) * 2 - 1
    kps_norm[:, 1] = (kps_norm[:, 1] / (H - 1)) * 2 - 1
    # kps_norm = kps_norm.unsqueeze(0).unsqueeze(1) # Shape for grid_sample
    kps_norm = kps_norm.unsqueeze(0).unsqueeze(0)
    # kps_norm = kps_norm.to(desc_map.device)
    kps_norm = kps_norm.to(desc_map.device)

    # Extract descriptors at keypoint locations
    descriptors = F.grid_sample(desc_map, kps_norm, align_corners=False)
    descriptors = descriptors.squeeze(0).squeeze(1).T
    
    # Normalization Optimization: L2-normalize only the k descriptors. 
    descriptors = F.normalize(descriptors, p=2, dim=1)
    
    # return descriptors
    return descriptors.cpu().numpy()

class ZedCAM:
    def __init__(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        # init_params.ca
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_fps = 15
        err = self.zed.open(init_params)
        runtime_parameters = sl.RuntimeParameters()
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Zed camera open failed:{err}")
        if self.zed.grab(runtime_parameters) != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Zed camera open failed: {self.zed.grab(runtime_parameters)}")
        self.runtime_parameters = sl.RuntimeParameters()
        self.mat_left = sl.Mat()
        self.mat_right = sl.Mat()
        self.depth_mat = sl.Mat()
        self.point_cloud = sl.Mat()

        self.image_width = self.zed.get_camera_information().camera_configuration.resolution.width
        self.image_height = self.zed.get_camera_information().camera_configuration.resolution.height
        self.camera_matrix = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam.fx
        self.K = self._get_intrinsics()

    def _get_intrinsics(self):
        calib = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
        return np.array([[calib.fx, 0, calib.cx],
                         [0, calib.fy, calib.cy],
                         [0,0,1]])
    
    def get_stereo_frames(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Get left image
            self.zed.retrieve_image(self.mat_left, sl.VIEW.LEFT, sl.MEM.CPU)
            left_image = self.mat_left.get_data()
            left_gray = cv2.cvtColor(left_image, cv2.COLOR_RGBA2GRAY)
            
            # Get right image
            self.zed.retrieve_image(self.mat_right, sl.VIEW.RIGHT, sl.MEM.CPU)
            right_image = self.mat_right.get_data()
            right_gray = cv2.cvtColor(right_image, cv2.COLOR_RGBA2GRAY)
            
            # Get depth map
            self.zed.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
            depth = self.depth_mat.get_data()
            
            return left_gray, right_gray, depth
        else:
            return None, None, None

    def get_frame(self):
        # Keep original method for backward compatibility
        left_gray, _, _ = self.get_stereo_frames()
        return left_gray
    
    # Function to triangulate 3D points from stereo keypoints
    def triangulate_3d_points(self, kp_left, kp_right, K, baseline = 0.12):
        """
        Triangulate 3D points from stereo keypoint matches
        """
        points_3d = []
        
        for i in range(len(kp_left)):
            # Get pixel coordinates
            xl, yl = kp_left[i]
            xr, yr = kp_right[i]
            
            # Calculate disparity
            disparity = xl - xr
            
            # Skip if disparity is too small (infinite depth)
            if disparity < 1.0:
                continue
                
            # Calculate 3D coordinates
            Z = (K[0, 0] * baseline) / disparity
            X = (xl - K[0, 2]) * Z / K[0, 0]
            Y = (yl - K[1, 2]) * Z / K[1, 1]
            
            points_3d.append([X, Y, Z])
        
        return np.array(points_3d)
    
    def close(self):
        self.zed.close()

# --- 3. Main Execution Block ---
if __name__ == '__main__':
    # --- Configuration ---
    # !!! USER: Update these paths !!!
    # sequence = '09'
    # KITTI_SEQUENCE_PATH = f'./kitti-data/sequences/{sequence}/image_2' # Path to your KITTI image sequence
    # KITTI_CALIB_FILE = f'./kitti-data/sequences/{sequence}/calib.txt'   # Path to your KITTI calibration file
    SUPERPOINT_WEIGHTS_PATH = './superpoint_v1.pth' # Path to pretrained weights
    # KITTI_POSES_FILE = os.path.join('./kitti-data/poses/', f'{sequence}.txt')

    # if os.path.exists(KITTI_POSES_FILE):
    #     gt_trajectory = load_kitti_poses(KITTI_POSES_FILE)
    #     print(f"Loaded {len(gt_trajectory)} ground truth poses")
    # else:
    #     print("Ground truth poses file not found. Will plot estimated trajectory only.")
    #     gt_trajectory = None
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    # device = 'cpu'
    print(f"Using device: {device}")

    # --- Load Model ---
    model = SuperPoint().to(device)
    model.load_state_dict(torch.load(SUPERPOINT_WEIGHTS_PATH, map_location=device))
    model.eval()
    print("SuperPoint model loaded.")
    
    stream = ZedCAM()
    H, W = stream.image_height, stream.image_width
    K = stream.K

    prev_left, prev_right, prev_depth = stream.get_stereo_frames()
    if prev_left is None:
        print("Failed to grab image")
        exit()
    prev_left_tensor = torch.from_numpy(prev_left).float().unsqueeze(0).unsqueeze(0)/255.0
    prev_right_tensor = torch.from_numpy(prev_right).float().unsqueeze(0).unsqueeze(0)/255.0

    # --- Visual Odometry Loop ---
    
    R_total = np.eye(3)
    t_total = np.zeros((3, 1))
    trajectory = []
    plt.ion()  # Turn on interactive mode
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    trajectory_plot, = ax.plot([], [], [], 'b-', linewidth=2, marker='o', markersize=3, label='Estimated Trajectory')
    keypoints_scatter = ax.scatter([], [], [], c='red', s=10, alpha=0.6, label='3D Keypoints')

    ax.set_title('Visual Odometry Trajectory with 3D Keypoints')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.grid(True)
    # ax.axis('equal')
    
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    
    # avg_time = t1 = time.time()
    prev_gray = None

    while True:
        print(f"Using device: {device}")
        # print(f"Processing frame pair {i+1}/{len(dataloader)}...")
        curr_left, curr_right, curr_depth = stream.get_stereo_frames()
        if curr_left is None:
            print("Failed to grab image")
            exit()
        if prev_left is not None and is_similar(prev_left, curr_left):
            print("Skipping similar frame.")
            continue
        curr_left_tensor = torch.from_numpy(curr_left).float().unsqueeze(0).unsqueeze(0)/255.0
        curr_right_tensor = torch.from_numpy(curr_right).float().unsqueeze(0).unsqueeze(0)/255.0
        
        # img1_tensor, img2_tensor = img1_tensor.to(device), img2_tensor.to(device)
        # prev_tensor, curr_tensor = prev_tensor.to(device), curr_tensor.to(device)
        
        with torch.no_grad():
            # --- Feature Extraction ---
            det_prev_left, desc_prev_left_out = model(prev_left_tensor.to(device))
            det_prev_right, desc_prev_right_out = model(prev_right_tensor.to(device))
            det_curr_left, desc_curr_left_out = model(curr_left_tensor.to(device))
            det_curr_right, desc_curr_right_out = model(curr_right_tensor.to(device))

            # --- Post-processing ---
            kp_prev_left, _ = get_superpoint_keypoints(det_prev_left, H, W)
            kp_prev_right, _ = get_superpoint_keypoints(det_prev_right, H, W)
            kp_curr_left, _ = get_superpoint_keypoints(det_curr_left, H, W)
            kp_curr_right, _ = get_superpoint_keypoints(det_curr_right, H, W)
            
            if (kp_prev_left.shape[0] < 8 or kp_curr_left.shape[0] < 8 or 
                kp_prev_right.shape[0] < 8 or kp_curr_right.shape[0] < 8):
                print("Not enough keypoints found. Skipping frame.")
                continue

            # Get descriptors
            desc_prev_left = get_superpoint_descriptors(desc_prev_left_out, kp_prev_left, H, W)
            desc_prev_right = get_superpoint_descriptors(desc_prev_right_out, kp_prev_right, H, W)
            desc_curr_left = get_superpoint_descriptors(desc_curr_left_out, kp_curr_left, H, W)
            desc_curr_right = get_superpoint_descriptors(desc_curr_right_out, kp_curr_right, H, W)

        # --- Feature Matching --- 
        # desc1_np = desc1
        # desc2_np = desc2
        
        matches_temporal = bf.knnMatch(desc_prev_left, desc_curr_left, k=2)
        good_matches_temporal = [m for m,n in matches_temporal if m.distance < 0.75*n.distance]
        
        # Ratio test to filter good matches
        # good_matches = [m for m,n in matches if m.distance<0.75*n.distance]
                
        if len(good_matches_temporal) < 8:
            print("Not enough good temporal matches. Skipping frame.")
            continue
            
        pts_prev_left = np.float32([kp_prev_left[m.queryIdx] for m in good_matches_temporal]).reshape(-1, 2)
        pts_curr_left = np.float32([kp_curr_left[m.trainIdx] for m in good_matches_temporal]).reshape(-1, 2)
        
        # --- Pose Estimation ---
        # Find Essential Matrix
        E, mask = cv2.findEssentialMat(pts_prev_left, pts_curr_left, cameraMatrix=K, method=cv2.RANSAC, prob=0.999, threshold=1.0)

        # Recover Pose 
        _, R, t, _ = cv2.recoverPose(E, pts_prev_left, pts_curr_left, cameraMatrix=K, mask=mask)
        
        # --- Update Trajectory ---
        t_world = R_total @ t + t_total
        R_total = R_total @ R
        t_total = t_world
        trajectory.append(t_total.flatten())

        # Match stereo pairs for 3D keypoint visualization
        matches_stereo_curr = bf.knnMatch(desc_curr_left, desc_curr_right, k=2)
        good_matches_stereo = [m for m,n in matches_stereo_curr if m.distance < 0.75*n.distance]

        if len(good_matches_stereo) > 0:
            # Get matched keypoints
            kp_curr_left_matched = np.float32([kp_curr_left[m.queryIdx] for m in good_matches_stereo])
            kp_curr_right_matched = np.float32([kp_curr_right[m.trainIdx] for m in good_matches_stereo])
            
            baseline = 0.12
            # Triangulate 3D points
            points_3d = stream.triangulate_3d_points(kp_curr_left_matched, kp_curr_right_matched, K, baseline)
            
            # Filter points within reasonable depth range
            if len(points_3d) > 0:
                valid_points = points_3d[(points_3d[:, 2] > 0.5) & (points_3d[:, 2] < 50.0)]  # 0.5m to 50m
                
                if len(valid_points) > 0:
                    # Transform 3D points to world coordinates
                    points_3d_world = (R_total @ valid_points.T + t_total).T
                    
                    # Update 3D keypoints scatter plot
                    keypoints_scatter.remove()
                    keypoints_scatter = ax.scatter(points_3d_world[:, 0], points_3d_world[:, 1], points_3d_world[:, 2], 
                                                 c='red', s=10, alpha=0.6, label='3D Keypoints')

        if trajectory:
            trajectory_np = np.array(trajectory)
            # Update the plot
            trajectory_plot.set_data(trajectory_np[:, 0], trajectory_np[:,1])
            trajectory_plot.set_3d_properties(trajectory_np[:, 2])
            ax.set_xlim([trajectory_np[:, 0].min() - 1, trajectory_np[:, 0].max() + 1])
            ax.set_ylim([trajectory_np[:, 1].min() - 1, trajectory_np[:, 1].max() + 1])
            ax.set_zlim([trajectory_np[:, 2].min() - 1, trajectory_np[:, 2].max() + 1])

            # Update axis limits to include both trajectory and keypoints
            all_x = trajectory_np[:, 0]
            all_y = trajectory_np[:, 1] 
            all_z = trajectory_np[:, 2]
            
            if len(valid_points) > 0:
                all_x = np.concatenate([all_x, points_3d_world[:, 0]])
                all_y = np.concatenate([all_y, points_3d_world[:, 1]])
                all_z = np.concatenate([all_z, points_3d_world[:, 2]])

            margin = 1.0
            ax.set_xlim([all_x.min() - margin, all_x.max() + margin])
            ax.set_ylim([all_y.min() - margin, all_y.max() + margin]) 
            ax.set_zlim([all_z.min() - margin, all_z.max() + margin])

            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.01)

        cv2.waitKey(1000)
        prev_left = curr_left.copy()
        prev_right = curr_right.copy()
        prev_left_tensor = curr_left_tensor.clone()
        prev_right_tensor = curr_right_tensor.clone()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stream.close()
            cv2.destroyAllWindows()
            plt.ioff()  # Turn off interactive mode
            plt.show()  # Final display (optional)
            break
    plt.show()  # Final display (optional)