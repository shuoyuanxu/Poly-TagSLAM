import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import procrustes
import re

def load_lio_sam(filepath):
    """Load LIO_SAM CSV file"""
    df = pd.read_csv(filepath)
    # Ensure we have x, y, z columns
    if 'z' not in df.columns:
        print("Warning: 'z' column not found in LIO_SAM data, setting z=0")
        df['z'] = 0
    return df

def load_total_station(filepath):
    """Load Total Station CSV file and filter out unwanted points"""
    df = pd.read_csv(filepath)
    
    # Filter out points containing 'c' or 'b' (case insensitive)
    # Keep only points that match pattern like a0, a1, etc.
    df = df[df['Point Name'].str.match(r'^a\d+$', case=False, na=False)]
    
    # Check for z/height column
    z_column = None
    for col in ['Z', 'z', 'Height', 'height', 'Z (Elevation)', 'Elevation', 'elevation']:
        if col in df.columns:
            z_column = col
            break
    
    if z_column is None:
        print("Warning: No Z/Height column found in Total Station data, setting z=0")
        df['Z'] = 0
    elif z_column != 'Z':
        df['Z'] = df[z_column]
    
    return df

def extract_id_from_point_name(point_name):
    """Extract numeric ID from point name (e.g., 'a5' -> 5)"""
    match = re.search(r'\d+', point_name)
    if match:
        return int(match.group())
    return None

def create_correspondences(lio_sam_df, total_station_df, manual_correspondences=None):
    """
    Create correspondences between LIO_SAM IDs and Total Station point names
    
    Args:
        lio_sam_df: DataFrame with LIO_SAM data
        total_station_df: DataFrame with Total Station data
        manual_correspondences: Dict mapping point_name -> id (e.g., {'a0': 0, 'a1': 1})
                               If None, will auto-match based on numeric suffix
    
    Returns:
        DataFrame with matched points
    """
    if manual_correspondences is None:
        # Auto-match: assume a0 -> 0, a1 -> 1, etc.
        manual_correspondences = {}
        for _, row in total_station_df.iterrows():
            point_name = row['Point Name']
            numeric_id = extract_id_from_point_name(point_name)
            if numeric_id is not None:
                manual_correspondences[point_name] = numeric_id
    
    matched_data = []
    for point_name, lio_id in manual_correspondences.items():
        # Find in total station
        ts_row = total_station_df[total_station_df['Point Name'] == point_name]
        if ts_row.empty:
            print(f"Warning: Point {point_name} not found in Total Station data")
            continue
            
        # Find in LIO_SAM
        lio_row = lio_sam_df[lio_sam_df['id'] == lio_id]
        if lio_row.empty:
            print(f"Warning: ID {lio_id} not found in LIO_SAM data")
            continue
        
        matched_data.append({
            'point_name': point_name,
            'lio_id': lio_id,
            'lio_x': lio_row.iloc[0]['x'],
            'lio_y': lio_row.iloc[0]['y'],
            'lio_z': lio_row.iloc[0]['z'],
            'ts_x': ts_row.iloc[0]['X (East)'],
            'ts_y': ts_row.iloc[0]['Y (North)'],
            'ts_z': ts_row.iloc[0]['Z']
        })
    
    return pd.DataFrame(matched_data)

def compute_transformation_2d(source_points, target_points):
    """
    Compute 2D rigid transformation (rotation, translation, NO scale) from source to target
    Uses only x,y coordinates
    
    Args:
        source_points: Nx2 array of source coordinates (x, y)
        target_points: Nx2 array of target coordinates (x, y)
    
    Returns:
        rotation_angle: rotation angle in radians
        translation: 2x1 translation vector
        scale: scalar scale factor (fixed at 1.0)
    """
    # Center the points
    source_center = np.mean(source_points, axis=0)
    target_center = np.mean(target_points, axis=0)
    
    source_centered = source_points - source_center
    target_centered = target_points - target_center
    
    # Compute rotation using SVD (without normalization for rigid transform)
    H = source_centered.T @ target_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # Ensure proper rotation (determinant should be 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Extract rotation angle from rotation matrix
    rotation_angle = np.arctan2(R[1, 0], R[0, 0])
    
    # Compute translation (without scale)
    translation = target_center - R @ source_center
    
    # Set scale to 1.0 (no scaling - rigid transformation only)
    scale = 1.0
    
    return rotation_angle, translation, scale, R

def compute_transformation_3d(source_points, target_points):
    """
    Compute 3D rigid transformation (rotation, translation, NO scale) from source to target
    
    Args:
        source_points: Nx3 array of source coordinates
        target_points: Nx3 array of target coordinates
    
    Returns:
        rotation_matrix: 3x3 rotation matrix
        translation: 3x1 translation vector
        scale: scalar scale factor (fixed at 1.0)
    """
    # Center the points
    source_center = np.mean(source_points, axis=0)
    target_center = np.mean(target_points, axis=0)
    
    source_centered = source_points - source_center
    target_centered = target_points - target_center
    
    # Compute rotation using SVD (without normalization for rigid transform)
    H = source_centered.T @ target_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # Ensure proper rotation (determinant should be 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Compute translation (without scale)
    translation = target_center - R @ source_center
    
    # Set scale to 1.0 (no scaling - rigid transformation only)
    scale = 1.0
    
    return R, translation, scale

def transform_points_2d(points, rotation_angle, t, s):
    """Apply 2D similarity transformation to points"""
    R = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)],
                  [np.sin(rotation_angle), np.cos(rotation_angle)]])
    return (s * points @ R.T) + t

def transform_points(points, R, t, s):
    """Apply similarity transformation to points"""
    return (s * points @ R.T) + t

def compute_errors(transformed_points, target_points):
    """Compute point-wise errors between transformed and target points"""
    errors = np.linalg.norm(transformed_points - target_points, axis=1)
    return errors

def rotation_matrix_to_euler(R):
    """Convert rotation matrix to Euler angles (roll, pitch, yaw) in degrees"""
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    
    return np.degrees([roll, pitch, yaw])

def run_2d_analysis(lio_sam_df, total_station_df, matched_df, 
                    show_lio_labels=True, show_ts_labels=True,
                    lio_label_fontsize=10, ts_label_fontsize=7,
                    enable_outlier_filtering=True, outlier_threshold=2.0,
                    max_iterations=20, n_registration_points=None):
    """
    Run 2D alignment analysis between LIO_SAM and Total Station data
    
    Args:
        lio_sam_df: DataFrame with LIO_SAM data
        total_station_df: DataFrame with Total Station data
        matched_df: DataFrame with matched correspondences
        show_lio_labels: Whether to show LIO_SAM ID labels
        show_ts_labels: Whether to show Total Station point name labels
        lio_label_fontsize: Font size for LIO_SAM labels
        ts_label_fontsize: Font size for Total Station labels
        enable_outlier_filtering: Whether to enable outlier removal
        outlier_threshold: Distance threshold for outlier detection (meters)
        max_iterations: Maximum iterations for outlier removal
        n_registration_points: Number of points to use for registration (None = all)
    """
    print("\n" + "="*60)
    print("2D ALIGNMENT ANALYSIS")
    print("="*60)
    
    if len(matched_df) < 2:
        print("Error: Need at least 2 matched points for 2D transformation!")
        return
    
    # Number of points to use for computing transformation
    if n_registration_points is None:
        n_registration_points = len(matched_df)
    else:
        n_registration_points = min(len(matched_df), n_registration_points)
    
    print(f"\nUsing {n_registration_points} points for initial registration")
    
    # Split into registration and validation sets
    registration_matches = matched_df.iloc[:n_registration_points]
    validation_matches = matched_df.iloc[n_registration_points:]
    
    # Prepare points for transformation (2D - only x,y)
    lio_reg_points = registration_matches[['lio_x', 'lio_y']].values
    ts_reg_points = registration_matches[['ts_x', 'ts_y']].values
    
    # Compute initial transformation
    print("\nComputing 2D transformation...")
    rotation_angle, t, s, R_2d = compute_transformation_2d(lio_reg_points, ts_reg_points)
    print(f"Scale: {s:.4f}")
    print(f"Rotation angle: {np.degrees(rotation_angle):.2f} degrees")
    print(f"Translation: [{t[0]:.4f}, {t[1]:.4f}]")
    
    # Outlier filtering with iterative refinement
    outliers_removed = []
    current_matched_df = matched_df.copy()
    
    if enable_outlier_filtering:
        print(f"\n{'='*60}")
        print("OUTLIER FILTERING (2D)")
        print(f"{'='*60}")
        
        for iteration in range(max_iterations):
            # Transform all current matched points (2D)
            lio_matched_points = current_matched_df[['lio_x', 'lio_y']].values
            ts_matched_points = current_matched_df[['ts_x', 'ts_y']].values
            lio_matched_transformed = transform_points_2d(lio_matched_points, rotation_angle, t, s)
            
            # Compute errors for all matched points
            errors = compute_errors(lio_matched_transformed, ts_matched_points)
            
            # Find outliers
            outlier_mask = errors > outlier_threshold
            n_outliers = np.sum(outlier_mask)
            
            if n_outliers == 0:
                print(f"\nIteration {iteration + 1}: No outliers found (threshold: {outlier_threshold}m)")
                break
            
            # Record outliers
            outlier_data = current_matched_df[outlier_mask].copy()
            outlier_data['error'] = errors[outlier_mask]
            outliers_removed.append(outlier_data)
            
            print(f"\nIteration {iteration + 1}:")
            print(f"  Found {n_outliers} outlier(s):")
            for idx, row in outlier_data.iterrows():
                print(f"    - Point {row['point_name']} (ID {row['lio_id']}): error = {row['error']:.3f}m")
            
            # Remove outliers
            current_matched_df = current_matched_df[~outlier_mask].reset_index(drop=True)
            
            if len(current_matched_df) < 2:
                print(f"  WARNING: Only {len(current_matched_df)} points remaining. Need at least 2 for 2D transformation.")
                print(f"  Reverting to previous iteration.")
                current_matched_df = matched_df[~matched_df.index.isin(pd.concat(outliers_removed[:-1]).index)].reset_index(drop=True) if len(outliers_removed) > 1 else matched_df
                outliers_removed = outliers_removed[:-1]
                break
            
            # Recompute transformation without outliers
            lio_matched_points_clean = current_matched_df[['lio_x', 'lio_y']].values
            ts_matched_points_clean = current_matched_df[['ts_x', 'ts_y']].values
            rotation_angle, t, s, R_2d = compute_transformation_2d(lio_matched_points_clean, ts_matched_points_clean)
            
            print(f"  Recomputed transformation with {len(current_matched_df)} points")
            print(f"    Scale: {s:.4f}")
            print(f"    Rotation: {np.degrees(rotation_angle):.2f}°")
        
        # Summary of outlier filtering
        if outliers_removed:
            all_outliers = pd.concat(outliers_removed)
            print(f"\n{'='*60}")
            print(f"OUTLIER FILTERING SUMMARY (2D)")
            print(f"{'='*60}")
            print(f"Total outliers removed: {len(all_outliers)}")
            print(f"Remaining matched points: {len(current_matched_df)}")
        else:
            print(f"\nNo outliers detected (all errors < {outlier_threshold}m)")
    
    # Update matched_df to the filtered version
    matched_df_filtered = current_matched_df.copy()
    
    # Transform all LIO_SAM points using final transformation
    lio_all_points = lio_sam_df[['x', 'y']].values
    lio_transformed = transform_points_2d(lio_all_points, rotation_angle, t, s)
    
    # Prepare points for final error computation
    lio_reg_points = matched_df_filtered[['lio_x', 'lio_y']].values
    ts_reg_points = matched_df_filtered[['ts_x', 'ts_y']].values
    
    # Compute registration errors
    lio_reg_transformed = transform_points_2d(lio_reg_points, rotation_angle, t, s)
    reg_errors = compute_errors(lio_reg_transformed, ts_reg_points)
    print(f"\nRegistration errors (2D):")
    print(f"  RMSE: {np.sqrt(np.mean(reg_errors**2)):.4f} m")
    print(f"  Mean: {np.mean(reg_errors):.4f} m")
    print(f"  Max:  {np.max(reg_errors):.4f} m")
    
    # Create figure for 2D plot
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111)
    
    # Plot all transformed LIO_SAM points
    ax.scatter(lio_transformed[:, 0], lio_transformed[:, 1],
                c='blue', marker='o', s=50, alpha=0.6, label='LIO_Tag (transformed)')
    
    # Plot all Total Station points
    ax.scatter(total_station_df['X (East)'], total_station_df['Y (North)'],
                c='red', marker='s', s=100, alpha=0.6, label='Total Station')
    
    # Add LIO_SAM ID labels for all transformed points
    if show_lio_labels:
        for idx, row in lio_sam_df.iterrows():
            ax.text(lio_transformed[idx, 0], lio_transformed[idx, 1],
                    str(int(row['id'])), 
                    fontsize=lio_label_fontsize, ha='center', va='center',
                    color='darkblue', fontweight='bold')
    
    # Add Total Station point name labels
    if show_ts_labels:
        for _, row in total_station_df.iterrows():
            ax.text(row['X (East)'], row['Y (North)'], row['Point Name'],
                    fontsize=ts_label_fontsize, ha='center', va='bottom',
                    color='darkred', fontweight='bold')
    
    # Highlight registration points
    reg_lio_transformed = transform_points_2d(lio_reg_points, rotation_angle, t, s)
    ax.scatter(reg_lio_transformed[:, 0], reg_lio_transformed[:, 1],
                c='cyan', marker='o', s=200, alpha=0.8,
                edgecolors='black', linewidths=2, label='Registration points')
    
    # Draw error vectors for filtered matched points (inliers)
    for i, row in enumerate(matched_df_filtered.iterrows()):
        _, row_data = row
        lio_idx = lio_sam_df[lio_sam_df['id'] == row_data['lio_id']].index[0]
        transformed_point = lio_transformed[lio_idx]
        ts_point = [row_data['ts_x'], row_data['ts_y']]
        
        color = 'green'
        label = 'Registration errors' if i == 0 else ''
        ax.plot([transformed_point[0], ts_point[0]],
                [transformed_point[1], ts_point[1]],
                color=color, alpha=0.5, linewidth=2, label=label)
    
    # Draw outliers if any
    if outliers_removed and enable_outlier_filtering:
        all_outliers = pd.concat(outliers_removed)
        for i, (idx, row_data) in enumerate(all_outliers.iterrows()):
            lio_idx = lio_sam_df[lio_sam_df['id'] == row_data['lio_id']].index[0]
            transformed_point = lio_transformed[lio_idx]
            ts_point = [row_data['ts_x'], row_data['ts_y']]
            
            # Draw outlier line in red with dashed style
            ax.plot([transformed_point[0], ts_point[0]],
                    [transformed_point[1], ts_point[1]],
                    color='red', alpha=0.7, linewidth=2, linestyle='--',
                    label='Outlier errors' if i == 0 else '')
            
            # Mark outlier points with X
            ax.scatter(transformed_point[0], transformed_point[1],
                      c='red', marker='x', s=200, linewidths=3,
                      label='Outliers' if i == 0 else '')
    
    ax.set_xlabel('X (East) [m]', fontsize=12)
    ax.set_ylabel('Y (North) [m]', fontsize=12)
    
    # Create title with outlier information if applicable
    title = '2D Landmark Comparison - Transformed to Total Station Coordinates'
    if outliers_removed and enable_outlier_filtering:
        title += f'\n({len(pd.concat(outliers_removed))} outlier(s) removed, threshold={outlier_threshold}m)'
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig('landmark_comparison_2d_only.png', dpi=300, bbox_inches='tight')
    print("\nPlot saved as 'landmark_comparison_2d_only.png'")
    plt.show()
    
    # Save transformed LIO_SAM data (2D)
    lio_sam_transformed_df = lio_sam_df.copy()
    lio_sam_transformed_df['x_transformed'] = lio_transformed[:, 0]
    lio_sam_transformed_df['y_transformed'] = lio_transformed[:, 1]
    lio_sam_transformed_df.to_csv('LIO_SAM_transformed_2d.csv', index=False)
    print("Transformed LIO_SAM data saved as 'LIO_SAM_transformed_2d.csv'")
    
    # Print summary statistics
    print(f"\n{'='*60}")
    print("2D TRANSFORMATION SUMMARY")
    print(f"{'='*60}")
    print(f"Scale factor: {s:.4f} (rigid transformation - no scaling)")
    print(f"Rotation angle: {np.degrees(rotation_angle):.2f}°")
    print(f"Translation: [{t[0]:.4f}, {t[1]:.4f}] m")
    print(f"\nRegistration RMSE: {np.sqrt(np.mean(reg_errors**2)):.4f} m")
    print(f"Number of points used: {len(matched_df_filtered)}")

def run_3d_analysis(lio_sam_df, total_station_df, matched_df,
                    show_lio_labels=True, show_ts_labels=True,
                    lio_label_fontsize=10, ts_label_fontsize=7,
                    enable_outlier_filtering=True, outlier_threshold=2.0,
                    max_iterations=20, n_registration_points=None):
    """
    Run 3D alignment analysis between LIO_SAM and Total Station data
    
    Args:
        lio_sam_df: DataFrame with LIO_SAM data
        total_station_df: DataFrame with Total Station data
        matched_df: DataFrame with matched correspondences
        show_lio_labels: Whether to show LIO_SAM ID labels
        show_ts_labels: Whether to show Total Station point name labels
        lio_label_fontsize: Font size for LIO_SAM labels
        ts_label_fontsize: Font size for Total Station labels
        enable_outlier_filtering: Whether to enable outlier removal
        outlier_threshold: Distance threshold for outlier detection (meters)
        max_iterations: Maximum iterations for outlier removal
        n_registration_points: Number of points to use for registration (None = all)
    """
    print("\n" + "="*60)
    print("3D ALIGNMENT ANALYSIS")
    print("="*60)
    
    if len(matched_df) < 3:
        print("Error: Need at least 3 matched points for 3D transformation!")
        return
    
    # Number of points to use for computing transformation
    if n_registration_points is None:
        n_registration_points = len(matched_df)
    else:
        n_registration_points = min(len(matched_df), n_registration_points)
    
    print(f"\nUsing {n_registration_points} points for initial registration")
    
    # Split into registration and validation sets
    registration_matches = matched_df.iloc[:n_registration_points]
    validation_matches = matched_df.iloc[n_registration_points:]
    
    # Prepare points for transformation (3D)
    lio_reg_points = registration_matches[['lio_x', 'lio_y', 'lio_z']].values
    ts_reg_points = registration_matches[['ts_x', 'ts_y', 'ts_z']].values
    
    # Compute transformation
    print("\nComputing 3D transformation...")
    R, t, s = compute_transformation_3d(lio_reg_points, ts_reg_points)
    print(f"Scale: {s:.4f}")
    
    # Convert rotation to Euler angles for easier interpretation
    roll, pitch, yaw = rotation_matrix_to_euler(R)
    print(f"Rotation (Euler angles):")
    print(f"  Roll:  {roll:.2f} degrees")
    print(f"  Pitch: {pitch:.2f} degrees")
    print(f"  Yaw:   {yaw:.2f} degrees")
    print(f"Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")
    
    # Outlier filtering with iterative refinement
    outliers_removed = []
    current_matched_df = matched_df.copy()
    
    if enable_outlier_filtering:
        print(f"\n{'='*60}")
        print("OUTLIER FILTERING (3D)")
        print(f"{'='*60}")
        
        for iteration in range(max_iterations):
            # Transform all current matched points
            lio_matched_points = current_matched_df[['lio_x', 'lio_y', 'lio_z']].values
            ts_matched_points = current_matched_df[['ts_x', 'ts_y', 'ts_z']].values
            lio_matched_transformed = transform_points(lio_matched_points, R, t, s)
            
            # Compute errors for all matched points
            errors = compute_errors(lio_matched_transformed, ts_matched_points)
            
            # Find outliers
            outlier_mask = errors > outlier_threshold
            n_outliers = np.sum(outlier_mask)
            
            if n_outliers == 0:
                print(f"\nIteration {iteration + 1}: No outliers found (threshold: {outlier_threshold}m)")
                break
            
            # Record outliers
            outlier_data = current_matched_df[outlier_mask].copy()
            outlier_data['error'] = errors[outlier_mask]
            outliers_removed.append(outlier_data)
            
            print(f"\nIteration {iteration + 1}:")
            print(f"  Found {n_outliers} outlier(s):")
            for idx, row in outlier_data.iterrows():
                print(f"    - Point {row['point_name']} (ID {row['lio_id']}): error = {row['error']:.3f}m")
            
            # Remove outliers
            current_matched_df = current_matched_df[~outlier_mask].reset_index(drop=True)
            
            if len(current_matched_df) < 3:
                print(f"  WARNING: Only {len(current_matched_df)} points remaining. Need at least 3 for transformation.")
                print(f"  Reverting to previous iteration.")
                current_matched_df = matched_df[~matched_df.index.isin(pd.concat(outliers_removed[:-1]).index)].reset_index(drop=True) if len(outliers_removed) > 1 else matched_df
                outliers_removed = outliers_removed[:-1]
                break
            
            # Recompute transformation without outliers
            lio_matched_points_clean = current_matched_df[['lio_x', 'lio_y', 'lio_z']].values
            ts_matched_points_clean = current_matched_df[['ts_x', 'ts_y', 'ts_z']].values
            R, t, s = compute_transformation_3d(lio_matched_points_clean, ts_matched_points_clean)
            
            roll, pitch, yaw = rotation_matrix_to_euler(R)
            print(f"  Recomputed transformation with {len(current_matched_df)} points")
            print(f"    Scale: {s:.4f}")
            print(f"    Rotation: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")
        
        # Summary of outlier filtering
        if outliers_removed:
            all_outliers = pd.concat(outliers_removed)
            print(f"\n{'='*60}")
            print(f"OUTLIER FILTERING SUMMARY (3D)")
            print(f"{'='*60}")
            print(f"Total outliers removed: {len(all_outliers)}")
            print(f"Remaining matched points: {len(current_matched_df)}")
        else:
            print(f"\nNo outliers detected (all errors < {outlier_threshold}m)")
    
    # Update matched_df to the filtered version
    matched_df_filtered = current_matched_df.copy()
    
    # Transform all LIO_SAM points using final transformation
    lio_all_points = lio_sam_df[['x', 'y', 'z']].values
    lio_transformed = transform_points(lio_all_points, R, t, s)
    
    # Prepare points for final error computation
    lio_reg_points = matched_df_filtered[['lio_x', 'lio_y', 'lio_z']].values
    ts_reg_points = matched_df_filtered[['ts_x', 'ts_y', 'ts_z']].values
    
    # Compute registration errors
    lio_reg_transformed = transform_points(lio_reg_points, R, t, s)
    reg_errors = compute_errors(lio_reg_transformed, ts_reg_points)
    print(f"\nRegistration errors (3D):")
    print(f"  RMSE: {np.sqrt(np.mean(reg_errors**2)):.4f} m")
    print(f"  Mean: {np.mean(reg_errors):.4f} m")
    print(f"  Max:  {np.max(reg_errors):.4f} m")
    
    # Create figure with single 2D plot (East-North view)
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111)
    
    # Plot all transformed LIO_SAM points
    ax.scatter(lio_transformed[:, 0], lio_transformed[:, 1],
                c='blue', marker='o', s=50, alpha=0.6, label='LIO_Tag (transformed)')
    
    # Plot all Total Station points
    ax.scatter(total_station_df['X (East)'], total_station_df['Y (North)'],
                c='red', marker='s', s=100, alpha=0.6, label='Total Station')
    
    # Add LIO_SAM ID labels for all transformed points
    if show_lio_labels:
        for idx, row in lio_sam_df.iterrows():
            ax.text(lio_transformed[idx, 0], lio_transformed[idx, 1],
                    str(int(row['id'])), 
                    fontsize=lio_label_fontsize, ha='center', va='center',
                    color='darkblue', fontweight='bold')
    
    # Add Total Station point name labels
    if show_ts_labels:
        for _, row in total_station_df.iterrows():
            ax.text(row['X (East)'], row['Y (North)'], row['Point Name'],
                    fontsize=ts_label_fontsize, ha='center', va='bottom',
                    color='darkred', fontweight='bold')
    
    # Highlight registration points
    reg_lio_transformed = transform_points(lio_reg_points, R, t, s)
    ax.scatter(reg_lio_transformed[:, 0], reg_lio_transformed[:, 1],
                c='cyan', marker='o', s=200, alpha=0.8,
                edgecolors='black', linewidths=2, label='Registration points')
    
    # Draw error vectors for filtered matched points (inliers)
    for i, row in enumerate(matched_df_filtered.iterrows()):
        _, row_data = row
        lio_idx = lio_sam_df[lio_sam_df['id'] == row_data['lio_id']].index[0]
        transformed_point = lio_transformed[lio_idx]
        ts_point = [row_data['ts_x'], row_data['ts_y']]
        
        color = 'green'
        label = 'Registration errors' if i == 0 else ''
        ax.plot([transformed_point[0], ts_point[0]],
                [transformed_point[1], ts_point[1]],
                color=color, alpha=0.5, linewidth=2, label=label)
    
    # Draw outliers if any
    if outliers_removed and enable_outlier_filtering:
        all_outliers = pd.concat(outliers_removed)
        for i, (idx, row_data) in enumerate(all_outliers.iterrows()):
            lio_idx = lio_sam_df[lio_sam_df['id'] == row_data['lio_id']].index[0]
            transformed_point = lio_transformed[lio_idx]
            ts_point = [row_data['ts_x'], row_data['ts_y']]
            
            # Draw outlier line in red with dashed style
            ax.plot([transformed_point[0], ts_point[0]],
                    [transformed_point[1], ts_point[1]],
                    color='red', alpha=0.7, linewidth=2, linestyle='--',
                    label='Outlier errors' if i == 0 else '')
            
            # Mark outlier points with X
            ax.scatter(transformed_point[0], transformed_point[1],
                      c='red', marker='x', s=200, linewidths=3,
                      label='Outliers' if i == 0 else '')
    
    ax.set_xlabel('X (East) [m]', fontsize=12)
    ax.set_ylabel('Y (North) [m]', fontsize=12)
    
    # Create title with outlier information if applicable
    title = '2D View of 3D Landmark Comparison - Transformed to Total Station Coordinates (East-North)'
    if outliers_removed and enable_outlier_filtering:
        title += f'\n({len(pd.concat(outliers_removed))} outlier(s) removed, threshold={outlier_threshold}m)'
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig('landmark_comparison_3d.png', dpi=300, bbox_inches='tight')
    print("\nPlot saved as 'landmark_comparison_3d.png'")
    plt.show()
    
    # Save transformed LIO_SAM data
    lio_sam_transformed_df = lio_sam_df.copy()
    lio_sam_transformed_df['x_transformed'] = lio_transformed[:, 0]
    lio_sam_transformed_df['y_transformed'] = lio_transformed[:, 1]
    lio_sam_transformed_df['z_transformed'] = lio_transformed[:, 2]
    lio_sam_transformed_df.to_csv('LIO_SAM_transformed_3d.csv', index=False)
    print("Transformed LIO_SAM data saved as 'LIO_SAM_transformed_3d.csv'")
    
    # Print summary statistics
    print(f"\n{'='*60}")
    print("3D TRANSFORMATION SUMMARY")
    print(f"{'='*60}")
    print(f"Scale factor: {s:.4f} (rigid transformation - no scaling)")
    print(f"Rotation (Euler angles):")
    print(f"  Roll:  {roll:.2f}°")
    print(f"  Pitch: {pitch:.2f}°")
    print(f"  Yaw:   {yaw:.2f}°")
    print(f"Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m")
    print(f"\nRegistration RMSE: {np.sqrt(np.mean(reg_errors**2)):.4f} m")
    print(f"Number of points used: {len(matched_df_filtered)}")

def main():
    # ============ CONFIGURATION ============
    
    # File paths - MODIFY THESE
    lio_sam_file = 'DlojuneMapping.csv'
    total_station_file = 'TotalStation_idfixed.csv'
    
    # Analysis mode: '2d', '3d', or 'both'
    analysis_mode = 'both'  # MODIFY THIS
    
    # Label display options
    show_lio_labels = True  # Show LIO_SAM ID labels
    show_ts_labels = True   # Show Total Station point name labels
    lio_label_fontsize = 10  # Font size for LIO_SAM labels
    ts_label_fontsize = 7   # Font size for Total Station labels
    
    # Outlier filtering options
    enable_outlier_filtering = True  # Enable outlier removal
    outlier_threshold = 2.0          # Distance threshold in meters
    max_iterations = 20               # Maximum refinement iterations
    
    # Number of points to use for registration (None = use all matched points)
    n_registration_points = None
    
    # ======================================
    
    # Load data
    print("Loading data...")
    lio_sam_df = load_lio_sam(lio_sam_file)
    total_station_df = load_total_station(total_station_file)
    
    print(f"LIO_SAM: {len(lio_sam_df)} landmarks")
    print(f"Total Station: {len(total_station_df)} landmarks (after filtering)")
    print(f"\nTotal Station points: {sorted(total_station_df['Point Name'].tolist())}")
    
    # Define correspondences
    # Option 1: Auto-match (assumes a0->0, a1->1, etc.)
    correspondences = None  # Will auto-match
    
    # Option 2: Manual correspondences (uncomment and modify as needed)
    # correspondences = {
    #     'a0': 0,
    #     'a1': 1,
    #     'a2': 2,
    #     'a3': 3,
    #     'a5': 5,
    # }
    
    # Create matched pairs
    print("\nCreating correspondences...")
    matched_df = create_correspondences(lio_sam_df, total_station_df, correspondences)
    print(f"Found {len(matched_df)} matched pairs")
    print(matched_df[['point_name', 'lio_id']])
    
    # Run analyses based on mode
    if analysis_mode in ['2d', 'both']:
        run_2d_analysis(lio_sam_df, total_station_df, matched_df,
                       show_lio_labels=show_lio_labels,
                       show_ts_labels=show_ts_labels,
                       lio_label_fontsize=lio_label_fontsize,
                       ts_label_fontsize=ts_label_fontsize,
                       enable_outlier_filtering=enable_outlier_filtering,
                       outlier_threshold=outlier_threshold,
                       max_iterations=max_iterations,
                       n_registration_points=n_registration_points)
    
    if analysis_mode in ['3d', 'both']:
        run_3d_analysis(lio_sam_df, total_station_df, matched_df,
                       show_lio_labels=show_lio_labels,
                       show_ts_labels=show_ts_labels,
                       lio_label_fontsize=lio_label_fontsize,
                       ts_label_fontsize=ts_label_fontsize,
                       enable_outlier_filtering=enable_outlier_filtering,
                       outlier_threshold=outlier_threshold,
                       max_iterations=max_iterations,
                       n_registration_points=n_registration_points)

if __name__ == "__main__":
    main()
