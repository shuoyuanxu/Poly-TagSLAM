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

def main():
    # ============ CONFIGURATION ============
    
    # File paths - MODIFY THESE
    lio_sam_file = 'afteroptimisation.csv'
    total_station_file = 'TotalStation_idfixed.csv'
    
    # Label display options
    show_lio_labels = True  # Show LIO_SAM ID labels
    show_ts_labels = True   # Show Total Station point name labels
    lio_label_fontsize = 10  # Font size for LIO_SAM labels
    ts_label_fontsize = 7   # Font size for Total Station labels
    
    # Outlier filtering options
    enable_outlier_filtering = True  # Enable outlier removal
    outlier_threshold = 2.0          # Distance threshold in meters
    max_iterations = 20               # Maximum refinement iterations
    
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
    
    if len(matched_df) < 3:
        print("Error: Need at least 3 matched points for transformation!")
        return
    
    # Number of points to use for computing transformation
    n_registration_points = min(len(matched_df), 3)  # MODIFY THIS
    print(f"\nUsing {n_registration_points} points for registration")
    
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
        print("OUTLIER FILTERING")
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
            print(f"OUTLIER FILTERING SUMMARY")
            print(f"{'='*60}")
            print(f"Total outliers removed: {len(all_outliers)}")
            print(f"Remaining matched points: {len(current_matched_df)}")
        else:
            print(f"\nNo outliers detected (all errors < {outlier_threshold}m)")
    
    # Update matched_df to the filtered version
    matched_df_filtered = current_matched_df.copy()
    
    # Split into registration and validation sets (using filtered data)
    n_registration_points = min(len(matched_df_filtered), 5)  # MODIFY THIS
    print(f"\nUsing {n_registration_points} points for final registration")
    
    registration_matches = matched_df_filtered.iloc[:n_registration_points]
    validation_matches = matched_df_filtered.iloc[n_registration_points:]
    
    # Transform all LIO_SAM points using final transformation
    lio_all_points = lio_sam_df[['x', 'y', 'z']].values
    lio_transformed = transform_points(lio_all_points, R, t, s)
    
    # Prepare points for final error computation
    lio_reg_points = registration_matches[['lio_x', 'lio_y', 'lio_z']].values
    ts_reg_points = registration_matches[['ts_x', 'ts_y', 'ts_z']].values
    
    # Compute registration errors
    lio_reg_transformed = transform_points(lio_reg_points, R, t, s)
    reg_errors = compute_errors(lio_reg_transformed, ts_reg_points)
    print(f"\nRegistration errors (3D):")
    print(f"  RMSE: {np.sqrt(np.mean(reg_errors**2)):.4f} m")
    print(f"  Mean: {np.mean(reg_errors):.4f} m")
    print(f"  Max:  {np.max(reg_errors):.4f} m")
    
    # Compute validation errors if we have validation points
    if len(validation_matches) > 0:
        lio_val_points = validation_matches[['lio_x', 'lio_y', 'lio_z']].values
        ts_val_points = validation_matches[['ts_x', 'ts_y', 'ts_z']].values
        lio_val_transformed = transform_points(lio_val_points, R, t, s)
        val_errors = compute_errors(lio_val_transformed, ts_val_points)
        print(f"\nValidation errors (3D):")
        print(f"  RMSE: {np.sqrt(np.mean(val_errors**2)):.4f} m")
        print(f"  Mean: {np.mean(val_errors):.4f} m")
        print(f"  Max:  {np.max(val_errors):.4f} m")
    
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
        
        color = 'green' if i < n_registration_points else 'orange'
        label = 'Registration errors' if i == 0 else ('Validation errors' if i == n_registration_points else '')
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
    title = '2D Landmark Comparison - Transformed to Total Station Coordinates (East-North View)'
    if outliers_removed and enable_outlier_filtering:
        title += f'\n({len(pd.concat(outliers_removed))} outlier(s) removed, threshold={outlier_threshold}m)'
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig('landmark_comparison_2d.png', dpi=300, bbox_inches='tight')
    print("\nPlot saved as 'landmark_comparison_2d.png'")
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
    print("TRANSFORMATION SUMMARY")
    print(f"{'='*60}")
    print(f"Scale factor: {s:.4f} (rigid transformation - no scaling)")
    print(f"Rotation (Euler angles):")
    print(f"  Roll:  {roll:.2f}°")
    print(f"  Pitch: {pitch:.2f}°")
    print(f"  Yaw:   {yaw:.2f}°")
    print(f"Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m")
    print(f"\nRegistration RMSE: {np.sqrt(np.mean(reg_errors**2)):.4f} m")
    if len(validation_matches) > 0:
        print(f"Validation RMSE: {np.sqrt(np.mean(val_errors**2)):.4f} m")

if __name__ == "__main__":
    main()
