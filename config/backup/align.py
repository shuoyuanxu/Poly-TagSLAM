import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def read_landmarks(file_path):
    """
    Reads a CSV file containing landmark data with columns: id, x, y.
    Returns a DataFrame.
    """
    df = pd.read_csv(file_path)
    return df[['id', 'x', 'y']]

def kabsch_algorithm(P, Q):
    """
    Computes the optimal rotation matrix using the Kabsch algorithm.
    """
    # Center the point clouds
    P_mean = np.mean(P, axis=0)
    Q_mean = np.mean(Q, axis=0)
    P_centered = P - P_mean
    Q_centered = Q - Q_mean

    # Compute covariance matrix
    H = P_centered.T @ Q_centered

    # Singular Value Decomposition
    U, S, Vt = np.linalg.svd(H)

    # Compute rotation
    R = Vt.T @ U.T

    # Special reflection case
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute translation
    t = Q_mean.T - R @ P_mean.T

    return R, t

def plot_landmarks(P_transformed, Q):
    """
    Plots the transformed landmarks against the ground truth landmarks.
    """
    plt.figure(figsize=(8, 6))
    plt.scatter(Q[:, 0], Q[:, 1], c='blue', label='Ground Truth')
    plt.scatter(P_transformed[:, 0], P_transformed[:, 1], c='red', marker='x', label='Transformed Landmarks')
    plt.title('Transformed Landmarks vs Ground Truth')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def main(file1, file2, num_tags):
    # Read landmark data
    df1 = read_landmarks(file1)
    df2 = read_landmarks(file2)

    # Merge datasets on 'id'
    merged_df = pd.merge(df1, df2, on='id', suffixes=('_1', '_2'))

    # Optionally, limit to the first 'num_tags' landmarks
    if num_tags is not None:
        merged_df = merged_df.head(num_tags)

    # Extract matched coordinates
    P = merged_df[['x_1', 'y_1']].to_numpy()
    Q = merged_df[['x_2', 'y_2']].to_numpy()

    # Compute rotation and translation
    R, t = kabsch_algorithm(P, Q)

    print("Rotation Matrix:")
    print(R)
    print("\nTranslation Vector:")
    print(t)

    # Apply transformation to all points in df1
    points1 = df1[['x', 'y']].to_numpy()
    transformed_points = (R @ points1.T).T + t.T

    # Save transformed points to a new CSV file
    df1_transformed = df1.copy()
    df1_transformed['x'] = transformed_points[:, 0]
    df1_transformed['y'] = transformed_points[:, 1]
    df1_transformed.to_csv('transformed_landmarks.csv', index=False)
    print("\nTransformed landmarks saved to 'transformed_landmarks.csv'.")

    # Plot the transformed landmarks against the ground truth
    plot_landmarks(transformed_points, df2[['x', 'y']].to_numpy())

if __name__ == "__main__":
    # Example usage:
    # Replace 'landmarks.csv' and 'ground_truth.csv' with your file paths.
    # Set num_tags to the desired number of landmarks to use (or None to use all).
    main('landmark_best.csv', 'ground_truth.csv', num_tags=None)

