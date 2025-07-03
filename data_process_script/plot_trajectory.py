import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

LANE_WIDTH = 3.5

def calculate_lane_boundaries(points):
    """Calculates the left and right boundaries of a lane.

    Args:
        points (np.ndarray): An array of shape (N, 2) representing the centerline points.

    Returns:
        tuple: A tuple containing two numpy arrays for the left and right boundaries.
    """
    left_boundary = []
    right_boundary = []

    for i in range(len(points)):
        if i == 0:
            # Forward difference for the first point
            tangent = points[i + 1] - points[i]
        elif i == len(points) - 1:
            # Backward difference for the last point
            tangent = points[i] - points[i - 1]
        else:
            # Central difference for intermediate points
            tangent = points[i + 1] - points[i - 1]

        normal = np.array([-tangent[1], tangent[0]])
        normal /= np.linalg.norm(normal)

        left_point = points[i] + (LANE_WIDTH / 2) * normal
        right_point = points[i] - (LANE_WIDTH / 2) * normal

        left_boundary.append(left_point)
        right_boundary.append(right_point)

    return np.array(left_boundary), np.array(right_boundary)

def draw_lanes(lane_files_directory):
    """Reads lane data from CSV files and plots the lane boundaries.

    Args:
        lane_files_directory (str): The path to the directory containing lane CSV files.
    """
    # plt.figure()

    for i in range(1, 5):
        file_path = os.path.join(lane_files_directory, f'lane{i}.csv')
        if os.path.exists(file_path):
            df = pd.read_csv(file_path)
            centerline_points = df[['x', 'y']].values

            left_boundary, right_boundary = calculate_lane_boundaries(centerline_points)

            plt.plot(-centerline_points[:, 1],centerline_points[:, 0],  '--', color='gray', label=f'Centerline {i}', alpha=0.5, linewidth=1)
            plt.plot(-left_boundary[:, 1],left_boundary[:, 0],  color='black', label=f'Lane {i}', linewidth=1.5)
            plt.plot(-right_boundary[:, 1],right_boundary[:, 0],  color='black', linewidth=1.5)
        else:
            print(f"Warning: {file_path} not found.")

    # plt.xlabel('X Coordinate')
    # plt.ylabel('Y Coordinate')
    # plt.title('Lane Boundaries')
    # plt.legend()
    # plt.grid(True)

    # plt.savefig('lanes_plot.png', dpi=300)

def plot_trajectories_from_file(file_path, output_dir):
    """Plots trajectories from a single Excel file.

    Args:
        file_path (str): The path to the Excel file.
        output_dir (str): The directory to save the plot image.
    """
    try:
        xls = pd.ExcelFile(file_path)
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
        return

    vehicle_sheets = [sheet for sheet in xls.sheet_names if sheet.startswith('vehicle_')]
    if not vehicle_sheets:
        return

    # Identify the ego vehicle (the one with the smallest ID)
    ego_sheet = min(vehicle_sheets, key=lambda s: int(s.split('_')[1]))

    plt.figure(figsize=(16, 5))
    plt.style.use('seaborn-whitegrid')

    # Plot other vehicles first
    for sheet_name in vehicle_sheets:
        if sheet_name == ego_sheet:
            continue
        df = xls.parse(sheet_name)
        if 'x' in df.columns and 'y' in df.columns:
            plt.plot(df['y'].to_numpy(), df['x'].to_numpy(), label=sheet_name, alpha=0.8, linewidth=2.5, linestyle='--')


    draw_lanes('/home/xzh2/ros1/gpir_Modify/rosrecord/lanes')
    # Plot ego vehicle
    df_ego = xls.parse(ego_sheet)
    if 'x' in df_ego.columns and 'y' in df_ego.columns:
        plt.plot(df_ego['y'].to_numpy(), df_ego['x'].to_numpy(), label=f"{ego_sheet} (Ego)", color='red', linewidth=4, linestyle='-')

        # Add time annotations
        if not df_ego.empty and df_ego.columns[0].lower() in ['time', 'timestamp']:
            time_col = df_ego.columns[0]
            start_time = df_ego[time_col].iloc[0]
            df_ego['relative_time'] = df_ego[time_col] - start_time

            last_annotated_time = -np.inf
            for _, row in df_ego.iterrows():
                if row['relative_time'] >= last_annotated_time + 0.5:
                    plt.plot(row['y'], row['x'], 'o', color='blue', markersize=8)
                    plt.text(row['y'], row['x'] + 0.5, f"{row['relative_time']:.1f}s", fontsize=10, ha='center', va='bottom', bbox=dict(facecolor='white', edgecolor='none', alpha=0.7))
                    last_annotated_time = row['relative_time']

    

    # plt.title(f'Trajectory from {os.path.basename(file_path)}', fontsize=18, pad=20)
    plt.xlabel('Y', fontsize=14, labelpad=10)
    plt.ylabel('X', fontsize=14, labelpad=10)
    # plt.legend(fontsize=12, loc='best', framealpha=0.8)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlim(0,175)
    plt.ylim(-16,-4)
    plt.gca().invert_xaxis()
    plt.tight_layout()

    output_filename = os.path.join(output_dir, f"{os.path.splitext(os.path.basename(file_path))[0]}.png")
    plt.savefig(output_filename)
    plt.close()
    print(f"Saved plot to {output_filename}")

def main():
    """Main function to process all Excel files in the directory."""
    # The user mentioned path is relative to the workspace root, so we construct the absolute path.
    # Assuming the script is run from the workspace root.
    input_dir = '/home/xzh2/ros1/gpir_Modify/rosrecord/Exp1/Exp1-main/result-for-plot-traj'
    output_dir = '/home/xzh2/ros1/gpir_Modify/rosrecord/Exp1/Exp1-main/result-for-plot-traj/plots'

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for filename in os.listdir(input_dir):
        if filename.endswith('.xlsx'):
            file_path = os.path.join(input_dir, filename)
            plot_trajectories_from_file(file_path, output_dir)

if __name__ == '__main__':
    # Before running, make sure you have the required libraries installed:
    # pip install pandas openpyxl matplotlib
    main()