import pandas as pd
import matplotlib.pyplot as plt
import sys
import glob
import os

def plot_platoon_data(csv_file):
    df = pd.read_csv(csv_file)
    df['time'] = (df['timestamp'] - df['timestamp'].iloc[0]) / 1e9

    # Set up figure: 2 rows, 1 column (matching reference layout)
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(12, 10))

    # Plot Positions (keeping original column names: l_x, f1_x, f2_x, f3_x)
    ax_pos.plot(df['time'], df['l_x'], label='Leader', color='black', linewidth=1.5, linestyle='-')
    ax_pos.plot(df['time'], df['f1_x'], label='Follower 1', color='green', linewidth=1.5, linestyle='-')
    ax_pos.plot(df['time'], df['f2_x'], label='Follower 2', color='red', linewidth=1.5, linestyle='-')
    ax_pos.plot(df['time'], df['f3_x'], label='Follower 3', color='blue', linewidth=1.5, linestyle='-')
    ax_pos.set_ylabel('$y_k(t)$ [m]', fontsize=12)
    ax_pos.set_title('Position', fontsize=14)
    ax_pos.grid(True, linestyle='--', alpha=0.7, color='grey')
    ax_pos.legend(loc='lower right')

    # Plot Velocities (keeping original column names: l_v, f1_v, f2_v, f3_v)
    ax_vel.plot(df['time'], df['l_v'], label='Leader', color='black', linewidth=1.5, linestyle='-')
    ax_vel.plot(df['time'], df['f1_v'], label='Follower 1', color='green', linewidth=1.5, linestyle='-')
    ax_vel.plot(df['time'], df['f2_v'], label='Follower 2', color='red', linewidth=1.5, linestyle='-')
    ax_vel.plot(df['time'], df['f3_v'], label='Follower 3', color='blue', linewidth=1.5, linestyle='-')
    ax_vel.set_ylabel('$v_k(t)$ [m/s]', fontsize=12)
    ax_vel.set_xlabel('t [sec]', fontsize=12)
    ax_vel.set_title('Velocity v(t)', fontsize=14)
    ax_vel.grid(True, linestyle='--', alpha=0.7, color='grey')
    ax_vel.legend(loc='upper right')

    plt.tight_layout()
    plt.show()

def main():
    list_of_files = glob.glob('platoon_data_*.csv')
    if not list_of_files:
        print("No data found!")
        return
    csv_file = max(list_of_files, key=os.path.getctime)
    plot_platoon_data(csv_file)

if __name__ == '__main__':
    main()