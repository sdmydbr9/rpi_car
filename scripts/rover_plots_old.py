import os
import glob
import pandas as pd
import matplotlib
# MUST BE SET BEFORE IMPORTING PYPLOT FOR HEADLESS OPERATION
matplotlib.use('Agg') 
import matplotlib.pyplot as plt

def get_latest_log():
    # Find all CSV files in the rover_logs directory
    list_of_files = glob.glob('rover_logs/*.csv')
    if not list_of_files:
        print("❌ No log files found in the 'rover_logs' directory.")
        return None
    # Return the file with the most recent creation/modification time
    latest_file = max(list_of_files, key=os.path.getctime)
    return latest_file

def generate_dashboard():
    log_file = get_latest_log()
    if not log_file: return

    print(f"📊 Loading data from: {log_file}")
    
    # Read the CSV
    df = pd.read_csv(log_file)
    
    # Convert Timestamp to relative seconds starting from 0
    df['Time_Sec'] = df['Timestamp'] - df['Timestamp'].iloc[0]

    # Create a figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 14), sharex=True)
    fig.suptitle(f"Rover Telemetry Dashboard\n{os.path.basename(log_file)}", fontsize=16, fontweight='bold')

    # --- PLOT 1: Throttle vs. Sonar Distance ---
    ax1.plot(df['Time_Sec'], df['Sonar_Dist_cm'], label='Sonar Distance (cm)', color='blue', alpha=0.7)
    ax1.set_ylabel('Distance (cm)', color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1.axhline(40, color='red', linestyle='--', alpha=0.5, label='Min Speed Dist (40cm)')
    
    ax1_twin = ax1.twinx()
    ax1_twin.plot(df['Time_Sec'], df['Target_Throttle'], label='Target Throttle (%)', color='green', linewidth=2)
    ax1_twin.set_ylabel('Throttle %', color='green')
    ax1_twin.tick_params(axis='y', labelcolor='green')
    ax1.set_title("Speed Control vs. Forward Obstacles")
    
    # Combine legends for ax1
    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_twin.get_legend_handles_labels()
    ax1_twin.legend(lines + lines2, labels + labels2, loc='upper right')

    # --- PLOT 2: Laser Clearances vs. Steering ---
    ax2.plot(df['Time_Sec'], df['Laser_Left_cm'], label='Left Clearance', color='orange', linestyle='--')
    ax2.plot(df['Time_Sec'], df['Laser_Right_cm'], label='Right Clearance', color='purple', linestyle='--')
    ax2.set_ylabel('Clearance (cm)')
    
    ax2_twin = ax2.twinx()
    ax2_twin.plot(df['Time_Sec'], df['Target_Steering'], label='Steering Command', color='black', linewidth=2)
    ax2_twin.set_ylabel('Steering (-100 L to 100 R)', color='black')
    ax2_twin.set_ylim(-110, 110)
    ax2.set_title("Slalom Logic: Laser Clearances vs. Steering Command")
    
    lines, labels = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2_twin.legend(lines + lines2, labels + labels2, loc='upper right')

    # --- PLOT 3: Steering Command vs. Actual Gyro Yaw ---
    ax3.plot(df['Time_Sec'], df['Target_Steering'], label='Steering Command', color='black', alpha=0.4)
    ax3.set_ylabel('Steering Cmd (-100 to 100)')
    ax3.set_ylim(-110, 110)
    
    ax3_twin = ax3.twinx()
    ax3_twin.plot(df['Time_Sec'], df['Gyro_Z_degs'], label='Actual Yaw (Gyro Z °/s)', color='red', linewidth=2)
    ax3_twin.set_ylabel('Yaw Rate (°/s)', color='red')
    ax3.set_title("Physical Response: Commanded Steering vs. Actual Rotation")
    
    lines, labels = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3_twin.get_legend_handles_labels()
    ax3_twin.legend(lines + lines2, labels + labels2, loc='upper right')

    ax3.set_xlabel('Time (Seconds)')

    # Formatting and saving
    plt.tight_layout(rect=[0, 0.03, 1, 0.96]) # Adjust for suptitle
    
    output_filename = "latest_rover_dashboard.png"
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"✅ Dashboard generated successfully! Saved as: {output_filename}")

if __name__ == "__main__":
    generate_dashboard()
