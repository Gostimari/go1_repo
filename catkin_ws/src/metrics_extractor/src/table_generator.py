import pandas as pd
import numpy as np
from pathlib import Path
import io
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Initialize storage lists
file_distance_info = []
file_distance_info_raw = []
all_velocities = []
last_distances = []
last_instabilities = []
file_torque_means = []
mean_foot_force = []
mean_power_consumption = []
mean_voltage_consumption = []
elapsed_time_s = []
position_x = []
position_y = []
position_z = []

goal_counts = {'REACHED GOAL 1': 0, 'REACHED GOAL 2': 0, 'REACHED GOAL 3': 0}

# Define column order
COLUMNS = ['travelled_distance', 'instant_velocity', 
          'instability_index', 'mean_torque', 'mean_foot_force', 'power_consumption', 'voltage_consumption']

COLUMNS_RAW = ['position_x', 'position_y', 
              'position_z', 'elapsed_time_s', 'elapse_time_ns']


def parse_file_datetime(filename):
    """Extract datetime from filename format: logfile_metrics-YYYY-MM-DD-HH-MM.csv"""
    try:
        date_str = filename.stem.split('-', 1)[1]  # Get part after 'logfile_metrics-'
        return datetime.strptime(date_str, "%Y-%m-%d-%H-%M")
    except (IndexError, ValueError):
        return None


############################ METRICS FILES ############################
#######################################################################


def process_file(file_path):
    """Process a single CSV file and return metrics"""
    valid_lines = []
    has_goal2 = False
    has_goal3 = False
    final_distance = 0
    
    with open(file_path, 'r') as f:
        content = f.read()
        
        # Check for explicit goals in this file
        current_file_goals = {
            'REACHED GOAL 1': content.count('REACHED GOAL 1'),
            'REACHED GOAL 2': content.count('REACHED GOAL 2'),
            'REACHED GOAL 3': content.count('REACHED GOAL 3')
        }
        
        # Update global counts
        for goal, count in current_file_goals.items():
            goal_counts[goal] += count
        
        has_goal2 = current_file_goals['REACHED GOAL 2'] > 0
        has_goal3 = current_file_goals['REACHED GOAL 3'] > 0
        
        f.seek(0)  # Reset to process data
        for line in f:
            parts = line.strip().split(',')
            if len(parts) == 7:
                try:
                    valid_lines.append(line)
                    final_distance = float(parts[0])  # Track last distance
                except ValueError:
                    continue
    
    # Check for implicit Goal 3 in this specific file
    if has_goal2 and final_distance > 150 and current_file_goals['REACHED GOAL 3'] == 0:
        goal_counts['REACHED GOAL 3'] += 1
        has_goal3 = True

    if has_goal3:
        # Create DataFrame
        df = pd.read_csv(
            io.StringIO(''.join(valid_lines)),
            header=None,
            names=COLUMNS
        )

        return {
            'last_distance': df['travelled_distance'].iloc[-1],
            'all_velocities': df['instant_velocity'].tolist(),
            'last_instability': df['instability_index'].iloc[-1],
            'mean_torque': df['mean_torque'].mean(),
            'mean_foot_force': df['mean_foot_force'].mean(),
            'power_consumption': df['power_consumption'].mean(),
            'voltage_consumption': df['voltage_consumption'].mean()
        }
    else:
        return None
    
    if not valid_lines:
        return None
        

# Collect and sort files chronologically
all_files = []
for csv_file in Path('.').glob('../logfiles/elev/logfile_metrics-*.csv'):
    dt = parse_file_datetime(csv_file)
    if dt:  # Only process files with valid dates
        all_files.append((dt, csv_file))
    else:
        print(f"Warning: Skipped file with invalid date format - {csv_file.name}")

# Sort files by datetime
sorted_files = sorted(all_files, key=lambda x: x[0])

# Process files in chronological order
for dt, csv_file in sorted_files:
    metrics = process_file(csv_file)
    if metrics:
        file_distance_info.append({
            'datetime': dt,
            'filename': csv_file.name,
            'last_distance': metrics['last_distance'],
            'last_instability': metrics['last_instability'],
            'mean_torque': metrics['mean_torque'],
            'mean_foot_force': metrics['mean_foot_force'],
            'power_consumption': metrics['power_consumption'],
            'voltage_consumption': metrics['voltage_consumption']
        })
        # Add to aggregate lists
        last_distances.append(metrics['last_distance'])
        all_velocities.extend(metrics['all_velocities'])
        if not np.isnan(metrics['last_instability']):
            last_instabilities.append(metrics['last_instability'])
        file_torque_means.append(metrics['mean_torque'])
        mean_foot_force.append(metrics['mean_foot_force'])
        mean_power_consumption.append(metrics['power_consumption'])
        mean_voltage_consumption.append(metrics['voltage_consumption'])


############################ METRICS RAW FILES ############################
###########################################################################

def process_file_raw(file_path_raw):
    """Process a single CSV file and return metrics"""
    valid_lines = []
    has_goal2_raw = False
    final_distance_raw = 0
    
    with open(file_path_raw, 'r') as f:
        content = f.read()

        # Check for explicit goals in this file
        current_file_goals_raw = {
            'REACHED GOAL 1': content.count('REACHED GOAL 1'),
            'REACHED GOAL 2': content.count('REACHED GOAL 2'),
            'REACHED GOAL 3': content.count('REACHED GOAL 3')
        }

        has_goal2_raw = current_file_goals_raw['REACHED GOAL 2'] > 0
        
        f.seek(0)  # Reset to process data
        for line in f:
            parts = line.strip().split(',')
            if len(parts) == 5:
                try:
                    valid_lines.append(line)
                    final_distance_raw = float(parts[0])  # Track last distance
                except ValueError:
                    continue

    # Check for implicit Goal 3 in this specific file
    if not has_goal2_raw and final_distance_raw < 100:
        return None
        
    if not valid_lines:
        return None
        
    # Create DataFrame
    df = pd.read_csv(
        io.StringIO(''.join(valid_lines)),
        header=None,
        names=COLUMNS_RAW
    )
    
    return {
        'position_x': df['position_x'].tolist(),
        'position_y': df['position_y'].tolist(),
        'position_z': df['position_z'].tolist(),
        'elapsed_time_s': df['elapsed_time_s'].tolist(),
    }


# Collect and sort raw files chronologically
all_raw_files = []
for csv_raw_file in Path('.').glob('../logfiles/elev/logfile_raw-*.csv'):
    dt = parse_file_datetime(csv_raw_file)
    if dt:  # Only process files with valid dates
        all_raw_files.append((dt, csv_raw_file))
    else:
        print(f"Warning: Skipped file with invalid date format - {csv_raw_file.name}")

# Sort raw files by datetime
sorted_raw_files = sorted(all_raw_files, key=lambda x: x[0])

# Process raw files in chronological order
for dt, csv_raw_file in sorted_raw_files:
    metrics_raw = process_file_raw(csv_raw_file)
    if metrics_raw:
        file_distance_info_raw.append({
            'datetime': dt,
            'filename': csv_raw_file.name,
            'elapsed_time_s': metrics_raw['elapsed_time_s'],
            'position_x': metrics_raw['position_x'],
            'position_y': metrics_raw['position_y'],
            'position_z': metrics_raw['position_z']
        })
        # Add to aggregate lists
        elapsed_time_s.append(metrics_raw['elapsed_time_s'])
        position_x.append(metrics_raw['position_x'])
        position_y.append(metrics_raw['position_y'])
        position_z.append(metrics_raw['position_z'])

###########################################################################
###########################################################################

# Calculate statistics for each metric
def calculate_stats(data, unit=""):
    return {
        'mean': np.mean(data),
        'median': np.median(data),
        'stdev': np.std(data),
        'unit': unit
    }

stats = {
    'Travelled Distance': calculate_stats(last_distances, "m"),
    'Instant Velocity': calculate_stats(all_velocities, "m/s"),
    'Instability Index': calculate_stats(last_instabilities),
    'Mean Torque': calculate_stats(file_torque_means, "N·m"),
    'Mean Foot Force': calculate_stats(mean_foot_force, "N"),
    'Mean Power Consumption': calculate_stats(mean_power_consumption, "W"),
    'Mean Voltage Consumption': calculate_stats(mean_voltage_consumption, "V"),
    'Elapsed Time': elapsed_time_s[0][-1] - elapsed_time_s[0][0]
}

# Calculate success rates
#total_files = len(file_distance_info)
total_files = 1
#print(f"{total_files} Total Files")
#print(f"{goal_counts['REACHED GOAL 1']} Goal 1")
success_rates = {
    'Goal 1': goal_counts['REACHED GOAL 1'] / total_files,
    'Goal 2': goal_counts['REACHED GOAL 2'] / total_files,
    'Goal 3': goal_counts['REACHED GOAL 3'] / total_files,
    'Overall': sum(goal_counts.values()) / (total_files * 3)
}

# Verification printout
print("\n=== Statistical Summary ===")
for metric, values in stats.items():
    if metric == 'Elapsed Time':
        print(f"\n{metric}:")
        print(f"{values}")
    else:
        print(f"\n{metric}:")
        print(f"  Mean: {values['mean']:.3f} {values['unit']}")
        print(f"  Median: {values['median']:.3f} {values['unit']}")
        print(f"  Std Dev: {values['stdev']:.3f} {values['unit']}")

print("\n=== Goal Achievement ===")
for goal, count in goal_counts.items():
    print(f"{goal}: {count} times")
for goal, rate in success_rates.items():
    print(f"{goal} Success Rate: {rate*100:.1f}%")

# Update LaTeX table footnote
latex_table = f"""
\\documentclass{{article}}
\\usepackage{{booktabs}}
\\usepackage{{siunitx}}

\\begin{{document}}

\\begin{{table}}[htbp]
\\centering
\\caption{{Elevation Metrics Table}}
\\label{{tab:simulation-metrics}}
\\sisetup{{
    table-format=1.3,
    table-number-alignment=center,
}}
\\begin{{tabular}}{{@{{}} l S[table-format=3.2] S[table-format=3.2] S[table-format=1.3] @{{}}}}
\\toprule
\\textbf{{Metric}} & \\textbf{{Mean}} & \\textbf{{Median}} & \\textbf{{Std Dev}} \\\\
\\midrule
Travelled Distance (m) & {stats['Travelled Distance']['mean']:.2f} & {stats['Travelled Distance']['median']:.2f} & {stats['Travelled Distance']['stdev']:.2f} \\\\
Velocity (m/s) & {stats['Instant Velocity']['mean']:.3f} & {stats['Instant Velocity']['median']:.3f} & {stats['Instant Velocity']['stdev']:.3f} \\\\
Instability Index & {stats['Instability Index']['mean']:.3f} & {stats['Instability Index']['median']:.3f} & {stats['Instability Index']['stdev']:.3f} \\\\
Torque (N·m) & {stats['Mean Torque']['mean']:.2f} & {stats['Mean Torque']['median']:.2f} & {stats['Mean Torque']['stdev']:.2f} \\\\
Foot Force (N) & {stats['Mean Foot Force']['mean']:.2f} & {stats['Mean Foot Force']['median']:.2f} & {stats['Mean Foot Force']['stdev']:.2f} \\\\
Power Draw (A) & {stats['Mean Power Consumption']['mean']/1000:.2f} & {stats['Mean Power Consumption']['median']/1000:.2f} & {stats['Mean Power Consumption']['stdev']/1000:.2f} \\\\
Voltage (V) & {stats['Mean Voltage Consumption']['mean']/1000:.2f} & {stats['Mean Voltage Consumption']['median']/1000:.2f} & {stats['Mean Voltage Consumption']['stdev']/1000:.2f} \\\\
Elapsed Time(s) & {stats['Elapsed Time']:.2f} \\\\

\\bottomrule
\\end{{tabular}}
\\end{{table}}

\\end{{document}}
"""

# Goal 1 Success(\%) & {{{success_rates['Goal 1']*100:.1f}}} \\\\
# Goal 2 Success(\%) & {{{success_rates['Goal 2']*100:.1f}}} \\\\
# Goal 3 Success(\%) & {{{success_rates['Goal 3']*100:.1f}}} \\\\
# Overall Goal Success(\%) & {{{success_rates['Overall']*100:.1f}}} \\\\
# Successful Runs & {{{len(file_distance_info)}/50}} \\\\

# Save output
with open('../tables/elev/elev_table.tex', 'w') as f:
    f.write(latex_table)

print("\nLaTeX table with statistics generated successfully!")

#############################################################################################################################################
################################################################## PLOTS ####################################################################
#############################################################################################################################################

# Create 3D plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot each experiment's path
for i in range(len(position_x)):
    x = position_x[i]
    y = position_y[i]
    z = position_z[i]
    
    # Plot main path (semi-transparent)
    ax.plot(x, y, z, 
            alpha=0.2,          # Transparency
            linewidth=0.7,       # Thin lines
            color='blue')        # Consistent color

# Highlight Start/End points
for i in range(len(position_x)):
    # Starting point (green)
    ax.scatter(position_x[i][0], position_y[i][0], position_z[i][0], 
               color='lime', s=20, marker='o')
    
    # Ending point (red)
    ax.scatter(position_x[i][-1], position_y[i][-1], position_z[i][-1],     
               color='red', s=20, marker='x')

# Plot settings
ax.set_xlabel('X (m)', labelpad=12)
ax.set_ylabel('Y (m)', labelpad=12)
ax.set_zlabel('Z (m)', labelpad=12)
ax.set_title('Robot Navigation Paths in 1 Experiment', pad=20)
ax.grid(True, alpha=0.3)
ax.view_init(elev=25, azim=-120)  # Good viewing angle

# Legend
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], color='blue', lw=2, label='Robot Path'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='lime', markersize=8, label='Start Point'),
    Line2D([0], [0], marker='x', color='red', lw=0, markersize=8, label='End Point')
]
ax.legend(handles=legend_elements, loc='best')

plt.tight_layout()
plt.savefig('../tables/elev/elev_3d_paths.png', dpi=300, bbox_inches='tight')
print("3D path visualization saved to ../tables/elev/elev_3d_paths.png")


# Create 2D plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111)

# Plot each experiment's path
for i in range(len(position_x)):
    # Define the angle of rotation in radians
    theta = np.radians(-20)  # 45 degrees

    # Define the x, y values
    x = np.array(position_x[i])
    y = np.array(position_y[i])

    # Apply the rotation
    x_rotated = x * np.cos(theta) - y * np.sin(theta)
    y_rotated = x * np.sin(theta) + y * np.cos(theta)
    
    # Plot main path (semi-transparent)
    ax.plot(x_rotated, y_rotated, 
            alpha=1.0,          # Opaque
            linewidth=5.0,       # Thin lines
            color='red',
            linestyle='--')        # Consistent color

# # -19.02 -13.71
# # 9.72 -22.27
# # -10.79 -6.89
# #Goals
ax.scatter(19.02, 13.71, 
            color='blue', s=20, marker='o')
ax.scatter(-9.72, 22.27, 
            color='green', s=20, marker='o')
ax.scatter(10.79, 6.89, 
            color='lime', s=20, marker='o')

# Set background transparent
ax.set_facecolor('none')
fig.patch.set_facecolor('none')

# Remove axis ticks and frames
ax.axis('off')

# Save the plot with transparent background
plt.savefig('../tables/elev/elev2_2d_paths.png', dpi=300, bbox_inches='tight', transparent=True, pad_inches=0)
print("3D path visualization saved to ../tables/elev/elev2_2d_paths.png")


# Read the file with header
# data = pd.read_csv('../logfiles/new_metrics/elev/odom.csv')

# # Extract X and Y positions (skip first 457 rows if needed)
# position_x = data["field.pose.pose.position.x"].iloc[421:].values
# position_y = data["field.pose.pose.position.y"].iloc[421:].values

# # Create 2D plot
# fig = plt.figure(figsize=(12, 10))
# ax = fig.add_subplot(111)

# # Define the angle of rotation in radians
# theta = np.radians(-20)  # 45 degrees

# # Define the x, y values
# x = np.array(position_x)
# y = np.array(position_y)

# # Apply the rotation
# x_rotated = x * np.cos(theta) - y * np.sin(theta)
# y_rotated = x * np.sin(theta) + y * np.cos(theta)

# # Plot main path (semi-transparent)
# ax.plot(x_rotated, y_rotated, 
#         alpha=1.0,          # Opaque
#         linewidth=5.0,       # Thin lines
#         color='red',
#         linestyle='--')        # Consistent color

# # Set background transparent
# ax.set_facecolor('none')
# fig.patch.set_facecolor('none')

# # Remove axis ticks and frames
# ax.axis('off')

# # Save the plot
# plt.tight_layout()
# plt.savefig('../tables/elev/elev2_2d_paths.png', dpi=300, bbox_inches='tight', transparent=True)
# print("2D path visualization saved to ../tables/elev/elev2_2d_paths.png")


