import pandas as pd
import numpy as np
from pathlib import Path
import io
from datetime import datetime

# Initialize storage lists
file_distance_info = []
all_velocities = []
last_distances = []
mean_instabilities = []
file_torque_means = []
file_foot_force_means = []
file_power_consumption_means = []
file_voltage_consumption_means = []
goal_counts = {'REACHED GOAL 1': 0, 'REACHED GOAL 2': 0, 'REACHED GOAL 3': 0}

# Define column order
COLUMNS = ['travelled_distance', 'instant_velocity', 
          'instability_index', 'mean_torque', 'mean_foot_force', 'power_consumption', 'voltage_consumption']

def process_file(file_path):
    """Process a single CSV file and return metrics"""
    valid_lines = []
    has_goal2 = False
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
    # if has_goal2 and final_distance > 1000 and current_file_goals['REACHED GOAL 3'] == 0:
    #     goal_counts['REACHED GOAL 3'] += 1
    
    if not valid_lines:
        return None
        
    # Create DataFrame
    df = pd.read_csv(
        io.StringIO(''.join(valid_lines)),
        header=None,
        names=COLUMNS
    )
    
    return {
        'last_distance': df['travelled_distance'].iloc[-1],
        'all_velocities': df['instant_velocity'].tolist(),
        'mean_instability': df['instability_index'].mean(),
        'mean_torque': df['mean_torque'].mean(),
        'mean_foot_force': df['mean_foot_force'].mean(),
        'mean_power_consumption': df['power_consumption'].mean(),
        'mean_voltage_consumption': df['voltage_consumption'].mean()
    }

def parse_file_datetime(filename):
    """Extract datetime from filename format: logfile_metrics-YYYY-MM-DD-HH-MM.csv"""
    try:
        date_str = filename.stem.split('-', 1)[1]  # Get part after 'logfile_metrics-'
        return datetime.strptime(date_str, "%Y-%m-%d-%H-%M")
    except (IndexError, ValueError):
        return None

# Collect and sort files chronologically
all_files = []
for csv_file in Path('.').glob('../logfiles/trav/logfile_metrics-*.csv'):
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
            'mean_instability': metrics['mean_instability'],
            'mean_torque': metrics['mean_torque'],
            'mean_foot_force': metrics['mean_foot_force'],
            'mean_power_consumption': metrics['mean_power_consumption'],
            'mean_voltage_consumption': metrics['mean_voltage_consumption']
        })
        # Add to aggregate lists
        last_distances.append(metrics['last_distance'])
        all_velocities.extend(metrics['all_velocities'])
        if not np.isnan(metrics['mean_instability']):
            mean_instabilities.append(metrics['mean_instability'])
        file_torque_means.append(metrics['mean_torque'])
        file_foot_force_means.append(metrics['mean_foot_force'])
        file_power_consumption_means.append(metrics['mean_power_consumption'])
        file_voltage_consumption_means.append(metrics['mean_voltage_consumption'])

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
    'Instability Index': calculate_stats(mean_instabilities),
    'Mean Torque': calculate_stats(file_torque_means, "N·m"),
    'Mean Foot Force': calculate_stats(file_foot_force_means, "N"),
    'Mean Power': calculate_stats(file_power_consumption_means, "W"),
    'Mean Voltage': calculate_stats(file_voltage_consumption_means, "A"),
}

# Calculate success rates
total_files = len(file_distance_info)
success_rates = {
    'Goal 1': goal_counts['REACHED GOAL 1'] / total_files,
    'Goal 2': goal_counts['REACHED GOAL 2'] / total_files,
    'Goal 3': goal_counts['REACHED GOAL 3'] / total_files,
    'Overall': sum(goal_counts.values()) / (total_files * 3)
}

# Verification printout
print("\n=== Statistical Summary ===")
for metric, values in stats.items():
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
\\caption{{Traversability Metrics Table}}
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
Power (W) & {stats['Mean Power']['mean']:.2f} & {stats['Mean Power']['median']:.2f} & {stats['Mean Power']['stdev']:.2f} \\\\
Voltage (A) & {stats['Mean Voltage']['mean']:.2f} & {stats['Mean Voltage']['median']:.2f} & {stats['Mean Voltage']['stdev']:.2f} \\\\
Goal 1 Success & \\multicolumn{{3}}{{r}}{{{success_rates['Goal 1']*100:.1f}\\%}} \\\\
Goal 2 Success & \\multicolumn{{3}}{{r}}{{{success_rates['Goal 2']*100:.1f}\\%}} \\\\
Goal 3 Success & \\multicolumn{{3}}{{r}}{{{success_rates['Goal 3']*100:.1f}\\%}} \\\\
Overall Success & \\multicolumn{{3}}{{r}}{{{success_rates['Overall']*100:.1f}\\%}} \\\\
\\bottomrule
\\end{{tabular}}
\\end{{table}}

\\end{{document}}
"""

# Save output
with open('../tables/trav/trav_table.tex', 'w') as f:
    f.write(latex_table)

print("\nLaTeX table with statistics generated successfully!")