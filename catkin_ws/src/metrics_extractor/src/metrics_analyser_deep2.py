#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
import os
import numpy as np
import glob
from collections import defaultdict

# Configure plot settings
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 12,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'savefig.dpi': 300,
    'pdf.fonttype': 42,
    'axes.grid': True,
    'grid.alpha': 0.3
})

def save_individual_plots(time, metrics, avg_events, output_dir):
    """Save each metric as a separate PDF file with event markers"""
    metric_info = [
        ('distance', 'Travelled Distance', 'blue'),
        ('velocity', 'Velocity', 'green'),
        ('instability', 'Instability Index', 'red'),
        ('torque', 'Mean Torque', 'purple'),
    ]

    for metric, title, color in metric_info:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(time, metrics[metric], color=color, linewidth=1.5)
        ax.set_title(title)
        ax.set_xlabel('Time (seconds)')
        
        # Add event markers
        for event_type, t, goal_num in avg_events:
            if t < time[-1]:  # Only plot if within time range
                label = f"Goal {goal_num} Reached" if event_type == 'goal_reached' else "Mission Aborted"
                line_color = 'green' if event_type == 'goal_reached' else 'red'
                ax.axvline(x=t, color=line_color, linestyle='--', alpha=0.7)
                if metric == 'distance':
                    ax.text(t, ax.get_ylim()[1]*0.95, label,
                           color=line_color, ha='right', va='top', rotation=90)

        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"{metric}.pdf"), bbox_inches='tight')
        plt.close()

def main():
    # Directory containing your CSV files (change this to your actual directory)
    input_dir = '../logfiles/trav/'  # Replace with your directory path
    file_pattern = 'logfile_metrics-''*'''
    
    # Find all matching files
    csv_files = glob.glob(os.path.join(input_dir, file_pattern))
    
    if not csv_files:
        print(f"No files found matching pattern: {os.path.join(input_dir, file_pattern)}")
        return
    
    print(f"Found {len(csv_files)} files to process")
    
    # Output directory
    output_dir = '../plots/trav'
    
    # Create output directory if needed
    os.makedirs(output_dir, exist_ok=True)

 # Initialize data storage
    all_metrics = []
    goal_events = defaultdict(list)  # To store goal events by goal number
    abort_events = []
    max_length = 0

    # Process each CSV file
    for csv_file in csv_files:
        try:
            metrics = {
                'distance': [], 'velocity': [], 'instability': [],
                'torque': []
            }
            events = []

            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                line_count = 0
                
                for row in reader:
                    if len(row) == 4:
                        try:
                            metrics['distance'].append(float(row[0]))
                            metrics['velocity'].append(float(row[1]))
                            metrics['instability'].append(float(row[2]))
                            metrics['torque'].append(float(row[3]))
                            line_count += 1
                        except ValueError:
                            continue
                    elif len(row) == 1:
                        event_text = row[0].strip()
                        if "REACHED GOAL 1" in event_text:
                            goal_number = int(event_text.split()[-1])
                            # Store both position and time (assuming 5Hz sampling)
                            events.append(('goal_reached_1', line_count-1, goal_number))
                        elif "REACHED GOAL 2" in event_text:
                            goal_number = int(event_text.split()[-1])
                            # Store both position and time (assuming 5Hz sampling)
                            events.append(('goal_reached_2', line_count-1, goal_number))
                        elif "REACHED GOAL 3" in event_text:
                            goal_number = int(event_text.split()[-1])
                            # Store both position and time (assuming 5Hz sampling)
                            events.append(('goal_reached_3', line_count-1, goal_number))
                        elif "ABORTED" in event_text:
                            events.append(('aborted', line_count-1))

            # Track maximum length
            current_length = len(metrics['distance'])
            if current_length > max_length:
                max_length = current_length
                
            all_metrics.append(metrics)
            
            # Collect all goal events by goal number
            for event in events:
                if event[0] == 'goal_reached':
                    goal_events[event[2]].append(event[1])  # Store position for this goal number
                elif event[0] == 'aborted':
                    abort_events.append(event[1])
            
        except Exception as e:
            print(f"Warning: Error processing file {csv_file}: {str(e)}")
            continue

    if not all_metrics:
        print("No valid data found in any files")
        return

    # Calculate average metrics
    avg_metrics = {
        'distance': np.zeros(max_length),
        'velocity': np.zeros(max_length),
        'instability': np.zeros(max_length),
        'torque': np.zeros(max_length)
    }
    
    counts = np.zeros(max_length)
    
    for metrics in all_metrics:
        length = len(metrics['distance'])
        counts[:length] += 1
        for key in avg_metrics:
            avg_metrics[key][:length] += np.array(metrics[key])
    
    for key in avg_metrics:
        avg_metrics[key] = np.divide(avg_metrics[key], counts, 
                                   out=np.zeros_like(avg_metrics[key]), 
                                   where=counts!=0).tolist()

    # Calculate average event times
    avg_events = []
    
    # Process goal events
    for goal_num, positions in goal_events.items():
        if positions:  # Only if we have some events for this goal
            avg_pos = np.mean(positions)
            avg_time = avg_pos * 0.2  # Convert position to time (5Hz sampling)
            avg_events.append(('goal_reached', avg_time, goal_num))
    
    # Process abort events if needed
    if abort_events:
        avg_abort_pos = np.mean(abort_events)
        avg_abort_time = avg_abort_pos * 0.2
        avg_events.append(('aborted', avg_abort_time, 0))  # 0 as placeholder for goal number

    # Sort events by time
    avg_events.sort(key=lambda x: x[1])

    # Generate time axis (5Hz sampling)
    time = [i * 0.2 for i in range(max_length)]

    # Generate and save plots
    save_individual_plots(time, avg_metrics, avg_events, output_dir)

    print(f"Saved PDF plots to {output_dir} directory")

if __name__ == '__main__':
    main()