import numpy as np
import matplotlib.pyplot as plt

def plot_signal_timing(n_intersections, cycle_length, phase_shift, green_ratio, inbound_dev, progression_speed, queue_clearance_time, time_steps=10, distance=1):
    """
    Plot the space-time diagram for n intersections.
    
    Parameters:
    - n_intersections: Number of intersections.
    - cycle_length: Signal cycle length (in seconds).
    - phase_shift: Phase shift between intersections (in seconds).
    - green_ratio: Ratio of green time within each cycle (0 < green_ratio < 1).
    - time_steps: Number of time steps to display.
    - distance: Distance between intersections (in arbitrary units).
    """
    green_start = np.zeros_like(green_ratio)
    green_end = np.zeros_like(green_ratio)
    band_start = np.zeros(n_intersections)
    band_end = np.zeros(n_intersections)

    # Calculate green time and red time
    green_time = cycle_length * green_ratio
    red_time = cycle_length - green_time

    # Determine green and red phases
    start_time = np.cumsum(phase_shift)
    green_start[0, :] = start_time
    green_start[1, :] = green_start[0, :] + inbound_dev
    green_end = green_start + green_time
    red_start = green_end
    red_end = red_start + red_time

    # Create the figure and axis
    fig, ax = plt.subplots(figsize=(10, 6))

    # Loop through each intersection
    for i in range(n_intersections):
        for j in range(time_steps):
            cycle_time = j * cycle_length

            # Draw the green phase
            gap = 10
            ax.plot([green_start[0, i] + cycle_time, green_end[0, i] + cycle_time], [distance[i] - gap, distance[i] - gap], color='g', linewidth=2)
            ax.plot([green_start[1, i] + cycle_time, green_end[1, i] + cycle_time], [distance[i] + gap, distance[i] + gap], color='g', linewidth=2)
            
            # Draw the red phase
            ax.plot([red_start[0, i] + cycle_time, red_end[0, i] + cycle_time], [distance[i] - gap, distance[i] - gap], color='r', linewidth=2)
            ax.plot([red_start[1, i] + cycle_time, red_end[1, i] + cycle_time], [distance[i] + gap, distance[i] + gap], color='r', linewidth=2)
    
        # 绘制绿波带的直线
        if i == 0:
            band_start[i] = green_start[0, i]
            band_end[i] = green_end[0, i]
        else:
            band_start[i] = band_start[i - 1] + (distance[i] - distance[i - 1]) / progression_speed[i - 1] - queue_clearance_time[i]*cycle_length
            band_end[i] = band_end[i - 1] + (distance[i] - distance[i - 1]) / progression_speed[i - 1] - queue_clearance_time[i]*cycle_length
            plt.plot([band_start[i - 1], band_start[i] + queue_clearance_time[i]*cycle_length], [distance[i - 1], distance[i]])
            plt.plot([band_end[i - 1], band_end[i] + queue_clearance_time[i]*cycle_length], [distance[i - 1], distance[i]])
    # Set labels and titles
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Intersection Position')
    ax.set_title('Space-Time Diagram for Signalized Intersections')
    ax.grid(True)

    plt.savefig(r'E:\workspace\python\BusRouteTSP\tools\result\signal plan.png')
    plt.show()

    

# Example usage
# n_intersections = 5  # Number of intersections
# cycle_length = 60    # Signal cycle length (in seconds)
# phase_shift = 10     # Phase shift between intersections (in seconds)
# plot_signal_timing(n_intersections, cycle_length, phase_shift)
