#!/usr/bin/env python3
"""
ESP32 Liberty Pad Serial Data Plotter
Reads sensor data from ESP32 serial output and plots it in real-time.

Expected data format from ESP32:
dist:<value>,velo:<value>,acc:<value>,jerk:<value>

Usage:
python serial_plotter.py [--port /dev/cu.usbmodem1101] [--baud 115200] [--save data.csv]
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import argparse
import csv
import re
import threading
from datetime import datetime

class SerialPlotter:
    def __init__(self, port='/dev/cu.usbmodem1101', baudrate=115200, max_points=10000, save_file=None):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points  # 10,000 points for 10 seconds at 1ms per point
        self.save_file = save_file
        
        # Data storage
        self.times = deque(maxlen=max_points)
        self.distances = deque(maxlen=max_points)
        self.velocities = deque(maxlen=max_points)
        self.accelerations = deque(maxlen=max_points)
        self.jerks = deque(maxlen=max_points)
        
        # Serial connection
        self.serial_conn = None
        self.is_connected = False
        self.start_time = time.time()
        
        # CSV file for data logging
        self.csv_file = None
        self.csv_writer = None
        
        # Thread control
        self.running = True
        
        # Pattern to match the data format
        self.data_pattern = re.compile(r'dist:(-?\d+),velo:(-?\d+),acc:(-?\d+),jerk:(-?\d+)')
        
    def connect_serial(self):
        """Establish serial connection"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Allow time for connection to stabilize
            self.is_connected = True
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
    
    def setup_csv_logging(self):
        """Setup CSV file for data logging"""
        if self.save_file:
            try:
                self.csv_file = open(self.save_file, 'w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                # Write header
                self.csv_writer.writerow(['timestamp', 'time_ms', 'distance', 'velocity', 'acceleration', 'jerk'])
                print(f"Data will be saved to {self.save_file}")
            except Exception as e:
                print(f"Failed to setup CSV logging: {e}")
                self.save_file = None
    
    def parse_serial_line(self, line):
        """Parse a line of serial data"""
        line = line.strip()
        match = self.data_pattern.match(line)
        
        if match:
            distance = int(match.group(1))
            velocity = int(match.group(2))
            acceleration = int(match.group(3))
            jerk = int(match.group(4))
            
            return distance, velocity, acceleration, jerk
        return None
    
    def read_serial_data(self):
        """Background thread to read serial data"""
        while self.running and self.is_connected:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore')
                    data = self.parse_serial_line(line)
                    
                    if data:
                        current_time = (time.time() - self.start_time) * 1000  # Convert to milliseconds
                        distance, velocity, acceleration, jerk = data
                        
                        # Add data to deques
                        self.times.append(current_time)
                        self.distances.append(distance)
                        self.velocities.append(velocity)
                        self.accelerations.append(acceleration)
                        self.jerks.append(jerk)
                        
                        # Log to CSV if enabled
                        if self.csv_writer:
                            self.csv_writer.writerow([
                                datetime.now().isoformat(),
                                int(current_time),
                                distance, velocity, acceleration, jerk
                            ])
                            # Flush every 100 samples to improve performance
                            if len(self.times) % 100 == 0:
                                self.csv_file.flush()
                
                # Minimal delay for faster response
                time.sleep(0.0001)  # 0.1ms delay
                
            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                self.is_connected = False
                break
            except Exception as e:
                print(f"Unexpected error in serial read: {e}")
    
    def animate(self, frame):
        """Animation function for matplotlib"""
        if not self.times:
            return []
        
        # Clear the plot
        self.ax.clear()
        
        # Convert deques to lists for plotting
        times_list = list(self.times)
        
        # Plot all parameters on the same graph
        if self.distances:
            self.ax.plot(times_list, list(self.distances), 'b-', label='Distance (8-bit)', linewidth=2, alpha=0.8)
        
        if self.velocities:
            self.ax.plot(times_list, list(self.velocities), 'g-', label='Velocity', linewidth=2, alpha=0.8)
        
        if self.accelerations:
            self.ax.plot(times_list, list(self.accelerations), 'r-', label='Acceleration', linewidth=2, alpha=0.8)
        
        if self.jerks:
            self.ax.plot(times_list, list(self.jerks), 'm-', label='Jerk', linewidth=2, alpha=0.8)
        
        # Set labels and title
        self.ax.set_xlabel('Time (milliseconds)')
        self.ax.set_ylabel('Sensor Values')
        self.ax.set_title('ESP32 Liberty Pad - Key Sensor Data (Real-time)', fontsize=14)
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right')
        
        # Set x-axis limits - show last 10 seconds (10,000ms)
        if times_list:
            x_min = max(0, times_list[-1] - 10000)  # Show last 10 seconds
            x_max = times_list[-1] + 100
            self.ax.set_xlim(x_min, x_max)
        
        return []
    
    def start_plotting(self):
        """Start the real-time plotting"""
        if not self.connect_serial():
            return
        
        self.setup_csv_logging()
        
        # Setup the plot - single plot instead of subplots
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 8))
        self.fig.suptitle('ESP32 Liberty Pad - All Sensor Data', fontsize=16)
        
        # Start serial reading thread
        serial_thread = threading.Thread(target=self.read_serial_data)
        serial_thread.daemon = True
        serial_thread.start()
        
        # Create animation with faster update rate
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=20, blit=False, cache_frame_data=False
        )
        
        plt.tight_layout()
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopping plotter...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Serial connection closed")
        
        if self.csv_file:
            self.csv_file.close()
            print(f"Data saved to {self.save_file}")

def main():
    parser = argparse.ArgumentParser(description='Plot ESP32 Liberty Pad sensor data in real-time')
    parser.add_argument('--port', default='/dev/cu.usbmodem1101', 
                       help='Serial port (default: /dev/cu.usbmodem1101)')
    parser.add_argument('--baud', type=int, default=115200, 
                       help='Baud rate (default: 115200)')
    parser.add_argument('--save', help='Save data to CSV file')
    parser.add_argument('--points', type=int, default=10000,
                       help='Maximum number of points to display (default: 10000 for 10 seconds at 1ms/point)')
    
    args = parser.parse_args()
    
    plotter = SerialPlotter(
        port=args.port,
        baudrate=args.baud,
        max_points=args.points,
        save_file=args.save
    )
    
    print(f"Starting Liberty Pad data plotter...")
    print(f"Port: {args.port}")
    print(f"Baud rate: {args.baud}")
    print(f"Max points: {args.points}")
    if args.save:
        print(f"Saving data to: {args.save}")
    print("\nPress Ctrl+C to stop")
    
    plotter.start_plotting()

if __name__ == "__main__":
    main()