#!/usr/bin/env python3
"""
Rotation Test Results Analysis Script
- Loads CSV results from rotation test automation
- Groups measurements by speed and duration
- Calculates averages for each set of three measurements
- Performs linear regression for each speed (duration vs rotation)
- Prints best fit line parameters and plots results
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import glob
import os
from datetime import datetime
from pathlib import Path

class RotationAnalyzer:
    def __init__(self):
        self.results_df = None
        self.grouped_data = {}
        self.fit_results = {}
        # Default directories relative to this script
        self.base_dir = Path(__file__).resolve().parent
        self.results_dir = self.base_dir / "results"
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
    def load_latest_results(self, pattern="rotation_test_results_*.csv"):
        """Load the most recent results CSV file"""
        search_glob = str(self.results_dir / pattern)
        csv_files = glob.glob(search_glob)
        
        if not csv_files:
            print(f"No CSV files found matching pattern: {search_glob}")
            return False
            
        # Get the most recent file
        latest_file = max(csv_files, key=os.path.getctime)
        print(f"Loading results from: {latest_file}")
        
        try:
            self.results_df = pd.read_csv(latest_file)
            print(f"Loaded {len(self.results_df)} test results")
            return True
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            return False
            
    def load_results(self, filename):
        """Load results from a specific CSV file"""
        try:
            self.results_df = pd.read_csv(filename)
            print(f"Loaded {len(self.results_df)} test results from {filename}")
            return True
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            return False
            
    def parse_command_parameters(self):
        """Parse command strings to extract power and duration"""
        if self.results_df is None:
            print("No data loaded")
            return False
            
        # Parse commands like "T 0.15 1000" to extract power and duration
        powers = []
        durations = []
        
        for cmd in self.results_df['command']:
            parts = cmd.split()
            if len(parts) >= 3 and parts[0] == 'T':
                try:
                    power = float(parts[1])
                    duration = int(parts[2])
                    powers.append(power)
                    durations.append(duration)
                except ValueError:
                    powers.append(None)
                    durations.append(None)
            else:
                powers.append(None)
                durations.append(None)
                
        self.results_df['power'] = powers
        self.results_df['duration_parsed'] = durations
        
        # Filter out any rows that couldn't be parsed
        self.results_df = self.results_df.dropna(subset=['power', 'duration_parsed'])
        print(f"Successfully parsed {len(self.results_df)} rotation commands")
        return True
        
    def group_and_average_results(self):
        """Group results by power and duration, calculate averages"""
        if self.results_df is None:
            print("No data loaded")
            return False
            
        # Group by power and duration
        grouped = self.results_df.groupby(['power', 'duration_parsed'])
        
        self.grouped_data = {}
        
        print("\n" + "="*60)
        print("GROUPED RESULTS (Averages of 3 measurements)")
        print("="*60)
        
        for (power, duration), group in grouped:
            rotations = group['total_rotation'].values
            
            avg_rotation = np.mean(rotations)
            std_rotation = np.std(rotations)
            count = len(rotations)
            
            if power not in self.grouped_data:
                self.grouped_data[power] = {'durations': [], 'rotations': [], 'std_devs': [], 'counts': []}
                
            self.grouped_data[power]['durations'].append(duration)
            self.grouped_data[power]['rotations'].append(avg_rotation)
            self.grouped_data[power]['std_devs'].append(std_rotation)
            self.grouped_data[power]['counts'].append(count)
            
            print(f"Power {power:4.2f}, Duration {duration:4d}ms: Avg = {avg_rotation:7.1f}° ± {std_rotation:4.1f}° (n={count}, {avg_rotation/360:+.2f} turns)")
            
        return True
        
    def calculate_best_fit_lines(self):
        """Calculate linear regression for each power level (constrained through origin)"""
        if not self.grouped_data:
            print("No grouped data available")
            return False
            
        self.fit_results = {}
        
        print("\n" + "="*60)
        print("LINEAR REGRESSION RESULTS (Constrained through origin)")
        print("="*60)
        
        for power in sorted(self.grouped_data.keys()):
            data = self.grouped_data[power]
            durations = np.array(data['durations'])
            rotations = np.array(data['rotations'])
            
            # Perform linear regression through origin
            # For y = slope * x, minimize sum of (y_i - slope * x_i)²
            # Solution: slope = Σ(x_i * y_i) / Σ(x_i²)
            slope = np.sum(durations * rotations) / np.sum(durations**2)
            intercept = 0.0  # Constrained to pass through origin
            
            # Calculate R² for regression through origin
            y_mean = np.mean(rotations)
            ss_tot = np.sum((rotations - y_mean)**2)
            ss_res = np.sum((rotations - slope * durations)**2)
            r_value = np.sqrt(1 - (ss_res / ss_tot)) if ss_tot != 0 else 0
            r_squared = r_value**2
            
            # Calculate standard error
            residuals = rotations - slope * durations
            std_err = np.sqrt(np.sum(residuals**2) / (len(durations) - 1)) if len(durations) > 1 else 0
            
            # Store results
            self.fit_results[power] = {
                'slope': slope,
                'intercept': intercept,
                'r_value': r_value,
                'r_squared': r_squared,
                'p_value': 0,  # Not calculated for constrained regression
                'std_err': std_err,
                'durations': durations,
                'rotations': rotations,
                'std_devs': data['std_devs']
            }
            
            # Print results
            print(f"\nPower = {power:.2f}:")
            print(f"  Best fit line (through origin): rotation = {slope:.4f} * duration")
            print(f"  R² = {r_squared:.4f} (correlation = {r_value:.4f})")
            print(f"  Standard error = {std_err:.4f}")
            
            # Calculate degrees per second
            degrees_per_ms = slope
            degrees_per_sec = degrees_per_ms * 1000
            print(f"  Rate: {degrees_per_sec:.2f} degrees/second")
            
            # Calculate rotation rate in revolutions per minute (RPM)
            rpm = (degrees_per_sec / 360) * 60
            print(f"  Rate: {rpm:.2f} RPM")
            
        return True
        
    def plot_results(self, save_plot=True):
        """Create plots showing data points and best fit lines"""
        if not self.fit_results:
            print("No fit results available")
            return False
            
        # Create subplots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        axes = [ax1, ax2, ax3, ax4]
        powers = sorted(self.fit_results.keys())
        
        colors = ['blue', 'red', 'green', 'orange']
        
        for i, power in enumerate(powers):
            if i >= len(axes):
                break
                
            ax = axes[i]
            data = self.fit_results[power]
            
            durations = data['durations']
            rotations = data['rotations']
            std_devs = data['std_devs']
            slope = data['slope']
            intercept = data['intercept']
            r_squared = data['r_squared']
            
            # Plot data points with error bars
            ax.errorbar(durations, rotations, yerr=std_devs, 
                       fmt='o', color=colors[i], capsize=5, capthick=2, 
                       label=f'Measured data (n=3 each)')
            
            # Plot best fit line (through origin)
            x_fit = np.linspace(0, max(durations), 100)  # Start from 0
            y_fit = slope * x_fit  # No intercept term
            ax.plot(x_fit, y_fit, '--', color=colors[i], linewidth=2,
                   label=f'Best fit (origin): y = {slope:.4f}x')
            
            # Add horizontal lines for full rotations
            for n_rotations in range(-2, 3):
                ax.axhline(y=n_rotations*360, color='gray', linestyle=':', alpha=0.5)
                if n_rotations != 0:
                    ax.text(max(durations)*0.02, n_rotations*360 + 20, f'{n_rotations} turn{"s" if abs(n_rotations) != 1 else ""}', 
                           fontsize=8, alpha=0.7)
            
            ax.set_xlabel('Duration (ms)')
            ax.set_ylabel('Rotation (degrees)')
            ax.set_title(f'Power = {power:.2f}\\nR² = {r_squared:.4f} (Through Origin)')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"rotation_analysis_{timestamp}.png"
            out_path = self.results_dir / filename
            plt.savefig(out_path, dpi=300, bbox_inches='tight')
            print(f"\nPlot saved as: {out_path}")
            
        plt.show()
        return True
        
    def plot_velocity_vs_power(self, save_plot=True):
        """Create a plot showing rotation velocity (degrees/sec) vs power setting"""
        if not self.fit_results:
            print("No fit results available")
            return False
            
        # Extract power settings and corresponding velocities
        powers = []
        velocities = []
        
        for power in sorted(self.fit_results.keys()):
            data = self.fit_results[power]
            slope = data['slope']  # degrees per millisecond
            velocity = slope * 1000  # convert to degrees per second
            
            powers.append(power)
            velocities.append(velocity)
            
        # Create the plot
        plt.figure(figsize=(10, 6))
        plt.plot(powers, velocities, 'bo-', linewidth=2, markersize=8, label='Measured velocity')
        
        # Add a linear fit to the velocity vs power relationship (constrained through origin)
        powers_array = np.array(powers)
        velocities_array = np.array(velocities)
        
        # Linear regression through origin: y = slope * x
        # For y = slope * x, minimize sum of (y_i - slope * x_i)²
        # Solution: slope = Σ(x_i * y_i) / Σ(x_i²)
        slope_vel = np.sum(powers_array * velocities_array) / np.sum(powers_array**2)
        intercept_vel = 0.0  # Constrained to pass through origin
        
        # Calculate R² for regression through origin
        y_mean = np.mean(velocities_array)
        ss_tot = np.sum((velocities_array - y_mean)**2)
        ss_res = np.sum((velocities_array - slope_vel * powers_array)**2)
        r_value_vel = np.sqrt(1 - (ss_res / ss_tot)) if ss_tot != 0 else 0
        
        # Plot the fitted line
        x_fit = np.linspace(0, max(powers), 100)  # Start from 0 to show origin
        y_fit = slope_vel * x_fit  # No intercept term
        plt.plot(x_fit, y_fit, 'r--', linewidth=2, 
                label=f'Linear fit (origin): y = {slope_vel:.1f}x\\nR² = {r_value_vel**2:.4f}')
        
        # Formatting
        plt.xlabel('Power Setting', fontsize=12)
        plt.ylabel('Rotation Velocity (degrees/second)', fontsize=12)
        plt.title('Rotation Velocity vs Power Setting', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=11)
        
        # Add data point labels
        for i, (power, velocity) in enumerate(zip(powers, velocities)):
            plt.annotate(f'{velocity:.1f}°/s', 
                        (power, velocity), 
                        textcoords="offset points", 
                        xytext=(0,10), 
                        ha='center',
                        fontsize=9,
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
        
        plt.tight_layout()
        
        if save_plot:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"rotation_velocity_vs_power_{timestamp}.png"
            out_path = self.results_dir / filename
            plt.savefig(out_path, dpi=300, bbox_inches='tight')
            print(f"\nVelocity plot saved as: {out_path}")
            
        plt.show()
        
        # Print the velocity data
        print("\\n" + "="*50)
        print("ROTATION VELOCITY vs POWER SETTING")
        print("="*50)
        print(f"{'Power':<8} {'Velocity (°/s)':<15} {'RPM':<8}")
        print("-" * 50)
        
        for power, velocity in zip(powers, velocities):
            rpm = (velocity / 360) * 60
            print(f"{power:<8.2f} {velocity:<15.2f} {rpm:<8.2f}")
            
        print(f"\\nLinear relationship (through origin): Velocity = {slope_vel:.1f} × Power")
        print(f"Correlation coefficient: R = {r_value_vel:.4f}")
        print(f"R-squared: {r_value_vel**2:.4f}")
        
        return True
        
    def print_summary_table(self):
        """Print a summary table of all results"""
        if not self.fit_results:
            print("No fit results available")
            return False
            
        print("\n" + "="*80)
        print("SUMMARY TABLE (Constrained through origin)")
        print("="*80)
        print(f"{'Power':<8} {'Slope':<12} {'R²':<10} {'Deg/sec':<10} {'RPM':<8}")
        print("-" * 80)
        
        for power in sorted(self.fit_results.keys()):
            data = self.fit_results[power]
            slope = data['slope']
            r_squared = data['r_squared']
            deg_per_sec = slope * 1000
            rpm = (deg_per_sec / 360) * 60
            
            print(f"{power:<8.2f} {slope:<12.6f} {r_squared:<10.4f} {deg_per_sec:<10.2f} {rpm:<8.2f}")
            
        return True
        
    def analyze_all(self, filename=None):
        """Run complete analysis pipeline"""
        print("Rotation Test Results Analysis")
        print("=" * 40)
        
        # Load data
        if filename:
            if not self.load_results(filename):
                return False
        else:
            if not self.load_latest_results():
                return False
                
        # Parse and analyze
        if not self.parse_command_parameters():
            return False
            
        if not self.group_and_average_results():
            return False
            
        if not self.calculate_best_fit_lines():
            return False
            
        self.print_summary_table()
        
        # Plot results
        try:
            self.plot_results()
            self.plot_velocity_vs_power()
        except Exception as e:
            print(f"Plotting failed (but analysis is complete): {e}")
            
        return True

def main():
    analyzer = RotationAnalyzer()
    
    # Check command line arguments for specific file
    import sys
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        analyzer.analyze_all(filename)
    else:
        analyzer.analyze_all()

if __name__ == "__main__":
    main()
