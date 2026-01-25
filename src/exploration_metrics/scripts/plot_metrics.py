#!/usr/bin/env python3
"""
Exploration Metrics Visualization Script

Usage:
    # Plot from CSV file
    ros2 run exploration_metrics plot_metrics.py --file /tmp/exploration_metrics.csv

    # Live plot from ROS topic
    ros2 run exploration_metrics plot_metrics.py --live

    # Compare multiple experiments
    ros2 run exploration_metrics plot_metrics.py --files exp1.csv exp2.csv --labels "Run1" "Run2"

    # Save plot to file
    ros2 run exploration_metrics plot_metrics.py --file metrics.csv --output plot.png
"""

import argparse
import sys
import os

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import numpy as np

# Try to import ROS2 for live mode
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class MetricsPlotter:
    def __init__(self):
        self.fig = None
        self.axes = None

    def plot_from_csv(self, csv_files, labels=None, output_file=None):
        """Plot exploration metrics from one or more CSV files."""
        if labels is None:
            labels = [os.path.basename(f) for f in csv_files]

        # Read CSV files
        dataframes = []
        for csv_file in csv_files:
            try:
                df = pd.read_csv(csv_file)
                dataframes.append(df)
                print(f"Loaded: {csv_file} ({len(df)} rows)")
            except Exception as e:
                print(f"Error loading {csv_file}: {e}")
                continue

        if not dataframes:
            print("No data to plot")
            return

        # Create figure with subplots
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('Exploration Metrics Analysis', fontsize=14, fontweight='bold')

        # Plot 1: Exploration Percentage vs Time
        ax1 = self.axes[0, 0]
        for df, label in zip(dataframes, labels):
            ax1.plot(df['elapsed_time'], df['exploration_pct'], label=label, linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Exploration (%)')
        ax1.set_title('Exploration Progress')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(0, 105)

        # Plot 2: Exploration Rate vs Time
        ax2 = self.axes[0, 1]
        for df, label in zip(dataframes, labels):
            if 'avg_exploration_rate' in df.columns:
                ax2.plot(df['elapsed_time'], df['avg_exploration_rate'], label=f'{label} (avg)', linewidth=2)
            if 'exploration_rate' in df.columns:
                ax2.plot(df['elapsed_time'], df['exploration_rate'], label=f'{label} (inst)',
                        linewidth=1, alpha=0.5)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Rate (%/s)')
        ax2.set_title('Exploration Rate')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot 3: 2D vs 3D Comparison
        ax3 = self.axes[1, 0]
        for df, label in zip(dataframes, labels):
            if 'og_pct' in df.columns:
                ax3.plot(df['elapsed_time'], df['og_pct'], label=f'{label} (2D)', linewidth=2)
            if 'octomap_pct' in df.columns:
                ax3.plot(df['elapsed_time'], df['octomap_pct'], label=f'{label} (3D)',
                        linewidth=2, linestyle='--')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Exploration (%)')
        ax3.set_title('2D vs 3D Exploration')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim(0, 105)

        # Plot 4: Space Ratios
        ax4 = self.axes[1, 1]
        if len(dataframes) == 1:
            df = dataframes[0]
            if all(col in df.columns for col in ['free_ratio', 'occupied_ratio', 'unknown_ratio']):
                ax4.stackplot(df['elapsed_time'],
                             df['free_ratio'] * 100,
                             df['occupied_ratio'] * 100,
                             df['unknown_ratio'] * 100,
                             labels=['Free', 'Occupied', 'Unknown'],
                             colors=['#90EE90', '#FF6B6B', '#B0B0B0'],
                             alpha=0.8)
                ax4.set_xlabel('Time (s)')
                ax4.set_ylabel('Ratio (%)')
                ax4.set_title('Space Classification')
                ax4.legend(loc='upper right')
                ax4.set_ylim(0, 100)
        else:
            # For multiple files, show IoU comparison
            for df, label in zip(dataframes, labels):
                if 'iou_free' in df.columns:
                    ax4.plot(df['elapsed_time'], df['iou_free'], label=f'{label} (Free IoU)', linewidth=2)
                if 'iou_occupied' in df.columns:
                    ax4.plot(df['elapsed_time'], df['iou_occupied'], label=f'{label} (Occ IoU)',
                            linewidth=2, linestyle='--')
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('IoU')
            ax4.set_title('Intersection over Union')
            ax4.legend()
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save or show
        if output_file:
            plt.savefig(output_file, dpi=150, bbox_inches='tight')
            print(f"Plot saved to: {output_file}")
        else:
            plt.show()

    def print_summary(self, csv_file):
        """Print summary statistics from a CSV file."""
        try:
            df = pd.read_csv(csv_file)
        except Exception as e:
            print(f"Error loading {csv_file}: {e}")
            return

        print("\n" + "=" * 60)
        print("EXPLORATION METRICS SUMMARY")
        print("=" * 60)

        # Final values
        final = df.iloc[-1]
        print(f"\nFinal Exploration: {final['exploration_pct']:.1f}%")

        if 'og_pct' in df.columns:
            print(f"  2D (OccupancyGrid): {final['og_pct']:.1f}%")
        if 'octomap_pct' in df.columns:
            print(f"  3D (OctoMap): {final['octomap_pct']:.1f}%")

        print(f"\nTotal Time: {final['elapsed_time']:.1f} seconds")

        # Rate statistics
        if 'avg_exploration_rate' in df.columns:
            avg_rate = df['avg_exploration_rate'].mean()
            max_rate = df['avg_exploration_rate'].max()
            print(f"\nExploration Rate:")
            print(f"  Average: {avg_rate:.3f} %/s")
            print(f"  Maximum: {max_rate:.3f} %/s")

        # Cell/Voxel statistics
        if 'gt_known_cells' in df.columns and final['gt_known_cells'] > 0:
            print(f"\n2D Map Statistics:")
            print(f"  Ground Truth Cells: {final['gt_known_cells']}")
            print(f"  Matched Cells: {final['matched_cells']}")

        if 'gt_known_voxels' in df.columns and final['gt_known_voxels'] > 0:
            print(f"\n3D Map Statistics:")
            print(f"  Ground Truth Voxels: {final['gt_known_voxels']}")
            print(f"  Matched Voxels: {final['matched_voxels']}")

        # IoU
        if 'iou_free' in df.columns:
            print(f"\nFinal IoU Scores:")
            print(f"  Free Space IoU: {final['iou_free']:.3f}")
            print(f"  Occupied Space IoU: {final['iou_occupied']:.3f}")

        print("\n" + "=" * 60)


class LivePlotter:
    """Live plotting from ROS2 topic."""

    def __init__(self, topic='/exploration/metrics'):
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 not available for live plotting")

        self.topic = topic
        self.times = []
        self.percentages = []
        self.rates = []
        self.og_pcts = []
        self.octomap_pcts = []

        # Initialize ROS2
        rclpy.init()
        self.node = rclpy.create_node('metrics_plotter')

        # Import message type
        from exploration_metrics.msg import ExplorationMetrics
        self.node.create_subscription(
            ExplorationMetrics,
            topic,
            self.callback,
            10
        )

        # Setup plot
        self.fig, self.axes = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle('Live Exploration Metrics', fontsize=14, fontweight='bold')

        # Animation
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=500, blit=False
        )

    def callback(self, msg):
        """ROS2 callback for metrics messages."""
        self.times.append(msg.elapsed_time)
        self.percentages.append(msg.exploration_percentage)
        self.rates.append(msg.avg_exploration_rate)
        self.og_pcts.append(msg.occupancy_grid_percentage)
        self.octomap_pcts.append(msg.octomap_percentage)

        # Keep last 1000 points
        max_points = 1000
        if len(self.times) > max_points:
            self.times = self.times[-max_points:]
            self.percentages = self.percentages[-max_points:]
            self.rates = self.rates[-max_points:]
            self.og_pcts = self.og_pcts[-max_points:]
            self.octomap_pcts = self.octomap_pcts[-max_points:]

    def update_plot(self, frame):
        """Update plot animation."""
        # Spin ROS2 to get new messages
        rclpy.spin_once(self.node, timeout_sec=0.01)

        if not self.times:
            return

        # Clear and redraw
        for ax in self.axes:
            ax.clear()

        # Plot 1: Exploration percentage
        self.axes[0].plot(self.times, self.percentages, 'b-', linewidth=2, label='Total')
        self.axes[0].plot(self.times, self.og_pcts, 'g--', linewidth=1, label='2D', alpha=0.7)
        self.axes[0].plot(self.times, self.octomap_pcts, 'r--', linewidth=1, label='3D', alpha=0.7)
        self.axes[0].set_xlabel('Time (s)')
        self.axes[0].set_ylabel('Exploration (%)')
        self.axes[0].set_title(f'Exploration Progress: {self.percentages[-1]:.1f}%')
        self.axes[0].legend(loc='lower right')
        self.axes[0].grid(True, alpha=0.3)
        self.axes[0].set_ylim(0, 105)

        # Plot 2: Exploration rate
        self.axes[1].plot(self.times, self.rates, 'g-', linewidth=2)
        self.axes[1].set_xlabel('Time (s)')
        self.axes[1].set_ylabel('Rate (%/s)')
        self.axes[1].set_title(f'Exploration Rate: {self.rates[-1]:.3f} %/s')
        self.axes[1].grid(True, alpha=0.3)

        self.fig.tight_layout()

    def run(self):
        """Start live plotting."""
        plt.show()
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Exploration Metrics Visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('--file', '-f', type=str,
                        help='Single CSV file to plot')
    parser.add_argument('--files', '-F', type=str, nargs='+',
                        help='Multiple CSV files to compare')
    parser.add_argument('--labels', '-l', type=str, nargs='+',
                        help='Labels for multiple files')
    parser.add_argument('--output', '-o', type=str,
                        help='Output file path (PNG, PDF, etc.)')
    parser.add_argument('--live', action='store_true',
                        help='Live plot from ROS2 topic')
    parser.add_argument('--topic', '-t', type=str,
                        default='/exploration/metrics',
                        help='ROS2 topic for live mode')
    parser.add_argument('--summary', '-s', action='store_true',
                        help='Print summary statistics')

    args = parser.parse_args()

    # Determine mode
    if args.live:
        if not ROS2_AVAILABLE:
            print("Error: ROS2 not available. Install rclpy for live plotting.")
            sys.exit(1)
        print(f"Starting live plot from topic: {args.topic}")
        plotter = LivePlotter(args.topic)
        plotter.run()

    elif args.file or args.files:
        csv_files = args.files if args.files else [args.file]
        labels = args.labels

        plotter = MetricsPlotter()

        if args.summary:
            for csv_file in csv_files:
                plotter.print_summary(csv_file)

        plotter.plot_from_csv(csv_files, labels, args.output)

    else:
        parser.print_help()
        print("\nError: Please specify --file, --files, or --live")
        sys.exit(1)


if __name__ == '__main__':
    main()
