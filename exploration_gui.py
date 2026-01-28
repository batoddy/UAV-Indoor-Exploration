#!/usr/bin/env python3
"""
UAV Exploration GUI - Basit arayuz
Simulasyonu baslatir ve kesfi kontrol eder
"""

import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import time

class ExplorationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("UAV Exploration Controller")
        self.root.geometry("400x300")
        self.root.resizable(False, False)

        self.terminals = []
        self.simulation_started = False

        # Ana frame
        main_frame = ttk.Frame(root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Baslik
        title_label = ttk.Label(main_frame, text="UAV Otonom Kesif Sistemi",
                                font=('Helvetica', 14, 'bold'))
        title_label.pack(pady=(0, 20))

        # Durum etiketi
        self.status_var = tk.StringVar(value="Durum: Hazir")
        status_label = ttk.Label(main_frame, textvariable=self.status_var,
                                 font=('Helvetica', 10))
        status_label.pack(pady=(0, 20))

        # Butonlar frame
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=10)

        # Simulasyonu Ac butonu
        self.sim_btn = ttk.Button(btn_frame, text="Simulasyonu Ac",
                                   command=self.start_simulation,
                                   width=20)
        self.sim_btn.pack(pady=5)

        # Kesfe Basla butonu
        self.start_btn = ttk.Button(btn_frame, text="Kesfe Basla",
                                     command=self.start_exploration,
                                     state=tk.DISABLED,
                                     width=20)
        self.start_btn.pack(pady=5)

        # Kesfi Durdur butonu
        self.stop_btn = ttk.Button(btn_frame, text="Kesfi Durdur",
                                    command=self.stop_exploration,
                                    state=tk.DISABLED,
                                    width=20)
        self.stop_btn.pack(pady=5)

        # Tum Terminalleri Kapat butonu
        self.close_btn = ttk.Button(btn_frame, text="Tum Terminalleri Kapat",
                                     command=self.close_all_terminals,
                                     width=20)
        self.close_btn.pack(pady=15)

        # Pencere kapatildiginda terminalleri de kapat
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def run_in_terminal(self, title, command, working_dir=None):
        """Komutu yeni bir gnome-terminal'de calistir"""
        if working_dir:
            full_cmd = f"cd {working_dir} && {command}"
        else:
            full_cmd = command

        # gnome-terminal ile ac
        proc = subprocess.Popen([
            'gnome-terminal',
            '--title', title,
            '--', 'bash', '-c', f'{full_cmd}; exec bash'
        ])
        self.terminals.append(proc)
        return proc

    def start_simulation(self):
        """Tum simulasyon komponentlerini baslat"""
        self.status_var.set("Durum: Simulasyon baslatiliyor...")
        self.sim_btn.config(state=tk.DISABLED)
        self.root.update()

        # (title, command, working_dir, wait_after_seconds)
        commands = [
            ("PX4 SITL",
             'PX4_GZ_WORLD=lawn PX4_GZ_MODEL_POSE="3,-2,0,0,0,0" make px4_sitl gz_x500_depth',
             os.path.expanduser("~/PX4-Autopilot"),
             1.0),

            ("MicroXRCE Agent",
             "MicroXRCEAgent udp4 -p 8888",
             None,
             1.0),

            ("QGroundControl",
             os.path.expanduser("~/QGC/QGroundControl.AppImage"),
             None,
             10.0),

            ("PX4 Bridge",
             "source /home/batoddy/uav_ws/install/setup.bash && ros2 launch cmd_vel_to_px4 px4_bridge.launch.py",
             None,
             5.0),

            ("Frontier Exploration",
             "source /home/batoddy/uav_ws/install/setup.bash && ros2 launch frontier_exploration frontier_exploration.launch.py",
             None,
             5.0),

            ("Exploration Planner",
             "source /home/batoddy/uav_ws/install/setup.bash && ros2 launch exploration_planner exploration_planner.launch.py rviz:=true",
             None,
             3.0),

            ("Exploration Metrics",
             "source /home/batoddy/uav_ws/install/setup.bash && ros2 launch exploration_metrics exploration_metrics.launch.py map:=complex_office comparison_mode:=both",
             None,
             0),
        ]

        for title, cmd, workdir, wait_time in commands:
            self.run_in_terminal(title, cmd, workdir)
            self.root.update()
            if wait_time > 0:
                time.sleep(wait_time)

        self.simulation_started = True
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.NORMAL)
        self.status_var.set("Durum: Simulasyon calisiyor")
        self.root.update()

    def run_ros2_command(self, cmd):
        """ROS2 komutunu source edip calistir"""
        full_cmd = f"source /opt/ros/humble/setup.bash && source /home/batoddy/uav_ws/install/setup.bash && {cmd}"
        return subprocess.run(
            ['bash', '-c', full_cmd],
            capture_output=True, text=True, timeout=10
        )

    def start_exploration(self):
        """Kesfi baslat"""
        try:
            result = self.run_ros2_command(
                "ros2 service call /exploration/start std_srvs/srv/Trigger"
            )

            if result.returncode == 0:
                self.status_var.set("Durum: Kesif aktif")
                messagebox.showinfo("Basarili", "Kesif baslatildi!")
            else:
                messagebox.showerror("Hata", f"Servis cagrisi basarisiz:\n{result.stderr}")
        except subprocess.TimeoutExpired:
            messagebox.showerror("Hata", "Servis cagrisi zaman asimina ugradi")
        except Exception as e:
            messagebox.showerror("Hata", f"Hata olustu: {str(e)}")

    def stop_exploration(self):
        """Kesfi durdur"""
        try:
            result = self.run_ros2_command(
                "ros2 service call /exploration/stop std_srvs/srv/Trigger"
            )

            if result.returncode == 0:
                self.status_var.set("Durum: Kesif durduruldu")
                messagebox.showinfo("Basarili", "Kesif durduruldu!")
            else:
                messagebox.showerror("Hata", f"Servis cagrisi basarisiz:\n{result.stderr}")
        except subprocess.TimeoutExpired:
            messagebox.showerror("Hata", "Servis cagrisi zaman asimina ugradi")
        except Exception as e:
            messagebox.showerror("Hata", f"Hata olustu: {str(e)}")

    def close_all_terminals(self):
        """Tum terminalleri kapat"""
        subprocess.run(['pkill', '-f', 'px4_sitl'], capture_output=True)
        subprocess.run(['pkill', '-f', 'px4'], capture_output=True)
        subprocess.run(['pkill', '-f', 'MicroXRCEAgent'], capture_output=True)
        subprocess.run(['pkill', '-f', 'QGroundControl'], capture_output=True)
        subprocess.run(['pkill', '-f', 'ros2'], capture_output=True)
        subprocess.run(['pkill', '-f', 'rviz'], capture_output=True)

        # Gazebo processlerini oldur
        subprocess.run(['pkill', '-f', 'gzserver'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gzclient'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gz sim'], capture_output=True)
        subprocess.run(['pkill', '-f', 'gz-sim'], capture_output=True)
        subprocess.run(['pkill', '-f', 'ruby'], capture_output=True)

        # Gazebo'yu zorla oldur (SIGKILL)
        subprocess.run(['pkill', '-9', '-f', 'gz'], capture_output=True)
        subprocess.run(['pkill', '-9', '-f', 'gazebo'], capture_output=True)

        # Kalan gz processlerini temizle
        subprocess.run(['killall', '-9', 'gz', 'gzserver', 'gzclient', 'ruby'],
                       capture_output=True, stderr=subprocess.DEVNULL)

        self.terminals.clear()
        self.simulation_started = False
        self.sim_btn.config(state=tk.NORMAL)
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_var.set("Durum: Hazir")
        self.root.update()
        messagebox.showinfo("Bilgi", "Tum terminaller kapatildi")

    def on_closing(self):
        """Pencere kapatilirken"""
        if self.simulation_started:
            if messagebox.askyesno("Cikis", "Simulasyon calisiyor. Terminalleri de kapatilsin mi?"):
                self.close_all_terminals()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = ExplorationGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
