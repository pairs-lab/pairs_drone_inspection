#!/usr/bin/env python3
import rospy
import tkinter as tk
from tkinter import ttk
import threading
import time

from apriltag_ros.msg import AprilTagDetectionArray
from pairs_msgs.srv import Vec4, Vec4Request 

class WarehouseCommander:
    def __init__(self):
        rospy.init_node('warehouse_commander_gui', anonymous=True)
        self.uav_name = rospy.get_param('~uav_name', 'uav1')
        
        # Rack configuration
        self.BAY_WIDTH = 2.70
        self.LEVEL_HEIGHT = 1.20
        self.STANDOFF_DEPTH = 1.50
        
        self.tag_detected = False
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_z = 0.0

        self.current_bin = None # Track the current bin index the drone is at

        # Flags and errors for Visual Servoing
        self.servo_active = False
        self.servo_error_x = None
        self.servo_error_y = None

        self.last_x = -5.5
        self.last_y = 0.0
        self.last_z = 1.5

        # 6 racks configuration
        self.racks_config = {
            1: {"id": 101, "y_start": 6.3, "dir": 1},
            2: {"id": 102, "y_start": 3.7, "dir": -1},
            3: {"id": 103, "y_start": 1.3, "dir": 1},
            4: {"id": 104, "y_start": -1.3, "dir": -1},
            5: {"id": 105, "y_start": -3.7, "dir": 1},
            6: {"id": 106, "y_start": -6.3, "dir": -1},
        }
        self.selected_rack = 3

        import subprocess
        rospy.loginfo("Starting Apriltag detection module...")
        self.apriltag_process = subprocess.Popen(["roslaunch", "inspection_gazebo", "apriltag.launch", f"uav_name:={self.uav_name}"])

        tag_topic = f'/{self.uav_name}/tag_detections'
        rospy.Subscriber(tag_topic, AprilTagDetectionArray, self.tag_callback)

        service_topic = f'/{self.uav_name}/control_manager/goto'
        rospy.wait_for_service(service_topic)
        self.goto_client = rospy.ServiceProxy(service_topic, Vec4)
        
        self.gui_thread = threading.Thread(target=self.build_gui)
        self.gui_thread.start()

    def tag_callback(self, msg):
        # Only lock the coordinate when starting Phase 2 (zigzag flight)
        # If not locked, allow continuous updates for the most accurate coordinates when the drone stops
        if getattr(self, 'lock_anchor', False):
            return
            
        target_id = self.racks_config[self.selected_rack]["id"]
        rack_y = self.racks_config[self.selected_rack]["y_start"]
        for detection in msg.detections:
            if target_id in detection.id:
                pose = detection.pose.pose.pose
                cam_x = pose.position.x
                cam_y = pose.position.y
                cam_z = pose.position.z
                
                self.anchor_x = -5.5 + cam_z
                self.anchor_y = rack_y - cam_x
                self.anchor_z = 1.5 - cam_y
                
                self.tag_detected = True
                return
        
        # Scan for bin tags (ID 94) to center
        if getattr(self, 'servo_active', False):
            best_z = 999.0
            best_cam_x = None
            best_cam_y = None
            
            for detection in msg.detections:
                if 94 in detection.id:
                    cam_z = detection.pose.pose.pose.position.z
                    # Only take the nearest Tag to avoid confusing neighboring bins
                    if cam_z < best_z:
                        best_z = cam_z
                        best_cam_x = detection.pose.pose.pose.position.x
                        best_cam_y = detection.pose.pose.pose.position.y
                        
            if best_cam_x is not None:
                self.servo_error_x = best_cam_x
                self.servo_error_y = best_cam_y

    def send_flight_command(self, x, y, z, heading):
        import math
        try:
            req = Vec4Request()
            req.goal = [float(x), float(y), float(z), float(heading)]
            response = self.goto_client(req)
            if response.success:
                dist = math.hypot(x - self.last_x, y - self.last_y, z - self.last_z)
                self.last_x = float(x)
                self.last_y = float(y)
                self.last_z = float(z)
                return True, dist
            return False, 0.0
        except rospy.ServiceException as e:
            rospy.logerr(f"Lỗi gửi lệnh: {e}")
            return False, 0.0

    def execute_global_approach(self):
        """ Phase 1: Fly to shared corridor X=-5.5 and fly along the rack """
        rack_y = self.racks_config[self.selected_rack]["y_start"]
        target_id = self.racks_config[self.selected_rack]["id"]
        rospy.loginfo(f"[Phase 1] Approaching Rack {self.selected_rack} (Y={rack_y}) targeting Anchor Tag {target_id}...")
        
        self.tag_detected = False
        self.lock_anchor = False
        self.current_bin = None
        
        # Step 1: Move to shared corridor X = -5.5 (to avoid hitting old rack)
        success, dist = self.send_flight_command(-5.5, self.last_y, 1.5, 0.0)
        time.sleep(max(3.0, dist * 1.0))
        
        # Step 2: Fly along the corridor to the new rack
        success, dist = self.send_flight_command(-5.5, rack_y, 1.5, 0.0)
        time.sleep(max(3.0, dist * 1.0))
        
        timeout = 15.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.tag_detected:
                rospy.loginfo("[Phase 1] Locked Anchor Tag! Ready for Phase 2.")
                return
            time.sleep(0.5)
            
        rospy.logwarn("[Phase 1] Reached the end of the rack and waited 15s but Tag not found. Please check the camera!")

    def run_visual_servo(self, rack_dir, target_y, heading):
        self.servo_active = True
        self.servo_error_x = None
        self.servo_error_y = None
        time.sleep(1.0) # Wait for camera recognition
        
        for step in range(5):
            if not getattr(self, 'servo_active', False):
                break
                
            if self.servo_error_x is None:
                rospy.logwarn("  [Servo] Could not see Bin Tag (ID 94), skipping Centering.")
                break
                
            if abs(self.servo_error_x) < 0.05 and abs(self.servo_error_y) < 0.05:
                rospy.loginfo("  [Servo] Perfectly centered!")
                break
                
            # Calculate new coordinates to compensate for error (Proportional Gain = 0.7)
            new_x = self.last_x + (self.servo_error_x * rack_dir * 0.7)
            new_z = self.last_z - (self.servo_error_y * 0.7)
            
            self.send_flight_command(new_x, target_y, new_z, heading)
            
            self.servo_error_x = None # Clear old error
            time.sleep(2.5) # Wait for drone flight and camera stabilization
            
        self.servo_active = False

    def execute_zigzag_sequence(self, target_bin, full_scan=False):
        """ Phase 2: Zigzag flight sequentially through the Bins """
        if not self.tag_detected:
            rospy.logwarn("[ERROR] Could not lock Anchor Tag. Please press 'Find Rack' button first!")
            return

        self.lock_anchor = True
        
        ZIGZAG_ORDER = [1, 2, 3, 6, 5, 4, 7, 8, 9, 12, 11, 10, 13, 14, 15, 18, 17, 16]
        
        if full_scan:
            rospy.loginfo("[Phase 2] Starting FULL RACK ZIGZAG SCAN (1 -> 18)")
            path = ZIGZAG_ORDER
        else:
            rospy.loginfo(f"[Phase 2] Generating Zigzag trajectory to BIN {target_bin}")
            end_idx = ZIGZAG_ORDER.index(target_bin)
            if self.current_bin is None:
                path = ZIGZAG_ORDER[0 : end_idx + 1]
            else:
                start_idx = ZIGZAG_ORDER.index(self.current_bin)
                if start_idx < end_idx:
                    path = ZIGZAG_ORDER[start_idx + 1 : end_idx + 1]
                elif start_idx > end_idx:
                    path = ZIGZAG_ORDER[start_idx - 1 : end_idx - 1 : -1]
                else:
                    rospy.loginfo("Drone is already at this bin!")
                    return
        
        rack_dir = self.racks_config[self.selected_rack]["dir"]
        target_y = self.anchor_y - (self.STANDOFF_DEPTH * rack_dir)
        heading = 1.57 if rack_dir == 1 else -1.57

        # If at the start of the aisle, move back to the middle of the aisle before starting the lateral slide
        if abs(self.last_x - (-5.5)) < 0.2:
            self.send_flight_command(-5.5, target_y, 1.5, heading)
            time.sleep(4.0)

        for step_bin in path:
            rospy.loginfo(f"\n=> Moving to BIN {step_bin}...")
            bin_index = step_bin - 1
            col = bin_index % 3
            row = bin_index // 3
            
            offset_x = 1.4 + col * self.BAY_WIDTH
            offset_z = row * self.LEVEL_HEIGHT
            
            target_x = self.anchor_x + offset_x
            target_z = self.anchor_z - 0.4 + offset_z
            
            # Take off to Bin (Point to point safely because it's along the Manhattan axis)
            success, dist = self.send_flight_command(target_x, target_y, target_z, heading)
            time.sleep(max(3.0, dist * 1.5)) 
            
            # Perform centering at this Bin
            self.run_visual_servo(rack_dir, target_y, heading)
            
            # Update current bin
            self.current_bin = step_bin
            
            # Scan/Take photo
            rospy.loginfo(f"[✓] Finished scanning BIN {step_bin}")
            time.sleep(1.0)
            
        rospy.loginfo(f"\n[Finished] Zigzag trajectory completed at BIN {target_bin if not full_scan else 18}")

    def build_gui(self):
        root = tk.Tk()
        root.title("PAIRS Autonomous GCS")
        root.geometry("350x350")

        tk.Label(root, text="RELATIVE AUTONOMY", font=("Arial", 12, "bold")).pack(pady=10)

        # SELECT RACK
        frame_rack = tk.Frame(root)
        frame_rack.pack(pady=5)
        tk.Label(frame_rack, text="Select Rack: ").pack(side=tk.LEFT)
        self.rack_var = tk.StringVar()
        combo_rack = ttk.Combobox(frame_rack, textvariable=self.rack_var, values=[f"Rack {i}" for i in range(1, 7)], state="readonly", width=10)
        combo_rack.current(2) # Default Rack 3
        combo_rack.pack(side=tk.LEFT)
        
        def on_rack_change(event):
            self.selected_rack = int(self.rack_var.get().split()[1])
            self.tag_detected = False
            self.lock_anchor = False
            self.current_bin = None
        combo_rack.bind("<<ComboboxSelected>>", on_rack_change)

        # PHASE 1 BUTTON
        tk.Button(root, text="1. AUTO FIND RACK", bg="orange", fg="black", 
                  font=("Arial", 10, "bold"), width=30, height=2, 
                  command=lambda: threading.Thread(target=self.execute_global_approach).start()).pack(pady=5)

        # STATUS
        status_label = tk.Label(root, text="Not seeing Anchor Tag", fg="red", font=("Arial", 10, "italic"))
        status_label.pack(pady=5)

        def update_status():
            if self.tag_detected:
                status_label.config(text="Locked Anchor Tag - Ready", fg="green")
            else:
                status_label.config(text="Not seeing Anchor Tag", fg="red")
            root.after(500, update_status)
        root.after(500, update_status)

        # SELECT BIN
        frame_bin = tk.Frame(root)
        frame_bin.pack(pady=10)
        tk.Label(frame_bin, text="Select Bin: ").pack(side=tk.LEFT)
        bin_var = tk.StringVar()
        combo_bin = ttk.Combobox(frame_bin, textvariable=bin_var, values=[f"Bin {i}" for i in range(1, 19)], state="readonly", width=10)
        combo_bin.current(0)
        combo_bin.pack(side=tk.LEFT)

        # PHASE 2 BUTTON
        tk.Button(root, text="2.MOVE TO BIN", bg="#0066cc", fg="white", 
                  font=("Arial", 10, "bold"), width=30, height=2, 
                  command=lambda: threading.Thread(target=self.execute_zigzag_sequence, args=(int(bin_var.get().split()[1]), False)).start()).pack(pady=5)
                  
        # PHASE 3 BUTTON: SCAN ENTIRE RACK (1 -> 18)
        tk.Button(root, text="3. SCAN ENTIRE RACK", bg="#28a745", fg="white", 
                  font=("Arial", 10, "bold"), width=30, height=2, 
                  command=lambda: threading.Thread(target=self.execute_zigzag_sequence, args=(18, True)).start()).pack(pady=10)

        def on_closing():
            if hasattr(self, 'apriltag_process'):
                self.apriltag_process.terminate()
            root.destroy()
            rospy.signal_shutdown("GUI closed")
        
        root.protocol("WM_DELETE_WINDOW", on_closing)
        root.mainloop()

if __name__ == '__main__':
    try:
        nav = WarehouseCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass