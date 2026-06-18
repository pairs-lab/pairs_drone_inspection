import tkinter as tk
from tkinter import ttk
from datetime import datetime
from PIL import Image, ImageTk
from src import config

class OperatorConsole:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Operator Console")
        self.root.geometry("480x800")
        self.root.configure(bg=config.BG_COLOR)
        
        self.cam_window = tk.Toplevel(self.root)
        self.cam_window.title("Drone FPV Camera")
        self.cam_window.geometry("480x360")
        self.cam_window.configure(bg="#000000")
        self.cam_window.protocol("WM_DELETE_WINDOW", lambda: self.cam_window.withdraw())

        self.lbl_cam = tk.Label(self.cam_window, text="FPV CAMERA (NO SIGNAL)", bg="black", fg="#475569", font=("Arial", 12, "bold"))
        self.lbl_cam.pack(expand=True, fill="both")
        
        self.ui_cards = {}
        self.build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def build_ui(self):
        tk.Label(self.root, text="WAREHOUSE GRID — 3 × 6", font=("Arial", 10, "bold"), bg=config.BG_COLOR, fg="#475569", anchor="w").pack(fill="x", padx=20, pady=(20, 0))
        self.lbl_status = tk.Label(self.root, text="STATUS: READY TO FLY", font=("Arial", 11, "bold"), bg=config.BG_COLOR, fg=config.TEXT_MAIN)
        self.lbl_status.pack(pady=5)

        grid_frame = tk.Frame(self.root, bg=config.BG_COLOR)
        grid_frame.pack(padx=15)

        for i, text in enumerate(["A", "B", "C"]):
            tk.Label(grid_frame, text=text, font=("Arial", 10, "bold"), bg=config.BG_COLOR, fg=config.TEXT_MAIN).grid(row=0, column=i, pady=5)

        for row_idx, lvl in enumerate([1, 2, 3, 4, 5, 6]):
            for col_idx, col in enumerate(["A", "B", "C"]):
                bid = f"{col}{lvl}"
                data = config.bin_data[bid]

                card_frame = tk.Frame(grid_frame, bg=config.CARD_BG, highlightbackground=config.CARD_BORDER, highlightthickness=1, width=120, height=80)
                card_frame.grid(row=row_idx+1, column=col_idx, padx=5, pady=5)
                card_frame.pack_propagate(False) 

                lbl_id = tk.Label(card_frame, text=bid, font=("Arial", 12, "bold"), bg=config.CARD_BG, fg=config.TEXT_MAIN, anchor="w")
                lbl_id.pack(fill="x", padx=10, pady=(8, 0))
                lbl_pn = tk.Label(card_frame, text=data["part"], font=("Arial", 8), bg=config.CARD_BG, fg=config.TEXT_SUB, anchor="w")
                lbl_pn.pack(fill="x", padx=10)
                lbl_qty = tk.Label(card_frame, text=f"Qty: {data['qty']}", font=("Arial", 10, "bold"), bg=config.CARD_BG, fg=config.TEXT_QTY, anchor="w")
                lbl_qty.pack(fill="x", padx=10, pady=(2, 0))

                for widget in [card_frame, lbl_id, lbl_pn, lbl_qty]:
                    widget.bind("<Button-1>", lambda e, b=bid: self.on_card_click(b))

                self.ui_cards[bid] = {"frame": card_frame, "lbl_id": lbl_id, "lbl_pn": lbl_pn, "lbl_qty": lbl_qty}

        tk.Button(self.root, text="RETURN HOME", bg="#334155", fg="white", font=("Arial", 10, "bold"), 
                  relief="flat", activebackground="#475569", activeforeground="white",
                  command=lambda: self.on_card_click("HOME")).pack(pady=10, ipadx=20, ipady=5)

        tk.Label(self.root, text="INSPECTION HISTORY", font=("Arial", 10, "bold"), bg=config.BG_COLOR, fg="#475569", anchor="w").pack(fill="x", padx=20, pady=(0, 5))

        style = ttk.Style()
        style.theme_use("default")
        style.configure("Treeview", background=config.BG_COLOR, foreground="white", fieldbackground=config.BG_COLOR, borderwidth=0, rowheight=30)
        style.configure("Treeview.Heading", background=config.BG_COLOR, foreground=config.TEXT_SUB, borderwidth=0, font=("Arial", 10, "bold"))
        style.map("Treeview", background=[('selected', config.CARD_BG)])

        cols = ("Bin", "Time", "Scanned", "Status")
        self.history_tree = ttk.Treeview(self.root, columns=cols, show="headings", height=8)
        for col in cols:
            self.history_tree.heading(col, text=col, anchor="w" if col != "Status" else "center")
        
        self.history_tree.column("Bin", width=50, anchor="w")
        self.history_tree.column("Time", width=100, anchor="w")
        self.history_tree.column("Scanned", width=150, anchor="w")
        self.history_tree.column("Status", width=100, anchor="center")
        
        self.history_tree.tag_configure('success_tag', foreground=config.COMPLETED_TEXT)
        self.history_tree.tag_configure('error_tag', foreground=config.ERROR_BORDER)
        self.history_tree.pack(padx=20, fill="x")

    def on_card_click(self, bin_id):
        if not config.is_flying and bin_id not in config.inspected_bins:
            config.current_target_id = bin_id
        elif bin_id == "HOME" and not config.is_flying:
            config.current_target_id = "HOME"

    def set_card_state(self, bin_id, state):
        def _update():
            card = self.ui_cards.get(bin_id)
            if not card: return
            if state == "active":
                card["frame"].config(highlightbackground=config.ACTIVE_BORDER)
                self.lbl_status.config(text=f"STATUS: SCANNING BIN {bin_id}", fg="orange")
            elif state == "completed":
                card["frame"].config(bg=config.COMPLETED_BG, highlightbackground=config.COMPLETED_BG)
                card["lbl_id"].config(bg=config.COMPLETED_BG, fg=config.COMPLETED_TEXT)
                card["lbl_pn"].config(bg=config.COMPLETED_BG)
                card["lbl_qty"].config(bg=config.COMPLETED_BG)
                self.lbl_status.config(text=f"OK! BIN {bin_id} IS MATCHING", fg=config.COMPLETED_TEXT)
            elif state == "error":
                card["frame"].config(bg=config.ERROR_BG, highlightbackground=config.ERROR_BORDER)
                card["lbl_id"].config(bg=config.ERROR_BG, fg=config.ERROR_TEXT)
                card["lbl_pn"].config(bg=config.ERROR_BG, text="NOT MATCHING!", fg="white")
                card["lbl_qty"].config(bg=config.ERROR_BG, fg="white")
                self.lbl_status.config(text=f"ERROR: Mismatch at bin {bin_id}!", fg="#ef4444")
        self.root.after(0, _update)

    def handle_inspection_result(self, bin_id, status):
        def _update():
            self.set_card_state(bin_id, status)
            now = datetime.now().strftime("%I:%M:%S %p")
            if status == "error":
                part_info, tag, disp_status = "UNKNOWN (MISMATCH)", "error_tag", "mismatch"
            else:
                part_info = f"{config.bin_data[bin_id]['part']} / {config.bin_data[bin_id]['qty']}"
                tag, disp_status = "success_tag", "completed"
            self.history_tree.insert("", "0", values=(bin_id, now, part_info, disp_status), tags=(tag,))
        self.root.after(0, _update)

    def update_camera_ui(self, pixels):
        def _update():
            if not config.app_running: return
            if pixels is None:
                self.lbl_cam.config(image='', text="FPV CAMERA (NO SIGNAL)")
                self.lbl_cam.image = None
            else:
                if self.cam_window.state() == 'withdrawn':
                    self.cam_window.deiconify()
                imgtk = ImageTk.PhotoImage(image=Image.fromarray(pixels))
                self.lbl_cam.config(image=imgtk, text="")
                self.lbl_cam.image = imgtk
        self.root.after(0, _update)

    def on_closing(self):
        config.app_running = False
        self.cam_window.destroy()
        self.root.destroy()

    def run(self):
        self.root.mainloop()