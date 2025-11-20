import random
import serial
import serial.tools.list_ports
import threading
import time
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque

# ======================================================
# ------------------- CONFIG ---------------------------
# ======================================================
USE_COM_PORT = False  # Start in test mode
COM_PORT = "COM5"
BAUD_RATE = 115200

UPDATE_INTERVAL = 100   # ms
WINDOW_SIZE = 200       # points on graphs

# ======================================================
# ------------------- DATA SOURCE ----------------------
# ======================================================

def get_random_measurements():
    v1 = round(random.uniform(0, 3.3), 3)
    v2 = round(random.uniform(0, 3.3), 3)
    v3 = round(random.uniform(0, 3.3), 3)
    v4 = round(random.uniform(0, 3.3), 3)
    current = round(random.uniform(0, 2), 3)
    temp = round(random.uniform(20, 80), 2)
    return v1, v2, v3, v4, current, temp


def com_reader_thread(port, callback):
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    while True:
        try:
            line = ser.readline().decode().strip()
            parts = line.split()
            if len(parts) == 6:
                callback(tuple(map(float, parts)))
        except Exception:
            pass


# ======================================================
# ------------------- UI + GRAPH -----------------------
# ======================================================

class App:
    def __init__(self, root):
        self.root = root
        root.title("PIC32 Live Measurement UI")

        self.buffers = {
            "V1": deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE),
            "V2": deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE),
            "V3": deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE),
            "V4": deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE),
            "Current": deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE),
            "Temp": deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE),
        }

        self.labels = {}
        self.time_buffer = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
        self.start_time = time.time()

        self.create_layout()

        self.latest_values = (0, 0, 0, 0, 0, 0)

        self.update_ui()

    # ---------------- UI Layout ----------------
    def create_layout(self):
        frame_top = ttk.Frame(self.root)
        frame_top.pack(side=tk.TOP, fill=tk.X)

        # =========================
        #     COM Port Picker
        # =========================
        com_frame = ttk.Frame(frame_top)
        com_frame.pack(side=tk.LEFT, padx=10)

        ttk.Label(com_frame, text="COM Port:", font=("Arial", 12)).pack(side=tk.LEFT)

        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.com_var = tk.StringVar()
        self.com_dropdown = ttk.Combobox(
            com_frame, textvariable=self.com_var, values=ports, width=10, state="readonly"
        )
        self.com_dropdown.pack(side=tk.LEFT, padx=5)

        self.connect_button = ttk.Button(com_frame, text="Connect", command=self.connect_serial)
        self.connect_button.pack(side=tk.LEFT)

        # =========================
        # Numeric Displays
        # =========================
        for name in self.buffers.keys():
            lbl = ttk.Label(frame_top, text=f"{name}: 0.0", font=("Arial", 14))
            lbl.pack(side=tk.LEFT, padx=15)
            self.labels[name] = lbl

        # =========================
        # Graph 1: Voltages
        # =========================
        fig1, self.ax_v = plt.subplots(figsize=(6,3))
        self.lines_v = {name: self.ax_v.plot([], [], label=name)[0]
                        for name in ["V1","V2","V3","V4"]}
        self.ax_v.set_title("Voltages (V1, V2, V3, V4)")
        self.ax_v.legend(loc="upper left")
        self.canvas_v = FigureCanvasTkAgg(fig1, master=self.root)
        self.canvas_v.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # =========================
        # Graph 2: Current
        # =========================
        fig2, self.ax_i = plt.subplots(figsize=(6,3))
        self.line_i, = self.ax_i.plot([], [], label="Current")
        self.ax_i.set_title("Current")
        self.ax_i.legend(loc="upper left")
        self.canvas_i = FigureCanvasTkAgg(fig2, master=self.root)
        self.canvas_i.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # =========================
        # Graph 3: Temperature
        # =========================
        fig3, self.ax_t = plt.subplots(figsize=(6,3))
        self.line_t, = self.ax_t.plot([], [], label="Temp")
        self.ax_t.set_title("Temperature")
        self.ax_t.legend(loc="upper left")
        self.canvas_t = FigureCanvasTkAgg(fig3, master=self.root)
        self.canvas_t.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ====================================================
    # Connect Button Handler
    # ====================================================
    def connect_serial(self):
        port = self.com_var.get()
        if not port:
            print("No COM port selected.")
            return

        print(f"Connecting to {port}...")

        global USE_COM_PORT, COM_PORT
        USE_COM_PORT = True
        COM_PORT = port

        thread = threading.Thread(
            target=com_reader_thread,
            args=(port, self.update_from_thread),
            daemon=True
        )
        thread.start()

        self.connect_button.config(text="Connected", state="disabled")
        self.com_dropdown.config(state="disabled")

    # ====================================================
    def update_from_thread(self, values):
        self.latest_values = values

    # ====================================================
    def update_ui(self):
        elapsed = time.time() - self.start_time
        self.time_buffer.append(elapsed)

        if USE_COM_PORT:
            v1, v2, v3, v4, curr, temp = self.latest_values
        else:
            v1, v2, v3, v4, curr, temp = get_random_measurements()

        vals = {"V1": v1, "V2": v2, "V3": v3, "V4": v4, "Current": curr, "Temp": temp}
        for k, v in vals.items():
            self.labels[k].config(text=f"{k}: {v}")

        for k, v in vals.items():
            self.buffers[k].append(v)

        times = list(self.time_buffer)

        # Voltages
        for name in ["V1", "V2", "V3", "V4"]:
            self.lines_v[name].set_data(times, list(self.buffers[name]))
        self.ax_v.set_xlim(times[0], times[-1])
        volt_vals = [v for n in ["V1", "V2", "V3", "V4"] for v in self.buffers[n]]
        self.ax_v.set_ylim(min(volt_vals)-0.5, max(volt_vals)+0.5)

        # Current
        self.line_i.set_data(times, list(self.buffers["Current"]))
        self.ax_i.set_xlim(times[0], times[-1])
        curr_vals = list(self.buffers["Current"])
        self.ax_i.set_ylim(min(curr_vals)-0.5, max(curr_vals)+0.5)

        # Temperature
        self.line_t.set_data(times, list(self.buffers["Temp"]))
        self.ax_t.set_xlim(times[0], times[-1])
        temp_vals = list(self.buffers["Temp"])
        self.ax_t.set_ylim(min(temp_vals)-1, max(temp_vals)+1)

        self.canvas_v.draw()
        self.canvas_i.draw()
        self.canvas_t.draw()

        self.root.after(UPDATE_INTERVAL, self.update_ui)


# ======================================================
# ------------------- MAIN -----------------------------
# ======================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
