# arduino_battery_ui_com6.py
import serial
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
COM_PORT = "COM12"
BAUD_RATE = 115200
UPDATE_INTERVAL = 100   # ms
WINDOW_SIZE = 200       # points on graphs

# ======================================================
# --------- SERIAL READER (PARSES ARDUINO CSV) ----------
# ======================================================
def com_reader_thread(port, callback):
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != 9:
                continue  # skip malformed lines
            try:
                v1 = float(parts[0]) / 1000.0
                v2 = float(parts[1]) / 1000.0
                v3 = float(parts[2]) / 1000.0
                v4 = float(parts[3]) / 1000.0
                current = float(parts[4])
                temp = float(parts[5])
                soc = int(float(parts[6]))
                adcFET = int(float(parts[7]))
                balanceFET = int(float(parts[8]))
                callback((v1, v2, v3, v4, current, temp, soc, balanceFET))
            except Exception as e:
                print("Parse error:", e, "line:", line)
        except Exception as e:
            print("Serial read error:", e)

# ======================================================
# ------------------- UI + GRAPH -----------------------
# ======================================================
class App:
    def __init__(self, root):
        self.root = root
        root.title("Arduino Battery Monitor UI")

        # time series buffers
        self.buffers = {name: deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
                        for name in ["V1","V2","V3","V4","Current","Temp"]}
        self.labels = {}
        self.time_buffer = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
        self.start_time = time.time()
        self.latest_values = None

        self.create_layout()
        self.connect_serial()
        self.update_ui()

    # ====================================================
    def create_layout(self):
        top = ttk.Frame(self.root)
        top.pack(side=tk.TOP, fill=tk.X)

        # numeric readouts
        reads = ttk.Frame(top)
        reads.pack(side=tk.LEFT, padx=20)
        for name in ["V1","V2","V3","V4","Current","Temp"]:
            lbl = ttk.Label(reads, text=f"{name}: 0.000", font=("Arial", 12))
            lbl.pack(anchor="w")
            self.labels[name] = lbl

        # right side: SOC gauge, battery bar, balancing indicators
        right = ttk.Frame(top)
        right.pack(side=tk.RIGHT, padx=10)

        self.gauge_size = 140
        self.gauge_canvas = tk.Canvas(right, width=self.gauge_size, height=self.gauge_size, bg="white", highlightthickness=0)
        self.gauge_canvas.pack(pady=6)
        self.gauge_center = (self.gauge_size//2, self.gauge_size//2)
        self.gauge_radius = (self.gauge_size//2) - 10
        self.soc_var = tk.StringVar(value="SOC: 0%")
        ttk.Label(right, textvariable=self.soc_var, font=("Arial", 12, "bold")).pack()

        self.batt_w = 120
        self.batt_h = 36
        self.batt_canvas = tk.Canvas(right, width=self.batt_w+10, height=self.batt_h+20, bg="white", highlightthickness=0)
        self.batt_canvas.pack(pady=6)
        self.batt_canvas.create_rectangle(2, 4, 2 + self.batt_w, 4 + self.batt_h, outline="black", width=2, tags="batt_outline")
        self.batt_canvas.create_rectangle(2 + self.batt_w, 4 + (self.batt_h//3), 2 + self.batt_w + 8, 4 + (2*self.batt_h//3), outline="black", width=2)

        bal_frame = ttk.Frame(right)
        bal_frame.pack(pady=6)
        ttk.Label(bal_frame, text="Balancing:", font=("Arial", 10)).pack(anchor="w")
        self.bal_canvas = tk.Canvas(bal_frame, width=160, height=36, bg="white", highlightthickness=0)
        self.bal_canvas.pack()
        self.bal_positions = []
        start_x = 10
        y = 18
        for i in range(4):
            x = start_x + i*(30)
            self.bal_positions.append((x,y))
            self.bal_canvas.create_oval(x-8, y-8, x+8, y+8, fill="#CCCCCC", outline="#666666", width=1, tags=f"bal{i+1}")
            self.bal_canvas.create_text(x, y+18, text=f"C{i+1}", font=("Arial", 8))

        # Voltages plot
        fig1, self.ax_v = plt.subplots(figsize=(6,2.2))
        self.lines_v = {name: self.ax_v.plot([], [], label=name)[0] for name in ["V1","V2","V3","V4"]}
        self.ax_v.set_title("Voltages (V)")
        self.ax_v.legend(loc="upper left")
        self.canvas_v = FigureCanvasTkAgg(fig1, master=self.root)
        self.canvas_v.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Current plot
        fig2, self.ax_i = plt.subplots(figsize=(6,2))
        self.line_i, = self.ax_i.plot([], [], label="Current (A)")
        self.ax_i.set_title("Current (A)")
        self.ax_i.legend(loc="upper left")
        self.canvas_i = FigureCanvasTkAgg(fig2, master=self.root)
        self.canvas_i.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Temperature plot
        fig3, self.ax_t = plt.subplots(figsize=(6,2))
        self.line_t, = self.ax_t.plot([], [], label="Temp (C)")
        self.ax_t.set_title("Temperature (°C)")
        self.ax_t.legend(loc="upper left")
        self.canvas_t = FigureCanvasTkAgg(fig3, master=self.root)
        self.canvas_t.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ====================================================
    def connect_serial(self):
        print(f"Connecting to {COM_PORT}...")
        thread = threading.Thread(target=com_reader_thread,
                                  args=(COM_PORT, self.update_from_thread),
                                  daemon=True)
        thread.start()

    # ====================================================
    def update_from_thread(self, values):
        self.latest_values = values

    # ====================================================
    def draw_gauge(self, soc):
        c = self.gauge_center
        r = self.gauge_radius
        self.gauge_canvas.delete("gauge")
        self.gauge_canvas.create_oval(c[0]-r, c[1]-r, c[0]+r, c[1]+r, outline="#ddd", width=10, tags="gauge")
        extent = int((soc / 100.0) * 360)
        self.gauge_canvas.create_arc(c[0]-r, c[1]-r, c[0]+r, c[1]+r, start=90 - extent, extent=extent,
                                     style="pieslice", outline="", tags="gauge", fill="#4caf50")
        inner_r = r - 18
        self.gauge_canvas.create_oval(c[0]-inner_r, c[1]-inner_r, c[0]+inner_r, c[1]+inner_r, fill="white", outline="", tags="gauge")
        self.gauge_canvas.create_text(c[0], c[1], text=f"{soc}%", font=("Arial", 14, "bold"), tags="gauge")

    def draw_battery_bar(self, soc):
        self.batt_canvas.delete("batt_fill")
        x0, y0 = 4, 6
        x1, y1 = 4 + self.batt_w - 2, 6 + self.batt_h - 2
        pct = max(0, min(100, soc)) / 100.0
        fill_w = int((x1 - x0) * pct)
        if pct > 0.66: color = "#4caf50"
        elif pct > 0.33: color = "#ffb300"
        else: color = "#f44336"
        self.batt_canvas.create_rectangle(x0, y0, x0 + fill_w, y1, fill=color, width=0, tags="batt_fill")
        self.batt_canvas.delete("batt_text")
        self.batt_canvas.create_text(self.batt_w//2, self.batt_h//2 + 4, text=f"{soc}%", font=("Arial", 10, "bold"), tags="batt_text")

    def update_balancing_indicators(self, bal_index):
        for i in range(4):
            tag = f"bal{i+1}"
            if (bal_index == (i+1)):
                self.bal_canvas.itemconfig(tag, fill="#00cc00")
            else:
                self.bal_canvas.itemconfig(tag, fill="#CCCCCC")

    # ====================================================
    def update_ui(self):
        elapsed = time.time() - self.start_time
        self.time_buffer.append(elapsed)

        vals = self.latest_values
        if vals is None:
            # fallback dummy data
            vals = (0,0,0,0,0,0,0,0)
        v1, v2, v3, v4, curr, temp, soc, bal = vals

        for name, val in zip(["V1","V2","V3","V4","Current","Temp"], [v1,v2,v3,v4,curr,temp]):
            self.labels[name].config(text=f"{name}: {val:.3f}" + (" V" if "V" in name else (" A" if "Current" in name else " °C")))

        self.buffers["V1"].append(v1)
        self.buffers["V2"].append(v2)
        self.buffers["V3"].append(v3)
        self.buffers["V4"].append(v4)
        self.buffers["Current"].append(curr)
        self.buffers["Temp"].append(temp)

        times = list(self.time_buffer)
        for name in ["V1","V2","V3","V4"]:
            self.lines_v[name].set_data(times, list(self.buffers[name]))
        if len(times) >= 2: self.ax_v.set_xlim(times[0], times[-1])
        vvals = [v for n in ["V1","V2","V3","V4"] for v in self.buffers[n]]
        if vvals:
            vmin, vmax = min(vvals), max(vvals)
            if vmin == vmax: vmin -= 0.1; vmax += 0.1
            self.ax_v.set_ylim(vmin - 0.05, vmax + 0.05)

        self.line_i.set_data(times, list(self.buffers["Current"]))
        if len(times) >= 2: self.ax_i.set_xlim(times[0], times[-1])
        cvals = list(self.buffers["Current"])
        if cvals:
            cmin, cmax = min(cvals), max(cvals)
            if cmin == cmax: cmin -= 0.1; cmax += 0.1
            self.ax_i.set_ylim(cmin - 0.1, cmax + 0.1)

        self.line_t.set_data(times, list(self.buffers["Temp"]))
        if len(times) >= 2: self.ax_t.set_xlim(times[0], times[-1])
        tvals = list(self.buffers["Temp"])
        if tvals:
            tmin, tmax = min(tvals), max(tvals)
            if tmin == tmax: tmin -= 1; tmax += 1
            self.ax_t.set_ylim(tmin - 1, tmax + 1)

        try:
            self.canvas_v.draw()
            self.canvas_i.draw()
            self.canvas_t.draw()
        except Exception:
            pass

        soc_clamped = int(max(0, min(100, soc)))
        self.soc_var.set(f"SOC: {soc_clamped}%")
        self.draw_gauge(soc_clamped)
        self.draw_battery_bar(soc_clamped)
        self.update_balancing_indicators(bal)

        self.root.after(UPDATE_INTERVAL, self.update_ui)

# ======================================================
# ------------------- MAIN -----------------------------
# ======================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
