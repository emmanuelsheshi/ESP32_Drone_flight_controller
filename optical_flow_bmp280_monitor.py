import asyncio
from collections import deque
import queue
import re
import threading
import time
from dataclasses import dataclass
from typing import Optional

import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from bleak import BleakClient, BleakScanner


BLE_DEVICE_NAME = "OpticalFlowADE"
BLE_CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


@dataclass
class Telemetry:
    velocity_x: Optional[float] = None
    velocity_y: Optional[float] = None
    velocity_z: Optional[float] = None
    flow_quality: Optional[int] = None
    distance_cm: Optional[float] = None
    relative_alt_cm: Optional[float] = None
    fused_height_cm: Optional[float] = None
    fused_source: Optional[str] = None
    bmp_temp_c: Optional[float] = None
    bmp_press_pa: Optional[float] = None
    bmp_alt_m: Optional[float] = None
    flight_state: Optional[str] = None
    arm_state: Optional[int] = None


def parse_float(text: str) -> Optional[float]:
    try:
        return float(text)
    except (ValueError, TypeError):
        return None


def parse_int(text: str) -> Optional[int]:
    try:
        return int(text)
    except (ValueError, TypeError):
        return None


def parse_telemetry_line(line: str) -> Telemetry:
    t = Telemetry()

    patterns = {
        "vx": r"Vel X:\s*([+-]?\d+(?:\.\d+)?)\s*cm/s",
        "vy": r"Vel Y:\s*([+-]?\d+(?:\.\d+)?)\s*cm/s",
        "vz": r"Vel Z:\s*([+-]?\d+(?:\.\d+)?)\s*cm/s",
        "quality": r"Quality:\s*(\d+)",
        "dist": r"Dist:\s*([+-]?\d+(?:\.\d+)?)\s*cm",
        "rel_alt": r"Rel Alt:\s*([+-]?\d+(?:\.\d+)?)\s*cm",
        "fused": r"Fused H:\s*([+-]?\d+(?:\.\d+)?)\s*cm\s*\((LiDAR|LIDAR|Lidar|BMP)\)",
        "temp": r"Temp:\s*([+-]?\d+(?:\.\d+)?)\s*C",
        "press": r"Press:\s*([+-]?\d+(?:\.\d+)?)\s*Pa",
        "alt": r"Alt:\s*([+-]?\d+(?:\.\d+)?)\s*m",
        "state": r"STATE:\s*([A-Z_]+)",
        "arm": r"ARM:\s*([01])",
    }

    m = re.search(patterns["vx"], line)
    if m:
        t.velocity_x = parse_float(m.group(1))

    m = re.search(patterns["vy"], line)
    if m:
        t.velocity_y = parse_float(m.group(1))

    m = re.search(patterns["vz"], line)
    if m:
        t.velocity_z = parse_float(m.group(1))

    m = re.search(patterns["quality"], line)
    if m:
        t.flow_quality = parse_int(m.group(1))

    m = re.search(patterns["dist"], line)
    if m:
        t.distance_cm = parse_float(m.group(1))

    m = re.search(patterns["rel_alt"], line)
    if m:
        t.relative_alt_cm = parse_float(m.group(1))

    m = re.search(patterns["fused"], line, flags=re.IGNORECASE)
    if m:
        t.fused_height_cm = parse_float(m.group(1))
        source = m.group(2)
        t.fused_source = "LiDAR" if source.lower() == "lidar" else "BMP"

    m = re.search(patterns["temp"], line)
    if m:
        t.bmp_temp_c = parse_float(m.group(1))

    m = re.search(patterns["press"], line)
    if m:
        t.bmp_press_pa = parse_float(m.group(1))

    m = re.search(patterns["alt"], line)
    if m:
        t.bmp_alt_m = parse_float(m.group(1))

    m = re.search(patterns["state"], line)
    if m:
        t.flight_state = m.group(1)

    m = re.search(patterns["arm"], line)
    if m:
        t.arm_state = parse_int(m.group(1))

    return t


class BleTelemetryWorker:
    def __init__(self, line_queue: queue.Queue[str], status_queue: queue.Queue[str]):
        self.line_queue = line_queue
        self.status_queue = status_queue
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self._line_buffer = bytearray()

    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()

    def _set_status(self, message: str) -> None:
        self.status_queue.put(message)

    def _on_notify(self, _: int, data: bytearray) -> None:
        self._line_buffer.extend(data)
        while True:
            nl_index = self._line_buffer.find(b"\n")
            if nl_index == -1:
                break
            raw = self._line_buffer[:nl_index]
            del self._line_buffer[: nl_index + 1]
            line = raw.decode("utf-8", errors="ignore").strip()
            if line:
                self.line_queue.put(line)

    async def _run_async(self) -> None:
        while not self.stop_event.is_set():
            self._set_status("Scanning for BLE device...")
            device = await BleakScanner.find_device_by_filter(
                lambda d, _: d.name == BLE_DEVICE_NAME,
                timeout=5.0,
            )

            if device is None:
                self._set_status("Device not found. Retrying...")
                await asyncio.sleep(1.0)
                continue

            self._set_status(f"Connecting to {device.name}...")
            try:
                async with BleakClient(device) as client:
                    await client.start_notify(BLE_CHAR_TX_UUID, self._on_notify)
                    self._set_status("Connected")

                    while client.is_connected and not self.stop_event.is_set():
                        await asyncio.sleep(0.1)

                    try:
                        await client.stop_notify(BLE_CHAR_TX_UUID)
                    except Exception:
                        pass
            except Exception as ex:
                self._set_status(f"Connection error: {ex}")
                await asyncio.sleep(1.0)

        self._set_status("Disconnected")

    def _run(self) -> None:
        try:
            asyncio.run(self._run_async())
        except Exception as ex:
            self._set_status(f"Worker stopped: {ex}")


class TelemetryMonitorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Flight Telemetry Monitor")
        self.root.geometry("1180x700")

        self.line_queue: queue.Queue[str] = queue.Queue()
        self.status_queue: queue.Queue[str] = queue.Queue()
        self.worker = BleTelemetryWorker(self.line_queue, self.status_queue)

        self.last_rx_time = 0.0
        self.rx_rate_hz = 0.0
        self.plot_window_sec = 30.0
        self.plot_points = 300
        self.start_time = time.time()

        self.plot_t = deque(maxlen=self.plot_points)
        self.plot_fused_h = deque(maxlen=self.plot_points)
        self.plot_quality = deque(maxlen=self.plot_points)
        self.plot_vz = deque(maxlen=self.plot_points)

        self.status_var = tk.StringVar(value="Idle")
        self.frame_summary_var = tk.StringVar(value="Flow: - | Height: - | Src: - | Flight: -")
        self.detail_summary_var = tk.StringVar(value="Motion: -\nHeight: -\nEnvironment: -\nFlight: -")

        self.vx_var = tk.StringVar(value="-")
        self.vy_var = tk.StringVar(value="-")
        self.vz_var = tk.StringVar(value="-")
        self.quality_var = tk.StringVar(value="-")

        self.dist_var = tk.StringVar(value="-")
        self.rel_alt_var = tk.StringVar(value="-")
        self.fused_h_var = tk.StringVar(value="-")
        self.fused_src_var = tk.StringVar(value="-")

        self.temp_var = tk.StringVar(value="-")
        self.press_var = tk.StringVar(value="-")
        self.alt_var = tk.StringVar(value="-")

        self.flow_state_var = tk.StringVar(value="UNKNOWN")
        self.bmp_state_var = tk.StringVar(value="UNKNOWN")
        self.fused_state_var = tk.StringVar(value="NO DATA")
        self.rate_var = tk.StringVar(value="0.0 Hz")
        self.flight_state_var = tk.StringVar(value="-")
        self.arm_var = tk.StringVar(value="-")

        self.plot_canvas: Optional[FigureCanvasTkAgg] = None
        self.fig: Optional[Figure] = None
        self.ax_h = None
        self.ax_q = None
        self.ax_vz = None
        self.line_h = None
        self.line_q = None
        self.line_vz = None
        self.details_text: Optional[tk.Text] = None
        self.has_flow_sample = False
        self.has_bmp_sample = False

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._poll_queues)
        self.root.after(250, self._update_plot)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=10)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(7, weight=1)

        ttk.Button(top, text="Connect", command=self._connect).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(top, text="Disconnect", command=self._disconnect).grid(row=0, column=1, padx=(0, 16))

        ttk.Label(top, text="BLE Status:").grid(row=0, column=2, sticky="w")
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=3, sticky="w", padx=(6, 20))

        ttk.Label(top, text="RX Rate:").grid(row=0, column=4, sticky="w")
        ttk.Label(top, textvariable=self.rate_var).grid(row=0, column=5, sticky="w", padx=(6, 20))

        ttk.Label(top, text="Snapshot:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(top, textvariable=self.frame_summary_var).grid(
            row=1, column=1, columnspan=7, sticky="w", pady=(8, 0)
        )

        ttk.Label(top, text="Frame Details:").grid(row=2, column=0, sticky="nw", pady=(4, 0))
        details_frame = ttk.Frame(top)
        details_frame.grid(row=2, column=1, columnspan=7, sticky="ew", pady=(4, 0))
        details_frame.columnconfigure(0, weight=1)
        details_frame.rowconfigure(0, weight=1)

        self.details_text = tk.Text(details_frame, height=5, wrap="word")
        self.details_text.grid(row=0, column=0, sticky="ew")
        self.details_text.insert("1.0", self.detail_summary_var.get())
        self.details_text.config(state="disabled")

        content = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        content.grid(row=1, column=0, sticky="nsew")
        content.columnconfigure(0, weight=1)
        content.rowconfigure(0, weight=1)

        tabs = ttk.Notebook(content)
        tabs.grid(row=0, column=0, sticky="nsew")

        dashboard_tab = ttk.Frame(tabs)
        dashboard_tab.columnconfigure(0, weight=1)
        dashboard_tab.columnconfigure(1, weight=1)
        dashboard_tab.rowconfigure(0, weight=1)
        tabs.add(dashboard_tab, text="Dashboard")

        plots_tab = ttk.Frame(tabs)
        plots_tab.columnconfigure(0, weight=1)
        plots_tab.rowconfigure(0, weight=1)
        tabs.add(plots_tab, text="Live Plots")

        summary = ttk.LabelFrame(dashboard_tab, text="Status", padding=6)
        summary.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(8, 8))
        ttk.Label(summary, text="Flow:").grid(row=0, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.flow_state_var).grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Label(summary, text="BMP:").grid(row=1, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.bmp_state_var).grid(row=1, column=1, sticky="w", padx=(8, 0))
        ttk.Label(summary, text="Height Source:").grid(row=2, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.fused_state_var).grid(row=2, column=1, sticky="w", padx=(8, 0))
        ttk.Label(summary, text="Flight State:").grid(row=3, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.flight_state_var).grid(row=3, column=1, sticky="w", padx=(8, 0))
        ttk.Label(summary, text="Armed:").grid(row=4, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.arm_var).grid(row=4, column=1, sticky="w", padx=(8, 0))

        flow = ttk.LabelFrame(dashboard_tab, text="Optical Flow", padding=12)
        flow.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=(8, 8))
        self._metric(flow, 0, "Velocity X (cm/s)", self.vx_var)
        self._metric(flow, 1, "Velocity Y (cm/s)", self.vy_var)
        self._metric(flow, 2, "Velocity Z (cm/s)", self.vz_var)
        self._metric(flow, 3, "Flow Quality", self.quality_var)

        height = ttk.LabelFrame(dashboard_tab, text="Height + BMP280", padding=12)
        height.grid(row=1, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        self._metric(height, 0, "Distance (cm)", self.dist_var)
        self._metric(height, 1, "Relative Alt (cm)", self.rel_alt_var)
        self._metric(height, 2, "Fused Height (cm)", self.fused_h_var)
        self._metric(height, 3, "Fused Source", self.fused_src_var)
        self._metric(height, 4, "BMP Temp (C)", self.temp_var)
        self._metric(height, 5, "BMP Pressure (Pa)", self.press_var)
        self._metric(height, 6, "BMP Altitude (m)", self.alt_var)

        flight = ttk.LabelFrame(dashboard_tab, text="Flight State", padding=12)
        flight.grid(row=1, column=1, sticky="nsew", padx=(8, 0), pady=(0, 8))
        self._metric(flight, 0, "Mode", self.flight_state_var)
        self._metric(flight, 1, "Arm", self.arm_var)

        self._build_plot_area(plots_tab)

    def _build_plot_area(self, parent: ttk.Frame) -> None:
        self.fig = Figure(figsize=(8.5, 5.2), dpi=100)
        self.ax_h = self.fig.add_subplot(311)
        self.ax_q = self.fig.add_subplot(312)
        self.ax_vz = self.fig.add_subplot(313)

        self.line_h, = self.ax_h.plot([], [], color="#007f5f", linewidth=1.8, label="Fused Height (cm)")
        self.line_q, = self.ax_q.plot([], [], color="#bc6c25", linewidth=1.8, label="Flow Quality")
        self.line_vz, = self.ax_vz.plot([], [], color="#0a9396", linewidth=1.8, label="Vel Z (cm/s)")

        self.ax_h.set_ylabel("Height")
        self.ax_q.set_ylabel("Quality")
        self.ax_vz.set_ylabel("Vel Z")
        self.ax_vz.set_xlabel("Time (s)")

        self.ax_h.grid(True, alpha=0.25)
        self.ax_q.grid(True, alpha=0.25)
        self.ax_vz.grid(True, alpha=0.25)

        self.ax_h.set_ylim(0, 200)
        self.ax_q.set_ylim(0, 255)
        self.ax_vz.set_ylim(-120, 120)

        self.ax_h.legend(loc="upper left")
        self.ax_q.legend(loc="upper left")
        self.ax_vz.legend(loc="upper left")

        self.fig.tight_layout(pad=1.0)
        self.plot_canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.plot_canvas.draw()
        self.plot_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def _metric(self, parent: ttk.LabelFrame, row: int, label: str, value_var: tk.StringVar) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Label(parent, textvariable=value_var).grid(row=row, column=1, sticky="e", pady=2)

    def _connect(self) -> None:
        self.worker.start()

    def _disconnect(self) -> None:
        self.worker.stop()

    def _on_close(self) -> None:
        self.worker.stop()
        self.root.after(150, self.root.destroy)

    def _apply_telemetry(self, telemetry: Telemetry, raw_line: str) -> None:
        now = time.time()
        if self.last_rx_time > 0:
            dt = now - self.last_rx_time
            if dt > 0:
                instant = 1.0 / dt
                self.rx_rate_hz = (0.8 * self.rx_rate_hz) + (0.2 * instant) if self.rx_rate_hz else instant
        self.last_rx_time = now
        self.rate_var.set(f"{self.rx_rate_hz:.1f} Hz")

        raw_preview = raw_line[:220]

        if telemetry.velocity_x is not None:
            self.vx_var.set(f"{telemetry.velocity_x:.2f}")
        if telemetry.velocity_y is not None:
            self.vy_var.set(f"{telemetry.velocity_y:.2f}")
        if telemetry.velocity_z is not None:
            self.vz_var.set(f"{telemetry.velocity_z:.2f}")
        if telemetry.flow_quality is not None:
            self.quality_var.set(str(telemetry.flow_quality))

        if telemetry.distance_cm is not None:
            self.dist_var.set(f"{telemetry.distance_cm:.2f}")
        if telemetry.relative_alt_cm is not None:
            self.rel_alt_var.set(f"{telemetry.relative_alt_cm:.2f}")
        if telemetry.fused_height_cm is not None:
            self.fused_h_var.set(f"{telemetry.fused_height_cm:.2f}")
        if telemetry.fused_source is not None:
            self.fused_src_var.set(telemetry.fused_source)
        if telemetry.fused_height_cm is not None:
            self.fused_state_var.set(self.fused_source_to_state(telemetry.fused_source))
        else:
            self.fused_state_var.set("NO DATA")

        if telemetry.flight_state is not None:
            self.flight_state_var.set(telemetry.flight_state)

        if telemetry.bmp_temp_c is not None:
            self.temp_var.set(f"{telemetry.bmp_temp_c:.2f}")
        if telemetry.bmp_press_pa is not None:
            self.press_var.set(f"{telemetry.bmp_press_pa:.2f}")
        if telemetry.bmp_alt_m is not None:
            self.alt_var.set(f"{telemetry.bmp_alt_m:.2f}")

        flow_ok = telemetry.velocity_x is not None and telemetry.velocity_y is not None and telemetry.flow_quality is not None
        bmp_ok = telemetry.bmp_temp_c is not None and telemetry.bmp_press_pa is not None and telemetry.bmp_alt_m is not None
        if flow_ok:
            self.flow_state_var.set("OK")
            self.has_flow_sample = True
        elif not self.has_flow_sample:
            self.flow_state_var.set("NO DATA")

        if bmp_ok:
            self.bmp_state_var.set("OK")
            self.has_bmp_sample = True
        elif not self.has_bmp_sample:
            self.bmp_state_var.set("NO DATA")

        if telemetry.arm_state is None:
            self.arm_var.set("-")
        else:
            self.arm_var.set("ARMED" if telemetry.arm_state == 1 else "DISARMED")

        q_text = "-" if telemetry.flow_quality is None else str(telemetry.flow_quality)
        h_text = "-" if telemetry.fused_height_cm is None else f"{telemetry.fused_height_cm:.1f} cm"
        src_text = "-" if telemetry.fused_source is None else telemetry.fused_source
        flight_text = self.flight_state_var.get()
        self.frame_summary_var.set(f"Flow Q: {q_text} | Height: {h_text} | Source: {src_text} | Flight: {flight_text}")

        motion_detail = (
            f"Motion: VX {self.vx_var.get()} | VY {self.vy_var.get()} | VZ {self.vz_var.get()} | Q {q_text}"
        )
        height_detail = (
            f"Height: Dist {self.dist_var.get()} | RelAlt {self.rel_alt_var.get()} | Fused {self.fused_h_var.get()} ({self.fused_src_var.get()})"
        )
        env_detail = (
            f"Environment: Temp {self.temp_var.get()} | Press {self.press_var.get()} | Alt {self.alt_var.get()}"
        )
        flight_detail = f"Flight: {self.flight_state_var.get()} | Armed: {self.arm_var.get()}"
        details_text = f"{motion_detail}\n{height_detail}\n{env_detail}\n{flight_detail}\nRaw: {raw_preview}"
        self.detail_summary_var.set(details_text)
        if self.details_text is not None:
            self.details_text.config(state="normal")
            self.details_text.delete("1.0", tk.END)
            self.details_text.insert("1.0", details_text)
            self.details_text.config(state="disabled")

        elapsed = now - self.start_time
        self.plot_t.append(elapsed)
        self.plot_fused_h.append(float("nan") if telemetry.fused_height_cm is None else telemetry.fused_height_cm)
        self.plot_quality.append(float("nan") if telemetry.flow_quality is None else float(telemetry.flow_quality))
        self.plot_vz.append(float("nan") if telemetry.velocity_z is None else telemetry.velocity_z)

    def _update_plot(self) -> None:
        if self.line_h is not None and len(self.plot_t) > 1:
            x = list(self.plot_t)
            y_h = list(self.plot_fused_h)
            y_q = list(self.plot_quality)
            y_vz = list(self.plot_vz)

            self.line_h.set_data(x, y_h)
            self.line_q.set_data(x, y_q)
            self.line_vz.set_data(x, y_vz)

            x_end = x[-1]
            x_start = max(0.0, x_end - self.plot_window_sec)

            self.ax_h.set_xlim(x_start, x_end + 0.1)
            self.ax_q.set_xlim(x_start, x_end + 0.1)
            self.ax_vz.set_xlim(x_start, x_end + 0.1)

            valid_h = [v for v in y_h if v == v]
            valid_vz = [v for v in y_vz if v == v]

            if valid_h:
                h_min = min(valid_h)
                h_max = max(valid_h)
                pad_h = max(5.0, (h_max - h_min) * 0.15)
                self.ax_h.set_ylim(h_min - pad_h, h_max + pad_h)

            if valid_vz:
                vz_min = min(valid_vz)
                vz_max = max(valid_vz)
                pad_vz = max(10.0, (vz_max - vz_min) * 0.15)
                self.ax_vz.set_ylim(vz_min - pad_vz, vz_max + pad_vz)

            if self.plot_canvas is not None:
                self.plot_canvas.draw_idle()

        self.root.after(250, self._update_plot)

    def _poll_queues(self) -> None:
        try:
            while True:
                status = self.status_queue.get_nowait()
                self.status_var.set(status)
        except queue.Empty:
            pass

        try:
            while True:
                raw = self.line_queue.get_nowait()
                telemetry = parse_telemetry_line(raw)
                self._apply_telemetry(telemetry, raw)
        except queue.Empty:
            pass

        self.root.after(100, self._poll_queues)

    @staticmethod
    def fused_source_to_state(source: Optional[str]) -> str:
        if source is None:
            return "UNKNOWN"
        return "LiDAR" if source.lower() == "lidar" else "BMP"


def main() -> None:
    root = tk.Tk()
    app = TelemetryMonitorApp(root)
    app._connect()
    root.mainloop()


if __name__ == "__main__":
    main()
