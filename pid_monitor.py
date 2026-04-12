"""
PID Force Control Monitor — macOS / PyCharm compatible

Features
--------
- Live serial monitor for Arduino force-control output
- Live force vs target plotting
- Default PID values: Kp=10, Ki=0.5, Kd=0
- Saved PID persistence to trials/stable_pid.json
- GO / STOP / Tare / Params / Help / Save PID / Save Plot
- Offline CSV graphing with --graph

Run
---
    python pid_monitor.py
    python pid_monitor.py --port /dev/cu.usbmodem14101
    python pid_monitor.py --graph trials/trial_xyz.csv
"""

import argparse
import csv
import json
import os
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib

for _backend in ["macosx", "Qt5Agg", "TkAgg", "Agg"]:
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        continue

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import serial
import serial.tools.list_ports


BAUD_RATE = 115200
MAX_PLOT_PTS = 600
CSV_FOLDER = "trials"
PID_SAVE_FILE = os.path.join(CSV_FOLDER, "stable_pid.json")
DEFAULT_PID = {"Kp": 10.0, "Ki": 0.5, "Kd": 0.0}

times = deque(maxlen=MAX_PLOT_PTS)
forces = deque(maxlen=MAX_PLOT_PTS)
targets = deque(maxlen=MAX_PLOT_PTS)

csv_writer = None
csv_file_obj = None
recording = False
trial_name = ""
ser = None
status_msg = "NOT CONNECTED"
log_lines = []
t_offset = None

stable_pid = DEFAULT_PID.copy()

fig = None
ax = None
line_force = None
line_target = None
title_txt = None
log_text = None
pid_status_txt = None
ax_pid_lbl = None
txt_box = None


def log(msg: str) -> None:
    print(msg)
    log_lines.append(msg)
    if len(log_lines) > 9:
        log_lines.pop(0)


def list_ports():
    return list(serial.tools.list_ports.comports())


def find_best_port():
    keywords = ["arduino", "ch340", "cp210", "ftdi", "usbmodem", "usbserial"]
    for p in list_ports():
        combined = ((p.description or "") + " " + (p.device or "")).lower()
        if any(k in combined for k in keywords):
            return p.device
    ports = list_ports()
    return ports[0].device if ports else None


def send_command(cmd: str) -> None:
    global status_msg
    cmd = cmd.strip()
    if not cmd:
        return
    if ser is None or not ser.is_open:
        log("[ERR]  Not connected.")
        status_msg = "NOT CONNECTED"
        return
    try:
        ser.write((cmd + "\n").encode("utf-8"))
        log(f"[SENT] {cmd}")
    except Exception as e:
        log(f"[ERR]  {e}")


def save_stable_pid(kp: float, ki: float, kd: float) -> None:
    os.makedirs(CSV_FOLDER, exist_ok=True)
    data = {"Kp": kp, "Ki": ki, "Kd": kd, "saved": datetime.now().isoformat()}
    with open(PID_SAVE_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    stable_pid["Kp"] = kp
    stable_pid["Ki"] = ki
    stable_pid["Kd"] = kd
    log(f"[PID]  Saved: Kp={kp}  Ki={ki}  Kd={kd}  → {PID_SAVE_FILE}")
    update_pid_status_label()


def load_stable_pid() -> bool:
    if not os.path.exists(PID_SAVE_FILE):
        stable_pid.update(DEFAULT_PID)
        log("[PID]  Using default PID (no saved file found)")
        return False
    try:
        with open(PID_SAVE_FILE, encoding="utf-8") as f:
            data = json.load(f)
        stable_pid["Kp"] = data["Kp"]
        stable_pid["Ki"] = data["Ki"]
        stable_pid["Kd"] = data["Kd"]
        log(f"[PID]  Loaded: Kp={data['Kp']}  Ki={data['Ki']}  Kd={data['Kd']}")
        log(f"[PID]  (saved {data.get('saved', '?')[:10]})")
        return True
    except Exception as e:
        log(f"[PID]  Could not load {PID_SAVE_FILE}: {e}")
        stable_pid.update(DEFAULT_PID)
        return False


def apply_stable_pid() -> None:
    kp = stable_pid["Kp"]
    ki = stable_pid["Ki"]
    kd = stable_pid["Kd"]
    send_command(f"kp:{kp}")
    time.sleep(0.05)
    send_command(f"ki:{ki}")
    time.sleep(0.05)
    send_command(f"kd:{kd}")
    log(f"[PID]  Applied: Kp={kp}  Ki={ki}  Kd={kd}")


def start_csv() -> None:
    global csv_writer, csv_file_obj, trial_name, recording
    os.makedirs(CSV_FOLDER, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    trial_name = f"trial_{ts}"
    path = os.path.join(CSV_FOLDER, f"{trial_name}.csv")
    csv_file_obj = open(path, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file_obj)
    csv_writer.writerow([
        "Time_ms", "RawADC", "Voltage_V",
        "Force_lbs", "Target_lbs", "Kp", "Ki", "Kd"
    ])
    recording = True
    log(f"[CSV]  → {path}")


def stop_csv() -> None:
    global csv_writer, csv_file_obj, recording
    recording = False
    if csv_file_obj:
        csv_file_obj.close()
        csv_file_obj = None
        csv_writer = None
        log("[CSV]  File closed.")


def serial_reader() -> None:
    global status_msg, t_offset

    while True:
        try:
            if ser is None or not ser.is_open:
                time.sleep(0.5)
                continue

            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue

            if raw.startswith("DATA,"):
                parts = raw.split(",")
                if len(parts) == 9:
                    t_raw = int(parts[1]) / 1000.0
                    force = float(parts[4])
                    target = float(parts[5])

                    if t_offset is None:
                        t_offset = t_raw
                    t_s = t_raw - t_offset

                    times.append(t_s)
                    forces.append(force)
                    targets.append(target)

                    if recording and csv_writer:
                        csv_writer.writerow(parts[1:])

                    status_msg = (
                        f"Force {force:.1f} lbs  |  Target {target:.1f} lbs  |  "
                        f"Kp={parts[6]}  Ki={parts[7]}  Kd={parts[8]}"
                    )

            elif raw.startswith("HEADER,"):
                times.clear()
                forces.clear()
                targets.clear()
                t_offset = None
                start_csv()
                log(f"[ARD]  {raw}")

            elif raw.startswith("STATUS,"):
                msg = raw[7:]
                log(f"[STS]  {msg}")
                if "RETRACT_COMPLETE" in msg or "STOPPING" in msg:
                    stop_csv()
                    status_msg = "Ready — press GO to start a new trial."

            elif raw.startswith("SET,"):
                log(f"[SET]  {raw[4:]}")

            elif raw.startswith("ERROR,"):
                log(f"[!!!]  {raw[6:]}")
                stop_csv()
                status_msg = f"ERROR: {raw[6:]}"

            else:
                log(f"[ARD]  {raw}")

        except Exception as e:
            log(f"[RDR]  {e}")
            time.sleep(0.2)


def update_pid_status_label() -> None:
    if pid_status_txt is None or ax_pid_lbl is None:
        return
    source = "DEFAULT" if not os.path.exists(PID_SAVE_FILE) else os.path.basename(PID_SAVE_FILE)
    pid_status_txt.set_text(
        f"PID ({source}) | Kp={stable_pid['Kp']}  Ki={stable_pid['Ki']}  Kd={stable_pid['Kd']}"
    )
    ax_pid_lbl.set_facecolor("#f0fdf4")


def on_go(_):
    apply_stable_pid()
    time.sleep(0.1)
    send_command("go")


def on_stop(_):
    send_command("s")


def on_tare(_):
    send_command("tare")


def on_prm(_):
    send_command("params")


def on_hlp(_):
    send_command("help")


def on_sav_pid(_):
    save_stable_pid(stable_pid["Kp"], stable_pid["Ki"], stable_pid["Kd"])


def on_sav_plt(_):
    os.makedirs(CSV_FOLDER, exist_ok=True)
    path = os.path.join(CSV_FOLDER, f"{trial_name or 'plot'}_graph.png")
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    log(f"[SVD]  {path}")


def on_send(_):
    cmd = txt_box.text.strip()
    if not cmd:
        return
    for part in cmd.lower().split():
        for key in ("kp", "ki", "kd"):
            if part.startswith(f"{key}:"):
                try:
                    stable_pid[key.capitalize()] = float(part.split(":")[1])
                except ValueError:
                    pass
    update_pid_status_label()
    send_command(cmd)
    txt_box.set_val("")


def animate(_):
    if len(times) >= 2:
        xs = list(times)
        line_force.set_data(xs, list(forces))
        line_target.set_data(xs, list(targets))
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)
        ax.set_xlim(0, max(xs) + 0.5)

    title_txt.set_text(status_msg)
    log_text.set_text("\n".join(log_lines))
    return line_force, line_target, title_txt, log_text


def graph_csv(filepath: str) -> None:
    t_vals, f_vals, tgt_vals = [], [], []
    with open(filepath, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            t_vals.append(float(row["Time_ms"]) / 1000.0)
            f_vals.append(float(row["Force_lbs"]))
            tgt_vals.append(float(row["Target_lbs"]))

    if t_vals:
        t0 = t_vals[0]
        t_vals = [t - t0 for t in t_vals]

    fig2, ax2 = plt.subplots(figsize=(10, 5))
    ax2.plot(t_vals, f_vals, label="Force (lbs)", lw=1.8)
    ax2.plot(t_vals, tgt_vals, label="Target (lbs)", lw=1.5, linestyle="--")

    if tgt_vals:
        tgt = tgt_vals[-1]
        band = tgt * 0.02
        ax2.axhspan(tgt - band, tgt + band, alpha=0.25, label="±2% band")
        settled = next((t for t, f in zip(t_vals, f_vals) if abs(f - tgt) <= band), None)
        if settled is not None:
            ax2.axvline(settled, lw=1.2, linestyle=":", label=f"Settled @ {settled:.2f}s")

    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force (lbs)")
    ax2.set_xlim(left=0)
    ax2.set_title(f"Force Response — {os.path.basename(filepath)}")
    ax2.legend()
    ax2.grid(True, alpha=0.4)
    plt.tight_layout()
    out = filepath.replace(".csv", "_plot.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"[SAVED] {out}")
    plt.show()


def build_gui():
    global fig, ax, line_force, line_target, title_txt, log_text, pid_status_txt, ax_pid_lbl, txt_box

    fig = plt.figure(figsize=(12, 7))
    fig.patch.set_facecolor("white")

    ax = fig.add_axes([0.07, 0.40, 0.90, 0.53])
    ax.set_facecolor("white")
    ax.tick_params(colors="black")
    for spine in ax.spines.values():
        spine.set_edgecolor("#aaa")
    ax.set_xlabel("Time (s)", color="black")
    ax.set_ylabel("Force (lbs)", color="black")
    ax.grid(True, color="#ddd", linewidth=0.5)

    line_force, = ax.plot([], [], lw=1.8, label="Force (lbs)")
    line_target, = ax.plot([], [], lw=1.5, linestyle="--", label="Target")
    title_txt = ax.set_title("", color="black", fontsize=10)
    ax.legend(loc="upper left", facecolor="white", labelcolor="black", edgecolor="#ccc")

    ax_log = fig.add_axes([0.07, 0.20, 0.90, 0.17])
    ax_log.set_facecolor("#f5f5f5")
    ax_log.axis("off")
    log_text = ax_log.text(
        0.01, 0.97, "", transform=ax_log.transAxes,
        color="#1a7a3a", fontsize=8, va="top", fontfamily="monospace"
    )

    ax_pid_lbl = fig.add_axes([0.07, 0.14, 0.90, 0.055])
    ax_pid_lbl.set_facecolor("#eff6ff")
    ax_pid_lbl.axis("off")
    pid_status_txt = ax_pid_lbl.text(
        0.5, 0.5, "", transform=ax_pid_lbl.transAxes,
        color="#1e40af", fontsize=9, ha="center", va="center",
        fontfamily="monospace", fontweight="bold"
    )
    update_pid_status_label()

    BC, BH = "#374151", "#4b5563"
    ax_go = fig.add_axes([0.07, 0.06, 0.10, 0.055])
    ax_stop = fig.add_axes([0.185, 0.06, 0.10, 0.055])
    ax_tare = fig.add_axes([0.30, 0.06, 0.10, 0.055])
    ax_prm = fig.add_axes([0.41, 0.06, 0.10, 0.055])
    ax_sav_pid = fig.add_axes([0.52, 0.06, 0.13, 0.055])
    ax_sav_plt = fig.add_axes([0.66, 0.06, 0.13, 0.055])
    ax_hlp = fig.add_axes([0.80, 0.06, 0.10, 0.055])

    btn_go = Button(ax_go, "GO", color="#166534", hovercolor="#15803d")
    btn_stop = Button(ax_stop, "STOP", color="#7f1d1d", hovercolor="#b91c1c")
    btn_tare = Button(ax_tare, "Tare", color=BC, hovercolor=BH)
    btn_prm = Button(ax_prm, "Params", color=BC, hovercolor=BH)
    btn_sav_pid = Button(ax_sav_pid, "Save PID", color="#1e3a5f", hovercolor="#1d4ed8")
    btn_sav_plt = Button(ax_sav_plt, "Save Plot", color=BC, hovercolor=BH)
    btn_hlp = Button(ax_hlp, "Help", color=BC, hovercolor=BH)

    for b in [btn_go, btn_stop, btn_tare, btn_prm, btn_sav_pid, btn_sav_plt, btn_hlp]:
        b.label.set_color("white")
        b.label.set_fontsize(9)

    ax_cmd = fig.add_axes([0.07, 0.005, 0.72, 0.048])
    ax_send = fig.add_axes([0.80, 0.005, 0.17, 0.048])

    txt_box = TextBox(ax_cmd, "Command: ", initial="", color="#1f2937", hovercolor="#374151", label_pad=0.05)
    txt_box.label.set_color("white")
    txt_box.text_disp.set_color("#93c5fd")
    txt_box.text_disp.set_fontsize(9)

    btn_send = Button(ax_send, "Send", color="#374151", hovercolor="#4b5563")
    btn_send.label.set_color("white")
    btn_send.label.set_fontsize(9)

    txt_box.on_submit(lambda t: on_send(None))
    btn_go.on_clicked(on_go)
    btn_stop.on_clicked(on_stop)
    btn_tare.on_clicked(on_tare)
    btn_prm.on_clicked(on_prm)
    btn_sav_pid.on_clicked(on_sav_pid)
    btn_sav_plt.on_clicked(on_sav_plt)
    btn_hlp.on_clicked(on_hlp)
    btn_send.on_clicked(on_send)


def main() -> None:
    global ser, status_msg

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", default=BAUD_RATE, type=int)
    parser.add_argument("--graph", default=None)
    args = parser.parse_args()

    if args.graph:
        graph_csv(args.graph)
        return

    print("\n=== PID Force Control Monitor ===")
    found_saved = load_stable_pid()
    if found_saved:
        print(f"  Stable PID: Kp={stable_pid['Kp']}  Ki={stable_pid['Ki']}  Kd={stable_pid['Kd']}")
    else:
        print("  No stable_pid.json found.")
        print("  Using default PID: Kp=10  Ki=0.5  Kd=0")

    print("\n=== Available Serial Ports ===")
    for p in list_ports():
        print(f"  {p.device:35s} {p.description}")
    print("=" * 38)

    build_gui()

    port = args.port or find_best_port()

    if not port:
        print("\nERROR: No port found.")
        status_msg = "NO PORT — plug in Arduino and restart"
    else:
        print(f"\nConnecting to: {port}")
        print("Close Serial Monitor in Arduino IDE first.\n")
        try:
            ser = serial.Serial(port, args.baud, timeout=1)
            time.sleep(2)
            log(f"[OK]   Connected to {port}")
            time.sleep(0.5)
            apply_stable_pid()
            status_msg = (
                f"Ready — Kp={stable_pid['Kp']}  Ki={stable_pid['Ki']}  "
                f"Kd={stable_pid['Kd']} — press GO"
            )
            threading.Thread(target=serial_reader, daemon=True).start()
        except serial.SerialException as e:
            print(f"\nERROR: {e}")
            print("Fix: close Serial Monitor in Arduino IDE, then re-run.")
            status_msg = "PORT BUSY — close Serial Monitor and restart"

    animation.FuncAnimation(fig, animate, interval=80, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
