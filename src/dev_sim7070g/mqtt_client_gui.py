# -*- coding: utf-8 -*-
"""
GUI: Gui lenh Bat/Tat bom qua MQTT (ma hoa), co nut Gui va Auto gui theo chu ky.

Chay: python mqtt_client_gui.py  (tu thu muc chua file nay hoac tu root project)
Can: pip install paho-mqtt pycryptodome (tkinter co san trong Python).
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from datetime import datetime
from typing import Optional

# Cho phep import mqtt_client khi chay tu bat ky thu muc nao
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

# tkinter (built-in)
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

# Import MQTT client va config
from mqtt_client import CONFIG, MqttClient

# -----------------------------------------------------------------------------
# Config (giong mqtt_client)
# -----------------------------------------------------------------------------
NODE_ID = CONFIG["NODE_ID"]
BROKER = CONFIG["MQTT_SERVER"]
PORT = CONFIG["MQTT_PORT"]
USERNAME = CONFIG["USERNAME"]
PASSWORD = CONFIG["PASSWORD"]


class MqttGuiApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MQTT Bat / Tat bom")
        self.root.geometry("520x420")
        self.root.minsize(400, 320)

        self.client: Optional[MqttClient] = None
        self._auto_running = False
        self._auto_job: Optional[str] = None
        self._auto_interval_sec = 30
        self._auto_send_on = True  # True = Bat, False = Tat

        self._build_ui()
        self._connect_mqtt()

    def _build_ui(self):
        # ---- Trang thai ----
        f_status = ttk.LabelFrame(self.root, text="Trang thai", padding=6)
        f_status.pack(fill=tk.X, padx=8, pady=4)
        self.lbl_status = ttk.Label(f_status, text="Dang ket noi...")
        self.lbl_status.pack(anchor=tk.W)

        # ---- Gui lenh (Bat / Tat) ----
        f_cmd = ttk.LabelFrame(self.root, text="Gui lenh", padding=8)
        f_cmd.pack(fill=tk.X, padx=8, pady=4)

        row1 = ttk.Frame(f_cmd)
        row1.pack(fill=tk.X)
        self.btn_on = ttk.Button(row1, text="Bat (X=1)", command=self._send_on)
        self.btn_on.pack(side=tk.LEFT, padx=(0, 8))
        self.btn_off = ttk.Button(row1, text="Tat (X=0)", command=self._send_off)
        self.btn_off.pack(side=tk.LEFT, padx=(0, 8))

        # ---- Auto gui ----
        f_auto = ttk.LabelFrame(self.root, text="Auto gui", padding=8)
        f_auto.pack(fill=tk.X, padx=8, pady=4)

        row_auto = ttk.Frame(f_auto)
        row_auto.pack(fill=tk.X)
        ttk.Label(row_auto, text="Gui:").pack(side=tk.LEFT, padx=(0, 4))
        self.var_auto_cmd = tk.StringVar(value="on")
        ttk.Radiobutton(row_auto, text="Bat (X=1)", variable=self.var_auto_cmd, value="on").pack(side=tk.LEFT, padx=(0, 12))
        ttk.Radiobutton(row_auto, text="Tat (X=0)", variable=self.var_auto_cmd, value="off").pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(row_auto, text="Chu ky (giay):").pack(side=tk.LEFT, padx=(12, 4))
        self.ent_interval = ttk.Entry(row_auto, width=6)
        self.ent_interval.insert(0, "30")
        self.ent_interval.pack(side=tk.LEFT, padx=(0, 8))
        self.btn_auto_start = ttk.Button(row_auto, text="Bat Auto", command=self._start_auto)
        self.btn_auto_start.pack(side=tk.LEFT, padx=(8, 4))
        self.btn_auto_stop = ttk.Button(row_auto, text="Dung Auto", command=self._stop_auto, state=tk.DISABLED)
        self.btn_auto_stop.pack(side=tk.LEFT)

        # ---- Log nhan tin ----
        f_log = ttk.LabelFrame(self.root, text="Log (tin nhan nhan duoc da giai ma)", padding=6)
        f_log.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self.txt_log = scrolledtext.ScrolledText(f_log, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _log(self, msg: str) -> None:
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {msg}"
        def do():
            self.txt_log.configure(state=tk.NORMAL)
            self.txt_log.insert(tk.END, line + "\n")
            self.txt_log.see(tk.END)
            self.txt_log.configure(state=tk.DISABLED)
        self.root.after(0, do)

    def _set_status(self, text: str) -> None:
        self.root.after(0, lambda: self.lbl_status.configure(text=text))

    def _connect_mqtt(self) -> None:
        self._set_status("Dang ket noi...")
        self.client = MqttClient(
            node_id=NODE_ID,
            username=USERNAME,
            password=PASSWORD,
            broker=BROKER,
            port=PORT,
        )
        self.client.set_on_message(self._on_mqtt_message)
        if not self.client.connect():
            self._set_status("Loi ket noi MQTT")
            self._log("[Loi] Khong ket noi duoc broker.")
            return
        self.client.loop_start()
        # Cap nhat trang thai sau khi connect (trong on_connect)
        threading.Timer(1.5, self._check_connected).start()

    def _check_connected(self) -> None:
        if self.client and self.client._connected:
            self._set_status(f"Da ket noi | {BROKER}:{PORT} | node={NODE_ID[:8]}...")
            self._log("[MQTT] Da ket noi va sub topic.")
        else:
            self._set_status("Chua ket noi (cho them hoac kiem tra mang)")

    def _on_mqtt_message(self, topic: str, raw: str, plain: Optional[str]) -> None:
        if plain is None:
            self._log(f"[RX] {topic} -> [rejected] decrypt failed or outside time window")
        else:
            self._log(f"[RX] {topic} -> {plain}")

    def _send_on(self) -> None:
        if not self.client or not self.client._connected:
            messagebox.showwarning("Canh bao", "Chua ket noi MQTT.")
            return
        ok = self.client.publish_command({"X": 1}, encrypt=True)
        if ok:
            self._log("[TX] Gui lenh Bat (X=1)")

    def _send_off(self) -> None:
        if not self.client or not self.client._connected:
            messagebox.showwarning("Canh bao", "Chua ket noi MQTT.")
            return
        ok = self.client.publish_command({"X": 0}, encrypt=True)
        if ok:
            self._log("[TX] Gui lenh Tat (X=0)")

    def _start_auto(self) -> None:
        try:
            sec = int(self.ent_interval.get().strip())
            if sec < 5:
                sec = 5
            if sec > 86400:
                sec = 86400
        except ValueError:
            sec = 30
        self._auto_interval_sec = sec
        self._auto_send_on = self.var_auto_cmd.get() == "on"
        self._auto_running = True
        self.btn_auto_start.configure(state=tk.DISABLED)
        self.btn_auto_stop.configure(state=tk.NORMAL)
        self.ent_interval.configure(state=tk.DISABLED)
        self._log(f"[Auto] Bat gui {'Bat (X=1)' if self._auto_send_on else 'Tat (X=0)'} moi {self._auto_interval_sec}s")
        self._schedule_auto()

    def _schedule_auto(self) -> None:
        if not self._auto_running:
            return
        if self.client and self.client._connected:
            cmd = {"X": 1} if self._auto_send_on else {"X": 0}
            self.client.publish_command(cmd, encrypt=True)
            self._log(f"[Auto] Gui {'X=1' if self._auto_send_on else 'X=0'} (sau {self._auto_interval_sec}s gui lai)")
        self._auto_job = self.root.after(self._auto_interval_sec * 1000, self._schedule_auto)

    def _stop_auto(self) -> None:
        self._auto_running = False
        if self._auto_job:
            self.root.after_cancel(self._auto_job)
            self._auto_job = None
        self.btn_auto_start.configure(state=tk.NORMAL)
        self.btn_auto_stop.configure(state=tk.DISABLED)
        self.ent_interval.configure(state=tk.NORMAL)
        self._log("[Auto] Dung auto gui.")

    def _on_close(self) -> None:
        self._stop_auto()
        if self.client:
            self.client.disconnect()
        self.root.destroy()
        sys.exit(0)


def main() -> None:
    app = MqttGuiApp()
    app.root.mainloop()


if __name__ == "__main__":
    main()
