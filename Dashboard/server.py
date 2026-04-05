"""
SafeShift Vehicle Safety & Maintenance Simulation Server
=========================================================
WebSocket backend for the Safeshift dashboard.

Simulates a full ESP32-based IoT retrofit safety system for an industrial
excavator, broadcasting sensor data to connected clients at 500 ms intervals.

Run with:
    pip install websockets
    python server.py
"""

import asyncio
import json
import math
import os
import pickle
import random
import time
from collections import deque
from datetime import datetime

import numpy as np
import pandas as pd

try:
    import websockets
except ImportError:
    print("Missing dependency — run: pip install websockets")
    raise


# ══════════════════════════════════════════════════════════════════════════════
#  Sensor Simulation Classes
# ══════════════════════════════════════════════════════════════════════════════

class HydraulicPressureSensor:
    """
    Simulates a hydraulic pressure transducer (0–400 bar range).

    Behaviour:
      - Set-point is provided by the UI slider via set_pressure().
      - Gaussian noise of ±2 bar is added to each reading.
      - 800 ms spike suppression: immediately after a set-point change the raw
        (clean) value is used to avoid simulating hydraulic surge artefacts.
      - A 10-sample rolling average smooths the noisy signal.
    """
    MAX_PRESSURE_BAR = 280

    def __init__(self):
        self.pressure_bar      = 180.0        # current set-point (UI)
        self._last_set         = 180.0        # previous set-point (change detection)
        self._change_time      = 0.0          # wall-clock time of last change
        self._history          = deque(maxlen=10)  # rolling average buffer

    def set_pressure(self, value: float):
        """Update the pressure set-point from the UI slider."""
        value = max(0.0, min(400.0, float(value)))
        if abs(value - self._last_set) > 1.0:
            self._change_time = time.time()
            self._last_set    = value
        self.pressure_bar = value

    def read(self) -> dict:
        """Return a noisy, averaged pressure reading with status classification."""
        now = time.time()
        # Suppress noise for 800 ms after a set-point change (hydraulic surge)
        if now - self._change_time < 0.8:
            raw = self.pressure_bar
        else:
            raw = self.pressure_bar + random.gauss(0, 2)

        self._history.append(raw)
        averaged = sum(self._history) / len(self._history)
        averaged = max(0.0, min(400.0, averaged))

        pct = (averaged / self.MAX_PRESSURE_BAR) * 100.0
        if averaged >= self.MAX_PRESSURE_BAR:
            status = "CRITICAL"
        elif averaged >= self.MAX_PRESSURE_BAR * 0.85:   # 85 % = 238 bar
            status = "WARNING"
        else:
            status = "OK"

        return {
            "pressure_bar":    round(averaged, 1),
            "pressure_pct":    round(pct, 1),
            "pressure_status": status,
        }


class TemperatureSensor:
    """
    Simulates an engine oil / coolant temperature sensor.

    Behaviour:
      - Idle base temperature: 75 °C.
      - Target temperature scales linearly with RPM: 75 + (rpm/2500) × 40.
      - Thermal lag: base tracks the target with a 5 % per-tick exponential
        approach (realistic slow warm-up / cool-down).
      - Gaussian noise of ±0.5 °C per reading.
    """
    MAX_TEMP_C  = 105
    IDLE_TEMP_C = 75.0

    def __init__(self):
        self._base_temp  = self.IDLE_TEMP_C
        self.rpm         = 0.0    # kept in sync by SimState
        self._override   = None   # manual set-point from UI slider (°C)

    def set_temp(self, value: float):
        """Pin temperature to a manual set-point from the UI slider."""
        self._override = max(20.0, min(140.0, float(value)))
        self._base_temp = self._override   # jump immediately, no lag

    def read(self) -> dict:
        """Return a thermally-lagged, noisy temperature reading."""
        if self._override is not None:
            target = self._override
        else:
            target = self.IDLE_TEMP_C + (self.rpm / 2500.0) * 40.0
        self._base_temp += (target - self._base_temp) * 0.05   # lag
        noisy = self._base_temp + random.gauss(0, 0.5)
        noisy = max(20.0, min(140.0, noisy))

        if noisy >= self.MAX_TEMP_C:
            status = "CRITICAL"
        elif noisy >= 90.0:
            status = "WARNING"
        else:
            status = "OK"

        return {"temp_c": round(noisy, 1), "temp_status": status}


class VibrationSensor:
    """
    Simulates a 3-axis MEMS accelerometer / vibration sensor.

    Behaviour:
      - Normal vibration scales with RPM: 0.1 + (rpm/2500) × 0.4 g.
      - Fault mode adds a 2.5 g spike (simulates bearing failure).
      - Gaussian noise of ±0.05 g per reading.
    """
    MAX_VIBRATION_G = 1.8

    def __init__(self):
        self.rpm            = 0.0
        self.fault_injected = False

    def read(self) -> dict:
        """Return a noisy vibration reading; adds fault spike when enabled."""
        base = 0.1 + (self.rpm / 2500.0) * 0.4
        if self.fault_injected:
            base += 2.5
        noisy = max(0.0, base + random.gauss(0, 0.05))

        if noisy >= self.MAX_VIBRATION_G:
            status = "CRITICAL"
        elif noisy >= 1.2:
            status = "WARNING"
        else:
            status = "OK"

        return {"vibration_g": round(noisy, 2), "vibration_status": status}


class BlindSpotSensor:
    """
    Simulates five HC-SR04 / JSN-SR04 ultrasonic proximity sensors arranged
    around the vehicle for blind-spot detection.

    Zones (matching excavator exclusion-zone rings):
      CRITICAL : 0 – 150 cm   (inner red ring)
      WARNING  : 150 – 300 cm (middle amber ring)
      ADVISORY : 300 – 500 cm (outer green ring)
      CLEAR    : > 500 cm
    """
    SENSORS = ["rear_left", "rear_center", "rear_right", "side_left", "side_right"]

    def __init__(self):
        self._distances = {s: 500.0 for s in self.SENSORS}

    def set_distance(self, sensor: str, value: float):
        """Update a single sensor's simulated distance (from the UI slider)."""
        if sensor in self._distances:
            self._distances[sensor] = max(0.0, min(500.0, float(value)))

    @staticmethod
    def _zone(d: float) -> str:
        if d <= 150:  return "CRITICAL"
        if d <= 300:  return "WARNING"
        if d <= 500:  return "ADVISORY"
        return "CLEAR"

    def read(self) -> dict:
        """Return per-sensor distance + zone, with small ultrasonic noise."""
        result = {}
        for s in self.SENSORS:
            d = max(0.0, min(500.0, self._distances[s] + random.gauss(0, 2)))
            result[s] = {"distance_cm": round(d, 0), "zone": self._zone(d)}
        return result


class ThermalCamera:
    """
    Simulates a Melexis AMG8833 8×8 (64-pixel) thermal imaging sensor.

    Scenarios (set by UI or pedestrian detection toggle):
      'clear'      — ambient noise ~22 °C throughout grid
      'human'      — a warm blob (28–34 °C) covering 4–12 contiguous pixels,
                     regenerated every ~10 s for realism
      'hot_object' — large central region >50 °C (>20 pixels)
      'bag'        — ambient temperature only, no contrast (not yet wired to UI)
    """

    GRID_W = 8
    GRID_H = 8

    def __init__(self):
        self.scenario       = "clear"
        self._blob_pixels: set = set()
        self._blob_temp     = 31.0
        self._regen_blob()

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _regen_blob(self):
        """Re-randomise blob position, size (4–12 px) and temperature."""
        count    = random.randint(4, 12)
        start_r  = random.randint(0, self.GRID_H - 3)
        start_c  = random.randint(0, self.GRID_W - 3)
        pixels   = {(start_r, start_c)}
        attempts = 0
        while len(pixels) < count and attempts < 120:
            r, c   = random.choice(list(pixels))
            dr, dc = random.choice([(-1,0),(1,0),(0,-1),(0,1)])
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.GRID_H and 0 <= nc < self.GRID_W:
                pixels.add((nr, nc))
            attempts += 1
        self._blob_pixels = pixels
        self._blob_temp   = random.uniform(28.0, 34.0)

    # ── Public interface ──────────────────────────────────────────────────────

    def read(self) -> list:
        """Return a flat 64-element list of pixel temperatures (°C)."""
        grid = []
        for row in range(self.GRID_H):
            for col in range(self.GRID_W):
                ambient = random.gauss(22.0, 0.8)
                if self.scenario == "human" and (row, col) in self._blob_pixels:
                    val = self._blob_temp + random.gauss(0, 0.5)
                elif self.scenario == "hot_object":
                    val = random.uniform(50.0, 65.0) if 1 <= row <= 6 and 1 <= col <= 6 else ambient
                else:
                    val = ambient
                grid.append(round(val, 1))
        return grid

    def human_detected(self, grid: list) -> bool:
        """True when 4–20 pixels fall in the human body-temperature band (28–40 °C)."""
        warm = sum(1 for v in grid if 28.0 <= v <= 40.0)
        return 4 <= warm <= 20


class RCWL0516:
    """
    Simulates a RCWL-0516 Doppler microwave radar motion sensor.

    The output is a simple boolean controlled by the UI toggle.
    It gates the thermal human-detection pipeline — thermal analysis only runs
    when the RCWL has first detected movement in the exclusion zone.
    """
    def __init__(self):
        self.motion_detected = False


class MicroswitchSensor:
    """
    Simulates a normally-open microswitch wired as a seatbelt buckle sensor.
    Returns True when the buckle is engaged (belt is buckled).
    """
    def __init__(self):
        self.buckled = True


class ReedSwitch:
    """
    Simulates a reed switch mounted on the cab door frame.
    Returns True when the door is closed (magnet aligns with switch).
    """
    def __init__(self):
        self.closed = True


class RuntimeCounter:
    """
    Simulates a vehicle hours meter.

    Each 500 ms broadcast tick advances by TICK_HRS simulated hours so the
    service-due countdown is visible during a demo without waiting days.
    """
    SERVICE_INTERVAL_HRS = 250
    TICK_HRS             = 0.01  # simulated hrs per 500 ms tick

    def __init__(self):
        self.runtime_hours = 232.0   # start close to service threshold for demo

    def tick(self):
        self.runtime_hours += self.TICK_HRS

    def read(self) -> dict:
        """Return runtime hours and time-to-next-service with status."""
        hours_since = self.runtime_hours % self.SERVICE_INTERVAL_HRS
        hours_to    = self.SERVICE_INTERVAL_HRS - hours_since

        if hours_to <= 0:
            service_status = "OVERDUE"
        elif hours_to <= 20:
            service_status = "DUE_SOON"
        else:
            service_status = "OK"

        return {
            "runtime_hrs":          round(self.runtime_hours, 1),
            "service_interval_hrs": self.SERVICE_INTERVAL_HRS,
            "hours_to_service":     round(hours_to, 1),
            "hours_since_service":  round(hours_since, 1),
            "service_status":       service_status,
        }


# ══════════════════════════════════════════════════════════════════════════════
#  ESP32 Controller Logic
# ══════════════════════════════════════════════════════════════════════════════

class ESP32Controller:
    """
    Emulates the embedded ESP32 firmware that aggregates sensor readings,
    enforces safety interlocks, and maintains the alert log.

    Relay logic (relay = True means the vehicle IS operable):
        relay = door.closed
            AND belt.buckled
            AND pressure < MAX_PRESSURE_BAR
            AND NOT pedestrian_confirmed

    Pedestrian confirmation (two-factor):
        pedestrian_confirmed = rcwl.motion_detected AND thermal.human_detected()

    Alert debouncing: the same alert key cannot fire more than once every
    ALERT_DEBOUNCE_S seconds to avoid log flooding.
    """
    MAX_PRESSURE_BAR    = 280
    MAX_TEMP_C          = 105
    MAX_VIBRATION_G     = 1.8
    SERVICE_WARNING_HRS = 20

    ALERT_DEBOUNCE_S = 3.0
    MAX_ALERTS       = 50

    def __init__(self):
        self._alert_log:      list = []
        self._last_alert_ts:  dict = {}   # debounce key → last fire time
        self._alerts_today:   int  = 0

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _alert(self, level: str, message: str, detail: str):
        """Append alert to log with debounce; newest entries at front."""
        key = f"{level}:{message}"
        now = time.time()
        if now - self._last_alert_ts.get(key, 0) < self.ALERT_DEBOUNCE_S:
            return   # debounced
        self._last_alert_ts[key] = now
        self._alerts_today += 1
        entry = {
            "level":     level,
            "message":   message,
            "detail":    detail,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
        }
        self._alert_log.insert(0, entry)
        # Trim to max length
        if len(self._alert_log) > self.MAX_ALERTS:
            self._alert_log = self._alert_log[:self.MAX_ALERTS]

    # ── Public interface ──────────────────────────────────────────────────────

    def process(self, readings: dict) -> dict:
        """
        Evaluate all sensor readings and return relay state, interlock reasons,
        the current alert log, and today's alert count.
        """
        safety = readings["safety"]
        maint  = readings["maintenance"]

        relay             = True
        interlock_reasons = []

        # ── Door interlock ──────────────────────────────────────────────────
        if not safety["door_closed"]:
            relay = False
            interlock_reasons.append("Cab door open")
            self._alert("CRITICAL", "Door open", "Cab door not secured")

        # ── Seatbelt interlock ──────────────────────────────────────────────
        if not safety["belt_buckled"]:
            relay = False
            interlock_reasons.append("Seatbelt not buckled")
            self._alert("CRITICAL", "Seatbelt", "Operator seatbelt unbuckled")

        # ── Hydraulic pressure ──────────────────────────────────────────────
        p = safety["pressure_bar"]
        if p >= self.MAX_PRESSURE_BAR:
            relay = False
            interlock_reasons.append(f"Pressure overload ({p:.0f} bar)")
            self._alert("CRITICAL", "Overload", f"{p:.0f} bar")
        elif safety["pressure_status"] == "WARNING":
            self._alert("WARNING", "High pressure", f"{p:.0f} bar")

        # ── Pedestrian / exclusion zone ─────────────────────────────────────
        if safety["pedestrian_confirmed"]:
            relay = False
            interlock_reasons.append("Pedestrian in exclusion zone")
            self._alert("CRITICAL", "Pedestrian detected",
                        "Human presence confirmed via thermal + motion")
        elif safety["rcwl_motion"]:
            self._alert("WARNING", "Motion detected", "Awaiting thermal confirmation")

        # ── Engine temperature ──────────────────────────────────────────────
        t = maint["temp_c"]
        if t >= self.MAX_TEMP_C:
            self._alert("CRITICAL", "Overtemperature", f"{t:.1f} °C")
        elif t >= 90.0:
            self._alert("WARNING", "High temperature", f"{t:.1f} °C")

        # ── Vibration ───────────────────────────────────────────────────────
        v = maint["vibration_g"]
        if v >= self.MAX_VIBRATION_G:
            self._alert("CRITICAL", "Excessive vibration", f"{v:.2f} g")
        elif v >= 1.2:
            self._alert("WARNING", "High vibration", f"{v:.2f} g")

        # ── Fault injection indicator ───────────────────────────────────────
        if maint.get("fault_injected"):
            self._alert("WARNING", "Fault injected", "Vibration fault active")

        # ── Service interval ────────────────────────────────────────────────
        h = maint["hours_to_service"]
        if h <= 0:
            self._alert("WARNING", "Service overdue",
                        f"{abs(h):.1f} h past interval")
        elif h <= self.SERVICE_WARNING_HRS:
            self._alert("INFO", "Service due soon", f"{h:.1f} h remaining")

        # ── Blind spot proximity ────────────────────────────────────────────
        for sensor, data in safety["blind_spots"].items():
            label = sensor.replace("_", " ").title()
            if data["zone"] == "CRITICAL":
                self._alert("CRITICAL", f"Blind spot – {label}",
                            f"{data['distance_cm']:.0f} cm")
            elif data["zone"] == "WARNING":
                self._alert("WARNING", f"Proximity – {label}",
                            f"{data['distance_cm']:.0f} cm")

        return {
            "relay":            relay,
            "interlock_reasons": interlock_reasons,
            "alerts":           self._alert_log,
            "alerts_today":     self._alerts_today,
        }


# ══════════════════════════════════════════════════════════════════════════════
#  XGBoost Failure Predictor
# ══════════════════════════════════════════════════════════════════════════════

class FailurePredictor:
    """
    Wraps the retrained XGBoost machine-failure classifier.

    All features are derived exclusively from data the dashboard provides:

      Dashboard source            → Model feature
      ─────────────────────────   ─────────────────────────────────────
      temp_c  (air temp sensor)   → Air temperature K  (temp_c + 273.15)
      rpm     (motor speed)       → Rotational speed rpm
      hours_since_service         → Tool wear min  (scaled 0-250 h → 0-253 min)

    Derived (computed, no extra sensors needed):
      omega         = rpm × 2π / 60
      pct_tool_life = tool_wear_min / 220

    Torque, Process temperature, and Type are NOT used in this model —
    they were dropped during retraining because the dashboard cannot
    provide real-time values for them.
    """

    MAX_TOOL_WEAR_MIN  = 253.0   # training data max
    SERVICE_INTERVAL_H = 250.0   # matches RuntimeCounter

    FEATURE_COLS = [
        "Air temperature K",
        "Rotational speed rpm",
        "Tool wear min",
        "omega",
        "pct_tool_life",
    ]

    def __init__(self):
        self._model = None
        model_path = os.path.join(
            os.path.dirname(__file__), "xgboost_model.pkl"
        )
        try:
            with open(model_path, "rb") as f:
                self._model = pickle.load(f)
            print("[AI] XGBoost failure model loaded successfully.")
        except FileNotFoundError:
            print(f"[!] Model not found at {model_path} — prediction disabled.")
        except Exception as exc:
            print(f"[!] Failed to load model: {exc} — prediction disabled.")

    def predict(self, rpm: float, temp_c: float, hours_since_svc: float) -> dict:
        """
        Run inference and return failure probability + binary prediction.
        Returns a safe fallback dict if the model could not be loaded.
        """
        if self._model is None:
            return {"failure_probability": 0.0, "failure_predicted": False, "available": False}

        try:
            # Dashboard temp_c is the AIR temperature sensor reading
            air_temp_k    = temp_c + 273.15
            tool_wear     = (hours_since_svc / self.SERVICE_INTERVAL_H) * self.MAX_TOOL_WEAR_MIN
            omega         = rpm * 2 * np.pi / 60
            pct_tool_life = tool_wear / 220.0

            features = pd.DataFrame([[
                air_temp_k, rpm, tool_wear, omega, pct_tool_life
            ]], columns=self.FEATURE_COLS)

            proba     = self._model.predict_proba(features)[0]
            predicted = bool(self._model.predict(features)[0])

            return {
                "failure_probability": round(float(proba[1]), 4),
                "failure_predicted":   predicted,
                "available":           True,
            }
        except Exception as exc:
            print(f"[!] Prediction error: {exc}")
            return {"failure_probability": 0.0, "failure_predicted": False, "available": False}


# ══════════════════════════════════════════════════════════════════════════════
#  Central Simulation State
# ══════════════════════════════════════════════════════════════════════════════

class SimState:
    """
    Aggregates all sensor instances and processes control messages from the UI.
    On each tick() it reads every sensor, runs the ESP32 controller logic, and
    returns the full JSON state object for broadcast.
    """

    def __init__(self):
        # Sensor instances
        self.pressure_sensor = HydraulicPressureSensor()
        self.temp_sensor     = TemperatureSensor()
        self.vibration_sensor= VibrationSensor()
        self.blind_spot      = BlindSpotSensor()
        self.thermal_cam     = ThermalCamera()
        self.rcwl            = RCWL0516()
        self.microswitch     = MicroswitchSensor()
        self.reed_switch     = ReedSwitch()
        self.runtime         = RuntimeCounter()
        self.esp32           = ESP32Controller()
        self.predictor       = FailurePredictor()

        # Shared state
        self.rpm             = 1800.0
        self.fault_injected  = False
        self._start_time     = time.time()
        self._blob_tick      = 0   # blob regen counter

        # Sync RPM with sensors immediately
        self._sync_rpm()

    def _sync_rpm(self):
        """Push current RPM set-point to sensors that depend on it."""
        self.temp_sensor.rpm      = self.rpm
        self.vibration_sensor.rpm = self.rpm

    # ── Control message handler ───────────────────────────────────────────────

    def handle_control(self, msg: dict):
        """Dispatch a UI control message to the appropriate sensor/state."""
        t = msg.get("type")
        v = msg.get("value")

        if t == "set_pressure":
            self.pressure_sensor.set_pressure(float(v))

        elif t == "set_rpm":
            self.rpm = max(0.0, min(2500.0, float(v)))
            self._sync_rpm()

        elif t == "set_temp":
            self.temp_sensor.set_temp(float(v))

        elif t == "set_service_hours":
            self.runtime.runtime_hours = max(0.0, min(500.0, float(v)))

        elif t == "toggle_door":
            # value=True means "door open" toggle is ON → door is open
            self.reed_switch.closed = not bool(v)

        elif t == "toggle_belt":
            # value=True means "seatbelt removed" toggle is ON → not buckled
            self.microswitch.buckled = not bool(v)

        elif t == "toggle_pedestrian":
            is_on = bool(v)
            self.thermal_cam.scenario = "human" if is_on else "clear"
            if is_on:
                # Auto-enable RCWL when pedestrian scenario is triggered
                self.rcwl.motion_detected = True

        elif t == "toggle_rcwl":
            self.rcwl.motion_detected = bool(v)

        elif t == "inject_fault":
            self.fault_injected                  = bool(v)
            self.vibration_sensor.fault_injected = self.fault_injected

        elif t == "set_blind_spot":
            sensor = msg.get("sensor", "")
            self.blind_spot.set_distance(sensor, float(v))

    # ── Main tick ─────────────────────────────────────────────────────────────

    def tick(self) -> dict:
        """Advance simulation by one 500 ms step and return full broadcast state."""
        self.runtime.tick()

        # Regenerate thermal blob roughly every 10 s (20 ticks × 500 ms)
        self._blob_tick += 1
        if self._blob_tick >= 20:
            self._blob_tick = 0
            self.thermal_cam._regen_blob()

        # ── Read sensors ──────────────────────────────────────────────────────
        p_data       = self.pressure_sensor.read()
        t_data       = self.temp_sensor.read()
        v_data       = self.vibration_sensor.read()
        bs_data      = self.blind_spot.read()
        thermal_grid = self.thermal_cam.read()
        rt_data      = self.runtime.read()

        # Pedestrian confirmation: RCWL gates thermal analysis
        pedestrian_confirmed = (
            self.rcwl.motion_detected and
            self.thermal_cam.human_detected(thermal_grid)
        )

        # ── Assemble readings dict for ESP32 processing ───────────────────────
        readings = {
            "safety": {
                **p_data,
                "door_closed":           self.reed_switch.closed,
                "belt_buckled":          self.microswitch.buckled,
                "blind_spots":           bs_data,
                "rcwl_motion":           self.rcwl.motion_detected,
                "pedestrian_confirmed":  pedestrian_confirmed,
                "thermal_grid":          thermal_grid,
            },
            "maintenance": {
                "rpm":            round(self.rpm + random.gauss(0, 10), 0),
                **t_data,
                **v_data,
                "fault_injected": self.fault_injected,
                **rt_data,
            },
        }

        esp = self.esp32.process(readings)
        uptime = int(time.time() - self._start_time)

        # Run AI failure prediction using dashboard-available data
        prediction = self.predictor.predict(
            rpm             = self.rpm,
            temp_c          = t_data["temp_c"],
            hours_since_svc = rt_data["hours_since_service"],
        )

        return {
            "relay":             esp["relay"],
            "interlock_reasons": esp["interlock_reasons"],
            "safety":            readings["safety"],
            "maintenance":       readings["maintenance"],
            "alerts":            esp["alerts"],
            "prediction":        prediction,
            "stats": {
                "uptime_seconds": uptime,
                "alerts_today":   esp["alerts_today"],
                "runtime_hrs":    rt_data["runtime_hrs"],
            },
        }


# ══════════════════════════════════════════════════════════════════════════════
#  WebSocket Server
# ══════════════════════════════════════════════════════════════════════════════

sim              = SimState()
connected_clients: set = set()


async def broadcast_loop():
    """Tick the simulation and push full state to all clients every 500 ms."""
    while True:
        state   = sim.tick()
        payload = json.dumps(state)

        if connected_clients:
            dead = set()
            for ws in list(connected_clients):
                try:
                    await ws.send(payload)
                except Exception:
                    dead.add(ws)
            connected_clients.difference_update(dead)

        await asyncio.sleep(0.5)


async def handler(websocket):
    """Manage a single WebSocket client: receive controls, stay connected."""
    addr = websocket.remote_address
    print(f"[+] Client connected:    {addr}")
    connected_clients.add(websocket)
    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
                sim.handle_control(msg)
            except json.JSONDecodeError:
                print(f"[!] Bad JSON from {addr}: {raw[:80]!r}")
            except Exception as exc:
                print(f"[!] Control error from {addr}: {exc}")
    except Exception:
        pass
    finally:
        connected_clients.discard(websocket)
        print(f"[-] Client disconnected: {addr}")


async def main():
    print("=" * 52)
    print("  SafeShift server running on ws://localhost:8765")
    print("  Open dashboard.html in your browser")
    print("  Press Ctrl+C to stop")
    print("=" * 52)

    async with websockets.serve(handler, "localhost", 8765):
        await broadcast_loop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[*] SafeShift server stopped cleanly.")
