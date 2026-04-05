# SafeShift — Vehicle Safety & Maintenance Dashboard

A student engineering competition project simulating a retrofit IoT safety
system for industrial excavators. Real-time sensor simulation with an embedded
XGBoost failure prediction model.

---

## Project structure

```
SafeShift/
├── server.py            # Python WebSocket simulation backend
├── dashboard.html       # Single-file frontend (open directly in browser)
├── retrain.py           # Model retraining script
├── xgboost_model.pkl    # Trained XGBoost failure prediction model
├── ai4i_fe.csv          # Training dataset (AI4I 2020 Predictive Maintenance)
├── requirements.txt     # Python dependencies
└── README.md
```

---

## Quick start

### 1. Create a virtual environment and install dependencies

```bash
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Start the server

```bash
python server.py
```

You should see:

```
====================================================
  SafeShift server running on ws://localhost:8765
  Open dashboard.html in your browser
  Press Ctrl+C to stop
====================================================
[AI] XGBoost failure model loaded successfully.
```

### 3. Open the dashboard

Open `dashboard.html` directly in your browser (no web server needed).

---

## Architecture

```
dashboard.html  ──WebSocket──▶  server.py
   (browser)    ◀──JSON 500ms──  (Python asyncio)
                                     │
                                     ├─ Sensor simulation classes
                                     ├─ ESP32Controller (safety logic)
                                     └─ FailurePredictor (XGBoost)
```

### Backend (`server.py`)

Simulates a full sensor suite and broadcasts a JSON state object to all
connected clients every 500 ms:

| Sensor class         | Simulates                                    |
|----------------------|----------------------------------------------|
| `HydraulicPressureSensor` | 0–400 bar with noise + spike suppression |
| `TemperatureSensor`  | Thermal lag model, scales with RPM           |
| `VibrationSensor`    | RPM-dependent + optional fault injection     |
| `RuntimeCounter`     | Service hours meter (accelerated for demo)   |

The `ESP32Controller` class enforces safety interlocks:
- Relay opens if door is open, seatbelt is off, or pressure exceeds 280 bar
- Alerts are debounced (same alert cannot repeat within 3 seconds)
- Alert log capped at 50 entries, newest first

### Frontend (`dashboard.html`)

- Vanilla JS + CSS, no build step, no npm
- Chart.js (CDN) for the temperature sparkline only
- WebSocket auto-reconnects every 2 seconds if the server drops

---

## AI Failure Prediction

The XGBoost model predicts machine failure probability using only the three
inputs available from the dashboard:

| Dashboard data         | Model feature            |
|------------------------|--------------------------|
| Air temperature (°C)   | Air temperature (K)      |
| Motor RPM              | Rotational speed (rpm)   |
| Service hours          | Tool wear (min, scaled)  |

Two derived features are computed on the fly:
- `omega` = rpm × 2π / 60
- `pct_tool_life` = tool_wear / 220

**Model performance** (test set, 20% holdout, stratified):

| Metric     | Value |
|------------|-------|
| ROC-AUC    | 0.92  |
| PR-AUC     | 0.33  |
| Recall (failure class) | 0.52 |

> PR-AUC is the most meaningful metric here given the 97/3 class imbalance.
> The model is weighted with `scale_pos_weight` to improve minority class recall.

### Retraining

To retrain the model from the raw dataset:

```bash
python retrain.py
```

This will clean the data (drops 27 inconsistent label rows), re-engineer
features, retrain with the same hyperparameters, print evaluation metrics,
and overwrite `xgboost_model.pkl`.

---

## Operator controls

| Control              | Effect                                              |
|----------------------|-----------------------------------------------------|
| Hydraulic pressure   | Sets pressure set-point (0–400 bar)                 |
| Motor RPM            | Sets motor speed (0–2500 rpm)                       |
| Air temperature      | Overrides the thermal simulation (20–130 °C)        |
| Service hours        | Sets the runtime counter (0–500 h)                  |
| Door open toggle     | Opens the cab door → triggers interlock             |
| Seatbelt removed     | Removes seatbelt → triggers interlock               |
| Inject vibration fault | Adds a 2.5 g spike to the vibration sensor       |
| Connect Bluetooth    | UI indicator only (no real connection)              |
| Internet             | UI indicator only (no real connection)              |

---

## Dependencies

| Package      | Version  | Purpose                        |
|--------------|----------|--------------------------------|
| websockets   | ≥ 16.0   | Async WebSocket server         |
| xgboost      | ≥ 3.2.0  | Failure prediction model       |
| scikit-learn | ≥ 1.7.0  | Train/test split, metrics      |
| numpy        | ≥ 2.2.0  | Numerical computation          |
| pandas       | ≥ 2.3.0  | Feature DataFrame construction |
