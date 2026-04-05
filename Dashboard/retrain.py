"""
RetroGuard — Model Retraining Script
=====================================
Cleans the AI4I dataset and retrains an XGBoost classifier using only
the features that the RetroGuard dashboard can actually provide:

    Dashboard source          → Model feature
    ──────────────────────    ────────────────────────────────────
    temp_c  (air temp sensor) → Air temperature [K]  (+ 273.15)
    rpm     (motor speed)     → Rotational speed [rpm]
    hours_since_service       → Tool wear [min]  (scaled proxy)

Derived features (fully computable from the above three):
    omega         = rpm × 2π / 60
    pct_tool_life = tool_wear / 220

DROPPED features (not available on dashboard at runtime):
    Torque [Nm] and all torque-derived columns  (wear_x_torque,
    rpm_x_torque, torque_sq, log_torque, power, power_kW)
    Process temperature [K]  (always ≈ air_temp + 10 K, adds no info)
    delta_Temp               (constant ~10 K, adds no info)
    Type                     (Pearson r = 0.005 with failure)

Run:
    python retrain.py
"""

import os
import pickle
import warnings

import numpy as np
import pandas as pd
from sklearn.metrics import (
    average_precision_score,
    classification_report,
    confusion_matrix,
    roc_auc_score,
)
from sklearn.model_selection import StratifiedKFold, cross_val_score, train_test_split
from xgboost import XGBClassifier

warnings.filterwarnings("ignore")

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH   = os.path.join(SCRIPT_DIR, "ai4i_fe.csv")
OUT_MODEL  = os.path.join(SCRIPT_DIR, "xgboost_model.pkl")

# ── Feature columns (what the new model uses) ─────────────────────────────────
FEATURE_COLS = [
    "Air temperature K",
    "Rotational speed rpm",
    "Tool wear min",
    "omega",
    "pct_tool_life",
]
TARGET = "Machine failure"

# ─────────────────────────────────────────────────────────────────────────────
print("=" * 60)
print("  RetroGuard — XGBoost Retraining")
print("=" * 60)

# ── 1. Load data ──────────────────────────────────────────────────────────────
print("\n[1] Loading data …")
df = pd.read_csv(CSV_PATH)
print(f"    Loaded {len(df):,} rows, {df.shape[1]} columns")

# ── 2. Clean: drop rows where failure sub-type label contradicts Machine failure ──
print("\n[2] Cleaning label inconsistencies …")
subtype_any = df[["TWF", "HDF", "PWF", "OSF", "RNF"]].any(axis=1).astype(int)
inconsistent = (subtype_any != df[TARGET])
n_dropped = inconsistent.sum()
df = df[~inconsistent].reset_index(drop=True)
print(f"    Dropped {n_dropped} inconsistent rows → {len(df):,} remain")
print(f"    Failure rate: {df[TARGET].mean():.4f}  "
      f"({df[TARGET].sum()} failures / {len(df):,} total)")

# ── 3. Feature engineering (dashboard-available only) ─────────────────────────
print("\n[3] Engineering features …")
df["Air temperature K"]  = df["Air temperature [K]"]
df["Rotational speed rpm"] = df["Rotational speed [rpm]"]
df["Tool wear min"]      = df["Tool wear [min]"]
df["omega"]              = df["Rotational speed rpm"] * 2 * np.pi / 60
df["pct_tool_life"]      = df["Tool wear min"] / 220.0

X = df[FEATURE_COLS].copy()
y = df[TARGET].copy()
print(f"    Feature set: {FEATURE_COLS}")
print(f"    X shape: {X.shape}")

# ── 4. Train / test split (stratified) ───────────────────────────────────────
print("\n[4] Splitting data …")
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42, stratify=y
)
n_neg = (y_train == 0).sum()
n_pos = (y_train == 1).sum()
spw   = round(n_neg / n_pos, 1)
print(f"    Train: {len(X_train):,}  (neg={n_neg}, pos={n_pos})")
print(f"    Test:  {len(X_test):,}")
print(f"    scale_pos_weight = {spw}")

# ── 5. Train XGBoost with class-imbalance weighting ──────────────────────────
print("\n[5] Training XGBoost …")
model = XGBClassifier(
    n_estimators       = 600,
    max_depth          = 5,
    learning_rate      = 0.05,
    subsample          = 0.8,
    colsample_bytree   = 0.8,
    scale_pos_weight   = spw,       # handles 97/3 imbalance
    min_child_weight   = 3,
    gamma              = 0.1,
    reg_alpha          = 0.05,
    reg_lambda         = 1.0,
    use_label_encoder  = False,
    eval_metric        = "aucpr",   # optimise for precision-recall AUC
    random_state       = 42,
    verbosity          = 0,
)
model.fit(X_train, y_train)
print("    Done.")

# ── 6. Evaluate ───────────────────────────────────────────────────────────────
print("\n[6] Evaluating …")
y_pred = model.predict(X_test)
y_prob = model.predict_proba(X_test)[:, 1]

roc  = roc_auc_score(y_test, y_prob)
prauc = average_precision_score(y_test, y_prob)

print(f"\n    ROC-AUC : {roc:.4f}")
print(f"    PR-AUC  : {prauc:.4f}")
print()
print(classification_report(y_test, y_pred, target_names=["No Failure", "Failure"]))
print("    Confusion matrix (rows=actual, cols=predicted):")
cm = confusion_matrix(y_test, y_pred)
print(f"      TN={cm[0,0]:4d}  FP={cm[0,1]:4d}")
print(f"      FN={cm[1,0]:4d}  TP={cm[1,1]:4d}")

# 5-fold cross-validation on full dataset
print("\n    5-fold cross-val (PR-AUC) …")
cv_model = XGBClassifier(
    n_estimators=600, max_depth=5, learning_rate=0.05,
    subsample=0.8, colsample_bytree=0.8, scale_pos_weight=spw,
    min_child_weight=3, gamma=0.1, reg_alpha=0.05, reg_lambda=1.0,
    use_label_encoder=False, eval_metric="aucpr", random_state=42, verbosity=0,
)
cv_scores = cross_val_score(
    cv_model, X, y,
    cv=StratifiedKFold(n_splits=5, shuffle=True, random_state=42),
    scoring="average_precision",
)
print(f"    PR-AUC per fold: {[round(s,4) for s in cv_scores]}")
print(f"    Mean ± Std:  {cv_scores.mean():.4f} ± {cv_scores.std():.4f}")

# Feature importances
print("\n    Feature importances (gain):")
imp = dict(zip(FEATURE_COLS, model.feature_importances_))
for feat, score in sorted(imp.items(), key=lambda x: -x[1]):
    bar = "█" * int(score * 40)
    print(f"      {feat:<25s} {score:.4f}  {bar}")

# ── 7. Save model ─────────────────────────────────────────────────────────────
print(f"\n[7] Saving model to {OUT_MODEL} …")
with open(OUT_MODEL, "wb") as f:
    pickle.dump(model, f)
print("    Saved.")

print("\n" + "=" * 60)
print("  Retraining complete.")
print("=" * 60)
