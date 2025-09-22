#!/usr/bin/env python3
"""
Train all sensor ML models for production navigation system
"""

import os
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, models
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import glob
import json


def create_gravity_enhancement_model(input_shape=(9,)):
    """Create a model for enhancing gravity field measurements"""
    model = models.Sequential([
        layers.Dense(64, activation='relu', input_shape=input_shape),
        layers.BatchNormalization(),
        layers.Dropout(0.2),

        layers.Dense(128, activation='relu'),
        layers.BatchNormalization(),
        layers.Dropout(0.2),

        layers.Dense(256, activation='relu'),
        layers.BatchNormalization(),
        layers.Dropout(0.2),

        layers.Dense(128, activation='relu'),
        layers.BatchNormalization(),
        layers.Dropout(0.2),

        layers.Dense(9, activation='linear')  # Output enhanced tensor components
    ])

    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=0.001),
        loss='mse',
        metrics=['mae']
    )

    return model


def create_magnetometer_calibration_model(seq_length=20, features=6):
    """Create LSTM model for magnetometer calibration"""
    model = models.Sequential([
        layers.LSTM(64, return_sequences=True, input_shape=(seq_length, features)),
        layers.BatchNormalization(),
        layers.Dropout(0.3),

        layers.LSTM(32, return_sequences=False),
        layers.BatchNormalization(),
        layers.Dropout(0.3),

        layers.Dense(32, activation='relu'),
        layers.Dense(4)  # 3 calibrated values + 1 heading correction
    ])

    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=0.001),
        loss='mse',
        metrics=['mae']
    )

    return model


def create_barometer_drift_model(seq_length=30, features=3):
    """Create LSTM model for barometer drift correction"""
    model = models.Sequential([
        layers.LSTM(32, return_sequences=True, input_shape=(seq_length, features)),
        layers.BatchNormalization(),
        layers.Dropout(0.2),

        layers.LSTM(16, return_sequences=False),
        layers.BatchNormalization(),
        layers.Dropout(0.2),

        layers.Dense(16, activation='relu'),
        layers.Dense(1)  # Altitude correction
    ])

    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=0.001),
        loss='mse',
        metrics=['mae']
    )

    return model


def train_gravity_model():
    """Train gravity enhancement model with synthetic data"""
    print("\n" + "="*50)
    print("TRAINING GRAVITY ENHANCEMENT MODEL")
    print("="*50)

    # Generate synthetic training data
    # In production, use real gravity measurements
    n_samples = 10000

    # Input: Low-resolution gravity tensor (9 components)
    X = np.random.randn(n_samples, 9) * 10  # Scale to realistic values

    # Target: High-resolution gravity tensor (with added detail)
    y = X + np.random.randn(n_samples, 9) * 0.5  # Add fine details

    # Split data
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    # Create and train model
    model = create_gravity_enhancement_model()

    history = model.fit(
        X_train, y_train,
        validation_split=0.2,
        epochs=30,
        batch_size=32,
        verbose=1,
        callbacks=[
            keras.callbacks.EarlyStopping(patience=5, restore_best_weights=True),
            keras.callbacks.ReduceLROnPlateau(patience=3, factor=0.5)
        ]
    )

    # Evaluate
    test_loss, test_mae = model.evaluate(X_test, y_test, verbose=0)
    print(f"\nTest Loss: {test_loss:.4f}")
    print(f"Test MAE: {test_mae:.4f}")

    # Save model
    os.makedirs("models", exist_ok=True)
    model.save("models/gravity_enhancer.h5")
    print("✓ Gravity enhancement model saved")

    return model


def train_magnetometer_model():
    """Train magnetometer calibration model"""
    print("\n" + "="*50)
    print("TRAINING MAGNETOMETER CALIBRATION MODEL")
    print("="*50)

    # Generate synthetic training data
    n_sequences = 5000
    seq_length = 20

    # Input: Raw magnetometer + attitude (6 features)
    X = np.random.randn(n_sequences, seq_length, 6)

    # Target: Calibrated mag (3) + heading correction (1)
    y = np.random.randn(n_sequences, 4) * 0.1  # Small corrections

    # Split data
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    # Create and train model
    model = create_magnetometer_calibration_model()

    history = model.fit(
        X_train, y_train,
        validation_split=0.2,
        epochs=30,
        batch_size=32,
        verbose=1,
        callbacks=[
            keras.callbacks.EarlyStopping(patience=5, restore_best_weights=True),
            keras.callbacks.ReduceLROnPlateau(patience=3, factor=0.5)
        ]
    )

    # Evaluate
    test_loss, test_mae = model.evaluate(X_test, y_test, verbose=0)
    print(f"\nTest Loss: {test_loss:.4f}")
    print(f"Test MAE: {test_mae:.4f}")

    # Save model
    model.save("models/magnetometer_calibrator.h5")
    print("✓ Magnetometer calibration model saved")

    return model


def train_barometer_model():
    """Train barometer drift correction model"""
    print("\n" + "="*50)
    print("TRAINING BAROMETER DRIFT MODEL")
    print("="*50)

    # Generate synthetic training data
    n_sequences = 5000
    seq_length = 30

    # Input: Pressure, temperature, altitude (3 features)
    X = np.random.randn(n_sequences, seq_length, 3)

    # Target: Altitude correction
    y = np.random.randn(n_sequences, 1) * 2  # Drift corrections in meters

    # Split data
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    # Create and train model
    model = create_barometer_drift_model()

    history = model.fit(
        X_train, y_train,
        validation_split=0.2,
        epochs=30,
        batch_size=32,
        verbose=1,
        callbacks=[
            keras.callbacks.EarlyStopping(patience=5, restore_best_weights=True),
            keras.callbacks.ReduceLROnPlateau(patience=3, factor=0.5)
        ]
    )

    # Evaluate
    test_loss, test_mae = model.evaluate(X_test, y_test, verbose=0)
    print(f"\nTest Loss: {test_loss:.4f}")
    print(f"Test MAE: {test_mae:.4f}")

    # Save model
    model.save("models/barometer_corrector.h5")
    print("✓ Barometer drift model saved")

    return model


def main():
    """Train all models"""
    print("\n" + "="*60)
    print("TRAINING ALL SENSOR ML MODELS")
    print("="*60)

    # Check if we already have the IMU model
    if os.path.exists("models/lstm_imu_corrector.h5"):
        print("\n✓ IMU correction model already exists")
    else:
        print("\n! IMU model not found - run train_lstm_imu.py first")

    # Train other models
    models_trained = []

    try:
        train_gravity_model()
        models_trained.append("gravity_enhancer")
    except Exception as e:
        print(f"Failed to train gravity model: {e}")

    try:
        train_magnetometer_model()
        models_trained.append("magnetometer_calibrator")
    except Exception as e:
        print(f"Failed to train magnetometer model: {e}")

    try:
        train_barometer_model()
        models_trained.append("barometer_corrector")
    except Exception as e:
        print(f"Failed to train barometer model: {e}")

    # Save training metadata
    metadata = {
        "models_trained": models_trained,
        "training_date": pd.Timestamp.now().isoformat(),
        "framework": "tensorflow",
        "version": tf.__version__
    }

    with open("models/training_metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)

    print("\n" + "="*60)
    print(f"TRAINING COMPLETE - {len(models_trained)} models trained")
    print("Models saved to: models/")
    print("="*60)


if __name__ == "__main__":
    main()