"""
Training Script for AI-Enhanced Navigation Models

This script trains both the U-Net gravity enhancer and LSTM IMU corrector
using data collected from the navigation system.
"""

import os
import sys
import numpy as np
import pandas as pd
from pathlib import Path
import argparse
from typing import Tuple, List
import matplotlib.pyplot as plt

# Add parent directory to path
sys.path.append(str(Path(__file__).parent))

from gravity_enhancement.unet_model import GravityUNet, DataAugmentation
from error_correction.lstm_imu_corrector import (
    IMUErrorCorrectorLSTM, IMUDataProcessor
)

import tensorflow as tf
import torch
from torch.utils.data import DataLoader, TensorDataset
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler


class NavigationDataLoader:
    """Load and preprocess navigation training data"""

    def __init__(self, data_dir: str = "data"):
        self.data_dir = Path(data_dir)
        self.imu_data = None
        self.gravity_data = None
        self.state_data = None
        self.map_data = None

    def load_all_data(self):
        """Load all available training data"""

        print("Loading training data...")

        # Load IMU data
        imu_files = list(self.data_dir.glob("imu/*.csv"))
        if imu_files:
            imu_dfs = [pd.read_csv(f) for f in imu_files]
            self.imu_data = pd.concat(imu_dfs, ignore_index=True)
            print(f"Loaded {len(self.imu_data)} IMU samples")

        # Load gravity data
        gravity_files = list(self.data_dir.glob("gravity/*.csv"))
        if gravity_files:
            gravity_dfs = [pd.read_csv(f) for f in gravity_files]
            self.gravity_data = pd.concat(gravity_dfs, ignore_index=True)
            print(f"Loaded {len(self.gravity_data)} gravity samples")

        # Load state data
        state_files = list(self.data_dir.glob("states/*.csv"))
        if state_files:
            state_dfs = [pd.read_csv(f) for f in state_files]
            self.state_data = pd.concat(state_dfs, ignore_index=True)
            print(f"Loaded {len(self.state_data)} state samples")

        # Load map feature data
        map_files = list(self.data_dir.glob("maps/*.csv"))
        if map_files:
            map_dfs = [pd.read_csv(f) for f in map_files]
            self.map_data = pd.concat(map_dfs, ignore_index=True)
            print(f"Loaded {len(self.map_data)} map samples")

    def prepare_gravity_data(self, grid_size: int = 128) -> Tuple[np.ndarray, np.ndarray]:
        """
        Prepare gravity data for U-Net training

        Returns:
            low_res: Low-resolution gravity fields
            high_res: High-resolution ground truth
        """

        if self.gravity_data is None:
            raise ValueError("No gravity data loaded")

        print("Preparing gravity training data...")

        # Extract gravity tensor components
        tensor_cols = ['Txx', 'Txy', 'Txz', 'Tyx', 'Tyy', 'Tyz', 'Tzx', 'Tzy', 'Tzz']
        gravity_tensors = self.gravity_data[tensor_cols].values

        # Create synthetic low/high resolution pairs
        n_samples = len(gravity_tensors) // (grid_size * grid_size)
        low_res_data = []
        high_res_data = []

        for i in range(n_samples):
            start_idx = i * grid_size * grid_size
            end_idx = start_idx + grid_size * grid_size

            if end_idx > len(gravity_tensors):
                break

            # Reshape into grid
            tensor_grid = gravity_tensors[start_idx:end_idx].reshape(grid_size, grid_size, 9)

            # Create low-resolution version by downsampling
            low_res = tensor_grid[::2, ::2, :]  # Downsample by 2
            low_res = np.repeat(np.repeat(low_res, 2, axis=0), 2, axis=1)  # Upsample back

            # Add noise to low-res
            noise = np.random.normal(0, 0.1, low_res.shape)
            low_res = low_res + noise

            low_res_data.append(low_res)
            high_res_data.append(tensor_grid)

        return np.array(low_res_data), np.array(high_res_data)

    def prepare_imu_data(self, window_size: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        Prepare IMU data for LSTM training

        Returns:
            imu_windows: Windowed IMU measurements
            error_labels: Corresponding bias and scale errors
        """

        if self.imu_data is None:
            raise ValueError("No IMU data loaded")

        print("Preparing IMU training data...")

        # Extract relevant columns
        measured_cols = ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']
        true_cols = ['true_acc_x', 'true_acc_y', 'true_acc_z',
                     'true_gyro_x', 'true_gyro_y', 'true_gyro_z']
        bias_cols = ['bias_acc_x', 'bias_acc_y', 'bias_acc_z',
                     'bias_gyro_x', 'bias_gyro_y', 'bias_gyro_z']

        measured = self.imu_data[measured_cols].values
        true = self.imu_data[true_cols].values
        biases = self.imu_data[bias_cols].values

        # Calculate scale factors
        scale_factors = np.ones_like(biases)
        scale_factors[:, :3] = 1.0 + np.random.normal(0, 0.01, (len(biases), 3))
        scale_factors[:, 3:] = 1.0 + np.random.normal(0, 0.001, (len(biases), 3))

        # Create windows
        processor = IMUDataProcessor(window_size=window_size)
        imu_windows = processor.create_windows(measured)

        # Create corresponding error labels
        error_labels = []
        for i in range(len(imu_windows)):
            # Use the bias and scale from the last sample in the window
            window_end = i * processor.stride + window_size - 1
            if window_end < len(biases):
                bias = biases[window_end]
                scale = scale_factors[window_end]
                error_labels.append(np.concatenate([bias, scale]))

        return imu_windows, np.array(error_labels)


def train_gravity_unet(data_loader: NavigationDataLoader, epochs: int = 50):
    """Train the U-Net model for gravity enhancement"""

    print("\n" + "="*50)
    print("Training Gravity Enhancement U-Net")
    print("="*50)

    # Prepare data
    X, y = data_loader.prepare_gravity_data()

    if len(X) == 0:
        print("Insufficient gravity data for training")
        return None

    # Split data
    X_train, X_val, y_train, y_val = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    print(f"Training samples: {len(X_train)}")
    print(f"Validation samples: {len(X_val)}")

    # Create TensorFlow datasets
    train_dataset = tf.data.Dataset.from_tensor_slices((X_train, y_train))
    train_dataset = train_dataset.batch(8).prefetch(tf.data.AUTOTUNE)

    val_dataset = tf.data.Dataset.from_tensor_slices((X_val, y_val))
    val_dataset = val_dataset.batch(8).prefetch(tf.data.AUTOTUNE)

    # Initialize and compile model
    model = GravityUNet(input_shape=(128, 128, 9))
    model.compile_model(learning_rate=1e-4)

    # Train model
    history = model.train(
        train_dataset,
        val_dataset,
        epochs=epochs,
        batch_size=8
    )

    # Plot training history
    plt.figure(figsize=(12, 4))

    plt.subplot(1, 2, 1)
    plt.plot(history.history['loss'], label='Train Loss')
    plt.plot(history.history['val_loss'], label='Val Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.title('Gravity U-Net Training Loss')

    plt.subplot(1, 2, 2)
    plt.plot(history.history['mae'], label='Train MAE')
    plt.plot(history.history['val_mae'], label='Val MAE')
    plt.xlabel('Epoch')
    plt.ylabel('MAE')
    plt.legend()
    plt.title('Gravity U-Net MAE')

    plt.tight_layout()
    plt.savefig('gravity_unet_training.png')
    plt.show()

    # Export to ONNX
    model.save_onnx('models/gravity_unet.onnx')

    return model


def train_imu_lstm(data_loader: NavigationDataLoader, epochs: int = 50):
    """Train the LSTM model for IMU error correction"""

    print("\n" + "="*50)
    print("Training IMU Error Corrector LSTM")
    print("="*50)

    # Prepare data
    X, y = data_loader.prepare_imu_data(window_size=100)

    if len(X) == 0:
        print("Insufficient IMU data for training")
        return None

    # Split data
    X_train, X_val, y_train, y_val = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    print(f"Training samples: {len(X_train)}")
    print(f"Validation samples: {len(X_val)}")

    # Convert to PyTorch tensors
    X_train_tensor = torch.FloatTensor(X_train)
    y_train_tensor = torch.FloatTensor(y_train)
    X_val_tensor = torch.FloatTensor(X_val)
    y_val_tensor = torch.FloatTensor(y_val)

    # Create data loaders
    train_dataset = TensorDataset(X_train_tensor, y_train_tensor)
    val_dataset = TensorDataset(X_val_tensor, y_val_tensor)

    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)

    # Initialize model
    model = IMUErrorCorrectorLSTM(
        input_size=6,
        hidden_size=128,
        num_layers=3,
        output_size=12
    )

    # Train model
    model.train_model(
        train_loader,
        val_loader,
        epochs=epochs,
        lr=1e-3
    )

    # Export to ONNX
    os.makedirs('models', exist_ok=True)
    model.export_onnx('models/imu_corrector.onnx')

    return model


def evaluate_models(gravity_model, imu_model, data_loader):
    """Evaluate trained models on test data"""

    print("\n" + "="*50)
    print("Model Evaluation")
    print("="*50)

    # Evaluate gravity model
    if gravity_model is not None:
        X_test, y_test = data_loader.prepare_gravity_data()
        if len(X_test) > 0:
            # Take last 10% for testing
            test_start = int(0.9 * len(X_test))
            X_test = X_test[test_start:]
            y_test = y_test[test_start:]

            predictions = gravity_model.predict(X_test[:10])  # Test on 10 samples
            mse = np.mean((predictions - y_test[:10])**2)
            print(f"Gravity U-Net Test MSE: {mse:.6f}")

    # Evaluate IMU model
    if imu_model is not None:
        X_test, y_test = data_loader.prepare_imu_data()
        if len(X_test) > 0:
            # Take last 10% for testing
            test_start = int(0.9 * len(X_test))
            X_test = X_test[test_start:]
            y_test = y_test[test_start:]

            # Test predictions
            test_sample = X_test[0]
            corrections = imu_model.predict_corrections(test_sample)
            print(f"IMU LSTM Predicted Biases: {corrections['biases']}")
            print(f"IMU LSTM Predicted Scales: {corrections['scale_factors']}")


def main():
    """Main training pipeline"""

    parser = argparse.ArgumentParser(description='Train AI-Enhanced Navigation Models')
    parser.add_argument('--data-dir', type=str, default='ml/data',
                       help='Directory containing training data')
    parser.add_argument('--epochs-gravity', type=int, default=50,
                       help='Number of epochs for gravity U-Net training')
    parser.add_argument('--epochs-imu', type=int, default=50,
                       help='Number of epochs for IMU LSTM training')
    parser.add_argument('--train-gravity', action='store_true',
                       help='Train gravity enhancement model')
    parser.add_argument('--train-imu', action='store_true',
                       help='Train IMU error correction model')
    parser.add_argument('--evaluate', action='store_true',
                       help='Evaluate trained models')

    args = parser.parse_args()

    # Load data
    data_loader = NavigationDataLoader(args.data_dir)
    data_loader.load_all_data()

    # Train models
    gravity_model = None
    imu_model = None

    if args.train_gravity:
        gravity_model = train_gravity_unet(data_loader, args.epochs_gravity)

    if args.train_imu:
        imu_model = train_imu_lstm(data_loader, args.epochs_imu)

    # Evaluate models
    if args.evaluate:
        evaluate_models(gravity_model, imu_model, data_loader)

    print("\n" + "="*50)
    print("Training Complete!")
    print("="*50)
    print("\nTrained models saved to 'models/' directory")
    print("Use the ONNX models in C++ for inference")


if __name__ == "__main__":
    # If no arguments provided, train both models
    if len(sys.argv) == 1:
        sys.argv.extend(['--train-gravity', '--train-imu', '--evaluate'])

    main()