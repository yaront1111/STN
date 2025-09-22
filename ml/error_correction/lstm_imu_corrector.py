"""
LSTM Model for IMU Error Correction

This model learns to predict and correct IMU biases and scale factors
based on historical sensor data patterns.

The LSTM architecture is ideal because:
1. It captures temporal dependencies in sensor errors
2. It can learn long-term drift patterns
3. It adapts to changing sensor characteristics
"""

import torch
import torch.nn as nn
import numpy as np
from typing import Tuple, Dict
import onnx
import torch.onnx


class IMUErrorCorrectorLSTM(nn.Module):
    """LSTM-based IMU error correction model"""

    def __init__(self,
                 input_size: int = 6,  # acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
                 hidden_size: int = 128,
                 num_layers: int = 3,
                 output_size: int = 12,  # 6 biases + 6 scale factors
                 dropout: float = 0.2):
        """
        Initialize LSTM model for IMU error correction

        Args:
            input_size: Number of input features (6 for IMU)
            hidden_size: Size of LSTM hidden state
            num_layers: Number of LSTM layers
            output_size: Number of output features (biases + scale factors)
            dropout: Dropout rate for regularization
        """
        super(IMUErrorCorrectorLSTM, self).__init__()

        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.output_size = output_size

        # Input normalization
        self.input_norm = nn.BatchNorm1d(input_size)

        # LSTM layers with dropout
        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0,
            bidirectional=True  # Use bidirectional for better context
        )

        # Attention mechanism for focusing on important time steps
        self.attention = nn.MultiheadAttention(
            embed_dim=hidden_size * 2,  # *2 for bidirectional
            num_heads=8,
            dropout=dropout
        )

        # Output layers
        self.fc1 = nn.Linear(hidden_size * 2, hidden_size)
        self.dropout = nn.Dropout(dropout)
        self.fc2 = nn.Linear(hidden_size, output_size)

        # Residual connection for stability
        self.residual_weight = nn.Parameter(torch.tensor(0.1))

        # Initialize weights
        self._init_weights()

    def _init_weights(self):
        """Initialize model weights using Xavier initialization"""
        for name, param in self.named_parameters():
            if 'weight' in name and len(param.shape) >= 2:
                nn.init.xavier_uniform_(param)
            elif 'bias' in name:
                nn.init.zeros_(param)

    def forward(self, x: torch.Tensor, hidden: Tuple = None) -> Tuple[torch.Tensor, Tuple]:
        """
        Forward pass through the model

        Args:
            x: Input tensor of shape (batch, sequence_length, features)
            hidden: Optional hidden state from previous batch

        Returns:
            corrections: Predicted bias and scale corrections
            hidden: Updated hidden state
        """
        batch_size, seq_len, _ = x.shape

        # Normalize input
        x_norm = x.reshape(-1, self.input_size)
        x_norm = self.input_norm(x_norm)
        x_norm = x_norm.reshape(batch_size, seq_len, self.input_size)

        # LSTM forward pass
        lstm_out, hidden = self.lstm(x_norm, hidden)

        # Apply attention mechanism
        # Reshape for attention: (seq_len, batch, features)
        lstm_out_t = lstm_out.transpose(0, 1)
        attended, _ = self.attention(lstm_out_t, lstm_out_t, lstm_out_t)
        attended = attended.transpose(0, 1)  # Back to (batch, seq_len, features)

        # Use the last time step output
        last_output = attended[:, -1, :]

        # Dense layers for final prediction
        out = torch.relu(self.fc1(last_output))
        out = self.dropout(out)
        corrections = self.fc2(out)

        # Apply residual connection for stability
        # Start with identity (no correction)
        identity = torch.zeros_like(corrections)
        identity[:, 6:] = 1.0  # Scale factors start at 1
        corrections = identity + self.residual_weight * corrections

        return corrections, hidden

    def predict_corrections(self, imu_window: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Predict IMU corrections from a window of measurements

        Args:
            imu_window: Numpy array of shape (window_size, 6)

        Returns:
            Dictionary with 'biases' and 'scale_factors'
        """
        self.eval()
        with torch.no_grad():
            # Convert to tensor and add batch dimension
            x = torch.FloatTensor(imu_window).unsqueeze(0)

            # Forward pass
            corrections, _ = self.forward(x)

            # Extract biases and scale factors
            biases = corrections[0, :6].numpy()
            scale_factors = corrections[0, 6:].numpy()

        return {
            'biases': biases,
            'scale_factors': scale_factors
        }

    def train_model(self, train_loader, val_loader, epochs=50, lr=1e-3):
        """Train the LSTM model"""

        optimizer = torch.optim.Adam(self.parameters(), lr=lr)
        scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            optimizer, mode='min', factor=0.5, patience=5
        )
        criterion = CombinedIMULoss()

        best_val_loss = float('inf')
        patience_counter = 0
        max_patience = 10

        for epoch in range(epochs):
            # Training phase
            self.train()
            train_loss = 0.0
            for batch_idx, (imu_data, true_errors) in enumerate(train_loader):
                optimizer.zero_grad()

                # Forward pass
                predictions, _ = self.forward(imu_data)

                # Compute loss
                loss = criterion(predictions, true_errors, imu_data)

                # Backward pass
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.parameters(), max_norm=1.0)
                optimizer.step()

                train_loss += loss.item()

            # Validation phase
            self.eval()
            val_loss = 0.0
            with torch.no_grad():
                for imu_data, true_errors in val_loader:
                    predictions, _ = self.forward(imu_data)
                    loss = criterion(predictions, true_errors, imu_data)
                    val_loss += loss.item()

            # Average losses
            train_loss /= len(train_loader)
            val_loss /= len(val_loader)

            # Learning rate scheduling
            scheduler.step(val_loss)

            print(f"Epoch {epoch+1}/{epochs} - "
                  f"Train Loss: {train_loss:.6f}, "
                  f"Val Loss: {val_loss:.6f}")

            # Early stopping
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                patience_counter = 0
                torch.save(self.state_dict(), 'imu_corrector_best.pth')
            else:
                patience_counter += 1
                if patience_counter >= max_patience:
                    print(f"Early stopping at epoch {epoch+1}")
                    break

        # Load best model
        self.load_state_dict(torch.load('imu_corrector_best.pth'))

    def export_onnx(self, filename='imu_corrector.onnx', sequence_length=100):
        """Export model to ONNX format for C++ deployment"""

        self.eval()

        # Create dummy input
        dummy_input = torch.randn(1, sequence_length, self.input_size)

        # Export to ONNX
        torch.onnx.export(
            self,
            dummy_input,
            filename,
            export_params=True,
            opset_version=11,
            do_constant_folding=True,
            input_names=['imu_input'],
            output_names=['corrections'],
            dynamic_axes={
                'imu_input': {0: 'batch_size', 1: 'sequence_length'},
                'corrections': {0: 'batch_size'}
            }
        )

        print(f"Model exported to {filename}")

        # Verify the exported model
        onnx_model = onnx.load(filename)
        onnx.checker.check_model(onnx_model)
        print("ONNX model verified successfully")

        return filename


class CombinedIMULoss(nn.Module):
    """Combined loss function for IMU error correction"""

    def __init__(self,
                 bias_weight: float = 1.0,
                 scale_weight: float = 1.0,
                 physics_weight: float = 0.0):  # Disabled for flight
        super(CombinedIMULoss, self).__init__()
        self.bias_weight = bias_weight
        self.scale_weight = scale_weight
        self.physics_weight = 0.0  # Force disabled - invalid for dynamic flight

    def forward(self, predictions, targets, raw_imu):
        """
        Calculate combined loss

        Args:
            predictions: Predicted corrections (biases and scales)
            targets: True corrections
            raw_imu: Raw IMU measurements for physics constraints

        Returns:
            Combined loss value
        """

        # Split predictions and targets
        pred_biases = predictions[:, :6]
        pred_scales = predictions[:, 6:]
        true_biases = targets[:, :6]
        true_scales = targets[:, 6:]

        # Bias loss (MSE)
        bias_loss = torch.mean((pred_biases - true_biases) ** 2)

        # Scale factor loss (MSE with emphasis near 1.0)
        scale_loss = torch.mean((pred_scales - true_scales) ** 2)

        # REMOVED: Physics-based loss was incorrect for dynamic flight
        # The 9.81 m/s² constraint is only valid when stationary
        # During flight, acceleration includes vehicle dynamics
        # physics_loss = 0.0  # Disabled

        # Combined loss (physics loss removed for flight scenarios)
        total_loss = (self.bias_weight * bias_loss +
                     self.scale_weight * scale_loss)

        return total_loss


class IMUDataProcessor:
    """Preprocessor for IMU data"""

    def __init__(self, window_size: int = 100, stride: int = 50):
        self.window_size = window_size
        self.stride = stride
        self.scaler_acc = None
        self.scaler_gyro = None

    def create_windows(self, imu_data: np.ndarray) -> np.ndarray:
        """Create sliding windows from continuous IMU data"""

        windows = []
        for i in range(0, len(imu_data) - self.window_size + 1, self.stride):
            window = imu_data[i:i + self.window_size]
            windows.append(window)

        return np.array(windows)

    def normalize(self, data: np.ndarray) -> np.ndarray:
        """Normalize IMU data using statistics"""

        # Typical IMU ranges
        acc_std = 2.0  # m/s²
        gyro_std = 0.5  # rad/s

        normalized = data.copy()
        normalized[:, :3] /= acc_std  # Normalize accelerometer
        normalized[:, 3:] /= gyro_std  # Normalize gyroscope

        return normalized

    def extract_features(self, window: np.ndarray) -> np.ndarray:
        """Extract statistical features from IMU window"""

        features = []

        # Mean
        features.extend(np.mean(window, axis=0))

        # Standard deviation
        features.extend(np.std(window, axis=0))

        # Maximum
        features.extend(np.max(window, axis=0))

        # Minimum
        features.extend(np.min(window, axis=0))

        # RMS
        features.extend(np.sqrt(np.mean(window**2, axis=0)))

        return np.array(features)


if __name__ == "__main__":
    # Example usage
    model = IMUErrorCorrectorLSTM()
    print(f"Model parameters: {sum(p.numel() for p in model.parameters())}")

    # Test forward pass
    dummy_input = torch.randn(8, 100, 6)  # batch=8, seq=100, features=6
    output, hidden = model(dummy_input)
    print(f"Output shape: {output.shape}")

    # Export to ONNX
    model.export_onnx()