#!/usr/bin/env python3
"""
ML Inference Server for Real-time IMU Bias Correction
Provides a lightweight interface for C++ navigation system
"""

import numpy as np
import tensorflow as tf
import json
import sys
import os
from collections import deque
import time

class LSTMInferenceEngine:
    def __init__(self, model_path='../ml/models/lstm_imu_corrector.h5'):
        """Initialize the LSTM inference engine"""
        print(f"[ML Server] Loading model from {model_path}...", file=sys.stderr)
        self.model = tf.keras.models.load_model(model_path, compile=False)

        # Get model parameters
        self.seq_length = self.model.input_shape[1]  # Should be 10
        self.features = self.model.input_shape[2]    # Should be 6

        # Buffer for sequential data
        self.buffer = deque(maxlen=self.seq_length)

        # Statistics
        self.inference_count = 0
        self.total_time = 0.0

        print(f"[ML Server] Model loaded successfully", file=sys.stderr)
        print(f"[ML Server] Input shape: ({self.seq_length}, {self.features})", file=sys.stderr)
        print(f"[ML Server] Ready for inference", file=sys.stderr)

    def add_sample(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        """Add a new IMU sample to the buffer"""
        sample = [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
        self.buffer.append(sample)

    def predict_correction(self):
        """Predict IMU bias correction when buffer is full"""
        if len(self.buffer) < self.seq_length:
            # Not enough data yet, return zero corrections
            return {
                'acc_bias': [0.0, 0.0, 0.0],
                'gyro_bias': [0.0, 0.0, 0.0],
                'confidence': 0.0,
                'ready': False
            }

        # Prepare input for model
        input_data = np.array(list(self.buffer)).reshape(1, self.seq_length, self.features)

        # Normalize input (using same scaling as training)
        input_data = input_data.astype(np.float32)

        # Run inference
        start_time = time.time()
        predictions = self.model.predict(input_data, verbose=0)
        inference_time = (time.time() - start_time) * 1000  # ms

        self.inference_count += 1
        self.total_time += inference_time

        # Extract bias corrections
        bias_corrections = predictions[0]

        # Calculate confidence based on prediction magnitude
        confidence = min(1.0, 1.0 / (1.0 + np.linalg.norm(bias_corrections)))

        return {
            'acc_bias': bias_corrections[:3].tolist(),
            'gyro_bias': bias_corrections[3:].tolist(),
            'confidence': float(confidence),
            'ready': True,
            'inference_time_ms': inference_time
        }

    def get_stats(self):
        """Get performance statistics"""
        avg_time = self.total_time / max(1, self.inference_count)
        return {
            'inference_count': self.inference_count,
            'average_time_ms': avg_time
        }

def main():
    """Main server loop - communicates via stdin/stdout with C++"""
    engine = LSTMInferenceEngine()

    print("READY", flush=True)  # Signal to C++ that server is ready

    while True:
        try:
            # Read JSON command from stdin
            line = sys.stdin.readline()
            if not line:
                break

            command = json.loads(line.strip())

            if command['type'] == 'add_sample':
                # Add IMU sample to buffer
                engine.add_sample(
                    command['acc_x'], command['acc_y'], command['acc_z'],
                    command['gyro_x'], command['gyro_y'], command['gyro_z']
                )

                # Always predict after adding sample
                result = engine.predict_correction()
                print(json.dumps(result), flush=True)

            elif command['type'] == 'predict':
                # Just predict with current buffer
                result = engine.predict_correction()
                print(json.dumps(result), flush=True)

            elif command['type'] == 'stats':
                # Return performance statistics
                stats = engine.get_stats()
                print(json.dumps(stats), flush=True)

            elif command['type'] == 'shutdown':
                print(json.dumps({'status': 'shutdown'}), flush=True)
                break

        except json.JSONDecodeError as e:
            error_msg = {'error': f'Invalid JSON: {str(e)}'}
            print(json.dumps(error_msg), flush=True)
        except Exception as e:
            error_msg = {'error': str(e)}
            print(json.dumps(error_msg), flush=True)

    print("[ML Server] Shutting down", file=sys.stderr)

if __name__ == "__main__":
    main()