#!/usr/bin/env python3
"""
Unified ML Inference Server for All Sensors
Handles IMU, gravity, magnetometer, barometer, and terrain corrections
"""

import numpy as np
import tensorflow as tf
import json
import sys
import os
from collections import deque
import time
from typing import Dict, Any, List, Tuple
import logging

# Configure logging to stderr so it doesn't interfere with stdout communication
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    stream=sys.stderr
)

class UnifiedMLEngine:
    def __init__(self):
        """Initialize all ML models for different sensor types"""
        self.models = {}
        self.buffers = {}
        self.stats = {}

        # Load available models
        self._load_models()

        logging.info("Unified ML Engine initialized")

    def _load_models(self):
        """Load all available models"""
        # Try both paths - from ml directory or from build directory
        import os
        if os.path.exists("models/"):
            model_dir = "models/"
        elif os.path.exists("../ml/models/"):
            model_dir = "../ml/models/"
        else:
            logging.error("Cannot find models directory")
            return

        # IMU Bias Correction Model
        if os.path.exists(f"{model_dir}lstm_imu_corrector.h5"):
            try:
                self.models['imu'] = tf.keras.models.load_model(
                    f"{model_dir}lstm_imu_corrector.h5",
                    compile=False
                )
                self.buffers['imu'] = deque(maxlen=100)
                self.stats['imu'] = {'count': 0, 'total_time': 0.0}
                logging.info("IMU correction model loaded")
            except Exception as e:
                logging.error(f"Failed to load IMU model: {e}")

        # Gravity Enhancement Model (placeholder for now)
        if os.path.exists(f"{model_dir}gravity_enhancer.h5"):
            try:
                self.models['gravity'] = tf.keras.models.load_model(
                    f"{model_dir}gravity_enhancer.h5",
                    compile=False
                )
                self.buffers['gravity'] = deque(maxlen=10)
                self.stats['gravity'] = {'count': 0, 'total_time': 0.0}
                logging.info("Gravity enhancement model loaded")
            except Exception as e:
                logging.error(f"Failed to load gravity model: {e}")

        # Magnetometer Calibration Model (placeholder)
        if os.path.exists(f"{model_dir}magnetometer_calibrator.h5"):
            try:
                self.models['magnetometer'] = tf.keras.models.load_model(
                    f"{model_dir}magnetometer_calibrator.h5",
                    compile=False
                )
                self.buffers['magnetometer'] = deque(maxlen=20)
                self.stats['magnetometer'] = {'count': 0, 'total_time': 0.0}
                logging.info("Magnetometer calibration model loaded")
            except Exception as e:
                logging.error(f"Failed to load magnetometer model: {e}")

        # Barometer Drift Model (placeholder)
        if os.path.exists(f"{model_dir}barometer_corrector.h5"):
            try:
                self.models['barometer'] = tf.keras.models.load_model(
                    f"{model_dir}barometer_corrector.h5",
                    compile=False
                )
                self.buffers['barometer'] = deque(maxlen=30)
                self.stats['barometer'] = {'count': 0, 'total_time': 0.0}
                logging.info("Barometer correction model loaded")
            except Exception as e:
                logging.error(f"Failed to load barometer model: {e}")

    def process_imu(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Process IMU correction request"""
        if 'imu' not in self.models:
            return {
                'type': 'imu_correction',
                'ready': False,
                'error': 'IMU model not loaded'
            }

        # Add sample to buffer
        sample = [
            command.get('acc_x', 0.0),
            command.get('acc_y', 0.0),
            command.get('acc_z', 0.0),
            command.get('gyro_x', 0.0),
            command.get('gyro_y', 0.0),
            command.get('gyro_z', 0.0)
        ]
        self.buffers['imu'].append(sample)

        # Check if we have enough data
        if len(self.buffers['imu']) < 100:
            return {
                'type': 'imu_correction',
                'acc_bias': [0.0, 0.0, 0.0],
                'gyro_bias': [0.0, 0.0, 0.0],
                'confidence': 0.0,
                'ready': False
            }

        # Run inference
        try:
            input_data = np.array(list(self.buffers['imu'])).reshape(1, 100, 6)

            start_time = time.time()
            predictions = self.models['imu'].predict(input_data, verbose=0)
            inference_time = (time.time() - start_time) * 1000

            # Update stats
            self.stats['imu']['count'] += 1
            self.stats['imu']['total_time'] += inference_time

            # Extract corrections
            bias_corrections = predictions[0]

            # Calculate confidence based on prediction stability
            confidence = min(1.0, 1.0 / (1.0 + np.linalg.norm(bias_corrections)))

            return {
                'type': 'imu_correction',
                'acc_bias': bias_corrections[:3].tolist(),
                'gyro_bias': bias_corrections[3:].tolist(),
                'confidence': float(confidence),
                'ready': True,
                'inference_time_ms': inference_time
            }
        except Exception as e:
            logging.error(f"IMU inference error: {e}")
            return {
                'type': 'imu_correction',
                'ready': False,
                'error': str(e)
            }

    def process_gravity(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Process gravity enhancement request"""
        if 'gravity' not in self.models:
            # Return synthetic enhancement for now
            return {
                'type': 'gravity_enhancement',
                'enhanced_tensor': [
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0]
                ],
                'confidence': 0.5,
                'ready': True,
                'synthetic': True
            }

        # TODO: Implement actual gravity enhancement
        return {
            'type': 'gravity_enhancement',
            'ready': False
        }

    def process_magnetometer(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Process magnetometer calibration request"""
        if 'magnetometer' not in self.models:
            # Return simple calibration for now
            mag_x = command.get('mag_x', 0.0)
            mag_y = command.get('mag_y', 0.0)
            mag_z = command.get('mag_z', 0.0)

            # Simple hard-iron correction
            calibrated = [
                mag_x * 1.01,  # Scale correction
                mag_y * 1.01,
                mag_z * 1.01
            ]

            return {
                'type': 'magnetometer_calibration',
                'calibrated_mag': calibrated,
                'heading_correction': 0.0,
                'confidence': 0.7,
                'ready': True,
                'synthetic': True
            }

        # TODO: Implement actual magnetometer calibration
        return {
            'type': 'magnetometer_calibration',
            'ready': False
        }

    def process_barometer(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Process barometer drift correction request"""
        if 'barometer' not in self.models:
            # Simple temperature-based correction
            pressure = command.get('pressure', 1013.25)
            temperature = command.get('temperature', 20.0)

            # Simple drift model
            drift_correction = (temperature - 20.0) * 0.1

            return {
                'type': 'barometer_correction',
                'altitude_correction': drift_correction,
                'confidence': 0.6,
                'ready': True,
                'synthetic': True
            }

        # TODO: Implement actual barometer correction
        return {
            'type': 'barometer_correction',
            'ready': False
        }

    def process_terrain(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Process terrain correlation request"""
        # For now, return simple correlation
        radar_alt = command.get('radar_altitude', 0.0)
        estimated_pos = command.get('position', [0.0, 0.0, 0.0])

        # Simple synthetic correction
        return {
            'type': 'terrain_correlation',
            'position_correction': [0.0, 0.0, 0.0],
            'confidence': 0.5,
            'ready': True,
            'synthetic': True
        }

    def get_stats(self) -> Dict[str, Any]:
        """Get performance statistics for all models"""
        stats = {}
        for model_name, model_stats in self.stats.items():
            if model_stats['count'] > 0:
                stats[model_name] = {
                    'inference_count': model_stats['count'],
                    'average_time_ms': model_stats['total_time'] / model_stats['count']
                }
        return {
            'type': 'stats',
            'models': stats
        }

    def process_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Route command to appropriate processor"""
        cmd_type = command.get('type', '')

        if cmd_type == 'add_sample':  # Legacy IMU command
            return self.process_imu(command)
        elif cmd_type == 'imu_correction':
            return self.process_imu(command)
        elif cmd_type == 'gravity_enhancement':
            return self.process_gravity(command)
        elif cmd_type == 'magnetometer_calibration':
            return self.process_magnetometer(command)
        elif cmd_type == 'barometer_correction':
            return self.process_barometer(command)
        elif cmd_type == 'terrain_correlation':
            return self.process_terrain(command)
        elif cmd_type == 'stats':
            return self.get_stats()
        elif cmd_type == 'shutdown':
            return {'type': 'shutdown', 'status': 'ok'}
        else:
            return {
                'type': 'error',
                'message': f'Unknown command type: {cmd_type}'
            }


def main():
    """Main server loop"""
    engine = UnifiedMLEngine()

    # Signal ready
    print("READY", flush=True)
    logging.info("Server ready and listening")

    while True:
        try:
            # Read command from stdin
            line = sys.stdin.readline()
            if not line:
                break

            # Parse JSON command
            command = json.loads(line.strip())

            # Process command
            result = engine.process_command(command)

            # Return result
            print(json.dumps(result), flush=True)

            # Check for shutdown
            if command.get('type') == 'shutdown':
                break

        except json.JSONDecodeError as e:
            error_msg = {'type': 'error', 'message': f'Invalid JSON: {str(e)}'}
            print(json.dumps(error_msg), flush=True)
        except Exception as e:
            error_msg = {'type': 'error', 'message': str(e)}
            print(json.dumps(error_msg), flush=True)
            logging.error(f"Processing error: {e}")

    logging.info("Server shutting down")


if __name__ == "__main__":
    main()