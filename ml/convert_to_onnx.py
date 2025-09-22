#!/usr/bin/env python3
"""
Convert trained TensorFlow/Keras models to ONNX format for C++ deployment
"""

import os
import tensorflow as tf
import tf2onnx
import onnx
import numpy as np

def convert_lstm_model():
    """Convert LSTM IMU corrector model to ONNX"""
    print("Loading LSTM model from models/lstm_imu_corrector.h5...")

    # Load the trained model
    model = tf.keras.models.load_model('models/lstm_imu_corrector.h5', compile=False)

    # Get input shape from model
    input_shape = model.input_shape
    seq_length = input_shape[1]  # Should be 10
    features = input_shape[2]    # Should be 6

    print(f"Model input shape: {input_shape}")
    print(f"Model output shape: {model.output_shape}")

    # Define input signature for ONNX conversion
    spec = (tf.TensorSpec((None, seq_length, features), tf.float32, name="imu_sequence"),)

    print("Converting to ONNX...")
    model_proto, _ = tf2onnx.convert.from_keras(
        model,
        input_signature=spec,
        opset=13,  # ONNX opset version
        output_path="models/lstm_imu_corrector.onnx"
    )

    print("✓ Model converted to ONNX format")

    # Verify the ONNX model
    print("\nVerifying ONNX model...")
    onnx_model = onnx.load("models/lstm_imu_corrector.onnx")
    onnx.checker.check_model(onnx_model)
    print("✓ ONNX model validation passed")

    # Get model info
    print("\nONNX Model Information:")
    print(f"  Input name: {onnx_model.graph.input[0].name}")
    print(f"  Input shape: {[d.dim_value for d in onnx_model.graph.input[0].type.tensor_type.shape.dim]}")
    print(f"  Output name: {onnx_model.graph.output[0].name}")
    print(f"  Output shape: {[d.dim_value for d in onnx_model.graph.output[0].type.tensor_type.shape.dim]}")

    # Test inference with dummy data
    print("\nTesting ONNX inference with dummy data...")
    import onnxruntime as ort

    session = ort.InferenceSession("models/lstm_imu_corrector.onnx")

    # Create dummy input (batch_size=1, seq_length=10, features=6)
    dummy_input = np.random.randn(1, seq_length, features).astype(np.float32)

    # Run inference
    input_name = session.get_inputs()[0].name
    output = session.run(None, {input_name: dummy_input})

    print(f"✓ ONNX inference successful")
    print(f"  Input shape: {dummy_input.shape}")
    print(f"  Output shape: {output[0].shape}")
    print(f"  Output (bias corrections): {output[0][0]}")

    return True

if __name__ == "__main__":
    try:
        success = convert_lstm_model()
        if success:
            print("\n" + "="*50)
            print("SUCCESS: Model ready for C++ deployment")
            print("  - H5 model: models/lstm_imu_corrector.h5")
            print("  - ONNX model: models/lstm_imu_corrector.onnx")
            print("="*50)
    except Exception as e:
        print(f"\nERROR: Conversion failed - {e}")
        import traceback
        traceback.print_exc()