"""
U-Net Model for Gravity Map Enhancement

This model takes low-resolution gravity gradient measurements
and produces high-resolution gravity field estimates.

The U-Net architecture is ideal for this task because:
1. It preserves spatial information through skip connections
2. It can learn multi-scale features
3. It's proven effective for physical field reconstruction
"""

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
import numpy as np


class GravityUNet:
    """U-Net for gravity field super-resolution and denoising"""

    def __init__(self, input_shape=(128, 128, 9), output_channels=9):
        """
        Initialize U-Net model for gravity enhancement

        Args:
            input_shape: (height, width, channels) - 9 channels for gravity tensor
            output_channels: Number of output channels (9 for full tensor)
        """
        self.input_shape = input_shape
        self.output_channels = output_channels
        self.model = self._build_model()

    def _build_model(self):
        """Build the U-Net architecture"""

        inputs = layers.Input(shape=self.input_shape)

        # Encoder (contracting path)
        # Block 1
        c1 = layers.Conv2D(64, (3, 3), activation='relu', padding='same')(inputs)
        c1 = layers.Conv2D(64, (3, 3), activation='relu', padding='same')(c1)
        p1 = layers.MaxPooling2D((2, 2))(c1)

        # Block 2
        c2 = layers.Conv2D(128, (3, 3), activation='relu', padding='same')(p1)
        c2 = layers.Conv2D(128, (3, 3), activation='relu', padding='same')(c2)
        p2 = layers.MaxPooling2D((2, 2))(c2)

        # Block 3
        c3 = layers.Conv2D(256, (3, 3), activation='relu', padding='same')(p2)
        c3 = layers.Conv2D(256, (3, 3), activation='relu', padding='same')(c3)
        p3 = layers.MaxPooling2D((2, 2))(c3)

        # Block 4
        c4 = layers.Conv2D(512, (3, 3), activation='relu', padding='same')(p3)
        c4 = layers.Conv2D(512, (3, 3), activation='relu', padding='same')(c4)
        p4 = layers.MaxPooling2D((2, 2))(c4)

        # Bottleneck
        c5 = layers.Conv2D(1024, (3, 3), activation='relu', padding='same')(p4)
        c5 = layers.Conv2D(1024, (3, 3), activation='relu', padding='same')(c5)

        # Decoder (expansive path)
        # Block 6
        u6 = layers.Conv2DTranspose(512, (2, 2), strides=(2, 2), padding='same')(c5)
        u6 = layers.concatenate([u6, c4])
        c6 = layers.Conv2D(512, (3, 3), activation='relu', padding='same')(u6)
        c6 = layers.Conv2D(512, (3, 3), activation='relu', padding='same')(c6)

        # Block 7
        u7 = layers.Conv2DTranspose(256, (2, 2), strides=(2, 2), padding='same')(c6)
        u7 = layers.concatenate([u7, c3])
        c7 = layers.Conv2D(256, (3, 3), activation='relu', padding='same')(u7)
        c7 = layers.Conv2D(256, (3, 3), activation='relu', padding='same')(c7)

        # Block 8
        u8 = layers.Conv2DTranspose(128, (2, 2), strides=(2, 2), padding='same')(c7)
        u8 = layers.concatenate([u8, c2])
        c8 = layers.Conv2D(128, (3, 3), activation='relu', padding='same')(u8)
        c8 = layers.Conv2D(128, (3, 3), activation='relu', padding='same')(c8)

        # Block 9
        u9 = layers.Conv2DTranspose(64, (2, 2), strides=(2, 2), padding='same')(c8)
        u9 = layers.concatenate([u9, c1], axis=3)
        c9 = layers.Conv2D(64, (3, 3), activation='relu', padding='same')(u9)
        c9 = layers.Conv2D(64, (3, 3), activation='relu', padding='same')(c9)

        # Output layer - no activation for regression
        outputs = layers.Conv2D(self.output_channels, (1, 1))(c9)

        # Add physics-informed constraints
        outputs = PhysicsConstraintLayer()(outputs)

        model = keras.Model(inputs=[inputs], outputs=[outputs])
        return model

    def compile_model(self, learning_rate=1e-4):
        """Compile the model with appropriate loss and metrics"""

        self.model.compile(
            optimizer=keras.optimizers.Adam(learning_rate=learning_rate),
            loss=self.physics_informed_loss,
            metrics=[
                'mae',
                self.tensor_symmetry_metric,
                self.laplacian_constraint_metric
            ]
        )

    def physics_informed_loss(self, y_true, y_pred):
        """
        Physics-informed loss function that enforces:
        1. Data fidelity
        2. Tensor symmetry
        3. Laplace's equation (trace = 0 in free space)
        """

        # Data fidelity loss (MSE)
        data_loss = tf.reduce_mean(tf.square(y_true - y_pred))

        # Reshape to extract tensor components
        batch_size = tf.shape(y_pred)[0]
        height = tf.shape(y_pred)[1]
        width = tf.shape(y_pred)[2]

        # Reshape to [batch, height, width, 3, 3]
        tensor_pred = tf.reshape(y_pred, [batch_size, height, width, 3, 3])

        # Symmetry loss: T should be symmetric
        tensor_transpose = tf.transpose(tensor_pred, [0, 1, 2, 4, 3])
        symmetry_loss = tf.reduce_mean(tf.square(tensor_pred - tensor_transpose))

        # Laplacian constraint: trace(T) = 0 in free space
        trace = tf.linalg.trace(tensor_pred)
        laplacian_loss = tf.reduce_mean(tf.square(trace))

        # Combined loss with weights
        total_loss = data_loss + 0.1 * symmetry_loss + 0.05 * laplacian_loss

        return total_loss

    def tensor_symmetry_metric(self, y_true, y_pred):
        """Metric to monitor tensor symmetry"""
        batch_size = tf.shape(y_pred)[0]
        height = tf.shape(y_pred)[1]
        width = tf.shape(y_pred)[2]

        tensor_pred = tf.reshape(y_pred, [batch_size, height, width, 3, 3])
        tensor_transpose = tf.transpose(tensor_pred, [0, 1, 2, 4, 3])

        return tf.reduce_mean(tf.abs(tensor_pred - tensor_transpose))

    def laplacian_constraint_metric(self, y_true, y_pred):
        """Metric to monitor Laplacian constraint"""
        batch_size = tf.shape(y_pred)[0]
        height = tf.shape(y_pred)[1]
        width = tf.shape(y_pred)[2]

        tensor_pred = tf.reshape(y_pred, [batch_size, height, width, 3, 3])
        trace = tf.linalg.trace(tensor_pred)

        return tf.reduce_mean(tf.abs(trace))

    def train(self, train_data, val_data, epochs=100, batch_size=32):
        """Train the U-Net model"""

        # Callbacks
        callbacks = [
            keras.callbacks.EarlyStopping(
                patience=10,
                restore_best_weights=True,
                monitor='val_loss'
            ),
            keras.callbacks.ReduceLROnPlateau(
                factor=0.5,
                patience=5,
                min_lr=1e-7,
                monitor='val_loss'
            ),
            keras.callbacks.ModelCheckpoint(
                'gravity_unet_best.h5',
                save_best_only=True,
                monitor='val_loss'
            )
        ]

        history = self.model.fit(
            train_data,
            validation_data=val_data,
            epochs=epochs,
            batch_size=batch_size,
            callbacks=callbacks,
            verbose=1
        )

        return history

    def predict(self, input_tensor):
        """Predict high-resolution gravity field"""
        return self.model.predict(input_tensor)

    def save_onnx(self, filename='gravity_unet.onnx'):
        """Export model to ONNX format for C++ deployment"""
        import tf2onnx

        spec = (tf.TensorSpec((None, 128, 128, 9), tf.float32, name="input"),)
        output_path = filename

        model_proto, _ = tf2onnx.convert.from_keras(
            self.model,
            input_signature=spec,
            opset=13,
            output_path=output_path
        )

        print(f"Model exported to {output_path}")
        return output_path


class PhysicsConstraintLayer(layers.Layer):
    """Custom layer to enforce physical constraints on gravity tensor"""

    def call(self, inputs):
        """
        Enforce physical constraints:
        1. Symmetry: T_ij = T_ji
        2. Zero trace in free space: T_xx + T_yy + T_zz = 0
        """

        batch_size = tf.shape(inputs)[0]
        height = tf.shape(inputs)[1]
        width = tf.shape(inputs)[2]

        # Reshape to tensor form
        tensor = tf.reshape(inputs, [batch_size, height, width, 3, 3])

        # Enforce symmetry
        tensor_symmetric = 0.5 * (tensor + tf.transpose(tensor, [0, 1, 2, 4, 3]))

        # Enforce trace constraint (subtract mean of diagonal)
        trace = tf.linalg.trace(tensor_symmetric)
        trace_correction = tf.expand_dims(tf.expand_dims(trace / 3.0, -1), -1)
        eye = tf.eye(3)
        correction = trace_correction * eye

        tensor_constrained = tensor_symmetric - correction

        # Reshape back
        output = tf.reshape(tensor_constrained, [batch_size, height, width, 9])

        return output


class DataAugmentation:
    """Data augmentation for gravity field training"""

    @staticmethod
    def augment(tensor, label):
        """Apply augmentations to gravity tensor data"""

        # Random rotation
        if np.random.random() > 0.5:
            k = np.random.randint(1, 4)
            tensor = np.rot90(tensor, k)
            label = np.rot90(label, k)

        # Random flip
        if np.random.random() > 0.5:
            tensor = np.fliplr(tensor)
            label = np.fliplr(label)

        if np.random.random() > 0.5:
            tensor = np.flipud(tensor)
            label = np.flipud(label)

        # Add noise
        noise_level = np.random.uniform(0, 0.05)
        noise = np.random.normal(0, noise_level, tensor.shape)
        tensor = tensor + noise

        return tensor, label


if __name__ == "__main__":
    # Example usage
    model = GravityUNet()
    model.compile_model()
    print(model.model.summary())

    # Export to ONNX
    model.save_onnx()