# Sensor Fusion Architecture: EKF for Assistive Orientation

## State Vector
x = [orientation_quaternion, angular_velocity_bias, linear_acceleration]

## Sensors Integrated
1. 3-Axis MEMS Accelerometer (100 Hz): Provides gravity vector reference.
2. 3-Axis Rate Gyroscope (100 Hz): High-rate integration for immediate attitude changes.
3. 3-Axis Magnetometer (25 Hz): Absolute magnetic North reference.
