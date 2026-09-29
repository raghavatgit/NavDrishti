# Indoor Positioning AHRS Drift Mitigation

## Magnetic Disturbance Rejection
When local magnetic field $|B| - |B_{\text{ref}}| > \delta$, magnetometer updates are decoupled from the orientation filter, relying on gyro integration.
