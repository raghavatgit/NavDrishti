import 'dart:math';

class InertialOdometry {
  double _lastMagnitude = 9.81;
  int stepCount = 0;
  double headingRadians = 0.0;

  void processAccelerometer(double ax, double ay, double az) {
    final mag = sqrt(ax * ax + ay * ay + az * az);
    if (_lastMagnitude > 11.5 && mag <= 10.2) {
      stepCount++;
    }
    _lastMagnitude = mag;
  }

  void updateHeading(double compassHeadingDeg) {
    headingRadians = compassHeadingDeg * (pi / 180.0);
  }
}
