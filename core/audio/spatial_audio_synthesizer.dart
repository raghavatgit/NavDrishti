import 'dart:math';

class SpatialAudioSynthesizer {
  static Map<String, double> computePanning(double angleRadians) {
    final leftGain = (cos(angleRadians) + 1.0) / 2.0;
    final rightGain = (sin(angleRadians) + 1.0) / 2.0;
    return {'left': leftGain, 'right': rightGain};
  }
}
