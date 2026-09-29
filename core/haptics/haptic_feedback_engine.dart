class HapticFeedbackEngine {
  static const int shortPulseMs = 80;
  static const int longWarningMs = 300;

  static List<int> getPatternForDistance(double distanceMeters) {
    if (distanceMeters < 0.8) {
      return [0, longWarningMs, 50, longWarningMs]; // Imminent danger
    } else if (distanceMeters < 2.0) {
      return [0, shortPulseMs, 100, shortPulseMs]; // Caution
    }
    return [];
  }
}
