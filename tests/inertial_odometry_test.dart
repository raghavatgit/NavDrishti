void main() {
  // Unit test verifying that peak accelerations correctly trigger step increments
  int steps = 0;
  final readings = [9.8, 10.1, 12.0, 11.8, 9.5, 9.8];
  for (int i = 1; i < readings.length; i++) {
    if (readings[i - 1] > 11.5 && readings[i] <= 10.2) {
      steps++;
    }
  }
  assert(steps == 1, 'Expected exactly 1 step detected');
}
