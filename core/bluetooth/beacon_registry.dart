class BeaconAnchor {
  final String uuid;
  final int major;
  final int minor;
  final double xMeters;
  final double yMeters;

  BeaconAnchor({
    required this.uuid,
    required this.major,
    required this.minor,
    required this.xMeters,
    required this.yMeters,
  });
}
