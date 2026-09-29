void main() {
  final header = [0xA5, 0x01, 0x00, 0x0A, 0x0B, 0x04];
  assert(header[0] == 0xA5, 'Magic byte verification failed');
  assert(header[5] == 0x04, 'Payload length mismatch');
}
