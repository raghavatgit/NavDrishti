export class TelemetryWebSocketService {
  private ws: WebSocket | null = null;
  connect(url: string) {
    this.ws = new WebSocket(url);
    this.ws.binaryType = 'arraybuffer';
  }
}
