export interface TrajectoryPoint {
  x: number;
  y: number;
  timestamp: number;
}

export class TrajectoryViewer {
  private points: TrajectoryPoint[] = [];

  addPoint(p: TrajectoryPoint): void {
    this.points.push(p);
  }

  getBounds(): { minX: number; maxX: number; minY: number; maxY: number } {
    const xs = this.points.map(p => p.x);
    const ys = this.points.map(p => p.y);
    return {
      minX: Math.min(...xs, 0),
      maxX: Math.max(...xs, 100),
      minY: Math.min(...ys, 0),
      maxY: Math.max(...ys, 100)
    };
  }
}
