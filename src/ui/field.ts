// Canvas renderer: the game field with the robot, trails and targets.
// Field frame: origin at center, +x right, +y up, inches. The background
// image is the 12'x12' playing surface, so -72..72 maps edge to edge.

import type { World } from '../sim/world.ts';
import type { PathPoint } from './pathModel.ts';

const FIELD = 144; // inches
const MARGIN = 6; // inches of breathing room around the perimeter

/** length in px of the arrival-heading arrow; its tip is the drag handle */
export const HEADING_ARM_PX = 26;

export interface FieldTarget {
  x: number;
  y: number;
}

export class FieldView {
  canvas: HTMLCanvasElement;
  target: FieldTarget | null = null;
  showGhost = true;
  showTrails = true;
  plannerPoints: PathPoint[] = [];
  plannerSelected = -1;

  private ctx: CanvasRenderingContext2D;
  private size = 0;
  private fieldImg = new Image();

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d')!;
    this.fieldImg.src = '/field.jpg';
    const observer = new ResizeObserver(() => this.resize());
    observer.observe(canvas.parentElement!);
    this.resize();
  }

  private resize(): void {
    const parent = this.canvas.parentElement!;
    const px = Math.min(parent.clientWidth, parent.clientHeight);
    if (px <= 0) return;
    const dpr = window.devicePixelRatio || 1;
    this.size = px;
    this.canvas.width = px * dpr;
    this.canvas.height = px * dpr;
    this.canvas.style.width = `${px}px`;
    this.canvas.style.height = `${px}px`;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  private get scale(): number {
    return this.size / (FIELD + MARGIN * 2);
  }

  /** field inches -> canvas px */
  toPx(x: number, y: number): [number, number] {
    const s = this.scale;
    return [this.size / 2 + x * s, this.size / 2 - y * s];
  }

  /** canvas px -> field inches */
  toField(px: number, py: number): [number, number] {
    const s = this.scale;
    return [(px - this.size / 2) / s, (this.size / 2 - py) / s];
  }

  render(world: World, timeMs: number): void {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.size, this.size);

    this.drawField(ctx);
    if (this.showTrails) this.drawTrails(ctx, world);
    if (this.plannerPoints.length > 0) this.drawPlannerPath(ctx);
    if (this.target) this.drawTarget(ctx, this.target, timeMs);

    // odometry's belief, drawn as a ghost under the true robot
    if (this.showGhost) {
      const odom = world.chassis.getPose(true);
      this.drawRobot(ctx, odom.x, odom.y, odom.theta, world, true);
    }
    this.drawRobot(ctx, world.robot.x, world.robot.y, world.robot.theta, world, false);
  }

  private drawField(ctx: CanvasRenderingContext2D): void {
    const s = this.scale;
    const [left, top] = this.toPx(-FIELD / 2, FIELD / 2);
    const sidePx = FIELD * s;

    if (this.fieldImg.complete && this.fieldImg.naturalWidth > 0) {
      ctx.drawImage(this.fieldImg, left, top, sidePx, sidePx);
    } else {
      ctx.fillStyle = '#3a3d42';
      ctx.fillRect(left, top, sidePx, sidePx);
    }

    // faint 24" tile grid + center crosshair for coordinate reading
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.10)';
    ctx.lineWidth = 1;
    for (let i = 1; i < 6; i++) {
      ctx.beginPath();
      ctx.moveTo(left + i * 24 * s, top);
      ctx.lineTo(left + i * 24 * s, top + sidePx);
      ctx.moveTo(left, top + i * 24 * s);
      ctx.lineTo(left + sidePx, top + i * 24 * s);
      ctx.stroke();
    }

    // perimeter
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.22)';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(left, top, sidePx, sidePx);
  }

  private drawTrails(ctx: CanvasRenderingContext2D, world: World): void {
    const drawTrail = (trail: { x: number; y: number }[], style: string, dash: number[]) => {
      if (trail.length < 2) return;
      ctx.strokeStyle = style;
      ctx.lineWidth = 2.5;
      ctx.lineJoin = 'round';
      ctx.setLineDash(dash);
      ctx.beginPath();
      const [x0, y0] = this.toPx(trail[0].x, trail[0].y);
      ctx.moveTo(x0, y0);
      for (let i = 1; i < trail.length; i++) {
        const [x, y] = this.toPx(trail[i].x, trail[i].y);
        ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    };

    // odometry's belief: dashed blue. Ground truth: solid Apple blue
    drawTrail(world.odomTrail, 'rgba(96, 165, 250, 0.9)', [6, 5]);
    drawTrail(world.trueTrail, '#0a84ff', []);
  }

  private drawPlannerPath(ctx: CanvasRenderingContext2D): void {
    const points = this.plannerPoints;

    // connecting segments; backwards approaches are dashed
    ctx.lineWidth = 2;
    for (let i = 1; i < points.length; i++) {
      const [x0, y0] = this.toPx(points[i - 1].x, points[i - 1].y);
      const [x1, y1] = this.toPx(points[i].x, points[i].y);
      ctx.strokeStyle = 'rgba(175, 82, 222, 0.85)';
      ctx.setLineDash(points[i].forwards ? [] : [6, 5]);
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
    }
    ctx.setLineDash([]);

    points.forEach((p, i) => {
      const [x, y] = this.toPx(p.x, p.y);
      const isStart = i === 0;

      const selected = i === this.plannerSelected;

      // arrival-heading arrow for pose points (and the start heading),
      // ending in a handle that can be dragged to rotate it
      if (isStart || p.kind === 'pose') {
        const mathTheta = ((90 - p.theta) * Math.PI) / 180;
        const hx = x + HEADING_ARM_PX * Math.cos(mathTheta);
        const hy = y - HEADING_ARM_PX * Math.sin(mathTheta);
        ctx.strokeStyle = '#af52de';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(hx, hy);
        ctx.stroke();
        ctx.fillStyle = selected ? '#ffffff' : '#af52de';
        ctx.strokeStyle = selected ? '#af52de' : '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(hx, hy, 4.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }

      if (selected) {
        ctx.strokeStyle = '#0a84ff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x, y, 13, 0, Math.PI * 2);
        ctx.stroke();
      }

      ctx.fillStyle = isStart ? '#30d158' : '#af52de';
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(x, y, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.font = "600 10px 'JetBrains Mono', monospace";
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(isStart ? 'S' : String(i), x, y + 0.5);
    });
    ctx.textBaseline = 'alphabetic';
  }

  private drawTarget(ctx: CanvasRenderingContext2D, target: FieldTarget, timeMs: number): void {
    const [x, y] = this.toPx(target.x, target.y);
    const pulse = 9 + 2.5 * Math.sin(timeMs / 260);
    ctx.strokeStyle = '#0a84ff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, pulse, 0, Math.PI * 2);
    ctx.stroke();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.9)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x - 13, y);
    ctx.lineTo(x + 13, y);
    ctx.moveTo(x, y - 13);
    ctx.lineTo(x, y + 13);
    ctx.stroke();
  }

  private drawRobot(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, thetaMath: number,
    world: World, ghost: boolean,
  ): void {
    const s = this.scale;
    const w = world.config.robotWidth * s;
    const l = world.config.robotLength * s;
    const [px, py] = this.toPx(x, y);

    ctx.save();
    ctx.translate(px, py);
    // canvas y is flipped, so math CCW becomes canvas CW
    ctx.rotate(-thetaMath);

    if (ghost) {
      // odometry ghost: dashed blue outline
      ctx.strokeStyle = 'rgba(96, 165, 250, 0.95)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 4]);
      ctx.strokeRect(-l / 2, -w / 2, l, w);
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(l / 2, 0);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.restore();
      return;
    }

    // drop shadow for lift off the field photo
    ctx.shadowColor = 'rgba(0, 0, 0, 0.45)';
    ctx.shadowBlur = 8;
    ctx.shadowOffsetY = 2;

    // chassis body
    ctx.fillStyle = 'rgba(18, 21, 27, 0.94)';
    ctx.strokeStyle = '#f4f5f7';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-l / 2, -w / 2, l, w, 4);
    ctx.fill();
    ctx.shadowColor = 'transparent';
    ctx.stroke();

    // wheels
    ctx.fillStyle = '#2c333d';
    const wheelL = l * 0.26;
    const wheelW = w * 0.13;
    for (const side of [-1, 1]) {
      for (const fore of [-1, 0, 1]) {
        ctx.beginPath();
        ctx.roundRect(
          fore * l * 0.31 - wheelL / 2,
          side * (w / 2 - wheelW * 0.62) - wheelW / 2,
          wheelL, wheelW, 2,
        );
        ctx.fill();
      }
    }

    // heading wedge
    ctx.fillStyle = '#0a84ff';
    ctx.beginPath();
    ctx.moveTo(l / 2 - 3, 0);
    ctx.lineTo(l * 0.12, -w * 0.2);
    ctx.lineTo(l * 0.12, w * 0.2);
    ctx.closePath();
    ctx.fill();

    // center dot
    ctx.fillStyle = '#f4f5f7';
    ctx.beginPath();
    ctx.arc(0, 0, 2.2, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore();
  }
}
