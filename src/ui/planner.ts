// Path planner UI: click waypoints on the field, edit them, simulate the
// route, and generate the equivalent C++ for the real robot. The data model
// and code generation live in pathModel.ts.

import type { Chassis } from '../lib/chassis.ts';
import { HEADING_ARM_PX, type FieldView } from './field.ts';
import { parseAutoCode } from './parseAuto.ts';
import {
  DEFAULT_TIMEOUT,
  bearing,
  buildProgram,
  clampField,
  fmt,
  generateCpp,
  mirrorPoints,
  newPoint,
  parseSavedPath,
  pathLength,
  snap,
  wrapHeading,
  type PathPoint,
  type SavedPath,
} from './pathModel.ts';

export type { PathPoint } from './pathModel.ts';

const HIT_RADIUS_PX = 11;
const HANDLE_RADIUS_PX = 8;
const STORAGE_KEY = 'odyssey-path';
const HISTORY_LIMIT = 100;

type Drag =
  | { kind: 'none' }
  | { kind: 'point'; index: number }
  | { kind: 'heading'; index: number };

export class Planner {
  points: PathPoint[] = [];
  timeout = DEFAULT_TIMEOUT;
  active = false;

  /** fired after every change to the path, undo state or selection */
  onChange: (() => void) | null = null;

  private field: FieldView;
  private listEl: HTMLElement;
  private codeEl: HTMLElement;
  private drag: Drag = { kind: 'none' };
  private dragged = false;
  private selected = -1;

  // undo/redo store serialized snapshots; `last` mirrors the current state
  private past: string[] = [];
  private future: string[] = [];
  private last: string;

  constructor(field: FieldView, listEl: HTMLElement, codeEl: HTMLElement) {
    this.field = field;
    this.listEl = listEl;
    this.codeEl = codeEl;

    const saved = parseSavedPath(localStorage.getItem(STORAGE_KEY));
    if (saved) {
      this.points = saved.points;
      this.timeout = saved.timeout;
    }
    this.last = this.serialize();

    this.bindCanvas();
    this.bindKeys();
    this.refresh();
  }

  // ------------------------------------------------------------------
  // history + persistence
  // ------------------------------------------------------------------

  private serialize(): string {
    const state: SavedPath = { points: this.points, timeout: this.timeout };
    return JSON.stringify(state);
  }

  /** record the current state as an undo step (no-op if nothing changed) */
  private commit(): void {
    const now = this.serialize();
    if (now === this.last) return;
    this.past.push(this.last);
    if (this.past.length > HISTORY_LIMIT) this.past.shift();
    this.future = [];
    this.last = now;
    try {
      localStorage.setItem(STORAGE_KEY, now);
    } catch {
      /* storage full or unavailable: the path just won't persist */
    }
    this.onChange?.();
  }

  private restore(snapshot: string): void {
    const saved = parseSavedPath(snapshot)!;
    this.points = saved.points;
    this.timeout = saved.timeout;
    this.last = snapshot;
    this.selected = Math.min(this.selected, this.points.length - 1);
    try {
      localStorage.setItem(STORAGE_KEY, snapshot);
    } catch {
      /* see commit() */
    }
    this.refresh();
  }

  get canUndo(): boolean {
    return this.past.length > 0;
  }

  get canRedo(): boolean {
    return this.future.length > 0;
  }

  undo(): void {
    const prev = this.past.pop();
    if (prev === undefined) return;
    this.future.push(this.last);
    this.restore(prev);
  }

  redo(): void {
    const next = this.future.pop();
    if (next === undefined) return;
    this.past.push(this.last);
    this.restore(next);
  }

  get length(): number {
    return pathLength(this.points);
  }

  // ------------------------------------------------------------------
  // canvas interaction: add, select, drag, rotate, delete
  // ------------------------------------------------------------------

  private canvasPoint(e: MouseEvent): [number, number] {
    const rect = this.field.canvas.getBoundingClientRect();
    return [e.clientX - rect.left, e.clientY - rect.top];
  }

  private hitTest(px: number, py: number): number {
    for (let i = this.points.length - 1; i >= 0; i--) {
      const [x, y] = this.field.toPx(this.points[i].x, this.points[i].y);
      if (Math.hypot(px - x, py - y) <= HIT_RADIUS_PX) return i;
    }
    return -1;
  }

  private hasHeading(i: number): boolean {
    return i === 0 || this.points[i].kind === 'pose';
  }

  /** the heading handle sits at the tip of the arrival-heading arrow */
  private handleHitTest(px: number, py: number): number {
    for (let i = this.points.length - 1; i >= 0; i--) {
      if (!this.hasHeading(i)) continue;
      const p = this.points[i];
      const [x, y] = this.field.toPx(p.x, p.y);
      const a = ((90 - p.theta) * Math.PI) / 180;
      const hx = x + HEADING_ARM_PX * Math.cos(a);
      const hy = y - HEADING_ARM_PX * Math.sin(a);
      if (Math.hypot(px - hx, py - hy) <= HANDLE_RADIUS_PX) return i;
    }
    return -1;
  }

  private select(index: number): void {
    if (this.selected === index) return;
    this.selected = index;
    this.syncSelection();
    this.onChange?.();
  }

  private bindCanvas(): void {
    const canvas = this.field.canvas;

    canvas.addEventListener('mousedown', (e) => {
      if (!this.active || e.button !== 0) return;
      const [px, py] = this.canvasPoint(e);
      this.dragged = false;

      const handle = this.handleHitTest(px, py);
      if (handle >= 0) {
        this.drag = { kind: 'heading', index: handle };
        this.select(handle);
        return;
      }
      const hit = this.hitTest(px, py);
      this.drag = hit >= 0 ? { kind: 'point', index: hit } : { kind: 'none' };
      if (hit >= 0) this.select(hit);
    });

    canvas.addEventListener('mousemove', (e) => {
      if (!this.active) return;
      const [px, py] = this.canvasPoint(e);

      if (this.drag.kind === 'none') {
        const overHandle = this.handleHitTest(px, py) >= 0;
        canvas.style.cursor = overHandle ? 'crosshair' : this.hitTest(px, py) >= 0 ? 'grab' : '';
        return;
      }

      const p = this.points[this.drag.index];
      if (this.drag.kind === 'point') {
        const [x, y] = this.field.toField(px, py);
        p.x = snap(clampField(x));
        p.y = snap(clampField(y));
        this.updateRowInputs(this.drag.index);
      } else {
        const [cx, cy] = this.field.toPx(p.x, p.y);
        const compass = (Math.atan2(px - cx, cy - py) * 180) / Math.PI;
        p.theta = wrapHeading(Math.round(compass / 5) * 5);
        this.updateRowInputs(this.drag.index);
      }
      this.dragged = true;
      this.renderCode();
    });

    window.addEventListener('mouseup', () => {
      if (this.drag.kind !== 'none' && this.dragged) this.commit();
      this.drag = { kind: 'none' };
    });

    canvas.addEventListener('click', (e) => {
      if (!this.active) return;
      const [px, py] = this.canvasPoint(e);
      // a click that ended a drag, or landed on an existing point, adds nothing
      if (this.dragged || this.hitTest(px, py) >= 0 || this.handleHitTest(px, py) >= 0) {
        this.dragged = false;
        return;
      }
      const [x, y] = this.field.toField(px, py);
      this.addPoint(snap(clampField(x)), snap(clampField(y)));
    });

    // double-click a waypoint to flip it between Point and Pose
    canvas.addEventListener('dblclick', (e) => {
      if (!this.active) return;
      const [px, py] = this.canvasPoint(e);
      const hit = this.hitTest(px, py);
      if (hit <= 0) return;
      const p = this.points[hit];
      p.kind = p.kind === 'pose' ? 'point' : 'pose';
      this.refresh();
      this.commit();
    });

    canvas.addEventListener('contextmenu', (e) => {
      if (!this.active) return;
      const [px, py] = this.canvasPoint(e);
      const hit = this.hitTest(px, py);
      if (hit >= 0) {
        e.preventDefault();
        this.removePoint(hit);
      }
    });
  }

  private bindKeys(): void {
    window.addEventListener('keydown', (e) => {
      if (!this.active) return;
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'TEXTAREA' || tag === 'INPUT' || tag === 'SELECT') return;

      const mod = e.ctrlKey || e.metaKey;
      const key = e.key.toLowerCase();
      if (mod && key === 'z') {
        e.preventDefault();
        if (e.shiftKey) this.redo();
        else this.undo();
      } else if (mod && key === 'y') {
        e.preventDefault();
        this.redo();
      } else if ((e.key === 'Delete' || e.key === 'Backspace') && this.selected >= 0) {
        e.preventDefault();
        this.removePoint(this.selected);
      } else if (e.key === 'Escape') {
        this.select(-1);
      } else if (e.key.startsWith('Arrow') && this.selected >= 0) {
        // nudge the selected waypoint: 0.5" per press, 2" with Shift
        e.preventDefault();
        const step = e.shiftKey ? 2 : 0.5;
        const p = this.points[this.selected];
        if (e.key === 'ArrowLeft') p.x = clampField(p.x - step);
        if (e.key === 'ArrowRight') p.x = clampField(p.x + step);
        if (e.key === 'ArrowUp') p.y = clampField(p.y + step);
        if (e.key === 'ArrowDown') p.y = clampField(p.y - step);
        this.updateRowInputs(this.selected);
        this.renderCode();
        this.commit();
      }
    });
  }

  // ------------------------------------------------------------------
  // model
  // ------------------------------------------------------------------

  addPoint(x: number, y: number): void {
    const point = newPoint(x, y);
    const prev = this.points[this.points.length - 1];
    // face the direction of travel, so switching to Pose needs no fiddling
    if (prev && (prev.x !== x || prev.y !== y)) point.theta = bearing(prev, point);
    this.points.push(point);
    this.selected = this.points.length - 1;
    this.refresh();
    this.commit();
  }

  removePoint(index: number): void {
    this.points.splice(index, 1);
    this.selected = Math.min(this.selected, this.points.length - 1);
    this.refresh();
    this.commit();
  }

  movePoint(index: number, delta: -1 | 1): void {
    const to = index + delta;
    if (to < 0 || to >= this.points.length) return;
    [this.points[index], this.points[to]] = [this.points[to], this.points[index]];
    this.selected = to;
    this.refresh();
    this.commit();
  }

  mirror(axis: 'x' | 'y'): void {
    if (this.points.length === 0) return;
    this.points = mirrorPoints(this.points, axis);
    this.refresh();
    this.commit();
  }

  setTimeoutMs(ms: number): void {
    this.timeout = ms;
    this.renderCode();
  }

  /** persist a timeout edit as its own undo step */
  commitTimeout(): void {
    this.commit();
  }

  clear(): void {
    if (this.points.length === 0) return;
    this.points = [];
    this.selected = -1;
    this.refresh();
    this.commit();
  }

  /** sync the field overlay, the list UI and the generated code */
  refresh(): void {
    this.field.plannerPoints = this.points;
    this.field.plannerSelected = this.selected;
    this.renderList();
    this.renderCode();
    this.onChange?.();
  }

  generateCpp(): string {
    return generateCpp(this.points, this.timeout);
  }

  /**
   * Reverse-engineer pasted Odyssey auto code into waypoints: replaces the
   * current path and returns any parse notes to surface to the user.
   */
  importCode(code: string): string[] {
    const result = parseAutoCode(code);
    if (result.points.length > 0) {
      this.points = result.points;
      this.selected = -1;
      if (result.timeout !== null) this.timeout = Math.max(500, Math.round(result.timeout));
      this.refresh();
      this.commit();
    }
    return result.warnings;
  }

  buildProgram(): (chassis: Chassis) => Promise<void> {
    return buildProgram(this.points, this.timeout);
  }

  // ------------------------------------------------------------------
  // list + code UI
  // ------------------------------------------------------------------

  private rowInputs: { x: HTMLInputElement; y: HTMLInputElement; theta: HTMLInputElement }[] = [];

  private updateRowInputs(index: number): void {
    const row = this.rowInputs[index];
    if (!row) return;
    row.x.value = fmt(this.points[index].x);
    row.y.value = fmt(this.points[index].y);
    row.theta.value = fmt(this.points[index].theta);
  }

  private syncSelection(): void {
    this.field.plannerSelected = this.selected;
    this.listEl.querySelectorAll('.path-item').forEach((el, i) => {
      el.classList.toggle('selected', i === this.selected);
      if (i === this.selected) el.scrollIntoView({ block: 'nearest' });
    });
  }

  private renderList(): void {
    this.listEl.innerHTML = '';
    this.rowInputs = [];

    if (this.points.length === 0) {
      const empty = document.createElement('div');
      empty.className = 'path-empty';
      empty.textContent = 'No waypoints yet — click the field to add the start point.';
      this.listEl.append(empty);
      return;
    }

    this.points.forEach((p, i) => {
      const isStart = i === 0;
      const item = document.createElement('div');
      item.className = `path-item${i === this.selected ? ' selected' : ''}`;
      item.addEventListener('mousedown', () => this.select(i));

      const row = document.createElement('div');
      row.className = 'path-row';
      const sub = document.createElement('div');
      sub.className = 'path-row sub';
      item.append(row, sub);

      const badge = document.createElement('span');
      badge.className = `path-badge${isStart ? ' start' : ''}`;
      badge.textContent = isStart ? 'S' : String(i);
      row.append(badge);

      const num = (
        parent: HTMLElement, value: number, step: number, title: string,
        onInput: (v: number) => void,
      ) => {
        const input = document.createElement('input');
        input.type = 'number';
        input.step = String(step);
        input.value = fmt(value);
        input.title = title;
        input.className = 'mono';
        input.addEventListener('input', () => {
          const v = Number(input.value);
          if (input.value !== '' && Number.isFinite(v)) {
            onInput(v);
            this.renderCode();
          }
        });
        // one undo step per finished edit, not per keystroke
        input.addEventListener('change', () => this.commit());
        parent.append(input);
        return input;
      };

      const xInput = num(row, p.x, 0.5, 'x, inches', (v) => (p.x = clampField(v)));
      const yInput = num(row, p.y, 0.5, 'y, inches', (v) => (p.y = clampField(v)));

      let leadInput: HTMLInputElement | null = null;
      const refreshEnabled = () => {
        theta.disabled = !this.hasHeading(i);
        if (leadInput) leadInput.disabled = p.kind !== 'pose';
      };

      if (isStart) {
        const label = document.createElement('span');
        label.className = 'path-kind-label';
        label.textContent = 'start pose';
        row.append(label);
      } else {
        const kind = document.createElement('select');
        kind.title = 'Motion used to reach this point';
        kind.innerHTML = '<option value="point">Point</option><option value="pose">Pose</option>';
        kind.value = p.kind;
        kind.addEventListener('change', () => {
          p.kind = kind.value as PathPoint['kind'];
          refreshEnabled();
          this.field.plannerPoints = this.points;
          this.renderCode();
          this.commit();
        });
        row.append(kind);
      }

      const theta = num(row, p.theta, 15, 'heading, compass degrees', (v) => (p.theta = wrapHeading(v)));
      theta.classList.add('path-theta');
      this.rowInputs[i] = { x: xInput, y: yInput, theta };

      const del = this.iconButton('×', `Delete waypoint ${isStart ? 'S' : i}`, () => this.removePoint(i));
      del.classList.add('path-del');
      row.append(del);

      // second line: per-point motion options and ordering
      if (!isStart) {
        const speedLabel = document.createElement('label');
        speedLabel.className = 'path-opt';
        speedLabel.append('speed');
        num(speedLabel, p.maxSpeed, 5, 'max speed, 0 to 127', (v) => {
          p.maxSpeed = Math.max(0, Math.min(127, v));
        });
        sub.append(speedLabel);

        const leadLabel = document.createElement('label');
        leadLabel.className = 'path-opt';
        leadLabel.append('lead');
        leadInput = num(leadLabel, p.lead, 0.05, 'carrot lead, 0 to 1 (pose points)', (v) => {
          p.lead = Math.max(0, Math.min(1, v));
        });
        sub.append(leadLabel);

        const rev = document.createElement('label');
        rev.className = 'path-rev';
        rev.title = 'Drive backwards into this point';
        const check = document.createElement('input');
        check.type = 'checkbox';
        check.checked = !p.forwards;
        check.addEventListener('change', () => {
          p.forwards = !check.checked;
          this.renderCode();
          this.commit();
        });
        rev.append(check, document.createTextNode('rev'));
        sub.append(rev);
      }

      const order = document.createElement('span');
      order.className = 'path-order';
      const up = this.iconButton('↑', `Move waypoint ${i} earlier`, () => this.movePoint(i, -1));
      const down = this.iconButton('↓', `Move waypoint ${i} later`, () => this.movePoint(i, 1));
      up.disabled = i === 0;
      down.disabled = i === this.points.length - 1;
      order.append(up, down);
      sub.append(order);

      refreshEnabled();
      this.listEl.append(item);
    });
  }

  private iconButton(label: string, aria: string, onClick: () => void): HTMLButtonElement {
    const btn = document.createElement('button');
    btn.className = 'btn small icon-only';
    btn.textContent = label;
    btn.setAttribute('aria-label', aria);
    btn.title = aria;
    btn.addEventListener('click', onClick);
    return btn;
  }

  private renderCode(): void {
    this.codeEl.textContent = this.generateCpp();
  }
}
