// Application wiring: world lifecycle, program runner, panels.

import { Waypoint } from '../lib/chassis.ts';
import { Pose } from '../lib/pose.ts';
import { Omniwheel } from '../lib/trackingWheel.ts';
import { angleError, degToRad, radToDeg, sanitizeAngle } from '../lib/util.ts';
import { ProgramStopped } from '../sim/clock.ts';
import { World, defaultConfig, type WorldConfig } from '../sim/world.ts';
import { examples } from '../examples.ts';
import { FieldView } from './field.ts';
import { Planner } from './planner.ts';
import type { Chassis } from '../lib/chassis.ts';

const $ = <T extends HTMLElement>(sel: string): T => document.querySelector(sel) as T;

/** the namespace handed to user programs, mirroring `odyssey::` */
const odysseyNs = Object.freeze({
  Waypoint,
  Pose,
  Omniwheel,
  degToRad,
  radToDeg,
  angleError,
  sanitizeAngle,
});

type RunState = 'idle' | 'running' | 'driver';

export class App {
  private config: WorldConfig = defaultConfig();
  private world: World;
  private field: FieldView;
  private planner!: Planner;
  private epoch = 0;
  private state: RunState = 'idle';
  private keys = new Set<string>();
  private speed = 1;
  private carryMs = 0;
  private lastFrame = 0;

  constructor() {
    this.world = this.buildWorld();
    this.field = new FieldView($('#field'));
    this.planner = new Planner(this.field, $('#path-list'), $('#path-code'));
    this.bindControls();
    this.bindEditor();
    this.bindFieldInput();
    this.bindPlanner();
    this.buildGauges();
    this.buildTuning();
    this.loop();
  }

  // ------------------------------------------------------------------
  // world lifecycle
  // ------------------------------------------------------------------

  private buildWorld(): World {
    const world = new World(structuredClone(this.config));
    world.onTick = () => this.driverTick();
    return world;
  }

  /** tear down every running task and start a fresh robot */
  private resetWorld(keepPose: boolean): void {
    this.epoch++;
    const pose = this.world.getOdomPose();
    this.world.dispose();
    this.world = this.buildWorld();
    if (keepPose) this.world.chassis.setPose(pose.x, pose.y, pose.theta);
    this.setState('idle');
  }

  private setState(state: RunState): void {
    this.state = state;
    $('#status-state').textContent =
      state === 'running' ? 'PROGRAM RUNNING' : state === 'driver' ? 'DRIVER CONTROL' : 'IDLE';
  }

  // ------------------------------------------------------------------
  // engine loop: real time -> 10ms sim ticks, then render
  // ------------------------------------------------------------------

  private async loop(): Promise<void> {
    this.lastFrame = performance.now();
    for (;;) {
      const now: number = await new Promise(requestAnimationFrame);
      let budget = (now - this.lastFrame) * this.speed + this.carryMs;
      this.lastFrame = now;
      if (budget > 2000) budget = 2000; // background tab catch-up cap
      let ticks = Math.floor(budget / 10);
      this.carryMs = budget - ticks * 10;
      while (ticks-- > 0) await this.world.step();
      this.render(now);
    }
  }

  // ------------------------------------------------------------------
  // program runner
  // ------------------------------------------------------------------

  private runProgram(): void {
    const code = ($('#code') as unknown as HTMLTextAreaElement).value;
    const AsyncFunction = Object.getPrototypeOf(async function () {}).constructor as new (
      ...args: string[]
    ) => (...fnArgs: unknown[]) => Promise<void>;

    let fn: (...fnArgs: unknown[]) => Promise<void>;
    try {
      fn = new AsyncFunction('chassis', 'odyssey', 'console', 'delay', `'use strict';\n${code}`);
    } catch (err) {
      this.log(`✗ ${String(err)}`, 'error');
      return;
    }
    this.execute(fn, 'program');
  }

  /** reset the world and run an autonomous routine against it */
  private execute(fn: (...fnArgs: unknown[]) => Promise<void>, label: string): void {
    this.resetWorld(false);
    this.clearConsole();

    const myEpoch = this.epoch;
    const world = this.world;
    const consoleProxy = {
      log: (...args: unknown[]) => this.log(args.map(formatValue).join(' ')),
      error: (...args: unknown[]) => this.log(args.map(formatValue).join(' '), 'error'),
      warn: (...args: unknown[]) => this.log(args.map(formatValue).join(' '), 'info'),
    };

    this.setState('running');
    this.log(`▶ ${label} started`, 'info');

    fn(world.chassis, odysseyNs, consoleProxy, (ms: number) => world.clock.delay(ms)).then(
      async () => {
        if (myEpoch !== this.epoch) return;
        // let trailing async motions finish before declaring victory
        try {
          await world.chassis.waitUntilDone();
        } catch {
          return;
        }
        if (myEpoch !== this.epoch) return;
        this.log(`✓ finished at t = ${(world.clock.millis() / 1000).toFixed(2)} s`, 'ok');
        this.setState('idle');
      },
      (err: unknown) => {
        if (myEpoch !== this.epoch) return;
        if (err instanceof ProgramStopped) return;
        this.log(`✗ ${err instanceof Error ? err.message : String(err)}`, 'error');
        this.setState('idle');
      },
    );
  }

  private stopProgram(): void {
    this.resetWorld(true);
    this.log('■ stopped', 'info');
  }

  // ------------------------------------------------------------------
  // driver control
  // ------------------------------------------------------------------

  private driverTick(): void {
    if (!($('#driver') as unknown as HTMLInputElement).checked) return;
    if (this.world.chassis.isInMotion()) return;
    const k = this.keys;
    const throttle = (k.has('w') || k.has('arrowup') ? 127 : 0) + (k.has('s') || k.has('arrowdown') ? -127 : 0);
    const turn = (k.has('d') || k.has('arrowright') ? 90 : 0) + (k.has('a') || k.has('arrowleft') ? -90 : 0);
    this.world.chassis.arcade(throttle, turn);
  }

  // ------------------------------------------------------------------
  // UI bindings
  // ------------------------------------------------------------------

  private bindControls(): void {
    $('#theme-toggle').addEventListener('click', () => {
      const root = document.documentElement;
      const next = root.dataset.theme === 'dark' ? 'light' : 'dark';
      root.dataset.theme = next;
      localStorage.setItem('odyssey-theme', next);
    });

    $('#btn-run').addEventListener('click', () => this.runProgram());
    $('#btn-stop').addEventListener('click', () => this.stopProgram());
    $('#btn-reset').addEventListener('click', () => {
      this.resetWorld(false);
      this.log('⟲ reset', 'info');
    });

    ($('#speed') as unknown as HTMLSelectElement).addEventListener('change', (e) => {
      this.speed = Number((e.target as HTMLSelectElement).value);
    });

    ($('#driver') as unknown as HTMLInputElement).addEventListener('change', (e) => {
      const on = (e.target as HTMLInputElement).checked;
      if (on && this.state === 'idle') this.setState('driver');
      if (!on && this.state === 'driver') {
        this.world.chassis.tank(0, 0);
        this.setState('idle');
      }
    });

    ($('#noise') as unknown as HTMLInputElement).addEventListener('change', (e) => {
      const on = (e.target as HTMLInputElement).checked;
      this.config.noise.enabled = on;
      this.world.config.noise.enabled = on; // applies live
      this.syncTuningInputs();
    });

    ($('#ghost') as unknown as HTMLInputElement).addEventListener('change', (e) => {
      this.field.showGhost = (e.target as HTMLInputElement).checked;
    });

    ($('#trails') as unknown as HTMLInputElement).addEventListener('change', (e) => {
      this.field.showTrails = (e.target as HTMLInputElement).checked;
    });

    window.addEventListener('keydown', (e) => {
      const tag = (e.target as HTMLElement).tagName;
      if (tag === 'TEXTAREA' || tag === 'INPUT' || tag === 'SELECT') return;
      this.keys.add(e.key.toLowerCase());
      if (e.key.startsWith('Arrow')) e.preventDefault();
    });
    window.addEventListener('keyup', (e) => this.keys.delete(e.key.toLowerCase()));
    window.addEventListener('blur', () => this.keys.clear());

    // tabs
    for (const btn of document.querySelectorAll<HTMLButtonElement>('.tab-btn')) {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.tab-btn').forEach((b) => {
          b.classList.remove('active');
          b.setAttribute('aria-selected', 'false');
        });
        btn.classList.add('active');
        btn.setAttribute('aria-selected', 'true');
        for (const name of ['auto', 'path', 'tune']) {
          $(`#tab-${name}`).classList.toggle('hidden', btn.dataset.tab !== name);
        }
        this.planner.active = btn.dataset.tab === 'path';
        if (this.planner.active) {
          // the click-target quick actions would fight with point placement
          this.field.target = null;
          $('#quick').classList.add('hidden');
        }
      });
    }
  }

  private bindEditor(): void {
    const select = $('#examples') as unknown as HTMLSelectElement;
    const code = $('#code') as unknown as HTMLTextAreaElement;
    for (let i = 0; i < examples.length; i++) {
      const option = document.createElement('option');
      option.value = String(i);
      option.textContent = examples[i].name;
      select.append(option);
    }
    select.addEventListener('change', () => {
      code.value = examples[Number(select.value)].code;
    });
    code.value = examples[0].code;

    // tab key indents instead of leaving the editor
    code.addEventListener('keydown', (e) => {
      if (e.key === 'Tab') {
        e.preventDefault();
        const { selectionStart, selectionEnd, value } = code;
        code.value = `${value.slice(0, selectionStart)}  ${value.slice(selectionEnd)}`;
        code.selectionStart = code.selectionEnd = selectionStart + 2;
      }
    });
  }

  private bindFieldInput(): void {
    const canvas = this.field.canvas;
    const quick = $('#quick');
    const coords = $('#quick-coords');

    canvas.addEventListener('mousemove', (e) => {
      const rect = canvas.getBoundingClientRect();
      const [x, y] = this.field.toField(e.clientX - rect.left, e.clientY - rect.top);
      $('#status-coords').textContent = `${x.toFixed(1)}", ${y.toFixed(1)}"`;
    });
    canvas.addEventListener('mouseleave', () => {
      $('#status-coords').textContent = '—';
    });

    canvas.addEventListener('click', (e) => {
      if (this.planner.active) return;
      const rect = canvas.getBoundingClientRect();
      let [x, y] = this.field.toField(e.clientX - rect.left, e.clientY - rect.top);
      x = Math.max(-72, Math.min(72, x));
      y = Math.max(-72, Math.min(72, y));
      this.field.target = { x, y };
      coords.textContent = `(${x.toFixed(1)}, ${y.toFixed(1)})`;
      quick.classList.remove('hidden');
    });

    const target = () => this.field.target!;
    const thetaInput = $('#qa-theta') as unknown as HTMLInputElement;

    $('#qa-turn').addEventListener('click', () => {
      void this.world.chassis.turnToPoint(target().x, target().y, 3000);
    });
    $('#qa-move').addEventListener('click', () => {
      void this.world.chassis.moveToPoint(target().x, target().y, 6000);
    });
    $('#qa-pose').addEventListener('click', () => {
      void this.world.chassis.moveToPose(target().x, target().y, Number(thetaInput.value), 8000);
    });
    $('#qa-clear').addEventListener('click', () => {
      this.field.target = null;
      quick.classList.add('hidden');
    });
  }

  private bindPlanner(): void {
    const timeoutInput = $('#path-timeout') as unknown as HTMLInputElement;
    timeoutInput.addEventListener('input', () => {
      const v = Number(timeoutInput.value);
      if (Number.isFinite(v) && v > 0) this.planner.setTimeoutMs(v);
    });
    timeoutInput.addEventListener('change', () => this.planner.commitTimeout());

    const undo = $('#path-undo') as unknown as HTMLButtonElement;
    const redo = $('#path-redo') as unknown as HTMLButtonElement;
    const stats = $('#path-stats');
    const run = $('#path-run') as unknown as HTMLButtonElement;
    // keep the toolbar and timeout field in step with the planner's state
    this.planner.onChange = () => {
      run.disabled = this.planner.points.length < 2;
      undo.disabled = !this.planner.canUndo;
      redo.disabled = !this.planner.canRedo;
      timeoutInput.value = String(this.planner.timeout);
      const n = this.planner.points.length;
      stats.textContent =
        n === 0 ? '' : `${n} pt${n === 1 ? '' : 's'} · ${this.planner.length.toFixed(1)}"`;
    };
    this.planner.onChange();

    undo.addEventListener('click', () => this.planner.undo());
    redo.addEventListener('click', () => this.planner.redo());
    $('#path-mirror-x').addEventListener('click', () => this.planner.mirror('x'));
    $('#path-mirror-y').addEventListener('click', () => this.planner.mirror('y'));
    $('#path-clear').addEventListener('click', () => this.planner.clear());

    $('#path-run').addEventListener('click', () => {
      if (this.planner.points.length < 2) {
        this.log('✗ add a start point and at least one waypoint first', 'error');
        return;
      }
      const program = this.planner.buildProgram();
      this.execute((chassis: unknown) => program(chassis as Chassis), 'path');
    });

    // reverse-engineer pasted auto code into the planner
    const importBtn = $('#path-import-btn') as unknown as HTMLButtonElement;
    const importCode = $('#path-import-code') as unknown as HTMLTextAreaElement;
    const importNotes = $('#path-import-notes');
    importBtn.addEventListener('click', () => {
      const code = importCode.value;
      if (!code.trim()) {
        importNotes.textContent = 'Paste your autonomous code above first.';
        return;
      }
      const warnings = this.planner.importCode(code);
      const n = this.planner.points.length;
      const noteLines = [
        n === 0
          ? '✗ No usable chassis motions found.'
          : n === 1
            ? '✓ Start pose imported.'
            : `✓ ${n} waypoints imported — shown on the field and sanitized below.`,
        ...warnings.map((w) => `⚠ ${w}`),
      ];
      importNotes.textContent = noteLines.join('\n');
      this.clearConsole();
      for (const line of noteLines) {
        this.log(line.replace(/^([✓✗⚠]) /, '$1 '), line.startsWith('✗') ? 'error' : line.startsWith('⚠') ? 'info' : 'ok');
      }
      if (n > 0) this.log('▶ press Simulate to run the reconstructed path', 'info');
    });

    $('#path-copy').addEventListener('click', async () => {
      const btn = $('#path-copy');
      try {
        await navigator.clipboard.writeText(this.planner.generateCpp());
        btn.textContent = 'Copied!';
      } catch {
        btn.textContent = 'Copy failed';
      }
      setTimeout(() => (btn.textContent = 'Copy'), 1500);
    });
  }

  // ------------------------------------------------------------------
  // gauges
  // ------------------------------------------------------------------

  private gaugeValues: Record<string, HTMLElement> = {};
  private gaugeBars: Record<string, HTMLElement> = {};

  private buildGauges(): void {
    const wrap = $('#gauges');
    const make = (id: string, label: string, bar = false) => {
      const el = document.createElement('div');
      el.className = 'gauge';
      el.innerHTML = `<div class="label">${label}</div><div class="value">—</div>${
        bar ? '<div class="bar"><div class="fill"></div><div class="center"></div></div>' : ''
      }`;
      wrap.append(el);
      this.gaugeValues[id] = el.querySelector('.value')!;
      if (bar) this.gaugeBars[id] = el.querySelector('.fill')!;
    };
    make('x', 'X (in)');
    make('y', 'Y (in)');
    make('theta', 'Heading');
    make('drift', 'Odom drift');
    make('speed', 'Speed');
    make('left', 'Left power', true);
    make('right', 'Right power', true);
  }

  private updateGauges(): void {
    const odom = this.world.getOdomPose();
    const truth = this.world.getTruePose();
    const robot = this.world.robot;

    this.gaugeValues.x.textContent = odom.x.toFixed(1);
    this.gaugeValues.y.textContent = odom.y.toFixed(1);
    this.gaugeValues.theta.textContent = `${sanitizeAngle(odom.theta, false).toFixed(1)}°`;
    this.gaugeValues.drift.textContent = `${Math.hypot(odom.x - truth.x, odom.y - truth.y).toFixed(2)}"`;
    this.gaugeValues.speed.textContent = `${((robot.vLeft + robot.vRight) / 2).toFixed(1)} in/s`;

    const setBar = (id: string, cmd: number | null) => {
      const power = cmd ?? 0;
      this.gaugeValues[id].textContent = cmd === null ? 'brake' : String(Math.round(power));
      const fill = this.gaugeBars[id];
      const pct = (Math.abs(power) / 127) * 50;
      fill.style.left = power < 0 ? `${50 - pct}%` : '50%';
      fill.style.width = `${pct}%`;
    };
    setBar('left', robot.leftMotors.command);
    setBar('right', robot.rightMotors.command);

    $('#status-time').textContent = `t = ${(this.world.clock.millis() / 1000).toFixed(2)} s`;
  }

  // ------------------------------------------------------------------
  // tuning panel
  // ------------------------------------------------------------------

  private buildTuning(): void {
    const wrap = $('#tuning');
    wrap.innerHTML = '';

    const pidFields: [string, string][] = [
      ['kP', 'kP'], ['kI', 'kI'], ['kD', 'kD'], ['windupRange', 'Windup range'],
      ['smallError', 'Small error'], ['smallErrorTimeout', 'Small timeout (ms)'],
      ['largeError', 'Large error'], ['largeErrorTimeout', 'Large timeout (ms)'],
      ['slew', 'Slew / 10ms'],
    ];

    const groups: { title: string; fields: [string, string, 'number' | 'checkbox'][] }[] = [
      {
        title: 'Drivetrain',
        fields: [
          ['trackWidth', 'Track width (in)', 'number'],
          ['wheelDiameter', 'Wheel diameter (in)', 'number'],
          ['rpm', 'Drive RPM', 'number'],
          ['horizontalDrift', 'Horizontal drift', 'number'],
          ['motorTau', 'Motor response τ (s)', 'number'],
          ['robotWidth', 'Robot width (in)', 'number'],
          ['robotLength', 'Robot length (in)', 'number'],
        ],
      },
      {
        title: 'Lateral PID (inches)',
        fields: pidFields.map(([k, l]) => [`lateral.${k}`, l, 'number'] as [string, string, 'number']),
      },
      {
        title: 'Angular PID (degrees)',
        fields: pidFields.map(([k, l]) => [`angular.${k}`, l, 'number'] as [string, string, 'number']),
      },
      {
        title: 'Odometry sensors',
        fields: [
          ['vertical.enabled', 'Vertical wheel', 'checkbox'],
          ['vertical.diameter', '↳ diameter (in)', 'number'],
          ['vertical.offset', '↳ offset (in)', 'number'],
          ['horizontal.enabled', 'Horizontal wheel', 'checkbox'],
          ['horizontal.diameter', '↳ diameter (in)', 'number'],
          ['horizontal.offset', '↳ offset (in)', 'number'],
          ['useImu', 'IMU', 'checkbox'],
        ],
      },
      {
        title: 'Sensor noise',
        fields: [
          ['noise.enabled', 'Enabled', 'checkbox'],
          ['noise.wheelSlip', 'Wheel slip (fraction)', 'number'],
          ['noise.imuDrift', 'IMU drift (°/s)', 'number'],
          ['noise.imuNoise', 'IMU noise (°)', 'number'],
        ],
      },
    ];

    for (const group of groups) {
      const div = document.createElement('div');
      div.className = 'tune-group';
      div.innerHTML = `<h3>${group.title}</h3>`;
      const grid = document.createElement('div');
      grid.className = 'tune-grid';
      for (const [path, label, type] of group.fields) {
        const field = document.createElement('label');
        field.className = 'tune-field';
        field.innerHTML = `<span>${label}</span><input type="${type}" data-path="${path}" step="any" />`;
        grid.append(field);
      }
      div.append(grid);
      wrap.append(div);
    }

    this.syncTuningInputs();

    $('#btn-apply').addEventListener('click', () => {
      for (const input of wrap.querySelectorAll<HTMLInputElement>('input[data-path]')) {
        const value = input.type === 'checkbox' ? input.checked : Number(input.value);
        if (input.type !== 'checkbox' && !Number.isFinite(value as number)) continue;
        setPath(this.config, input.dataset.path!, value);
      }
      ($('#noise') as unknown as HTMLInputElement).checked = this.config.noise.enabled;
      this.resetWorld(false);
      this.log('⚙ configuration applied, robot rebuilt', 'info');
    });

    $('#btn-defaults').addEventListener('click', () => {
      this.config = defaultConfig();
      this.syncTuningInputs();
      ($('#noise') as unknown as HTMLInputElement).checked = false;
      this.resetWorld(false);
      this.log('⚙ defaults restored', 'info');
    });
  }

  private syncTuningInputs(): void {
    for (const input of document.querySelectorAll<HTMLInputElement>('#tuning input[data-path]')) {
      const value = getPath(this.config, input.dataset.path!);
      if (input.type === 'checkbox') input.checked = Boolean(value);
      else input.value = String(value);
    }
  }

  // ------------------------------------------------------------------
  // console + render
  // ------------------------------------------------------------------

  private log(message: string, kind: 'log' | 'info' | 'error' | 'ok' = 'log'): void {
    const consoleEl = $('#console');
    const entry = document.createElement('div');
    entry.className = `entry ${kind === 'log' ? '' : kind}`;
    entry.textContent = `[${(this.world.clock.millis() / 1000).toFixed(2)}] ${message}`;
    consoleEl.append(entry);
    consoleEl.scrollTop = consoleEl.scrollHeight;
  }

  private clearConsole(): void {
    $('#console').innerHTML = '';
  }

  private render(timeMs: number): void {
    this.field.render(this.world, timeMs);
    this.updateGauges();
  }
}

function formatValue(value: unknown): string {
  if (typeof value === 'object' && value !== null) {
    try {
      return JSON.stringify(value);
    } catch {
      return String(value);
    }
  }
  return String(value);
}

function getPath(obj: unknown, path: string): unknown {
  let current: unknown = obj;
  for (const key of path.split('.')) current = (current as Record<string, unknown>)[key];
  return current;
}

function setPath(obj: unknown, path: string, value: unknown): void {
  const keys = path.split('.');
  let current = obj as Record<string, unknown>;
  for (const key of keys.slice(0, -1)) current = current[key] as Record<string, unknown>;
  current[keys[keys.length - 1]] = value;
}
