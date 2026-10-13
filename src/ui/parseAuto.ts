// Reverse-engineer pasted Odyssey auto code into planner waypoints. Odyssey auto code into planner waypoints.
// Pure string -> data model, no DOM, so it can be tested headlessly in tests/smoke.ts.
//
// Understands the same calls the code editor / generated C++ use:
//   chassis.setPose(x, y, theta)            -> start pose
//   chassis.moveToPoint(x, y, ...)          -> point waypoint
//   chassis.moveToPose(x, y, theta, ...)    -> pose waypoint
//   chassis.turnToHeading(theta, ...)       -> sets the previous waypoint's heading
//   chassis.turnToPoint(x, y, ...)          -> sets the previous waypoint's heading
//   chassis.follow([new Waypoint(x, y, s)...], ...) -> run of point waypoints
//
// Both the JS editor syntax (`chassis.moveToPose(0, 0, 90, 5000, { lead: 0.6 })`)
// and the C++ syntax with designated initializers
// (`chassis.moveToPose(0, 0, 90, 5000, {.lead = 0.6});`) parse.

import {
  DEFAULT_LEAD,
  DEFAULT_MAX_SPEED,
  bearing,
  clampField,
  newPoint,
  wrapHeading,
  type PathPoint,
} from './pathModel.ts';

export interface ParsedAuto {
  points: PathPoint[];
  /** ms timeout taken from the motion calls, or null if none were found */
  timeout: number | null;
  warnings: string[];
}

/** calls we know how to read; anything else in the pasted code is ignored */
const CALL = /chassis\s*\.\s*(setPose|moveToPoint|moveToPose|turnToHeading|turnToPoint|swingToHeading|swingToPoint|follow)\s*\(/gi;

const NUM = /^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?(?:_deg)?$/;

/** strip comments so they can never break match-up of parens */
function stripComments(code: string): string {
  return code
    .replace(/\/\*[\s\S]*?\*\//g, ' ')
    .replace(/\/\/.*$/gm, ' ');
}

/** scan forward from an opening paren to the matching close, respecting strings */
function readBalanced(code: string, openAt: number): { inner: string; end: number; ok: boolean } {
  let depth = 1;
  let str: string | null = null;
  let i = openAt + 1;
  while (i < code.length) {
    const c = code[i];
    if (str) {
      if (c === '\\') i++;
      else if (c === str) str = null;
    } else if (c === '"' || c === "'" || c === '`') {
      str = c;
    } else if (c === '(' || c === '[' || c === '{') {
      depth++;
    } else if (c === ')' || c === ']' || c === '}') {
      depth--;
      if (depth === 0) return { inner: code.slice(openAt + 1, i), end: i, ok: true };
    }
    i++;
  }
  return { inner: code.slice(openAt + 1), end: code.length, ok: false };
}

/** split an argument list on top-level commas (brackets/braces/strings protected) */
function splitArgs(arg: string): string[] {
  const parts: string[] = [];
  let depth = 0;
  let str: string | null = null;
  let cur = '';
  for (let i = 0; i < arg.length; i++) {
    const c = arg[i];
    if (str) {
      if (c === '\\') {
        cur += c + (arg[i + 1] ?? '');
        i++;
      } else {
        if (c === str) str = null;
        cur += c;
      }
    } else if (c === '"' || c === "'" || c === '`') {
      str = c;
      cur += c;
    } else if (c === '(' || c === '[' || c === '{') {
      depth++;
      cur += c;
    } else if (c === ')' || c === ']' || c === '}') {
      depth--;
      cur += c;
    } else if (c === ',' && depth === 0) {
      parts.push(cur);
      cur = '';
    } else {
      cur += c;
    }
  }
  if (cur.trim() !== '') parts.push(cur);
  return parts.map((p) => p.trim());
}

function num(arg: string | undefined): number | null {
  if (arg === undefined) return null;
  const v = NUM.test(arg.trim()) ? Number(arg.replace(/_deg$/i, '')) : NaN;
  return Number.isFinite(v) ? v : null;
}

/**
 * Pull key: value / .key = value pairs out of the options-object arguments
 * of a call (anything after the positional parameters). Handles both
 * `{ lead: 0.6 }` (JS) and `{.lead = 0.6}` (C++ designated initializers).
 */
function parseOptions(args: string[], from: number): Record<string, number | boolean> {
  const opts: Record<string, number | boolean> = {};
  for (const arg of args.slice(from)) {
    const re = /(?:^|[{,]\s*)\.?([A-Za-z_]\w*)\s*(?:=|:)\s*([^,}]+)/g;
    let m: RegExpExecArray | null;
    while ((m = re.exec(arg))) {
      const [, key, raw] = m;
      const value = raw.trim();
      if (value === 'true') opts[key] = true;
      else if (value === 'false') opts[key] = false;
      else if (NUM.test(value)) opts[key] = Number(value.replace(/_deg$/i, ''));
    }
  }
  return opts;
}

function makeStart(x: number, y: number, theta: number): PathPoint {
  const p = newPoint(clampField(x), clampField(y));
  p.kind = 'pose';
  p.theta = wrapHeading(theta);
  return p;
}

function makeWaypoint(x: number, y: number, prev: PathPoint | undefined): PathPoint {
  const p = newPoint(clampField(x), clampField(y));
  if (prev && (prev.x !== p.x || prev.y !== p.y)) p.theta = bearing(prev, p);
  return p;
}

/** most common timeout across the motions (callers pass one value to the code generator) */
function pickTimeout(timeouts: number[], warnings: string[]): number | null {
  if (timeouts.length === 0) return null;
  const rounds = Math.min(...timeouts);
  if (Math.max(...timeouts) !== rounds) {
    warnings.push(`motions use different timeouts; generated path uses the shortest (${rounds} ms)`);
  }
  return rounds;
}

export function parseAutoCode(source: string): ParsedAuto {
  const code = stripComments(source);
  const points: PathPoint[] = [];
  const timeouts: number[] = [];
  const warnings: string[] = [];
  let sawSetPose = false;
  let sawMotion = false;

  CALL.lastIndex = 0;
  for (let m = CALL.exec(code); m !== null; m = CALL.exec(code)) {
    const name = m[1].toLowerCase();
    const open = m.index + m[0].length - 1;
    const { inner, ok } = readBalanced(code, open);
    if (!ok) {
      warnings.push(`couldn't read chassis.${name}(…) — unbalanced parentheses; skipped`);
      continue;
    }
    const args = splitArgs(inner);

    switch (name) {
      case 'setpose': {
        const [x, y, theta] = [num(args[0]), num(args[1]), num(args[2] ?? '0')];
        if (x === null || y === null || theta === null) {
          warnings.push('setPose: could not evaluate coordinates; skipped');
          break;
        }
        points[0] = makeStart(x, y, theta);
        sawSetPose = true;
        break;
      }

      case 'movetopoint':
      case 'movetopose': {
        const pose = name === 'movetopose';
        const x = num(args[0]);
        const y = num(args[1]);
        const theta = pose ? num(args[2] ?? '0') : 0;
        if (x === null || y === null || theta === null) {
          warnings.push(`moveTo${pose ? 'Pose' : 'Point'}: could not evaluate coordinates; skipped`);
          continue;
        }
        const opts = parseOptions(args, pose ? 4 : 3);
        const t = num(args[pose ? 3 : 2]);
        if (t !== null) timeouts.push(t);

        let p: PathPoint;
        if (!sawSetPose && !sawMotion && points.length === 0) {
          // no setPose in the pasted code: the first target doubles as the
          // start pose in the reconstructed path (the robot's real start
          // position is unknowable from the code alone)
          p = makeStart(x, y, theta);
          warnings.push('no setPose() found — using the first motion’s target as the start pose');
        } else {
          p = makeWaypoint(x, y, points[points.length - 1]);
          if (pose) {
            p.kind = 'pose';
            p.theta = wrapHeading(theta);
          }
        }
        // options apply to the start point too (the first motion drives out
        // of it), whether it is an explicit setPose or synthesized above
        if (pose) {
          p.lead = typeof opts.lead === 'number' ? Math.max(0, Math.min(1, opts.lead)) : DEFAULT_LEAD;
        }
        if (opts.forwards === false) p.forwards = false;
        if (typeof opts.maxSpeed === 'number') p.maxSpeed = Math.max(0, Math.min(127, opts.maxSpeed));
        else if (typeof opts.minSpeed === 'number') p.maxSpeed = Math.max(0, Math.min(127, opts.minSpeed));
        points.push(p);
        sawMotion = true;
        break;
      }

      case 'turntoheading':
      case 'swingtoheading': {
        const theta = num(args[0]);
        if (theta === null) {
          warnings.push(`${name}: could not evaluate heading; skipped`);
          break;
        }
        const t = num(args[1]);
        if (t !== null) timeouts.push(t);
        const prev = points[points.length - 1];
        if (prev) {
          prev.kind = 'pose';
          prev.theta = wrapHeading(theta);
          if (name === 'swingtoheading') {
            warnings.push('swing turn recorded as a plain heading change (no sideslip in the path model)');
          }
        } else {
          warnings.push(`${name} before any motion — no waypoint to attach the heading to; skipped`);
        }
        break;
      }

      case 'turntopoint':
      case 'swingtopoint': {
        const [x, y] = [num(args[0]), num(args[1])];
        if (x === null || y === null) {
          warnings.push(`${name}: could not evaluate coordinates; skipped`);
          break;
        }
        const prev = points[points.length - 1];
        if (prev) {
          prev.kind = 'pose';
          prev.theta = bearing(prev, { x, y });
          if (name === 'swingtopoint') {
            warnings.push('swing turn recorded as a plain turn (no sideslip in the path model)');
          }
        } else {
          warnings.push(`${name} before any motion — no waypoint to attach the heading to; skipped`);
        }
        break;
      }

      case 'follow': {
        if (!(/\[[\s\S]*\]/.test(args[0] ?? ''))) {
          warnings.push('follow: waypoint list is not a literal array — cannot extract it; skipped');
          break;
        }
        const waypointRe = /\bnew\s+(?:(?:odyssey|chassis)\.)?Waypoint\s*\(/gi;
        const pathCode = args[0].match(/\[[\s\S]*\]/)![0];
        let first = true;
        let count = 0;
        for (let w = waypointRe.exec(pathCode); w !== null; w = waypointRe.exec(pathCode)) {
          const { inner } = readBalanced(pathCode, w.index + w[0].length - 1);
          const [fx, fy, fspeed] = splitArgs(inner).map((a) => num(a));
          if (fx === null || fy === null) continue;
          count++;
          // the first waypoint is usually where setPose put the robot
          if (first && points.length > 0) {
            const start = points[0];
            if (Math.hypot(fx - start.x, fy - start.y) < 1.5) {
              if (fspeed !== null) start.maxSpeed = Math.max(0, Math.min(127, fspeed));
              first = false;
              continue;
            }
          }
          first = false;
          const prev = points[points.length - 1];
          const p = makeWaypoint(fx, fy, prev);
          p.maxSpeed = fspeed !== null ? Math.max(0, Math.min(127, fspeed)) : DEFAULT_MAX_SPEED;
          points.push(p);
        }
        if (count === 0) {
          warnings.push('follow: path array contains no literal Waypoint(…) entries; skipped');
        } else if (!sawSetPose && !sawMotion) {
          warnings.push('no setPose() found — using the first motion’s target as the start pose');
        }
        const timeout = num(args[2]);
        if (timeout !== null) timeouts.push(timeout);
        sawMotion = sawMotion || count > 0;
        break;
      }
    }
  }

  // a stray start with no motion is fine (drag or re-time it in the planner);
  // everything has been validated above, so just finalize the timeout
  const timeout = pickTimeout(timeouts, warnings);

  return { points, timeout, warnings };
}
