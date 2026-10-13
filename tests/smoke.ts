// Headless verification that the ported library drives the simulated robot
// to its targets. Run with: node tests/smoke.ts

import { Waypoint } from '../src/lib/chassis.ts';
import { World, defaultConfig } from '../src/sim/world.ts';
import {
  bearing, buildProgram, generateCpp, mirrorPoints, newPoint, parseSavedPath, type PathPoint,
} from '../src/ui/pathModel.ts';
import { parseAutoCode } from '../src/ui/parseAuto.ts';

let failures = 0;

function check(name: string, actual: number, expected: number, tolerance: number): void {
  const ok = Math.abs(actual - expected) <= tolerance;
  if (!ok) failures++;
  console.log(
    `${ok ? 'PASS' : 'FAIL'}  ${name}: ${actual.toFixed(2)} (want ${expected} ± ${tolerance})`,
  );
}

function angleDiff(a: number, b: number): number {
  let d = (a - b) % 360;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  return d;
}

/** run an autonomous program, ticking the world until it finishes */
async function run(world: World, program: () => Promise<void>): Promise<number> {
  let done = false;
  let error: unknown = null;
  program().then(
    () => (done = true),
    (e) => {
      error = e;
      done = true;
    },
  );
  let ticks = 0;
  while (!done && ticks < 30000) {
    await world.step();
    ticks++;
  }
  if (error) throw error;
  if (!done) throw new Error('program did not finish within 5 sim minutes');
  return ticks;
}

// ---------------------------------------------------------------------------
// ideal sensors: odometry should match ground truth almost exactly
// ---------------------------------------------------------------------------
{
  const world = new World(defaultConfig());
  const chassis = world.chassis;

  await run(world, async () => {
    chassis.setPose(0, 0, 0);

    await chassis.moveToPoint(0, 24, 4000, {}, false);
    check('moveToPoint y', chassis.getPose().y, 24, 1.5);
    check('moveToPoint x', chassis.getPose().x, 0, 1.5);

    await chassis.turnToHeading(90, 3000, {}, false);
    check('turnToHeading 90', angleDiff(chassis.getPose().theta, 90), 0, 2);

    await chassis.moveToPose(24, 48, 0, 6000, { lead: 0.6 }, false);
    check('moveToPose x', chassis.getPose().x, 24, 2.5);
    check('moveToPose y', chassis.getPose().y, 48, 2.5);
    check('moveToPose heading', angleDiff(chassis.getPose().theta, 0), 0, 8);

    await chassis.swingToHeading(90, 'left', 3000, {}, false);
    check('swingToHeading 90', angleDiff(chassis.getPose().theta, 90), 0, 3);

    await chassis.turnToPoint(0, 0, 3000, {}, false);
    const toOrigin = 90 - (Math.atan2(0 - chassis.getPose().y, 0 - chassis.getPose().x) * 180) / Math.PI;
    check('turnToPoint heading', angleDiff(chassis.getPose().theta, toOrigin), 0, 3);
  });

  // odometry vs ground truth with ideal sensors (centidegree quantization only)
  const odom = world.getOdomPose();
  const truth = world.getTruePose();
  check('odom x error (ideal)', odom.x - truth.x, 0, 0.5);
  check('odom y error (ideal)', odom.y - truth.y, 0, 0.5);
  check('odom heading error (ideal)', angleDiff(odom.theta, truth.theta), 0, 1);
}

// ---------------------------------------------------------------------------
// async motions + waitUntil/waitUntilDone, the C++ usage idiom
// ---------------------------------------------------------------------------
{
  const world = new World(defaultConfig());
  const chassis = world.chassis;

  await run(world, async () => {
    chassis.setPose(0, 0, 0);
    await chassis.moveToPoint(0, 36, 5000); // async: returns once started
    await chassis.waitUntil(18);
    const midway = chassis.getPose().y;
    check('waitUntil fires midway', midway > 12 && midway < 30 ? 1 : 0, 1, 0);
    await chassis.waitUntilDone();
    check('async moveToPoint y', chassis.getPose().y, 36, 1.5);
  });
}

// ---------------------------------------------------------------------------
// pure pursuit
// ---------------------------------------------------------------------------
{
  const world = new World(defaultConfig());
  const chassis = world.chassis;

  await run(world, async () => {
    chassis.setPose(0, 0, 0);
    const path = [
      new Waypoint(0, 0, 80),
      new Waypoint(0, 18, 80),
      new Waypoint(8, 32, 70),
      new Waypoint(24, 40, 60),
      new Waypoint(32, 40, 50),
      new Waypoint(38, 40, 40),
      new Waypoint(40, 40, 30),
    ];
    await chassis.follow(path, 12, 10000, true, false);
    check('pure pursuit end x', chassis.getPose().x, 40, 6);
    check('pure pursuit end y', chassis.getPose().y, 40, 6);
  });
}

// ---------------------------------------------------------------------------
// noisy sensors: odometry should drift, but only a little
// ---------------------------------------------------------------------------
{
  const config = defaultConfig();
  config.noise.enabled = true;
  const world = new World(config);
  const chassis = world.chassis;

  await run(world, async () => {
    chassis.setPose(0, 0, 0);
    await chassis.moveToPoint(0, 36, 5000, {}, false);
    await chassis.turnToHeading(90, 3000, {}, false);
    await chassis.moveToPoint(36, 36, 5000, {}, false);
  });

  const odom = world.getOdomPose();
  const truth = world.getTruePose();
  const drift = Math.hypot(odom.x - truth.x, odom.y - truth.y);
  console.log(`INFO  odom drift with noise: ${drift.toFixed(2)}"`);
  check('noisy odom drift bounded', drift, 0, 4);
}

// ---------------------------------------------------------------------------
// path planner model: C++ generation, mirroring, persistence, simulation
// ---------------------------------------------------------------------------
{
  const pts: PathPoint[] = [newPoint(-48, -48), newPoint(0, -24), newPoint(24, 24)];
  pts[0].theta = 45;
  pts[1].kind = 'pose';
  pts[1].theta = 90;
  pts[1].lead = 0.8;
  pts[2].forwards = false;
  pts[2].maxSpeed = 80;

  const cpp = generateCpp(pts, 4000);
  const expect = (name: string, ok: boolean) => {
    if (!ok) failures++;
    console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}`);
  };
  expect('cpp: setPose', cpp.includes('chassis.setPose(-48, -48, 45);'));
  expect('cpp: pose lead', cpp.includes('chassis.moveToPose(0, -24, 90, 4000, {.lead = 0.8});'));
  expect(
    'cpp: reverse + speed',
    cpp.includes('chassis.moveToPoint(24, 24, 4000, {.forwards = false, .maxSpeed = 80});'),
  );
  expect('cpp: default lead omitted', !generateCpp([pts[0], { ...pts[1], lead: 0.6 }], 4000).includes('lead'));
  expect(
    'bearing: compass directions',
    bearing({ x: 0, y: 0 }, { x: 0, y: 5 }) === 0 &&
      bearing({ x: 0, y: 0 }, { x: 5, y: 0 }) === 90 &&
      bearing({ x: 0, y: 0 }, { x: 0, y: -5 }) === 180 &&
      bearing({ x: 0, y: 0 }, { x: -5, y: 0 }) === 270,
  );
  expect('cpp: empty path hint', generateCpp([], 4000).startsWith('// Click the field'));

  const mx = mirrorPoints(pts, 'x');
  expect('mirror x flips x', mx[0].x === 48 && mx[2].x === -24);
  expect('mirror x heading', mx[0].theta === 315 && mx[1].theta === 270);
  const my = mirrorPoints(pts, 'y');
  expect('mirror y heading', my[0].theta === 135 && my[1].theta === 90);
  expect(
    'mirror twice is identity',
    JSON.stringify(mirrorPoints(mirrorPoints(pts, 'x'), 'x')) === JSON.stringify(pts),
  );

  const round = parseSavedPath(JSON.stringify({ points: pts, timeout: 2500 }));
  expect('persistence round-trip', round !== null && round.timeout === 2500 &&
    JSON.stringify(round.points) === JSON.stringify(pts));
  expect('persistence rejects junk', parseSavedPath('{"points":[{"x":"a"}]}') === null);
  expect('persistence rejects non-json', parseSavedPath('nope') === null);
  expect('persistence clamps to field', parseSavedPath('{"points":[{"x":900,"y":-900}]}')!.points[0].x === 72);

  // the generated program should actually drive the sim to the last waypoint
  const world = new World(defaultConfig());
  const route = [newPoint(0, 0), newPoint(0, 30), newPoint(30, 30)];
  await run(world, () => buildProgram(route, 5000)(world.chassis));
  const end = world.getTruePose();
  check('planner program end x', end.x, 30, 3);
  check('planner program end y', end.y, 30, 3);
}

// ---------------------------------------------------------------------------
// auto-code import: reverse-engineer pasted Odyssey code into waypoints
// ---------------------------------------------------------------------------
{
  const expect = (name: string, ok: boolean) => {
    if (!ok) failures++;
    console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}`);
  };

  // the JS the editor produces (example 2, Boomerang)
  const jsCode = `chassis.setPose(-48, -48, 45);
chassis.moveToPose(0, 0, 90, 5000, { lead: 0.6 });
await chassis.waitUntilDone();
chassis.moveToPose(48, 36, 0, 5000, { lead: 0.4 });
await chassis.waitUntilDone();
chassis.moveToPose(24, -24, 180, 5000, { forwards: false, lead: 0.6 });`;
  const jsParsed = parseAutoCode(jsCode);
  expect('parse js: 4 waypoints', jsParsed.points.length === 4);
  expect('parse js: start pose', jsParsed.points[0].x === -48 && jsParsed.points[0].theta === 45);
  expect('parse js: pose kinds', jsParsed.points.every((p) => p.kind === 'pose'));
  expect('parse js: first lead', jsParsed.points[1].lead === 0.6);
  expect('parse js: lead imported', jsParsed.points[2].lead === 0.4);
  expect('parse js: reverse imported', jsParsed.points[3].forwards === false);
  expect('parse js: shortest timeout', jsParsed.timeout === 5000);
  expect('parse js: no warnings', jsParsed.warnings.length === 0);

  // the C++ the planner emits, with designated initializer options
  const cppCode = [
    'chassis.setPose(12.0, -36.0, 270);',
    'chassis.moveToPose(-24, 0, 90, 4000, {.lead = 0.5});',
    'chassis.waitUntilDone();',
    'chassis.moveToPoint(0, 24, 4000, {.forwards = false, .maxSpeed = 80});',
    'chassis.waitUntilDone();',
  ].join('\n');
  const cppParsed = parseAutoCode(cppCode);
  expect('parse cpp: 2 driven waypoints', cppParsed.points.length === 3);
  expect('parse cpp: start theta', cppParsed.points[0].theta === 270);
  expect('parse cpp: pose heading', cppParsed.points[1].theta === 90 && cppParsed.points[1].lead === 0.5);
  expect('parse cpp: point kind', cppParsed.points[2].kind === 'point');
  expect('parse cpp: point faces travel', cppParsed.points[2].theta === 45);
  expect('parse cpp: reverse + speed', cppParsed.points[2].forwards === false && cppParsed.points[2].maxSpeed === 80);

  // turnToHeading pins the heading of the waypoint it follows
  const turnCode = [
    'chassis.setPose(0, 0, 0);',
    'chassis.moveToPoint(0, 24, 4000);',
    'chassis.turnToHeading(180, 2000);',
    'chassis.moveToPoint(24, 24, 4000, { maxSpeed: 100 });',
  ].join('\n');
  const turnParsed = parseAutoCode(turnCode);
  expect('parse turn: 3 waypoints', turnParsed.points.length === 3);
  expect('parse turn: heading pinned', turnParsed.points[1].theta === 180);
  expect('parse turn: point maxSpeed', turnParsed.points[2].maxSpeed === 100);

  // follow() waypoints extract to a run of point waypoints (skipping the start)
  const followCode = [
    'chassis.setPose(-60, -60, 0);',
    'chassis.follow([',
    '  new odyssey.Waypoint(-60, -60, 90),',
    '  new odyssey.Waypoint(-60, -24, 90),',
    '  new odyssey.Waypoint(0, 24, 70),',
    '], 12, 15000);',
  ].join('\n');
  const followParsed = parseAutoCode(followCode);
  expect('parse follow: start skipped', followParsed.points.length === 3);
  expect('parse follow: start maxSpeed', followParsed.points[0].maxSpeed === 90);
  expect('parse follow: waypoint speed', followParsed.points[1].maxSpeed === 90);
  expect('parse follow: last speed', followParsed.points[2].maxSpeed === 70);
  expect('parse follow: timeout', followParsed.timeout === 15000);

  // code with no start pose: first target becomes the start, with a note
  const noStart = parseAutoCode('let i = 0;\nchassis.moveToPoint(12, 24, 3000, { minSpeed: 60 });');
  expect('parse no-start: synthesized start', noStart.points.length === 1);
  expect('parse no-start: minSpeed as speed', noStart.points[0].maxSpeed === 60);
  expect('parse no-start: warns about setPose', noStart.warnings.some((w) => w.includes('setPose')));
  expect('parse no-start: min-skipped timeout', noStart.timeout === 3000);

  // broken / unrecognized code reports instead of throwing
  const junk = parseAutoCode('chassis.turnToHeading(90, 2000'); // unbalanced paren
  expect('parse junk: no points', junk.points.length === 0);
  expect('parse junk: warns', junk.warnings.some((w) => w.includes('unbalanced')));
  const empty = parseAutoCode('console.log("nothing here");');
  expect('parse empty: no points, no warnings', empty.points.length === 0 && empty.warnings.length === 0);

  // round trip: importing the sanitized output of an import yields the same path
  const sanitized = generateCpp(jsParsed.points, jsParsed.timeout ?? 4000);
  expect('round trip: sanitized parses clean', parseAutoCode(sanitized).points.length === jsParsed.points.length);
}

console.log(failures === 0 ? '\nall checks passed' : `\n${failures} check(s) FAILED`);
process.exit(failures === 0 ? 0 : 1);
