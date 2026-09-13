// Check illustrated contracts and boundary cases independently of the DOM.
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
const require = createRequire(import.meta.url);
const {exactInterval, mappedGrid, occlusion} = require('./assets/diagrams.js');
let cases = 0;
function check(name, operation) {
  try { operation(); cases++; }
  catch (error) { error.message = `${name}: ${error.message}`; throw error; }
}
check('Full sphere interval', () => {
  assert.deepEqual(exactInterval().hit, {enter: 4, exit: 6, worldEnter: 4, worldExit: 6,
    enterSurface: true, exitSurface: true, distanceInside: 2});
});
check('Starting inside clips only entry', () => {
  const model = exactInterval(5, 10);
  assert.equal(model.fullEnter, -1);
  assert.deepEqual(model.hit, {enter: 0, exit: 1, worldEnter: 5, worldExit: 6,
    enterSurface: false, exitSurface: true, distanceInside: 1});
});
check('Range clips only exit', () => {
  const hit = exactInterval(0, 5).hit;
  assert.equal(hit.exit, 5); assert.equal(hit.enterSurface, true); assert.equal(hit.exitSurface, false);
});
check('Closed contact at sphere entry', () => {
  const hit = exactInterval(0, 4).hit;
  assert.equal(hit.enter, 4); assert.equal(hit.exit, 4);
  assert.equal(hit.enterSurface, true); assert.equal(hit.exitSurface, false);
});
check('Zero range inside and outside', () => {
  assert.equal(exactInterval(0, 0).hit, null);
  const hit = exactInterval(5, 0).hit;
  assert.equal(hit.distanceInside, 0);
  assert.equal(hit.enterSurface, false); assert.equal(hit.exitSurface, false);
});
check('AABB candidate does not prove sphere contact', () => {
  assert.notEqual(exactInterval(0, 3).bounds, null); assert.equal(exactInterval(0, 3).hit, null);
});
check('Default mapped grid', () => {
  const model = mappedGrid();
  assert.equal(model.gridOrigin, 11); assert.equal(model.rayOrigin, 11.5);
  assert.deepEqual(model.cell, [1, 0, 0]); assert.deepEqual(model.point, [13, 0.5, 0.5]);
  assert.equal(model.distance, 1.5); assert.equal(model.startCell, 0);
});
check('Translation preserves cell and entry distance', () => {
  for (let ship = 6; ship <= 14; ship++) {
    const model = mappedGrid(ship, 2);
    assert.equal(model.distance, 1.5); assert.equal(model.point[0], ship + 3);
    assert.deepEqual(model.cell, [1, 0, 0]);
  }
});
check('Every cell size keeps the first hit within range', () => {
  for (let size = 1; size <= 3; size += 0.5) {
    const model = mappedGrid(10, size);
    assert.equal(model.distance, size - 0.5); assert.equal(model.startCell, 0);
    assert.ok(model.distance > 0 && model.distance < model.tMax);
    assert.equal(Math.floor(model.point[1] / size), 0);
    assert.equal(Math.floor(model.point[2] / size), 0);
  }
});
check('Default wall clips exit', () => {
  assert.deepEqual(occlusion().hit, {enter: 4, exit: 5, worldEnter: 4, worldExit: 5,
    enterSurface: true, exitSurface: false, distanceInside: 1});
});
check('Wall at entry allows closed contact', () => {
  const hit = occlusion(4).hit;
  assert.equal(hit.distanceInside, 0); assert.equal(hit.enterSurface, true); assert.equal(hit.exitSurface, false);
});
check('Wall before sphere rejects geometry despite candidate', () => {
  assert.equal(occlusion(3).hit, null); assert.notEqual(occlusion(3).bounds, null);
});
check('Wall at or beyond exit preserves both surfaces', () => {
  for (const wall of [6, 7, 8]) {
    const hit = occlusion(wall).hit;
    assert.equal(hit.exit, 6); assert.equal(hit.enterSurface, true); assert.equal(hit.exitSurface, true);
  }
});
check('All exact slider states stay inside the query and sphere', () => {
  for (let origin = 0; origin <= 6; origin += 0.5) {
    for (let range = 0; range <= 10; range += 0.5) {
      const {hit} = exactInterval(origin, range);
      if (!hit) { assert.ok(origin + range < 4); continue; }
      assert.ok(hit.enter >= 0 && hit.exit <= range && hit.enter <= hit.exit);
      assert.ok(hit.worldEnter >= 4 && hit.worldExit <= 6);
      assert.equal(hit.enterSurface, hit.worldEnter === 4);
      assert.equal(hit.exitSurface, hit.worldExit === 6);
    }
  }
});
console.log(`Checked ${cases} diagram contracts, including every exact-interval slider combination.`);
