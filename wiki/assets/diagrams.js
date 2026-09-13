/* Finite explanatory models for the documented examples; no Minecraft runtime. */
(function (scope) {
  'use strict';
  function clipInterval(fullEnter, fullExit, limit, origin = 0) {
    const enter = Math.max(0, fullEnter), exit = Math.min(limit, fullExit);
    if (enter > exit) return null;
    return {enter, exit, worldEnter: origin + enter, worldExit: origin + exit,
      enterSurface: enter === fullEnter, exitSurface: exit === fullExit, distanceInside: exit - enter};
  }
  function exactInterval(origin = 0, range = 10) {
    return {origin, range, fullEnter: 4 - origin, fullExit: 6 - origin,
      bounds: clipInterval(3 - origin, 7 - origin, range, origin),
      hit: clipInterval(4 - origin, 6 - origin, range, origin)};
  }
  function mappedGrid(ship = 10, cellSize = 2) {
    const gridOrigin = ship + 1, rayOrigin = ship + 1.5;
    const cellMin = gridOrigin + cellSize, cellMax = cellMin + cellSize;
    return {ship, cellSize, gridOrigin, rayOrigin, cellMin, cellMax,
      startCell: Math.floor((rayOrigin - gridOrigin) / cellSize), cell: [1, 0, 0],
      distance: cellMin - rayOrigin, point: [cellMin, 0.5, 0.5], tMax: 10};
  }
  function occlusion(wall = 5) {
    return {wall, limit: wall, fullEnter: 4, fullExit: 6,
      bounds: clipInterval(3, 7, wall), hit: clipInterval(4, 6, wall)};
  }
  const models = {clipInterval, exactInterval, mappedGrid, occlusion};
  if (typeof module !== 'undefined' && module.exports) module.exports = models;
  if (!scope.document) return;

  const NS = 'http://www.w3.org/2000/svg', mounted = new WeakMap();
  let sequence = 0;
  const format = value => String(Number(value.toFixed(2)));
  const point = (x, y = 0, z = 0) => `(${format(x)}, ${format(y)}, ${format(z)})`;
  const add = (parent, tag, attributes = {}, content) => {
    const node = document.createElementNS(NS, tag);
    for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
    if (content !== undefined) node.textContent = content;
    parent.append(node);
    return node;
  };
  const text = (svg, x, y, value, className = 'trace-svg-label', anchor = 'start') =>
    add(svg, 'text', {x, y, class: className, 'text-anchor': anchor}, value);
  const line = (svg, x1, y1, x2, y2, className) =>
    add(svg, 'line', {x1, y1, x2, y2, class: className});
  function sceneBase(svg, id, title, description) {
    svg.replaceChildren();
    add(svg, 'title', {id: `${id}-title`}, title);
    add(svg, 'desc', {id: `${id}-description`}, description);
    svg.setAttribute('aria-labelledby', `${id}-title ${id}-description`);
    svg.setAttribute('viewBox', '0 0 680 300');
  }
  function axis(svg, project, min, max, axisY = 242) {
    line(svg, 48, axisY, 640, axisY, 'trace-axis');
    for (let x = Math.ceil(min); x <= max; x++) {
      const px = project(x);
      line(svg, px, axisY - 4, px, axisY + 4, 'trace-axis');
      text(svg, px, axisY + 21, format(x), 'trace-svg-small', 'middle');
    }
    text(svg, 640, axisY + 46, 'World X · world units →', 'trace-svg-small', 'end');
  }
  function sphere(svg, project, centerY) {
    const unit = project(1) - project(0);
    add(svg, 'rect', {x: project(3), y: centerY - unit * 2, width: unit * 4,
      height: unit * 4, rx: 3, class: 'trace-bounds'});
    add(svg, 'circle', {cx: project(5), cy: centerY, r: unit, class: 'trace-shape'});
    text(svg, project(3), centerY - unit * 2 - 10, 'AABB: X = 3…7', 'trace-svg-small');
    text(svg, project(5), centerY - unit - 12, 'Sphere: center X = 5, radius 1', 'trace-svg-small', 'middle');
  }
  function ray(svg, project, origin, end, centerY) {
    line(svg, project(origin), centerY, project(end), centerY, 'trace-ray');
    if (end > origin) {
      const px = project(end);
      add(svg, 'path', {d: `M ${px - 8} ${centerY - 5} L ${px} ${centerY} L ${px - 8} ${centerY + 5}`, class: 'trace-arrow'});
    }
    add(svg, 'circle', {cx: project(origin), cy: centerY, r: 5, class: 'trace-origin'});
    text(svg, project(origin), centerY + 31, 'Ray origin', 'trace-svg-small', 'middle');
  }
  function hitMarks(svg, project, hit, centerY) {
    if (!hit) return;
    line(svg, project(hit.worldEnter), centerY, project(hit.worldExit), centerY, 'trace-hit-line');
    const coincident = hit.worldEnter === hit.worldExit;
    if (coincident && hit.enterSurface !== hit.exitSurface) {
      add(svg, 'circle', {cx: project(hit.worldEnter), cy: centerY, r: 7, class: 'trace-endpoint trace-clipped'});
      add(svg, 'circle', {cx: project(hit.worldEnter), cy: centerY, r: 2.5, class: 'trace-endpoint trace-surface'});
    } else {
      for (const [x, surface] of [[hit.worldEnter, hit.enterSurface], [hit.worldExit, hit.exitSurface]]) {
        add(svg, 'circle', {cx: project(x), cy: centerY, r: 6,
          class: `trace-endpoint ${surface ? 'trace-surface' : 'trace-clipped'}`});
      }
    }
    if (hit.worldEnter === hit.worldExit) {
      text(svg, project(hit.worldEnter), centerY + 57, `Entry = exit: X ${format(hit.worldEnter)}`, 'trace-svg-label', 'middle');
    } else {
      text(svg, project(hit.worldEnter) - 8, centerY + 57, `Entry X ${format(hit.worldEnter)}`, 'trace-svg-label', 'end');
      text(svg, project(hit.worldExit) + 8, centerY + 57, `Exit X ${format(hit.worldExit)}`, 'trace-svg-label');
    }
  }
  const resultRow = (label, value) => `<div><dt>${label}</dt><dd>${value}</dd></div>`;
  function hitResult(hit) {
    if (!hit) return '<p class="trace-result-state">No shape interval intersects this query.</p>';
    return `<p class="trace-result-state">${hit.distanceInside === 0 ? 'Zero-length contact' : 'Shape interval returned'}</p><dl class="trace-results">` +
      resultRow('Entry distance tEnter', `${format(hit.enter)} world units`) +
      resultRow('Exit distance tExit', `${format(hit.exit)} world units`) +
      resultRow('World entry point', point(hit.worldEnter)) + resultRow('World exit point', point(hit.worldExit)) +
      resultRow('enterSurface', String(hit.enterSurface)) + resultRow('exitSurface', String(hit.exitSurface)) + '</dl>';
  }
  const definitions = {
    'exact-interval': {
      title: 'From a sphere interval to a query result',
      lead: 'XY cross-section at Z = 0. The ray travels along +X at Y = 0. The dashed box contains the solid sphere.',
      controls: [
        {key: 'origin', label: 'Ray origin X', min: 0, max: 6, step: 0.5, value: 0},
        {key: 'range', label: 'Range tMax', min: 0, max: 10, step: 0.5, value: 10}
      ],
      caption: 'Default: a ray from (0, 0, 0) enters the sphere at (4, 0, 0) and exits at (6, 0, 0). Both surface flags are true. Moving the origin inside the sphere clips its entry to distance 0; reducing the range can clip the exit. All distances use world units.',
      draw(svg, id, state, result) {
        const model = exactInterval(state.origin, state.range), project = x => 55 + x * 35;
        sceneBase(svg, id, this.title, 'Fixed XY cross-section. Solid circle: sphere. Dashed rectangle: enclosing AABB. Thick segment: returned shape interval.');
        text(svg, 28, 24, 'Y ↑', 'trace-svg-small');
        sphere(svg, project, 137);
        line(svg, project(0), 137, project(16), 137, 'trace-guide');
        ray(svg, project, state.origin, state.origin + state.range, 137);
        hitMarks(svg, project, model.hit, 137);
        const end = state.origin + state.range;
        line(svg, project(end), 122, project(end), 152, 'trace-limit');
        text(svg, project(end), 219, `Range end X ${format(end)}`, 'trace-svg-small', 'middle');
        axis(svg, project, 0, 16);
        result.innerHTML = `<p class="trace-model-note">The provider reports full distances [${format(model.fullEnter)}, ${format(model.fullExit)}]. The tracer intersects them with [0, ${format(state.range)}].</p>` + hitResult(model.hit);
      }
    },
    'mapped-grid': {
      title: 'One grid cell, two coordinate systems',
      lead: 'XY cross-section at world Z = 0.5. The ship only translates along X. The ray follows +X at world Y = 0.5; occupied cell indices remain (1, 0, 0).',
      controls: [
        {key: 'ship', label: 'Ship translation X', min: 6, max: 14, step: 1, value: 10},
        {key: 'cellSize', label: 'Cell size', min: 1, max: 3, step: 0.5, value: 2}
      ],
      caption: 'Default: ship X = 10, local grid origin X = 1, cell size = 2. The ray starts at world (11.5, 0.5, 0.5). Occupied cell (1, 0, 0) starts at world X = 13, so its entry distance is 1.5 world units. Translation moves both ray and grid; changing cell size changes the hit distance.',
      draw(svg, id, state, result) {
        const model = mappedGrid(state.ship, state.cellSize), min = 5, max = 28;
        const project = x => 48 + (x - min) / (max - min) * 592;
        const unit = project(1) - project(0), baseY = 173, rayY = baseY - 0.5 * unit;
        const topY = baseY - state.cellSize * unit;
        sceneBase(svg, id, this.title, 'Fixed XY cross-section of grid row Y index 0. Cell 1 is occupied. Ship movement changes world coordinates but not the grid index.');
        text(svg, 28, 25, 'Y ↑', 'trace-svg-small');
        text(svg, 640, 25, 'Grid origin in ship: (1, 0, 0)', 'trace-svg-small', 'end');
        for (let cell = -1; cell <= 2; cell++) {
          const left = project(model.gridOrigin + cell * state.cellSize);
          add(svg, 'rect', {x: left, y: topY, width: state.cellSize * unit,
            height: state.cellSize * unit, class: cell === 1 ? 'trace-cell trace-occupied' : 'trace-cell'});
          text(svg, left + state.cellSize * unit / 2, topY - 13, String(cell), 'trace-svg-label', 'middle');
        }
        text(svg, project(model.gridOrigin), topY - 36, 'Cell X index (Y = 0, Z = 0)', 'trace-svg-small');
        ray(svg, project, model.rayOrigin, model.rayOrigin + 10, rayY);
        line(svg, project(model.rayOrigin), rayY, project(model.cellMin), rayY, 'trace-hit-line');
        add(svg, 'circle', {cx: project(model.cellMin), cy: rayY, r: 6, class: 'trace-endpoint trace-surface'});
        line(svg, project(model.cellMin), rayY + 10, project(model.cellMin), 209, 'trace-guide');
        text(svg, project(model.cellMin) + 8, 216, `Cell 1 entry: X ${format(model.cellMin)}`, 'trace-svg-label');
        line(svg, project(state.ship), 51, project(state.ship), 190, 'trace-frame-origin');
        text(svg, project(state.ship) - 6, 49, `Ship X ${format(state.ship)}`, 'trace-svg-small', 'end');
        axis(svg, project, min, max);
        result.innerHTML = '<p class="trace-result-state">First occupied cell: (1, 0, 0)</p><dl class="trace-results">' +
          resultRow('Grid origin in world', point(model.gridOrigin)) +
          resultRow('Ray origin in world', point(model.rayOrigin, 0.5, 0.5)) +
          resultRow('Occupied cell world X', `[${format(model.cellMin)}, ${format(model.cellMax)})`) +
          resultRow('World entry point', point(model.cellMin, 0.5, 0.5)) +
          resultRow('Entry distance', `${format(model.distance)} world units`) +
          resultRow('Range tMax', '10 world units') + '</dl>';
      }
    },
    occlusion: {
      title: 'A wall clips the shape interval',
      lead: 'XY cross-section at Z = 0. The ray follows +X at Y = 0. The vertical wall marker is the entry plane of the first occupied voxel.',
      controls: [{key: 'wall', label: 'Wall entry X', min: 3, max: 8, step: 1, value: 5}],
      caption: 'Default: the sphere spans X = 4…6 and the wall begins at X = 5. The returned interval is [4, 5]: enterSurface is true, exitSurface is false. A wall at X = 4 permits a zero-length contact; a wall before X = 4 hides the sphere. The ray origin and range stay at X = 0 and 10 world units.',
      draw(svg, id, state, result) {
        const model = occlusion(state.wall), project = x => 55 + x * 48;
        sceneBase(svg, id, this.title, 'Fixed XY cross-section. The first occupied voxel clips the query at its entry plane; a loose AABB can begin before a hidden sphere.');
        text(svg, 28, 24, 'Y ↑', 'trace-svg-small');
        add(svg, 'rect', {x: project(state.wall), y: 37, width: project(11) - project(state.wall), height: 193, class: 'trace-occluded'});
        sphere(svg, project, 142);
        ray(svg, project, 0, state.wall, 142);
        line(svg, project(state.wall), 142, project(10), 142, 'trace-hidden-ray');
        line(svg, project(state.wall), 37, project(state.wall), 230, 'trace-wall');
        text(svg, project(state.wall) + 9, 21, `Wall X ${format(state.wall)}`, 'trace-svg-label');
        text(svg, project(state.wall) + 9, 224, 'Beyond the visible limit', 'trace-svg-small');
        hitMarks(svg, project, model.hit, 142);
        axis(svg, project, 0, 11);
        result.innerHTML = `<p class="trace-model-note">Visible query: [0, ${format(model.limit)}]. AABB candidate: [${format(model.bounds.enter)}, ${format(model.bounds.exit)}]. The full sphere interval stays [4, 6].</p>` + hitResult(model.hit);
      }
    }
  };
  function mountFigure(host) {
    const definition = definitions[host.dataset.diagram];
    if (!definition || mounted.has(host)) return null;
    const id = `ashtrace-figure-${++sequence}`;
    const state = Object.fromEntries(definition.controls.map(control => [control.key, control.value]));
    const abort = new AbortController();
    host.classList.add('diagram-component', 'trace-diagram');
    host.innerHTML = `<div class="trace-diagram-heading"><h3>${definition.title}</h3><p>${definition.lead}</p></div>` +
      `<div class="trace-controls">${definition.controls.map(control => `<label for="${id}-${control.key}"><span>${control.label}<output for="${id}-${control.key}" data-value="${control.key}">${format(control.value)}</output></span><input type="range" id="${id}-${control.key}" data-control="${control.key}" min="${control.min}" max="${control.max}" step="${control.step}" value="${control.value}" aria-describedby="${id}-units"></label>`).join('')}<button type="button" class="trace-reset">Reset example</button></div>` +
      `<p class="trace-unit-note" id="${id}-units">Controls use world units and change the query inputs. The XY view stays fixed.</p>` +
      '<div class="trace-scene-viewport" tabindex="0" role="region" aria-label="Spatial diagram. Scroll horizontally if needed."><svg class="trace-scene" role="img"></svg></div>' +
      '<p class="trace-legend"><span>Dashed outline: AABB</span><span>Thick segment: returned interval</span><span>Solid dot: surface · hollow dot: clipped endpoint</span><span>Dot inside ring: both endpoint types coincide</span></p>' +
      '<div class="trace-result" aria-live="polite" aria-atomic="true"></div>' +
      `<p class="trace-caption">${definition.caption}</p>`;
    if (host.dataset.diagram === 'mapped-grid') host.querySelector('.trace-legend').textContent =
      'Outlined cells: empty · filled cell: occupied (1, 0, 0) · thick ray segment: distance to entry';
    const svg = host.querySelector('svg'), result = host.querySelector('.trace-result');
    const render = () => {
      for (const [key, value] of Object.entries(state)) {
        host.querySelector(`[data-value="${key}"]`).textContent = format(value);
        host.querySelector(`[data-control="${key}"]`).setAttribute('aria-valuetext', `${format(value)} world units`);
      }
      definition.draw(svg, id, state, result);
    };
    for (const input of host.querySelectorAll('input')) input.addEventListener('input', () => {
      state[input.dataset.control] = Number(input.value); render();
    }, {signal: abort.signal});
    host.querySelector('button').addEventListener('click', () => {
      for (const control of definition.controls) {
        state[control.key] = control.value;
        host.querySelector(`[data-control="${control.key}"]`).value = String(control.value);
      }
      render();
    }, {signal: abort.signal});
    const cleanup = () => {abort.abort(); mounted.delete(host);};
    mounted.set(host, cleanup); render();
    return cleanup;
  }
  scope.WikiDiagrams = {models, mount(root) {
    const cleanups = [...root.querySelectorAll('[data-diagram]')].map(mountFigure).filter(Boolean);
    return () => cleanups.forEach(cleanup => cleanup());
  }};
})(typeof window === 'undefined' ? globalThis : window);
