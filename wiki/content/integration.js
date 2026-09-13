(() => {
  const {code, table, note} = window.WIKI_HTML;
  window.WIKI_PAGES.push(
    {
      id: 'examples', category: 'Integration', title: 'Minecraft integration examples', navTitle: 'Examples', kind: 'guide',
      description: 'Connect a trace to a mounted tool, a rotated target, or a wall without mixing geometry with gameplay decisions.',
      sections: [
        {id: 'choose-inputs', title: 'Choose the data your plugin supplies', html:
          table(['Scenario', 'Input owned by your plugin', 'Query to use'], [
            ['Mining tool on a moving machine', 'Tool frame, local ray, range, and occupied grid cells.', '<a href="#/quick-start">FrameGridRayTracer3</a>'],
            ['Hit-scan against custom targets', 'World-space bounds and a callback that intersects each target shape.', '<a href="#/exact-tracing">FrameExactRayTracer3</a>'],
            ['Target visibility through a block world', 'The same target state plus a grid occupancy predicate.', '<a href="#/occlusion">FrameOccludedExactRayTracer3</a>'],
            ['Nearby target prefilter', 'Indexed bounds, center, and radius in the index coordinate space.', '<a href="#/proximity-sweeps">querySphere / nearest</a>']
          ]) + '<p>Ashtrace queries numerical data. Build one index per world, or otherwise keep independent worlds in separate coordinate spaces. A <code>Vector3</code> alone does not identify a Minecraft world.</p>'},
        {id: 'rotated-target', title: 'Intersect a rotated target', html:
          '<p>A long box rotates 45° around Z and moves to world X = 6. Its AABB encloses empty corners. Ashspace creates the world-space oriented box and its enclosing AABB; Ashcore supplies the full shape interval.</p><p>The ray travels from world zero along +X. The shape entry is approximately X = 5.293 and the exit is X = 6.707. Both points lie inside the wider AABB interval.</p>' +
          '<figure><div class="static-trace-viewport" role="region" aria-label="Spatial diagram. Scroll horizontally if needed." tabindex="0"><svg viewBox="0 0 640 200" role="img" aria-label="XY slice of a rotated rectangular target centered at world X 6, inside a dashed AABB. The horizontal ray intersects the actual rectangle after entering the AABB." style="width:100%;color:var(--text)"><rect x="260" y="32" width="150" height="150" fill="none" stroke="var(--muted)" stroke-dasharray="5 5"/><rect x="250" y="85.75" width="170" height="42.5" transform="rotate(-45 335 107)" fill="var(--accent-soft)" stroke="var(--accent)" stroke-width="2"/><path d="M80 107h435m-9-5 9 5-9 5" stroke="var(--muted)" fill="none"/><path d="M305 107h60" stroke="var(--accent)" stroke-width="5"/><g fill="currentColor" font-size="13" font-family="system-ui,sans-serif"><text x="25" y="25">XY slice · Z = 0</text><text x="248" y="23">Enclosing AABB</text><text x="80" y="98">World ray →</text><text x="438" y="160">45° target</text><text x="260" y="199">5.293</text><text x="352" y="199">6.707</text><text x="480" y="98">+X</text></g><path d="M305 114v65m60-65v65" stroke="var(--border)"/></svg></div><figcaption>The highlighted segment is the path through the target in the XY slice. The dashed box only selects the candidate.</figcaption></figure>' +
          code('java', 'RotatedTargetExample.java', `import java.util.List;
import java.util.Locale;
import nsk.nu.ashcore.api.collision.CollisionTests;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.OrientedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Quaternion;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.space.SpaceConverter3;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameExactRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public final class RotatedTargetExample {
    public static void main(String[] args) {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId body = new FrameId("body");
        frames.define(body, frames.root(), new RigidTransform3(
                Quaternion.fromAxisAngle(new Vector3(0, 0, 1), Math.PI / 4),
                new Vector3(6, 0, 0)));
        SpaceConverter3 converter = new SpaceConverter3(frames);
        AxisAlignedBox local = new AxisAlignedBox(
                new Vector3(-2, -0.5, -0.5), new Vector3(2, 0.5, 0.5));
        OrientedBox shape = converter.orientedBox(local, body, frames.root());
        AxisAlignedBox bounds = converter.axisAlignedBox(local, body, frames.root());
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(bounds, shape)));
        var tracer = new FrameExactRayTracer3<>(frames, index);
        RayIntersector3<OrientedBox> geometry = (value, ray, min, max, output) -> {
            var interval = CollisionTests.rayVsOrientedBoxInterval(ray, value);
            if (interval.hit() && interval.tExit() >= min && interval.tEnter() <= max) {
                output.accept(new RayIntersection3(interval.tEnter(), interval.tExit()));
            }
        };
        Ray ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        var hit = tracer.firstHit(frames.root(), ray, 12, geometry);
        if (hit == null || !hit.enterSurface() || !hit.exitSurface()
                || Math.abs(hit.tEnter() - (6 - Math.sqrt(0.5))) > 1e-12
                || Math.abs(hit.tExit() - (6 + Math.sqrt(0.5))) > 1e-12) {
            throw new AssertionError("Expected the rotated box surfaces");
        }
        System.out.printf(Locale.ROOT, "entry=%.3f exit=%.3f%n", hit.tEnter(), hit.tExit());
    }
}`) + code('output', 'Expected output', 'entry=5.293 exit=6.707')},
        {id: 'update-state', title: 'Update a moving target before the next query', html:
          '<p>The example stores an immutable world-space shape as its payload. After a pose change, recompute both the shape and its world-space bound. Rebuild a static index, or update the dynamic index and the geometry source before the next query.</p><p><code>updateBounds</code> changes the bound only. To replace the example’s immutable payload, remove and insert the entry again. An alternative is to index a stable object ID and let the geometry provider resolve the current shape by that ID.</p>' +
          note('Keep one coherent state', '<p>Keep frames, bounds, payload geometry, and occupancy fixed for the whole query. A frame snapshot freezes the graph, not the other data. Do not mutate the index or reuse its query buffer recursively inside a callback.</p>') +
          '<p>A series of pose queries samples those poses. It does not detect every collision between them. <a href="#/proximity-sweeps">AABB sweeps</a> handle translation, and do not compute continuous contact during rotation.</p>'},
        {id: 'apply-result', title: 'Apply the result in your plugin', html:
          '<ol class="steps my-6 list-none p-0 [counter-reset:steps]"><li class="relative m-0 pb-[25px] pl-[43px] max-[680px]:pl-[37px]"><strong>Capture the query inputs.</strong><p>Read server-owned state in an execution context that permits those reads. If you query a copied data set elsewhere, keep that data set stable.</p></li><li class="relative m-0 pb-[25px] pl-[43px] max-[680px]:pl-[37px]"><strong>Define occupancy and target geometry.</strong><p>Choose which cells block the tool. A solid-cell rule, a transparent-block rule, and a partial collision shape describe different scenes.</p></li><li class="relative m-0 pb-[25px] pl-[43px] max-[680px]:pl-[37px]"><strong>Run the matching trace.</strong><p>Handle <code>null</code> for no single hit. Inspect surface flags before treating a clipped endpoint as a surface.</p></li><li class="relative m-0 pb-[25px] pl-[43px] max-[680px]:pl-[37px]"><strong>Apply your gameplay decision.</strong><p>Use the returned identity and world point to select a target or show an effect. Your plugin decides damage, permissions, cooldowns, and chunk access.</p></li></ol><p>For voxel walls, use <a href="#/occlusion">occluded exact tracing</a> when visibility must refer to the target’s shape. A visible AABB candidate alone does not establish a visible surface.</p>'}
      ]
    },
    {
      id: 'troubleshooting', category: 'Reference', title: 'Troubleshooting', kind: 'reference',
      description: 'Match an unexpected result to its coordinate space, query boundary, or provider contract.',
      sections: [
        {id: 'surface-before-target', title: 'The hit is in empty space before the target', html:
          '<p>Check the result type. <code>TraceHit3</code> reports the accepted AABB entry. The boolean <code>NarrowPhase3</code> callback cannot replace that point with a surface intersection. Use <code>FrameExactRayTracer3</code> and emit full shape intervals through <code>RayIntersector3</code>.</p><p>If exact results look wrong, check that the callback uses the supplied world ray and that the indexed bound encloses the same world-space shape. See <a href="#/exact-tracing">exact tracing</a>.</p>'},
        {id: 'wrong-voxel', title: 'The wrong cell is occupied', html:
          '<p>The original voxel tracer constructors use unit cells at world zero. A tracer created with <code>forGrid</code> uses the supplied mapper’s frame, origin, and uniform cell side length. Its occupancy arguments are integer indices in that mapped grid, even though the hit distances and point use world coordinates.</p><p>Use floor-based cell indexing for negative coordinates: world X = −0.2 belongs to unit cell X = −1. Keep the object index’s spatial-hash cell size separate from the voxel grid’s cell size. See <a href="#/grid-tracing">grid tracing</a>.</p>'},
        {id: 'wall-edge', title: 'A contact at the wall or range limit still appears', html:
          '<p>Object intervals are closed, including their limit. Contact exactly at a wall may be a zero-length visible interval. The DDA visits voxels in <code>[0, tMax)</code>, so a cell beginning exactly at <code>tMax</code> is excluded.</p><p>A starting occupied cell clips the object query to zero. AABB or shape contact at the origin can still match. A zero range performs no occupancy calls; a zero-length segment is rejected. See <a href="#/occlusion">occlusion boundary cases</a>.</p>'},
        {id: 'surface-flags', title: 'A range limit is reported as a surface', html:
          '<p>Check whether your geometry provider preclips its interval. Emit the original entry and exit, including negative entry or exit beyond the query range. Ashtrace performs clipping and derives <code>enterSurface</code> and <code>exitSurface</code> from the original endpoints.</p><p>Use Ashcore’s <code>rayVsOrientedBoxInterval</code> for the oriented-box example. A segment interval uses clipped fractions, which cannot be passed directly as world-ray distances.</p>'},
        {id: 'runtime-and-costs', title: 'A query fails or becomes unexpectedly expensive', html:
          table(['Symptom', 'Check', 'Next step'], [
            ['<code>require("dda")</code> cannot resolve a provider.', 'Ashgrid runtime classes and merged service descriptors in the final JAR.', '<a href="#/installation?section=plugin-packaging">Packaging</a>'],
            ['A moved object still uses its previous shape.', 'The payload geometry as well as the bound; a frame change does not update an index.', '<a href="#/examples?section=update-state">Moving targets</a>'],
            ['The first dynamic BVH query takes more work.', 'Whether mutations have marked its internal tree for rebuild.', '<a href="#/performance">Operation costs</a>'],
            ['A spatial hash query throws for a large range.', 'The number of cells in the query’s enclosing box and integer coordinate limits.', '<a href="#/indexes">Index limits</a>'],
            ['A small <code>maxHits</code> still runs many callbacks.', 'Whether the selected API collects and sorts all intervals before truncation.', '<a href="#/query-contracts">Query contracts</a>'],
            ['Results differ between runs or indexes.', 'Entry order, mutation history, fixed input state, callback determinism, and tie rules.', '<a href="#/query-contracts">Ordering and ownership</a>']
          ])}
      ]
    }
  );
})();
