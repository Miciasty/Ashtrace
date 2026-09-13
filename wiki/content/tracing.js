/* Tracing contracts checked against Ashtrace 2.0.0 source and integration tests. */
(() => {
  const {code, table, note} = window.WIKI_HTML;

  window.WIKI_PAGES.push(
    {
      id: 'coordinate-frames', category: 'Tracing', kind: 'concept', title: 'Coordinate frames',
      description: 'Define a ray relative to a tool or vehicle, then keep its frame, geometry and occupancy consistent for the whole query.',
      readingTime: 6,
      sections: [
        {id: 'local-and-world', title: 'From a mounted tool to a world ray', html: `
          <p>A drill attached to a vehicle points in the vehicle's local coordinates. Its world position changes when the vehicle moves. A <code>FrameId</code> identifies that coordinate frame; <code>FrameGraph3</code> records its position and rotation relative to a parent.</p>
          <p>Pass the source frame and source ray to a frame tracer. Ashspace converts that ray to the graph's root, which the tracer treats as world space. Indexed object bounds and the geometry supplied by <code>RayIntersector3</code> must use that same world space.</p>
          ${table(['Value', 'Coordinate space / unit'], [
            ['<code>sourceRay</code> or <code>sourceSegment</code>', 'Coordinates relative to <code>sourceFrame</code>.'],
            ['Indexed <code>AxisAlignedBox</code>', 'World coordinates, regardless of the source frame.'],
            ['Callback <code>worldRay</code>', 'Root/world coordinates with a normalized direction.'],
            ['Ray <code>tEnter</code>, <code>tExit</code>, <code>tMax</code>', 'Distance from the transformed ray origin in world units.'],
            ['Result points', 'World coordinates. Grid cell indices remain coordinates of the selected grid.']
          ])}
          <p>Rigid transforms preserve lengths: rotation and translation do not scale the ray's range. Ashspace uses a right-handed coordinate system with Y up. If your adapter maps one world unit to one Minecraft block, a range of <code>10</code> measures ten blocks.</p>
        `},
        {id: 'snapshot-example', title: 'Keep a query on one frame state', html: `
          <p>This example snapshots a tool at world X=10, then moves the live tool to X=20. A fixed target occupies X=12..13. The frozen query still hits the target at distance 1.5; the live ray starts beyond it and misses.</p>
          ${code('java', 'FrameSnapshotExample.java', `
import java.util.List;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public class FrameSnapshotExample {
    public static void main(String[] args) {
        var frames = FrameGraph3.worldRoot();
        var tool = new FrameId("tool");
        frames.define(tool, frames.root(), RigidTransform3.translation(10, 0, 0));
        var frozenFrames = frames.snapshot();
        var bounds = new AxisAlignedBox(new Vector3(12, 0, 0), new Vector3(13, 1, 1));
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(bounds, "target")));
        var frozen = new FrameBroadPhaseRayTracer3<>(frozenFrames, index);
        var live = new FrameBroadPhaseRayTracer3<>(frames, index);
        var ray = new Ray(new Vector3(0.5, 0.5, 0.5), new Vector3(1, 0, 0));

        frames.define(tool, frames.root(), RigidTransform3.translation(20, 0, 0));
        var hit = frozen.firstHit(tool, ray, 10, (value, worldRay, enter, exit) -> true);
        if (hit == null || hit.tEnter() != 1.5 || hit.worldPoint().x() != 12) {
            throw new AssertionError("Expected the frozen tool at X=10");
        }
        if (live.anyHit(tool, ray, 10, (value, worldRay, enter, exit) -> true)) {
            throw new AssertionError("The live tool has passed the target");
        }
        System.out.println("Frozen distance: " + hit.tEnter() + "; live query: miss");
    }
}
          `)}
          ${code('output', 'Expected output', 'Frozen distance: 1.5; live query: miss')}
          <p>The result here describes the target's AABB. See <a href="#/exact-tracing">exact tracing</a> when the target's enclosed shape has a different surface.</p>
        `},
        {id: 'snapshot-scope', title: 'What a snapshot captures', html: `
          <p><code>frames.snapshot()</code> creates a frozen frame graph. Later frame changes leave it unchanged. <code>grid.snapshot()</code> freezes the graph retained by a <code>FrameGridSpaceMapper3</code> and keeps its mapping configuration. Create either snapshot while the source graph is stable.</p>
          <p>These snapshots capture frame state. You must separately preserve the indexed bounds, provider geometry and occupancy used by the query. A result stores its distances and points, but retains its payload reference. Mutating that payload does not recompute old results.</p>
          ${note('After a pose change', '<p>Refresh both world geometry and its enclosing AABB before the next query. A dynamic index\'s <code>updateBounds</code> changes only the bounds. If the payload stores immutable world geometry, replace the entry, or use a stable object ID to resolve the updated shape.</p>')}
          <p>For a moving voxel structure, use <a href="#/grid-tracing">a mapped grid</a>. Its live mapper follows later frame changes; a frozen mapper continues to use the captured pose.</p>
        `},
        {id: 'threading', title: 'Keep the complete query state stable', html: `
          <p>Tracers retain the objects you supply. They do not lock or automatically snapshot them. Keep the frame graph, index, provider geometry, occupancy and traverser configuration fixed throughout a complete call, including its callbacks.</p>
          <p>Mutable indexes require external synchronization when shared. Even a read on <code>DynamicBvhBroadPhase3</code> can rebuild its internal tree. A static index can support concurrent reads after safe publication, provided callback and payload access is thread-safe.</p>
          <p>For work away from the server's world-access context, first prepare the data under the rules of your server API. Then query a coherent set of frozen frames, static bounds and application-owned geometry and occupancy. A frame snapshot alone does not make live Minecraft world access safe.</p>
          <p>Keep one <code>TraceQueryBuffer3</code> exclusive to each active query. See <a href="#/performance">performance and buffer reuse</a> and <a href="#/query-contracts">query contracts</a> for ownership and ordering rules.</p>
        `}
      ]
    },
    {
      id: 'exact-tracing', category: 'Tracing', kind: 'guide', title: 'Exact tracing',
      description: 'Return entry and exit on a supplied shape, with flags that distinguish a real surface from the end of the query.',
      readingTime: 8,
      sections: [
        {id: 'shape-versus-bounds', title: 'Locate the shape inside its bounds', html: `
          <p>An AABB can enclose empty space around a custom hitbox. <code>FrameBroadPhaseRayTracer3</code> returns the accepted AABB interval. A boolean acceptance callback can reject that candidate, but cannot replace its distances with the shape's distances.</p>
          <p><code>FrameExactRayTracer3</code> asks your <code>RayIntersector3</code> for each candidate's shape intervals. It then clips and selects those intervals. For a sphere centered at X=5 with radius 1, a ray from X=0 along +X enters at 4 and exits at 6. Its larger AABB begins at 3.</p>
          <div data-diagram="exact-interval"></div>
          <p>The diagram is a section through the sphere's center. The initial query range is 10 world units. Changing the range shows where the query stops inside the sphere.</p>
          ${note('Meaning of exact', '<p>Your geometry provider owns the intersection calculation and its numerical accuracy. Ashtrace selects the shape intervals it receives. It does not supply a general mesh solver or collision response.</p>')}
        `},
        {id: 'sphere-example', title: 'Read both surface points', html: `
          <p>This complete example supplies a small-coordinate sphere solver. The normal query returns X=4 and X=6. A range of 5 clips the exit to X=5. A ray starting at the sphere's center has entry zero and reaches its real exit after one world unit.</p>
          ${code('java', 'ExactSphereExample.java', `
import java.util.List;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameExactRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public class ExactSphereExample {
    private record Ball(Vector3 center, double radius) {}

    public static void main(String[] args) {
        var frames = FrameGraph3.worldRoot();
        var ball = new Ball(new Vector3(5, 0, 0), 1);
        var bounds = new AxisAlignedBox(new Vector3(3, -2, -2), new Vector3(7, 2, 2));
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(bounds, ball)));
        var tracer = new FrameExactRayTracer3<>(frames, index);
        RayIntersector3<Ball> geometry = (value, ray, min, max, output) -> {
            var offset = ray.origin().sub(value.center());
            double b = offset.dot(ray.direction());
            double discriminant = b * b - (offset.lengthSq() - value.radius() * value.radius());
            if (discriminant < 0) return;
            double root = Math.sqrt(discriminant);
            double enter = -b - root;
            double exit = -b + root;
            if (exit >= min && enter <= max) {
                output.accept(new RayIntersection3(enter, exit));
            }
        };
        var ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        var hit = tracer.firstHit(frames.root(), ray, 10, geometry);
        if (hit == null || hit.tEnter() != 4 || hit.tExit() != 6
                || !hit.enterSurface() || !hit.exitSurface()) {
            throw new AssertionError("Expected both sphere surfaces");
        }
        var partial = tracer.firstHit(frames.root(), ray, 5, geometry);
        if (partial == null || partial.tExit() != 5 || partial.exitSurface()) {
            throw new AssertionError("The range stops inside the sphere");
        }
        var inside = tracer.firstHit(frames.root(), new Ray(ball.center(), ray.direction()), 10, geometry);
        if (inside == null || inside.tEnter() != 0 || inside.enterSurface()
                || inside.tExit() != 1 || !inside.exitSurface()) {
            throw new AssertionError("Expected an inside origin and a real exit");
        }
        System.out.println("Entry X=" + hit.worldEnterPoint().x()
                + "; exit X=" + hit.worldExitPoint().x());
    }
}
          `)}
          ${code('output', 'Expected output', 'Entry X=4.0; exit X=6.0')}
          <p>The solver illustrates the interval contract; it does not establish accuracy for arbitrary coordinates or near-tangent inputs. For an oriented box, Ashcore's <code>CollisionTests.rayVsOrientedBoxInterval</code> supplies a full interval. See <a href="#/examples">worked examples</a> for shape integration.</p>
        `},
        {id: 'provider-contract', title: 'Supply full intervals before clipping', html: `
          <p>The callback receives a world ray and the candidate's query-clipped <code>[tMin,tMax]</code> interval. Emit zero or more finite <code>RayIntersection3</code> intervals that intersect it. Preserve each actual entry and exit, including a negative entry or an exit beyond the supplied limit.</p>
          <p>Ashtrace clips the interval to the query and sets the surface flags. If you preclip the provider output, a range limit can be mislabeled as a real surface. For segment routes, use a full ray intersection too: Ashcore's clipped segment fractions in <code>[0,1]</code> cannot be passed directly as world distances.</p>
          ${table(['Provider output', 'Meaning'], [
            ['<code>[-2,8]</code>', 'The ray origin lies inside this connected part of the shape.'],
            ['<code>[3,3]</code>', 'A tangent or surface-only intersection.'],
            ['<code>[2,4]</code> and <code>[7,9]</code>', 'Two material intervals separated by a cavity or gap.'],
            ['No emitted interval', 'This candidate has no shape intersection in the supplied range.']
          ])}
          <p>Merge overlapping or touching part intervals before emitting a solid's union. Preserve positive gaps. Ashtrace neither merges provider output nor checks its topology. The indexed AABB must enclose every reported shape part; a query-clipped interval outside the candidate interval causes <code>IllegalArgumentException</code>.</p>
          <p>Emit synchronously. Do not retain the output consumer or mutate query state from the callback. Exceptions propagate to the caller.</p>
        `},
        {id: 'select-and-interpret', title: 'Choose the result your mechanic needs', html: `
          ${table(['Operation / field', 'Interpretation'], [
            ['<code>firstHit</code>', 'Smallest clipped entry, then smallest exit. Returns <code>null</code> on a miss.'],
            ['<code>lastHit</code>', 'Greatest clipped exit, then greatest entry. This may differ from the last element of <code>allHits</code>.'],
            ['<code>allHits</code>', 'Immutable list sorted by entry, then exit. One object can contribute several intervals.'],
            ['<code>anyHit</code>', 'Whether any interval matches. It does not select the nearest object.'],
            ['<code>enterSurface()</code> / <code>exitSurface()</code>', 'Whether that endpoint is an actual boundary reported by the provider.'],
            ['<code>distanceInside()</code>', '<code>tExit - tEnter</code> for this clipped interval. It may cover only part of the path through the object.']
          ])}
          <p>For example, intervals A=<code>[8,12]</code> and B=<code>[9,11]</code> sort as A, B. <code>lastHit</code> returns A because its exit is farther away. On a full tie, the tracer retains candidate visit order, then provider emission order.</p>
          <p><code>allHits(..., maxHits)</code> collects provider intervals before sorting and truncation. <code>anyHit</code> can stop later candidates after a match; the active provider call still finishes. Segment methods use a finite, nonzero segment's length as the limit. See <a href="#/performance">operation costs</a>.</p>
          <p>Use <a href="#/occlusion"><code>FrameOccludedExactRayTracer3</code></a> to clip these shape intervals at a voxel wall. Damage, penetration and energy loss remain decisions in your plugin.</p>
        `}
      ]
    },
    {
      id: 'grid-tracing', category: 'Tracing', kind: 'guide', title: 'Grid tracing',
      description: 'Find occupied cells in a world grid or a grid attached to a moving frame, while keeping distances in world units.',
      readingTime: 6,
      sections: [
        {id: 'choose-grid', title: 'Choose the grid that owns the cells', html: `
          <p><code>FrameGridRayTracer3</code> visits cells through an Ashgrid <code>VoxelTraverser</code>. Your occupancy callback receives integer cell coordinates and decides whether a cell is occupied. Ashtrace does not fetch Minecraft blocks or define which materials stop a ray.</p>
          <p>The constructor <code>new FrameGridRayTracer3(frames, traverser)</code> uses unit world cells rooted at <code>(0,0,0)</code>. Cell X=-1 contains X=-0.2. For a structure with its own frame, origin or cell size, use <code>FrameGridRayTracer3.forGrid(mapper, traverser)</code>.</p>
          <p>The mapped path converts the ray to the grid frame, subtracts the grid origin and divides position and distance by cell size. Traversal uses unit cell coordinates. The result restores world-distance parameters and a world entry point; its integer indices still identify cells of the mapped grid.</p>
        `},
        {id: 'mapped-example', title: 'Trace a cell on a moving ship', html: `
          <p>The ship is translated ten world units along X. Its grid origin is local X=1 and each cell spans two units. The ray begins at local X=1.5 and enters occupied cell <code>(1,0,0)</code> at world X=13, after 1.5 world units.</p>
          <div data-diagram="mapped-grid"></div>
          <p>The diagram shows a section along the ray. Grid indices and world X labels describe the same cells in different coordinate systems.</p>
          ${code('java', 'MappedGridExample.java', `
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashgrid.implementation.grid.indexing.SquareXZChunkScheme;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.grid.FrameGridSpaceMapper3;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;

public class MappedGridExample {
    public static void main(String[] args) {
        var frames = FrameGraph3.worldRoot();
        var ship = new FrameId("ship");
        frames.define(ship, frames.root(), RigidTransform3.translation(10, 0, 0));
        var grid = new FrameGridSpaceMapper3(
                frames, ship, 2, new Vector3(1, 0, 0), new SquareXZChunkScheme(16));
        var traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        var tracer = FrameGridRayTracer3.forGrid(grid, traverser);
        var ray = new Ray(new Vector3(1.5, 0.5, 0.5), new Vector3(1, 0, 0));
        var hit = tracer.firstHit(ship, ray, 10, (x, y, z) -> x == 1 && y == 0 && z == 0);
        if (hit == null || hit.x() != 1 || hit.y() != 0 || hit.z() != 0
                || hit.tEnter() != 1.5 || hit.worldPoint().x() != 13) {
            throw new AssertionError("Expected cell (1,0,0) at world X=13");
        }
        System.out.println("Cell (" + hit.x() + "," + hit.y() + "," + hit.z()
                + "), distance=" + hit.tEnter() + ", world X=" + hit.worldPoint().x());
    }
}
          `)}
          ${code('output', 'Expected output', 'Cell (1,0,0), distance=1.5, world X=13.0')}
          <p>Use <code>grid.snapshot()</code> when a later query must keep this frame state. See <a href="#/coordinate-frames">coordinate frames</a> for what the snapshot captures.</p>
        `},
        {id: 'grid-results', title: 'Read cells, intervals and boundaries', html: `
          <p><code>firstHit</code> returns the first occupied <code>GridRayHit3</code>, or <code>null</code>. <code>allHits</code> returns an immutable list in traverser visit order. Its <code>maxHits</code> overload stops traversal after that many occupied hits; the limit must be positive.</p>
          ${table(['Field / case', 'Meaning with the dda provider'], [
            ['<code>x()</code>, <code>y()</code>, <code>z()</code>', 'Integer cell indices, including for a mapped grid.'],
            ['<code>tEnter()</code>, <code>tExit()</code>', 'Cell visit interval measured from the world ray origin, clipped to the query range.'],
            ['<code>worldPoint()</code>', 'World position at this cell visit\'s entry.'],
            ['Starting cell', 'Checked by the occupancy callback when <code>tMax &gt; 0</code>.'],
            ['<code>tMax = 0</code>', 'No cell visits and no occupancy calls.'],
            ['Cell starts exactly at <code>tMax</code>', 'Excluded: voxel traversal covers <code>[0,tMax)</code>.']
          ])}
          <p>At an exact boundary tie, the tested <code>dda</code> provider steps X, then Y, then Z. Intermediate visits may have zero length. DDA does not visit every cell touched at an edge or corner; it is not a supercover traversal. Another traverser can change the first occupied cell selected at a tie.</p>
          <p><code>firstSegmentHit</code> and <code>allSegmentHits</code> use the segment's length as the traversal limit. Zero-length and non-finite segments throw <code>IllegalArgumentException</code>. Ashgrid's <code>LineOfSight</code> skips the starting cell, while this tracer follows <code>Raycast</code> and checks it.</p>
        `},
        {id: 'grid-configuration', title: 'Keep configuration units separate', html: `
          <p>The mapper's <code>cellSize</code> sets voxel geometry. Its chunk scheme supplies grouping information and does not change cell shape. A <a href="#/indexes">spatial hash</a> has a separate <code>cellSize</code> for indexing objects; that setting does not configure voxel traversal.</p>
          <p>If you wrap a traverser with <code>VoxelTraversers.clipped</code>, configure its bounds in unit cell coordinates. Convert any world clipping box before using it for the mapped traversal. Mapping adds no boundary epsilon.</p>
          <p>Cell coordinates and distance conversions must remain representable. Mapped tracing rejects overflow and a nonzero origin offset or positive limit that divides to zero. Ashgrid's integer-coordinate limits also apply. See <a href="#/troubleshooting">troubleshooting</a> for invalid inputs and provider loading, or <a href="#/occlusion">occlusion</a> to combine the grid with object hits.</p>
        `}
      ]
    },
    {
      id: 'occlusion', category: 'Tracing', kind: 'guide', title: 'Occlusion',
      description: 'Shorten an object query at the first occupied voxel and interpret contact, clipped exits and hidden geometry.',
      readingTime: 7,
      sections: [
        {id: 'visible-interval', title: 'Stop the object query at the wall', html: `
          <p>A beam can hit an object before a voxel wall while its far side remains hidden. An occluded tracer first finds the first occupied cell. It then shortens the object query to that cell's entry distance.</p>
          <p><code>FrameOccludedExactRayTracer3</code> clips supplied shape intervals to this visible range. <code>FrameOccludedBroadPhaseRayTracer3</code> clips accepted AABB candidates. Choose the exact route when your mechanic needs the enclosed shape's surface or distance inside it.</p>
          <div data-diagram="occlusion"></div>
          <p>This section through a sphere uses the <a href="#/exact-tracing">exact tracing example</a>: entry at 4, exit at 6, and a wall beginning at 5. The visible interval is <code>[4,5]</code>. Its exit is the wall limit, so <code>exitSurface()</code> is false.</p>
        `},
        {id: 'occlusion-example', title: 'Clip a sphere at an occupied cell', html: `
          <p>The example marks cells at X=5 as occupied. The sphere crosses that wall. Its returned entry remains a sphere surface at X=4; its returned exit becomes X=5.</p>
          ${code('java', 'OcclusionExample.java', `
import java.util.List;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameOccludedExactRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public class OcclusionExample {
    private record Ball(Vector3 center, double radius) {}

    public static void main(String[] args) {
        var frames = FrameGraph3.worldRoot();
        var ball = new Ball(new Vector3(5, 0, 0), 1);
        var bounds = new AxisAlignedBox(new Vector3(3, -2, -2), new Vector3(7, 2, 2));
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(bounds, ball)));
        var traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        var tracer = new FrameOccludedExactRayTracer3<>(frames, index, traverser);
        RayIntersector3<Ball> geometry = (value, ray, min, max, output) -> {
            var offset = ray.origin().sub(value.center());
            double b = offset.dot(ray.direction());
            double discriminant = b * b - (offset.lengthSq() - value.radius() * value.radius());
            if (discriminant < 0) return;
            double root = Math.sqrt(discriminant);
            double enter = -b - root;
            double exit = -b + root;
            if (exit >= min && enter <= max) {
                output.accept(new RayIntersection3(enter, exit));
            }
        };
        var ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        var hit = tracer.firstVisibleHit(frames.root(), ray, 10, geometry, (x, y, z) -> x == 5);
        if (hit == null || hit.tEnter() != 4 || hit.tExit() != 5
                || !hit.enterSurface() || hit.exitSurface()) {
            throw new AssertionError("Expected the sphere interval clipped by the wall");
        }
        System.out.println("Visible interval: [" + hit.tEnter() + "," + hit.tExit()
                + "], exitSurface=" + hit.exitSurface());
    }
}
          `)}
          ${code('output', 'Expected output', 'Visible interval: [4.0,5.0], exitSurface=false')}
          <p>This small-coordinate solver uses the same full-interval contract as <a href="#/exact-tracing">exact tracing</a>. It preserves the sphere's actual exit at 6 when emitting the interval; Ashtrace applies the wall limit.</p>
        `},
        {id: 'wall-boundaries', title: 'Understand contact at the limit', html: `
          <p>Voxel traversal uses <code>[0,tMax)</code>. The object stage uses the closed interval <code>[0,limit]</code>, where <code>limit</code> is the first occupied cell's entry or the original range. These endpoint rules let an object touch the wall at the limit.</p>
          ${table(['Situation', 'Result'], [
            ['Shape enters exactly at wall entry', 'A zero-length contact can match. A later shape exit is clipped and has <code>exitSurface=false</code>.'],
            ['Origin is inside an occupied callback cell', 'Visible limit is zero. A shape or AABB containing the origin can still match.'],
            ['<code>tMax = 0</code>', 'No occupancy calls. The object stage can still match contact at the origin.'],
            ['Occupied cell begins exactly at <code>tMax</code>', 'That cell is not visited. Object contact at <code>tMax</code> remains included.'],
            ['No occupied cell', 'The object stage uses the original <code>tMax</code>.']
          ])}
          <p>The tested DDA tie rules decide which occupied cell supplies the limit when boundaries coincide. If your mechanic must treat a boundary contact as blocked, apply that gameplay rule to the returned interval. See <a href="#/query-contracts">query contracts</a> for closed object bounds and invalid limits.</p>
        `},
        {id: 'choose-occluded-query', title: 'Select shapes or accepted bounds', html: `
          <p>On the exact route, use <code>firstVisibleHit</code> for the earliest interval, <code>lastVisibleHit</code> for the greatest visible exit, <code>visibleHits</code> for ordered intervals, or <code>anyVisibleHit</code> for existence. Their segment counterparts use a finite, nonzero segment's length.</p>
          <p>A loose AABB may begin before the wall while the enclosed shape lies behind it. The broad-phase route can accept that clipped AABB. A geometry acceptance callback must respect its supplied clipped interval; returning <code>true</code> alone does not establish surface visibility. The exact route rejects the shape when its intervals are wholly behind the wall.</p>
          <p>For a moving or scaled-cell grid, use either tracer's <code>forGrid(grid, index, traverser)</code> factory. Both object and voxel stages share the mapper's frame graph. Keep frames, occupancy, bounds and provider geometry stable across both stages.</p>
          ${note('Result limits and work', '<p>Exact <code>visibleHits(..., maxHits)</code> gathers and sorts provider intervals before truncation. The broad-phase occluded <code>visibleHits</code> also runs all candidate acceptance callbacks before truncating. A small result limit does not cap that work. See <a href="#/performance">performance</a>.</p>')}
          <p>Continue with <a href="#/broad-phase">candidate queries</a>, <a href="#/proximity-sweeps">proximity and sweeps</a>, or the <a href="#/api-reference">API reference</a>.</p>
        `}
      ]
    }
  );
})();
