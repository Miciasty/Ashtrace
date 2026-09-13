/* Ashtrace 2.0.0: bounds queries, mutable indexes, and public contracts. */
(() => {
  const { code, table, note } = window.WIKI_HTML;
  const api = (pkg, name, page, description) => [
    `<code>${pkg}</code>`, `<a href="#/${page}"><code>${name}</code></a>`, description
  ];
  const apiTable = rows => table(['Package', 'Type', 'Contract and next article'], rows);
  window.WIKI_PAGES.push(
    {
      id: 'broad-phase', category: 'Broad phase', title: 'Bounds candidates', kind: 'concept', readingTime: 8,
      description: 'Query indexed boxes, interpret their intervals, and choose ordered selection or an early-stop visitor.',
      intro: '<p>An AABB is an axis-aligned bounding box. Ashtrace indexes these boxes to find objects that may need a more detailed test. A candidate says that the query reaches the indexed box. What happens inside that box depends on the geometry your application supplies.</p>',
      sections: [
        {
          id: 'entries-and-overlap', title: 'Store a box and a payload',
          html: `<p><code>AabbEntry3&lt;T&gt;</code> pairs world-space <code>AxisAlignedBox</code> bounds with a non-null payload. A payload can be an entity ID, an immutable shape description, or an application object. Every matching entry is emitted once, even when two entries contain equal payload values.</p>
          <p><code>query(bounds, consumer)</code> emits payloads whose boxes overlap the query box. Contact at a face, edge, or corner counts. A zero-width box is a valid closed box; it can represent a plane, line, or point.</p>
          <p><code>queryRay(ray, tMax, consumer)</code> emits <code>BroadPhaseRayHit3</code> records with the payload and a closed interval <code>[tEnter, tExit]</code>, clipped to <code>[0, tMax]</code>. Both parameters measure distance from the ray origin in world units. <code>Ray</code> normalizes its direction, so direction <code>(10, 0, 0)</code> does not multiply the range by ten.</p>
          ${code('java', 'BoundsCandidatesExample.java', `import java.util.ArrayList;
import java.util.List;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public final class BoundsCandidatesExample {
    private static AxisAlignedBox box(double minX, double maxX) {
        return new AxisAlignedBox(new Vector3(minX, 0, 0), new Vector3(maxX, 1, 1));
    }

    public static void main(String[] args) {
        var index = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(4, 5), "far"),
                new AabbEntry3<>(box(1, 2), "near")));
        var overlaps = new ArrayList<String>();
        index.query(box(2, 4), overlaps::add);
        System.out.println("touching=" + overlaps);
        Ray ray = new Ray(new Vector3(0, 0.5, 0.5), new Vector3(10, 0, 0));
        index.queryRay(ray, 6, hit -> System.out.println(
                hit.value() + "=[" + hit.tEnter() + "," + hit.tExit() + "]"));
        var frames = FrameGraph3.worldRoot();
        var tracer = new FrameBroadPhaseRayTracer3<>(frames, index);
        var first = tracer.firstHit(frames.root(), ray, 6, (value, worldRay, enter, exit) -> true);
        if (first == null) throw new AssertionError("Expected a bounds hit");
        System.out.println("first=" + first.value() + " at " + first.worldPoint().x());
    }
}`)}
          ${code('output', 'Expected output', `touching=[far, near]
far=[4.0,5.0]
near=[1.0,2.0]
first=near at 1.0`)}
          <p>The linear index emits in input order, so its raw ray query returns <code>far</code> first. The frame pipeline sorts the intervals and selects <code>near</code>. Use <code>querySegment</code> for a finite segment: its start becomes the ray origin, and its length becomes the distance limit. The segment must have a finite, positive length.</p>`
        },
        {
          id: 'acceptance-and-shape', title: 'Accepting a candidate keeps its box interval',
          html: `<p><code>FrameBroadPhaseRayTracer3.NarrowPhase3</code> receives the payload, the transformed world ray, and the candidate interval. Return <code>false</code> to reject the candidate. Return <code>true</code> to keep its AABB interval and AABB entry point in <code>TraceHit3</code>.</p>
          <p>Consider a ray from world X=0 along +X. These two shapes fit inside the following boxes:</p>
          ${table(['Object', 'Indexed X bounds', 'First enclosed surface'], [
            ['A', '<code>[1, 10]</code>', '<code>X=9</code>'],
            ['B', '<code>[3, 5]</code>', '<code>X=4</code>']
          ])}
          <p>If the callback accepts both, <code>firstHit</code> returns A at X=1. B has the nearer shape surface, but its box begins later. A boolean callback cannot substitute surface distances for the returned distances. Use <a href="#/exact-tracing"><code>FrameExactRayTracer3</code> with <code>RayIntersector3</code></a> when surface position, entry/exit through a shape, or surface ordering determines the result.</p>
          ${note('The callback receives a clipped interval', '<p>Restrict a geometry acceptance test to the supplied <code>[tEnter, tExit]</code>. Under <a href="#/occlusion">voxel occlusion</a>, accepting a surface beyond that interval can report a box as visible even though its enclosed surface is behind the blocking voxel.</p>')}`
        },
        {
          id: 'ordered-results', title: 'Know which query orders results',
          html: `${table(['Implementation', 'Overlap and sphere', 'Ray intervals', 'Sweep intervals'], [
            ['<code>LinearAabbBroadPhase3</code>', 'Input order.', 'Input order.', 'Entry time, exit time, input position.'],
            ['<code>BvhAabbBroadPhase3</code>', 'Left-to-right tree/leaf order.', 'Entry distance, exit distance, input position.', 'Entry time, exit time, input position.'],
            ['<code>DynamicSpatialHashBroadPhase3</code>', 'Ascending handles.', 'Entry distance, exit distance, handle.', 'Entry time, exit time, handle.'],
            ['<code>DynamicBvhBroadPhase3</code>', 'Static BVH rules for its latest snapshot.', 'Static BVH rules; surviving insertion order supplies input positions.', 'Static BVH rules.']
          ])}
          <p><code>FrameBroadPhaseRayTracer3.firstHit</code> and <code>allHits</code> sort by AABB entry, then exit. Full ties preserve broad-phase emission order. The frame pipeline supplies ordered AABB selection even when the underlying <code>queryRay</code> is unsorted.</p>
          <p>A query's candidate set can agree across indexes while its order differs. For repeatable ties, preserve the implementation, dependency versions, cell size, ordered input, mutation history, Java environment, and callback behavior. See <a href="#/query-contracts">query contracts</a> for exact-hit ordering and result limits.</p>`
        },
        {
          id: 'visitors', title: 'Stop when existence is enough',
          html: `<p><code>visitRay</code> calls a visitor with the payload and interval directly. Return <code>true</code> to continue or <code>false</code> to stop. Its return value describes enumeration: <code>true</code> means the visitor allowed completion; <code>false</code> means it requested a stop, even on the final candidate.</p>
          ${code('java', 'BoundsVisitorExample.java', `import java.util.List;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public final class BoundsVisitorExample {
    public static void main(String[] args) {
        var box = new AxisAlignedBox(new Vector3(2, 0, 0), new Vector3(3, 1, 1));
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(box, "target")));
        Ray ray = new Ray(new Vector3(0, 0.5, 0.5), new Vector3(1, 0, 0));
        int[] calls = {0};
        boolean completed = index.visitRay(ray, 5, (value, enter, exit) -> {
            calls[0]++;
            return false;
        });
        System.out.println("completed=" + completed + ", calls=" + calls[0]);
        System.out.println("any=" + index.anyRay(ray, 5));
    }
}`)}
          ${code('output', 'Expected output', `completed=false, calls=1
any=true`)}
          <p><code>anyRay</code> asks whether any box intersects the ray interval. <code>FrameBroadPhaseRayTracer3.anyHit</code> also applies your acceptance callback. Neither operation selects the nearest candidate.</p>
          <p>The visitor order is input order for the linear index, tree/leaf order for BVHs, and ascending handles for the hash. Built-in indexes stop subsequent traversal or candidate tests when requested. A custom index inheriting the default <code>visitRay</code> still completes its own <code>queryRay</code>; the fallback only suppresses later visitor calls. The hash also completes bucket collection, deduplication, and handle sorting before the first visitor call.</p>`
        }
      ]
    },
    {
      id: 'indexes', category: 'Broad phase', title: 'Choose and maintain an index', navTitle: 'Indexes', kind: 'guide', readingTime: 7,
      description: 'Compare the four implementations, update moving bounds, and size spatial-hash workloads.',
      intro: '<p>All four built-in indexes support AABB overlap, rays and segments, sphere queries, nearest bounds, and translating AABB sweeps. Choose an implementation according to how your objects move and how much of the scene each query reaches.</p>',
      sections: [
        {
          id: 'choose-an-index', title: 'Match the index to the workload',
          html: `${table(['Implementation', 'Construction and updates', 'Useful starting point'], [
            ['<code>LinearAabbBroadPhase3&lt;T&gt;</code>', '<code>new LinearAabbBroadPhase3&lt;&gt;(entries)</code>. Copies the entry list; create a new index to change it.', 'Small scenes, correctness baselines, or queries touching most objects. Queries scan the list.'],
            ['<code>BvhAabbBroadPhase3&lt;T&gt;</code>', '<code>new BvhAabbBroadPhase3&lt;&gt;(entries)</code>. Builds a static bounding-volume hierarchy.', 'Many queries over stable bounds. Pruning helps when queries discard large regions.'],
            ['<code>DynamicSpatialHashBroadPhase3&lt;T&gt;</code>', '<code>new DynamicSpatialHashBroadPhase3&lt;&gt;(cellSize)</code>. Changes cell memberships on each mutation.', 'Moving objects and local queries. Check bucket occupancy and the cells covered by large boxes.'],
            ['<code>DynamicBvhBroadPhase3&lt;T&gt;</code>', '<code>new DynamicBvhBroadPhase3&lt;&gt;()</code>. Mutations mark an internal BVH snapshot dirty; the next query rebuilds it.', 'Batches of updates followed by many queries. The first query after each mutation batch pays for a full rebuild.']
          ])}
          <p>The static BVH and linear index expose <code>size()</code>. The linear index also exposes an immutable <code>entries()</code> list. Neither static index copies payload objects. These are workload choices, not latency guarantees; <a href="#/performance">operation costs</a> describe the work performed.</p>`
        },
        {
          id: 'dynamic-lifecycle', title: 'Retain a handle while an object moves',
          html: `<p>Mutable implementations implement <code>MutableRayBroadPhase3&lt;T&gt;</code>. Store the positive <code>long</code> returned by <code>insert</code> beside the application object. After moving it, replace its world-space bounds with <code>updateBounds</code>. Call <code>remove</code> when it leaves the indexed scene.</p>
          ${code('java', 'DynamicIndexExample.java', `import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.contracts.MutableRayBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicBvhBroadPhase3;

public final class DynamicIndexExample {
    private static AxisAlignedBox box(double x) {
        return new AxisAlignedBox(new Vector3(x, 0, 0), new Vector3(x + 1, 1, 1));
    }

    public static void main(String[] args) {
        MutableRayBroadPhase3<String> index = new DynamicBvhBroadPhase3<>();
        long handle = index.insert(box(2), "machine");
        Ray ray = new Ray(new Vector3(0, 0.5, 0.5), new Vector3(1, 0, 0));
        System.out.println("before=" + index.anyRay(ray, 3));
        System.out.println("updated=" + index.updateBounds(handle, box(8)));
        System.out.println("after=" + index.anyRay(ray, 3));
        System.out.println("removed=" + index.remove(handle) + ", size=" + index.size());
        System.out.println("missing=" + index.updateBounds(handle, box(2)));
        index.clear();
        long next = index.insert(box(2), "replacement");
        System.out.println("newHandleIsLarger=" + (next > handle));
    }
}`)}
          ${code('output', 'Expected output', `before=true
updated=true
after=false
removed=true, size=0
missing=false
newHandleIsLarger=true`)}
          <p><code>updateBounds</code> keeps the handle and insertion position. With valid bounds, updating a missing handle returns <code>false</code>; removing a missing handle also returns <code>false</code>. Removal and reinsertion create a new handle and insertion position. <code>clear()</code> removes entries without resetting handle allocation. Handles increase monotonically and are never reused; exhaustion throws <code>IllegalStateException</code>.</p>`
        },
        {
          id: 'hash-cell-size', title: 'Set the spatial-hash cell size deliberately',
          html: `<p><code>cellSize</code> is a finite, positive length in world units. It configures broad-phase buckets independently of <a href="#/grid-tracing">voxel cell size</a>. Cell membership is <code>floor(coordinate / cellSize)</code>, including both AABB endpoints. With cell size 1, a box from X=0 through X=1 occupies both X buckets 0 and 1; X=-0.2 belongs to bucket -1.</p>
          <p>Smaller cells can reduce unrelated objects per bucket, but a large object then occupies more buckets. Larger cells reduce membership work while increasing candidate references in a local query. Ray queries enumerate the ray segment's enclosing AABB, which can cover many empty cells for a long diagonal.</p>
          <p>Cell coordinates must fit a signed Java <code>int</code>. Every enumerated three-dimensional range is limited to <code>Integer.MAX_VALUE</code> cells. An out-of-range coordinate or oversized range throws <code>IllegalArgumentException</code>. A rejected insert or bounds update leaves the index unchanged; a rejected insert does not consume a handle.</p>
          ${note('A representable range can still be expensive', '<p>The built-in limit is an arithmetic guard, not a practical workload budget. Set application limits for query extent, object size, and memory use before accepting externally supplied bounds.</p>')}`
        },
        {
          id: 'stable-scene', title: 'Keep one scene state throughout a query',
          html: `<p><code>AxisAlignedBox</code>, its vector corners, and <code>AabbEntry3</code> are immutable value records. Moving a payload does not move its indexed box. Update both callback geometry and its bounds before the next query. A static index needs a replacement index when its bounds change.</p>
          <p>Keep the index, frame graph, payload geometry, and callback state stable until a query returns. Callbacks must not mutate that state. Mutable indexes require external synchronization for all shared operations, including concurrent reads: a read of <code>DynamicBvhBroadPhase3</code> can rebuild its internal snapshot.</p>
          <p>A static index can be shared after safe publication when callback and payload access are also thread-safe. Static bounds do not freeze mutable payloads. For a coherent application snapshot, capture frame state and geometry together, then build or update the corresponding world-space bounds. See <a href="#/coordinate-frames">coordinate frames</a> for frame snapshots.</p>`
        }
      ]
    },
    {
      id: 'proximity-sweeps', category: 'Broad phase', title: 'Proximity and moving boxes', navTitle: 'Proximity and sweeps', kind: 'guide', readingTime: 7,
      description: 'Measure distance to indexed AABBs and interpret translating-box contact as a fraction of a motion.',
      intro: '<p>Proximity queries filter by distance to bounds. A swept AABB query asks when a box moving by a fixed translation overlaps those bounds. Both operate in world space, so convert local geometry before calling an index directly.</p>',
      sections: [
        {
          id: 'sphere-and-nearest', title: 'Find nearby bounds',
          html: `<p><code>querySphere(center, radius, consumer)</code> emits payloads whose AABBs meet a closed sphere. The center and radius use world units; radius zero tests whether the center is inside or on the box. This does not test an enclosed sphere, mesh, or rotated box.</p>
          <p><code>nearest(point, maxDistance)</code> returns a <code>BroadPhaseNearestHit3</code> or <code>null</code>. It measures the shortest Euclidean distance to the closed AABB, with zero distance inside. The inclusive limit is an ordinary distance, while <code>distanceSquared()</code> is in world units squared.</p>
          ${code('java', 'ProximityExample.java', `import java.util.ArrayList;
import java.util.List;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public final class ProximityExample {
    public static void main(String[] args) {
        var box = new AxisAlignedBox(new Vector3(3, 0, 0), new Vector3(4, 1, 1));
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(box, "target")));
        var nearby = new ArrayList<String>();
        index.querySphere(new Vector3(0, 0.5, 0.5), 3, nearby::add);
        System.out.println("sphere=" + nearby);
        var nearest = index.nearest(new Vector3(1, 0.5, 0.5), 2);
        if (nearest == null) throw new AssertionError("Expected inclusive distance limit");
        System.out.println("squared=" + nearest.distanceSquared()
                + ", distance=" + Math.sqrt(nearest.distanceSquared()));
        System.out.println("inside=" + index.nearest(new Vector3(3.5, 0.5, 0.5), 0).distanceSquared());
        System.out.println("tooFar=" + index.nearest(new Vector3(0, 0.5, 0.5), 2));
    }
}`)}
          ${code('output', 'Expected output', `sphere=[target]
squared=4.0, distance=2.0
inside=0.0
tooFar=null`)}
          <p>For equal nearest distances, the linear index and hash retain the first surviving input/insertion entry. The BVH visits the nearer child first, takes the left child on a tie, and retains the first equal-distance leaf entry encountered. Dynamic BVH inherits that rule. Changing the index can change the winning payload at the same distance.</p>`
        },
        {
          id: 'sweep-time', title: 'Read sweep time as a fraction of the motion',
          html: `<p>Pass the starting <code>movingBounds</code> and a world-space <code>delta</code> to <code>querySweptAabb</code>. At normalized time <code>t</code>, the moving box is translated by <code>delta × t</code>. A <code>BroadPhaseSweepHit3</code> contains the closed contact interval in <code>[0, 1]</code>.</p>
          <p>Here the moving box starts at X=[0,1] and translates four units along +X. The target occupies X=[3,4]. First contact occurs at X=[2,3]: two units into a four-unit motion, so <code>tEnter=0.5</code>. At <code>tExit=1</code>, the boxes still touch at X=4.</p>
          <figure class="my-6 rounded-[7px] border border-line bg-surface p-4">
            <div class="static-trace-viewport" role="region" aria-label="Spatial diagram. Scroll horizontally if needed." tabindex="0">
            <svg viewBox="0 0 720 340" role="img" aria-labelledby="sweep-title sweep-desc" style="display:block;width:100%;height:auto">
              <title id="sweep-title">A moving AABB meets the target halfway through its translation</title>
              <desc id="sweep-desc">Three XY cross sections with overlapping Y and Z. At time zero, the moving box spans X zero to one; the target spans three to four. At time one half, the moving box spans two to three and touches the target's left face. At time one, it spans four to five and touches the right face.</desc>
              <g font-family="ui-monospace, monospace" font-size="13" fill="currentColor">
                <text x="16" y="23">XY cross section · Y=[0,1], overlapping Z</text>
                <text x="18" y="84">t = 0</text><text x="18" y="164">t = 0.5</text><text x="18" y="244">t = 1</text>
                <text x="562" y="84">start</text><text x="562" y="164">first contact</text><text x="562" y="244">last contact</text>
                <text x="153" y="310">0</text><text x="233" y="310">1</text><text x="313" y="310">2</text><text x="393" y="310">3</text><text x="473" y="310">4</text><text x="553" y="310">5</text><text x="601" y="310">X</text>
              </g>
              <g stroke="currentColor" stroke-width="1" opacity="0.25"><path d="M160 48V285 M240 48V285 M320 48V285 M400 48V285 M480 48V285 M560 48V285" stroke-dasharray="3 5"/><path d="M150 287H596"/></g>
              <g stroke="currentColor" stroke-width="2" fill="currentColor" fill-opacity="0.12"><rect x="400" y="56" width="80" height="45"/><rect x="400" y="136" width="80" height="45"/><rect x="400" y="216" width="80" height="45"/></g>
              <g stroke="#26a879" stroke-width="2.5" fill="#26a879" fill-opacity="0.16"><rect x="160" y="56" width="80" height="45"/><rect x="320" y="136" width="80" height="45"/><rect x="480" y="216" width="80" height="45"/><path d="M247 78H380 M371 71L380 78L371 85" fill="none"/></g>
              <g font-family="ui-monospace, monospace" font-size="12" fill="currentColor"><text x="168" y="123">moving box</text><text x="410" y="123">target</text><text x="261" y="67">delta = +4 X</text></g>
            </svg>
            </div>
            <figcaption>Green outlines show the same box at three times. Grey outlines show the fixed target. The interval [0.5,1] measures progress through the supplied translation.</figcaption>
          </figure>
          ${code('java', 'MovingBoxExample.java', `import java.util.ArrayList;
import java.util.List;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public final class MovingBoxExample {
    private static AxisAlignedBox box(double minX, double maxX) {
        return new AxisAlignedBox(new Vector3(minX, 0, 0), new Vector3(maxX, 1, 1));
    }

    public static void main(String[] args) {
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(box(3, 4), "target")));
        var hits = new ArrayList<BroadPhaseSweepHit3<String>>();
        Vector3 delta = new Vector3(4, 0, 0);
        index.querySweptAabb(box(0, 1), delta, hits::add);
        var hit = hits.getFirst();
        System.out.println("contact=[" + hit.tEnter() + "," + hit.tExit() + "]");
        System.out.println("travelToContact=" + delta.length() * hit.tEnter());
    }
}`)}
          ${code('output', 'Expected output', `contact=[0.5,1.0]
travelToContact=2.0`)}`
        },
        {
          id: 'motion-limits', title: 'Define what the motion query can establish',
          html: `<p>A sweep translates a fixed-size axis-aligned box against a stable indexed scene. It does not rotate the box, move indexed targets, or calculate contact normals, collision response, sliding, damage, or a safe post-collision pose. If the boxes enclose detailed shapes, their contact interval is a candidate for your geometry test.</p>
          <p>With zero motion, overlapping boxes return <code>[0,1]</code>; separated boxes produce no result. Starting in overlap gives <code>tEnter=0</code>. Contact exactly at the motion endpoint is included. Nonzero motion components are used as supplied rather than rounded to zero by a tolerance.</p>
          <p>An overlap query on a box enclosing an entire motion path retrieves candidates without contact times. For rotating geometry, bounds enclosing just the initial and final poses can miss the intervening path. Establish bounds that enclose that path before using them to retrieve candidates.</p>
          <p>The four built-in indexes sort sweep hits by entry time, exit time, then their documented stable input position or handle. See <a href="#/broad-phase">bounds candidates</a> for ordering and <a href="#/performance">operation costs</a> for collection and sorting.</p>`
        }
      ]
    }
  );
  window.WIKI_PAGES.push(
    {
      id: 'query-contracts', category: 'Reference', title: 'Query contracts', navTitle: 'Query contracts', kind: 'reference', readingTime: 9,
      description: 'Check units, endpoints, ordering, limits, result ownership, and reusable-buffer behavior.',
      sections: [
        {
          id: 'result-units', title: 'Interpret each result in its own units',
          html: `${table(['Result', 'Parameters and coordinates', 'Meaning'], [
            ['<code>BroadPhaseRayHit3&lt;T&gt;</code>', '<code>tEnter</code>, <code>tExit</code>: nonnegative world distances.', 'Closed AABB interval within the ray or segment range.'],
            ['<code>TraceHit3&lt;T&gt;</code>', 'Same distances; <code>worldPoint</code> is the clipped AABB entry position.', 'Accepted candidate. <code>worldEnterPoint()</code> aliases <code>worldPoint()</code>. Use the same world ray for <code>worldExitPoint(worldRay)</code>.'],
            ['<code>RayIntersection3</code>', 'Finite signed world distances before clipping.', 'One full connected inside interval supplied by the geometry provider. Negative entry can describe an origin inside the shape.'],
            ['<code>ExactTraceHit3&lt;T&gt;</code>', 'Nonnegative world distances and world entry/exit points.', 'Clipped provider interval. <code>enterSurface</code> and <code>exitSurface</code> identify actual boundaries; <code>distanceInside()</code> is <code>tExit − tEnter</code>.'],
            ['<code>GridRayHit3</code>', '<code>x,y,z</code>: grid indices. Parameters: world distances. <code>worldPoint</code>: world position.', 'Occupied cell visited by the configured traverser. A mapped grid keeps its own cell coordinates.'],
            ['<code>BroadPhaseNearestHit3&lt;T&gt;</code>', '<code>distanceSquared</code>: world units squared.', 'Distance to the closed AABB; zero inside.'],
            ['<code>BroadPhaseSweepHit3&lt;T&gt;</code>', '<code>tEnter</code>, <code>tExit</code>: normalized time in <code>[0,1]</code>.', 'Contact while translating an AABB by the supplied delta.']
          ])}
          <p>Ray <code>tMax</code>, proximity <code>radius</code>, and nearest <code>maxDistance</code> are finite, nonnegative world distances. Sweep <code>delta</code> is a finite world-space vector. AABB bounds must be finite even though Ashcore's <code>AxisAlignedBox</code> constructor can represent non-finite corners. Keep derived points, squared distances, and mapped coordinates representable too; finite inputs alone do not prevent floating-point overflow at extreme magnitudes.</p>`
        },
        {
          id: 'boundaries-and-empty-results', title: 'Handle empty results and boundary contact',
          html: `${table(['Condition', 'Behavior'], [
            ['No match', 'Consumer queries emit nothing; list queries return an empty immutable list; single-result queries return <code>null</code>; existence queries return <code>false</code>.'],
            ['AABB face, edge, or corner contact', 'Included, including a zero-length interval.'],
            ['Ray <code>tMax=0</code>', 'AABB and exact queries can match at the origin with <code>[0,0]</code>. The built-in DDA voxel traversal visits no cells.'],
            ['Object contact exactly at <code>tMax</code>', 'Included by the closed object interval. DDA visits <code>[0,tMax)</code>, excluding a cell first entered at that endpoint.'],
            ['Zero-length segment', '<code>IllegalArgumentException</code>. Segment tracing requires finite positive length.'],
            ['Invalid numeric argument', 'Negative or non-finite distance limits, or non-finite bounds, throw <code>IllegalArgumentException</code>. Required null arguments throw <code>NullPointerException</code>.'],
            ['Nonpositive <code>maxHits</code>', '<code>IllegalArgumentException</code>; zero is not a request for an empty list.'],
            ['Callback throws', 'The exception propagates. Reusable buffers release retained payload references during cleanup.']
          ])}
          <p>A ray starting inside an accepted AABB has entry zero and the origin as its hit point. An exact result distinguishes the clipped origin from a real entry surface. At a voxel blocker, the visible object interval is closed at the blocker entry, so an object touching that boundary can be returned. See <a href="#/occlusion">occlusion</a> for combined behavior.</p>`
        },
        {
          id: 'selection-and-ties', title: 'Select first, last, or any intentionally',
          html: `<p>Broad-phase pipeline first/all queries sort AABB intervals by entry, then exit, preserving broad-phase emission order on full ties. Exact first/all queries sort clipped shape intervals by entry, then exit; full ties retain <code>visitRay</code> candidate order followed by provider emission order. The raw index's sorted ray emission order and its visitor order can differ.</p>
          <p><code>FrameExactRayTracer3.lastHit</code> selects the largest clipped exit, then the largest entry. It can differ from the final element of <code>allHits</code>, whose primary sort key is entry. Check <code>exitSurface</code> before treating the last endpoint as a shape boundary. One payload can contribute several disjoint intervals.</p>
          <p><code>anyHit</code> and <code>anyVisibleHit</code> answer existence; their callback order follows the underlying visitor and does not promise a nearest object. Keep callbacks deterministic and query state stable for repeatable results. The library does not promise insertion-order-independent results or cross-version bitwise identity.</p>`
        },
        {
          id: 'limits-and-work', title: 'A result limit does not always limit query work',
          html: `${table(['Query', 'What stops early', 'Work still performed'], [
            ['Broad-phase pipeline <code>firstHit</code> / limited <code>allHits</code>', 'Acceptance callbacks stop after the required accepted results.', 'The index emits all candidates; the pipeline collects and sorts them first.'],
            ['Occluded broad-phase <code>visibleHits</code>', 'Only the returned list is truncated to <code>maxHits</code>.', 'All acceptance callbacks for the visible object query run before truncation.'],
            ['Exact limited <code>allHits</code> / <code>visibleHits</code>', 'Only the returned interval list is truncated.', 'All candidate providers run; accepted intervals are collected and sorted. The limit counts intervals, which can share one payload.'],
            ['Exact <code>firstHit</code> / <code>lastHit</code>', 'Selection retains one best interval.', 'All candidate providers run to establish the correct selection.'],
            ['Existence through <code>visitRay</code>', 'Later candidates can stop after the first accepted result.', 'The active exact provider call runs to completion. Hash candidate collection and custom-index fallback scans can still finish.'],
            ['Grid <code>firstHit</code> / limited <code>allHits</code>', 'Traversal stops at the first occupied cell or requested occupied-cell count.', 'Frame conversion and visits up to that cell.']
          ])}
          <p><code>tMax</code> controls spatial extent; <code>maxHits</code> controls returned count. Neither is a general execution-time budget. Use <a href="#/performance">operation costs</a> to account for candidates, sorting, callbacks, and lazy BVH rebuilds.</p>`
        },
        {
          id: 'result-ownership-and-buffers', title: 'Reuse capacity while keeping results independent',
          html: `<p>Pipeline result lists are immutable snapshots of their hit records. Later buffer reuse or index mutation does not change saved hit coordinates and parameters. Payload values remain references to the original objects, so a mutable payload can still change after a query.</p>
          <p><code>TraceQueryBuffer3&lt;T&gt;</code> reuses temporary candidate/result-list capacity for one query at a time. Pass it to the supported ray overloads of broad-phase first/all queries, exact all queries, or occluded exact visible queries. Buffers clear retained payload references on success and failure while keeping capacity.</p>
          ${code('java', 'QueryBufferExample.java', `import java.util.List;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.TraceQueryBuffer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

public final class QueryBufferExample {
    public static void main(String[] args) {
        var frames = FrameGraph3.worldRoot();
        var box = new AxisAlignedBox(new Vector3(2, 0, 0), new Vector3(3, 1, 1));
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(box, "target")));
        var tracer = new FrameBroadPhaseRayTracer3<>(frames, index);
        var buffer = new TraceQueryBuffer3<String>();
        var ray = new Ray(new Vector3(0, 0.5, 0.5), new Vector3(1, 0, 0));
        var saved = tracer.allHits(frames.root(), ray, 5, (v, r, a, b) -> true, 1, buffer);
        var later = tracer.allHits(frames.root(), ray, 5, (v, r, a, b) -> false, 1, buffer);
        System.out.println("saved=" + saved.size() + ", later=" + later.size());
        System.out.println("savedEntry=" + saved.getFirst().tEnter());
        buffer.trimToSize();
    }
}`)}
          ${code('output', 'Expected output', `saved=1, later=0
savedEntry=2.0`)}
          <p>A buffer is not thread-safe. Recursive reuse from a callback throws <code>IllegalStateException</code>; so does <code>trimToSize()</code> while it is in use. Trim between queries to release retained capacity after a large workload. Buffer reuse does not make a query allocation-free: records, output lists, callbacks, sorting, and index work can still allocate.</p>`
        }
      ]
    },
    {
      id: 'performance', category: 'Reference', title: 'Operation costs', navTitle: 'Performance', kind: 'reference', readingTime: 8,
      description: 'Account for candidate collection, tree rebuilds, hash-cell enumeration, callbacks, and retained storage.',
      intro: '<p>These cost models describe Ashtrace 2.0.0 operations. They explain where work grows; they are not measured latency guarantees. Choose a query by the result you need, then measure it with your plugin’s object distribution and update pattern.</p>',
      sections: [
        {
          id: 'cost-variables', title: 'Count the work behind one query',
          html: `${table(['Symbol', 'Quantity'], [
            ['<code>n</code>, <code>C</code>, <code>K</code>', 'Indexed entries; emitted AABB candidates before filtering; BVH nodes and leaf entries examined (worst case linear in n).'],
            ['<code>H</code>, <code>R</code>, <code>U</code>', 'Hash cells visited, including empty cells; raw bucket references scanned, including repeated handles; unique retrieved handles.'],
            ['<code>S</code>, <code>Rold</code>', 'Cells covered by new bounds; total bucket lengths scanned/shifted when unregistering old bounds.'],
            ['<code>D</code>, <code>v</code>, <code>k</code>', 'Frame ancestry work; visited voxels; returned hits.'],
            ['<code>Q</code>, <code>V</code>', 'Complete broad-phase query work, including its collection/sorting; index work through <code>visitRay</code>.'],
            ['<code>F</code>, <code>G</code>, <code>M</code>', 'Total acceptance-callback cost; total geometry-provider cost; emitted shape intervals.']
          ])}
          <p>Time excludes application callbacks unless a row includes them. Auxiliary space excludes returned results and retained index storage. Hash-map costs assume ordinary hash distribution and amortized growth. A callback scanning geometry can dominate the index cost.</p>`
        },
        {
          id: 'index-costs', title: 'Index construction, mutation, and queries',
          html: `${table(['Operation', 'Time', 'Auxiliary space or retained work'], [
            ['Linear AABB / ray / sphere / nearest', '<code>O(n)</code>', '<code>O(1)</code> live query state; ray records allocate.'],
            ['Linear sweep', '<code>O(n + C log C)</code>', '<code>O(C)</code> sorting buffer.'],
            ['Static BVH construction', '<code>O(n log² n)</code>', '<code>O(n)</code> build/storage. Median splits sort at each level.'],
            ['BVH AABB / sphere / nearest', '<code>O(K)</code>, worst <code>O(n)</code>', '<code>O(log n)</code> balanced-tree stack.'],
            ['BVH ray / sweep', '<code>O(K + C log C)</code>', '<code>O(C + log n)</code>; collection and sorting.'],
            ['Hash insert / remove / update', 'Expected <code>O(S)</code> / <code>O(Rold)</code> / <code>O(Rold + Snew)</code>', 'One retained reference per covered cell. List buckets require scanning and shifting on removal.'],
            ['Hash AABB / sphere', 'Expected <code>O(H + R + U log U)</code>', '<code>O(U)</code> deduplication and handle sorting.'],
            ['Hash ray / sweep', 'Expected <code>O(H + R + U log U + C log C)</code>', '<code>O(U + C)</code>. Enumerates the enclosing AABB.'],
            ['Hash nearest', '<code>O(n)</code>', '<code>O(1)</code>; scans entries without buckets.'],
            ['Dynamic BVH insert / update / remove', 'Expected amortized <code>O(1)</code>', 'Marks the snapshot dirty and retains the previous snapshot.'],
            ['Dynamic BVH first query after changes', '<code>O(n log² n)</code> rebuild plus static BVH query cost', '<code>O(n)</code> rebuild, temporarily retaining the old snapshot too.']
          ])}
          <p>Static indexes retain <code>O(n)</code> entries/tree data. A dynamic BVH retains live entries and its last snapshot. Removed payload references can remain in the snapshot until another query rebuilds it. The hash retains entries, occupied buckets, and cell references; map/list capacities can reflect earlier workload peaks.</p>`
        },
        {
          id: 'pipeline-costs', title: 'Account for the tracing pipeline too',
          html: `${table(['Operation', 'Time', 'Auxiliary work'], [
            ['Frame grid first/all ray or segment hits', '<code>O(D + v)</code> plus occupancy cost', '<code>O(D)</code> conversion; all hits return <code>O(k)</code> output. First/limited traversal can stop early.'],
            ['Frame broad-phase first/all/limited hits', '<code>O(D + Q + C log C + F)</code>', '<code>O(C + D)</code> plus index workspace. Even first-hit collects and sorts all candidates.'],
            ['Occluded broad-phase first/visible hits', 'Previous row plus <code>O(D + v)</code> and occupancy cost', 'Same candidate storage. <code>visibleHits</code> runs all acceptance callbacks before truncation.'],
            ['Index <code>visitRay</code> / <code>anyRay</code>', 'Linear <code>O(n)</code>; BVH <code>O(K)</code>; hash expected <code>O(H + R + U log U)</code> worst-query work', 'Stops later candidate tests. BVH avoids result sorting; hash still collects, deduplicates, and sorts handles. Dynamic BVH can first rebuild.'],
            ['Frame broad-phase <code>anyHit</code>', '<code>O(D + V + F)</code>', 'No pipeline candidate list/sort; callback order follows <code>visitRay</code>.'],
            ['Exact first / last / any', '<code>O(D + V + G + M)</code>', '<code>O(1)</code> live selection state plus conversion/index workspace. Only any-hit stops later candidates.'],
            ['Exact all / limited', '<code>O(D + V + G + M log M)</code>', '<code>O(M + D)</code> plus index workspace. All providers run before sorting/truncation.'],
            ['Exact occluded variants', 'Exact row plus <code>O(D + v)</code> and occupancy cost', 'Same interval storage; voxel visibility clips the interval limit.']
          ])}
          <p>Reusing <code>TraceQueryBuffer3</code> can reuse list capacity. It does not remove these scans or sorts, and no query path has an allocation-free contract. See <a href="#/query-contracts">query contracts</a> for returned-count limits and early termination.</p>`
        },
        {
          id: 'measure-your-scene', title: 'Measure the queries your plugin makes',
          html: `<p>Start with the linear index as a candidate-set baseline. Compare another implementation with the same ordered entries, rays, acceptance rules, and geometry provider. Check candidate sets and distances; implementation-specific ordering and equal-distance winners can differ.</p>
          <p>Measure mutation time separately from the next query when evaluating dynamic BVH. Compare batches of updates followed by many queries with one update before every query. For the hash, record cell size, object and query extents, raw bucket references, and unique candidates. Empty cells are enumerated, so a sparse scene can still have expensive large queries.</p>
          <p>Many overlapping boxes can make <code>C</code> approach <code>n</code>. At fixed cell size, doubling a three-dimensional hash query's extent on every axis visits approximately eight times as many cells. Tight bounds and limited ranges reduce candidate work when they still enclose the actual geometry or motion.</p>
          <p>The repository contains manual workload drivers <code>nsk.nu.ashtrace.benchmark.manual.BroadPhaseBenchmarkMain</code> and <code>nsk.nu.ashtrace.benchmark.manual.TraceWorkloadBenchmarkMain</code> under <code>src/test/java</code>. Their results describe their generated workloads. These drivers are outside the shipped public API and do not establish a server tick-time guarantee.</p>`
        }
      ]
    },
    {
      id: 'api-reference', category: 'Reference', title: 'API map', navTitle: 'API reference', kind: 'reference', readingTime: 8,
      description: 'Find the exact package for every Ashtrace public contract, result, pipeline, and index implementation.',
      intro: '<p>This map covers the public types in Ashtrace 2.0.0. Type links open articles that explain their use. The Java packages below differ from the Maven group ID, <code>dev.nasaka.blackframe</code>.</p>',
      sections: [
        {
          id: 'broadphase-contracts', title: 'Broad-phase query contracts',
          html: apiTable([
            api('nsk.nu.ashtrace.api.broadphase.contracts', 'BroadPhase3', 'broad-phase', '<code>query(bounds, consumer)</code>; closed overlap, one emission per matching entry, stable query state.'),
            api('nsk.nu.ashtrace.api.broadphase.contracts', 'RayQueryableBroadPhase3', 'broad-phase', '<code>queryRay</code>, <code>querySegment</code>, <code>visitRay</code>, <code>anyRay</code>; extends <code>BroadPhase3</code>.'),
            api('nsk.nu.ashtrace.api.broadphase.contracts', 'RayCandidateVisitor3', 'broad-phase', '<code>visit(value, tEnter, tExit)</code>; return false to stop.'),
            api('nsk.nu.ashtrace.api.broadphase.contracts', 'ProximityQueryableBroadPhase3', 'proximity-sweeps', '<code>querySphere</code> and <code>nearest</code>; distances to closed AABBs.'),
            api('nsk.nu.ashtrace.api.broadphase.contracts', 'SweepQueryableBroadPhase3', 'proximity-sweeps', '<code>querySweptAabb(movingBounds, delta, consumer)</code>; translating bounds and normalized time.'),
            api('nsk.nu.ashtrace.api.broadphase.contracts', 'MutableRayBroadPhase3', 'indexes', '<code>insert</code>, <code>updateBounds</code>, <code>remove</code>, <code>size</code>, <code>clear</code>; combines ray, proximity, and sweep contracts.')
          ])
        },
        {
          id: 'result-models', title: 'Entry and result records',
          html: apiTable([
            api('nsk.nu.ashtrace.api.broadphase.model', 'AabbEntry3', 'broad-phase', '<code>(bounds, value)</code>: indexed world bounds and retained payload.'),
            api('nsk.nu.ashtrace.api.broadphase.model', 'BroadPhaseRayHit3', 'query-contracts', '<code>(value, tEnter, tExit)</code>: closed ray/segment AABB interval.'),
            api('nsk.nu.ashtrace.api.broadphase.model', 'BroadPhaseNearestHit3', 'proximity-sweeps', '<code>(value, distanceSquared)</code>: distance to the closed AABB.'),
            api('nsk.nu.ashtrace.api.broadphase.model', 'BroadPhaseSweepHit3', 'proximity-sweeps', '<code>(value, tEnter, tExit)</code>: normalized sweep contact interval.'),
            api('nsk.nu.ashtrace.api.trace.model', 'TraceHit3', 'broad-phase', '<code>(value, tEnter, tExit, worldPoint)</code>; helpers <code>worldEnterPoint()</code>, <code>worldExitPoint(worldRay)</code>.'),
            api('nsk.nu.ashtrace.api.trace.model', 'RayIntersection3', 'exact-tracing', '<code>(tEnter, tExit)</code>: full signed shape interval before clipping.'),
            api('nsk.nu.ashtrace.api.trace.model', 'ExactTraceHit3', 'exact-tracing', '<code>(value, tEnter, tExit, worldEnterPoint, worldExitPoint, enterSurface, exitSurface)</code>; <code>distanceInside()</code>.'),
            api('nsk.nu.ashtrace.api.trace.model', 'GridRayHit3', 'grid-tracing', '<code>(x, y, z, tEnter, tExit, worldPoint)</code>: grid indices with world-distance hit data.')
          ])
        },
        {
          id: 'tracing-pipelines', title: 'Geometry callbacks and frame pipelines',
          html: apiTable([
            api('nsk.nu.ashtrace.api.trace.contracts', 'RayIntersector3', 'exact-tracing', '<code>intersect(value, worldRay, tMin, tMax, output)</code>; synchronously emit full shape intervals.'),
            api('nsk.nu.ashtrace.api.trace.pipeline', 'FrameBroadPhaseRayTracer3', 'broad-phase', '<code>firstHit</code>, <code>anyHit</code>, <code>allHits</code>, and segment variants; accepted AABB intervals. Exposes <code>frames()</code>, <code>broadPhase()</code>.'),
            api('nsk.nu.ashtrace.api.trace.pipeline', 'FrameBroadPhaseRayTracer3.NarrowPhase3', 'broad-phase', 'Nested callback. <code>test(value, worldRay, tEnter, tExit)</code> accepts or rejects an AABB interval.'),
            api('nsk.nu.ashtrace.api.trace.pipeline', 'FrameExactRayTracer3', 'exact-tracing', '<code>firstHit</code>, <code>lastHit</code>, <code>anyHit</code>, <code>allHits</code>, and segment variants. Exposes <code>frames()</code>, <code>broadPhase()</code>.'),
            api('nsk.nu.ashtrace.api.trace.pipeline', 'FrameGridRayTracer3', 'grid-tracing', '<code>firstHit</code>, <code>allHits</code>, and segment variants. Unit-world constructor or mapped <code>forGrid</code>. Exposes <code>frames()</code>, <code>traverser()</code>, <code>grid()</code>.'),
            api('nsk.nu.ashtrace.api.trace.pipeline', 'FrameOccludedBroadPhaseRayTracer3', 'occlusion', '<code>firstVisibleHit</code>, <code>anyVisibleHit</code>, <code>visibleHits</code>, <code>firstVisibleSegmentHit</code>, <code>visibleSegmentHits</code>; constructor or <code>forGrid</code>.'),
            api('nsk.nu.ashtrace.api.trace.pipeline', 'FrameOccludedExactRayTracer3', 'occlusion', '<code>firstVisibleHit</code>, <code>lastVisibleHit</code>, <code>anyVisibleHit</code>, <code>visibleHits</code>, and segment variants; constructor or <code>forGrid</code>.'),
            api('nsk.nu.ashtrace.api.trace.pipeline', 'TraceQueryBuffer3', 'query-contracts', 'Exclusive reusable list capacity for supported ray-query overloads; <code>trimToSize()</code> releases idle capacity.')
          ]) + '<p>Both occluded pipelines expose <code>objectTracer()</code> and <code>voxelTracer()</code>. They use a voxel blocker to restrict the object query. The linked articles provide usage details and runnable examples.</p>'
        },
        {
          id: 'implementations', title: 'Built-in indexes and internal helpers',
          html: apiTable([
            api('nsk.nu.ashtrace.implementation.broadphase.staticindex', 'LinearAabbBroadPhase3', 'indexes', 'Copied list baseline; constructor takes <code>List&lt;AabbEntry3&lt;T&gt;&gt;</code>; exposes <code>size()</code>, <code>entries()</code>.'),
            api('nsk.nu.ashtrace.implementation.broadphase.staticindex', 'BvhAabbBroadPhase3', 'indexes', 'Static BVH; constructor takes <code>List&lt;AabbEntry3&lt;T&gt;&gt;</code>; exposes <code>size()</code>.'),
            api('nsk.nu.ashtrace.implementation.broadphase.dynamic', 'DynamicSpatialHashBroadPhase3', 'indexes', 'Mutable cell buckets; constructor takes finite positive <code>cellSize</code>; exposes <code>cellSize()</code> and mutable operations.'),
            api('nsk.nu.ashtrace.implementation.broadphase.dynamic', 'DynamicBvhBroadPhase3', 'indexes', 'No-argument constructor; mutable entries and a lazily rebuilt BVH snapshot.'),
            api('nsk.nu.ashtrace.implementation.broadphase.internal', 'BroadPhaseMath3', 'query-contracts', 'Publicly visible implementation helper for inclusive tests, ray/sweep intervals, enclosing bounds, distances, and finite-value checks.'),
            api('nsk.nu.ashtrace.implementation.broadphase.internal', 'BroadPhaseMath3.Interval', 'query-contracts', 'Nested helper record <code>(tEnter, tExit)</code> used inside index implementations.')
          ]) + '<p>The <code>implementation.broadphase.internal</code> package is included for a complete public-symbol map. Application examples use the query contracts and result records, which provide argument validation and documented result semantics.</p>'
        },
        {
          id: 'dependency-types', title: 'Types supplied by other Blackframe libraries',
          html: `${apiTable([
            api('nsk.nu.ashcore.api.geometry', 'AxisAlignedBox, Ray, Segment3, OrientedBox', 'exact-tracing', 'Bounds and geometry inputs. <code>Ray</code> normalizes direction; Ashcore collision tests can supply OBB intervals.'),
            api('nsk.nu.ashcore.api.math', 'Vector3, Quaternion', 'coordinate-frames', 'Immutable vector and rotation values.'),
            api('nsk.nu.ashcore.api.collision', 'CollisionTests', 'exact-tracing', 'Geometry tests for caller-owned intersectors.'),
            api('nsk.nu.ashcore.api.spi', 'ServiceRegistry', 'installation', 'Loads the Ashgrid voxel traverser by provider ID.'),
            api('nsk.nu.ashspace.api.frame', 'FrameGraph3, FrameId', 'coordinate-frames', 'Frame hierarchy and frame identities.'),
            api('nsk.nu.ashspace.api.transform', 'RigidTransform3', 'coordinate-frames', 'Rigid translation and rotation between frames.'),
            api('nsk.nu.ashspace.api.space', 'SpaceConverter3', 'coordinate-frames', 'Converts source-frame geometry to another frame.'),
            api('nsk.nu.ashspace.api.grid', 'FrameGridSpaceMapper3', 'grid-tracing', 'Grid frame, local origin, and cell-size mapping.'),
            api('nsk.nu.ashgrid.api.voxel.traversal', 'VoxelTraverser', 'grid-tracing', 'Cell traversal; built-in provider ID <code>dda</code>.'),
            api('nsk.nu.ashgrid.api.voxel.query', 'Raycast.Occupancy', 'grid-tracing', 'Nested callback receiving grid cell indices and reporting occupancy.')
          ])}
          <p>Ashtrace declares no plugin commands or Bukkit/Paper listeners. Your integration owns world selection, block/entity access, scheduling, geometry, and gameplay consequences. <a href="#/installation">Installation</a> lists dependency versions; <a href="#/examples">examples</a> connect these types into complete tracing flows.</p>`
        }
      ]
    }
  );
})();
