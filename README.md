# Ashtrace

Ashtrace finds ray, overlap, proximity and moving-box candidates in indexed AABBs, selects caller-supplied
shape intersections, and traces voxel grids attached to coordinate frames.

Version **2.0.0** is available from [Maven Central](https://central.sonatype.com/artifact/dev.nasaka.blackframe/ashtrace/2.0.0).

The [WIKI](https://miciasty.github.io/Ashtrace/) provides installation instructions, runnable examples,
interactive explanations, and the API reference. See [WIKI authoring and publication](wiki/README.md)
to preview or build the documentation from this checkout.

> [!NOTE]
> Ashtrace is the tracing layer above Ashspace and Ashgrid:
>
> - converts source-frame rays/segments to world space,
> - filters object candidates with broad-phase AABB queries,
> - selects entry/exit intervals supplied by a geometry callback,
> - optionally clips object intervals at the first occupied voxel in a mapped grid.
>
> Mesh extraction belongs to Ashmesh. Navigation/pathfinding belongs to Ashnav.

## When to use it

Use Ashtrace when:

- you need deterministic broad-phase candidate filtering over AABBs,
- you need frame-aware ray queries that combine Ashspace transforms with Ashgrid traversal,
- you need reusable low-level tracing blocks for LOS, hit-scan, and candidate prefiltering.

Do not use Ashtrace when:

- you need rendering or mesh extraction pipelines,
- you need full navigation/pathfinding logic,
- you need project-specific hit resolution, damage, or gameplay rules.

## Example: a drill on a moving machine

You have a mining drill mounted on a moving machine:

1. Drill ray is defined in the drill local frame.
2. The machine moves/rotates in world space.
3. You convert the local ray to world space using frame graph rules.
4. You trace voxels in deterministic order and return the first solid block hit.

Ashtrace centralizes this pipeline so every tool uses the same deterministic trace behavior.

## Requirements and quick start

Use JDK 21 or newer. Add this dependency to your Maven project:

```xml
<dependency>
  <groupId>dev.nasaka.blackframe</groupId>
  <artifactId>ashtrace</artifactId>
  <version>2.0.0</version>
</dependency>
```

Maven downloads Ashtrace and its transitive dependencies from Maven Central: Ashcore 1.2.0,
Ashgrid 1.3.0 and Ashspace 2.0.0. No additional repository configuration or local dependency
installation is required.

To build Ashtrace from source, use Maven 3.9+ and run `mvn -B clean verify` in this checkout.

Minimal frame-aware voxel trace:

```java
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.trace.model.GridRayHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;

public final class AshtraceQuickStart {
    public static void main(String[] args) {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId world = frames.root();
        FrameId tool = new FrameId("tool");
        frames.define(tool, world, RigidTransform3.translation(10, 0, 0));

        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);

        Ray localRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));
        GridRayHit3 hit = tracer.firstHit(tool, localRay, 10.0, (x, y, z) -> x == 13 && y == 1 && z == 0);

        System.out.println(hit);
    }
}
```

Minimal broad-phase trace pipeline. Accepting every candidate selects `targetA`'s AABB entry at
world X=12, distance 1.8 from the transformed ray origin. If these boxes only enclose more detailed
shapes, this result does not locate those shapes' surfaces.

```java
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.model.TraceHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;

import java.util.List;

public final class AshtraceBroadPhaseQuickStart {
    public static void main(String[] args) {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId tool = new FrameId("tool");
        frames.define(tool, frames.root(), RigidTransform3.translation(10, 0, 0));

        BvhAabbBroadPhase3<String> broadPhase = new BvhAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(new AxisAlignedBox(new Vector3(12, 1, 0), new Vector3(13, 2, 1)), "targetA"),
                new AabbEntry3<>(new AxisAlignedBox(new Vector3(16, 1, 0), new Vector3(17, 2, 1)), "targetB")
        ));

        FrameBroadPhaseRayTracer3<String> tracer = new FrameBroadPhaseRayTracer3<>(frames, broadPhase);
        Ray localRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        TraceHit3<String> hit = tracer.firstHit(tool, localRay, 10.0, (value, worldRay, tEnter, tExit) -> true);
        System.out.println(hit);
    }
}
```

A grid attached to a moving frame. Cells have side length 2; grid origin `(1,0,0)` is expressed in
the ship frame. The occupancy callback receives grid indices. Cell `(1,0,0)` starts at world X=13,
1.5 world units from this ray's origin. Changing the ship transform affects the next query;
use `grid.snapshot()` when a fixed frame state is required.

```java
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

public final class AshtraceMappedGridQuickStart {
    public static void main(String[] args) {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId ship = new FrameId("ship");
        frames.define(ship, frames.root(), RigidTransform3.translation(10, 0, 0));
        FrameGridSpaceMapper3 grid = new FrameGridSpaceMapper3(
                frames, ship, 2, new Vector3(1, 0, 0), new SquareXZChunkScheme(16));
        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = FrameGridRayTracer3.forGrid(grid, traverser);
        Ray ray = new Ray(new Vector3(1.5, 0.5, 0.5), new Vector3(1, 0, 0));

        System.out.println(tracer.firstHit(ship, ray, 10, (x, y, z) -> x == 1 && y == 0 && z == 0));
    }
}
```

Entry and exit through a sphere enclosed by a larger AABB. The example owns its sphere solver;
Ashtrace owns candidate enumeration, interval selection, clipping and world result points. This
small-coordinate example is not a general numeric-robustness implementation for arbitrary geometry.

```java
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameExactRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

import java.util.List;

public final class AshtraceExactQuickStart {
    private record Ball(Vector3 center, double radius) {}

    public static void main(String[] args) {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        Ball ball = new Ball(new Vector3(5, 0, 0), 1);
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(
                new AxisAlignedBox(new Vector3(3, -2, -2), new Vector3(7, 2, 2)), ball)));
        var tracer = new FrameExactRayTracer3<>(frames, index);
        RayIntersector3<Ball> geometry = (value, worldRay, min, max, output) -> {
            Vector3 offset = worldRay.origin().sub(value.center());
            double b = offset.dot(worldRay.direction());
            double discriminant = b * b - (offset.lengthSq() - value.radius() * value.radius());
            if (discriminant < 0) return;
            double root = Math.sqrt(discriminant);
            double enter = -b - root;
            double exit = -b + root;
            if (exit >= min && enter <= max) output.accept(new RayIntersection3(enter, exit));
        };
        Ray ray = new Ray(new Vector3(0, 0, 0), new Vector3(1, 0, 0));
        var hit = tracer.firstHit(frames.root(), ray, 10, geometry);
        if (hit == null || !hit.enterSurface() || !hit.exitSurface()) throw new AssertionError("Missing surfaces");
        System.out.println("entry=" + hit.worldEnterPoint().x() + " exit=" + hit.worldExitPoint().x()
                + " distanceInside=" + hit.distanceInside());

        var partial = tracer.firstHit(frames.root(), ray, 5, geometry);
        if (partial.exitSurface() || partial.tExit() != 5) throw new AssertionError("Expected range clipping");
        Ray inside = new Ray(ball.center(), ray.direction());
        var outgoing = tracer.firstHit(frames.root(), inside, 10, geometry);
        if (outgoing.enterSurface() || !outgoing.exitSurface() || outgoing.tExit() != 1) {
            throw new AssertionError("Expected an origin inside the ball");
        }
    }
}
```

The first result contains both surface points: X=4 and X=6. A range of 5 stops inside the sphere,
so its exit is X=5 with `exitSurface=false`. A ray starting at the sphere's center has entry zero
with `enterSurface=false` and a real exit at X=6. Whether a projectile penetrates, how it loses
energy and what damage it causes remain application decisions.

An oriented box uses Ashspace to preserve the rotated shape and to compute its separate AABB.
The geometry callback uses Ashcore's full interval directly. The box below is rotated 45 degrees
around Z and moved to X=6. A ray along world X crosses the shape at approximately 5.293 and 6.707;
the enclosing AABB begins earlier. A second ray passes through an empty corner of that AABB.

```java
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

import java.util.List;

public final class AshtraceOrientedBoxQuickStart {
    public static void main(String[] args) {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId body = new FrameId("body");
        Vector3 zAxis = new Vector3(0, 0, 1);
        frames.define(body, frames.root(), new RigidTransform3(
                Quaternion.fromAxisAngle(zAxis, Math.PI / 4), new Vector3(6, 0, 0)));
        SpaceConverter3 converter = new SpaceConverter3(frames);
        AxisAlignedBox local = new AxisAlignedBox(new Vector3(-2, -0.5, -0.5), new Vector3(2, 0.5, 0.5));
        OrientedBox shape = converter.orientedBox(local, body, frames.root());
        AxisAlignedBox bounds = converter.axisAlignedBox(local, body, frames.root());
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(bounds, shape)));
        var tracer = new FrameExactRayTracer3<>(frames, index);
        RayIntersector3<OrientedBox> geometry = (value, worldRay, min, max, output) -> {
            var interval = CollisionTests.rayVsOrientedBoxInterval(worldRay, value);
            if (interval.hit() && interval.tExit() >= min && interval.tEnter() <= max) {
                output.accept(new RayIntersection3(interval.tEnter(), interval.tExit()));
            }
        };
        Ray ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        var hit = tracer.firstHit(frames.root(), ray, 12, geometry);
        if (hit == null || Math.abs(hit.tEnter() - (6 - Math.sqrt(0.5))) > 1e-12
                || Math.abs(hit.tExit() - (6 + Math.sqrt(0.5))) > 1e-12) throw new AssertionError("OBB interval");
        System.out.println("entry=" + hit.worldEnterPoint() + " exit=" + hit.worldExitPoint());
        Ray corner = new Ray(new Vector3(7.5, -1.5, -2), zAxis);
        if (!index.anyRay(corner, 4) || tracer.anyHit(frames.root(), corner, 4, geometry)) {
            throw new AssertionError("Expected empty space inside the AABB");
        }

        // Three settings of a prescribed half-turn about Z, with the bar centered at zero.
        frames.define(body, frames.root(), RigidTransform3.identity());
        AxisAlignedBox start = converter.axisAlignedBox(local, body, frames.root());
        frames.define(body, frames.root(), new RigidTransform3(new Quaternion(0, 0, 0, 1), Vector3.ZERO));
        AxisAlignedBox end = converter.axisAlignedBox(local, body, frames.root());
        if (!start.equals(end)) throw new AssertionError("Half-turn endpoint bounds");
        Ray crossing = new Ray(new Vector3(0, 1.5, -2), zAxis);
        var endpointIndex = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(start, "endpoints")));
        if (endpointIndex.anyRay(crossing, 4)) throw new AssertionError("Endpoint bounds should miss");

        frames.define(body, frames.root(), new RigidTransform3(Quaternion.fromAxisAngle(zAxis, Math.PI / 2), Vector3.ZERO));
        var middleShape = converter.orientedBox(local, body, frames.root());
        var middleBounds = converter.axisAlignedBox(local, body, frames.root());
        var middleIndex = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(middleBounds, middleShape)));
        var middleTracer = new FrameExactRayTracer3<>(frames, middleIndex);
        if (!middleTracer.anyHit(frames.root(), crossing, 4, geometry)) throw new AssertionError("Midpoint should hit");
        System.out.println("Rotated shape verified; endpoint bounds miss the midpoint contact.");
    }
}
```

The half-turn check demonstrates contact between the endpoints of one prescribed rotation. These are
three independent pose queries; they do not find a continuous time of contact or prove that sampling
catches every contact. The same bar can complete a full turn with identical start/end orientations.
`querySweptAabb` translates an AABB against the indexed state and returns normalized time in `[0,1]`;
it does not rotate the box. A caller-supplied bound covering the entire path can retrieve candidates
through `query`, but gives no contact time. Continuous rotation queries are not supported.

The example stores immutable world shapes. After a pose change, recompute both shape and bound;
rebuild a static index, or update a dynamic index and the geometry used by its provider before the
next query. `updateBounds` changes only bounds, so an immutable world-shape payload also needs
replacement by remove/insert, or the provider can resolve fresh geometry through a stable object ID.
Keep the complete state fixed during the query. Saved result points and distances retain their values;
payload references themselves are not deep copies.

For `p` box parts, the provider can collect the full Ashcore intervals, sort by entry then exit, and
merge overlapping or touching intervals before emitting them. Preserve each positive gap. This takes
`O(p log p)` time and `O(p)` workspace in the simple list implementation; one OBB query is `O(1)`.
Ashtrace then performs the selection/sorting costs in section 6. Bounds conversion also costs frame
ancestry work; dynamic updates have the index costs listed there. No allocation-free claim is made.
The multipart integration test covers overlapping parts, a cavity and clipping by a rotated grid with
cell size 2. The library does not implement a compound-body physics model or merge provider output.

Use `rayVsOrientedBoxInterval` even when a tracer query starts from a segment: it retains negative
entry and the exit beyond the query limit, so clipped endpoints receive the correct surface flags.
Ashcore's `segmentVsOrientedBoxInterval` instead returns clipped fractions in `[0,1]`; those values
must not be passed directly as world distances. Floating-point conversion and collision limits from
Ashspace/Ashcore still apply, especially near tangency and at large translations. The test tolerance
above compares small-coordinate results; it does not inflate the shape or establish a universal bound.

## How it works

1. `FrameGridRayTracer3` converts source-frame rays through Ashspace; `forGrid` accepts a grid mapper with its own frame, origin and cell size.
2. Voxel tracing supports first-hit and all-hit queries for both rays and finite segments.
3. `FrameBroadPhaseRayTracer3` performs frame-aware broad-phase candidate tracing with a boolean acceptance callback.
4. `FrameOccludedBroadPhaseRayTracer3` combines object broad-phase tracing with first-voxel occlusion clipping.
5. `FrameExactRayTracer3` selects actual shape intervals reported by `RayIntersector3`. Results contain both world points and distinguish surface endpoints from clipped endpoints. `FrameOccludedExactRayTracer3` adds voxel clipping.
6. `RayQueryableBroadPhase3`, `ProximityQueryableBroadPhase3`, and `SweepQueryableBroadPhase3` define deterministic query contracts implemented by `LinearAabbBroadPhase3`, `BvhAabbBroadPhase3`, `DynamicSpatialHashBroadPhase3`, and `DynamicBvhBroadPhase3`.
7. `visitRay` and `anyHit` can stop once a match is found. `TraceQueryBuffer3` reuses temporary list capacity across ordered queries.

## Operation costs

Definitions:

- `n`: number of indexed broad-phase entries.
- `v`: number of visited voxels along a ray.
- `C`: number of emitted bounds candidates, before acceptance filtering.
- `K`: BVH nodes and leaf entries examined; worst case `O(n)`.
- `H`: hash cells visited in the query's enclosing AABB, even if their buckets are empty.
- `R`: raw bucket references scanned, including repeated handles; `U`: unique retrieved handles (`U <= R`).
- `S`: cells covered by a new entry; `Rold`: total bucket lengths scanned/shifted when unregistering old bounds.
- `Q`: complete broad-phase query work, including its own sorting and candidate collection.
- `m`: acceptance callback calls (`m <= C`); `F`: their total cost, `O(m)` only for constant-cost callbacks.
- `D`: frame ancestry work in conversion; `k`: returned hits; `B`: occupied hash buckets.
- `V`: index work through `visitRay`; `G`: total geometry-provider work; `M`: emitted shape intervals.

Times below exclude user callbacks unless explicitly included. Auxiliary space excludes returned output
and retained index storage. Map access assumes ordinary hash distribution and amortized growth, not constant worst-case access.

| Operation | Time | Auxiliary space / retained work |
| --- | --- | --- |
| Linear AABB/ray/sphere/nearest | `O(n)` | `O(1)` live query state; ray hits allocate records. |
| Linear sweep | `O(n + C log C)` | `O(C)` sorting buffer. |
| Static BVH build | `O(n log² n)` | `O(n)` total build/storage, median splits with per-level sorting. |
| BVH AABB/sphere/nearest | `O(K)`, worst `O(n)` | `O(log n)` balanced-tree stack. Pruning depends on the workload. |
| BVH ray/sweep | `O(K + C log C)` | `O(C + log n)`; results are collected and sorted. |
| Hash insert / remove / update | expected `O(S)` / `O(Rold)` / `O(Rold + Snew)` | Buckets are lists: removal scans and shifts, not constant per cell. Registration retains one reference per covered cell. |
| Hash AABB/sphere | expected `O(H + R + U log U)` | `O(U)` deduplication and handle sorting, even when few entries match. |
| Hash ray/sweep | expected `O(H + R + U log U + C log C)` | `O(U + C)`; hash cells cover the enclosing box, not only the ray line. |
| Hash nearest | `O(n)` | `O(1)`; scans entries without using buckets. |
| Dynamic BVH insert/update/remove | expected amortized `O(1)` | Marks snapshot dirty; existing snapshot remains retained. |
| Dynamic BVH first query after mutations | build `O(n log² n)` **plus query cost above** | `O(n)` rebuild, temporarily retaining the old snapshot too. Later queries use static BVH costs. |
| Frame grid first/all ray or segment hits | `O(D + v)` plus occupancy cost | `O(D)` conversion, `O(k)` output for all hits; first/limited traversal can stop early. |
| Frame broad-phase first/all/limited hits | `O(D + Q + C log C + F)` | `O(C + D)` plus index query workspace and `O(k)` output. Even first-hit collects and sorts all candidates. |
| Frame occluded first/visible hits | previous row plus `O(D + v)` and occupancy cost | Same candidate storage. `visibleHits` runs all acceptance callbacks before truncating to `maxHits`. |
| Index `visitRay` / `anyRay` | Linear `O(n)`, BVH `O(K)`, hash expected `O(H + R + U log U)` worst case | Stops later candidate tests. BVH avoids result sorting; hash still collects/deduplicates/sorts handles first. Dynamic BVH may rebuild first. |
| Frame broad-phase `anyHit` | `O(D + V + F)` | No pipeline candidate list/sort; callback order follows `visitRay`. |
| Exact first / last / any | `O(D + V + G + M)` | `O(1)` live selection state plus conversion/index workspace. Each emitted interval can allocate a result. Only any-hit stops later candidates. |
| Exact all / limited | `O(D + V + G + M log M)` | `O(M + D)` plus index workspace; queries all providers before sorting and truncating. |
| Exact occluded variants | Exact row plus `O(D + v)` and occupancy cost | Same interval storage; the visible limit clips shape intervals. |

Static indexes retain `O(n)` entries/tree data. A dynamic BVH retains live entries and its last snapshot;
removing entries does not release the old snapshot until another query rebuilds it. The hash retains
entries, `B` buckets and their cell references; list/map capacities can reflect earlier peaks until
entries/buckets are removed. No query path is claimed allocation-free.

Many overlapping boxes can make `C` nearly `n`. Making a three-dimensional hash query twice as wide
on every axis visits about eight times as many cells at fixed cell size. These are cost models,
not measured latency guarantees. Performance depends on object distribution and mutation patterns.

## What a hit, nearest result and visible result mean

`TraceHit3` preserves the accepted AABB's closed `[tEnter,tExit]` interval, clipped to the query.
`worldPoint` (the hit point) is `worldRay.at(tEnter)`. Starting inside a box gives entry zero and the
ray origin as the point. `firstHit` sorts by AABB entry, then exit, then broad-phase emission order.
A boolean `NarrowPhase3` can reject candidates but cannot replace these distances or their ordering.

For example, A has bounds from X=1 to X=10 and a surface at X=9. B has bounds from X=3 to X=5
and a surface at X=4. A ray from X=0 accepts both: `firstHit` returns A at X=1 even though B's surface
is nearer. For exact nearest-surface selection, collect candidates, calculate each actual surface
parameters using `RayIntersector3` and let `FrameExactRayTracer3` select the minimum. The original
boolean callback and `TraceHit3` retain their AABB meaning. `TraceHit3.worldEnterPoint()` aliases
`worldPoint()`; `worldExitPoint(worldRay)` evaluates its clipped exit on the same world ray.
`nearest` similarly measures distance to the closed AABB, zero inside it, rather than an enclosed surface.
Sweeps describe a translating AABB's contact interval over normalized time `[0,1]`; time is not distance
and no collision response or rotation during motion is computed.

The original voxel tracer constructors use world unit cells rooted at `(0,0,0)`. Cell `(-1,0,0)`
includes X=-0.2. The `forGrid(mapper, ...)` factories use the mapper's frame, grid origin and uniform
cell size. Convert the ray to that frame, subtract the grid origin and divide position and distance
by cell size for traversal; results convert distances back to world units. Integer result coordinates
and occupancy arguments belong to the mapped grid. Its chunk scheme does not change voxel geometry.
The traverser receives a ray in unit cell coordinates. Configure `VoxelTraversers.clipped` bounds
in those cell coordinates too; a world-space clipping box needs conversion before configuration.
Mapped tracing rejects a nonzero origin offset that divides to zero, as well as unrepresentable
distance limits, before querying occupancy. This avoids silently moving the ray onto a cell boundary.
Spatial hash `cellSize` controls the object index independently. Rigid frame transforms have no scale or shear.
World conventions follow Ashspace: right-handed, Y up. Rays are normalized; `t` and `tMax` measure
world distance, including after rigid rotation/translation. Segment limits use their length.

With the tested `dda` provider, voxels cover `[0,tMax)` and exact ties step X, then Y, then Z,
including zero-length intermediate visits. DDA does not visit every cell touched at an edge/corner;
it is not supercover. Another traverser can change the occluder selected at a tie.
Ashtrace follows Ashgrid `Raycast` by checking the starting cell. Ashgrid `LineOfSight` deliberately
skips that cell, so its result can differ when the ray starts inside an occupied cell.
Object bounds use closed `[0,limit]`, where `limit` is the first occupied voxel's entry or `tMax`:

| Case | Result |
| --- | --- |
| AABB starts exactly at wall entry | Its zero-length contact interval can be accepted. |
| Ray starts in an occupied callback cell | Limit is zero; bounds containing the origin can still match. |
| `tMax=0` | No voxel visits or occupancy calls; AABBs containing the origin can match. |
| Occupied cell starts exactly at `tMax` | It is not visited; object contact at `tMax` is still included. |
| Zero-length segment | Rejected with `IllegalArgumentException` by all segment routes. |
| No occupied cell | Object limit stays at `tMax`. |

An AABB spanning X=1..10 can be accepted before a wall at X=5 even if its surface is at X=9.
"Visible" describes this clipped candidate. A callback doing detailed geometry must accept only
intersections in its supplied **clipped** interval; returning true without that check does not prove
surface visibility. Both sides must use the same fixed frame, occupancy and geometry state.

### Shape intervals and the last intersection

`RayIntersector3` emits full finite `RayIntersection3` intervals on the world ray's supporting line.
Entry may be negative; exit may exceed the query limit. The tracer clips to the closed query interval
and sets `enterSurface`/`exitSurface` only when that endpoint was not clipped. Do not preclip provider
output: that would label a range limit as a real surface. The indexed AABB must enclose the shape;
the query-clipped output must fit the candidate interval, otherwise the tracer rejects it.
The provider owns geometric correctness and numeric accuracy; the word "exact" distinguishes shape
measurements from AABB candidates, not exact arithmetic or a built-in mesh/primitive solver.

| Operation / result | Meaning |
| --- | --- |
| Exact `firstHit` | Interval with the smallest clipped entry, then exit. Starting inside returns entry zero. |
| `hit.worldExitPoint()` | Exit of this particular interval. Check `hit.exitSurface()` to distinguish a surface from a clipped endpoint. |
| Exact `lastHit` | Interval with the greatest clipped exit, then entry, across all matching objects and shape parts. It need not be the last element of `allHits`. |
| Exact `allHits` | All intervals sorted by entry, then exit. One object can appear several times. |
| `distanceInside()` | Length of this clipped interval. It can be only part of the full path through the object. |

A closed convex solid normally contributes one interval. A shell crossed on both sides contributes
two material intervals separated by its cavity. Emit them separately; a surface-only intersection or
tangent can be `[t,t]`. Full ties preserve `visitRay` candidate order, then provider output order.
Callbacks run synchronously and must not retain the output consumer. They must emit disjoint intervals
for each candidate; Ashtrace does not merge or validate the provider's topology.

`FrameOccludedExactRayTracer3` uses the same rules after shortening the query at the first occupied
voxel. A shape behind a wall is rejected even when its loose AABB begins before the wall. An interval
crossing the wall gets `exitSurface=false`; contact exactly at the wall is included. Its `forGrid`
factory shares the mapper's frame graph between object and voxel stages.

## Ordering, ownership and numeric limits

Repeatability covers the same implementation/dependency versions, Java environment, configuration,
ordered entries, mutation history, frame/grid state and deterministic callbacks. It does not promise
insertion-order independence, cross-platform bitwise equality or identical output across versions.
The common contract between indexes is matching candidate entries and interval/distance values;
AABB/sphere emission order and the selected equally near payload can differ.

| Index | AABB / sphere | Ray / sweep | Equal-distance nearest |
| --- | --- | --- | --- |
| Linear | Input list order | Ray: input order. Sweep: entry, exit, input position. | First input entry. |
| Static BVH | Left-to-right tree/leaf order. | Entry, exit, original input position. | First entry visited; nearer child first, left child on equal node distance. |
| Dynamic hash | Ascending handle. | Entry, exit, handle. | First surviving insertion. |
| Dynamic BVH | Static BVH rules on a snapshot of surviving insertion order. | Static BVH rules. | Static BVH rules. |

The table describes `queryRay`, not the new stoppable `visitRay`. The latter visits linear entries
in input order, BVH leaves left-to-right, hash handles in ascending order, and dynamic BVH entries
in snapshot tree order. It returns false when the visitor requests a stop, even on the last candidate.
Its default implementation supports existing custom indexes via `queryRay`: it suppresses later
visitor calls but cannot stop the custom query's internal work.

The original pipeline sorting adds entry/exit order to every index, preserving emission order on full ties.
Handles are positive and monotonically increasing; update preserves position, remove/reinsert appends,
and clear does not reuse handles. Exhaustion throws `IllegalStateException`. Snapshot rebuilds are
repeatable for the same surviving ordered entries, but changed geometry can change traversal order.
No rule depends on the payload's `Object.hashCode()`.

Static indexes copy the input list and retain immutable bounds and payload references. Returned lists
are immutable snapshots; changing a payload does not recompute stored bounds or prior results.
Mutable indexes, including lazy BVH **reads**, require external synchronization when shared.
Do not mutate the graph, index, occupancy or callback geometry during a complete tracing call.
Callbacks are subject to the same rule. Tracers retain supplied objects and take no automatic snapshot;
use an Ashspace frozen frame graph when needed. Static indexes can be read concurrently after safe
publication with thread-safe callbacks/payload access.

Use `anyRay`, `anyHit` or `anyVisibleHit` for existence queries when nearest-first callback order is
unnecessary. Built-in indexes stop subsequent traversal/filtering; the hash still gathers handles
before visiting them, and a dirty dynamic BVH still rebuilds. An exact provider already executing
continues to completion after emitting a match. Exceptions propagate and do not become a miss.

`TraceQueryBuffer3` overloads reuse candidate or shape-list capacity. Result lists remain immutable
and independent. Buffers clear payload references in `finally`, including when callbacks fail, but
retain capacity until `trimToSize()`. Do not share a buffer between concurrent queries or reenter it
from a callback; active reuse throws `IllegalStateException`. Sorting, result allocation, index work
and geometry-provider costs remain. This is not an allocation-free or fixed-memory API.

Bounds and vector components must be finite; zero-extent boxes and zero sweep motion are valid.
Null required references throw `NullPointerException`; non-finite or negative limits/radii and
non-finite bounds throw `IllegalArgumentException`. A valid empty query returns null or an empty list/emission.
Queries require representable coordinate differences, translated endpoints and squared distances;
extreme overflow/underflow is outside the accuracy guarantee. Finite input alone does not ensure
finite intermediate arithmetic. Large translations can lose local detail; no universal error bound
or arbitrary-world-size precision is promised. Nonzero ray/sweep components are never replaced with
a fixed geometric epsilon; only exact zero takes the parallel-axis path.

Hash cells use `floor(coordinate / cellSize)` and include both AABB endpoints. `cellSize` must be
finite and positive, cell indices must fit signed int, and an enumerated range may contain at most
`Integer.MAX_VALUE` cells. Rejected insert/update ranges preserve existing state. Int endpoints do
not wrap during iteration. This representation limit is not a practical memory budget: callers must
cap large ranges themselves. Voxel traversal has Ashgrid's int-coordinate limits and overflow checks.

## Supported API and migration

Supported usage includes public types/members under `nsk.nu.ashtrace.api` and the existing public
classes/constructors in `implementation.broadphase.staticindex` and `implementation.broadphase.dynamic`.
The official BVH constructor example is supported. `implementation.broadphase.internal`, private
helpers and private nested types are unsupported implementation details. No supported class or
signature is removed or moved in 2.0.0.

`NarrowPhase3` and `TraceHit3` keep their signatures and bounds-based meaning, including wall equality.
Code treating their output as an exact surface hit must adopt the candidate interpretation above.
Version `2.0.0` uses a major version for stricter numeric/range validation and the Ashspace 2.0
dependency contract. Nearest now respects `maxDistance` in BVH leaves; very small motion can return
previously missed contacts; hash decimal-boundary mapping uses division and rejects unsupported ranges.
The corrected Ashgrid dependency fixes negative-direction voxel intervals. Ashcore supplies corrected
ray normalization, while Ashspace rejects invalid frame/transform states. See each dependency's
migration notes before overriding the pinned versions.

Releases follow Semantic Versioning. There is no serialized wire format or cross-version
deterministic-stream compatibility promise.

## Glossary

- `broad-phase`: fast candidate filtering before expensive detailed tests.
- `frame-aware tracing`: tracing that first converts query data between coordinate frames.
- `NarrowPhase3`: the original boolean candidate-acceptance callback; it cannot return a surface distance.
- `RayIntersector3`: a synchronous geometry callback supplying full entry/exit intervals for one candidate.
- `proximity query`: sphere/point-nearest broad-phase lookup over AABBs.
- `sweep query`: broad-phase query for moving AABB over normalized motion time.
- `voxel traversal`: visiting integer grid cells crossed by a ray in deterministic order.
- `occlusion clipping`: limiting object trace distance by first occupied voxel hit.
- `traversal tie-break`: if multiple boundaries are hit at once, order follows traverser implementation rules.
- `tEnter/tExit`: distances along the normalized ray; the result type determines whether they describe bounds, voxels or provider-supplied geometry.
- `AABB`: axis-aligned bounding box.
- `OBB`: a box with its own orientation; its enclosing AABB can include empty space.

## License

Apache License 2.0. See [LICENSE](LICENSE).
