# Ashtrace

Ashtrace finds ray, overlap, proximity and moving-box candidates in indexed AABBs and converts frame-local traces to a world voxel grid.

This checkout builds **2.0.0-SNAPSHOT**, an unpublished development version. It uses the corrected
Ashcore **1.1.0-SNAPSHOT**, Ashgrid **1.3.0-SNAPSHOT** (including GRID-001), and Ashspace **2.0.0-SNAPSHOT**.
These development dependencies must be provisioned before building; their local availability does not
establish availability in a remote Maven repository. See [verification and migration](VERIFICATION.md).

> [!NOTE]
> Ashtrace is the tracing layer above Ashspace and Ashgrid:
> - converts source-frame rays/segments to world space,
> - filters object candidates with broad-phase AABB queries,
> - optionally clips AABB candidate intervals at the first occupied voxel.
> Mesh extraction belongs to Ashmesh. Navigation/pathfinding belongs to Ashnav.

## 1. Purpose

Ashtrace gives you reusable deterministic tracing primitives so plugins and engines can query space faster without mixing gameplay/domain rules into low-level code.

## 2. Problem

Spatial projects repeatedly rebuild the same low-level tracing pipeline:
- convert rays from local tool/object coordinates to world coordinates,
- find candidate objects via broad-phase AABB queries,
- support moving objects with mutable indexes,
- run proximity and sweep candidate queries,
- traverse voxels in deterministic order,
- stop object hits at the first voxel occluder when needed.

Without a shared layer, every project re-implements this differently and introduces inconsistent behavior.

## 3. When to use

Use Ashtrace when:
- you need deterministic broad-phase candidate filtering over AABBs,
- you need frame-aware ray queries that combine Ashspace transforms with Ashgrid traversal,
- you need reusable low-level tracing blocks for LOS, hit-scan, and candidate prefiltering.

Do not use Ashtrace when:
- you need rendering or mesh extraction pipelines,
- you need full navigation/pathfinding logic,
- you need project-specific hit resolution, damage, or gameplay rules.

## 4. Simple example (Minecraft plugin example)

You have a mining drill mounted on a moving machine:
1. Drill ray is defined in the drill local frame.
2. The machine moves/rotates in world space.
3. You convert the local ray to world space using frame graph rules.
4. You trace voxels in deterministic order and return the first solid block hit.

Ashtrace centralizes this pipeline so every tool uses the same deterministic trace behavior.

## 5. How it works

1. `FrameGridRayTracer3` converts a source-frame ray to world coordinates through Ashspace.
2. Voxel tracing supports first-hit and all-hit queries for both rays and finite segments.
3. `FrameBroadPhaseRayTracer3` performs frame-aware broad-phase candidate tracing with a boolean acceptance callback.
4. `FrameOccludedBroadPhaseRayTracer3` combines object broad-phase tracing with first-voxel occlusion clipping.
5. `RayQueryableBroadPhase3`, `ProximityQueryableBroadPhase3`, and `SweepQueryableBroadPhase3` define deterministic query contracts implemented by `LinearAabbBroadPhase3`, `BvhAabbBroadPhase3`, `DynamicSpatialHashBroadPhase3`, and `DynamicBvhBroadPhase3`.

## 6. Big-O for operations

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

Static indexes retain `O(n)` entries/tree data. A dynamic BVH retains live entries and its last snapshot;
removing entries does not release the old snapshot until another query rebuilds it. The hash retains
entries, `B` buckets and their cell references; list/map capacities can reflect earlier peaks until
entries/buckets are removed. No query path is claimed allocation-free.

Many overlapping boxes can make `C` nearly `n`. Making a three-dimensional hash query twice as wide
on every axis visits about eight times as many cells at fixed cell size. These are cost models,
not measured latency guarantees. The manual benchmark is workload-specific; no optimization claim
is attached to these corrections.

## 7. Core terms

- `broad-phase`: fast candidate filtering before expensive detailed tests.
- `frame-aware tracing`: tracing that first converts query data between coordinate frames.
- `narrow-phase`: the existing boolean candidate-acceptance callback; it cannot return a surface distance.
- `proximity query`: sphere/point-nearest broad-phase lookup over AABBs.
- `sweep query`: broad-phase query for moving AABB over normalized motion time.
- `voxel traversal`: visiting integer grid cells crossed by a ray in deterministic order.
- `occlusion clipping`: limiting object trace distance by first occupied voxel hit.
- `traversal tie-break`: if multiple boundaries are hit at once, order follows traverser implementation rules.
- `tEnter/tExit`: clipped entry/exit parameters of the indexed AABB or visited voxel, depending on result type.
- `AABB`: axis-aligned bounding box.

## 8. Quick-start

Requires Java 21+ and Maven. Both complete programs below are compiled with `--release 21` and run
against the packaged JARs during `mvn -B clean verify`. The voxel example also checks dependency SPI loading.
Ashtrace itself has no SPI providers.

Maven:

```xml
<dependency>
  <groupId>dev.nasaka.blackframe</groupId>
  <artifactId>ashtrace</artifactId>
  <version>2.0.0-SNAPSHOT</version>
</dependency>
```

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

## 9. Repository layout

Source tree is grouped by feature domain with matching Java package namespaces:
- `src/main/java/.../api/broadphase/contracts` - broad-phase query contracts.
- `src/main/java/.../api/broadphase/model` - broad-phase value records.
- `src/main/java/.../api/trace/pipeline` - frame-aware tracing pipelines.
- `src/main/java/.../api/trace/model` - trace result records.
- `src/main/java/.../implementation/broadphase/staticindex` - static broad-phase implementations.
- `src/main/java/.../implementation/broadphase/dynamic` - mutable broad-phase implementations.
- `src/main/java/.../implementation/broadphase/internal` - internal math helpers.
- `src/test/java/.../unit`, `.../integration`, `.../smoke`, `.../benchmark/manual` - test and benchmark domains.

## 10. What a hit, nearest result and visible result mean

`TraceHit3` preserves the accepted AABB's closed `[tEnter,tExit]` interval, clipped to the query.
`worldPoint` (the hit point) is `worldRay.at(tEnter)`. Starting inside a box gives entry zero and the
ray origin as the point. `firstHit` sorts by AABB entry, then exit, then broad-phase emission order.
A boolean `NarrowPhase3` can reject candidates but cannot replace these distances or their ordering.

For example, A has bounds from X=1 to X=10 and a surface at X=9. B has bounds from X=3 to X=5
and a surface at X=4. A ray from X=0 accepts both: `firstHit` returns A at X=1 even though B's surface
is nearer. For exact nearest-surface selection, collect candidates, calculate each actual surface
parameter inside its supplied interval and choose the minimum yourself. This API supplies no exact-hit result type.
`nearest` similarly measures distance to the closed AABB, zero inside it, rather than an enclosed surface.
Sweeps describe a translating AABB's contact interval over normalized time `[0,1]`; time is not distance
and no collision response or rotation during motion is computed.

Voxel occlusion uses world unit cells rooted at `(0,0,0)`. Cell `(-1,0,0)` includes X=-0.2.
Neither the spatial hash's `cellSize` nor an Ashspace grid mapper changes this grid. Convert other
units/origins explicitly before using a compatible tracer; rigid frame conversion has no scale or shear.
World conventions follow Ashspace: right-handed, Y up. Rays are normalized; `t` and `tMax` measure
world distance, including after rigid rotation/translation. Segment limits use their length.

With the tested `dda` provider, voxels cover `[0,tMax)` and exact ties step X, then Y, then Z,
including zero-length intermediate visits. DDA does not visit every cell touched at an edge/corner;
it is not supercover. Another traverser can change the occluder selected at a tie.
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

## 11. Ordering, ownership and numeric limits

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

Pipeline sorting adds entry/exit order to every index, preserving emission order on full ties.
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

## 12. Supported API and migration

Supported usage includes public types/members under `nsk.nu.ashtrace.api` and the existing public
classes/constructors in `implementation.broadphase.staticindex` and `implementation.broadphase.dynamic`.
The official BVH constructor example is supported. `implementation.broadphase.internal`, private
helpers and private nested types are unsupported implementation details. No supported class or
signature is intentionally removed or moved by these corrections.

`NarrowPhase3` and `TraceHit3` keep their signatures and bounds-based meaning, including wall equality.
Code treating their output as an exact surface hit must adopt the candidate interpretation above.
`2.0.0-SNAPSHOT` reserves a major version for stricter numeric/range validation and the Ashspace 2.0
dependency contract. Nearest now respects `maxDistance` in BVH leaves; very small motion can return
previously missed contacts; hash decimal-boundary mapping uses division and rejects unsupported ranges.
The corrected Ashgrid dependency fixes negative-direction voxel intervals. Ashcore supplies corrected
ray normalization, while Ashspace rejects invalid frame/transform states. See each dependency's
migration notes before overriding the pinned versions.

The release must retain these API/ordering contracts or explicitly version a change. There is no
serialized wire format or cross-version deterministic-stream compatibility promise. Existing 1.0.0
artifacts must not be overwritten. [VERIFICATION.md](VERIFICATION.md) records the actual JAR identities,
checks, publishing routes and remaining release prerequisites.

## License

Apache-2.0 Copyright 2025 Mateusz Aftanas
