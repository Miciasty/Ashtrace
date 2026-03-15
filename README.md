# Ashtrace

Low-level deterministic Java library for broad-phase acceleration helpers and frame-aware voxel tracing pipelines.

> [!NOTE]
> Ashtrace builds on Ashcore, Ashgrid, and Ashspace.
> - Mesh extraction belongs to Ashmesh.
> - Navigation and pathfinding belong to Ashnav.

## 1. Purpose

Ashtrace provides reusable tracing primitives so plugins and engines can query space faster without mixing high-level game logic into low-level libraries.

## 2. Problem

Spatial projects often need the same glue logic:
- broad-phase candidate filtering by bounds,
- mutable acceleration for moving objects,
- proximity and nearest-candidate queries over bounds,
- swept AABB candidate filtering for moving volumes,
- deterministic ray traversal through voxel spaces,
- frame-aware conversion from local tool/object space into world space before tracing,
- object tracing clipped by voxel occlusion.

Without a shared layer, every project re-implements this differently and introduces inconsistent behavior.

## 3. When to use

Use Ashtrace when:
- you need deterministic broad-phase candidate filtering,
- you need frame-aware ray queries that combine Ashspace transforms with Ashgrid traversal,
- you want reusable low-level tracing blocks for higher systems.

Do not use Ashtrace when:
- you need rendering or mesh extraction pipelines,
- you need full navigation/pathfinding logic,
- you need project-specific gameplay rules.

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
3. `FrameBroadPhaseRayTracer3` performs frame-aware broad-phase candidate tracing with narrow-phase acceptance.
4. `FrameOccludedBroadPhaseRayTracer3` combines object broad-phase tracing with first-voxel occlusion clipping.
5. `RayQueryableBroadPhase3`, `ProximityQueryableBroadPhase3`, and `SweepQueryableBroadPhase3` define deterministic query contracts implemented by `LinearAabbBroadPhase3`, `BvhAabbBroadPhase3`, `DynamicSpatialHashBroadPhase3`, and `DynamicBvhBroadPhase3`.

## 6. Big-O for operations

Definitions:
- `n`: number of indexed broad-phase entries.
- `k`: number of visited voxels along a ray.

| Operation | Complexity | Notes |
| --- | --- | --- |
| `LinearAabbBroadPhase3.query` | `O(n)` | Linear scan over registered AABBs. |
| `BvhAabbBroadPhase3` build | `O(n log^2 n)` | Median split with per-level sort. |
| `BvhAabbBroadPhase3.query` | average `O(log n + c)` | `c` is number of reported candidates. |
| `DynamicSpatialHashBroadPhase3.insert/update/remove` | average `O(k)` | `k` is number of covered hash cells. |
| `DynamicSpatialHashBroadPhase3.query` | average `O(c + h)` | `h` is visited hash cells, `c` is candidates. |
| `DynamicBvhBroadPhase3.insert/update/remove` | `O(1)` mutation, deferred rebuild | Mutations mark snapshot dirty; rebuild is delayed until a query. |
| `DynamicBvhBroadPhase3.first query after mutation` | `O(n log^2 n)` | Lazy BVH snapshot rebuild cost; later read-only queries use BVH query complexity. |
| `*.querySphere` | linear/BVH/hash-accelerated variant dependent | Sphere-vs-AABB candidate filtering. |
| `*.nearest` | variant dependent (`O(n)` worst case) | Returns closest AABB candidate within max distance. |
| `*.querySweptAabb` | variant dependent | Swept AABB candidate intervals in normalized time. |
| `FrameGridRayTracer3.firstHit` | `O(k)` worst case | Stops early on first occupied voxel. |
| `FrameGridRayTracer3.allHits` | `O(k)` | Walks visited cells and filters by occupancy. |
| `FrameGridRayTracer3.firstSegmentHit` | `O(k)` worst case | Same traversal with finite `tMax = segment length`. |
| `FrameBroadPhaseRayTracer3.firstHit` | average `O(q + m)` | `q` broad-phase query work, `m` narrow-phase checks. |
| `FrameOccludedBroadPhaseRayTracer3.firstVisibleHit` | average `O(q + m + k)` | Broad-phase object trace clipped by first voxel occluder. |

## 7. Core terms

- `broad-phase`: fast candidate filtering before expensive detailed tests.
- `frame-aware tracing`: tracing that first converts query data between coordinate frames.
- `narrow-phase`: detailed per-candidate acceptance step after broad-phase filtering.
- `proximity query`: sphere/point-nearest broad-phase lookup over AABBs.
- `sweep query`: broad-phase query for moving AABB over normalized motion time.
- `voxel traversal`: visiting integer grid cells crossed by a ray in deterministic order.
- `occlusion clipping`: limiting object trace distance by first occupied voxel hit.
- `traversal tie-break`: if multiple boundaries are hit at once, order follows traverser implementation rules.
- `tEnter/tExit`: ray interval where the ray is inside a visited cell.
- `AABB`: axis-aligned bounding box.

## 8. Quick-start

Requires Java 21+.

Maven:

```xml
<dependency>
  <groupId>dev.nasaka.blackframe</groupId>
  <artifactId>ashtrace</artifactId>
  <version>1.0.0</version>
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

Minimal broad-phase trace pipeline:

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

## License

Apache-2.0 Copyright 2025 Mateusz Aftanas
