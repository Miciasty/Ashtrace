# Ashtrace 2.0.0 — broad-phase article evidence

Authoring notes for `wiki/content/broadphase.js`. Public prose is English, matching the Blackframe wiki shell. Source paths below are relative to the Ashtrace repository unless a sibling project is named. No runtime feature was added or inferred from a different library's menu.

## Article-to-source map

| Article | Claims | Primary code and corroborating tests |
| --- | --- | --- |
| `broad-phase` | Closed AABB overlap, one result per entry including equal payloads, ray distance units, segment delegation, acceptance preserving box intervals, per-index order, visitor completion and custom fallback | `api/broadphase/contracts/{BroadPhase3,RayQueryableBroadPhase3,RayCandidateVisitor3}.java`; all four indexes; `api/trace/pipeline/FrameBroadPhaseRayTracer3.java`; tests `unit/api/broadphase/{RayQueryableBroadPhase3ContractTest,RayVisitContractTest,BroadPhaseOrderingTest}.java` and `unit/api/trace/CandidateSemanticsTest.java`. |
| `indexes` | Constructor choices, static bounds, handles, lazy full BVH rebuild, hash-cell division, inclusive endpoints, int coordinates, range-size guard and mutation atomicity, thread confinement | `implementation/broadphase/staticindex/{LinearAabbBroadPhase3,BvhAabbBroadPhase3}.java`; `implementation/broadphase/dynamic/{DynamicBvhBroadPhase3,DynamicSpatialHashBroadPhase3}.java`; `api/broadphase/contracts/MutableRayBroadPhase3.java`; tests `unit/implementation/broadphase/{DynamicBvhBroadPhase3Test,DynamicSpatialHashBroadPhase3Test,SpatialHashLimitsTest}.java`; `unit/api/broadphase/BroadPhaseOrderingTest.java`. |
| `proximity-sweeps` | Sphere vs closed AABB, nearest squared result and inclusive distance limit, ties, fixed translational sweep, normalized interval and zero motion | `api/broadphase/contracts/{ProximityQueryableBroadPhase3,SweepQueryableBroadPhase3}.java`; `api/broadphase/model/{BroadPhaseNearestHit3,BroadPhaseSweepHit3}.java`; `implementation/broadphase/internal/BroadPhaseMath3.java`; tests `unit/implementation/broadphase/LinearAabbBroadPhase3Test.java`, `unit/api/broadphase/BroadPhaseAgreementTest.java`. |
| `query-contracts` | Units, immutable results, boundaries, exceptions, exact ordering, limits vs work, exclusive buffer and reentry cleanup | All `api/trace/model/*.java`; `api/trace/contracts/RayIntersector3.java`; all `api/trace/pipeline/*.java`; tests `unit/api/trace/{CandidateSemanticsTest,ExactRayTracerTest}.java`, `unit/api/broadphase/RayVisitContractTest.java`, `unit/api/model/TraceModelsApiTest.java`; README “Operation costs” and “What a hit, nearest result and visible result mean”. |
| `performance` | Time, auxiliary space, retained snapshots, sorting, hash enclosing-AABB enumeration, callback cost, no allocation-free guarantee | README “Operation costs”, reconciled with all four indexes and trace pipelines; manual types `src/test/java/nsk/nu/ashtrace/benchmark/manual/{BroadPhaseBenchmarkMain,TraceWorkloadBenchmarkMain}.java`. No measured latency claim. |
| `api-reference` | Public-type/package inventory and key method families | Every `src/main/java/nsk/nu/ashtrace/**/*.java` and nested public types. Includes publicly visible internal `BroadPhaseMath3` and `Interval`, distinguished from application contracts. Dependency types checked against imports and sibling Ashcore/Ashspace/Ashgrid source. |

Paths beginning `api/` or `implementation/` expand under `src/main/java/nsk/nu/ashtrace/`. Paths beginning `unit/` expand under `src/test/java/nsk/nu/ashtrace/`.

## Terms

| Term | Meaning |
| --- | --- |
| AABB / bounds | Closed world-space axis-aligned box. Can enclose detailed geometry. |
| Candidate | An indexed entry whose AABB meets the query. Does not establish the enclosed shape's surface. |
| Acceptance callback | Nested `FrameBroadPhaseRayTracer3.NarrowPhase3`; boolean acceptance without replacing parameters. |
| Geometry provider / intersector | `RayIntersector3`; synchronously supplies full signed shape intervals. |
| Ray parameter | World distance along the normalized ray. |
| Sweep time | Fraction of the supplied translation, in `[0,1]`. |
| Nearest distance | Distance to closed bounds. Result is squared; query limit is not. |
| Snapshot | Saved index/frame/result state as documented per type. Does not clone payloads or make dynamic BVH thread-safe. |

## Runnable-example derivations

- `BoundsCandidatesExample.java`: linear entries X=[4,5], [1,2]. Query X=[2,4] touches both and emits input order. A normalized +X ray from X=0 emits [4,5], [1,2]. Frame pipeline sorts and returns `near` at X=1.
- `BoundsVisitorExample.java`: the only candidate requests a stop. `visitRay` returns false even on the last candidate; `anyRay` returns true.
- `DynamicIndexExample.java`: moving X=[2,3] to [8,9] removes the ray-range [0,3] intersection. Removal empties the index. Updating a missing handle with valid bounds returns false. Clear preserves handle allocation history.
- `ProximityExample.java`: a sphere centered at X=0 with radius 3 touches X=[3,4]. From X=1 the distance is 2, squared distance 4. Inside returns zero; a limit of 2 from X=0 returns null.
- `MovingBoxExample.java`: X=[0,1], delta=(4,0,0), target X=[3,4], Y/Z=[0,1]. Entry `(3-1)/4=0.5`, exit `(4-0)/4=1`; travel to contact `|delta| * 0.5 = 2`. SVG shows the corresponding XY cross sections at 0, 0.5, 1 with a textual alternative.
- `QueryBufferExample.java`: saved one-hit list is independent of reuse for an empty accepted set; saved entry remains 2.0. Trimming happens while idle.

Every Java block is a full class with `main`; the following output block gives expected stdout. The shared wiki checker compiles and runs these examples with the project classpath.

## Editorial and evidence decisions

- Article section counts: 4, 4, 3, 5, 4, 5. API map groups actual packages by reader purpose without inventing a Java hierarchy.
- One educational graphic demonstrates normalized sweep time. It has three exact positions, readable labels and a text equivalent; no generic node graphs.
- Core distinctions remain explicit: boolean acceptance retains box distances; nearest measures bounds; sweep time differs from distance; exact result limits do not limit providers; occluded broad-phase visible limits do not limit acceptance calls; dynamic BVH snapshots still need external synchronization.
- Arithmetic uses doubles. The public article asks callers to keep derived values representable without claiming arbitrary-magnitude robustness or inventing overflow guarantees.
