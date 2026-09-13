# Tracing article evidence — Ashtrace 2.0.0

The published prose is English to match the Blackframe Wikis and the parent task's explicit direction. This note is the authoring source map required by `Minecraft Plugins/DOCUMENTATION_DESIGN_TEMPLATE.md`; it does not add runtime requirements.

| Page / claim | Source | Independent test basis |
| --- | --- | --- |
| Frame conversion, world units, stable state | `src/main/java/nsk/nu/ashtrace/api/trace/pipeline/FrameBroadPhaseRayTracer3.java`; `FrameExactRayTracer3.java`; `FrameGridRayTracer3.java` | `unit/api/trace/CandidateSemanticsTest.java` translated/frozen rotated frame tests; `ExactRayTracerTest.java` world-ray provider test |
| Frame snapshots capture graph; payload and occupancy separately owned | Ashspace `src/main/java/nsk/nu/ashspace/api/frame/FrameGraph3.java`; `api/grid/FrameGridSpaceMapper3.java`; Ashtrace `README.md` ownership section | `integration/MappedGridTraceIntegrationTest.java`, `moving_and_frozen_mappers_affect_later_queries_as_documented`; `integration/OrientedBoxTraceIntegrationTest.java`, coherent pose updates |
| Exact full interval, clipping, surface flags | `api/trace/contracts/RayIntersector3.java`; `api/trace/model/RayIntersection3.java`; `ExactTraceHit3.java`; `pipeline/FrameExactRayTracer3.java` | `unit/api/trace/ExactRayTracerTest.java` origin/query limit, tangency, selection, invalid provider interval, delayed output tests |
| Sphere example and limits | `README.md`, `AshtraceExactQuickStart` | Exact interval behavior in `ExactRayTracerTest.java`; documentation examples will be compiled and executed by the Wiki verification task |
| Mapped grid translation 10, origin 1, cell size 2, hit X=13/distance 1.5 | `README.md`, `AshtraceMappedGridQuickStart`; `pipeline/FrameGridRayTracer3.java` | `integration/MappedGridTraceIntegrationTest.java`, scaled translated rotated grid and world-distance occlusion tests |
| DDA half-open limit, starting cell, ties, segment rejection | `FrameGridRayTracer3.java`; `README.md` grid contract; Ashgrid traversal contract | `unit/api/trace/CandidateSemanticsTest.java`; `integration/BlackframeGridContractIntegrationTest.java`; `MappedGridTraceIntegrationTest.java` segment endpoint test |
| Occlusion closed object limit and full shape interval clipping | `pipeline/FrameOccludedExactRayTracer3.java`; `FrameOccludedBroadPhaseRayTracer3.java` | `ExactRayTracerTest.java`, `wall_clips_real_exit_and_hides_surfaces_behind_loose_bounds`; `CandidateSemanticsTest.java`, wall contact and zero limit tests |
| Limited-query work, buffer ownership, lazy dynamic BVH reads | `FrameBroadPhaseRayTracer3.java`; `FrameExactRayTracer3.java`; `FrameOccludedBroadPhaseRayTracer3.java`; `TraceQueryBuffer3.java`; `implementation/broadphase/dynamic/DynamicBvhBroadPhase3.java` | `ExactRayTracerTest.java`, reusable buffer and reentry tests; `unit/api/broadphase/RayVisitContractTest.java` |

All Ashtrace paths above are relative to its repository, with test paths under `src/test/java/nsk/nu/ashtrace/`. Snapshot examples describe fixed target geometry. They do not imply the frame snapshot updates or copies indexed bounds, provider geometry, Minecraft blocks, or payload objects. `DynamicBvhBroadPhase3` has no public snapshot factory; its private lazy BVH is not a concurrency guarantee.

The three article diagrams use the same values as their adjacent Java examples: sphere center (5,0,0), radius 1, bounds X=3..7; mapped grid translation 10, origin 1 and cell size 2; occlusion wall entry at 5. Diagrams are longitudinal sections, not full 3D collision solvers.
