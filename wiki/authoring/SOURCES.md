# Ashtrace WIKI source map

Documented version: **2.0.0**, as declared in `pom.xml` and the local source. Baseline before WIKI changes: commit `246bfbf` on `docs/github-pages-wiki-20260913` (snapshot of `dce56c0`).

The user supplied the Blackframe documentation language instructions and WIKI template under `Minecraft Plugins/DOCUMENTATION_DESIGN_TEMPLATE`. The wording file contains an older statement that the template directory is empty; the existing files and `WIKI_DESIGN_TEMPLATE.md` supply the actual current visual template. Public prose follows its English language convention. The user's requirement for informative visualizations takes precedence over adding a figure to every mechanism.

The navigation follows the local Ashcore, Ashspace, Ashgrid, and Ashnav copies: Getting started, subject groups, and Reference; shared header, local search, article outline, and mobile drawer. Public palette-reference and fictional Shelter pages are omitted.

| Article / fact | Primary local evidence |
| --- | --- |
| Overview, version, Java requirement, transitive dependencies | `pom.xml`, `README.md`, public classes under `src/main/java` |
| Mounted tool and world cell distance | `FrameGridRayTracer3`, `GridRayHit3`, `FrameGridRayTracer3ApiTest`, `FrameTraceBroadPhaseIntegrationTest` |
| Full sphere intervals, clipping, surface flags | `FrameExactRayTracer3`, `RayIntersector3`, `ExactTraceHit3`, `ExactRayTracerTest` |
| Oriented box conversion and entry/exit | `OrientedBoxTraceIntegrationTest`, Ashspace `SpaceConverter3`, Ashcore `CollisionTests` |
| Grid coordinates, zero limit and end-exclusive DDA | `FrameGridRayTracer3`, `MappedGridTraceIntegrationTest`, `BlackframeGridContractIntegrationTest` |
| Occlusion, closed object intervals at the wall | `FrameOccludedExactRayTracer3`, `FrameOccludedBroadPhaseRayTracer3`, `CandidateSemanticsTest` |
| Service discovery and explicit class loader | Ashcore `api/spi/ServiceRegistry.java`; Ashgrid `META-INF/services/nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser` |
| Provider resources when shading | [Apache Maven Shade: ServicesResourceTransformer](https://maven.apache.org/plugins/maven-shade-plugin/examples/resource-transformers.html#concatenating-service-entries-with-the-servicesresourcetransformer) |
| Workflow and public file allowlist | Local peer WIKI `pages.yml` and `build.mjs`; Ashtrace `.github/workflows/maven.yml` |

See `tracing-sources.md` and `broadphase-sources.md` for detailed maps. `check-examples.mjs` validates the displayed programs against the checkout rather than assuming that a code block is correct because it came from another document.

Minecraft scenarios describe adapter responsibilities. They do not claim a built-in Bukkit adapter, supported server-version matrix, automatic chunk loading, game commands, damage model, or collision response.

## Editorial vocabulary

| Term | Meaning used in the WIKI | Identifiers / excluded ambiguity |
| --- | --- | --- |
| AABB / bounds | Closed axis-aligned box enclosing an indexed object. | `AxisAlignedBox`, `AabbEntry3`; not the enclosed surface. |
| Candidate | An entry that passed the bounds query. | `BroadPhaseRayHit3`, `TraceHit3`; not proof of an exact surface hit. |
| Shape interval | A connected interval supplied by the geometry provider, possibly clipped by the tracer. | `RayIntersection3`, `ExactTraceHit3`; not necessarily one interval per object. |
| Surface endpoint | An original provider endpoint retained after clipping. | `enterSurface`, `exitSurface`; not a query boundary created by clipping. |
| World | The frame graph's root coordinate space. | Does not implicitly encode a Minecraft world identifier. |
| Source frame | The frame in which the supplied ray or segment is expressed. | `sourceFrame`, `FrameId`; distinct from the grid frame. |
| Occupied cell / voxel | An integer cell accepted by the occupancy predicate. | `GridRayHit3`; not automatic server block access. |
| Ray distance | Distance along the normalized world ray, from its origin. | `tEnter`, `tExit`, `tMax` on ray results; world units. |
| Sweep time | Fraction of the supplied box displacement. | `BroadPhaseSweepHit3`; normalized `[0,1]`, not seconds or distance. |
| Snapshot | Frozen state of the specified frame graph or mapper. | Does not deep-copy payloads, occupancy, or geometry. |
| Query buffer | Exclusive reusable temporary collection storage. | `TraceQueryBuffer3`; not thread-safe and not allocation-free. |
| Visible interval | An object interval clipped at the first occupied voxel. | Its geometric meaning follows the selected AABB or exact tracer. |

Web lookup of the supplied GitHub repositories returned older indexed README content. The documented 2.0.0 API therefore follows the local versioned source, not that cached view. Peer library links use the user-supplied GitHub destinations. The Maven Central destination follows the current local README and coordinates; public deployment is separately verified only after publication.
