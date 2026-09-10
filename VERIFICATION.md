# Ashtrace verification and migration

Latest work: [oriented-box integration, TRACE-012](#2026-09-10--oriented-box-integration-trace-012).
The first record below describes the earlier correction checkpoint; its artifact hashes are historical.

## 2026-09-10 — Blackframe contract revision 2.0 corrections

Coordinates: `dev.nasaka.blackframe:ashtrace:2.0.0-SNAPSHOT`, an unpublished development version.
Work is confined to Ashtrace. Branch: `fix/ashtrace-contract-v2-20260910`; checkpoint: `70d3513`.
The checkpoint saved the previously untracked ISSUES.md; implementation matched `aaf1567` / `5c516fc`.
The correction commit is the commit adding this record. No tag, push, deploy or publication was performed.

### Decisions and compatibility

- TRACE-001: keep `NarrowPhase3` and `TraceHit3` as boolean acceptance of an indexed AABB interval.
  Entry, exit and `worldPoint` remain bounds measurements. No exact surface API was added.
  The two-surface regression accepts A's earlier loose bounds although B's surface is nearer.
- TRACE-002: keep closed object contact at voxel entry, including equality and origin contact.
  DDA voxel traversal is half-open at the final limit; tMax=0 visits no cells, while object bounds
  at the origin may match. Zero-length segments remain invalid. Tracers require unit world voxels
  and stable frames/index/occupancy/callback geometry throughout the combined call.
  Corrected negative traversal comes from Ashgrid 1.3.0-SNAPSHOT, not a DDA workaround in Ashtrace.
- TRACE-003: preserve each index's order; compare candidate sets and interval/nearest-distance
  values across implementations. Nearest tie winners and AABB/sphere emission orders need not match.
  Tests replay mutations, force BVH rebuilds, compare identical bounds and show insertion-sensitive ties.
- TRACE-004: account for candidate sorting, temporary lists, callback work, raw hash references,
  handle deduplication/sorting, bucket removal scans and deferred BVH rebuilds. No performance
  optimization or benchmark speedup is claimed. `visibleHits` still filters all candidates before truncation.
- TRACE-005: support public `api`, `implementation.broadphase.staticindex` and
  `implementation.broadphase.dynamic` usage; explicitly exclude `implementation.broadphase.internal`.
  A major development version reserves stricter input handling and the Ashspace 2.0 dependency contract.
- TRACE-006: local build, packaging and workflow corrections are verified below. Remote CI and
  publication remain unverified, and hosted builds need access to the development dependency artifacts.
  This record is not a release-readiness or publication claim.

Local correctness fixes also include BVH leaf-level enforcement of `maxDistance`, exact-zero
parallel-axis handling for ray/sweep slabs, division instead of reciprocal multiplication in ray
slabs, finite-bound validation, checked hash indices/range products, non-wrapping int-endpoint
iteration, validation before hash insert/update mutation, and rejection of handle exhaustion.
The hash maps cells with `floor(coordinate / cellSize)`; unsupported ranges now fail instead of
saturating indices or corrupting an update. Finite intermediate arithmetic and practical range
budgets remain caller requirements, as described in README.

No supported public type/member was removed or moved. `javap -public` compared **19 public types
and 138 declarations**, including the acceptance interface and concrete index constructors, with
the existing Ashtrace 1.0.0 JAR; no declarations were removed or changed. This checks signatures,
not universal behavioral compatibility. Old code relying on invalid input, BVH out-of-limit hits,
epsilon-discarded motion, legacy hash rounding or old dependency behavior must migrate.
The comparison JAR SHA-256 is `57c1ed064e184bd2f0e6e748378e924a919b6199afd261d7850cb191a6547b30`.

### Environment and executed checks

Windows 11 amd64, UTF-8, locale pl_PL; Eclipse Adoptium **JDK 21.0.12.1+1**;
Apache Maven **3.9.16** (`2bdd9fddda4b155ebf8000e807eb73fd829a51d5`); actionlint **1.7.7**.
Compiler 3.13.0 targets `release=21`; Surefire/Failsafe 3.2.5 require tests.
Javadoc 3.7.0 uses `all,-missing` and `failOnError=true`.
Core lifecycle, source/Javadoc packaging, help and dependency plugins are pinned.

| Check | Actual result |
| --- | --- |
| Original tests / original dependencies | 43 passed. |
| First added contract regressions on original code | 11 tests: 2 failures (BVH nearest), 1 error (negative-direction occlusion), 8 passed. |
| Small-motion regressions below the actual geometry epsilon | 2 tests, both failed before the slab correction. The first trial at 1e-10 did not reproduce this issue; 1e-14 did. |
| First corrected integration run | 54 passed with corrected dependency artifacts. |
| First complete packaging gate | 59 tests + 2 packaged-artifact tests passed. |
| Final `clean verify dependency:tree help:effective-pom` | **62 tests + 2 packaged-artifact tests passed; 0 failures, 0 errors, 0 skipped.** |
| Both complete README Java programs | Compiled with release 21 against main/dependency JARs only, then executed in separate JVMs; expected target/voxel results passed. |
| Dependency SPI | README voxel program resolves `dda` from the packaged Ashgrid JAR. Ashtrace itself has no providers or required service registration. |
| Public signature comparison | 19 types / 138 declarations, no removals. |
| Both GitHub workflow files | actionlint passed, optional ShellCheck/Pyflakes integrations disabled. |
| Whitespace check | `git diff --check` passed. |

Final Maven gate finished **2026-09-10 07:56:36 +02:00**, exit 0. The tests cover candidate versus
surface order, callback acceptance/rejection, equal wall boundaries, start inside an occluder,
empty voxel intervals, excluded voxel endpoints, negative directions, translation/normalization,
rotation with frozen frame state, all five query families, mutation/rebuild order, tiny/subnormal
ray or sweep components, empty indexes, non-finite bounds, int endpoints and rejected hash updates.
The old int-overflow loop was inspected but not deliberately run to nontermination.
This is the listed contract audit, not an exhaustive proof of all geometry or floating-point inputs.
Java 25 is configured in CI but was not run locally; no cross-environment bitwise result claim is made.

The default PATH exposed Java 8 and no Maven. Existing tool installations were invoked explicitly
with approved execution access. JDK/actionlint were read from Ashspace's ignored tool directory;
no tool or file there was changed. Maven used an isolated writable repository and settings under
Ashtrace `.verification/`; an existing user Maven cache served as a read-only artifact source.
No user's Maven settings/credentials were loaded by the local verification script.

Local command equivalents (the wrapper selects the JDK/Maven and isolated settings/repository):

```powershell
& ./.verification/install-dependencies.ps1
& ./.verification/maven.ps1 clean verify dependency:tree help:effective-pom '-Doutput=.verification/effective-pom.xml'
& ./.verification/check-api.ps1
& ../Ashspace/.verification/actionlint/actionlint.exe -shellcheck= -pyflakes= .github/workflows/maven.yml .github/workflows/publish.yml
git -c safe.directory=G:/Github/Blackframe/Ashtrace diff --check
```

The standard build after provisioning dependencies is `mvn -B clean verify`; it performs no upload.
Logs/scripts, effective POM and the isolated cache are ignored local evidence under `.verification/`.
Test reports live in `target/surefire-reports` and `target/failsafe-reports`. Complete verification
tests are committed; the local machine-specific wrapper is not a portable build requirement.

### Dependency identity

Corrected JARs were copied from already-built sibling artifacts with `maven-install-plugin:3.1.3:install-file`
and their matching POMs into **Ashtrace's isolated repository only**. No sibling was built or edited.
The sibling checkouts were clean at inspection. Commits identify source context; SHA-256 identifies
the actual binary executed. Local snapshots are not presumed published or permanently immutable.

| Resolved dependency | Scope | Source checkout | Binary SHA-256 |
| --- | --- | --- | --- |
| Ashcore 1.1.0-SNAPSHOT | compile | `e519440` | `9b7758dee82a8fa7338afcc7b7bf22fe02682aaff670c10dd4971be9390c845f` |
| Ashgrid 1.3.0-SNAPSHOT | compile | `8199f9b` | `b4a8d0ac87ebe34132f1f86d8870e8f32f5f270b95d263a0fc21e345a6e71285` |
| Ashspace 2.0.0-SNAPSHOT | compile | `a4c813b` | `232b5524c201d881cfb9287906ca2eea74c2f3ee372dc0202ea1c568f46a019d` |
| JUnit Jupiter 5.10.2 and its dependencies | test | Maven artifacts | Not included in the main JAR. |

Direct dependency versions override older transitive Ashcore/Ashgrid references; `dependency:tree`
and the effective POM confirm the versions above. There are no other production dependencies.
Baseline tests used Ashcore 1.0.1 (`0ea3a990d28a01aac97c574497c21be2f2ced5b90eaa03637c8499cbf1d62d0b`),
Ashgrid 1.2.0 (`b0ea0b634f77504747142b1e3475676c3ab40fea92c5f76075d47bb2d5d24ef7`) and
Ashspace 1.0.0 (`deeeeef9808052140508f26faf0a2b3a8c1ac16ba9bab9ba9428809c4190a03f`).
Using the old GRID-001-affected JAR is not a supported substitute for the corrected traversal test.

### Verified output artifacts

The main JAR contains classes compiled for Java 21 (class version 65), matching Maven coordinates,
`META-INF/LICENSE` and `META-INF/NOTICE`, and no JUnit classes or Ashtrace SPI registration.
The separate source and Javadoc JARs contain source/API pages. Tests and CI identify exact filenames.

| File under target | SHA-256 |
| --- | --- |
| ashtrace-2.0.0-SNAPSHOT.jar | `da6832888df3bb1485955ccf04df07c88cc83f1203ae5d0e3b6c203b8373de17` |
| ashtrace-2.0.0-SNAPSHOT-sources.jar | `0e70b64f4e880db82f25364e0883855e53c1b1cf9b2e3aa709b903b42fc625d9` |
| ashtrace-2.0.0-SNAPSHOT-javadoc.jar | `0ce86889051c2bd5a439eb9b3b5c05c973349fffad2a4b6bccc19f58571f629d` |

A fixed output timestamp is configured; reproducibility across toolchains/platforms was not measured.

### CI and publication prerequisites

`git ls-remote --symref origin HEAD` confirmed **main**, remote HEAD
`aaf15679af89b6624595e34ffabbe67415d18dda`. CI now covers all pushes and pull requests, with
Java 21/25, `clean verify`, and exact main/sources/Javadoc upload paths. These workflow files passed
syntax checks locally but were not pushed or executed on GitHub. **A fresh hosted runner cannot
assume access to the three unpublished dependency snapshots.** Before running hosted CI, provision
the identified artifacts through an approved development repository or replace them with verified
published release coordinates and rerun integration. The current POM does not invent a snapshot endpoint.

- **GitHub Packages:** configured by distributionManagement and publish.yml. The workflow requires a
  matching non-snapshot `v<version>` tag and non-snapshot direct Blackframe dependency versions,
  then clean verification before deploy. Credentials, destination availability and actual upload
  are unverified. Do not republish an existing version.
- **GitHub Release:** a published release can trigger the Packages workflow. Automatic release-asset
  attachment is intentionally not configured; CI artifact upload is not a GitHub Release publication.
- **Maven Central:** the existing `central` profile supplies signing and Central publishing. It was
  not activated. Its credential/signing/publication route remains unverified; the GitHub Packages
  workflow does not publish to Central. Historical publication cannot be inferred from this profile.
- **Next release:** make lower-layer release artifacts available, select unused final coordinates,
  rerun clean verification and hosted CI, then record tag/commit, destination, date, workflow URL
  and artifact-availability evidence. No deployment is needed to validate these local corrections.

## 2026-09-10 — Mapped grids, shape intervals and query reuse

Branch: `feat/ashtrace-complete-tracing-20260910`; checkpoint `5356ab5`, created before changes
from the clean corrected commit `3be89a0`. Version remains the unpublished `2.0.0-SNAPSHOT`.
All edits and build output remain in Ashtrace; siblings supplied read-only source, tests, tools and artifacts.

### Scope and compatibility

- TRACE-007: the `forGrid` factories attach voxel traversal and occlusion to an Ashspace mapper.
  Grid frame, origin and uniform cell size control cell coordinates; distances and hit points remain
  in world units. Original constructors keep the unit-world grid and their public signatures.
  Live and frozen mapper graphs are supported, with stable state required during each complete query.
- TRACE-008: `RayIntersector3` emits full, unclipped `RayIntersection3` intervals. New exact tracers
  select first/last/all/any intervals and return both world endpoints, clipping flags and interval length.
  Multiple intervals retain cavities/disconnected parts. Exact means provider-supplied geometry;
  the provider still owns the shape solver, enclosing-bound correctness and numeric accuracy.
  Primitive solvers and projectile physics were not moved into Ashtrace. The old boolean callback
  and result keep their bounds meaning; `TraceHit3` only gains endpoint convenience methods.
- TRACE-009: stoppable visitors in all four indexes, existence queries and optional reusable list
  capacity. Original ordered queries retain ordering, including full ties and acceptance order.
  Candidate wrapper objects and extra intermediate list copies were removed. Exact first/last
  select without collecting/sorting all intervals; exact all-hit queries still sort all emitted hits.
- TRACE-010: portable local provisioning/verification is ready; hosted CI and release availability
  remain dependent on making the corrected lower-layer artifacts available to the runner.

`javap -public` against the corrected baseline JAR preserved all **19 existing public types and
138 declarations**. A client and custom index implementation compiled against that baseline ran
with the new JAR without recompilation. A second client used the new default visitor and exact
pipeline through that old implementation. These are specific signature/binary checks, not a
guarantee against every conflict a third-party implementation could have with added method names.
New APIs have no previous-release ordering contract; their tie rules are documented in README/Javadoc.

### Executed checks

The dependency JAR identities and scopes remain those in the earlier record. The new manifest
also pins each matching POM's SHA-256. No production dependency or plugin version was added.

| Check | Actual result |
| --- | --- |
| Existing suite after implementation, before new tests | 62 passed. |
| First new-test compilation | Failed on an incorrect `SquareXZChunkScheme` import; corrected to Ashgrid's existing `implementation.grid.indexing` package. |
| Initial extension suite | 82 passed. |
| Final `clean verify dependency:tree`, Adoptium JDK 21.0.12.1+1 | **85 tests + 2 packaged-artifact tests passed**, no failures, errors or skips; finished 09:02:52 +02:00. |
| Same gate via `scripts/verify-local.ps1`, Oracle OpenJDK 25.0.2 | **85 + 2 passed**, no failures, errors or skips; finished 09:05:11 +02:00. |
| Four complete README programs | Compiled with release 21 against packaged JARs and executed in separate JVMs on each JDK. Original voxel/bounds, mapped grid, sphere entry/exit and clipping checks passed. |
| Artifact content and Javadoc | Main/source/documentation entries for new APIs, release-21 class version, coordinates and resources passed on both JDKs. |
| Existing public declarations / legacy binary / new visitor with legacy implementor | All passed. |
| Both workflows | actionlint 1.7.7 passed, optional ShellCheck/Pyflakes disabled. |

Both builds used Maven 3.9.16, Windows 11 amd64, UTF-8, pl_PL. Compiler release remains 21.
The 23 added tests cover mapped-grid sizes/origins/rotation/movement/snapshots, decimal/negative
cell positions, representability rejection, both geometry endpoints, negative direction, provider
world coordinates, surface flags, tangency, cavity intervals, true-distance ordering, wall equality,
segments, invalid output, expired emitters, visitor stopping, custom-index fallback and buffer
cleanup/reentry/independent results. README adds an actual small-coordinate sphere provider;
most contract tests deliberately supply known intervals to isolate tracer behavior from a shape solver.
No exhaustive geometry proof or cross-platform bitwise guarantee is inferred.

Logs and compatibility fixtures are retained locally under `.verification/`:
`extensions-verify-jdk21.log`, `extensions-verify-jdk25.log`, `extensions-api.log`,
`extensions-binary.log`, and the before/after benchmark logs. Maven reports are in `target`.
JDK 21 artifacts were preserved under `.verification/artifacts-jdk21`; `target` contains the JDK 25 build.

### Allocation and timing observation

`TraceWorkloadBenchmarkMain` uses a linear index of 5000 identical overlapping AABBs, one fixed
world ray, 100 warm-up calls and 500 measured calls. Every candidate is accepted; ordered first-hit
returns entry distance 2. A thread allocation counter records bytes and a volatile field consumes
the result. The baseline was measured before removing candidate wrappers/list copies.

| Route | ns/query | allocated bytes/query |
| --- | ---: | ---: |
| Baseline ordered firstHit | 179949 | 418002 |
| Updated ordered firstHit | 137551 | 237880 |
| Updated ordered firstHit with reused buffer | 85269 | 162592 |
| New anyHit, accepts first candidate | 2266 | 1248 |

These are single-process sequential observations on JDK 21, not JMH results, statistical confidence
intervals or portable latency guarantees. Reused capacity and fewer intermediate objects reduced
observed allocation for this workload. `anyHit` answers existence, so its timing is not a nearest-hit
speedup. Hash candidate collection and dynamic-BVH rebuilding remain; mutation-heavy or sparse
workloads were not benchmarked. No dynamic-tree rewrite is justified by this measurement alone.

### Extension artifact identity

| Artifact | JDK 21 SHA-256 | JDK 25 SHA-256 |
| --- | --- | --- |
| Main | `1a9c81c3f3d29163542c70e0ae2b5cfb91cd05dc135d403849445cf7e0d2ce10` | `902b9d0fdb51bfa3521ed9b41cf98e356f84e2bfa2cb634ee3837110030fe037` |
| Sources | `f27e02134ec8e632d7a5b89049bd985fb88974f5b4ada7be24854a6eb48ceb9e` | `f27e02134ec8e632d7a5b89049bd985fb88974f5b4ada7be24854a6eb48ceb9e` |
| Javadoc | `2956dcf16ddf2b701992a92700c2868c3334ec6aad133ea098c32d567d9b1fba` | `2b445958cdda3e5f76504515186d53f5ba6e3f01680371caaea55fd256ab4c79` |

Filenames remain `ashtrace-2.0.0-SNAPSHOT.jar`, `-sources.jar`, and `-javadoc.jar`.
The binary/documentation JAR hashes differ across JDK builds; cross-toolchain byte reproducibility
is not claimed. The pre-extension comparison JAR is the earlier `da683288...` artifact.

### Reproducing the local gate and finishing release preparation

With PowerShell, JDK 21+ and Maven installed, supply the directory containing the three already-built
dependency directories (`Ashcore`, `Ashgrid`, `Ashspace`). Each needs its matching root `pom.xml`
and `target/<artifactId>-<version>.jar`; no Git checkout or sibling build is performed by the script.

```powershell
& ./scripts/verify-local.ps1 -DependencyRoot ../
```

Use `-JavaHome`, `-MavenCommand` and `-SettingsFile` when tools/repositories need explicit paths.
The script checks every input against `scripts/development-dependencies.json` before installation,
then uses an isolated repository inside Ashtrace and runs `clean verify dependency:tree`. Without
`-SettingsFile` it writes an empty settings file locally instead of loading user Maven settings;
Maven still needs access to its build plugins and test dependencies. The local successful run used
the existing read-only artifact cache via explicit settings. `-nsu` disables snapshot update checks;
the manifest is an identity check for this verification set, not a general dependency lockfile.

Read-only remote inspection on 2026-09-10 returned dependency default heads:
Ashcore `ea6c715d2d451cc3636499a32b45230c525d8a72` (master), Ashgrid
`83da6d696c1b1cd1ff43422679094cc2a8b8c532` (master), Ashspace
`18db5782777f147476dfb40b9815fdebf63ef762` (main). These differ from the tested local correction
commits. Checking out those remote heads alone does not reproduce the verified dependency set.

To finish TRACE-010, provision these verified artifacts to hosted CI or publish verified lower-layer
releases and update the pinned coordinates/manifest accordingly, then rerun integration and the
hosted Java 21/25 matrix. Record its URL/commit before selecting a final release tag and destination.
The existing publishing workflow rejects snapshot Ashtrace/dependency versions. No push, remote CI,
deploy, GitHub release or Central publication was performed in this extension session.

## 2026-09-10 — Cross-library integration and quotient underflow

Branch `test/ashtrace-blackframe-integration-20260910`, checkpoint `8d70f81`, base `a6bf6f1`.
Initially all writes were confined to Ashtrace. After the integration test exposed a lower-layer
defect, the user authorized fixing a blocking defect in its owning library. Ashspace was fixed
on a separate branch/checkpoint (`51f5340`), correction commit **`f652173`**. Ashcore, Ashgrid and
Ashnav remained unchanged. No push, deployment or publication occurred.

### Reproduction and correction

The unchanged Ashspace `GridMappingIntegrationTest` failed with Ashtrace's actual dependency set:
origin zero, cellSize 2, point X = `-Double.MIN_VALUE`. Division returns -0.0; Ashspace selected
cell 0 while Ashgrid `VoxelSpace` selected -1. This is an extreme-input indexing discrepancy,
not a general outage in ordinary-scale tracing. It violates the shared cell contract, so it was
treated as an integration/release blocker. The old Ashspace build used Ashgrid 1.2.0, whose
behavior did not reveal disagreement with the corrected Ashgrid 1.3.0-SNAPSHOT helper.

Ashspace now retains the sign of an underflowed quotient for internal floor/ceil selection.
The correction includes positive/negative half-open range endpoints; it does not recover accurate
distances from a quotient too small for double. Its POM now pins Ashcore 1.1.0-SNAPSHOT and
Ashgrid 1.3.0-SNAPSHOT. See [SPACE-011 evidence](../Ashspace/VERIFICATION.md#2026-09-10--grid-quotient-underflow-space-011).

Ashtrace rejects an origin component whose nonzero grid-relative offset divides to zero, before
calling occupancy. This avoids silently placing the ray on the boundary or inventing a distance.
It is an explicit representability check on the mapped tracer; exact zero is still valid, and no
epsilon changes ordinary membership. Existing positive-distance underflow rejection remains.

The final diagnostic for point `(-Double.MIN_VALUE, 0.5, 0.5)` and cellSize 2 reports:

```text
point.x=-4.9E-324 quotient=-0.0
VoxelSpace x=-1
GridSpaceMapper3 x=-1
FrameGridSpaceMapper3 x=-1
Ashtrace rejects: grid origin coordinate is not representable
```

### Tests and results

`BlackframeGridContractIntegrationTest` and `BlackframeGeometryContractIntegrationTest` add 12
Ashtrace tests using the neighboring libraries' fixtures. They cover ray/sweep/sphere agreement
across all four indexes, Ashcore sphere entry/exit (including inside and tangent cases), normalized
extreme input directions, nested frame transforms, moving sparse storage, six axis directions,
eight direction octants, DDA zero-length corner visits, clipped traversers in cell coordinates,
large-world relative mapping, coordinate underflow and Raycast versus LineOfSight start-cell rules.

The optional `scripts/verify-blackframe-tests.ps1` first runs the full Ashtrace gate through
`verify-local.ps1`. It then copies the 11 files selected in `blackframe-contract-tests.json` into
a fresh run directory under `.verification/blackframe-tests`. It checks their SHA-256 before
copying and again after execution; the originals are neither edited nor built in place. A standalone
POM copies Ashtrace's pinned dependencies, properties and compiler/Surefire configuration. No
production dependency on a higher layer, extra test framework or published test artifact is added.

| Original library tests | Classes | Tests |
| --- | ---: | ---: |
| Ashcore normalization / collision API / collision boundaries / primitives | 4 | 23 |
| Ashgrid traversal/query / traversal boundaries / sparse storage | 3 | 17 |
| Ashspace chained frames / moving grid / grid mapping / frame-grid mapper | 4 | 14 |
| Total unchanged test sources | 11 | 54 |

| Verification | Actual result |
| --- | --- |
| Initial new Ashtrace scenarios | 10/11 passed; the sphere assertion compared 1.0 with 0.9999999999999998. It now uses the same 1e-12 unit-scale tolerance as Ashcore's primitive tests, without changing runtime geometry. |
| Initial imported original tests | 53/54 passed; the grid-mapping test exposed the dependency discrepancy. It was not disabled, edited or removed. |
| Ashtrace representability regression before the guard | Failed: no exception was thrown and occupancy could be queried in the wrong cell. |
| Ashspace mapping regressions before its fix | 7 tests, 3 failures (original mapping test and two new point/range regressions). |
| Ashspace `clean verify dependency:tree`, JDK 21 | **74 + 2 passed**, no failures/errors/skips. |
| Ashtrace full gate plus original library tests, JDK 21 | **97 + 2 + 54 passed**, no failures/errors/skips; gates finished 09:38:02 and 09:38:07 +02:00. |
| Same complete script, JDK 25 | **97 + 2 + 54 passed**, no failures/errors/skips; finished 09:38:39 and 09:38:43 +02:00. |
| Public API comparison | Ashtrace 26 existing types / 211 declarations; Ashspace 10 / 128. No removals or changed declarations. |
| Minimal boundary diagnostic after the fix | Both mappers match VoxelSpace; mapped tracing rejects loss of the nonzero coordinate. |

Environment: Maven 3.9.16, Adoptium JDK 21.0.12.1+1 and Oracle OpenJDK 25.0.2, Windows 11 amd64,
UTF-8, pl_PL. Both compile for release 21. Four README programs and packaged main/source/Javadoc
content checks ran on both JDKs. Ashspace's JAR was built on JDK 21 and consumed on both runtimes.

Two harness setup attempts failed before executing the imported tests: a JAR project cannot be a
Maven parent, and Maven rejected redundant namespace declarations on copied plugin elements. The
final standalone POM avoids both. One Ashspace clean attempt failed on an execution-account file
ownership conflict; cleanup was confined to its verified target directory before a successful gate.
These setup failures are retained in the logs and are distinct from the reproduced geometry defect.

Logs: `.verification/blackframe-corrected-jdk21.log`, `blackframe-corrected-jdk25.log`,
`blackframe-api.log`, `ashspace-underflow-api.log`, `grid-boundary-diagnostic.log`, and the earlier
`blackframe-*` / `ashspace-underflow-*` failure logs. Successful original-test run directories end
in `3496b6341f904d1f9f43bb1bd9990e04` (21) and `9d6d89aff5e94729bd015e4da07d1c96` (25); each
contains `imports.json` with source paths/hashes and its own Surefire reports.

### Current artifact identities and reproduction

The development manifest now selects Ashspace commit `f652173`, JAR SHA-256
`7f0e82346ee9c880a42e68eb24902439af7e9aba5ad7222b39930d9ab3cb0e94`, POM SHA-256
`3aefb1556e47d99fd6a86b736d31006af4543dcf04e2972afe71ea83c645c073`. Core/Grid identities are unchanged.
The older Ashspace identity in the preceding historical sections must not replace this corrected one.

| Ashtrace artifact | JDK 21 SHA-256 | JDK 25 SHA-256 |
| --- | --- | --- |
| Main | `753b370463d547b0b88d3b7be7712827527de0c3a3aee8c6d6d6c82552c213b0` | `c72a16030e4eebc1414f94c16d89ae4c5196e9f55ef1078dcad141848bdda67d` |
| Sources | `b068c76b0f359f1def4004655895f56c62b9843c8efdb85d97c9c044ed80d341` | `b068c76b0f359f1def4004655895f56c62b9843c8efdb85d97c9c044ed80d341` |
| Javadoc | `ce19e1a5570ca243c96877fc9719500083e899ebcdc4248d0c003e37245a0318` | `b5d57bdd74d105a9e47ab9a966b858581f725ac25365a957b35e9554138acd66` |

JDK 21 output was preserved in `.verification/integration-artifacts-jdk21`; `target` is the JDK 25
build. Hash differences across JDKs are not a cross-toolchain reproducibility claim.

With matching dependency artifacts and source tests available, reproduce from Ashtrace using:

```powershell
& ./scripts/verify-blackframe-tests.ps1 -DependencyRoot ../
```

Optional `-JavaHome`, `-MavenCommand` and `-SettingsFile` select the environment. All script output,
test copies and dependency installs stay in Ashtrace. Repairing Ashspace was a separate authorized
source change, not an action performed by this script. TRACE-010 still needs artifact distribution
and hosted CI before release; local success does not establish publication availability.

## 2026-09-10 — Oriented-box integration, TRACE-012

Branch `fix/ashtrace-rotated-primitives-20260910`, checkpoint `bbe70ff`, base `e07168f`.
All edits, dependency installs, copied tests and build outputs in this session stayed in Ashtrace.
Sibling artifacts and selected test sources were read, without rebuilding or editing those libraries.
The parent Blackframe directory is not a Git repository; the checkpoint belongs to Ashtrace.

### Integration and compatibility

TRACE-012 uses the existing `RayIntersector3` and exact pipelines. Ashspace converts a local AABB
to a world OBB and a separate enclosing AABB; Ashcore supplies the full supporting-line interval.
There is no new public adapter or intersection algorithm in Ashtrace. Production Java edits only
clarify the provider union contract, stable poses and the translation-only meaning of AABB sweep.
Existing bounds-based results and all public signatures retain their meaning. Ashtrace remains an
unpublished `2.0.0-SNAPSHOT`; adopting this dependency set requires the OBB-capable builds below.

`OrientedBoxTraceIntegrationTest` adds seven scenarios, each exercising linear, static BVH, spatial
hash and dynamic BVH indexes. Known-result fixtures cover rays/segments, negative full entry from
inside, reversed direction, tangency, zero/range limits, nested frames and world points, candidate-only
corner hits, shape ordering and each index's own full-tie order. Other fixtures cover provider-owned
overlap merging with a preserved cavity, clipping through a rotated grid of cell size 2, coherent
pose/bounds updates, saved result points and a half-turn whose endpoint bounds miss a middle contact.
The absolute `1e-12` assertion tolerance applies to these small-coordinate fixtures, not to general
geometry membership or a new runtime epsilon. Tangency uses an exact cyclic axis permutation.

The fifth complete README program, `AshtraceOrientedBoxQuickStart`, compiles/runs against JARs.
It demonstrates the OBB/AABB distinction and the half-turn counterexample. README explains world
ray distances versus Ashcore segment fractions, complete provider intervals, multipart union costs,
and updating immutable world-shape payloads as well as bounds. Pose samples do not establish a
continuous contact guarantee; CORE-013 remains deferred in its owning library.

### Actual dependencies and artifacts

`scripts/development-dependencies.json` pins full JAR and POM SHA-256 identities. `verify-local.ps1`
checked those files before installing them to `.verification/repository`. The resolved JAR hashes
also matched those inputs after verification. The dependency tree contains only the three compile
dependencies below and JUnit Jupiter 5.10.2 with test scope.

| Artifact | Source commit | JAR SHA-256 |
| --- | --- | --- |
| Ashcore 1.2.0-SNAPSHOT | `ddbf98cb092603e3543c71485a34b5e1a36264c0` | `4aca690477eea1f3943e3c8b333da6882ff9f76fd2d99f2ceb2a6a0a9d9e1379` |
| Ashgrid 1.3.0-SNAPSHOT | `8199f9b` | `b4a8d0ac87ebe34132f1f86d8870e8f32f5f270b95d263a0fc21e345a6e71285` |
| Ashspace 2.0.0-SNAPSHOT | `ebf6b9e13599c0eef01ebd66dbeb69e3a8493c2f` | `eec0770d713355b67e856a831ff3d370e41baf0fedabaa9005f86dd58171f389` |

This replaces the older Ashcore 1.1 and Ashspace quotient-underflow artifact set recorded above.
Historical hashes do not describe the current manifest. No released coordinates were overwritten.

| Ashtrace artifact | JDK | SHA-256 |
| --- | --- | --- |
| `ashtrace-2.0.0-SNAPSHOT.jar` | 21 | `6a9034819280d9c954a09e61ddae9ee0c19dd7a6e122ce74402c057d08af3170` |
| `ashtrace-2.0.0-SNAPSHOT.jar` | 25 | `c5129e1ea5f242eb7912775de16c38d2961360029545d59e188dca484b6fcfab` |
| `ashtrace-2.0.0-SNAPSHOT-sources.jar` | 25 | `ac0cb98417b6e166cfd15004542623d52544c15b91296a2590d7078c553d244d` |
| `ashtrace-2.0.0-SNAPSHOT-javadoc.jar` | 25 | `2787d5437eb13a92b8f3da96e07f4c244e01cc3402f13a4182f5a1439928aae7` |

Java 21 artifacts are retained in `.verification/rotated-artifacts-jdk21`; `target` holds the Java 25
build. Main/sources/Javadoc, Java 21 class version, coordinates, LICENSE/NOTICE and absence of JUnit
and artificial SPI in the main JAR passed `PackagedArtifactIT`. Dependency DDA service loading and
all five examples passed. Javadoc ran with `all,-missing` and `failOnError=true`.

### Executed verification

Maven **3.9.16**, Eclipse Adoptium **21.0.12.1+1** and Oracle OpenJDK **25.0.2**, Windows 11 amd64,
UTF-8, locale pl_PL; compilation release **21**. Existing tool installations were used read-only.

| Check | Result |
| --- | --- |
| Baseline `maven.ps1 -o test` on previous dependencies | 97 tests PASS |
| `verify-local.ps1` after dependency adoption | 97 tests + 2 artifact tests PASS |
| `maven.ps1 -o -Dtest=OrientedBoxTraceIntegrationTest test` | 7 new tests PASS |
| Final `verify-blackframe-tests.ps1`, JDK 21 | clean verify: 104 + 2 PASS; original lower-layer tests: 78 PASS |
| Final `verify-blackframe-tests.ps1`, JDK 25 | clean verify: 104 + 2 PASS; original lower-layer tests: 78 PASS |
| Public `javap` comparison against checkpoint JAR | 26 public types / 211 declarations, no removals or changes |

All executed tests reported zero failures, errors and skips. The lower-layer selection expanded from
11 to 14 unchanged source files by adding `OrientedBoxQueriesTest`, `ShapeTransforms3ApiTest` and
`ShapeSpaceConverter3ApiTest`. Source hashes were checked before/after each run; copies and their
POM live only under `.verification/blackframe-tests`. No sibling production build was executed.

Reproduction with the manifest's ready-built artifacts and the selected source tests:

```powershell
& ./scripts/verify-blackframe-tests.ps1 -DependencyRoot ../ -JavaHome '<JDK 21 directory>' -MavenCommand '<mvn.cmd path>'
& ./scripts/verify-blackframe-tests.ps1 -DependencyRoot ../ -JavaHome '<JDK 25 directory>' -MavenCommand '<mvn.cmd path>'
```

The local runs additionally used `-SettingsFile .verification/settings.xml`, an existing read-only
Maven cache source with an isolated writable repository. Build commands do not publish artifacts.
Full logs: `.verification/rotated-baseline.log`, `rotated-dependencies.log`, `rotated-first-tests.log`,
`rotated-final-jdk21.log`, `rotated-final-jdk25.log`, `rotated-api.log`. The API baseline was the
pre-change main JAR, SHA-256 `c72a16030e4eebc1414f94c16d89ae4c5196e9f55ef1078dcad141848bdda67d`.

The initial Maven attempt in the sandbox failed to launch `mvn.cmd` with a StandardOutputEncoding
error before running tests. The same tools ran successfully with execution escalation. Initial Git
reads from the parent directory found no repository; Ashtrace required a per-command `safe.directory`
override for sandbox ownership. No global Git configuration was changed. One documentation patch
had a stale context and was reapplied after checking the file; it did not change verification results.

TRACE-012 is complete locally. TRACE-010 still requires distribution of this updated dependency set
to the hosted runner, a successful Java 21/25 CI URL for the intended commit, and final release
coordinates/tag/destination checks. Hosted CI, push, deployment and publication were not performed;
local success does not establish remote artifact availability. No benchmark or new performance
guarantee was introduced.
