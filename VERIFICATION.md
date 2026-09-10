# Ashtrace verification and migration

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
