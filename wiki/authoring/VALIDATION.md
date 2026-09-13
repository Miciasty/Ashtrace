# WIKI validation — 2026-09-13

Documented library: Ashtrace 2.0.0. Validation uses the local checkout's Java source and the declared Ashcore 1.2.0, Ashgrid 1.3.0, and Ashspace 2.0.0 dependencies.

| Check | Result | Scope |
| --- | --- | --- |
| Locked documentation dependencies | PASS | `npm ci --offline --ignore-scripts --no-audit --no-fund`, 33 packages from the local cache. |
| Production build | PASS | Tailwind 4.3.3; static file allowlist copied to `wiki/_site`. |
| Authored content | PASS | 15 pages, 62 sections, 95 content links, sidebar entries, header destinations, version consistency, and local assets. |
| JavaScript syntax | PASS | All local script tags parse; all content scripts execute in their declared order. |
| Diagram models | PASS | 14 groups of checks, including every exact-interval slider combination, range clipping, starting inside, wall contact, and mapped-grid inputs. |
| Published Java examples | PASS | 12 complete programs compiled with `javac --release 21`, run with assertions enabled, and matched their displayed stdout. JDK: Temurin 21.0.12.1. |
| Desktop browser routes | PASS | All 15 pages rendered from `/_site/` with one active sidebar item and no document-level horizontal overflow. No console errors or warnings in the artifact tab. |
| Narrow browser routes | PASS | All 15 pages checked at a 390 × 844 viewport (375 CSS pixels of content beside the scrollbar); no document-level horizontal overflow. Tables, code, and diagrams scroll within their own containers. |
| Navigation and search | PASS | Mobile drawer opens and closes, Escape restores it, Maven link appears inside it, search finds `exitSurface`, Enter follows a section route, unknown routes provide a return link. |
| Clipboard | PASS | The exact Java example copies as source text; no toolbar text is included. |
| Diagrams and themes | PASS | Dark/light visual inspection; range and origin controls, cell-size change, wall positions, default reset, and keyboard controls. |

The runnable programs are `MountedToolExample`, `FrameSnapshotExample`, `ExactSphereExample`, `MappedGridExample`, `OcclusionExample`, `BoundsCandidatesExample`, `BoundsVisitorExample`, `DynamicIndexExample`, `ProximityExample`, `MovingBoxExample`, `QueryBufferExample`, and `RotatedTargetExample`.

Browser checks use the static publication artifact beneath a subdirectory, matching the relative-path requirement of a GitHub Pages project site. They do not establish that the public GitHub Pages deployment has run. GitHub Actions and the public URL require verification after merge and publication.

No Minecraft server was started. The examples validate Java library use and mathematical results. The plugin integration guidance describes the application adapter's responsibilities rather than claiming a tested server-specific adapter.
