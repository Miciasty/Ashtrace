# Ashtrace WIKI

Static documentation for Ashtrace 2.0.0. Public articles use American English and the shared Blackframe WIKI shell. The site runs from local assets and hash routes, including under the `/Ashtrace/` GitHub Pages subpath.

## Preview

Use Node.js 20 or newer. From `wiki`:

```powershell
npm ci
npm run dev
```

The default preview is `http://127.0.0.1:4173`. The development command builds and watches CSS. Reload the browser after saving content. `npm run preview` serves the existing files without rebuilding. To select another port in PowerShell, set `$env:PORT = '4293'` first.

## Build and check

```powershell
npm run build
```

This compiles Tailwind CSS, checks page IDs, navigation, internal links, product version and local assets, verifies the diagram calculations, and writes `wiki/_site`. The published directory contains only `index.html`, `.nojekyll`, `assets`, `content`, `LICENSE`, and `NOTICE`. Source styles, dependencies, authoring notes, and development scripts remain outside that directory.

To compile and run every Java example, first resolve Ashtrace's declared runtime dependencies. Run this command from the repository root:

```powershell
mvn -B -ntp org.apache.maven.plugins:maven-dependency-plugin:3.8.1:copy-dependencies -DincludeScope=runtime -DoutputDirectory=target/wiki-dependencies
```

Then run this command from `wiki` with JDK 21 or newer on the path, or with `JAVA_HOME` set:

```powershell
npm run check:examples
```

The checker compiles this checkout's Ashtrace source together with the exact Java code blocks, enables assertions, runs every program, and compares stdout with each displayed output block. It also accepts dependencies from a local Maven repository selected by `MAVEN_REPO_LOCAL`, defaulting to `~/.m2/repository`. Temporary classes are written beneath the ignored `.verification` directory. No Minecraft server is started.

## Publish on GitHub Pages

1. Set repository **Settings → Pages → Build and deployment → Source** to **GitHub Actions**.
2. Merge the WIKI changes into the repository's default branch (`master` in this checkout).
3. Open the **Ashtrace WIKI** Actions run. Its build must pass before the deployment job can run.
4. Open the Pages URL reported by the deployment job and check a deep link, search, and local assets.

The workflow also validates pull requests without deploying them. Manual runs deploy only when they target the default branch. The intended project URL is `https://miciasty.github.io/Ashtrace/`; a local build does not establish that the public site is live.

## Maintain the articles

Edit `content/site.js` for product details and sidebar order. Edit the subject files for articles:

- `getting-started.js`: overview, dependencies, mounted-tool quick start.
- `tracing.js`: frames, exact intervals, mapped grids, and occlusion.
- `broadphase.js`: indexes, spatial queries, contracts, performance, and API reference.
- `integration.js`: rotated targets, plugin integration, and troubleshooting.

Keep stable page and section IDs. The shell uses `#/page-id?section=section-id` links. Add content files to `index.html` in their execution order. `content/pages.js` defines the shared HTML helpers before the articles load.

Edit `src/*.css`; `assets/styles.css` is generated. Preserve the locally vendored Prism code and its license. A new Java example must be a complete class with `main` and have an immediately following expected-output block. Put assertions in the program for the behavior its prose explains.

The three interactive figures explain interval clipping, mapped grid distance, and voxel occlusion. Static figures explain the mounted-tool coordinates, rotated target, and translating AABB. Add a figure only when it makes a concrete result easier to understand. Keep the same values in its labels, code, output, and explanation.

Working source maps and validation results are in `authoring`. They are not published with the WIKI.
