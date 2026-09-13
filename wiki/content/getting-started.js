(() => {
  const {code, table, note} = window.WIKI_HTML;
  window.WIKI_PAGES.push(
    {
      id: 'overview', category: 'Getting started', title: 'Ashtrace documentation', navTitle: 'Overview', kind: 'concept',
      description: 'Find what a ray reaches: an object’s bounds, a shape surface, or an occupied grid cell.',
      intro: '<p>Ashtrace is a Java library for tracing rays and querying objects in space. Use it in a Minecraft plugin when a tool, projectile, or sensor needs a geometric result. Your plugin supplies the objects, occupied cells, and gameplay rules.</p>',
      sections: [
        {id: 'choose-a-query', title: 'Start with the result you need', html:
          table(['You need to…', 'Use', 'The result means'], [
            ['Find the block a mounted drill reaches.', '<a href="#/grid-tracing">Grid tracing</a>', 'The ray entered an occupied cell in your grid.'],
            ['Find entry and exit through a target.', '<a href="#/exact-tracing">Exact tracing</a>', 'Your geometry callback reported a shape interval.'],
            ['Stop a target query at a wall.', '<a href="#/occlusion">Voxel occlusion</a>', 'The first occupied cell limits the object query.'],
            ['Reduce the objects that need detailed tests.', '<a href="#/broad-phase">Broad-phase queries</a>', 'A ray or region intersects an enclosing box.'],
            ['Find nearby objects or a moving box’s candidates.', '<a href="#/proximity-sweeps">Proximity and sweeps</a>', 'Distance or contact time refers to indexed AABBs.']
          ])},
        {id: 'three-meanings', title: 'Know what was hit', html:
          '<p>An <strong>axis-aligned bounding box (AABB)</strong> encloses an object along the coordinate axes. It can contain empty space. A broad-phase hit locates that box; a boolean acceptance callback can reject the box, but cannot move the hit to the object’s surface.</p><p><strong>Exact tracing</strong> uses the full entry and exit intervals supplied by your geometry callback. It returns world points and flags that tell you whether an endpoint is a surface or a query limit.</p><p>A <strong>voxel</strong> is one integer-addressed cell. Grid tracing asks your occupancy callback which visited cells are occupied. It does not read a Minecraft world by itself.</p>' +
          note('One unit convention per scene', '<p>The examples use one world unit per Minecraft block. Rays have normalized directions, so ray distances use world units. A sweep instead reports a fraction of a supplied displacement.</p>')},
        {id: 'blackframe', title: 'Use Ashtrace with Blackframe', html:
          table(['Library', 'Role in a tracing query'], [
            ['<a href="https://github.com/Miciasty/Ashcore">Ashcore</a>', 'Vectors, rays, boxes, and geometric intersection helpers.'],
            ['<a href="https://github.com/Miciasty/Ashspace">Ashspace</a>', 'Coordinate frames, rigid transforms, and grid-to-world mapping.'],
            ['<a href="https://github.com/Miciasty/Ashgrid">Ashgrid</a>', 'Voxel traversal and grid data structures.'],
            ['Ashtrace', 'Candidate selection, frame conversion, interval clipping, and trace results.'],
            ['<a href="https://github.com/Miciasty/Ashnav">Ashnav</a>', 'Pathfinding for a navigation problem after you define traversable space. It is not an Ashtrace dependency.']
          ]) + '<p>Ashtrace 2.0.0 has no Bukkit dependency, server commands, permissions, or configuration file. Package it as a library in your application. It does not apply damage, load chunks, simulate projectile motion, or decide whether a material blocks a tool.</p>'},
        {id: 'first-steps', title: 'Run your first query', html:
          '<ol class="steps my-6 list-none p-0 [counter-reset:steps]"><li class="relative m-0 pb-[25px] pl-[43px] max-[680px]:pl-[37px]"><strong><a href="#/installation">Add the dependency</a></strong><p>Use Java 21 or newer and include Ashtrace 2.0.0.</p></li><li class="relative m-0 pb-[25px] pl-[43px] max-[680px]:pl-[37px]"><strong><a href="#/quick-start">Trace a mounted tool</a></strong><p>Run a complete Java example and check the expected cell and distance.</p></li><li class="relative m-0 pb-[25px] pl-[43px] max-[680px]:pl-[37px]"><strong><a href="#/examples">Connect the query to your plugin</a></strong><p>Choose the frame, occupancy rules, and object geometry before applying the result.</p></li></ol>'}
      ]
    },
    {
      id: 'installation', category: 'Getting started', title: 'Installation', kind: 'guide',
      description: 'Add Ashtrace 2.0.0 to a Java project and retain its runtime dependencies when packaging a plugin.',
      sections: [
        {id: 'requirements', title: 'Check the runtime', html:
          '<p>Use JDK 21 or newer to compile and run the library. For a server plugin, the server’s Java runtime must meet the same requirement. Ashtrace is a library JAR; your plugin provides the server entry point and integration with the server API.</p>' +
          table(['Artifact', 'Version', 'Resolution'], [
            ['<code>dev.nasaka.blackframe:ashtrace</code>', '2.0.0', 'Your direct dependency'],
            ['<code>dev.nasaka.blackframe:ashcore</code>', '1.2.0', 'Transitive dependency'],
            ['<code>dev.nasaka.blackframe:ashgrid</code>', '1.3.0', 'Transitive dependency'],
            ['<code>dev.nasaka.blackframe:ashspace</code>', '2.0.0', 'Transitive dependency']
          ])},
        {id: 'maven', title: 'Add the Maven dependency', html:
          '<p>Add this dependency to your project’s <code>pom.xml</code>. Maven Central supplies Ashtrace and the three dependencies above; no additional repository declaration is needed.</p>' +
          code('xml', 'pom.xml · dependencies', '<dependency>\n  <groupId>dev.nasaka.blackframe</groupId>\n  <artifactId>ashtrace</artifactId>\n  <version>2.0.0</version>\n</dependency>')},
        {id: 'gradle', title: 'Add the Gradle dependency', html:
          code('kotlin', 'build.gradle.kts', 'repositories {\n    mavenCentral()\n}\n\ndependencies {\n    implementation("dev.nasaka.blackframe:ashtrace:2.0.0")\n}\n\njava {\n    toolchain {\n        languageVersion.set(JavaLanguageVersion.of(21))\n    }\n}')},
        {id: 'plugin-packaging', title: 'Include the runtime libraries', html:
          '<p>Make Ashtrace and its dependencies available to your plugin’s class loader. When you build one shaded plugin JAR, include the dependency classes and merge service descriptors under <code>META-INF/services</code>. Ashgrid’s DDA provider is discovered through these descriptors.</p><p>For Maven Shade, add the following transformer inside the shade execution’s <code>configuration</code>. This is a configuration fragment for a project that already uses that plugin.</p>' +
          code('xml', 'Maven Shade · configuration fragment', '<transformers>\n  <transformer implementation="org.apache.maven.plugins.shade.resource.ServicesResourceTransformer"/>\n</transformers>') +
          '<p>Check the final packaged plugin as well as the IDE classpath. A JAR that loses service entries can compile successfully and still fail at <code>require("dda")</code>. Preserve the dependency license and notice files in your distribution.</p><p><code>ServiceRegistry.of(type)</code> normally uses the current thread’s context class loader. If that loader cannot see your plugin’s providers, select the loader explicitly: <code>ServiceRegistry.of(VoxelTraverser.class, VoxelTraverser.class.getClassLoader()).require("dda")</code>.</p>'},
        {id: 'verify-installation', title: 'Check the dependency with a trace', html:
          '<p>Run the <a href="#/quick-start">quick start</a> with your application’s runtime classpath. It checks service discovery, frame conversion, and an occupied cell result. To build Ashtrace itself from source, use Maven 3.9 or newer:</p>' +
          code('powershell', 'Terminal · Ashtrace repository', 'mvn -B clean verify') +
          '<p>The source build runs the library tests and packaged-artifact checks. It does not install or start a Minecraft server.</p>'}
      ]
    },
    {
      id: 'quick-start', category: 'Getting started', title: 'Trace a mounted tool', navTitle: 'Quick start', kind: 'guide',
      description: 'Convert a tool-local ray to world coordinates and find the first occupied unit cell.',
      sections: [
        {id: 'scene', title: 'Define the scene', html:
          '<p>The tool frame is translated 10 world units along X. Its local ray begins at <code>(0.2, 1.2, 0.2)</code> and points along +X. The transformed ray begins at <code>(10.2, 1.2, 0.2)</code>. Only world cell <code>(13, 1, 0)</code> is occupied.</p><p>This example uses unit cells rooted at world zero. The cell begins at world X = 13, so the expected distance is <code>13 − 10.2 = 2.8</code> world units.</p>' +
          '<figure><div class="static-trace-viewport" role="region" aria-label="Spatial diagram. Scroll horizontally if needed." tabindex="0"><svg viewBox="0 0 640 155" role="img" aria-label="XY slice at Z 0.2. Ray begins at world X 10.2, reaches occupied cell 13 at a distance of 2.8 world units." style="width:100%;color:var(--text)"><rect x="376" y="48" width="88" height="52" rx="3" fill="var(--accent-soft)" stroke="var(--accent)"/><path d="M112 48v52m88-52v52m88-52v52M112 100h440" fill="none" stroke="var(--border)"/><path d="M129.6 76H376m-9-5 9 5-9 5" fill="none" stroke="var(--accent)" stroke-width="2"/><circle cx="129.6" cy="76" r="4" fill="var(--text)"/><g fill="currentColor" font-family="system-ui,sans-serif" font-size="13"><text x="32" y="23">XY slice · Z = 0.2</text><text x="96" y="126">10</text><text x="185" y="126">11</text><text x="273" y="126">12</text><text x="361" y="126">13</text><text x="449" y="126">14</text><text x="475" y="126">World X</text><text x="74" y="58">Origin 10.2</text><text x="200" y="65">2.8 units →</text><text x="376" y="40">Occupied (13, 1, 0)</text></g></svg></div><figcaption>The ray stays at Y = 1.2 and Z = 0.2. Occupancy describes whole cells; the hit is the cell entry.</figcaption></figure>'},
        {id: 'run', title: 'Run the query', html:
          code('java', 'MountedToolExample.java', `import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;

public final class MountedToolExample {
    public static void main(String[] args) {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId tool = new FrameId("tool");
        frames.define(tool, frames.root(), RigidTransform3.translation(10, 0, 0));
        VoxelTraverser dda = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, dda);
        Ray localRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        var hit = tracer.firstHit(tool, localRay, 10,
                (x, y, z) -> x == 13 && y == 1 && z == 0);
        if (hit == null || hit.x() != 13 || hit.y() != 1 || hit.z() != 0
                || Math.abs(hit.tEnter() - 2.8) > 1e-12) {
            throw new AssertionError("Expected a cell entry 2.8 units from the ray origin");
        }
        System.out.printf(java.util.Locale.ROOT, "distance=%.1f world units%n", hit.tEnter());
    }
}`) + code('output', 'Expected output', 'distance=2.8 world units')},
        {id: 'result', title: 'Read the result', html:
          '<p><code>GridRayHit3</code> identifies the visited grid cell and its entry distance. If no visited cell is occupied, <code>firstHit</code> returns <code>null</code>. The predicate here is an in-memory example; replace it with your own cell data.</p>' +
          note('Range boundary', '<p>The DDA traversal visits cells in <code>[0, tMax)</code>. A cell entered exactly at the range limit is excluded. A range of zero performs no occupancy calls.</p>') +
          '<p>For a grid with its own frame, origin, or cell size, use <a href="#/grid-tracing">mapped grid tracing</a>. For an object’s actual surface, use <a href="#/exact-tracing">exact tracing</a>. Keep those meanings distinct when applying damage or activating a tool.</p>'}
      ]
    }
  );
})();
