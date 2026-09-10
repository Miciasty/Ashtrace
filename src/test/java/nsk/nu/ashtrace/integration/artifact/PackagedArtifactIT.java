package nsk.nu.ashtrace.integration.artifact;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import org.junit.jupiter.api.Test;

import javax.tools.ToolProvider;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;
import java.util.concurrent.TimeUnit;
import java.util.jar.JarFile;
import java.util.regex.Pattern;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PackagedArtifactIT {
    private final Path directory = Path.of(System.getProperty("artifactDirectory"));
    private final String name = System.getProperty("artifactName");

    @Test
    void main_sources_javadoc_and_resources_match_the_coordinates() throws Exception {
        String type = "nsk/nu/ashtrace/api/trace/pipeline/FrameBroadPhaseRayTracer3";
        try (var main = new JarFile(directory.resolve(name + ".jar").toFile());
             var sources = new JarFile(directory.resolve(name + "-sources.jar").toFile());
             var docs = new JarFile(directory.resolve(name + "-javadoc.jar").toFile())) {
            assertNotNull(main.getJarEntry(type + ".class"));
            assertNotNull(main.getJarEntry("META-INF/LICENSE"));
            assertNotNull(main.getJarEntry("META-INF/NOTICE"));
            assertNotNull(sources.getJarEntry(type + ".java"));
            assertNotNull(docs.getJarEntry(type + ".html"));
            assertNotNull(docs.getJarEntry("index.html"));
            assertFalse(main.stream().anyMatch(e -> e.getName().startsWith("org/junit/")
                    || e.getName().startsWith("META-INF/services/")));
            var properties = new Properties();
            try (var in = main.getInputStream(main.getJarEntry("META-INF/maven/dev.nasaka.blackframe/ashtrace/pom.properties"))) {
                properties.load(in);
            }
            assertEquals("dev.nasaka.blackframe", properties.getProperty("groupId"));
            assertEquals("ashtrace", properties.getProperty("artifactId"));
            assertEquals(System.getProperty("artifactVersion"), properties.getProperty("version"));
            try (var in = main.getInputStream(main.getJarEntry(type + ".class"))) {
                byte[] header = in.readNBytes(8);
                assertEquals(65, ((header[6] & 255) << 8) | (header[7] & 255));
            }
        }
    }

    @Test
    void both_readme_examples_compile_and_run_with_packaged_libraries_and_dda_service() throws Exception {
        Path examples = Files.createTempDirectory(directory, "readme-");
        String readme = Files.readString(Path.of(System.getProperty("projectDirectory"), "README.md"));
        var blocks = Pattern.compile("(?s)```java\\s*\\R(.*?)```").matcher(readme);
        var classNames = new ArrayList<String>();
        String classpath = String.join(File.pathSeparator, directory.resolve(name + ".jar").toString(),
                dependency(Ray.class).toString(), dependency(VoxelTraverser.class).toString(), dependency(FrameGraph3.class).toString());
        var compiler = ToolProvider.getSystemJavaCompiler();
        assertNotNull(compiler, "Verification requires a JDK");
        while (blocks.find()) {
            String code = blocks.group(1);
            var declaration = Pattern.compile("public final class (\\w+)").matcher(code);
            assertTrue(declaration.find(), "README Java examples must be complete programs");
            String className = declaration.group(1);
            classNames.add(className);
            Path source = examples.resolve(className + ".java");
            Files.writeString(source, code);
            var diagnostics = new ByteArrayOutputStream();
            int compiled = compiler.run(null, diagnostics, diagnostics, "--release", "21", "-encoding", "UTF-8",
                    "-classpath", classpath, "-d", examples.toString(), source.toString());
            assertEquals(0, compiled, diagnostics.toString(StandardCharsets.UTF_8));
            Path output = examples.resolve(className + ".log");
            Process process = new ProcessBuilder(Path.of(System.getProperty("java.home"), "bin", "java").toString(),
                    "-cp", examples + File.pathSeparator + classpath, className)
                    .redirectErrorStream(true).redirectOutput(output.toFile()).start();
            boolean finished = process.waitFor(30, TimeUnit.SECONDS);
            if (!finished) process.destroyForcibly();
            assertTrue(finished, "README example timed out");
            assertEquals(0, process.exitValue(), Files.readString(output));
            String expected = className.equals("AshtraceQuickStart") ? "x=13" : "value=targetA";
            assertTrue(Files.readString(output).contains(expected), Files.readString(output));
        }
        assertEquals(List.of("AshtraceQuickStart", "AshtraceBroadPhaseQuickStart"), classNames);
    }

    private static Path dependency(Class<?> type) throws Exception {
        Path path = Path.of(type.getProtectionDomain().getCodeSource().getLocation().toURI());
        assertTrue(Files.isRegularFile(path), "Dependency must resolve to a JAR: " + path);
        return path;
    }
}
