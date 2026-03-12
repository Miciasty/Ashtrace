package nsk.nu.ashtrace.benchmark.manual;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

import java.util.ArrayList;
import java.util.List;

/**
 * Simple local benchmark harness for comparing broad-phase implementations.
 *
 * <p>Run manually from IDE or CLI when tuning performance:
 * {@code mvn -DskipTests=true test-compile exec:java ...}</p>
 */
public final class BroadPhaseBenchmarkMain {
    private BroadPhaseBenchmarkMain() {}

    public static void main(String[] args) {
        int n = 50_000;
        int iterations = 200;

        List<AabbEntry3<Integer>> entries = new ArrayList<>(n);
        DynamicSpatialHashBroadPhase3<Integer> dynamic = new DynamicSpatialHashBroadPhase3<>(4.0);

        for (int i = 0; i < n; i++) {
            double x = (i % 500) * 1.25;
            double y = ((i / 500) % 50) * 1.25;
            double z = (i / 25_000) * 1.25;
            AxisAlignedBox box = new AxisAlignedBox(
                    new Vector3(x, y, z),
                    new Vector3(x + 1.0, y + 1.0, z + 1.0)
            );
            entries.add(new AabbEntry3<>(box, i));
            dynamic.insert(box, i);
        }

        LinearAabbBroadPhase3<Integer> linear = new LinearAabbBroadPhase3<>(entries);
        BvhAabbBroadPhase3<Integer> bvh = new BvhAabbBroadPhase3<>(entries);
        Ray ray = new Ray(new Vector3(0.2, 10.2, 0.2), new Vector3(1, 0, 0));
        AxisAlignedBox query = new AxisAlignedBox(new Vector3(100, 0, 0), new Vector3(350, 100, 10));

        long linearNs = time(iterations, () -> {
            int[] count = {0};
            linear.query(query, v -> count[0]++);
            linear.queryRay(ray, 1_000.0, v -> count[0]++);
        });

        long bvhNs = time(iterations, () -> {
            int[] count = {0};
            bvh.query(query, v -> count[0]++);
            bvh.queryRay(ray, 1_000.0, v -> count[0]++);
        });

        long dynamicNs = time(iterations, () -> {
            int[] count = {0};
            dynamic.query(query, v -> count[0]++);
            dynamic.queryRay(ray, 1_000.0, v -> count[0]++);
        });

        System.out.println("entries=" + n + ", iterations=" + iterations);
        System.out.println("linear ns/op   = " + linearNs / iterations);
        System.out.println("bvh ns/op      = " + bvhNs / iterations);
        System.out.println("dynamic ns/op  = " + dynamicNs / iterations);
    }

    private static long time(int iterations, Runnable task) {
        long start = System.nanoTime();
        for (int i = 0; i < iterations; i++) {
            task.run();
        }
        return System.nanoTime() - start;
    }
}
