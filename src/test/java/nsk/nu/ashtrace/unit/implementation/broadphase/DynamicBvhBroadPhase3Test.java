package nsk.nu.ashtrace.unit.implementation.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicBvhBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DynamicBvhBroadPhase3Test {

    @Test
    void insert_update_remove_and_ray_query_work() {
        // GIVEN
        DynamicBvhBroadPhase3<String> bvh = new DynamicBvhBroadPhase3<>();
        long near = bvh.insert(box(2, 0, 0, 3, 1, 1), "near");
        long far = bvh.insert(box(5, 0, 0, 6, 1, 1), "far");
        List<BroadPhaseRayHit3<String>> firstHits = new ArrayList<>();

        // WHEN
        bvh.queryRay(new Ray(new Vector3(0.2, 0.5, 0.5), new Vector3(1, 0, 0)), 10.0, firstHits::add);
        boolean updated = bvh.updateBounds(near, box(8, 0, 0, 9, 1, 1));
        boolean missing = bvh.updateBounds(far + 123, box(10, 0, 0, 11, 1, 1));
        boolean removed = bvh.remove(far);

        List<BroadPhaseRayHit3<String>> secondHits = new ArrayList<>();
        bvh.queryRay(new Ray(new Vector3(0.2, 0.5, 0.5), new Vector3(1, 0, 0)), 12.0, secondHits::add);

        // THEN
        assertEquals(2, firstHits.size());
        assertEquals("near", firstHits.get(0).value());
        assertEquals("far", firstHits.get(1).value());
        assertTrue(updated);
        assertFalse(missing);
        assertTrue(removed);
        assertEquals(1, secondHits.size());
        assertEquals("near", secondHits.get(0).value());
    }

    @Test
    void proximity_and_sweep_queries_are_supported() {
        // GIVEN
        DynamicBvhBroadPhase3<String> bvh = new DynamicBvhBroadPhase3<>();
        bvh.insert(box(2, 0, 0, 3, 1, 1), "near");
        bvh.insert(box(4, 0, 0, 5, 1, 1), "far");

        List<String> sphereHits = new ArrayList<>();
        List<BroadPhaseSweepHit3<String>> sweepHits = new ArrayList<>();

        // WHEN
        bvh.querySphere(new Vector3(2.5, 0.5, 0.5), 0.6, sphereHits::add);
        BroadPhaseNearestHit3<String> nearest = bvh.nearest(new Vector3(3.9, 0.5, 0.5), 2.0);
        BroadPhaseNearestHit3<String> none = bvh.nearest(new Vector3(20, 20, 20), 1.0);
        bvh.querySweptAabb(box(0, 0, 0, 1, 1, 1), new Vector3(5, 0, 0), sweepHits::add);

        // THEN
        assertEquals(List.of("near"), sphereHits);
        assertNotNull(nearest);
        assertEquals("far", nearest.value());
        assertNull(none);
        assertEquals(2, sweepHits.size());
        assertEquals("near", sweepHits.get(0).value());
        assertEquals("far", sweepHits.get(1).value());
    }

    @Test
    void ray_query_rejects_non_finite_tmax() {
        // GIVEN
        DynamicBvhBroadPhase3<String> bvh = new DynamicBvhBroadPhase3<>();
        bvh.insert(box(1, 1, 1, 2, 2, 2), "A");
        Ray ray = new Ray(new Vector3(0, 1.5, 1.5), new Vector3(1, 0, 0));

        // WHEN / THEN
        assertThrows(IllegalArgumentException.class, () -> bvh.queryRay(ray, Double.NaN, hit -> { }));
        assertThrows(IllegalArgumentException.class, () -> bvh.queryRay(ray, Double.POSITIVE_INFINITY, hit -> { }));
    }

    private static AxisAlignedBox box(
            double minX, double minY, double minZ,
            double maxX, double maxY, double maxZ
    ) {
        return new AxisAlignedBox(
                new Vector3(minX, minY, minZ),
                new Vector3(maxX, maxY, maxZ)
        );
    }
}
