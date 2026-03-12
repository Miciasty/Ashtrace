package nsk.nu.ashtrace.unit.implementation.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DynamicSpatialHashBroadPhase3Test {

    @Test
    void insert_update_remove_work_for_dynamic_scene() {
        // GIVEN
        DynamicSpatialHashBroadPhase3<String> hash = new DynamicSpatialHashBroadPhase3<>(1.0);
        long a = hash.insert(box(0, 0, 0, 1, 1, 1), "A");
        long b = hash.insert(box(2, 0, 0, 3, 1, 1), "B");
        List<String> hits = new ArrayList<>();

        // WHEN
        hash.query(box(0, 0, 0, 3, 1, 1), hits::add);
        boolean updated = hash.updateBounds(b, box(4, 0, 0, 5, 1, 1));
        boolean removed = hash.remove(a);
        List<String> after = new ArrayList<>();
        hash.query(box(0, 0, 0, 5, 1, 1), after::add);

        // THEN
        assertEquals(List.of("A", "B"), hits);
        assertTrue(updated);
        assertTrue(removed);
        assertEquals(List.of("B"), after);
        assertEquals(1, hash.size());
    }

    @Test
    void ray_query_returns_sorted_intervals() {
        // GIVEN
        DynamicSpatialHashBroadPhase3<String> hash = new DynamicSpatialHashBroadPhase3<>(1.0);
        hash.insert(box(5, 1, 0, 6, 2, 1), "B");
        hash.insert(box(2, 1, 0, 3, 2, 1), "A");
        Ray ray = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));
        List<BroadPhaseRayHit3<String>> hits = new ArrayList<>();

        // WHEN
        hash.queryRay(ray, 10.0, hits::add);

        // THEN
        assertEquals(2, hits.size());
        assertEquals("A", hits.get(0).value());
        assertEquals("B", hits.get(1).value());
    }

    @Test
    void repeated_query_is_deterministic() {
        // GIVEN
        DynamicSpatialHashBroadPhase3<String> hash = new DynamicSpatialHashBroadPhase3<>(1.0);
        hash.insert(box(0, 0, 0, 1, 1, 1), "A");
        hash.insert(box(2, 0, 0, 3, 1, 1), "B");
        hash.insert(box(4, 0, 0, 5, 1, 1), "C");

        List<String> first = new ArrayList<>();
        List<String> second = new ArrayList<>();

        // WHEN
        hash.query(box(-1, -1, -1, 6, 2, 2), first::add);
        hash.query(box(-1, -1, -1, 6, 2, 2), second::add);

        // THEN
        assertEquals(first, second);
    }

    @Test
    void invalid_inputs_are_rejected() {
        // GIVEN / WHEN / THEN
        assertThrows(IllegalArgumentException.class, () -> new DynamicSpatialHashBroadPhase3<>(0.0));
        assertThrows(IllegalArgumentException.class, () -> new DynamicSpatialHashBroadPhase3<>(Double.NaN));

        DynamicSpatialHashBroadPhase3<String> hash = new DynamicSpatialHashBroadPhase3<>(1.0);
        long handle = hash.insert(box(0, 0, 0, 1, 1, 1), "A");
        assertFalse(hash.updateBounds(handle + 123, box(1, 1, 1, 2, 2, 2)));
        assertFalse(hash.remove(handle + 123));
    }

    @Test
    void sphere_nearest_and_sweep_queries_work() {
        // GIVEN
        DynamicSpatialHashBroadPhase3<String> hash = new DynamicSpatialHashBroadPhase3<>(1.0);
        hash.insert(box(2, 0, 0, 3, 1, 1), "near");
        hash.insert(box(4, 0, 0, 5, 1, 1), "far");

        List<String> sphereHits = new ArrayList<>();
        List<BroadPhaseSweepHit3<String>> sweepHits = new ArrayList<>();

        // WHEN
        hash.querySphere(new Vector3(2.5, 0.5, 0.5), 0.6, sphereHits::add);
        BroadPhaseNearestHit3<String> nearest = hash.nearest(new Vector3(3.9, 0.5, 0.5), 2.0);
        BroadPhaseNearestHit3<String> none = hash.nearest(new Vector3(20, 20, 20), 1.0);
        hash.querySweptAabb(box(0, 0, 0, 1, 1, 1), new Vector3(5, 0, 0), sweepHits::add);

        // THEN
        assertEquals(List.of("near"), sphereHits);
        assertNotNull(nearest);
        assertEquals("far", nearest.value());
        assertNull(none);
        assertEquals(2, sweepHits.size());
        assertEquals("near", sweepHits.get(0).value());
        assertEquals("far", sweepHits.get(1).value());
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
