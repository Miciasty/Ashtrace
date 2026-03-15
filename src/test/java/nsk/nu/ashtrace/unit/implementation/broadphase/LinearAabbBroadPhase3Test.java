package nsk.nu.ashtrace.unit.implementation.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

class LinearAabbBroadPhase3Test {

    @Test
    void query_returns_intersections_in_stable_insertion_order() {
        // GIVEN
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(0, 0, 0, 2, 2, 2), "A"),
                new AabbEntry3<>(box(10, 0, 0, 12, 2, 2), "B"),
                new AabbEntry3<>(box(1, 1, 1, 3, 3, 3), "C")
        ));

        List<String> result = new ArrayList<>();

        // WHEN
        broadPhase.query(box(1.5, 1.5, 1.5, 2.5, 2.5, 2.5), result::add);

        // THEN
        assertEquals(List.of("A", "C"), result);
    }

    @Test
    void constructor_copies_input_entries() {
        // GIVEN
        List<AabbEntry3<String>> source = new ArrayList<>();
        source.add(new AabbEntry3<>(box(0, 0, 0, 1, 1, 1), "A"));

        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(source);
        source.clear();

        List<String> result = new ArrayList<>();

        // WHEN
        broadPhase.query(box(0, 0, 0, 1, 1, 1), result::add);

        // THEN
        assertEquals(1, broadPhase.size());
        assertEquals(List.of("A"), result);
    }

    @Test
    void ray_query_reports_candidate_intervals() {
        // GIVEN
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(2, 1, 0, 3, 2, 1), "A"),
                new AabbEntry3<>(box(5, 1, 0, 6, 2, 1), "B")
        ));
        Ray ray = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));
        List<BroadPhaseRayHit3<String>> hits = new ArrayList<>();

        // WHEN
        broadPhase.queryRay(ray, 10.0, hits::add);

        // THEN
        assertEquals(2, hits.size());
        assertEquals("A", hits.get(0).value());
        assertEquals("B", hits.get(1).value());
        assertEquals(1.8, hits.get(0).tEnter(), 1e-9);
        assertEquals(2.8, hits.get(0).tExit(), 1e-9);
    }

    @Test
    void sphere_and_nearest_queries_work() {
        // GIVEN
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(0, 0, 0, 1, 1, 1), "A"),
                new AabbEntry3<>(box(3, 0, 0, 4, 1, 1), "B")
        ));
        List<String> sphereHits = new ArrayList<>();

        // WHEN
        broadPhase.querySphere(new Vector3(0.5, 0.5, 0.5), 0.4, sphereHits::add);
        BroadPhaseNearestHit3<String> nearest = broadPhase.nearest(new Vector3(2.9, 0.5, 0.5), 3.0);
        BroadPhaseNearestHit3<String> none = broadPhase.nearest(new Vector3(10, 10, 10), 1.0);

        // THEN
        assertEquals(List.of("A"), sphereHits);
        assertNotNull(nearest);
        assertEquals("B", nearest.value());
        assertNull(none);
    }

    @Test
    void swept_aabb_reports_sorted_hit_intervals() {
        // GIVEN
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(4, 0, 0, 5, 1, 1), "far"),
                new AabbEntry3<>(box(2, 0, 0, 3, 1, 1), "near")
        ));
        AxisAlignedBox moving = box(0, 0, 0, 1, 1, 1);
        Vector3 delta = new Vector3(5, 0, 0);
        List<BroadPhaseSweepHit3<String>> hits = new ArrayList<>();

        // WHEN
        broadPhase.querySweptAabb(moving, delta, hits::add);

        // THEN
        assertEquals(2, hits.size());
        assertEquals("near", hits.get(0).value());
        assertEquals("far", hits.get(1).value());
        assertEquals(0.2, hits.get(0).tEnter(), 1e-9);
        assertEquals(0.6, hits.get(1).tEnter(), 1e-9);
    }

    @Test
    void ray_query_rejects_non_finite_tmax() {
        // GIVEN
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(1, 1, 1, 2, 2, 2), "A")
        ));
        Ray ray = new Ray(new Vector3(0, 1.5, 1.5), new Vector3(1, 0, 0));

        // WHEN / THEN
        assertThrows(IllegalArgumentException.class, () -> broadPhase.queryRay(ray, Double.NaN, hit -> { }));
        assertThrows(IllegalArgumentException.class, () -> broadPhase.queryRay(ray, Double.POSITIVE_INFINITY, hit -> { }));
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
