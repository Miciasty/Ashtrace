package nsk.nu.ashtrace.unit.implementation.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BvhAabbBroadPhase3Test {

    @Test
    void query_matches_linear_reference_candidates() {
        // GIVEN
        List<AabbEntry3<String>> entries = List.of(
                new AabbEntry3<>(box(0, 0, 0, 2, 2, 2), "A"),
                new AabbEntry3<>(box(5, 0, 0, 7, 2, 2), "B"),
                new AabbEntry3<>(box(1, 1, 1, 3, 3, 3), "C"),
                new AabbEntry3<>(box(-2, -2, -2, -1, -1, -1), "D"),
                new AabbEntry3<>(box(2, 0, 0, 4, 1, 1), "E")
        );

        LinearAabbBroadPhase3<String> linear = new LinearAabbBroadPhase3<>(entries);
        BvhAabbBroadPhase3<String> bvh = new BvhAabbBroadPhase3<>(entries);
        AxisAlignedBox query = box(1.5, 0.5, 0.5, 5.5, 2.5, 2.5);

        List<String> linearHits = new ArrayList<>();
        List<String> bvhHits = new ArrayList<>();

        // WHEN
        linear.query(query, linearHits::add);
        bvh.query(query, bvhHits::add);
        linearHits.sort(String::compareTo);
        bvhHits.sort(String::compareTo);

        // THEN
        assertEquals(linearHits, bvhHits);
        assertEquals(5, bvh.size());
    }

    @Test
    void query_is_deterministic_across_repeated_calls() {
        // GIVEN
        BvhAabbBroadPhase3<String> bvh = new BvhAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(0, 0, 0, 2, 2, 2), "A"),
                new AabbEntry3<>(box(3, 0, 0, 5, 2, 2), "B"),
                new AabbEntry3<>(box(3, 3, 0, 5, 5, 2), "C"),
                new AabbEntry3<>(box(0, 3, 0, 2, 5, 2), "D")
        ));
        AxisAlignedBox query = box(-1, -1, -1, 6, 6, 3);

        List<String> first = new ArrayList<>();
        List<String> second = new ArrayList<>();

        // WHEN
        bvh.query(query, first::add);
        bvh.query(query, second::add);

        // THEN
        assertEquals(first, second);
    }

    @Test
    void empty_index_returns_no_candidates() {
        // GIVEN
        BvhAabbBroadPhase3<String> bvh = new BvhAabbBroadPhase3<>(List.of());
        List<String> result = new ArrayList<>();

        // WHEN
        bvh.query(box(0, 0, 0, 1, 1, 1), result::add);

        // THEN
        assertTrue(result.isEmpty());
    }

    @Test
    void ray_query_returns_sorted_hits_by_entry_time() {
        // GIVEN
        BvhAabbBroadPhase3<String> bvh = new BvhAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(5, 1, 0, 6, 2, 1), "B"),
                new AabbEntry3<>(box(2, 1, 0, 3, 2, 1), "A")
        ));
        Ray ray = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));
        List<BroadPhaseRayHit3<String>> hits = new ArrayList<>();

        // WHEN
        bvh.queryRay(ray, 10.0, hits::add);

        // THEN
        assertEquals(2, hits.size());
        assertEquals("A", hits.get(0).value());
        assertEquals("B", hits.get(1).value());
        assertEquals(1.8, hits.get(0).tEnter(), 1e-9);
    }

    @Test
    void sphere_and_nearest_queries_are_supported() {
        // GIVEN
        BvhAabbBroadPhase3<String> bvh = new BvhAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(0, 0, 0, 1, 1, 1), "A"),
                new AabbEntry3<>(box(3, 0, 0, 4, 1, 1), "B")
        ));
        BvhAabbBroadPhase3<String> single = new BvhAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(3, 0, 0, 4, 1, 1), "B")
        ));
        List<String> sphereHits = new ArrayList<>();

        // WHEN
        bvh.querySphere(new Vector3(0.5, 0.5, 0.5), 0.5, sphereHits::add);
        BroadPhaseNearestHit3<String> nearest = bvh.nearest(new Vector3(2.8, 0.5, 0.5), 2.0);
        BroadPhaseNearestHit3<String> none = bvh.nearest(new Vector3(10, 10, 10), 1.0);
        BroadPhaseNearestHit3<String> edge = single.nearest(new Vector3(2.0, 0.5, 0.5), 1.0);

        // THEN
        assertEquals(List.of("A"), sphereHits);
        assertNotNull(nearest);
        assertEquals("B", nearest.value());
        assertNull(none);
        assertNotNull(edge);
        assertEquals("B", edge.value());
    }

    @Test
    void swept_aabb_hits_are_sorted() {
        // GIVEN
        BvhAabbBroadPhase3<String> bvh = new BvhAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(4, 0, 0, 5, 1, 1), "far"),
                new AabbEntry3<>(box(2, 0, 0, 3, 1, 1), "near")
        ));
        AxisAlignedBox moving = box(0, 0, 0, 1, 1, 1);
        Vector3 delta = new Vector3(5, 0, 0);
        List<BroadPhaseSweepHit3<String>> hits = new ArrayList<>();

        // WHEN
        bvh.querySweptAabb(moving, delta, hits::add);

        // THEN
        assertEquals(2, hits.size());
        assertEquals("near", hits.get(0).value());
        assertEquals("far", hits.get(1).value());
        assertEquals(0.2, hits.get(0).tEnter(), 1e-9);
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
