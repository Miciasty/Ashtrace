package nsk.nu.ashtrace.unit.api.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.contracts.BroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.ProximityQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.SweepQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicBvhBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

class BroadPhaseAgreementTest {

    @Test
    void nearest_rejects_leaf_entries_outside_limit_even_when_parent_bound_contains_point() {
        var entries = List.of(new AabbEntry3<>(box(-2, -1), 1), new AabbEntry3<>(box(1, 2), 2));
        for (var index : indexes(entries, 2)) {
            var proximity = (ProximityQueryableBroadPhase3<Integer>) index;
            assertNull(proximity.nearest(new Vector3(0, 0, 0), 0.5), index.getClass().getSimpleName());
            assertEquals(1, proximity.nearest(new Vector3(0, 0, 0), 1).distanceSquared());
        }
    }

    @Test
    void small_nonzero_ray_component_reaches_distant_bound() {
        var target = new Vector3(1e14, 1, 0);
        var entries = List.of(new AabbEntry3<>(new AxisAlignedBox(target, target), 1));
        for (var index : indexes(entries, 1e14)) {
            var hits = new ArrayList<BroadPhaseRayHit3<Integer>>();
            ((RayQueryableBroadPhase3<Integer>) index).queryRay(
                    new Ray(new Vector3(0, 0, 0), new Vector3(1, 1e-14, 0)), 1e14, hits::add);
            assertEquals(List.of(new BroadPhaseRayHit3<>(1, 1e14, 1e14)), hits);
        }
    }

    @Test
    void small_nonzero_sweep_motion_reaches_nearby_bound() {
        var target = new Vector3(5e-15, 0, 0);
        var entries = List.of(new AabbEntry3<>(new AxisAlignedBox(target, target), 1));
        var zero = new Vector3(0, 0, 0);
        for (var index : indexes(entries, 2)) {
            var hits = new ArrayList<BroadPhaseSweepHit3<Integer>>();
            ((SweepQueryableBroadPhase3<Integer>) index).querySweptAabb(
                    new AxisAlignedBox(zero, zero), new Vector3(1e-14, 0, 0), hits::add);
            assertEquals(List.of(new BroadPhaseSweepHit3<>(1, 0.5, 0.5)), hits);
        }
    }

    @Test
    void subnormal_ray_component_leaves_a_zero_width_slab() {
        var target = new Vector3(1, 0, 0);
        var entries = List.of(new AabbEntry3<>(new AxisAlignedBox(target, target), 1));
        for (var index : indexes(entries, 2)) {
            var hits = new ArrayList<BroadPhaseRayHit3<Integer>>();
            ((RayQueryableBroadPhase3<Integer>) index).queryRay(
                    new Ray(new Vector3(0, 0, 0), new Vector3(1, Double.MIN_VALUE, 0)), 2, hits::add);
            assertTrue(hits.isEmpty());
        }
    }

    @Test
    void nonfinite_query_bounds_are_rejected_even_by_empty_indexes() {
        var invalid = new AxisAlignedBox(new Vector3(Double.NaN, 0, 0), new Vector3(Double.NaN, 1, 1));
        for (var index : indexes(List.of(), 2)) {
            assertThrows(IllegalArgumentException.class, () -> index.query(invalid, v -> fail("Invalid query emitted a value")));
            assertThrows(IllegalArgumentException.class, () -> ((SweepQueryableBroadPhase3<Integer>) index)
                    .querySweptAabb(invalid, new Vector3(0, 0, 0), v -> fail("Invalid sweep emitted a value")));
        }
    }

    @Test
    void indexes_agree_on_candidate_sets_intervals_and_nearest_distance() {
        Random random = new Random(7319);
        var entries = new ArrayList<AabbEntry3<Integer>>();
        for (int i = 0; i < 40; i++) {
            double x = random.nextInt(20) - 10;
            entries.add(new AabbEntry3<>(box(x, x + 0.5 + random.nextInt(3)), i));
        }
        var indexes = indexes(entries, 2);
        for (int i = 0; i < 30; i++) {
            double x = random.nextInt(24) - 12;
            var expected = results(indexes.getFirst(), x);
            for (var index : indexes) assertEquals(expected, results(index, x), index.getClass().getSimpleName());
        }
    }

    @Test
    void empty_indexes_agree() {
        for (var index : indexes(List.of(), 2)) {
            assertEquals(List.of(new HashSet<>(), new HashSet<>(), new HashSet<>(), new HashSet<>(), -1.0), results(index, 0));
        }
    }

    private static List<Object> results(BroadPhase3<Integer> index, double x) {
        var boxes = new ArrayList<Integer>();
        var rays = new ArrayList<BroadPhaseRayHit3<Integer>>();
        var spheres = new ArrayList<Integer>();
        var sweeps = new ArrayList<BroadPhaseSweepHit3<Integer>>();
        index.query(box(x, x + 3), boxes::add);
        ((RayQueryableBroadPhase3<Integer>) index).queryRay(
                new Ray(new Vector3(x, 0.5, 0.5), new Vector3(-1, 0, 0)), 8, rays::add);
        var proximity = (ProximityQueryableBroadPhase3<Integer>) index;
        proximity.querySphere(new Vector3(x, 0.5, 0.5), 3, spheres::add);
        ((SweepQueryableBroadPhase3<Integer>) index).querySweptAabb(box(x, x + 1), new Vector3(5, 0, 0), sweeps::add);
        var nearest = proximity.nearest(new Vector3(x, 0.5, 0.5), 0.75);
        assertEquals(new HashSet<>(boxes).size(), boxes.size());
        assertEquals(new HashSet<>(rays).size(), rays.size());
        assertEquals(new HashSet<>(spheres).size(), spheres.size());
        assertEquals(new HashSet<>(sweeps).size(), sweeps.size());
        return List.of(new HashSet<>(boxes), new HashSet<>(rays), new HashSet<>(spheres), new HashSet<>(sweeps),
                nearest == null ? -1.0 : nearest.distanceSquared());
    }

    private static List<BroadPhase3<Integer>> indexes(List<AabbEntry3<Integer>> entries, double cellSize) {
        var hash = new DynamicSpatialHashBroadPhase3<Integer>(cellSize);
        var dynamic = new DynamicBvhBroadPhase3<Integer>();
        for (var entry : entries) {
            hash.insert(entry.bounds(), entry.value());
            dynamic.insert(entry.bounds(), entry.value());
        }
        return List.of(new LinearAabbBroadPhase3<>(entries), new BvhAabbBroadPhase3<>(entries), hash, dynamic);
    }

    private static AxisAlignedBox box(double minX, double maxX) {
        return new AxisAlignedBox(new Vector3(minX, 0, 0), new Vector3(maxX, 1, 1));
    }
}
