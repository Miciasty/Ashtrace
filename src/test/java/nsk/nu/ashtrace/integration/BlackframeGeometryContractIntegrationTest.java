package nsk.nu.ashtrace.integration;

import nsk.nu.ashcore.api.collision.CollisionTests;
import nsk.nu.ashcore.api.collision.SweptAABB;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Sphere;
import nsk.nu.ashcore.api.math.Quaternion;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.space.SpaceConverter3;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.contracts.ProximityQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.SweepQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameExactRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicBvhBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Composes Ashcore collision fixtures and Ashspace frame fixtures with all Ashtrace indexes. */
class BlackframeGeometryContractIntegrationTest {
    private final FrameGraph3 frames = FrameGraph3.worldRoot();

    @Test
    void all_indexes_agree_with_ashcore_box_entry_and_miss_results() {
        var box = new AxisAlignedBox(Vector3.ZERO, new Vector3(1, 1, 1));
        for (Ray ray : List.of(
                new Ray(new Vector3(-2, 0.5, 0.5), new Vector3(1, 0, 0)),
                new Ray(new Vector3(0.5, 0.5, 0.5), new Vector3(1, 0, 0)),
                new Ray(new Vector3(2, 0.5, 0.5), new Vector3(-1, 0, 0)),
                new Ray(new Vector3(-1, -1, 0), new Vector3(1, 1, 0)),
                new Ray(new Vector3(-1, 2, 0), new Vector3(1, 0, 0)),
                new Ray(Vector3.ZERO, new Vector3(-1, 0, 0)))) {
            assertRayAgreement(box, ray, 10);
        }
        // Ashcore CollisionBoundaryTest's small nonzero component; coarse hash cells keep this query bounded.
        assertRayAgreement(new AxisAlignedBox(Vector3.ZERO, new Vector3(1, 1e13, 1)),
                new Ray(new Vector3(-1, -1e13, 0.5), new Vector3(1e-13, 1, 0)), 2e13);
        // Ashcore NormalizationTest's input scales must not alter world-distance hit parameters.
        for (double scale : new double[]{Double.MIN_VALUE, 1e-300, 1e308, Double.MAX_VALUE}) {
            assertRayAgreement(box, new Ray(new Vector3(-2, 0.5, 0.5), new Vector3(scale, 0, 0)), 10);
        }
    }

    @Test
    void sweep_times_and_sphere_overlap_agree_with_ashcore_closed_contacts() {
        var box = new AxisAlignedBox(Vector3.ZERO, new Vector3(1, 1, 1));
        var moving = new AxisAlignedBox(new Vector3(-2, 0, 0), new Vector3(-1, 1, 1));
        for (var index : indexes(box, "box", 2)) {
            for (Vector3 delta : List.of(new Vector3(1, 0, 0), new Vector3(2, 0, 0),
                    new Vector3(-1, 0, 0), Vector3.ZERO)) {
                var actual = new ArrayList<BroadPhaseSweepHit3<String>>();
                index.sweep().querySweptAabb(moving, delta, actual::add);
                var expected = SweptAABB.test(moving, delta, box, 1);
                assertEquals(expected.hit(), !actual.isEmpty());
                if (expected.hit()) assertEquals(expected.t(), actual.getFirst().tEnter(), 1e-12);
            }
            var atRest = new ArrayList<BroadPhaseSweepHit3<String>>();
            index.sweep().querySweptAabb(box, Vector3.ZERO, atRest::add);
            assertEquals(0, atRest.getFirst().tEnter());
            for (Sphere sphere : List.of(new Sphere(new Vector3(-1, 0.5, 0.5), 1),
                    new Sphere(new Vector3(-1.01, 0.5, 0.5), 1), new Sphere(new Vector3(-3, -4, 0), 5),
                    new Sphere(new Vector3(-3, -4, 0), 4.99), new Sphere(new Vector3(0.5, 0.5, 0.5), 0))) {
                var actual = new ArrayList<String>();
                index.proximity().querySphere(sphere.center(), sphere.radius(), actual::add);
                assertEquals(CollisionTests.sphereVsBox(sphere, box), !actual.isEmpty());
            }
        }
    }

    @Test
    void ashcore_sphere_queries_supply_both_endpoints_including_inside_and_tangent_cases() {
        Sphere sphere = new Sphere(Vector3.ZERO, 1);
        var bounds = new AxisAlignedBox(new Vector3(-2, -2, -2), new Vector3(2, 2, 2));
        for (var index : indexes(bounds, sphere, 2)) {
            var tracer = new FrameExactRayTracer3<>(frames, index.ray());
            Ray incoming = new Ray(new Vector3(-2, 0, 0), new Vector3(2, 0, 0));
            var hit = tracer.firstHit(frames.root(), incoming, 10, sphereIntervals());
            // Like Ashcore's primitive tests, use an absolute tolerance for these unit-scale fixtures.
            assertEquals(CollisionTests.rayVsSphereT(incoming, sphere), hit.tEnter(), 1e-12);
            assertEquals(1, hit.tEnter(), 1e-12);
            assertEquals(3, hit.tExit(), 1e-12);
            assertEquals(-1, hit.worldEnterPoint().x(), 1e-12);
            assertEquals(1, hit.worldExitPoint().x(), 1e-12);
            assertEquals(0, hit.worldEnterPoint().y());
            assertEquals(0, hit.worldExitPoint().z());
            assertTrue(hit.enterSurface());
            assertTrue(hit.exitSurface());
            var inside = tracer.firstHit(frames.root(), new Ray(Vector3.ZERO, incoming.direction()), 10, sphereIntervals());
            assertEquals(0, inside.tEnter());
            assertEquals(1, inside.tExit(), 1e-12);
            assertFalse(inside.enterSurface());
            assertTrue(inside.exitSurface());
            var partial = tracer.firstHit(frames.root(), incoming, 2, sphereIntervals());
            assertEquals(2, partial.tExit());
            assertFalse(partial.exitSurface());
            var tangent = tracer.firstHit(frames.root(), new Ray(new Vector3(-2, 1, 0), incoming.direction()), 10, sphereIntervals());
            assertEquals(2, tangent.tEnter(), 1e-12);
            assertEquals(2, tangent.tExit(), 1e-12);
            assertNull(tracer.firstHit(frames.root(), new Ray(new Vector3(-2, Math.nextUp(1.0), 0), incoming.direction()),
                    10, sphereIntervals()));
        }
    }

    @Test
    void chained_rotated_frames_and_core_geometry_produce_world_surface_points() {
        var ship = new FrameId("ship");
        var turret = new FrameId("turret");
        frames.define(ship, frames.root(), new RigidTransform3(
                Quaternion.fromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2), new Vector3(10, 0, -4)));
        frames.define(turret, ship, RigidTransform3.translation(0, 2, 0));
        var converter = new SpaceConverter3(frames);
        Sphere sphere = converter.sphere(new Sphere(new Vector3(5, 0, 0), 1), turret, frames.root());
        AxisAlignedBox bounds = converter.axisAlignedBox(new AxisAlignedBox(new Vector3(3, -2, -2),
                new Vector3(7, 2, 2)), turret, frames.root());
        Ray ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        for (var index : indexes(bounds, sphere, 4)) {
            var hit = new FrameExactRayTracer3<>(frames.snapshot(), index.ray()).firstHit(turret, ray, 10, sphereIntervals());
            assertEquals(4, hit.tEnter(), 1e-12);
            assertEquals(6, hit.tExit(), 1e-12);
            assertEquals(10, hit.worldEnterPoint().x(), 1e-12);
            assertEquals(2, hit.worldEnterPoint().y(), 1e-12);
            assertEquals(-8, hit.worldEnterPoint().z(), 1e-12);
            assertEquals(-10, hit.worldExitPoint().z(), 1e-12);
        }
    }

    private void assertRayAgreement(AxisAlignedBox box, Ray ray, double max) {
        double expected = CollisionTests.rayVsBoxT(ray, box);
        for (var index : indexes(box, "box", Math.max(2, max))) {
            var hits = new ArrayList<BroadPhaseRayHit3<String>>();
            index.ray().queryRay(ray, max, hits::add);
            assertEquals(Double.isFinite(expected) && expected <= max, !hits.isEmpty());
            if (!hits.isEmpty()) assertEquals(expected, hits.getFirst().tEnter(), Math.max(1e-12, Math.ulp(expected) * 4));
        }
    }

    /** Test adapter for these bounded fixtures: query Ashcore from outside the sphere in both directions. */
    private static RayIntersector3<Sphere> sphereIntervals() {
        return (sphere, ray, min, max, output) -> {
            double projection = sphere.center().sub(ray.origin()).dot(ray.direction());
            double before = projection - sphere.radius() - 1;
            double after = projection + sphere.radius() + 1;
            double enter = CollisionTests.rayVsSphereT(new Ray(ray.at(before), ray.direction()), sphere);
            double exit = CollisionTests.rayVsSphereT(new Ray(ray.at(after), ray.direction().mul(-1)), sphere);
            if (Double.isFinite(enter) && Double.isFinite(exit)) output.accept(new RayIntersection3(before + enter, after - exit));
        };
    }

    private static <T> List<Index<T>> indexes(AxisAlignedBox bounds, T value, double cellSize) {
        var entry = new AabbEntry3<>(bounds, value);
        var linear = new LinearAabbBroadPhase3<>(List.of(entry));
        var bvh = new BvhAabbBroadPhase3<>(List.of(entry));
        var hash = new DynamicSpatialHashBroadPhase3<T>(cellSize);
        var dynamic = new DynamicBvhBroadPhase3<T>();
        hash.insert(bounds, value);
        dynamic.insert(bounds, value);
        return List.of(new Index<>(linear, linear, linear), new Index<>(bvh, bvh, bvh),
                new Index<>(hash, hash, hash), new Index<>(dynamic, dynamic, dynamic));
    }

    private record Index<T>(RayQueryableBroadPhase3<T> ray, SweepQueryableBroadPhase3<T> sweep,
                            ProximityQueryableBroadPhase3<T> proximity) {}
}
