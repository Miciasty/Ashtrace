package nsk.nu.ashtrace.integration;

import nsk.nu.ashcore.api.collision.CollisionTests;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.OrientedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Quaternion;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashgrid.implementation.grid.indexing.SquareXZChunkScheme;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.grid.FrameGridSpaceMapper3;
import nsk.nu.ashspace.api.space.SpaceConverter3;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.contracts.MutableRayBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.ExactTraceHit3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameExactRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameOccludedExactRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicBvhBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** TRACE-012: compose packaged Ashcore geometry and Ashspace conversions with the existing pipelines. */
class OrientedBoxTraceIntegrationTest {
    // Absolute comparison tolerance for these fixtures, whose coordinates and lengths are below 30.
    private static final double TOLERANCE = 1e-12;
    private final FrameGraph3 frames = FrameGraph3.worldRoot();
    private final SpaceConverter3 converter = new SpaceConverter3(frames);
    private final AxisAlignedBox bar = box(-2, -0.5, -0.5, 2, 0.5, 0.5);

    @Test
    void rotated_shape_distances_and_candidate_only_misses_agree_across_indexes() {
        FrameId body = define("body", Math.PI / 4, new Vector3(6, 0, 0));
        OrientedBox shape = converter.orientedBox(bar, body, frames.root());
        var entry = new AabbEntry3<>(converter.axisAlignedBox(bar, body, frames.root()), shape);
        Ray ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        Ray emptyCorner = new Ray(new Vector3(7.5, -1.5, -2), new Vector3(0, 0, 1));
        List<BroadPhaseRayHit3<OrientedBox>> reference = null;
        for (var index : indexes(List.of(entry))) {
            var candidates = new ArrayList<BroadPhaseRayHit3<OrientedBox>>();
            index.queryRay(ray, 12, candidates::add);
            if (reference == null) reference = candidates;
            else assertEquals(reference, candidates);
            var tracer = new FrameExactRayTracer3<>(frames, index);
            var hit = tracer.firstHit(frames.root(), ray, 12, intervals());
            assertInterval(hit, 6 - Math.sqrt(0.5), 6 + Math.sqrt(0.5), true, true);
            assertEquals(hit, tracer.lastHit(frames.root(), ray, 12, intervals()));
            assertEquals(List.of(hit), tracer.allHits(frames.root(), ray, 12, intervals()));
            assertTrue(tracer.anyHit(frames.root(), ray, 12, intervals()));
            assertTrue(candidates.getFirst().tEnter() < hit.tEnter());
            assertTrue(candidates.getFirst().tExit() > hit.tExit());
            assertTrue(index.anyRay(emptyCorner, 4));
            assertNull(tracer.firstHit(frames.root(), emptyCorner, 4, intervals()));
            assertFalse(tracer.anySegmentHit(frames.root(), new Segment3(emptyCorner.origin(), emptyCorner.at(4)), intervals()));
            assertNotNull(new FrameBroadPhaseRayTracer3<>(frames, index).firstHit(
                    frames.root(), emptyCorner, 4, (value, worldRay, enter, exit) -> true));
        }
    }

    @Test
    void nested_frames_preserve_world_units_inside_origins_and_segment_clipping() {
        FrameId parent = define("parent", Math.PI / 4, new Vector3(10, -3, 2));
        FrameId body = new FrameId("body");
        frames.define(body, parent, RigidTransform3.translation(2, 1, 0));
        OrientedBox shape = converter.orientedBox(bar, body, frames.root());
        var entry = new AabbEntry3<>(converter.axisAlignedBox(bar, body, frames.root()), shape);
        Ray ray = new Ray(new Vector3(-3, 0, 0), new Vector3(100, 0, 0));
        for (var index : indexes(List.of(entry))) {
            var tracer = new FrameExactRayTracer3<>(frames.snapshot(), index);
            var hit = tracer.firstHit(body, ray, 10, intervals());
            assertInterval(hit, 1, 5, true, true);
            assertPoint(converter.point(new Vector3(-2, 0, 0), body, frames.root()), hit.worldEnterPoint());
            assertPoint(converter.point(new Vector3(2, 0, 0), body, frames.root()), hit.worldExitPoint());
            assertInterval(tracer.firstHit(body, new Ray(Vector3.ZERO, ray.direction()), 10, intervals()), 0, 2, false, true);
            assertInterval(tracer.firstHit(body, new Ray(Vector3.ZERO, ray.direction()), 0, intervals()), 0, 0, false, false);
            assertNull(tracer.firstHit(body, ray, 0.5, intervals()));
            Segment3 partial = new Segment3(ray.origin(), Vector3.ZERO);
            assertInterval(tracer.firstSegmentHit(body, partial, intervals()), 1, 3, true, false);
            assertInterval(tracer.lastSegmentHit(body, partial, intervals()), 1, 3, true, false);
            assertEquals(1, tracer.allSegmentHits(body, partial, intervals()).size());
            var coreSegment = CollisionTests.segmentVsOrientedBoxInterval(
                    converter.segment(partial, body, frames.root()), shape);
            assertEquals(coreSegment.tEnter() * 3, tracer.firstSegmentHit(body, partial, intervals()).tEnter(), TOLERANCE);
            assertInterval(tracer.firstHit(body, new Ray(new Vector3(3, 0, 0), new Vector3(-1, 0, 0)), 10, intervals()),
                    1, 5, true, true);
        }
    }

    @Test
    void closed_tangency_and_range_endpoint_use_shape_boundaries() {
        FrameId body = new FrameId("cycle");
        // Exact cyclic axis permutation avoids making a trigonometric near-tangent a geometric oracle.
        frames.define(body, frames.root(), new RigidTransform3(new Quaternion(0.5, 0.5, 0.5, 0.5), Vector3.ZERO));
        AxisAlignedBox cube = box(-1, -1, -1, 1, 1, 1);
        OrientedBox shape = converter.orientedBox(cube, body, frames.root());
        for (var index : indexes(List.of(new AabbEntry3<>(converter.axisAlignedBox(cube, body, frames.root()), shape)))) {
            var tracer = new FrameExactRayTracer3<>(frames, index);
            Ray tangent = new Ray(new Vector3(-2, 0, 0), new Vector3(1, 1, 0));
            assertInterval(tracer.firstHit(body, tangent, 4, intervals()), Math.sqrt(2), Math.sqrt(2), true, true);
            assertNull(tracer.firstHit(body, new Ray(new Vector3(-2, 0.01, 0), tangent.direction()), 4, intervals()));
            Ray face = new Ray(new Vector3(-2, 0, 0), new Vector3(1, 0, 0));
            assertInterval(tracer.firstHit(body, face, 1, intervals()), 1, 1, true, false);
            assertInterval(tracer.firstSegmentHit(body, new Segment3(face.origin(), face.at(1)), intervals()), 1, 1, true, false);
        }
    }

    @Test
    void shape_sorting_can_reverse_bounds_order_and_full_ties_follow_each_index_visit_order() {
        FrameId body = define("body", Math.PI / 4, new Vector3(6, 0, 0));
        OrientedBox far = converter.orientedBox(bar, body, frames.root());
        OrientedBox near = new OrientedBox(new Vector3(3, 0, 0), new Vector3(0.5, 0.5, 0.5), far.orientation());
        var entries = List.of(new AabbEntry3<>(box(0, -3, -3, 9, 3, 3), far),
                new AabbEntry3<>(box(2, -1, -1, 4, 1, 1), near));
        Ray ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        for (var index : indexes(entries)) {
            var tracer = new FrameExactRayTracer3<>(frames, index);
            assertEquals(near, tracer.firstHit(frames.root(), ray, 10, intervals()).value());
            assertEquals(far, tracer.lastHit(frames.root(), ray, 10, intervals()).value());
            assertEquals(List.of(near, far), tracer.allHits(frames.root(), ray, 10, intervals()).stream().map(ExactTraceHit3::value).toList());
        }
        var bounds = converter.axisAlignedBox(bar, body, frames.root());
        var ties = List.of(new AabbEntry3<>(bounds, "a"), new AabbEntry3<>(bounds, "b"), new AabbEntry3<>(bounds, "c"));
        RayIntersector3<String> provider = (value, worldRay, min, max, output) -> intervals().intersect(far, worldRay, min, max, output);
        for (var index : indexes(ties)) {
            var order = new ArrayList<String>();
            index.visitRay(ray, 10, (value, enter, exit) -> { order.add(value); return true; });
            var tracer = new FrameExactRayTracer3<>(frames, index);
            var result = tracer.allHits(frames.root(), ray, 10, provider);
            assertEquals(order, result.stream().map(ExactTraceHit3::value).toList());
            assertEquals(order.getFirst(), tracer.firstHit(frames.root(), ray, 10, provider).value());
            assertEquals(order.getFirst(), tracer.lastHit(frames.root(), ray, 10, provider).value());
            assertEquals(result, tracer.allHits(frames.root(), ray, 10, provider));
        }
    }

    @Test
    void multipart_union_keeps_cavities_and_mapped_voxel_clipping_in_world_units() {
        FrameId body = define("parts", Math.PI / 4, new Vector3(10, -3, 2));
        var parts = List.of(box(7, -0.5, -0.5, 9, 0.5, 0.5), box(3, -0.5, -0.5, 5, 0.5, 0.5),
                box(2, -0.5, -0.5, 4, 0.5, 0.5)).stream().map(b -> converter.orientedBox(b, body, frames.root())).toList();
        var bounds = converter.axisAlignedBox(box(1, -1, -1, 10, 1, 1), body, frames.root());
        RayIntersector3<List<OrientedBox>> provider = unionIntervals();
        var grid = new FrameGridSpaceMapper3(frames, body, 2, Vector3.ZERO, new SquareXZChunkScheme(16));
        VoxelTraverser dda = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        Ray ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        for (var index : indexes(List.of(new AabbEntry3<>(bounds, parts)))) {
            var tracer = new FrameExactRayTracer3<>(frames, index);
            var hits = tracer.allHits(body, ray, 12, provider);
            assertEquals(2, hits.size());
            assertInterval(hits.getFirst(), 2, 5, true, true);
            assertInterval(hits.getLast(), 7, 9, true, true);
            assertEquals(hits.getFirst(), tracer.firstHit(body, ray, 12, provider));
            assertEquals(hits.getLast(), tracer.lastHit(body, ray, 12, provider));
            assertFalse(tracer.anyHit(body, new Ray(new Vector3(6, 0, 0), ray.direction()), 0.5, provider));
            assertThrows(UnsupportedOperationException.class, hits::clear);
            var occluded = FrameOccludedExactRayTracer3.forGrid(grid, index, dda);
            var visible = occluded.visibleHits(body, ray, 12, provider, (x, y, z) -> x == 2);
            assertEquals(1, visible.size());
            assertInterval(visible.getFirst(), 2, 4, true, false);
            assertPoint(converter.point(new Vector3(4, 0, 0), body, frames.root()), visible.getFirst().worldExitPoint());
            assertEquals(visible.getFirst(), occluded.firstVisibleHit(body, ray, 12, provider, (x, y, z) -> x == 2));
            assertEquals(visible.getFirst(), occluded.lastVisibleSegmentHit(body, new Segment3(Vector3.ZERO, ray.at(12)),
                    provider, (x, y, z) -> x == 2));
            assertFalse(occluded.anyVisibleHit(body, ray, 12, provider, (x, y, z) -> x == 0));
            // A wall in the cavity leaves the first material interval intact and excludes the second.
            assertInterval(occluded.lastVisibleHit(body, ray, 12, provider, (x, y, z) -> x == 3), 2, 5, true, true);
        }
    }

    @Test
    void coherent_pose_updates_refresh_all_indexes_and_preserve_previous_result_points() {
        FrameId body = define("moving", Math.PI / 4, new Vector3(6, 0, 0));
        Ray ray = new Ray(Vector3.ZERO, new Vector3(1, 0, 0));
        RayIntersector3<FrameId> provider = (value, worldRay, min, max, output) ->
                intervals().intersect(converter.orientedBox(bar, value, frames.root()), worldRay, min, max, output);
        var oldBounds = converter.axisAlignedBox(bar, body, frames.root());
        for (var index : indexes(List.of(new AabbEntry3<>(oldBounds, body)))) {
            frames.define(body, frames.root(), pose(Math.PI / 4, new Vector3(6, 0, 0)));
            var before = new FrameExactRayTracer3<>(frames, index).firstHit(frames.root(), ray, 20, provider);
            Vector3 saved = before.worldEnterPoint();
            frames.define(body, frames.root(), pose(Math.PI / 2, new Vector3(10, 0, 0)));
            AxisAlignedBox updated = converter.axisAlignedBox(bar, body, frames.root());
            RayQueryableBroadPhase3<FrameId> refreshed;
            if (index instanceof MutableRayBroadPhase3<FrameId> mutable) {
                assertTrue(mutable.updateBounds(1, updated));
                refreshed = mutable;
            } else if (index instanceof LinearAabbBroadPhase3<FrameId>) {
                refreshed = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(updated, body)));
            } else {
                refreshed = new BvhAabbBroadPhase3<>(List.of(new AabbEntry3<>(updated, body)));
            }
            var after = new FrameExactRayTracer3<>(frames, refreshed).firstHit(frames.root(), ray, 20, provider);
            assertInterval(after, 9.5, 10.5, true, true);
            assertEquals(saved, before.worldEnterPoint());
            assertEquals(6 - Math.sqrt(0.5), before.tEnter(), TOLERANCE);
            assertTrue(after.tEnter() > before.tExit());
        }
    }

    @Test
    void endpoint_bounds_do_not_cover_a_rotating_bar_between_poses() {
        FrameId body = define("rotating", 0, Vector3.ZERO);
        AxisAlignedBox thinBar = box(-2, -0.25, -0.25, 2, 0.25, 0.25);
        AxisAlignedBox start = converter.axisAlignedBox(thinBar, body, frames.root());
        frames.define(body, frames.root(), new RigidTransform3(new Quaternion(0, 0, 0, 1), Vector3.ZERO));
        AxisAlignedBox end = converter.axisAlignedBox(thinBar, body, frames.root());
        assertEquals(start, end);
        Ray crossing = new Ray(new Vector3(0, 1.5, -2), new Vector3(0, 0, 1));
        for (var index : indexes(List.of(new AabbEntry3<>(start, "endpoints")))) assertFalse(index.anyRay(crossing, 4));
        frames.define(body, frames.root(), pose(Math.PI / 2, Vector3.ZERO));
        var midpoint = converter.orientedBox(thinBar, body, frames.root());
        var middleBounds = converter.axisAlignedBox(thinBar, body, frames.root());
        for (var index : indexes(List.of(new AabbEntry3<>(middleBounds, midpoint)))) {
            var hit = new FrameExactRayTracer3<>(frames, index).firstHit(frames.root(), crossing, 4, intervals());
            assertInterval(hit, 1.75, 2.25, true, true);
        }
    }

    private FrameId define(String name, double angle, Vector3 translation) {
        FrameId id = new FrameId(name);
        frames.define(id, frames.root(), pose(angle, translation));
        return id;
    }

    private static RigidTransform3 pose(double angle, Vector3 translation) {
        return new RigidTransform3(Quaternion.fromAxisAngle(new Vector3(0, 0, 1), angle), translation);
    }

    private static RayIntersector3<OrientedBox> intervals() {
        return (shape, ray, min, max, output) -> {
            var interval = CollisionTests.rayVsOrientedBoxInterval(ray, shape);
            if (interval.hit() && interval.tExit() >= min && interval.tEnter() <= max) {
                output.accept(new RayIntersection3(interval.tEnter(), interval.tExit()));
            }
        };
    }

    /** The provider owns the union, including merging overlaps without filling empty gaps. */
    private static RayIntersector3<List<OrientedBox>> unionIntervals() {
        return (parts, ray, min, max, output) -> {
            var intervals = new ArrayList<RayIntersection3>();
            for (OrientedBox part : parts) intervals().intersect(part, ray, min, max, intervals::add);
            intervals.sort(Comparator.comparingDouble(RayIntersection3::tEnter).thenComparingDouble(RayIntersection3::tExit));
            RayIntersection3 current = null;
            for (RayIntersection3 next : intervals) {
                if (current == null) current = next;
                else if (next.tEnter() <= current.tExit()) current = new RayIntersection3(current.tEnter(), Math.max(current.tExit(), next.tExit()));
                else { output.accept(current); current = next; }
            }
            if (current != null) output.accept(current);
        };
    }

    private static <T> List<RayQueryableBroadPhase3<T>> indexes(List<AabbEntry3<T>> entries) {
        var hash = new DynamicSpatialHashBroadPhase3<T>(4);
        var dynamic = new DynamicBvhBroadPhase3<T>();
        for (var entry : entries) {
            hash.insert(entry.bounds(), entry.value());
            dynamic.insert(entry.bounds(), entry.value());
        }
        return List.of(new LinearAabbBroadPhase3<>(entries), new BvhAabbBroadPhase3<>(entries), hash, dynamic);
    }

    private static AxisAlignedBox box(double x1, double y1, double z1, double x2, double y2, double z2) {
        return new AxisAlignedBox(new Vector3(x1, y1, z1), new Vector3(x2, y2, z2));
    }

    private static void assertInterval(ExactTraceHit3<?> hit, double enter, double exit, boolean enterSurface, boolean exitSurface) {
        assertNotNull(hit);
        assertEquals(enter, hit.tEnter(), TOLERANCE);
        assertEquals(exit, hit.tExit(), TOLERANCE);
        assertEquals(enterSurface, hit.enterSurface());
        assertEquals(exitSurface, hit.exitSurface());
    }

    private static void assertPoint(Vector3 expected, Vector3 actual) {
        assertEquals(expected.x(), actual.x(), TOLERANCE);
        assertEquals(expected.y(), actual.y(), TOLERANCE);
        assertEquals(expected.z(), actual.z(), TOLERANCE);
    }
}
