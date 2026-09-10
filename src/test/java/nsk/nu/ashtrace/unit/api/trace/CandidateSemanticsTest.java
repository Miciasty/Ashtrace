package nsk.nu.ashtrace.unit.api.trace;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.math.Quaternion;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.model.TraceHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameOccludedBroadPhaseRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.fail;

class CandidateSemanticsTest {

    @Test
    void accepted_bounds_order_does_not_order_enclosed_surfaces() {
        // GIVEN: each payload models a surface perpendicular to X inside a loose AABB.
        FrameGraph3 frames = FrameGraph3.worldRoot();
        var index = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(1, 10), 9.0),
                new AabbEntry3<>(box(3, 5), 4.0)
        ));
        var tracer = new FrameBroadPhaseRayTracer3<>(frames, index);
        Ray ray = ray(0, 1);
        FrameBroadPhaseRayTracer3.NarrowPhase3<Double> intersects =
                (surface, worldRay, enter, exit) -> surface >= enter && surface <= exit;

        // WHEN
        var first = tracer.firstHit(frames.root(), ray, 12, intersects);
        var hits = tracer.allHits(frames.root(), ray, 12, intersects);

        // THEN: B has the nearer surface, but A has the earlier accepted bounds.
        assertEquals(9.0, first.value());
        assertEquals(1.0, first.tEnter());
        assertEquals(new Vector3(1, 0.5, 0.5), first.worldPoint());
        assertEquals(List.of(9.0, 4.0), hits.stream().map(TraceHit3::value).toList());
        assertEquals(4.0, tracer.firstHit(frames.root(), ray, 12,
                (surface, worldRay, enter, exit) -> surface == 4.0).value());
        assertNull(tracer.firstHit(frames.root(), ray, 12, (v, r, a, b) -> false));
        assertThrows(UnsupportedOperationException.class, () -> hits.clear());
    }

    @Test
    void accepted_bounds_before_wall_do_not_prove_surface_visibility() {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        var tracer = occluded(frames, new AabbEntry3<>(box(1, 10), "surface at x=9"));
        var accepted = tracer.firstVisibleHit(frames.root(), ray(0, 1), 12,
                (v, r, a, b) -> true, (x, y, z) -> x == 5);
        assertNotNull(accepted);
        assertEquals(1, accepted.tEnter());
        assertEquals(5, accepted.tExit());
        assertNull(tracer.firstVisibleHit(frames.root(), ray(0, 1), 12,
                (v, r, a, b) -> 9 >= a && 9 <= b, (x, y, z) -> x == 5));
    }

    @Test
    void wall_contact_is_included_for_both_ray_directions() {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        var positive = occluded(frames, new AabbEntry3<>(box(5, 6), "contact"));
        var hit = positive.firstVisibleHit(frames.root(), ray(0.5, 1), 10,
                (v, r, a, b) -> true, (x, y, z) -> x == 5);
        assertEquals(4.5, hit.tEnter());
        assertEquals(4.5, hit.tExit());

        var negative = occluded(frames, new AabbEntry3<>(box(2, 3), "contact"));
        var reverse = negative.firstVisibleHit(frames.root(), ray(5.5, -1), 10,
                (v, r, a, b) -> true, (x, y, z) -> x == 2);
        assertEquals(2.5, reverse.tEnter());
        assertEquals(2.5, reverse.tExit());
        assertEquals(3, reverse.worldPoint().x());
    }

    @Test
    void start_inside_wall_clips_to_origin_and_zero_limit_does_not_visit_voxels() {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        var tracer = occluded(frames, new AabbEntry3<>(box(0, 1), "origin"));
        var hit = tracer.firstVisibleHit(frames.root(), ray(0.5, 1), 10,
                (v, r, a, b) -> true, (x, y, z) -> true);
        assertEquals(0, hit.tEnter());
        assertEquals(0, hit.tExit());
        assertEquals(hit, tracer.firstVisibleHit(frames.root(), ray(0.5, 1), 0,
                (v, r, a, b) -> true, (x, y, z) -> {
                    fail("An empty voxel interval must not call occupancy");
                    return true;
                }));
        assertNull(occluded(frames, new AabbEntry3<>(box(2, 3), "beyond origin"))
                .firstVisibleHit(frames.root(), ray(0.5, 1), 10,
                        (v, r, a, b) -> true, (x, y, z) -> true));
    }

    @Test
    void voxel_endpoint_is_excluded_while_object_endpoint_is_included() {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        var grid = new FrameGridRayTracer3(frames, dda());
        assertNull(grid.firstHit(frames.root(), ray(0.5, 1), 0.5, (x, y, z) -> x == 1));
        var tracer = occluded(frames, new AabbEntry3<>(box(1, 2), "endpoint"));
        var calls = new ArrayList<Integer>();
        var hit = tracer.firstVisibleHit(frames.root(), ray(0.5, 1), 0.5,
                (v, r, a, b) -> true, (x, y, z) -> { calls.add(x); return x == 1; });
        assertEquals(List.of(0), calls);
        assertEquals(0.5, hit.tEnter());
        assertEquals(0.5, hit.tExit());
        Segment3 point = new Segment3(new Vector3(0, 0, 0), new Vector3(0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> tracer.firstVisibleSegmentHit(
                frames.root(), point, (v, r, a, b) -> true, (x, y, z) -> false));
    }

    @Test
    void translated_frame_preserves_distance_and_world_unit_cells_without_occlusion() {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId tool = new FrameId("tool");
        frames.define(tool, frames.root(), RigidTransform3.translation(10, 0, 0));
        var tracer = occluded(frames, new AabbEntry3<>(box(12, 13), "target"));
        var hit = tracer.firstVisibleHit(tool, ray(0.5, 7), 10,
                (v, r, a, b) -> true, (x, y, z) -> false);
        assertEquals(1.5, hit.tEnter());
        assertEquals(12, hit.worldPoint().x());
        var gridHit = tracer.voxelTracer().firstHit(tool, ray(0.5, 7), 10, (x, y, z) -> x == 12);
        assertEquals(hit.tEnter(), gridHit.tEnter());
    }

    @Test
    void frozen_rotated_frame_keeps_voxel_and_object_queries_in_the_same_world() {
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId tool = new FrameId("tool");
        frames.define(tool, frames.root(), new RigidTransform3(new Quaternion(0, 0, 1, 0), new Vector3(6, 0, 1)));
        var tracer = occluded(frames.snapshot(), new AabbEntry3<>(box(2, 3), "contact"));
        frames.define(tool, frames.root(), RigidTransform3.translation(100, 0, 0));

        var hit = tracer.firstVisibleHit(tool, ray(0.5, 1), 10,
                (v, r, a, b) -> true, (x, y, z) -> x == 2);
        assertEquals(2.5, hit.tEnter());
        assertEquals(2.5, hit.tExit());
        assertEquals(new Vector3(3, 0.5, 0.5), hit.worldPoint());
        var voxel = tracer.voxelTracer().firstHit(tool, ray(0.5, 1), 10, (x, y, z) -> x == 2);
        assertEquals(hit.tEnter(), voxel.tEnter());
    }

    private static FrameOccludedBroadPhaseRayTracer3<String> occluded(
            FrameGraph3 frames, AabbEntry3<String> entry
    ) {
        return new FrameOccludedBroadPhaseRayTracer3<>(frames,
                new LinearAabbBroadPhase3<>(List.of(entry)), dda());
    }

    private static VoxelTraverser dda() {
        return ServiceRegistry.of(VoxelTraverser.class).require("dda");
    }

    private static Ray ray(double x, double direction) {
        return new Ray(new Vector3(x, 0.5, 0.5), new Vector3(direction, 0, 0));
    }

    private static AxisAlignedBox box(double minX, double maxX) {
        return new AxisAlignedBox(new Vector3(minX, 0, 0), new Vector3(maxX, 1, 1));
    }
}
