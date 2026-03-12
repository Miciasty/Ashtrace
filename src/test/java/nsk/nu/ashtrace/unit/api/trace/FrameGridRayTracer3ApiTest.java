package nsk.nu.ashtrace.unit.api.trace;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.trace.model.GridRayHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

class FrameGridRayTracer3ApiTest {

    @Test
    void first_hit_uses_source_frame_to_world_conversion() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId world = frames.root();
        FrameId local = new FrameId("local");
        frames.define(local, world, RigidTransform3.translation(10, 0, -2));

        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);
        Ray localRay = new Ray(new Vector3(0.2, 1.2, 2.2), new Vector3(1, 0, 0));

        // WHEN
        GridRayHit3 hit = tracer.firstHit(local, localRay, 10.0, (x, y, z) -> x == 13 && y == 1 && z == 0);

        // THEN
        assertNotNull(hit);
        assertEquals(13, hit.x());
        assertEquals(1, hit.y());
        assertEquals(0, hit.z());
        assertEquals(13.0, hit.worldPoint().x(), 1e-9);
        assertEquals(1.2, hit.worldPoint().y(), 1e-9);
        assertEquals(0.2, hit.worldPoint().z(), 1e-9);
    }

    @Test
    void first_hit_returns_null_when_nothing_matches() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);
        Ray worldRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN
        GridRayHit3 hit = tracer.firstHit(frames.root(), worldRay, 3.0, (x, y, z) -> false);

        // THEN
        assertNull(hit);
    }

    @Test
    void all_hits_follow_deterministic_traverser_order_and_limit() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);
        Ray worldRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN
        List<GridRayHit3> hits = tracer.allHits(
                frames.root(),
                worldRay,
                10.0,
                (x, y, z) -> y == 1 && z == 0 && (x == 2 || x == 3 || x == 5),
                2
        );

        // THEN
        assertEquals(2, hits.size());
        assertEquals(2, hits.get(0).x());
        assertEquals(3, hits.get(1).x());
        assertEquals(1.8, hits.get(0).tEnter(), 1e-9);
        assertEquals(2.8, hits.get(1).tEnter(), 1e-9);
    }

    @Test
    void segment_tracing_limits_hits_to_segment_length() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);
        Segment3 segment = new Segment3(
                new Vector3(0.2, 1.2, 0.2),
                new Vector3(2.5, 1.2, 0.2)
        );

        // WHEN
        GridRayHit3 miss = tracer.firstSegmentHit(frames.root(), segment, (x, y, z) -> x == 3 && y == 1 && z == 0);
        GridRayHit3 hit = tracer.firstSegmentHit(frames.root(), segment, (x, y, z) -> x == 2 && y == 1 && z == 0);

        // THEN
        assertNull(miss);
        assertNotNull(hit);
        assertEquals(2, hit.x());
    }

    @Test
    void zero_length_segment_is_rejected() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);
        Segment3 pointSegment = new Segment3(new Vector3(1, 1, 1), new Vector3(1, 1, 1));

        // WHEN / THEN
        assertThrows(IllegalArgumentException.class, () -> tracer.firstSegmentHit(frames.root(), pointSegment, (x, y, z) -> true));
        assertThrows(IllegalArgumentException.class, () -> tracer.allSegmentHits(frames.root(), pointSegment, (x, y, z) -> true));
    }

    @Test
    void nan_tmax_is_rejected() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);
        Ray ray = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN / THEN
        assertThrows(IllegalArgumentException.class, () -> tracer.firstHit(frames.root(), ray, Double.NaN, (x, y, z) -> true));
        assertThrows(IllegalArgumentException.class, () -> tracer.allHits(frames.root(), ray, Double.NaN, (x, y, z) -> true));
    }
}
