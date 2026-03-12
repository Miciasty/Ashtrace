package nsk.nu.ashtrace.unit.api.trace;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.model.TraceHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameOccludedBroadPhaseRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

class FrameOccludedBroadPhaseRayTracer3ApiTest {

    @Test
    void first_visible_hit_is_clipped_by_voxel_occlusion() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId tool = new FrameId("tool");
        frames.define(tool, frames.root(), RigidTransform3.translation(10, 0, 0));

        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(12, 1, 0, 13, 2, 1), "near"),
                new AabbEntry3<>(box(16, 1, 0, 17, 2, 1), "far")
        ));
        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameOccludedBroadPhaseRayTracer3<String> tracer = new FrameOccludedBroadPhaseRayTracer3<>(
                frames,
                broadPhase,
                traverser
        );
        Ray localRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN
        TraceHit3<String> hit = tracer.firstVisibleHit(
                tool,
                localRay,
                10.0,
                (value, worldRay, t0, t1) -> true,
                (x, y, z) -> x == 15 && y == 1 && z == 0
        );
        TraceHit3<String> blocked = tracer.firstVisibleHit(
                tool,
                localRay,
                10.0,
                (value, worldRay, t0, t1) -> true,
                (x, y, z) -> x == 11 && y == 1 && z == 0
        );

        // THEN
        assertNotNull(hit);
        assertEquals("near", hit.value());
        assertNull(blocked);
    }

    @Test
    void visible_hits_support_limits_and_segments() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(2, 1, 0, 3, 2, 1), "A"),
                new AabbEntry3<>(box(4, 1, 0, 5, 2, 1), "B"),
                new AabbEntry3<>(box(6, 1, 0, 7, 2, 1), "C")
        ));
        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameOccludedBroadPhaseRayTracer3<String> tracer = new FrameOccludedBroadPhaseRayTracer3<>(
                frames,
                broadPhase,
                traverser
        );

        Ray ray = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));
        Segment3 segment = new Segment3(new Vector3(0.2, 1.2, 0.2), new Vector3(5.5, 1.2, 0.2));

        // WHEN
        List<TraceHit3<String>> hits = tracer.visibleHits(
                frames.root(),
                ray,
                10.0,
                (value, worldRay, t0, t1) -> true,
                (x, y, z) -> x == 5 && y == 1 && z == 0,
                10
        );
        List<TraceHit3<String>> limited = tracer.visibleHits(
                frames.root(),
                ray,
                10.0,
                (value, worldRay, t0, t1) -> true,
                (x, y, z) -> false,
                2
        );
        TraceHit3<String> segmentHit = tracer.firstVisibleSegmentHit(
                frames.root(),
                segment,
                (value, worldRay, t0, t1) -> true,
                (x, y, z) -> false
        );

        // THEN
        assertEquals(List.of("A", "B"), hits.stream().map(TraceHit3::value).toList());
        assertEquals(List.of("A", "B"), limited.stream().map(TraceHit3::value).toList());
        assertNotNull(segmentHit);
        assertEquals("A", segmentHit.value());
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
