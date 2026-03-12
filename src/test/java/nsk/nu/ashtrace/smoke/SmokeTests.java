package nsk.nu.ashtrace.smoke;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.trace.model.GridRayHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class SmokeTests {

    @Test
    void frame_aware_voxel_trace_finds_expected_hit() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId world = frames.root();
        FrameId tool = new FrameId("tool");
        frames.define(tool, world, RigidTransform3.translation(10, 0, 0));

        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);
        Ray localRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN
        GridRayHit3 hit = tracer.firstHit(tool, localRay, 10.0, (x, y, z) -> x == 13 && y == 1 && z == 0);

        // THEN
        assertNotNull(hit);
        assertEquals(13, hit.x());
        assertEquals(1, hit.y());
        assertEquals(0, hit.z());
        assertEquals(13.0, hit.worldPoint().x(), 1e-9);
    }
}
