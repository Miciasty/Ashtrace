package nsk.nu.ashtrace.integration;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class FrameDynamicBroadPhaseIntegrationTest {

    @Test
    void frame_trace_uses_dynamic_broad_phase_updates() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId tool = new FrameId("tool");
        frames.define(tool, frames.root(), RigidTransform3.translation(10, 0, 0));

        DynamicSpatialHashBroadPhase3<String> broadPhase = new DynamicSpatialHashBroadPhase3<>(1.0);
        long near = broadPhase.insert(box(12, 1, 0, 13, 2, 1), "near");
        broadPhase.insert(box(16, 1, 0, 17, 2, 1), "far");

        FrameBroadPhaseRayTracer3<String> tracer = new FrameBroadPhaseRayTracer3<>(frames, broadPhase);
        Ray localRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN
        var first = tracer.firstHit(tool, localRay, 10.0, (value, ray, t0, t1) -> true);
        broadPhase.updateBounds(near, box(18, 1, 0, 19, 2, 1));
        var second = tracer.firstHit(tool, localRay, 12.0, (value, ray, t0, t1) -> true);

        // THEN
        assertNotNull(first);
        assertNotNull(second);
        assertEquals("near", first.value());
        assertEquals("far", second.value());
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
