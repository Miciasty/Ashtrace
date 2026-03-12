package nsk.nu.ashtrace.integration;

import nsk.nu.ashcore.api.collision.CollisionTests;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

class FrameTraceBroadPhaseIntegrationTest {

    @Test
    void broad_phase_candidates_can_be_refined_by_ray_hit_time() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId world = frames.root();
        FrameId tool = new FrameId("tool");
        frames.define(tool, world, RigidTransform3.translation(10, 0, 0));

        VoxelTraverser traverser = ServiceRegistry.of(VoxelTraverser.class).require("dda");
        FrameGridRayTracer3 tracer = new FrameGridRayTracer3(frames, traverser);
        Ray localRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        AxisAlignedBox near = box(12, 1, 0, 13, 2, 1);
        AxisAlignedBox far = box(16, 1, 0, 17, 2, 1);

        List<AabbEntry3<String>> entries = List.of(
                new AabbEntry3<>(near, "near"),
                new AabbEntry3<>(far, "far")
        );

        LinearAabbBroadPhase3<String> linear = new LinearAabbBroadPhase3<>(entries);
        BvhAabbBroadPhase3<String> bvh = new BvhAabbBroadPhase3<>(entries);

        AxisAlignedBox broadQuery = box(10, 0, -1, 20, 3, 2);
        List<String> linearCandidates = new ArrayList<>();
        List<String> bvhCandidates = new ArrayList<>();
        linear.query(broadQuery, linearCandidates::add);
        bvh.query(broadQuery, bvhCandidates::add);

        // WHEN
        var hit = tracer.firstHit(tool, localRay, 10.0, (x, y, z) -> x == 13 && y == 1 && z == 0);

        double bestT = Double.POSITIVE_INFINITY;
        String best = null;
        for (String candidate : bvhCandidates) {
            AxisAlignedBox box = "near".equals(candidate) ? near : far;
            double t = CollisionTests.rayVsBoxT(new Ray(hit.worldPoint(), new Vector3(1, 0, 0)), box);
            if (t < bestT) {
                bestT = t;
                best = candidate;
            }
        }

        // THEN
        linearCandidates.sort(String::compareTo);
        bvhCandidates.sort(String::compareTo);
        assertEquals(linearCandidates, bvhCandidates);
        assertEquals(2, bvhCandidates.size());
        assertEquals("near", best);
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
