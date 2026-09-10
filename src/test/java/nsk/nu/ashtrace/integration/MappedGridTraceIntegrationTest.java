package nsk.nu.ashtrace.integration;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Quaternion;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.implementation.grid.indexing.SquareXZChunkScheme;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.grid.FrameGridSpaceMapper3;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameOccludedBroadPhaseRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameOccludedExactRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MappedGridTraceIntegrationTest {

    @Test
    void scaled_translated_rotated_grid_preserves_world_distance_and_grid_indices() {
        for (double size : new double[]{0.25, 0.5, 2, 3}) {
            var frames = FrameGraph3.worldRoot();
            var ship = new FrameId("ship");
            frames.define(ship, frames.root(), new RigidTransform3(new Quaternion(0, 0, 1, 0), new Vector3(10, 2, 5)));
            var grid = new FrameGridSpaceMapper3(frames, ship, size, new Vector3(1, -1, 2), new SquareXZChunkScheme(16));
            var tracer = FrameGridRayTracer3.forGrid(grid, dda());
            Ray ray = new Ray(new Vector3(1 + size * 0.25, -1 + size * 0.5, 2 + size * 0.5), new Vector3(1, 0, 0));
            var hit = tracer.firstHit(ship, ray, 4 * size, (x, y, z) -> x == 1 && y == 0 && z == 0);
            assertEquals(1, hit.x());
            assertEquals(0, hit.y());
            assertEquals(0, hit.z());
            assertEquals(0.75 * size, hit.tEnter(), 1e-12);
            assertEquals(1.75 * size, hit.tExit(), 1e-12);
            assertEquals(new Vector3(9 - size, 1 + size * 0.5, 3 - size * 0.5), hit.worldPoint());
            assertEquals(grid, tracer.grid());
            var all = tracer.allHits(ship, ray, 4 * size, (x, y, z) -> true, 2);
            assertEquals(List.of(0, 1), all.stream().map(h -> h.x()).toList());
        }
    }

    @Test
    void moving_and_frozen_mappers_affect_later_queries_as_documented() {
        var frames = FrameGraph3.worldRoot();
        var ship = new FrameId("ship");
        frames.define(ship, frames.root(), RigidTransform3.translation(10, 0, 0));
        var grid = new FrameGridSpaceMapper3(frames, ship, 2, new Vector3(0, 0, 0), new SquareXZChunkScheme(16));
        var live = FrameGridRayTracer3.forGrid(grid, dda());
        var frozen = FrameGridRayTracer3.forGrid(grid.snapshot(), dda());
        var ray = new Ray(new Vector3(0.5, 0.5, 0.5), new Vector3(1, 0, 0));
        frames.define(ship, frames.root(), RigidTransform3.translation(20, 0, 0));
        var liveHit = live.firstHit(ship, ray, 10, (x, y, z) -> x == 1);
        var frozenHit = frozen.firstHit(ship, ray, 10, (x, y, z) -> x == 1);
        assertEquals(22, liveHit.worldPoint().x());
        assertEquals(12, frozenHit.worldPoint().x());
        assertEquals(frozenHit.tEnter(), liveHit.tEnter());
        frames.remove(ship);
        assertThrows(IllegalArgumentException.class, () -> live.firstHit(ship, ray, 10, (x, y, z) -> true));
        assertEquals(frozenHit, frozen.firstHit(ship, ray, 10, (x, y, z) -> x == 1));
    }

    @Test
    void occlusion_uses_world_distance_for_both_bounds_and_exact_intervals() {
        var frames = FrameGraph3.worldRoot();
        var ship = new FrameId("ship");
        frames.define(ship, frames.root(), RigidTransform3.translation(10, 0, 0));
        var grid = new FrameGridSpaceMapper3(frames, ship, 2, new Vector3(0, 0, 0), new SquareXZChunkScheme(16));
        var index = new LinearAabbBroadPhase3<>(List.of(new AabbEntry3<>(
                new AxisAlignedBox(new Vector3(11, 0, 0), new Vector3(16, 1, 1)), "shape")));
        var bounds = FrameOccludedBroadPhaseRayTracer3.forGrid(grid, index, dda());
        var exact = FrameOccludedExactRayTracer3.forGrid(grid, index, dda());
        var ray = new Ray(new Vector3(0.5, 0.5, 0.5), new Vector3(1, 0, 0));
        var broadHit = bounds.firstVisibleHit(ship, ray, 10, (v, r, a, b) -> true, (x, y, z) -> x == 1);
        assertEquals(0.5, broadHit.tEnter());
        assertEquals(1.5, broadHit.tExit());
        assertTrue(bounds.anyVisibleHit(ship, ray, 10, (v, r, a, b) -> true, (x, y, z) -> x == 1));
        var hit = exact.firstVisibleHit(ship, ray, 10,
                (v, r, a, b, out) -> out.accept(new RayIntersection3(0.75, 4.5)), (x, y, z) -> x == 1);
        assertEquals(0.75, hit.tEnter());
        assertEquals(1.5, hit.tExit());
        assertEquals(12, hit.worldExitPoint().x());
        assertFalse(hit.exitSurface());
        assertNull(exact.firstVisibleHit(ship, ray, 10,
                (v, r, a, b, out) -> out.accept(new RayIntersection3(2.5, 4.5)), (x, y, z) -> x == 1));
    }

    @Test
    void decimal_origins_follow_mapper_floor_rules_without_epsilon() {
        var frames = FrameGraph3.worldRoot();
        for (double size : new double[]{0.1, 0.3, 2}) {
            var grid = new FrameGridSpaceMapper3(frames, frames.root(), size, new Vector3(-3, 2, 1), new SquareXZChunkScheme(16));
            var tracer = FrameGridRayTracer3.forGrid(grid, dda());
            for (double x : new double[]{-3.3, -3.0, Math.nextDown(-3.0), -2.7}) {
                var origin = new Vector3(x, 2.05, 1.05);
                var expected = grid.worldToCell(origin);
                var hit = tracer.firstHit(frames.root(), new Ray(origin, new Vector3(-1, 0, 0)), size, (a, b, c) -> true);
                assertEquals(expected.x(), hit.x());
                assertEquals(expected.y(), hit.y());
                assertEquals(expected.z(), hit.z());
                assertEquals(0, hit.tEnter());
            }
        }
    }

    @Test
    void segment_endpoint_and_empty_interval_keep_the_existing_rules() {
        var frames = FrameGraph3.worldRoot();
        var grid = new FrameGridSpaceMapper3(frames, frames.root(), 2, new Vector3(0, 0, 0), new SquareXZChunkScheme(16));
        var tracer = FrameGridRayTracer3.forGrid(grid, dda());
        var ray = new Ray(new Vector3(0.5, 0.5, 0.5), new Vector3(1, 0, 0));
        assertNull(tracer.firstHit(frames.root(), ray, 0, (x, y, z) -> { throw new AssertionError("empty interval"); }));
        var segment = new Segment3(ray.origin(), ray.at(1.5));
        assertNull(tracer.firstSegmentHit(frames.root(), segment, (x, y, z) -> x == 1));
        var hits = tracer.allSegmentHits(frames.root(), segment, (x, y, z) -> true);
        assertEquals(1, hits.size());
        assertEquals(1.5, hits.getFirst().tExit());
    }

    @Test
    void unrepresentable_cell_distance_is_rejected() {
        var frames = FrameGraph3.worldRoot();
        var ray = new Ray(new Vector3(0, 0, 0), new Vector3(1, 0, 0));
        for (double size : new double[]{Double.MIN_VALUE, Double.MAX_VALUE}) {
            var grid = new FrameGridSpaceMapper3(frames, frames.root(), size, new Vector3(0, 0, 0), new SquareXZChunkScheme(16));
            var tracer = FrameGridRayTracer3.forGrid(grid, dda());
            double max = size == Double.MIN_VALUE ? 1 : Double.MIN_VALUE;
            assertThrows(IllegalArgumentException.class, () -> tracer.firstHit(frames.root(), ray, max, (x, y, z) -> true));
        }
    }

    private static VoxelTraverser dda() {
        return ServiceRegistry.of(VoxelTraverser.class).require("dda");
    }
}
