package nsk.nu.ashtrace.integration;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Quaternion;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashgrid.api.grid.indexing.CellIndex3;
import nsk.nu.ashgrid.api.voxel.query.LineOfSight;
import nsk.nu.ashgrid.api.voxel.query.Raycast;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraversers;
import nsk.nu.ashgrid.implementation.grid.indexing.SquareXZChunkScheme;
import nsk.nu.ashgrid.implementation.raster.sparse.HashSparseGrid3i;
import nsk.nu.ashgrid.implementation.voxel.traversal.DDA3DTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.grid.FrameGridSpaceMapper3;
import nsk.nu.ashspace.api.space.SpaceConverter3;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameGridRayTracer3;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Scenarios from Ashgrid traversal/LOS tests and Ashspace mapper/moving-grid tests. */
class BlackframeGridContractIntegrationTest {
    private final FrameGraph3 frames = FrameGraph3.worldRoot();
    private final FrameId ship = new FrameId("ship");

    @Test
    void six_axis_directions_preserve_ashgrid_intervals_in_world_units() {
        frames.define(ship, frames.root(), RigidTransform3.translation(10, 2, -4));
        Vector3 origin = new Vector3(-0.25, 1, -2);
        for (double size : new double[]{0.125, 0.3, 2, 17}) {
            var mapper = mapper(size, origin);
            var tracer = FrameGridRayTracer3.forGrid(mapper, new DDA3DTraverser());
            for (int axis = 0; axis < 3; axis++) for (int sign : new int[]{-1, 1}) {
                double[] direction = {0, 0, 0};
                direction[axis] = sign;
                var ray = new Ray(origin.add(new Vector3(0.2, 0.2, 0.2).mul(size)),
                        new Vector3(direction[0], direction[1], direction[2]));
                var hits = tracer.allHits(ship, ray, size, (x, y, z) -> true);
                assertEquals(2, hits.size());
                double crossing = (sign < 0 ? 0.2 : 0.8) * size;
                assertEquals(0, hits.getFirst().tEnter());
                assertEquals(crossing, hits.getFirst().tExit(), 1e-12);
                assertEquals(crossing, hits.getLast().tEnter(), 1e-12);
                assertEquals(size, hits.getLast().tExit());
                int[] last = {0, 0, 0};
                last[axis] = sign;
                assertEquals(new CellIndex3(last[0], last[1], last[2]),
                        new CellIndex3(hits.getLast().x(), hits.getLast().y(), hits.getLast().z()));
            }
        }
    }

    @Test
    void negative_corner_ties_keep_ashgrid_zero_length_visits() {
        frames.define(ship, frames.root(), RigidTransform3.translation(10, 2, -4));
        var tracer = FrameGridRayTracer3.forGrid(mapper(0.5, Vector3.ZERO), new DDA3DTraverser());
        var hits = tracer.allHits(ship, new Ray(Vector3.ZERO, new Vector3(-1, -1, -1)), 0.25, (x, y, z) -> true);
        assertEquals(List.of(new CellIndex3(0, 0, 0), new CellIndex3(-1, 0, 0),
                        new CellIndex3(-1, -1, 0), new CellIndex3(-1, -1, -1)),
                hits.stream().map(h -> new CellIndex3(h.x(), h.y(), h.z())).toList());
        for (int i = 0; i < 3; i++) assertEquals(0, hits.get(i).tExit());
        assertEquals(0.25, hits.getLast().tExit());
    }

    @Test
    void mixed_directions_agree_with_mapper_membership_inside_each_interval() {
        frames.define(ship, frames.root(), new RigidTransform3(
                Quaternion.fromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2), new Vector3(10, 2, -4)));
        Vector3 origin = new Vector3(1, -1, 2);
        for (double size : new double[]{0.25, 0.3, 2}) {
            var mapper = mapper(size, origin);
            var tracer = FrameGridRayTracer3.forGrid(mapper, new DDA3DTraverser());
            for (int sx : new int[]{-1, 1}) for (int sy : new int[]{-1, 1}) for (int sz : new int[]{-1, 1}) {
                Ray ray = new Ray(origin.add(new Vector3(-2.25, -0.75, 1.125).mul(size)), new Vector3(sx, 2 * sy, 3 * sz));
                double previous = 0;
                for (var hit : tracer.allHits(ship, ray, 8 * size, (x, y, z) -> true)) {
                    assertEquals(previous, hit.tEnter(), 1e-12);
                    assertTrue(hit.tExit() >= hit.tEnter());
                    // Sample positive intervals well away from rounded boundary/tie positions.
                    if (hit.tExit() - hit.tEnter() > 1e-9) {
                        Vector3 sample = ray.at((hit.tEnter() + hit.tExit()) * 0.5);
                        assertEquals(mapper.localToCell(ship, sample), new CellIndex3(hit.x(), hit.y(), hit.z()));
                    }
                    previous = hit.tExit();
                }
                assertEquals(8 * size, previous);
            }
        }
    }

    @Test
    void clipped_ashgrid_traverser_uses_cell_coordinates_and_preserves_original_distances() {
        frames.define(ship, frames.root(), RigidTransform3.translation(10, 2, -4));
        var clipped = VoxelTraversers.clipped(new DDA3DTraverser(),
                new AxisAlignedBox(Vector3.ZERO, new Vector3(2, 1, 1)));
        var tracer = FrameGridRayTracer3.forGrid(mapper(0.5, Vector3.ZERO), clipped);
        Ray ray = new Ray(new Vector3(1.5, 0.25, 0.25), new Vector3(-1, 0, 0));
        var hits = tracer.allHits(ship, ray, 10, (x, y, z) -> true);
        assertEquals(List.of(2, 1, 0), hits.stream().map(h -> h.x()).toList());
        assertEquals(List.of(0.5, 0.5, 1.0), hits.stream().map(h -> h.tEnter()).toList());
        assertEquals(List.of(0.5, 1.0, 1.5), hits.stream().map(h -> h.tExit()).toList());
        assertEquals(11, hits.getFirst().worldPoint().x());
        assertEquals(hits.get(1), tracer.firstHit(ship, ray, 10, (x, y, z) -> x < 2));
        assertTrue(tracer.allHits(ship, new Ray(new Vector3(0, 0.5, 0.25), new Vector3(1, 0, 0)),
                10, (x, y, z) -> true).isEmpty());
    }

    @Test
    void stored_block_is_traced_after_moving_and_rotating_its_frame() {
        var blocks = new HashSparseGrid3i(0);
        blocks.set(1, 0, -2, 42);
        frames.define(ship, frames.root(), RigidTransform3.translation(10, 2, -4));
        var mapper = mapper(0.5, Vector3.ZERO);
        var live = FrameGridRayTracer3.forGrid(mapper, new DDA3DTraverser());
        var frozen = FrameGridRayTracer3.forGrid(mapper.snapshot(), new DDA3DTraverser());
        Ray local = new Ray(new Vector3(0.25, 0.25, -0.75), new Vector3(1, 0, 0));
        Ray oldWorld = new SpaceConverter3(frames).ray(local, ship, frames.root());
        Raycast.Occupancy occupancy = (x, y, z) -> blocks.get(x, y, z) == 42;
        var oldHit = live.firstHit(frames.root(), oldWorld, 2, occupancy);
        frames.define(ship, frames.root(), new RigidTransform3(new Quaternion(0, 0, 1, 0), new Vector3(-10, 2, 4)));
        Ray newWorld = new SpaceConverter3(frames).ray(local, ship, frames.root());
        var newHit = live.firstHit(frames.root(), newWorld, 2, occupancy);
        assertEquals(new CellIndex3(1, 0, -2), new CellIndex3(newHit.x(), newHit.y(), newHit.z()));
        assertEquals(0.25, newHit.tEnter());
        assertEquals(oldHit.tEnter(), newHit.tEnter());
        assertEquals(10.5, oldHit.worldPoint().x());
        assertEquals(-10.5, newHit.worldPoint().x());
        assertNull(live.firstHit(frames.root(), oldWorld, 2, occupancy));
        assertEquals(oldHit, frozen.firstHit(frames.root(), oldWorld, 2, occupancy));
        blocks.set(1, 0, -2, 0);
        assertNull(live.firstHit(frames.root(), newWorld, 2, occupancy));
        assertNull(frozen.firstHit(frames.root(), oldWorld, 2, occupancy));
    }

    @Test
    void relative_grid_trace_preserves_offsets_at_ashspace_large_world_origin() {
        frames.define(ship, frames.root(), RigidTransform3.translation(0x1.0p54, 0, 0));
        var tool = new FrameId("tool");
        frames.define(tool, ship, RigidTransform3.translation(1, 0, 0));
        var mapper = mapper(1, Vector3.ZERO);
        var tracer = FrameGridRayTracer3.forGrid(mapper, new DDA3DTraverser());
        Ray ray = new Ray(new Vector3(0.5, 0.5, 0.5), new Vector3(1, 0, 0));
        var hit = tracer.firstHit(tool, ray, 2, (x, y, z) -> x == 2);
        assertEquals(2, hit.x());
        assertEquals(0.5, hit.tEnter());
        assertEquals(1.5, hit.tExit());
        assertEquals(new CellIndex3(hit.x(), hit.y(), hit.z()), mapper.localToCell(tool, ray.at(1)));
        // World points have double precision; local cell membership does not imply a world round trip.
    }

    @Test
    void ashtrace_uses_raycast_start_cell_rules_not_line_of_sight_exclusion() {
        var dda = new DDA3DTraverser();
        Ray ray = new Ray(new Vector3(0.1, 0.1, 0.1), new Vector3(1, 0, 0));
        Raycast.Occupancy start = (x, y, z) -> x == 0;
        assertTrue(new LineOfSight(dda).clear(ray.origin(), ray.at(2.8), start::test));
        assertEquals(0, new Raycast(dda).first(ray, 2.8, start).tEnter());
        var tracer = new FrameGridRayTracer3(frames, dda);
        assertEquals(0, tracer.firstHit(frames.root(), ray, 2.8, start).tEnter());
        assertFalse(new LineOfSight(dda).clear(ray.origin(), ray.at(2.8), (x, y, z) -> x == 1));
        assertEquals(0.9, tracer.firstHit(frames.root(), ray, 2.8, (x, y, z) -> x == 1).tEnter(), 1e-12);
    }

    @Test
    void mapped_grid_rejects_coordinate_underflow_before_querying_occupancy() {
        frames.define(ship, frames.root(), RigidTransform3.identity());
        var tracer = FrameGridRayTracer3.forGrid(mapper(2, Vector3.ZERO), new DDA3DTraverser());
        int[] calls = {0};
        for (int axis = 0; axis < 3; axis++) for (int sign : new int[]{-1, 1}) {
            double[] point = {0.5, 0.5, 0.5};
            point[axis] = sign * Double.MIN_VALUE;
            Ray ray = new Ray(new Vector3(point[0], point[1], point[2]), new Vector3(1, 0, 0));
            assertThrows(IllegalArgumentException.class, () ->
                    tracer.firstHit(ship, ray, 1, (x, y, z) -> { calls[0]++; return true; }));
            assertThrows(IllegalArgumentException.class, () ->
                    tracer.allHits(ship, ray, 1, (x, y, z) -> { calls[0]++; return true; }));
        }
        assertEquals(0, calls[0]);
        // Exact zero remains a valid cell boundary; the guard introduces no epsilon.
        assertEquals(0, tracer.firstHit(ship, new Ray(new Vector3(0, 0.5, 0.5), new Vector3(1, 0, 0)),
                1, (x, y, z) -> true).x());
    }

    private FrameGridSpaceMapper3 mapper(double size, Vector3 origin) {
        return new FrameGridSpaceMapper3(frames, ship, size, origin, new SquareXZChunkScheme(16));
    }
}
