package nsk.nu.ashtrace.unit.api.trace;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Quaternion;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashcore.api.spi.ServiceRegistry;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.ExactTraceHit3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameExactRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameOccludedExactRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.TraceQueryBuffer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ExactRayTracerTest {
    private final FrameGraph3 frames = FrameGraph3.worldRoot();
    private final Ray ray = new Ray(new Vector3(0, 0.5, 0.5), new Vector3(1, 0, 0));

    @Test
    void entry_and_exit_are_actual_provider_points() {
        var tracer = tracer(entry(1, 10, "object"));
        var hit = tracer.firstHit(frames.root(), ray, 20, intervals(2, 5));
        assertEquals(new Vector3(2, 0.5, 0.5), hit.worldEnterPoint());
        assertEquals(new Vector3(5, 0.5, 0.5), hit.worldExitPoint());
        assertEquals(3, hit.distanceInside());
        assertTrue(hit.enterSurface());
        assertTrue(hit.exitSurface());
        assertEquals(hit, tracer.lastHit(frames.root(), ray, 20, intervals(2, 5)));
    }

    @Test
    void exact_selection_can_reverse_bounds_order_and_last_is_selected_by_exit() {
        var tracer = tracer(entry(1, 12, "A"), entry(2, 7, "B"), entry(4, 11, "C"));
        RayIntersector3<String> provider = (value, r, a, b, out) -> out.accept(switch (value) {
            case "A" -> new RayIntersection3(8, 12);
            case "B" -> new RayIntersection3(3, 7);
            default -> new RayIntersection3(9, 11);
        });
        assertEquals("B", tracer.firstHit(frames.root(), ray, 20, provider).value());
        assertEquals("A", tracer.lastHit(frames.root(), ray, 20, provider).value());
        var all = tracer.allHits(frames.root(), ray, 20, provider);
        assertEquals(List.of("B", "A", "C"), all.stream().map(ExactTraceHit3::value).toList());
        assertEquals(List.of(all.getFirst()), tracer.allHits(frames.root(), ray, 20, provider, 1));
    }

    @Test
    void disconnected_parts_produce_separate_sorted_intervals() {
        var tracer = tracer(entry(1, 10, "shell"));
        var hits = tracer.allHits(frames.root(), ray, 20, intervals(7, 9, 2, 4));
        assertEquals(List.of(2.0, 7.0), hits.stream().map(ExactTraceHit3::tEnter).toList());
        assertEquals(List.of(4.0, 9.0), hits.stream().map(ExactTraceHit3::tExit).toList());
        assertEquals(9, tracer.lastHit(frames.root(), ray, 20, intervals(7, 9, 2, 4)).tExit());
        assertThrows(UnsupportedOperationException.class, () -> hits.clear());
    }

    @Test
    void origin_and_query_limit_are_not_fabricated_surface_boundaries() {
        var tracer = tracer(entry(-5, 10, "inside"));
        var hit = tracer.firstHit(frames.root(), ray, 3, intervals(-2, 8));
        assertEquals(0, hit.tEnter());
        assertEquals(3, hit.tExit());
        assertFalse(hit.enterSurface());
        assertFalse(hit.exitSurface());
        var zero = tracer.firstHit(frames.root(), ray, 0, intervals(-2, 8));
        assertEquals(0, zero.distanceInside());
        assertFalse(zero.enterSurface());
        assertFalse(zero.exitSurface());
        var boundary = tracer.firstHit(frames.root(), ray, 8, intervals(0, 8));
        assertTrue(boundary.enterSurface());
        assertTrue(boundary.exitSurface());
    }

    @Test
    void tangency_and_surface_only_hits_have_equal_endpoints() {
        var hit = tracer(entry(1, 5, "surface")).firstHit(frames.root(), ray, 5, intervals(3, 3));
        assertEquals(hit.worldEnterPoint(), hit.worldExitPoint());
        assertEquals(0, hit.distanceInside());
        assertTrue(hit.enterSurface());
        assertTrue(hit.exitSurface());
    }

    @Test
    void full_ties_keep_candidate_then_provider_emission_order() {
        var tracer = tracer(entry(1, 5, "A"), entry(1, 5, "B"));
        assertEquals("A", tracer.firstHit(frames.root(), ray, 10, intervals(2, 4)).value());
        assertEquals("A", tracer.lastHit(frames.root(), ray, 10, intervals(2, 4)).value());
        assertEquals(List.of("A", "B"), tracer.allHits(frames.root(), ray, 10, intervals(2, 4))
                .stream().map(ExactTraceHit3::value).toList());
    }

    @Test
    void wall_clips_real_exit_and_hides_surfaces_behind_loose_bounds() {
        var index = new LinearAabbBroadPhase3<>(List.of(entry(1, 10, "shape")));
        var tracer = new FrameOccludedExactRayTracer3<>(frames, index, dda());
        var hit = tracer.firstVisibleHit(frames.root(), ray, 12, intervals(2, 8), (x, y, z) -> x == 5);
        assertEquals(2, hit.tEnter());
        assertEquals(5, hit.tExit());
        assertTrue(hit.enterSurface());
        assertFalse(hit.exitSurface());
        assertNull(tracer.firstVisibleHit(frames.root(), ray, 12, intervals(8, 9), (x, y, z) -> x == 5));
        var contact = tracer.firstVisibleHit(frames.root(), ray, 12, intervals(5, 8), (x, y, z) -> x == 5);
        assertEquals(5, contact.tEnter());
        assertEquals(5, contact.tExit());
        assertTrue(contact.enterSurface());
        assertFalse(contact.exitSurface());
    }

    @Test
    void segments_and_visible_selection_preserve_clipping() {
        Segment3 segment = new Segment3(ray.origin(), ray.at(4));
        var tracer = tracer(entry(1, 10, "shape"));
        assertTrue(tracer.anySegmentHit(frames.root(), segment, intervals(2, 8)));
        assertEquals(4, tracer.lastSegmentHit(frames.root(), segment, intervals(2, 8)).tExit());
        assertFalse(tracer.firstSegmentHit(frames.root(), segment, intervals(2, 8)).exitSurface());
        assertEquals(1, tracer.allSegmentHits(frames.root(), segment, intervals(2, 8)).size());
        var visible = new FrameOccludedExactRayTracer3<>(frames,
                new LinearAabbBroadPhase3<>(List.of(entry(1, 10, "shape"))), dda());
        assertTrue(visible.anyVisibleSegmentHit(frames.root(), segment, intervals(2, 8), (x, y, z) -> false));
        assertEquals(4, visible.lastVisibleSegmentHit(frames.root(), segment, intervals(2, 8), (x, y, z) -> false).tExit());
        assertEquals(2, visible.firstVisibleSegmentHit(frames.root(), segment, intervals(2, 8), (x, y, z) -> false).tEnter());
        assertEquals(1, visible.visibleSegmentHits(frames.root(), segment, intervals(2, 8), (x, y, z) -> false, 1).size());
        assertThrows(IllegalArgumentException.class, () -> tracer.firstSegmentHit(frames.root(),
                new Segment3(ray.origin(), ray.origin()), intervals(2, 8)));
    }

    @Test
    void invalid_provider_intervals_fail_and_buffer_remains_reusable() {
        var tracer = tracer(entry(2, 5, "shape"));
        var buffer = new TraceQueryBuffer3<String>();
        assertThrows(IllegalArgumentException.class, () -> tracer.allHits(frames.root(), ray, 10, intervals(1, 4), 10, buffer));
        assertThrows(IllegalArgumentException.class, () -> tracer.firstHit(frames.root(), ray, 10, intervals(3, 6)));
        assertThrows(IllegalArgumentException.class, () -> new RayIntersection3(5, 2));
        assertThrows(IllegalArgumentException.class, () -> new RayIntersection3(Double.NaN, 2));
        assertThrows(IllegalArgumentException.class, () -> new RayIntersection3(1, Double.POSITIVE_INFINITY));
        assertThrows(NullPointerException.class, () -> tracer.firstHit(frames.root(), ray, 10, (v, r, a, b, out) -> out.accept(null)));
        var hits = tracer.allHits(frames.root(), ray, 10, intervals(2, 4), 10, buffer);
        assertEquals(1, hits.size());
        tracer.allHits(frames.root(), ray, 10, (v, r, a, b, out) -> {}, 10, buffer);
        assertEquals(2, hits.getFirst().tEnter());
        buffer.trimToSize();
    }

    @Test
    void any_hit_stops_later_candidates_and_empty_queries_are_distinct_from_invalid_input() {
        var tracer = tracer(entry(1, 5, "A"), entry(1, 5, "B"));
        var calls = new ArrayList<String>();
        assertTrue(tracer.anyHit(frames.root(), ray, 10, (v, r, a, b, out) -> {
            calls.add(v);
            out.accept(new RayIntersection3(2, 4));
        }));
        assertEquals(List.of("A"), calls);
        assertNull(tracer.firstHit(frames.root(), ray, 10, (v, r, a, b, out) -> {}));
        assertNull(tracer.lastHit(frames.root(), ray, 10, (v, r, a, b, out) -> {}));
        assertFalse(tracer.anyHit(frames.root(), ray, 10, (v, r, a, b, out) -> {}));
        assertThrows(IllegalArgumentException.class, () -> tracer.firstHit(frames.root(), ray, Double.NaN, intervals(2, 4)));
        assertThrows(IllegalArgumentException.class, () -> tracer.allHits(frames.root(), ray, 10, intervals(2, 4), 0));
        assertThrows(NullPointerException.class, () -> tracer.firstHit(frames.root(), ray, 10, null));
    }

    @Test
    void reverse_ray_returns_entry_and_exit_in_travel_order() {
        var reverse = new Ray(new Vector3(10, 0.5, 0.5), new Vector3(-1, 0, 0));
        var hit = tracer(entry(2, 5, "shape")).firstHit(frames.root(), reverse, 10, intervals(5, 8));
        assertEquals(5, hit.worldEnterPoint().x());
        assertEquals(2, hit.worldExitPoint().x());
    }

    @Test
    void provider_receives_world_ray_after_rotation_and_translation() {
        var tool = new FrameId("tool");
        frames.define(tool, frames.root(), new RigidTransform3(new Quaternion(0, 0, 1, 0), new Vector3(10, 0, 0)));
        Ray local = new Ray(new Vector3(0, 0.5, -0.5), new Vector3(1, 0, 0));
        var hit = tracer(entry(2, 8, "shape")).firstHit(tool, local, 10, (v, r, a, b, out) -> {
            assertEquals(new Vector3(10, 0.5, 0.5), r.origin());
            assertEquals(new Vector3(-1, 0, 0), r.direction());
            assertEquals(2, a);
            assertEquals(8, b);
            out.accept(new RayIntersection3(3, 7));
        });
        assertEquals(new Vector3(7, 0.5, 0.5), hit.worldEnterPoint());
        assertEquals(new Vector3(3, 0.5, 0.5), hit.worldExitPoint());
    }

    @Test
    void delayed_output_and_reentrant_buffer_use_cannot_corrupt_results() {
        var tracer = tracer(entry(1, 10, "shape"));
        var buffer = new TraceQueryBuffer3<String>();
        var outputs = new ArrayList<Consumer<RayIntersection3>>();
        var hits = tracer.allHits(frames.root(), ray, 10, (v, r, a, b, out) -> {
            outputs.add(out);
            assertThrows(IllegalStateException.class, () ->
                    tracer.allHits(frames.root(), ray, 10, intervals(2, 5), 10, buffer));
            assertThrows(IllegalStateException.class, buffer::trimToSize);
            out.accept(new RayIntersection3(2, 4));
        }, 10, buffer);
        assertThrows(IllegalStateException.class, () -> outputs.getFirst().accept(new RayIntersection3(5, 6)));
        assertEquals(List.of(2.0), hits.stream().map(ExactTraceHit3::tEnter).toList());
        assertEquals(1, tracer.allHits(frames.root(), ray, 10, intervals(3, 5), 10, buffer).size());
        assertEquals(2, hits.getFirst().tEnter());
    }

    @Test
    void exact_result_rejects_nonfinite_points_and_invalid_distances() {
        Vector3 point = ray.origin();
        assertThrows(IllegalArgumentException.class, () -> new ExactTraceHit3<>("shape", -1, 2, point, point, true, true));
        assertThrows(IllegalArgumentException.class, () -> new ExactTraceHit3<>("shape", 2, 1, point, point, true, true));
        assertThrows(IllegalArgumentException.class, () -> new ExactTraceHit3<>("shape", 0, Double.NaN, point, point, true, true));
        assertThrows(IllegalArgumentException.class, () -> new ExactTraceHit3<>("shape", 0, 1, point,
                new Vector3(Double.POSITIVE_INFINITY, 0, 0), true, true));
        assertThrows(NullPointerException.class, () -> new ExactTraceHit3<>("shape", 0, 1, null, point, true, true));
    }

    @SafeVarargs
    private FrameExactRayTracer3<String> tracer(AabbEntry3<String>... entries) {
        return new FrameExactRayTracer3<>(frames, new LinearAabbBroadPhase3<>(List.of(entries)));
    }

    private static RayIntersector3<String> intervals(double... endpoints) {
        return (value, ray, min, max, output) -> {
            for (int i = 0; i < endpoints.length; i += 2) output.accept(new RayIntersection3(endpoints[i], endpoints[i + 1]));
        };
    }

    private static AabbEntry3<String> entry(double min, double max, String value) {
        return new AabbEntry3<>(new AxisAlignedBox(new Vector3(min, 0, 0), new Vector3(max, 1, 1)), value);
    }

    private static VoxelTraverser dda() {
        return ServiceRegistry.of(VoxelTraverser.class).require("dda");
    }
}
