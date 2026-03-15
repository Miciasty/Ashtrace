package nsk.nu.ashtrace.unit.api.trace;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.transform.RigidTransform3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.model.TraceHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

class FrameBroadPhaseRayTracer3ApiTest {

    @Test
    void first_hit_applies_narrow_phase_filter_in_sorted_candidate_order() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        FrameId local = new FrameId("local");
        frames.define(local, frames.root(), RigidTransform3.translation(10, 0, 0));

        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(12, 1, 0, 13, 2, 1), "A"),
                new AabbEntry3<>(box(14, 1, 0, 15, 2, 1), "B")
        ));
        FrameBroadPhaseRayTracer3<String> tracer = new FrameBroadPhaseRayTracer3<>(frames, broadPhase);
        Ray localRay = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN
        TraceHit3<String> hit = tracer.firstHit(local, localRay, 10.0, (value, ray, t0, t1) -> "B".equals(value));

        // THEN
        assertNotNull(hit);
        assertEquals("B", hit.value());
        assertEquals(3.8, hit.tEnter(), 1e-9);
        assertEquals(14.0, hit.worldPoint().x(), 1e-9);
    }

    @Test
    void all_hits_are_sorted_by_t_enter() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        BvhAabbBroadPhase3<String> broadPhase = new BvhAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(5, 1, 0, 6, 2, 1), "B"),
                new AabbEntry3<>(box(2, 1, 0, 3, 2, 1), "A")
        ));
        FrameBroadPhaseRayTracer3<String> tracer = new FrameBroadPhaseRayTracer3<>(frames, broadPhase);
        Ray ray = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN
        List<TraceHit3<String>> hits = tracer.allHits(frames.root(), ray, 10.0, (value, worldRay, t0, t1) -> true);

        // THEN
        assertEquals(2, hits.size());
        assertEquals("A", hits.get(0).value());
        assertEquals("B", hits.get(1).value());
        assertEquals(1.8, hits.get(0).tEnter(), 1e-9);
    }

    @Test
    void all_hits_support_max_hits_limit() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(2, 1, 0, 3, 2, 1), "A"),
                new AabbEntry3<>(box(4, 1, 0, 5, 2, 1), "B"),
                new AabbEntry3<>(box(6, 1, 0, 7, 2, 1), "C")
        ));
        FrameBroadPhaseRayTracer3<String> tracer = new FrameBroadPhaseRayTracer3<>(frames, broadPhase);
        Ray ray = new Ray(new Vector3(0.2, 1.2, 0.2), new Vector3(1, 0, 0));

        // WHEN
        List<TraceHit3<String>> limited = tracer.allHits(frames.root(), ray, 10.0, (v, wr, t0, t1) -> true, 2);

        // THEN
        assertEquals(List.of("A", "B"), limited.stream().map(TraceHit3::value).toList());
    }

    @Test
    void segment_tracing_respects_segment_length() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of(
                new AabbEntry3<>(box(2, 1, 0, 3, 2, 1), "A"),
                new AabbEntry3<>(box(5, 1, 0, 6, 2, 1), "B")
        ));
        FrameBroadPhaseRayTracer3<String> tracer = new FrameBroadPhaseRayTracer3<>(frames, broadPhase);
        Segment3 segment = new Segment3(new Vector3(0.2, 1.2, 0.2), new Vector3(3.5, 1.2, 0.2));

        // WHEN
        TraceHit3<String> first = tracer.firstSegmentHit(frames.root(), segment, (value, ray, t0, t1) -> true);
        List<TraceHit3<String>> all = tracer.allSegmentHits(frames.root(), segment, (value, ray, t0, t1) -> true);
        TraceHit3<String> none = tracer.firstSegmentHit(frames.root(), segment, (value, ray, t0, t1) -> "B".equals(value));

        // THEN
        assertNotNull(first);
        assertEquals("A", first.value());
        assertEquals(1, all.size());
        assertNull(none);
    }

    @Test
    void invalid_parameters_are_rejected() {
        // GIVEN
        FrameGraph3 frames = FrameGraph3.worldRoot();
        LinearAabbBroadPhase3<String> broadPhase = new LinearAabbBroadPhase3<>(List.of());
        FrameBroadPhaseRayTracer3<String> tracer = new FrameBroadPhaseRayTracer3<>(frames, broadPhase);
        Ray ray = new Ray(new Vector3(0, 0, 0), new Vector3(1, 0, 0));
        Segment3 pointSegment = new Segment3(new Vector3(1, 1, 1), new Vector3(1, 1, 1));

        // WHEN / THEN
        assertThrows(IllegalArgumentException.class, () -> tracer.firstHit(frames.root(), ray, Double.NaN, (v, wr, t0, t1) -> true));
        assertThrows(IllegalArgumentException.class, () -> tracer.allHits(frames.root(), ray, Double.POSITIVE_INFINITY, (v, wr, t0, t1) -> true));
        assertThrows(IllegalArgumentException.class, () -> tracer.firstSegmentHit(frames.root(), pointSegment, (v, wr, t0, t1) -> true));
        assertThrows(IllegalArgumentException.class, () -> tracer.allHits(frames.root(), ray, 10.0, (v, wr, t0, t1) -> true, 0));
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
