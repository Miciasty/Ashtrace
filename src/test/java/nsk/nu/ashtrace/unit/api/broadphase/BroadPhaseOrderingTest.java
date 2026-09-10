package nsk.nu.ashtrace.unit.api.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashtrace.api.broadphase.contracts.MutableRayBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.api.trace.model.TraceHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicBvhBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BroadPhaseOrderingTest {

    @Test
    void mutation_replay_and_snapshot_rebuild_preserve_declared_ties() {
        for (int kind = 0; kind < 2; kind++) {
            var first = mutations(kind == 0 ? new DynamicBvhBroadPhase3<>() : new DynamicSpatialHashBroadPhase3<>(1));
            var replay = mutations(kind == 0 ? new DynamicBvhBroadPhase3<>() : new DynamicSpatialHashBroadPhase3<>(1));
            assertEquals(first, replay);
        }
    }

    @Test
    void pipeline_sorts_intervals_but_full_ties_follow_source_order() {
        var entries = new ArrayList<AabbEntry3<String>>();
        entries.add(new AabbEntry3<>(box(5, 6), "far"));
        entries.add(new AabbEntry3<>(box(2, 4), "long"));
        entries.add(new AabbEntry3<>(box(2, 3), "A"));
        entries.add(new AabbEntry3<>(box(2, 3), "B"));
        for (int i = 0; i < 12; i++) entries.add(new AabbEntry3<>(box(20 + i, 21 + i), "filler" + i));
        FrameGraph3 frames = FrameGraph3.worldRoot();
        for (var index : List.of(new LinearAabbBroadPhase3<>(entries), new BvhAabbBroadPhase3<>(entries))) {
            var tracer = new FrameBroadPhaseRayTracer3<>(frames, index);
            var hits = tracer.allHits(frames.root(), ray(), 10, (v, r, a, b) -> true);
            assertEquals(List.of("A", "B", "long", "far"), hits.stream().map(TraceHit3::value).toList());
        }
        var reversed = List.of(new AabbEntry3<>(box(2, 3), "B"), new AabbEntry3<>(box(2, 3), "A"));
        var tracer = new FrameBroadPhaseRayTracer3<>(frames, new BvhAabbBroadPhase3<>(reversed));
        assertEquals("B", tracer.firstHit(frames.root(), ray(), 10, (v, r, a, b) -> true).value());
    }

    private static List<Object> mutations(MutableRayBroadPhase3<String> index) {
        long a = index.insert(box(2, 3), "A");
        long b = index.insert(box(2, 3), "B");
        for (int i = 0; i < 12; i++) index.insert(box(20 + i, 21 + i), "filler" + i);
        var log = new ArrayList<Object>();
        log.add(snapshot(index));
        assertEquals(log.getLast(), snapshot(index));
        assertTrue(index.updateBounds(a, box(7, 8)));
        log.add(snapshot(index));
        assertTrue(index.updateBounds(a, box(2, 3)));
        assertEquals(log.getFirst(), snapshot(index));
        assertTrue(index.remove(a));
        long reinserted = index.insert(box(2, 3), "A");
        assertTrue(reinserted > b);
        log.add(snapshot(index));
        assertEquals("B", index.nearest(new Vector3(2.5, 0.5, 0.5), 0).value());
        var frames = FrameGraph3.worldRoot();
        assertEquals("B", new FrameBroadPhaseRayTracer3<>(frames, index)
                .firstHit(frames.root(), ray(), 10, (v, r, x, y) -> true).value());
        index.clear();
        assertEquals(0, index.size());
        assertTrue(index.insert(box(2, 3), "C") > reinserted);
        assertFalse(index.updateBounds(a, box(0, 1)));
        assertFalse(index.remove(a));
        log.add(snapshot(index));
        return log;
    }

    private static List<Object> snapshot(MutableRayBroadPhase3<String> index) {
        var boxes = new ArrayList<String>();
        var spheres = new ArrayList<String>();
        var rays = new ArrayList<BroadPhaseRayHit3<String>>();
        var sweeps = new ArrayList<BroadPhaseSweepHit3<String>>();
        index.query(box(0, 10), boxes::add);
        index.querySphere(new Vector3(5, 0.5, 0.5), 5, spheres::add);
        index.queryRay(ray(), 10, rays::add);
        index.querySweptAabb(box(0, 1), new Vector3(10, 0, 0), sweeps::add);
        return List.of(boxes, spheres, rays, sweeps, index.nearest(new Vector3(0, 0, 0), 10));
    }

    private static Ray ray() {
        return new Ray(new Vector3(0, 0.5, 0.5), new Vector3(1, 0, 0));
    }

    private static AxisAlignedBox box(double minX, double maxX) {
        return new AxisAlignedBox(new Vector3(minX, 0, 0), new Vector3(maxX, 1, 1));
    }
}
