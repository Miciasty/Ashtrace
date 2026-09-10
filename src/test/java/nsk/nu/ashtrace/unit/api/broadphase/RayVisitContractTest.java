package nsk.nu.ashtrace.unit.api.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.TraceQueryBuffer3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicBvhBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class RayVisitContractTest {
    private final Ray ray = new Ray(new Vector3(0, 0.5, 0.5), new Vector3(1, 0, 0));

    @Test
    void all_indexes_visit_the_same_candidates_and_stop_after_requested_callback() {
        for (var index : indexes()) {
            var expected = new ArrayList<BroadPhaseRayHit3<Integer>>();
            index.queryRay(ray, 20, expected::add);
            var actual = new ArrayList<BroadPhaseRayHit3<Integer>>();
            assertTrue(index.visitRay(ray, 20, (value, a, b) -> { actual.add(new BroadPhaseRayHit3<>(value, a, b)); return true; }));
            assertEquals(new HashSet<>(expected), new HashSet<>(actual));
            assertEquals(expected.size(), actual.size());
            var replay = new ArrayList<BroadPhaseRayHit3<Integer>>();
            index.visitRay(ray, 20, (value, a, b) -> { replay.add(new BroadPhaseRayHit3<>(value, a, b)); return true; });
            assertEquals(actual, replay);
            int[] count = {0};
            assertFalse(index.visitRay(ray, 20, (v, a, b) -> ++count[0] < 2));
            assertEquals(2, count[0]);
            assertTrue(index.anyRay(ray, 20));
            assertFalse(index.anyRay(ray, 0));
            assertThrows(IllegalStateException.class, () -> index.visitRay(ray, 20, (v, a, b) -> { throw new IllegalStateException(); }));
            assertTrue(index.anyRay(ray, 20));
        }
    }

    @Test
    void custom_index_fallback_suppresses_callbacks_without_claiming_to_stop_its_scan() {
        int[] emitted = {0};
        var index = new RayQueryableBroadPhase3<Integer>() {
            public void query(AxisAlignedBox box, Consumer<Integer> consumer) {}
            public void queryRay(Ray ray, double max, Consumer<BroadPhaseRayHit3<Integer>> consumer) {
                for (int i = 0; i < 5; i++) { emitted[0]++; consumer.accept(new BroadPhaseRayHit3<>(i, 1, 2)); }
            }
        };
        int[] visited = {0};
        assertFalse(index.visitRay(ray, 10, (v, a, b) -> { visited[0]++; return false; }));
        assertEquals(1, visited[0]);
        assertEquals(5, emitted[0]);
    }

    @Test
    void reusable_buffer_keeps_order_releases_failed_query_state_and_rejects_reentry() {
        var frames = FrameGraph3.worldRoot();
        var tracer = new FrameBroadPhaseRayTracer3<>(frames, indexes().getFirst());
        var buffer = new TraceQueryBuffer3<Integer>();
        var expected = tracer.allHits(frames.root(), ray, 20, (v, r, a, b) -> true);
        var saved = tracer.allHits(frames.root(), ray, 20, (v, r, a, b) -> true, 2, buffer);
        assertEquals(expected.subList(0, 2), saved);
        assertThrows(IllegalStateException.class, () -> tracer.firstHit(frames.root(), ray, 20, (v, r, a, b) -> {
            tracer.firstHit(frames.root(), ray, 20, (x, y, z, w) -> true, buffer);
            return true;
        }, buffer));
        assertThrows(IllegalStateException.class, () -> tracer.firstHit(frames.root(), ray, 20, (v, r, a, b) -> {
            buffer.trimToSize();
            return true;
        }, buffer));
        assertEquals(expected.getFirst(), tracer.firstHit(frames.root(), ray, 20, (v, r, a, b) -> true, buffer));
        tracer.allHits(frames.root(), ray, 20, (v, r, a, b) -> false, 2, buffer);
        assertEquals(expected.subList(0, 2), saved);
        buffer.trimToSize();
        int[] calls = {0};
        assertTrue(tracer.anyHit(frames.root(), ray, 20, (v, r, a, b) -> { calls[0]++; return true; }));
        assertEquals(1, calls[0]);
        var hit = expected.getFirst();
        assertEquals(hit.worldPoint(), hit.worldEnterPoint());
        assertEquals(ray.at(hit.tExit()), hit.worldExitPoint(ray));
    }

    private List<RayQueryableBroadPhase3<Integer>> indexes() {
        var entries = new ArrayList<AabbEntry3<Integer>>();
        var hash = new DynamicSpatialHashBroadPhase3<Integer>(2);
        var dynamic = new DynamicBvhBroadPhase3<Integer>();
        for (int i = 0; i < 12; i++) {
            var box = new AxisAlignedBox(new Vector3(12 - i, 0, 0), new Vector3(13 - i, 1, 1));
            entries.add(new AabbEntry3<>(box, i));
            hash.insert(box, i);
            dynamic.insert(box, i);
        }
        return List.of(new LinearAabbBroadPhase3<>(entries), new BvhAabbBroadPhase3<>(entries), hash, dynamic);
    }
}
