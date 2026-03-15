package nsk.nu.ashtrace.unit.api.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import org.junit.jupiter.api.Test;

import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class RayQueryableBroadPhase3ContractTest {

    @Test
    void query_segment_delegates_to_query_ray_with_segment_length() {
        // GIVEN
        RecordingBroadPhase broadPhase = new RecordingBroadPhase();
        Segment3 segment = new Segment3(
                new Vector3(1.0, 2.0, 3.0),
                new Vector3(4.0, 6.0, 3.0)
        );

        // WHEN
        broadPhase.querySegment(segment, hit -> { });

        // THEN
        assertEquals(segment.a(), broadPhase.lastRay.origin());
        assertEquals(0.6, broadPhase.lastRay.direction().x(), 1e-9);
        assertEquals(0.8, broadPhase.lastRay.direction().y(), 1e-9);
        assertEquals(0.0, broadPhase.lastRay.direction().z(), 1e-9);
        assertEquals(5.0, broadPhase.lastTMax, 1e-9);
    }

    @Test
    void query_segment_rejects_invalid_arguments() {
        // GIVEN
        RecordingBroadPhase broadPhase = new RecordingBroadPhase();
        Segment3 zeroLength = new Segment3(new Vector3(1, 1, 1), new Vector3(1, 1, 1));
        Segment3 nonFinite = new Segment3(new Vector3(0, 0, 0), new Vector3(Double.POSITIVE_INFINITY, 0, 0));

        // WHEN / THEN
        assertThrows(NullPointerException.class, () -> broadPhase.querySegment(null, hit -> { }));
        assertThrows(NullPointerException.class, () -> broadPhase.querySegment(zeroLength, null));
        assertThrows(IllegalArgumentException.class, () -> broadPhase.querySegment(zeroLength, hit -> { }));
        assertThrows(IllegalArgumentException.class, () -> broadPhase.querySegment(nonFinite, hit -> { }));
    }

    private static final class RecordingBroadPhase implements RayQueryableBroadPhase3<String> {
        private Ray lastRay;
        private double lastTMax;

        @Override
        public void query(AxisAlignedBox queryBounds, Consumer<String> consumer) {
            // no-op for this contract test
        }

        @Override
        public void queryRay(Ray ray, double tMax, Consumer<BroadPhaseRayHit3<String>> consumer) {
            this.lastRay = ray;
            this.lastTMax = tMax;
        }
    }
}
