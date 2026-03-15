package nsk.nu.ashtrace.unit.api.model;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.api.trace.model.GridRayHit3;
import nsk.nu.ashtrace.api.trace.model.TraceHit3;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class TraceModelsApiTest {

    @Test
    void records_accept_valid_boundary_values() {
        // GIVEN / WHEN
        AabbEntry3<String> entry = new AabbEntry3<>(box(0, 0, 0, 1, 1, 1), "A");
        BroadPhaseRayHit3<String> rayHit = new BroadPhaseRayHit3<>("A", 0.0, 0.0);
        BroadPhaseNearestHit3<String> nearestHit = new BroadPhaseNearestHit3<>("A", 0.0);
        BroadPhaseSweepHit3<String> sweepHit = new BroadPhaseSweepHit3<>("A", 0.0, 1.0);
        GridRayHit3 gridHit = new GridRayHit3(1, 2, 3, 0.0, 0.0, new Vector3(1, 2, 3));
        TraceHit3<String> traceHit = new TraceHit3<>("A", 0.0, 0.0, new Vector3(1, 2, 3));

        // THEN
        assertEquals("A", entry.value());
        assertEquals(0.0, rayHit.tEnter(), 1e-9);
        assertEquals(0.0, nearestHit.distanceSquared(), 1e-9);
        assertEquals(1.0, sweepHit.tExit(), 1e-9);
        assertEquals(1, gridHit.x());
        assertEquals("A", traceHit.value());
    }

    @Test
    void records_reject_null_arguments() {
        // WHEN / THEN
        assertThrows(NullPointerException.class, () -> new AabbEntry3<String>(null, "A"));
        assertThrows(NullPointerException.class, () -> new AabbEntry3<>(box(0, 0, 0, 1, 1, 1), null));
        assertThrows(NullPointerException.class, () -> new BroadPhaseRayHit3<String>(null, 0.0, 1.0));
        assertThrows(NullPointerException.class, () -> new BroadPhaseNearestHit3<String>(null, 0.0));
        assertThrows(NullPointerException.class, () -> new BroadPhaseSweepHit3<String>(null, 0.0, 1.0));
        assertThrows(NullPointerException.class, () -> new GridRayHit3(1, 1, 1, 0.0, 1.0, null));
        assertThrows(NullPointerException.class, () -> new TraceHit3<String>(null, 0.0, 1.0, new Vector3(0, 0, 0)));
        assertThrows(NullPointerException.class, () -> new TraceHit3<>("A", 0.0, 1.0, null));
    }

    @Test
    void records_reject_invalid_ranges_and_non_finite_values() {
        // WHEN / THEN
        assertThrows(IllegalArgumentException.class, () -> new BroadPhaseRayHit3<>("A", -0.1, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new BroadPhaseRayHit3<>("A", Double.POSITIVE_INFINITY, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new BroadPhaseRayHit3<>("A", 1.0, 0.5));
        assertThrows(IllegalArgumentException.class, () -> new BroadPhaseNearestHit3<>("A", -1.0));
        assertThrows(IllegalArgumentException.class, () -> new BroadPhaseNearestHit3<>("A", Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new BroadPhaseSweepHit3<>("A", -0.1, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new BroadPhaseSweepHit3<>("A", 0.1, 1.1));
        assertThrows(IllegalArgumentException.class, () -> new BroadPhaseSweepHit3<>("A", 0.8, 0.7));
        assertThrows(IllegalArgumentException.class, () -> new GridRayHit3(1, 1, 1, Double.NaN, 1.0, new Vector3(0, 0, 0)));
        assertThrows(IllegalArgumentException.class, () -> new GridRayHit3(1, 1, 1, 1.0, 0.5, new Vector3(0, 0, 0)));
        assertThrows(IllegalArgumentException.class, () -> new TraceHit3<>("A", 1.0, 0.5, new Vector3(0, 0, 0)));
        assertThrows(IllegalArgumentException.class, () -> new TraceHit3<>("A", Double.NEGATIVE_INFINITY, 1.0, new Vector3(0, 0, 0)));
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
