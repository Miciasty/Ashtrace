package nsk.nu.ashtrace.api.broadphase.model;

/**
 * Ray/segment broad-phase candidate hit interval.
 */
public record BroadPhaseRayHit3<T>(T value, double tEnter, double tExit) {
    public BroadPhaseRayHit3 {
        if (value == null) throw new NullPointerException("value");
        if (Double.isNaN(tEnter) || Double.isInfinite(tEnter) || tEnter < 0.0) {
            throw new IllegalArgumentException("tEnter must be finite and >= 0");
        }
        if (Double.isNaN(tExit) || Double.isInfinite(tExit) || tExit < tEnter) {
            throw new IllegalArgumentException("tExit must be finite and >= tEnter");
        }
    }
}
