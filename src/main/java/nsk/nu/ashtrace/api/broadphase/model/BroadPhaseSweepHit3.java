package nsk.nu.ashtrace.api.broadphase.model;

/**
 * Broad-phase sweep candidate interval over normalized motion time in {@code [0, 1]}.
 */
public record BroadPhaseSweepHit3<T>(T value, double tEnter, double tExit) {
    public BroadPhaseSweepHit3 {
        if (value == null) throw new NullPointerException("value");
        if (Double.isNaN(tEnter) || Double.isInfinite(tEnter) || tEnter < 0.0 || tEnter > 1.0) {
            throw new IllegalArgumentException("tEnter must be finite and in [0, 1]");
        }
        if (Double.isNaN(tExit) || Double.isInfinite(tExit) || tExit < tEnter || tExit > 1.0) {
            throw new IllegalArgumentException("tExit must be finite and in [tEnter, 1]");
        }
    }
}
