package nsk.nu.ashtrace.api.broadphase.model;

/**
 * Nearest broad-phase candidate result.
 */
public record BroadPhaseNearestHit3<T>(T value, double distanceSquared) {
    public BroadPhaseNearestHit3 {
        if (value == null) throw new NullPointerException("value");
        if (Double.isNaN(distanceSquared) || Double.isInfinite(distanceSquared) || distanceSquared < 0.0) {
            throw new IllegalArgumentException("distanceSquared must be finite and >= 0");
        }
    }
}
