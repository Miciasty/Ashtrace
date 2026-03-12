package nsk.nu.ashtrace.api.trace.model;

import nsk.nu.ashcore.api.math.Vector3;

/**
 * Final trace hit after broad-phase candidate selection and optional narrow-phase filtering.
 */
public record TraceHit3<T>(T value, double tEnter, double tExit, Vector3 worldPoint) {
    public TraceHit3 {
        if (value == null) throw new NullPointerException("value");
        if (worldPoint == null) throw new NullPointerException("worldPoint");
        if (Double.isNaN(tEnter) || Double.isInfinite(tEnter) || tEnter < 0.0) {
            throw new IllegalArgumentException("tEnter must be finite and >= 0");
        }
        if (Double.isNaN(tExit) || Double.isInfinite(tExit) || tExit < tEnter) {
            throw new IllegalArgumentException("tExit must be finite and >= tEnter");
        }
    }
}
