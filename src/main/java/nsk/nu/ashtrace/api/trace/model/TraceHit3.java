package nsk.nu.ashtrace.api.trace.model;

import nsk.nu.ashcore.api.math.Vector3;

/**
 * Accepted world-space AABB interval after candidate filtering.
 * {@code tEnter}/{@code tExit} are closed bounds clipped to the query, measured in world distance.
 * {@code worldPoint} is {@code worldRay.at(tEnter)}, the clipped AABB entry point; it may lie
 * inside the box when the ray starts inside. It is not an enclosed object's surface intersection.
 * The payload is retained by reference; the result does not track later index or payload changes.
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
