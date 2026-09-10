package nsk.nu.ashtrace.api.trace.model;

import nsk.nu.ashcore.api.math.Vector3;

/**
 * A geometry-provider intersection restricted to the closed query/occlusion interval.
 * Both points are world positions and both parameters are world distances. Surface flags
 * distinguish actual shape boundaries from clipped endpoints: starting inside sets
 * enterSurface=false, and stopping before the full exit sets exitSurface=false.
 * "Exact" distinguishes the provider's shape from its AABB; numerical accuracy and the
 * shape intersection itself remain the provider's responsibility. Payloads are retained.
 */
public record ExactTraceHit3<T>(
        T value, double tEnter, double tExit,
        Vector3 worldEnterPoint, Vector3 worldExitPoint,
        boolean enterSurface, boolean exitSurface
) {
    public ExactTraceHit3 {
        if (value == null) throw new NullPointerException("value");
        requireFinite(worldEnterPoint, "worldEnterPoint");
        requireFinite(worldExitPoint, "worldExitPoint");
        if (!Double.isFinite(tEnter) || !Double.isFinite(tExit) || tEnter < 0.0 || tExit < tEnter) {
            throw new IllegalArgumentException("hit endpoints must be finite with 0 <= tEnter <= tExit");
        }
    }

    /** Distance inside this interval of the query; it can be truncated and is not material thickness. */
    public double distanceInside() {
        return tExit - tEnter;
    }

    private static void requireFinite(Vector3 point, String name) {
        if (point == null) throw new NullPointerException(name);
        if (!Double.isFinite(point.x()) || !Double.isFinite(point.y()) || !Double.isFinite(point.z())) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }
}
