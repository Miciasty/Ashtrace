package nsk.nu.ashtrace.api.trace.model;

/**
 * One full connected interval inside a bounded shape on the ray's supporting line.
 * Parameters are finite signed world distances; negative parameters lie behind the origin.
 * An interval spanning zero contains the origin. Endpoints describe actual shape boundaries,
 * before query/occlusion clipping.
 * A zero-length interval represents a tangent or a surface-only intersection.
 * Geometry providers must report separate intervals for disconnected parts and cavities.
 */
public record RayIntersection3(double tEnter, double tExit) {
    public RayIntersection3 {
        if (!Double.isFinite(tEnter) || !Double.isFinite(tExit) || tExit < tEnter) {
            throw new IllegalArgumentException("intersection endpoints must be finite and ordered");
        }
    }
}
