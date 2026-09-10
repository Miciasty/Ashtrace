package nsk.nu.ashtrace.api.broadphase.contracts;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;

import java.util.function.Consumer;

/**
 * Broad-phase contract that supports interval candidates for ray/segment queries.
 */
public interface RayQueryableBroadPhase3<T> extends BroadPhase3<T> {

    /**
     * Emits broad-phase candidates intersected by {@code ray} up to {@code tMax}.
     *
     * <p>{@code tMax} is a finite world-space distance along the normalized ray direction,
     * in closed interval {@code [0, tMax]}.</p>
     *
     * <p>Each emitted hit uses boundary-inclusive interval {@code [tEnter, tExit]} where
     * both values are in world-space distance units from {@code ray.origin()}.</p>
     * <p>At tMax=0, boxes containing the origin match with [0,0]. Neither zero nor very small
     * nonzero direction components are replaced by a tolerance. Ordering is implementation-specific.</p>
     */
    void queryRay(Ray ray, double tMax, Consumer<BroadPhaseRayHit3<T>> consumer);

    /**
     * Visit candidates until the visitor returns false. Returns true if enumeration completed,
     * false if a stop was requested, including on the last candidate. No nearest-first order is promised.
     * Built-in indexes stop their traversal/filtering; a custom implementation inheriting this
     * fallback still completes queryRay internally but makes no further visitor calls after stopping.
     * Callback exceptions propagate. The stable-state rules of BroadPhase3 apply.
     */
    default boolean visitRay(Ray ray, double tMax, RayCandidateVisitor3<T> visitor) {
        if (visitor == null) throw new NullPointerException("visitor");
        final boolean[] completed = {true};
        queryRay(ray, tMax, hit -> {
            if (completed[0]) completed[0] = visitor.visit(hit.value(), hit.tEnter(), hit.tExit());
        });
        return completed[0];
    }

    /** Whether any indexed AABB intersects the closed ray interval. */
    default boolean anyRay(Ray ray, double tMax) {
        return !visitRay(ray, tMax, (value, enter, exit) -> false);
    }

    /**
     * Emits broad-phase candidates intersected by finite {@code segment}.
     *
     * <p>Equivalent to {@code queryRay} with ray origin at {@code segment.a()},
     * ray direction toward {@code segment.b()}, and {@code tMax = segment.length()}.</p>
     */
    default void querySegment(Segment3 segment, Consumer<BroadPhaseRayHit3<T>> consumer) {
        if (segment == null) throw new NullPointerException("segment");
        if (consumer == null) throw new NullPointerException("consumer");

        var delta = segment.b().sub(segment.a());
        double length = delta.length();
        if (!Double.isFinite(length) || length <= 0.0) {
            throw new IllegalArgumentException("segment length must be finite and > 0");
        }

        queryRay(new Ray(segment.a(), delta), length, consumer);
    }
}
