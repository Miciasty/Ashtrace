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
     */
    void queryRay(Ray ray, double tMax, Consumer<BroadPhaseRayHit3<T>> consumer);

    /**
     * Emits broad-phase candidates intersected by finite {@code segment}.
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
