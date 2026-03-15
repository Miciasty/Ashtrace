package nsk.nu.ashtrace.api.broadphase.contracts;

import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;

import java.util.function.Consumer;

/**
 * Broad-phase proximity query contract.
 */
public interface ProximityQueryableBroadPhase3<T> extends BroadPhase3<T> {

    /**
     * Emits values whose bounds intersect sphere centered at {@code center} with {@code radius}.
     *
     * <p>{@code center} and {@code radius} are in world-space units.
     * Sphere-vs-AABB test is boundary-inclusive.</p>
     */
    void querySphere(Vector3 center, double radius, Consumer<T> consumer);

    /**
     * Returns nearest bounds candidate to {@code point} up to {@code maxDistance}.
     *
     * <p>{@code maxDistance} is in world-space units and is inclusive.
     * Returned distance is squared Euclidean distance in world-space units squared.</p>
     */
    BroadPhaseNearestHit3<T> nearest(Vector3 point, double maxDistance);
}
