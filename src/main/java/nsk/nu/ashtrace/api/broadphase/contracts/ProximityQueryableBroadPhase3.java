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
     */
    void querySphere(Vector3 center, double radius, Consumer<T> consumer);

    /**
     * Returns nearest bounds candidate to {@code point} up to {@code maxDistance}.
     */
    BroadPhaseNearestHit3<T> nearest(Vector3 point, double maxDistance);
}
