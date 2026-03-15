package nsk.nu.ashtrace.api.broadphase.contracts;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;

import java.util.function.Consumer;

/**
 * Broad-phase contract that supports sweeping an AABB through space.
 */
public interface SweepQueryableBroadPhase3<T> extends BroadPhase3<T> {

    /**
     * Emits broad-phase candidates intersected by {@code movingBounds} swept by {@code delta}.
     *
     * <p>{@code movingBounds} is world-space AABB (inclusive bounds), {@code delta} is world-space motion vector,
     * and returned intervals are boundary-inclusive normalized sweep time values in {@code [0, 1]}.</p>
     */
    void querySweptAabb(
            AxisAlignedBox movingBounds,
            Vector3 delta,
            Consumer<BroadPhaseSweepHit3<T>> consumer
    );
}
