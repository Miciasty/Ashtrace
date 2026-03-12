package nsk.nu.ashtrace.api.broadphase.contracts;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;

import java.util.function.Consumer;

/**
 * Deterministic broad-phase query contract over axis-aligned bounds.
 */
public interface BroadPhase3<T> {

    /**
     * Emits values whose bounds intersect {@code queryBounds}.
     */
    void query(AxisAlignedBox queryBounds, Consumer<T> consumer);
}
