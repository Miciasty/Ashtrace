package nsk.nu.ashtrace.api.broadphase.contracts;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;

/**
 * Mutable broad-phase contract for dynamic scenes.
 * Handles are monotonically increasing positive IDs, never reused by remove or clear.
 * Updates retain the handle and insertion position; removal and reinsertion create a new position.
 * Implementations reject handle exhaustion rather than wrap. All operations require external
 * synchronization when shared, including queries and lazy snapshot rebuilds.
 */
public interface MutableRayBroadPhase3<T> extends
        RayQueryableBroadPhase3<T>,
        ProximityQueryableBroadPhase3<T>,
        SweepQueryableBroadPhase3<T> {

    /**
     * Insert new entry and return stable handle id.
     */
    long insert(AxisAlignedBox bounds, T value);

    /**
     * Update bounds for existing handle.
     *
     * @return true if handle exists and was updated
     */
    boolean updateBounds(long handle, AxisAlignedBox bounds);

    /**
     * Remove entry by handle.
     *
     * @return true if handle existed and was removed
     */
    boolean remove(long handle);

    /**
     * Number of active entries.
     */
    int size();

    /**
     * Remove all entries.
     */
    void clear();
}
