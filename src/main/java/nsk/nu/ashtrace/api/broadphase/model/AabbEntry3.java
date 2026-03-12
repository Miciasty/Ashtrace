package nsk.nu.ashtrace.api.broadphase.model;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;

/**
 * Broad-phase entry represented by world-space bounds and payload value.
 */
public record AabbEntry3<T>(AxisAlignedBox bounds, T value) {
    public AabbEntry3 {
        if (bounds == null) throw new NullPointerException("bounds");
        if (value == null) throw new NullPointerException("value");
    }
}
