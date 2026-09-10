package nsk.nu.ashtrace.api.trace.model;

import nsk.nu.ashcore.api.math.Vector3;

/**
 * Voxel hit with grid-local cell indices, world-distance parameters and a world entry point.
 * With the unit-world constructor, cell indices are also world unit-grid coordinates.
 */
public record GridRayHit3(int x, int y, int z, double tEnter, double tExit, Vector3 worldPoint) {
    public GridRayHit3 {
        if (worldPoint == null) throw new NullPointerException("worldPoint");
        if (Double.isNaN(tEnter) || Double.isInfinite(tEnter) || tEnter < 0.0) {
            throw new IllegalArgumentException("tEnter must be finite and >= 0");
        }
        if (Double.isNaN(tExit) || Double.isInfinite(tExit) || tExit < tEnter) {
            throw new IllegalArgumentException("tExit must be finite and >= tEnter");
        }
    }
}
