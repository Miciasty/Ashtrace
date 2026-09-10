package nsk.nu.ashtrace.unit.implementation.broadphase;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.implementation.broadphase.dynamic.DynamicSpatialHashBroadPhase3;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SpatialHashLimitsTest {

    @Test
    void int_endpoints_are_visited_without_wrapping() {
        var index = new DynamicSpatialHashBroadPhase3<String>(1);
        for (double coordinate : new double[]{Integer.MIN_VALUE, Integer.MAX_VALUE}) {
            var bounds = point(coordinate);
            long handle = index.insert(bounds, "point");
            var hits = new ArrayList<String>();
            index.query(bounds, hits::add);
            assertEquals(List.of("point"), hits);
            assertTrue(index.remove(handle));
        }
    }

    @Test
    void rejected_range_does_not_remove_old_bounds_or_consume_handle() {
        var index = new DynamicSpatialHashBroadPhase3<String>(1);
        long handle = index.insert(point(0), "old");
        for (var bounds : List.of(point((double) Integer.MAX_VALUE + 1), point(Double.NaN),
                new AxisAlignedBox(new Vector3(-50000, -50000, 0), new Vector3(50000, 50000, 0)))) {
            assertThrows(IllegalArgumentException.class, () -> index.updateBounds(handle, bounds));
            assertThrows(IllegalArgumentException.class, () -> index.insert(bounds, "invalid"));
            assertEquals(1, index.size());
            assertEquals("old", index.nearest(new Vector3(0, 0, 0), 0).value());
            var hits = new ArrayList<String>();
            index.query(point(0), hits::add);
            assertEquals(List.of("old"), hits);
        }
        assertEquals(handle + 1, index.insert(point(1), "next"));
    }

    @Test
    void subnormal_cell_size_uses_division_without_infinite_reciprocal() {
        var index = new DynamicSpatialHashBroadPhase3<String>(Double.MIN_VALUE);
        var bounds = point(Double.MIN_VALUE);
        index.insert(bounds, "point");
        var hits = new ArrayList<String>();
        index.query(bounds, hits::add);
        assertEquals(List.of("point"), hits);
        assertThrows(IllegalArgumentException.class, () -> index.query(point(1), hits::add));
    }

    private static AxisAlignedBox point(double x) {
        var point = new Vector3(x, 0, 0);
        return new AxisAlignedBox(point, point);
    }
}
