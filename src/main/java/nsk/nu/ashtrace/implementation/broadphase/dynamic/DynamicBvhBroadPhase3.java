package nsk.nu.ashtrace.implementation.broadphase.dynamic;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.contracts.MutableRayBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.internal.BroadPhaseMath3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.BvhAabbBroadPhase3;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Mutable broad-phase that keeps insertion/update API and uses a lazily rebuilt BVH snapshot for queries.
 *
 * <p>Mutations are cheap and only mark snapshot as dirty. The next query rebuilds the snapshot
 * with deterministic BVH build cost.</p>
 */
public final class DynamicBvhBroadPhase3<T> implements MutableRayBroadPhase3<T> {
    private final Map<Long, Entry<T>> entries = new LinkedHashMap<>();
    private long nextHandle = 1L;
    private BvhAabbBroadPhase3<Payload<T>> snapshot = new BvhAabbBroadPhase3<>(List.of());
    private boolean dirty = false;

    @Override
    public long insert(AxisAlignedBox bounds, T value) {
        if (bounds == null) throw new NullPointerException("bounds");
        if (value == null) throw new NullPointerException("value");

        long handle = nextHandle++;
        entries.put(handle, new Entry<>(handle, bounds, value));
        dirty = true;
        return handle;
    }

    @Override
    public boolean updateBounds(long handle, AxisAlignedBox bounds) {
        if (bounds == null) throw new NullPointerException("bounds");
        Entry<T> entry = entries.get(handle);
        if (entry == null) return false;
        entry.bounds = bounds;
        dirty = true;
        return true;
    }

    @Override
    public boolean remove(long handle) {
        Entry<T> removed = entries.remove(handle);
        if (removed == null) return false;
        dirty = true;
        return true;
    }

    @Override
    public int size() {
        return entries.size();
    }

    @Override
    public void clear() {
        entries.clear();
        dirty = true;
    }

    @Override
    public void query(AxisAlignedBox queryBounds, Consumer<T> consumer) {
        if (queryBounds == null) throw new NullPointerException("queryBounds");
        if (consumer == null) throw new NullPointerException("consumer");

        ensureSnapshot();
        snapshot.query(queryBounds, payload -> consumer.accept(payload.value));
    }

    @Override
    public void queryRay(Ray ray, double tMax, Consumer<BroadPhaseRayHit3<T>> consumer) {
        if (ray == null) throw new NullPointerException("ray");
        if (consumer == null) throw new NullPointerException("consumer");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");

        ensureSnapshot();
        snapshot.queryRay(ray, tMax, hit -> consumer.accept(new BroadPhaseRayHit3<>(
                hit.value().value,
                hit.tEnter(),
                hit.tExit()
        )));
    }

    @Override
    public void querySphere(Vector3 center, double radius, Consumer<T> consumer) {
        BroadPhaseMath3.requireFiniteVector(center, "center");
        if (consumer == null) throw new NullPointerException("consumer");
        if (!Double.isFinite(radius) || radius < 0.0) {
            throw new IllegalArgumentException("radius must be finite and >= 0");
        }

        ensureSnapshot();
        snapshot.querySphere(center, radius, payload -> consumer.accept(payload.value));
    }

    @Override
    public BroadPhaseNearestHit3<T> nearest(Vector3 point, double maxDistance) {
        BroadPhaseMath3.requireFiniteVector(point, "point");
        if (!Double.isFinite(maxDistance) || maxDistance < 0.0) {
            throw new IllegalArgumentException("maxDistance must be finite and >= 0");
        }

        ensureSnapshot();
        BroadPhaseNearestHit3<Payload<T>> hit = snapshot.nearest(point, maxDistance);
        if (hit == null) return null;
        return new BroadPhaseNearestHit3<>(hit.value().value, hit.distanceSquared());
    }

    @Override
    public void querySweptAabb(
            AxisAlignedBox movingBounds,
            Vector3 delta,
            Consumer<BroadPhaseSweepHit3<T>> consumer
    ) {
        if (movingBounds == null) throw new NullPointerException("movingBounds");
        BroadPhaseMath3.requireFiniteVector(delta, "delta");
        if (consumer == null) throw new NullPointerException("consumer");

        ensureSnapshot();
        snapshot.querySweptAabb(movingBounds, delta, hit -> consumer.accept(new BroadPhaseSweepHit3<>(
                hit.value().value,
                hit.tEnter(),
                hit.tExit()
        )));
    }

    private void ensureSnapshot() {
        if (!dirty) return;

        ArrayList<AabbEntry3<Payload<T>>> snapshotEntries = new ArrayList<>(entries.size());
        for (Entry<T> entry : entries.values()) {
            snapshotEntries.add(new AabbEntry3<>(entry.bounds, new Payload<>(entry.handle, entry.value)));
        }
        snapshot = new BvhAabbBroadPhase3<>(snapshotEntries);
        dirty = false;
    }

    private static final class Entry<T> {
        private final long handle;
        private AxisAlignedBox bounds;
        private final T value;

        private Entry(long handle, AxisAlignedBox bounds, T value) {
            this.handle = handle;
            this.bounds = bounds;
            this.value = value;
        }
    }

    private static final class Payload<T> {
        private final long handle;
        private final T value;

        private Payload(long handle, T value) {
            this.handle = handle;
            this.value = value;
        }
    }
}
