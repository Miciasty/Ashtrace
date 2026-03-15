package nsk.nu.ashtrace.implementation.broadphase.dynamic;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.contracts.MutableRayBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.internal.BroadPhaseMath3;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

/**
 * Mutable spatial-hash broad-phase for dynamic scenes.
 *
 * <p>Entries are indexed into fixed-size hash cells. Queries are deterministic for the same insertion/update sequence.</p>
 */
public final class DynamicSpatialHashBroadPhase3<T> implements MutableRayBroadPhase3<T> {
    private final double cellSize;
    private final double invCellSize;
    private final Map<Long, Entry<T>> entries = new LinkedHashMap<>();
    private final Map<CellKey, ArrayList<Long>> buckets = new HashMap<>();
    private long nextHandle = 1L;

    public DynamicSpatialHashBroadPhase3(double cellSize) {
        if (!Double.isFinite(cellSize) || cellSize <= 0.0) {
            throw new IllegalArgumentException("cellSize must be finite and > 0");
        }
        this.cellSize = cellSize;
        this.invCellSize = 1.0 / cellSize;
    }

    /**
     * Spatial-hash cell size.
     */
    public double cellSize() {
        return cellSize;
    }

    @Override
    public long insert(AxisAlignedBox bounds, T value) {
        if (bounds == null) throw new NullPointerException("bounds");
        if (value == null) throw new NullPointerException("value");

        long handle = nextHandle++;
        Entry<T> entry = new Entry<>(handle, bounds, value);
        entries.put(handle, entry);
        register(entry);
        return handle;
    }

    @Override
    public boolean updateBounds(long handle, AxisAlignedBox bounds) {
        if (bounds == null) throw new NullPointerException("bounds");
        Entry<T> entry = entries.get(handle);
        if (entry == null) return false;
        unregister(entry);
        entry.bounds = bounds;
        register(entry);
        return true;
    }

    @Override
    public boolean remove(long handle) {
        Entry<T> entry = entries.remove(handle);
        if (entry == null) return false;
        unregister(entry);
        return true;
    }

    @Override
    public int size() {
        return entries.size();
    }

    @Override
    public void clear() {
        entries.clear();
        buckets.clear();
    }

    @Override
    public void query(AxisAlignedBox queryBounds, Consumer<T> consumer) {
        if (queryBounds == null) throw new NullPointerException("queryBounds");
        if (consumer == null) throw new NullPointerException("consumer");

        for (long handle : collectCandidateHandles(queryBounds)) {
            Entry<T> entry = entries.get(handle);
            if (entry == null) continue;
            if (BroadPhaseMath3.intersects(queryBounds, entry.bounds)) {
                consumer.accept(entry.value);
            }
        }
    }

    @Override
    public void queryRay(Ray ray, double tMax, Consumer<BroadPhaseRayHit3<T>> consumer) {
        if (ray == null) throw new NullPointerException("ray");
        if (consumer == null) throw new NullPointerException("consumer");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");

        AxisAlignedBox rayBounds = BroadPhaseMath3.rayBounds(ray, tMax);
        ArrayList<Candidate<T>> hits = new ArrayList<>();

        for (long handle : collectCandidateHandles(rayBounds)) {
            Entry<T> entry = entries.get(handle);
            if (entry == null) continue;
            BroadPhaseMath3.Interval interval = BroadPhaseMath3.rayBoxInterval(ray, entry.bounds, tMax);
            if (interval == null) continue;
            hits.add(new Candidate<>(handle, entry.value, interval.tEnter(), interval.tExit()));
        }

        hits.sort((a, b) -> {
            int c = Double.compare(a.tEnter, b.tEnter);
            if (c != 0) return c;
            c = Double.compare(a.tExit, b.tExit);
            if (c != 0) return c;
            return Long.compare(a.handle, b.handle);
        });

        for (Candidate<T> hit : hits) {
            consumer.accept(new BroadPhaseRayHit3<>(hit.value, hit.tEnter, hit.tExit));
        }
    }

    @Override
    public void querySphere(Vector3 center, double radius, Consumer<T> consumer) {
        BroadPhaseMath3.requireFiniteVector(center, "center");
        if (consumer == null) throw new NullPointerException("consumer");
        if (!Double.isFinite(radius) || radius < 0.0) {
            throw new IllegalArgumentException("radius must be finite and >= 0");
        }

        AxisAlignedBox queryBounds = new AxisAlignedBox(
                new Vector3(center.x() - radius, center.y() - radius, center.z() - radius),
                new Vector3(center.x() + radius, center.y() + radius, center.z() + radius)
        );

        for (long handle : collectCandidateHandles(queryBounds)) {
            Entry<T> entry = entries.get(handle);
            if (entry == null) continue;
            if (BroadPhaseMath3.intersectsSphere(entry.bounds, center, radius)) {
                consumer.accept(entry.value);
            }
        }
    }

    @Override
    public BroadPhaseNearestHit3<T> nearest(Vector3 point, double maxDistance) {
        BroadPhaseMath3.requireFiniteVector(point, "point");
        if (!Double.isFinite(maxDistance) || maxDistance < 0.0) {
            throw new IllegalArgumentException("maxDistance must be finite and >= 0");
        }

        double bestDistanceSquared = maxDistance * maxDistance;
        T best = null;
        for (Entry<T> entry : entries.values()) {
            double distanceSquared = BroadPhaseMath3.distanceSquaredToBox(point, entry.bounds);
            if (distanceSquared > bestDistanceSquared) continue;
            if (best == null || distanceSquared < bestDistanceSquared) {
                best = entry.value;
                bestDistanceSquared = distanceSquared;
            }
        }
        if (best == null) return null;
        return new BroadPhaseNearestHit3<>(best, bestDistanceSquared);
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

        AxisAlignedBox sweptBounds = BroadPhaseMath3.sweptBounds(movingBounds, delta);
        ArrayList<SweepCandidate<T>> hits = new ArrayList<>();
        for (long handle : collectCandidateHandles(sweptBounds)) {
            Entry<T> entry = entries.get(handle);
            if (entry == null) continue;
            BroadPhaseMath3.Interval interval = BroadPhaseMath3.sweepAabbInterval(movingBounds, delta, entry.bounds);
            if (interval == null) continue;
            hits.add(new SweepCandidate<>(handle, entry.value, interval.tEnter(), interval.tExit()));
        }

        hits.sort((a, b) -> {
            int c = Double.compare(a.tEnter, b.tEnter);
            if (c != 0) return c;
            c = Double.compare(a.tExit, b.tExit);
            if (c != 0) return c;
            return Long.compare(a.handle, b.handle);
        });

        for (SweepCandidate<T> hit : hits) {
            consumer.accept(new BroadPhaseSweepHit3<>(hit.value, hit.tEnter, hit.tExit));
        }
    }

    private void register(Entry<T> entry) {
        entry.cells.clear();
        IndexRange r = range(entry.bounds);
        for (int z = r.minZ; z <= r.maxZ; z++) {
            for (int y = r.minY; y <= r.maxY; y++) {
                for (int x = r.minX; x <= r.maxX; x++) {
                    CellKey key = new CellKey(x, y, z);
                    buckets.computeIfAbsent(key, k -> new ArrayList<>()).add(entry.handle);
                    entry.cells.add(key);
                }
            }
        }
    }

    private void unregister(Entry<T> entry) {
        for (CellKey key : entry.cells) {
            ArrayList<Long> handles = buckets.get(key);
            if (handles == null) continue;
            handles.remove(entry.handle);
            if (handles.isEmpty()) {
                buckets.remove(key);
            }
        }
        entry.cells.clear();
    }

    private List<Long> collectCandidateHandles(AxisAlignedBox queryBounds) {
        IndexRange r = range(queryBounds);
        HashSet<Long> unique = new HashSet<>();

        for (int z = r.minZ; z <= r.maxZ; z++) {
            for (int y = r.minY; y <= r.maxY; y++) {
                for (int x = r.minX; x <= r.maxX; x++) {
                    ArrayList<Long> handles = buckets.get(new CellKey(x, y, z));
                    if (handles == null) continue;
                    unique.addAll(handles);
                }
            }
        }

        ArrayList<Long> sorted = new ArrayList<>(unique);
        sorted.sort(Long::compareTo);
        return sorted;
    }

    private IndexRange range(AxisAlignedBox bounds) {
        return new IndexRange(
                toIndex(bounds.min().x()),
                toIndex(bounds.min().y()),
                toIndex(bounds.min().z()),
                toIndex(bounds.max().x()),
                toIndex(bounds.max().y()),
                toIndex(bounds.max().z())
        );
    }

    private int toIndex(double value) {
        return (int) Math.floor(value * invCellSize);
    }

    private record CellKey(int x, int y, int z) {
    }

    private record IndexRange(int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
    }

    private static final class Entry<T> {
        private final long handle;
        private AxisAlignedBox bounds;
        private final T value;
        private final ArrayList<CellKey> cells = new ArrayList<>();

        private Entry(long handle, AxisAlignedBox bounds, T value) {
            this.handle = handle;
            this.bounds = bounds;
            this.value = value;
        }
    }

    private static final class Candidate<T> {
        private final long handle;
        private final T value;
        private final double tEnter;
        private final double tExit;

        private Candidate(long handle, T value, double tEnter, double tExit) {
            this.handle = handle;
            this.value = value;
            this.tEnter = tEnter;
            this.tExit = tExit;
        }
    }

    private static final class SweepCandidate<T> {
        private final long handle;
        private final T value;
        private final double tEnter;
        private final double tExit;

        private SweepCandidate(long handle, T value, double tEnter, double tExit) {
            this.handle = handle;
            this.value = value;
            this.tEnter = tEnter;
            this.tExit = tExit;
        }
    }
}
