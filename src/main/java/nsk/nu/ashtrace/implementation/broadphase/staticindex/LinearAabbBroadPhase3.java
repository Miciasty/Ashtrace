package nsk.nu.ashtrace.implementation.broadphase.staticindex;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashtrace.api.broadphase.contracts.BroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.ProximityQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.contracts.SweepQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseNearestHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseSweepHit3;
import nsk.nu.ashtrace.implementation.broadphase.internal.BroadPhaseMath3;

import java.util.ArrayList;
import java.util.List;
import java.util.Comparator;
import java.util.function.Consumer;

/**
 * Allocation-light broad-phase baseline using deterministic linear scan.
 */
public final class LinearAabbBroadPhase3<T> implements
        BroadPhase3<T>,
        RayQueryableBroadPhase3<T>,
        ProximityQueryableBroadPhase3<T>,
        SweepQueryableBroadPhase3<T> {
    private final List<AabbEntry3<T>> entries;

    public LinearAabbBroadPhase3(List<AabbEntry3<T>> entries) {
        if (entries == null) throw new NullPointerException("entries");
        for (int i = 0; i < entries.size(); i++) {
            if (entries.get(i) == null) throw new NullPointerException("entries[" + i + "]");
        }
        this.entries = List.copyOf(entries);
    }

    /**
     * Number of indexed entries.
     */
    public int size() {
        return entries.size();
    }

    /**
     * Immutable snapshot of indexed entries.
     */
    public List<AabbEntry3<T>> entries() {
        return entries;
    }

    @Override
    public void query(AxisAlignedBox queryBounds, Consumer<T> consumer) {
        if (queryBounds == null) throw new NullPointerException("queryBounds");
        if (consumer == null) throw new NullPointerException("consumer");
        for (AabbEntry3<T> entry : entries) {
            if (BroadPhaseMath3.intersects(queryBounds, entry.bounds())) {
                consumer.accept(entry.value());
            }
        }
    }

    @Override
    public void queryRay(Ray ray, double tMax, Consumer<BroadPhaseRayHit3<T>> consumer) {
        if (ray == null) throw new NullPointerException("ray");
        if (consumer == null) throw new NullPointerException("consumer");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");

        AxisAlignedBox rayBounds = BroadPhaseMath3.rayBounds(ray, tMax);
        for (AabbEntry3<T> entry : entries) {
            if (!BroadPhaseMath3.intersects(rayBounds, entry.bounds())) continue;
            BroadPhaseMath3.Interval interval = BroadPhaseMath3.rayBoxInterval(ray, entry.bounds(), tMax);
            if (interval == null) continue;
            consumer.accept(new BroadPhaseRayHit3<>(entry.value(), interval.tEnter(), interval.tExit()));
        }
    }

    @Override
    public void querySphere(Vector3 center, double radius, Consumer<T> consumer) {
        BroadPhaseMath3.requireFiniteVector(center, "center");
        if (consumer == null) throw new NullPointerException("consumer");
        if (!Double.isFinite(radius) || radius < 0.0) {
            throw new IllegalArgumentException("radius must be finite and >= 0");
        }

        for (AabbEntry3<T> entry : entries) {
            if (BroadPhaseMath3.intersectsSphere(entry.bounds(), center, radius)) {
                consumer.accept(entry.value());
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
        for (AabbEntry3<T> entry : entries) {
            double distanceSquared = BroadPhaseMath3.distanceSquaredToBox(point, entry.bounds());
            if (distanceSquared > bestDistanceSquared) continue;
            if (best == null || distanceSquared < bestDistanceSquared) {
                best = entry.value();
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
        ArrayList<SweepCandidate<T>> candidates = new ArrayList<>();
        for (int i = 0; i < entries.size(); i++) {
            AabbEntry3<T> entry = entries.get(i);
            if (!BroadPhaseMath3.intersects(sweptBounds, entry.bounds())) continue;
            BroadPhaseMath3.Interval interval = BroadPhaseMath3.sweepAabbInterval(movingBounds, delta, entry.bounds());
            if (interval == null) continue;
            candidates.add(new SweepCandidate<>(entry.value(), interval.tEnter(), interval.tExit(), i));
        }

        candidates.sort(Comparator
                .comparingDouble((SweepCandidate<T> c) -> c.tEnter)
                .thenComparingDouble(c -> c.tExit)
                .thenComparingInt(c -> c.order));
        for (SweepCandidate<T> candidate : candidates) {
            consumer.accept(new BroadPhaseSweepHit3<>(candidate.value, candidate.tEnter, candidate.tExit));
        }
    }

    private record SweepCandidate<T>(T value, double tEnter, double tExit, int order) {
    }
}
