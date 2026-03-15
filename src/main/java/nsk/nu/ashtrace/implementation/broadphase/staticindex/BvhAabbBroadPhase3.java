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
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;

/**
 * Static BVH broad-phase over AABBs.
 *
 * <p>Build is deterministic for the same input order. Query output order is deterministic but not insertion-based.</p>
 */
public final class BvhAabbBroadPhase3<T> implements
        BroadPhase3<T>,
        RayQueryableBroadPhase3<T>,
        ProximityQueryableBroadPhase3<T>,
        SweepQueryableBroadPhase3<T> {
    private static final int LEAF_SIZE = 8;

    private final List<IndexedEntry<T>> ordered;
    private final Node root;

    public BvhAabbBroadPhase3(List<AabbEntry3<T>> entries) {
        if (entries == null) throw new NullPointerException("entries");

        ArrayList<IndexedEntry<T>> indexed = new ArrayList<>(entries.size());
        for (int i = 0; i < entries.size(); i++) {
            AabbEntry3<T> entry = entries.get(i);
            if (entry == null) throw new NullPointerException("entries[" + i + "]");
            indexed.add(new IndexedEntry<>(entry, i, centroid(entry.bounds())));
        }
        this.ordered = indexed;
        this.root = indexed.isEmpty() ? null : build(0, indexed.size());
    }

    /**
     * Number of indexed entries.
     */
    public int size() {
        return ordered.size();
    }

    @Override
    public void query(AxisAlignedBox queryBounds, Consumer<T> consumer) {
        if (queryBounds == null) throw new NullPointerException("queryBounds");
        if (consumer == null) throw new NullPointerException("consumer");
        if (root == null) return;
        query(root, queryBounds, consumer);
    }

    private void query(Node node, AxisAlignedBox queryBounds, Consumer<T> consumer) {
        if (!BroadPhaseMath3.intersects(node.bounds, queryBounds)) return;

        if (node.isLeaf()) {
            for (int i = node.start; i < node.end; i++) {
                AabbEntry3<T> entry = ordered.get(i).entry;
                if (BroadPhaseMath3.intersects(entry.bounds(), queryBounds)) {
                    consumer.accept(entry.value());
                }
            }
            return;
        }

        query(node.left, queryBounds, consumer);
        query(node.right, queryBounds, consumer);
    }

    @Override
    public void queryRay(Ray ray, double tMax, Consumer<BroadPhaseRayHit3<T>> consumer) {
        if (ray == null) throw new NullPointerException("ray");
        if (consumer == null) throw new NullPointerException("consumer");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");
        if (root == null) return;

        ArrayList<Candidate<T>> candidates = new ArrayList<>();
        queryRay(root, ray, tMax, candidates);
        candidates.sort(Comparator
                .comparingDouble((Candidate<T> c) -> c.tEnter)
                .thenComparingDouble(c -> c.tExit)
                .thenComparingInt(c -> c.stableIndex));
        for (Candidate<T> candidate : candidates) {
            consumer.accept(new BroadPhaseRayHit3<>(candidate.value, candidate.tEnter, candidate.tExit));
        }
    }

    private void queryRay(Node node, Ray ray, double tMax, List<Candidate<T>> out) {
        if (BroadPhaseMath3.rayBoxInterval(ray, node.bounds, tMax) == null) return;

        if (node.isLeaf()) {
            for (int i = node.start; i < node.end; i++) {
                IndexedEntry<T> indexed = ordered.get(i);
                BroadPhaseMath3.Interval interval = BroadPhaseMath3.rayBoxInterval(ray, indexed.entry.bounds(), tMax);
                if (interval != null) {
                    out.add(new Candidate<>(indexed.entry.value(), interval.tEnter(), interval.tExit(), indexed.stableIndex));
                }
            }
            return;
        }

        queryRay(node.left, ray, tMax, out);
        queryRay(node.right, ray, tMax, out);
    }

    @Override
    public void querySphere(Vector3 center, double radius, Consumer<T> consumer) {
        BroadPhaseMath3.requireFiniteVector(center, "center");
        if (consumer == null) throw new NullPointerException("consumer");
        if (!Double.isFinite(radius) || radius < 0.0) {
            throw new IllegalArgumentException("radius must be finite and >= 0");
        }
        if (root == null) return;
        querySphere(root, center, radius, consumer);
    }

    private void querySphere(Node node, Vector3 center, double radius, Consumer<T> consumer) {
        if (!BroadPhaseMath3.intersectsSphere(node.bounds, center, radius)) return;

        if (node.isLeaf()) {
            for (int i = node.start; i < node.end; i++) {
                AabbEntry3<T> entry = ordered.get(i).entry;
                if (BroadPhaseMath3.intersectsSphere(entry.bounds(), center, radius)) {
                    consumer.accept(entry.value());
                }
            }
            return;
        }

        querySphere(node.left, center, radius, consumer);
        querySphere(node.right, center, radius, consumer);
    }

    @Override
    public BroadPhaseNearestHit3<T> nearest(Vector3 point, double maxDistance) {
        BroadPhaseMath3.requireFiniteVector(point, "point");
        if (!Double.isFinite(maxDistance) || maxDistance < 0.0) {
            throw new IllegalArgumentException("maxDistance must be finite and >= 0");
        }
        if (root == null) return null;

        NearestCandidate<T> nearest = new NearestCandidate<>(maxDistance * maxDistance);
        nearest(root, point, nearest);
        if (nearest.value == null) return null;
        return new BroadPhaseNearestHit3<>(nearest.value, nearest.distanceSquared);
    }

    private void nearest(Node node, Vector3 point, NearestCandidate<T> nearest) {
        double nodeDistanceSquared = BroadPhaseMath3.distanceSquaredToBox(point, node.bounds);
        if (nodeDistanceSquared > nearest.distanceSquared) return;

        if (node.isLeaf()) {
            for (int i = node.start; i < node.end; i++) {
                AabbEntry3<T> entry = ordered.get(i).entry;
                double distanceSquared = BroadPhaseMath3.distanceSquaredToBox(point, entry.bounds());
                if (nearest.value == null || distanceSquared < nearest.distanceSquared) {
                    nearest.distanceSquared = distanceSquared;
                    nearest.value = entry.value();
                }
            }
            return;
        }

        double leftDistanceSquared = BroadPhaseMath3.distanceSquaredToBox(point, node.left.bounds);
        double rightDistanceSquared = BroadPhaseMath3.distanceSquaredToBox(point, node.right.bounds);

        if (leftDistanceSquared <= rightDistanceSquared) {
            nearest(node.left, point, nearest);
            nearest(node.right, point, nearest);
        } else {
            nearest(node.right, point, nearest);
            nearest(node.left, point, nearest);
        }
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
        if (root == null) return;

        ArrayList<SweepCandidate<T>> candidates = new ArrayList<>();
        querySweptAabb(root, movingBounds, delta, candidates);
        candidates.sort(Comparator
                .comparingDouble((SweepCandidate<T> c) -> c.tEnter)
                .thenComparingDouble(c -> c.tExit)
                .thenComparingInt(c -> c.stableIndex));
        for (SweepCandidate<T> candidate : candidates) {
            consumer.accept(new BroadPhaseSweepHit3<>(candidate.value, candidate.tEnter, candidate.tExit));
        }
    }

    private void querySweptAabb(
            Node node,
            AxisAlignedBox movingBounds,
            Vector3 delta,
            List<SweepCandidate<T>> out
    ) {
        if (BroadPhaseMath3.sweepAabbInterval(movingBounds, delta, node.bounds) == null) return;

        if (node.isLeaf()) {
            for (int i = node.start; i < node.end; i++) {
                IndexedEntry<T> indexed = ordered.get(i);
                BroadPhaseMath3.Interval interval = BroadPhaseMath3.sweepAabbInterval(
                        movingBounds,
                        delta,
                        indexed.entry.bounds()
                );
                if (interval != null) {
                    out.add(new SweepCandidate<>(
                            indexed.entry.value(),
                            interval.tEnter(),
                            interval.tExit(),
                            indexed.stableIndex
                    ));
                }
            }
            return;
        }

        querySweptAabb(node.left, movingBounds, delta, out);
        querySweptAabb(node.right, movingBounds, delta, out);
    }

    private Node build(int start, int end) {
        AxisAlignedBox bounds = bounds(start, end);
        int count = end - start;
        if (count <= LEAF_SIZE) {
            return Node.leaf(bounds, start, end);
        }

        int axis = longestAxis(bounds);
        Comparator<IndexedEntry<T>> comparator = switch (axis) {
            case 0 -> Comparator.<IndexedEntry<T>>comparingDouble(e -> e.centroid.x()).thenComparingInt(e -> e.stableIndex);
            case 1 -> Comparator.<IndexedEntry<T>>comparingDouble(e -> e.centroid.y()).thenComparingInt(e -> e.stableIndex);
            default -> Comparator.<IndexedEntry<T>>comparingDouble(e -> e.centroid.z()).thenComparingInt(e -> e.stableIndex);
        };
        ordered.subList(start, end).sort(comparator);

        int mid = start + (count / 2);
        Node left = build(start, mid);
        Node right = build(mid, end);
        return Node.branch(bounds, left, right);
    }

    private AxisAlignedBox bounds(int start, int end) {
        AabbEntry3<T> first = ordered.get(start).entry;
        double minX = first.bounds().min().x();
        double minY = first.bounds().min().y();
        double minZ = first.bounds().min().z();
        double maxX = first.bounds().max().x();
        double maxY = first.bounds().max().y();
        double maxZ = first.bounds().max().z();

        for (int i = start + 1; i < end; i++) {
            AxisAlignedBox b = ordered.get(i).entry.bounds();
            minX = Math.min(minX, b.min().x());
            minY = Math.min(minY, b.min().y());
            minZ = Math.min(minZ, b.min().z());
            maxX = Math.max(maxX, b.max().x());
            maxY = Math.max(maxY, b.max().y());
            maxZ = Math.max(maxZ, b.max().z());
        }

        return new AxisAlignedBox(new Vector3(minX, minY, minZ), new Vector3(maxX, maxY, maxZ));
    }

    private static int longestAxis(AxisAlignedBox box) {
        double dx = box.max().x() - box.min().x();
        double dy = box.max().y() - box.min().y();
        double dz = box.max().z() - box.min().z();
        if (dx >= dy && dx >= dz) return 0;
        if (dy >= dz) return 1;
        return 2;
    }

    private static Vector3 centroid(AxisAlignedBox box) {
        return box.min().add(box.max()).mul(0.5);
    }

    private static final class IndexedEntry<T> {
        private final AabbEntry3<T> entry;
        private final int stableIndex;
        private final Vector3 centroid;

        private IndexedEntry(AabbEntry3<T> entry, int stableIndex, Vector3 centroid) {
            this.entry = entry;
            this.stableIndex = stableIndex;
            this.centroid = centroid;
        }
    }

    private static final class Node {
        private final AxisAlignedBox bounds;
        private final int start;
        private final int end;
        private final Node left;
        private final Node right;

        private Node(AxisAlignedBox bounds, int start, int end, Node left, Node right) {
            this.bounds = bounds;
            this.start = start;
            this.end = end;
            this.left = left;
            this.right = right;
        }

        private static Node leaf(AxisAlignedBox bounds, int start, int end) {
            return new Node(bounds, start, end, null, null);
        }

        private static Node branch(AxisAlignedBox bounds, Node left, Node right) {
            return new Node(bounds, -1, -1, left, right);
        }

        private boolean isLeaf() {
            return left == null;
        }
    }

    private record Candidate<T>(T value, double tEnter, double tExit, int stableIndex) {
    }

    private static final class NearestCandidate<T> {
        private double distanceSquared;
        private T value;

        private NearestCandidate(double distanceSquared) {
            this.distanceSquared = distanceSquared;
        }
    }

    private record SweepCandidate<T>(T value, double tEnter, double tExit, int stableIndex) {
    }
}
