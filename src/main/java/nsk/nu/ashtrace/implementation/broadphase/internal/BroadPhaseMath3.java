package nsk.nu.ashtrace.implementation.broadphase.internal;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.NumericTolerance;
import nsk.nu.ashcore.api.math.Vector3;

public final class BroadPhaseMath3 {
    private BroadPhaseMath3() {}

    public static boolean intersects(AxisAlignedBox a, AxisAlignedBox b) {
        if (a.max().x() < b.min().x() || b.max().x() < a.min().x()) return false;
        if (a.max().y() < b.min().y() || b.max().y() < a.min().y()) return false;
        if (a.max().z() < b.min().z() || b.max().z() < a.min().z()) return false;
        return true;
    }

    public static AxisAlignedBox rayBounds(Ray ray, double tMax) {
        Vector3 a = ray.origin();
        Vector3 b = ray.at(tMax);
        return new AxisAlignedBox(
                new Vector3(Math.min(a.x(), b.x()), Math.min(a.y(), b.y()), Math.min(a.z(), b.z())),
                new Vector3(Math.max(a.x(), b.x()), Math.max(a.y(), b.y()), Math.max(a.z(), b.z()))
        );
    }

    public static AxisAlignedBox sweptBounds(AxisAlignedBox movingBounds, Vector3 delta) {
        Vector3 min = movingBounds.min();
        Vector3 max = movingBounds.max();
        Vector3 movedMin = min.add(delta);
        Vector3 movedMax = max.add(delta);
        return new AxisAlignedBox(
                new Vector3(
                        Math.min(min.x(), movedMin.x()),
                        Math.min(min.y(), movedMin.y()),
                        Math.min(min.z(), movedMin.z())
                ),
                new Vector3(
                        Math.max(max.x(), movedMax.x()),
                        Math.max(max.y(), movedMax.y()),
                        Math.max(max.z(), movedMax.z())
                )
        );
    }

    public static boolean intersectsSphere(AxisAlignedBox box, Vector3 center, double radius) {
        double d2 = distanceSquaredToBox(center, box);
        return d2 <= radius * radius;
    }

    public static double distanceSquaredToBox(Vector3 point, AxisAlignedBox box) {
        double dx = axisDistance(point.x(), box.min().x(), box.max().x());
        double dy = axisDistance(point.y(), box.min().y(), box.max().y());
        double dz = axisDistance(point.z(), box.min().z(), box.max().z());
        return dx * dx + dy * dy + dz * dz;
    }

    public static Interval sweepAabbInterval(AxisAlignedBox movingBounds, Vector3 delta, AxisAlignedBox targetBounds) {
        double tEnter = Double.NEGATIVE_INFINITY;
        double tExit = Double.POSITIVE_INFINITY;

        for (int axis = 0; axis < 3; axis++) {
            double movingMin = component(movingBounds.min(), axis);
            double movingMax = component(movingBounds.max(), axis);
            double targetMin = component(targetBounds.min(), axis);
            double targetMax = component(targetBounds.max(), axis);
            double motion = component(delta, axis);

            if (Math.abs(motion) <= NumericTolerance.GEOMETRY_EPS) {
                if (movingMax < targetMin || movingMin > targetMax) return null;
                continue;
            }

            double t0 = (targetMin - movingMax) / motion;
            double t1 = (targetMax - movingMin) / motion;
            if (t0 > t1) {
                double tmp = t0;
                t0 = t1;
                t1 = tmp;
            }

            tEnter = Math.max(tEnter, t0);
            tExit = Math.min(tExit, t1);
            if (tExit < tEnter) return null;
        }

        if (tExit < 0.0 || tEnter > 1.0) return null;
        double enter = Math.max(0.0, tEnter);
        double exit = Math.min(1.0, tExit);
        if (exit < enter) return null;
        return new Interval(enter, exit);
    }

    public static void requireFiniteVector(Vector3 vector, String name) {
        if (vector == null) throw new NullPointerException(name);
        if (!Double.isFinite(vector.x()) || !Double.isFinite(vector.y()) || !Double.isFinite(vector.z())) {
            throw new IllegalArgumentException(name + " must have finite components");
        }
    }

    public static Interval rayBoxInterval(Ray ray, AxisAlignedBox box, double tMax) {
        double tEnter = Double.NEGATIVE_INFINITY;
        double tExit = Double.POSITIVE_INFINITY;

        for (int axis = 0; axis < 3; axis++) {
            double o = component(ray.origin(), axis);
            double d = component(ray.direction(), axis);
            double min = component(box.min(), axis);
            double max = component(box.max(), axis);

            if (Math.abs(d) <= NumericTolerance.GEOMETRY_EPS) {
                if (o < min || o > max) return null;
                continue;
            }

            double inv = 1.0 / d;
            double t0 = (min - o) * inv;
            double t1 = (max - o) * inv;
            if (t0 > t1) {
                double tmp = t0;
                t0 = t1;
                t1 = tmp;
            }

            tEnter = Math.max(tEnter, t0);
            tExit = Math.min(tExit, t1);
            if (tExit < tEnter) return null;
        }

        if (tExit < 0.0 || tEnter > tMax) return null;
        double enter = Math.max(0.0, tEnter);
        double exit = Math.min(tMax, tExit);
        if (exit < enter) return null;
        return new Interval(enter, exit);
    }

    private static double component(Vector3 v, int axis) {
        return axis == 0 ? v.x() : axis == 1 ? v.y() : v.z();
    }

    private static double axisDistance(double value, double min, double max) {
        if (value < min) return min - value;
        if (value > max) return value - max;
        return 0.0;
    }

    public record Interval(double tEnter, double tExit) {}
}
