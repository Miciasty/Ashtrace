package nsk.nu.ashtrace.api.trace.contracts;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;

import java.util.function.Consumer;

/**
 * Supplies shape intersections for a broad-phase candidate. All coordinates and distances use world units.
 * Emit zero or more full, finite, disjoint inside intervals intersecting [tMin,tMax], in any repeatable order.
 * Report the actual boundary parameters, even when entry is negative or exit exceeds tMax;
 * the tracer clips them and marks the corresponding endpoint as non-surface.
 * A single surface may be represented as [t,t]. For multiple parts, the provider must merge
 * overlapping intervals without merging across a cavity. The tracer does not compute their union.
 *
 * <p>The shape must be enclosed by its indexed AABB. The tracer checks the reported interval's
 * query-clipped portion against the candidate interval, but cannot verify shape geometry or
 * out-of-query boundaries. Calculations may use Ashcore or caller-owned geometry providers.
 * Emit synchronously; do not retain the output consumer or mutate query state from callbacks.</p>
 *
 * <p>Intervals describe one stable pose, not motion between poses. Ashcore's full OBB ray interval
 * can be forwarded in world distances; its clipped segment fractions cannot be forwarded directly.
 * Refresh both indexed bounds and provider geometry after a frame or shape change.</p>
 */
@FunctionalInterface
public interface RayIntersector3<T> {
    void intersect(T value, Ray worldRay, double tMin, double tMax, Consumer<RayIntersection3> output);
}
