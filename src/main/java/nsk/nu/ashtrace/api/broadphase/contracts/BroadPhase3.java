package nsk.nu.ashtrace.api.broadphase.contracts;

import nsk.nu.ashcore.api.geometry.AxisAlignedBox;

import java.util.function.Consumer;

/**
 * Deterministic broad-phase query contract over axis-aligned bounds.
 * Results describe the indexed AABBs, which are only candidates for enclosed geometry.
 * Every matching entry is emitted once, even when payload values compare equal.
 * All bounds must be finite; zero extents are valid closed boxes.
 *
 * <p>Ordering is implementation-specific. Repeatability requires the same implementation and
 * dependency versions, configuration, ordered inputs, mutation history and deterministic callbacks
 * in the same Java environment. Cross-index candidate sets agree, but emission order and nearest
 * tie winners need not. There is no insertion-order-independent or cross-version bitwise guarantee.</p>
 *
 * <p>Do not mutate an index during queries or their callbacks. Mutable implementations are not
 * thread-safe, including concurrent reads that may rebuild a snapshot. Static indexes can be shared
 * after safe publication if callbacks and payload access are thread-safe. No index copies payloads.</p>
 */
public interface BroadPhase3<T> {

    /**
     * Emits values whose world-space bounds intersect {@code queryBounds}.
     *
     * <p>AABB intersection is boundary-inclusive on all axes
     * (touching faces/edges/corners counts as intersection).</p>
     */
    void query(AxisAlignedBox queryBounds, Consumer<T> consumer);
}
