package nsk.nu.ashtrace.api.trace.pipeline;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.space.SpaceConverter3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.ExactTraceHit3;
import nsk.nu.ashtrace.api.trace.model.RayIntersection3;

import java.util.Comparator;
import java.util.List;
import java.util.function.Predicate;

/**
 * Selects provider-supplied shape intervals rather than accepted AABB distances.
 * Queries require stable frames, index, provider geometry and callbacks. No locks or automatic snapshots.
 * The provider owns shape correctness; broad-phase bounds must enclose every reported shape part.
 *
 * <p>firstHit minimizes clipped entry, then exit. lastHit maximizes clipped exit, then entry:
 * it is the last interval reached, not the entry of the last object in allHits. Full ties retain
 * visitRay candidate order, then provider emission order. allHits uses stable entry/exit ordering.
 * Starting inside yields entry zero with enterSurface=false; a clipped exit is marked similarly.
 * One object can have multiple intervals, e.g. on either side of a cavity. No collision response is computed.</p>
 *
 * <p>For Q index traversal work, F total provider work and M emitted intervals, first/last cost
 * O(Q + F + M), with O(1) selection state plus index workspace. allHits also sorts O(M log M)
 * and uses O(M) storage. anyHit stops later candidates at the first accepted interval; the active
 * provider call still runs to completion, and custom indexes may use the non-stopping fallback.</p>
 */
public final class FrameExactRayTracer3<T> {
    private final FrameGraph3 frames;
    private final SpaceConverter3 converter;
    private final RayQueryableBroadPhase3<T> broadPhase;

    public FrameExactRayTracer3(FrameGraph3 frames, RayQueryableBroadPhase3<T> broadPhase) {
        if (frames == null) throw new NullPointerException("frames");
        if (broadPhase == null) throw new NullPointerException("broadPhase");
        this.frames = frames;
        this.converter = new SpaceConverter3(frames);
        this.broadPhase = broadPhase;
    }

    public FrameGraph3 frames() {
        return frames;
    }

    public RayQueryableBroadPhase3<T> broadPhase() {
        return broadPhase;
    }

    /** First clipped shape interval in entry/exit order, or null. Both boundary points are returned. */
    public ExactTraceHit3<T> firstHit(FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector) {
        return select(sourceFrame, sourceRay, tMax, intersector, false);
    }

    /** Last clipped shape interval by exit, then entry, or null. Inspect exitSurface before treating its endpoint as a surface. */
    public ExactTraceHit3<T> lastHit(FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector) {
        return select(sourceFrame, sourceRay, tMax, intersector, true);
    }

    /** Whether a provider reports any shape interval in [0,tMax]. Does not select the nearest object. */
    public boolean anyHit(FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector) {
        return !visit(sourceFrame, sourceRay, tMax, intersector, hit -> false);
    }

    /** All clipped intervals sorted by entry/exit, in an immutable independent list. */
    public List<ExactTraceHit3<T>> allHits(FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector) {
        return allHits(sourceFrame, sourceRay, tMax, intersector, Integer.MAX_VALUE, new TraceQueryBuffer3<>());
    }

    /** First maxHits intervals in exact order. Providers are fully queried before sorting and truncation. */
    public List<ExactTraceHit3<T>> allHits(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector, int maxHits
    ) {
        return allHits(sourceFrame, sourceRay, tMax, intersector, maxHits, new TraceQueryBuffer3<>());
    }

    /** As allHits, reusing temporary list capacity. maxHits must be positive; buffer is exclusive to the call. */
    public List<ExactTraceHit3<T>> allHits(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector,
            int maxHits, TraceQueryBuffer3<T> buffer
    ) {
        if (maxHits <= 0) throw new IllegalArgumentException("maxHits must be > 0");
        if (buffer == null) throw new NullPointerException("buffer");
        buffer.begin();
        try {
            visit(sourceFrame, sourceRay, tMax, intersector, hit -> { buffer.exactHits.add(hit); return true; });
            buffer.exactHits.sort(Comparator.comparingDouble((ExactTraceHit3<T> hit) -> hit.tEnter())
                    .thenComparingDouble(ExactTraceHit3::tExit));
            return List.copyOf(buffer.exactHits.subList(0, Math.min(maxHits, buffer.exactHits.size())));
        } finally {
            buffer.end();
        }
    }

    /** First interval on a finite, nonzero closed segment, or null. */
    public ExactTraceHit3<T> firstSegmentHit(FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector) {
        double length = segmentLength(segment);
        return firstHit(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, intersector);
    }

    /** Last interval on a finite, nonzero closed segment, or null. */
    public ExactTraceHit3<T> lastSegmentHit(FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector) {
        double length = segmentLength(segment);
        return lastHit(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, intersector);
    }

    /** Whether any shape intersects the finite, nonzero segment. */
    public boolean anySegmentHit(FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector) {
        double length = segmentLength(segment);
        return anyHit(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, intersector);
    }

    /** All intervals on the finite, nonzero segment in entry/exit order. */
    public List<ExactTraceHit3<T>> allSegmentHits(FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector) {
        return allSegmentHits(sourceFrame, segment, intersector, Integer.MAX_VALUE);
    }

    /** First maxHits intervals on the segment. A segment ending inside a shape has exitSurface=false. */
    public List<ExactTraceHit3<T>> allSegmentHits(
            FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector, int maxHits
    ) {
        double length = segmentLength(segment);
        return allHits(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, intersector, maxHits);
    }

    private ExactTraceHit3<T> select(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector, boolean last
    ) {
        Selection<T> selection = new Selection<>();
        visit(sourceFrame, sourceRay, tMax, intersector, hit -> {
            ExactTraceHit3<T> best = selection.hit;
            if (best == null || (last
                    ? hit.tExit() > best.tExit() || (hit.tExit() == best.tExit() && hit.tEnter() > best.tEnter())
                    : hit.tEnter() < best.tEnter() || (hit.tEnter() == best.tEnter() && hit.tExit() < best.tExit()))) {
                selection.hit = hit;
            }
            return true;
        });
        return selection.hit;
    }

    private boolean visit(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector,
            Predicate<ExactTraceHit3<T>> visitor
    ) {
        if (sourceFrame == null) throw new NullPointerException("sourceFrame");
        if (sourceRay == null) throw new NullPointerException("sourceRay");
        if (intersector == null) throw new NullPointerException("intersector");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");
        Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
        return broadPhase.visitRay(worldRay, tMax, (value, lower, upper) -> {
            final boolean[] active = {true};
            final boolean[] proceed = {true};
            try {
                intersector.intersect(value, worldRay, lower, upper, interval -> {
                    if (!active[0]) throw new IllegalStateException("intersection output used outside provider call");
                    if (!proceed[0]) return;
                    ExactTraceHit3<T> hit = clip(value, worldRay, interval, tMax, lower, upper);
                    if (hit != null) proceed[0] = visitor.test(hit);
                });
            } finally {
                active[0] = false;
            }
            return proceed[0];
        });
    }

    private ExactTraceHit3<T> clip(T value, Ray ray, RayIntersection3 interval, double tMax, double lower, double upper) {
        if (interval == null) throw new NullPointerException("intersection");
        if (interval.tExit() < 0.0 || interval.tEnter() > tMax) return null;
        double enter = Math.max(0.0, interval.tEnter());
        double exit = Math.min(tMax, interval.tExit());
        if (enter < lower || exit > upper) {
            throw new IllegalArgumentException("intersection is outside the candidate interval");
        }
        return new ExactTraceHit3<>(value, enter, exit, ray.at(enter), ray.at(exit),
                interval.tEnter() >= 0.0, interval.tExit() <= tMax);
    }

    static double segmentLength(Segment3 segment) {
        if (segment == null) throw new NullPointerException("segment");
        double length = segment.b().sub(segment.a()).length();
        if (!Double.isFinite(length) || length <= 0.0) throw new IllegalArgumentException("segment length must be finite and > 0");
        return length;
    }

    private static final class Selection<T> {
        private ExactTraceHit3<T> hit;
    }
}
