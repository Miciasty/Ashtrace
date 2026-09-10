package nsk.nu.ashtrace.api.trace.pipeline;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.space.SpaceConverter3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.trace.model.TraceHit3;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Frame-aware tracing pipeline over broad-phase ray/segment candidates.
 * Accepts indexed AABBs, without computing exact enclosed-surface distances. Results sort by
 * AABB {@code tEnter}, then {@code tExit}, then broad-phase emission order for full ties.
 * The graph, index, payload geometry and callback state must remain stable throughout a query;
 * callbacks must not mutate them. No snapshots or synchronization are supplied by this tracer.
 * Result lists are immutable snapshots retaining payload references.
 *
 * <p>With C emitted candidates, Q broad-phase work and F total acceptance-callback cost,
 * ordered first-hit and limited queries cost O(Q + C log C + F), plus frame conversion.
 * All candidates are collected before filtering; temporary storage is O(C).</p>
 */
public final class FrameBroadPhaseRayTracer3<T> {
    private final FrameGraph3 frames;
    private final SpaceConverter3 converter;
    private final RayQueryableBroadPhase3<T> broadPhase;

    @FunctionalInterface
    public interface NarrowPhase3<T> {
        /**
         * Tests an AABB candidate on the closed, query-clipped interval in world-distance units.
         * A geometric test must restrict its surface check to this interval, especially under voxel clipping.
         * Returning true does not replace the AABB interval or point with an exact surface result.
         * @return true to accept this bounds candidate; false to reject it
         */
        boolean test(T value, Ray worldRay, double tEnter, double tExit);
    }

    public FrameBroadPhaseRayTracer3(FrameGraph3 frames, RayQueryableBroadPhase3<T> broadPhase) {
        if (frames == null) throw new NullPointerException("frames");
        if (broadPhase == null) throw new NullPointerException("broadPhase");
        this.frames = frames;
        this.converter = new SpaceConverter3(frames);
        this.broadPhase = broadPhase;
    }

    /**
     * Underlying frame graph.
     */
    public FrameGraph3 frames() {
        return frames;
    }

    /**
     * Underlying broad-phase index.
     */
    public RayQueryableBroadPhase3<T> broadPhase() {
        return broadPhase;
    }

    /**
     * Return the first accepted AABB in the documented interval order, or null.
     * This need not be the nearest enclosed surface. At tMax=0, bounds containing the origin can match.
     * {@code tMax} is a finite world-space distance along the normalized world ray.
     */
    public TraceHit3<T> firstHit(
            FrameId sourceFrame,
            Ray sourceRay,
            double tMax,
            NarrowPhase3<T> narrowPhase
    ) {
        return firstHit(sourceFrame, sourceRay, tMax, narrowPhase, new TraceQueryBuffer3<>());
    }

    /** First accepted AABB, using exclusive reusable candidate-list capacity. */
    public TraceHit3<T> firstHit(
            FrameId sourceFrame, Ray sourceRay, double tMax,
            NarrowPhase3<T> narrowPhase, TraceQueryBuffer3<T> buffer
    ) {
        if (sourceFrame == null) throw new NullPointerException("sourceFrame");
        if (sourceRay == null) throw new NullPointerException("sourceRay");
        if (narrowPhase == null) throw new NullPointerException("narrowPhase");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");
        if (buffer == null) throw new NullPointerException("buffer");

        buffer.begin();
        try {
            Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
            orderedCandidates(worldRay, tMax, buffer);
            for (BroadPhaseRayHit3<T> candidate : buffer.candidates) {
                if (!narrowPhase.test(candidate.value(), worldRay, candidate.tEnter(), candidate.tExit())) continue;
                return new TraceHit3<>(candidate.value(), candidate.tEnter(), candidate.tExit(), worldRay.at(candidate.tEnter()));
            }
            return null;
        } finally {
            buffer.end();
        }
    }

    /**
     * Whether any candidate passes acceptance. Uses visitRay order rather than nearest-first order,
     * avoiding pipeline candidate sorting and stopping later acceptance calls as soon as possible.
     */
    public boolean anyHit(FrameId sourceFrame, Ray sourceRay, double tMax, NarrowPhase3<T> narrowPhase) {
        if (sourceFrame == null) throw new NullPointerException("sourceFrame");
        if (sourceRay == null) throw new NullPointerException("sourceRay");
        if (narrowPhase == null) throw new NullPointerException("narrowPhase");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");
        Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
        return !broadPhase.visitRay(worldRay, tMax, (value, enter, exit) -> !narrowPhase.test(value, worldRay, enter, exit));
    }

    /** Whether any accepted AABB intersects a finite nonzero closed segment. */
    public boolean anySegmentHit(FrameId sourceFrame, Segment3 segment, NarrowPhase3<T> narrowPhase) {
        double length = FrameExactRayTracer3.segmentLength(segment);
        return anyHit(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, narrowPhase);
    }

    /**
     * Return all accepted AABB intervals in the documented order, or an empty immutable list.
     * {@code tMax} is a finite world-space distance along the normalized world ray.
     */
    public List<TraceHit3<T>> allHits(
            FrameId sourceFrame,
            Ray sourceRay,
            double tMax,
            NarrowPhase3<T> narrowPhase
    ) {
        return allHits(sourceFrame, sourceRay, tMax, narrowPhase, Integer.MAX_VALUE);
    }

    /**
     * Return at most {@code maxHits} accepted AABB intervals; stop acceptance callbacks at that count.
     * {@code tMax} is a finite world-space distance along the normalized world ray.
     */
    public List<TraceHit3<T>> allHits(
            FrameId sourceFrame,
            Ray sourceRay,
            double tMax,
            NarrowPhase3<T> narrowPhase,
            int maxHits
    ) {
        return allHits(sourceFrame, sourceRay, tMax, narrowPhase, maxHits, new TraceQueryBuffer3<>());
    }

    /** Accepted AABB intervals with reusable candidate storage; the returned list is independent. */
    public List<TraceHit3<T>> allHits(
            FrameId sourceFrame, Ray sourceRay, double tMax, NarrowPhase3<T> narrowPhase,
            int maxHits, TraceQueryBuffer3<T> buffer
    ) {
        if (sourceFrame == null) throw new NullPointerException("sourceFrame");
        if (sourceRay == null) throw new NullPointerException("sourceRay");
        if (narrowPhase == null) throw new NullPointerException("narrowPhase");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");
        if (maxHits <= 0) throw new IllegalArgumentException("maxHits must be > 0");
        if (buffer == null) throw new NullPointerException("buffer");

        buffer.begin();
        try {
            Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
            orderedCandidates(worldRay, tMax, buffer);
            ArrayList<TraceHit3<T>> hits = new ArrayList<>(Math.min(buffer.candidates.size(), maxHits));
            for (BroadPhaseRayHit3<T> candidate : buffer.candidates) {
                if (!narrowPhase.test(candidate.value(), worldRay, candidate.tEnter(), candidate.tExit())) continue;
                hits.add(new TraceHit3<>(candidate.value(), candidate.tEnter(), candidate.tExit(), worldRay.at(candidate.tEnter())));
                if (hits.size() >= maxHits) break;
            }
            return List.copyOf(hits);
        } finally {
            buffer.end();
        }
    }

    /**
     * Trace the first accepted AABB along a closed segment; zero or non-finite length is rejected.
     */
    public TraceHit3<T> firstSegmentHit(
            FrameId sourceFrame,
            Segment3 sourceSegment,
            NarrowPhase3<T> narrowPhase
    ) {
        if (sourceSegment == null) throw new NullPointerException("sourceSegment");
        var delta = sourceSegment.b().sub(sourceSegment.a());
        double length = delta.length();
        if (!Double.isFinite(length) || length <= 0.0) {
            throw new IllegalArgumentException("sourceSegment length must be finite and > 0");
        }
        return firstHit(sourceFrame, new Ray(sourceSegment.a(), delta), length, narrowPhase);
    }

    /**
     * Trace all accepted hits for a source-frame segment.
     */
    public List<TraceHit3<T>> allSegmentHits(
            FrameId sourceFrame,
            Segment3 sourceSegment,
            NarrowPhase3<T> narrowPhase
    ) {
        return allSegmentHits(sourceFrame, sourceSegment, narrowPhase, Integer.MAX_VALUE);
    }

    /**
     * Trace accepted hits for a source-frame segment up to {@code maxHits}.
     */
    public List<TraceHit3<T>> allSegmentHits(
            FrameId sourceFrame,
            Segment3 sourceSegment,
            NarrowPhase3<T> narrowPhase,
            int maxHits
    ) {
        if (sourceSegment == null) throw new NullPointerException("sourceSegment");
        var delta = sourceSegment.b().sub(sourceSegment.a());
        double length = delta.length();
        if (!Double.isFinite(length) || length <= 0.0) {
            throw new IllegalArgumentException("sourceSegment length must be finite and > 0");
        }
        return allHits(sourceFrame, new Ray(sourceSegment.a(), delta), length, narrowPhase, maxHits);
    }

    private void orderedCandidates(Ray worldRay, double tMax, TraceQueryBuffer3<T> buffer) {
        broadPhase.queryRay(worldRay, tMax, buffer.candidates::add);
        // List.sort is stable, preserving broad-phase emission order on a full interval tie.
        buffer.candidates.sort(Comparator.comparingDouble((BroadPhaseRayHit3<T> hit) -> hit.tEnter())
                .thenComparingDouble(BroadPhaseRayHit3::tExit));
    }
}
