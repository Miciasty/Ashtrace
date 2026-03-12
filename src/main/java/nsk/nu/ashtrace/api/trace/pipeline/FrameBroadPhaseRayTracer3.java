package nsk.nu.ashtrace.api.trace.pipeline;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
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
 */
public final class FrameBroadPhaseRayTracer3<T> {
    private final FrameGraph3 frames;
    private final SpaceConverter3 converter;
    private final RayQueryableBroadPhase3<T> broadPhase;

    @FunctionalInterface
    public interface NarrowPhase3<T> {
        /**
         * @return true when candidate should be accepted as final hit
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
     * Trace first accepted hit for a source-frame ray.
     */
    public TraceHit3<T> firstHit(
            FrameId sourceFrame,
            Ray sourceRay,
            double tMax,
            NarrowPhase3<T> narrowPhase
    ) {
        if (sourceFrame == null) throw new NullPointerException("sourceFrame");
        if (sourceRay == null) throw new NullPointerException("sourceRay");
        if (narrowPhase == null) throw new NullPointerException("narrowPhase");
        if (Double.isNaN(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be >= 0 and not NaN");

        Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
        List<BroadPhaseRayHit3<T>> candidates = orderedCandidates(worldRay, tMax);
        for (BroadPhaseRayHit3<T> candidate : candidates) {
            if (!narrowPhase.test(candidate.value(), worldRay, candidate.tEnter(), candidate.tExit())) continue;
            return new TraceHit3<>(
                    candidate.value(),
                    candidate.tEnter(),
                    candidate.tExit(),
                    worldRay.at(candidate.tEnter())
            );
        }
        return null;
    }

    /**
     * Trace all accepted hits for a source-frame ray.
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
     * Trace accepted hits for a source-frame ray up to {@code maxHits}.
     */
    public List<TraceHit3<T>> allHits(
            FrameId sourceFrame,
            Ray sourceRay,
            double tMax,
            NarrowPhase3<T> narrowPhase,
            int maxHits
    ) {
        if (sourceFrame == null) throw new NullPointerException("sourceFrame");
        if (sourceRay == null) throw new NullPointerException("sourceRay");
        if (narrowPhase == null) throw new NullPointerException("narrowPhase");
        if (Double.isNaN(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be >= 0 and not NaN");
        if (maxHits <= 0) throw new IllegalArgumentException("maxHits must be > 0");

        Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
        List<BroadPhaseRayHit3<T>> candidates = orderedCandidates(worldRay, tMax);
        ArrayList<TraceHit3<T>> hits = new ArrayList<>(Math.min(candidates.size(), maxHits));

        for (BroadPhaseRayHit3<T> candidate : candidates) {
            if (!narrowPhase.test(candidate.value(), worldRay, candidate.tEnter(), candidate.tExit())) continue;
            hits.add(new TraceHit3<>(
                    candidate.value(),
                    candidate.tEnter(),
                    candidate.tExit(),
                    worldRay.at(candidate.tEnter())
            ));
            if (hits.size() >= maxHits) break;
        }
        return List.copyOf(hits);
    }

    /**
     * Trace first accepted hit for a source-frame segment.
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

    private List<BroadPhaseRayHit3<T>> orderedCandidates(Ray worldRay, double tMax) {
        ArrayList<OrderedCandidate<T>> ordered = new ArrayList<>();
        final int[] index = {0};
        broadPhase.queryRay(worldRay, tMax, candidate -> ordered.add(new OrderedCandidate<>(candidate, index[0]++)));
        ordered.sort(Comparator
                .comparingDouble((OrderedCandidate<T> c) -> c.candidate.tEnter())
                .thenComparingDouble(c -> c.candidate.tExit())
                .thenComparingInt(c -> c.order));

        ArrayList<BroadPhaseRayHit3<T>> out = new ArrayList<>(ordered.size());
        for (OrderedCandidate<T> candidate : ordered) {
            out.add(candidate.candidate);
        }
        return List.copyOf(out);
    }

    private record OrderedCandidate<T>(BroadPhaseRayHit3<T> candidate, int order) {
    }
}
