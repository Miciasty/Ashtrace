package nsk.nu.ashtrace.api.trace.pipeline;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashgrid.api.voxel.query.Raycast;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.trace.model.GridRayHit3;
import nsk.nu.ashtrace.api.trace.model.TraceHit3;

import java.util.List;

/**
 * Frame-aware tracing pipeline that combines broad-phase object hits with voxel occlusion.
 */
public final class FrameOccludedBroadPhaseRayTracer3<T> {
    private final FrameBroadPhaseRayTracer3<T> objectTracer;
    private final FrameGridRayTracer3 voxelTracer;

    public FrameOccludedBroadPhaseRayTracer3(
            FrameGraph3 frames,
            RayQueryableBroadPhase3<T> broadPhase,
            VoxelTraverser traverser
    ) {
        if (frames == null) throw new NullPointerException("frames");
        if (broadPhase == null) throw new NullPointerException("broadPhase");
        if (traverser == null) throw new NullPointerException("traverser");
        this.objectTracer = new FrameBroadPhaseRayTracer3<>(frames, broadPhase);
        this.voxelTracer = new FrameGridRayTracer3(frames, traverser);
    }

    /**
     * Underlying broad-phase object tracer.
     */
    public FrameBroadPhaseRayTracer3<T> objectTracer() {
        return objectTracer;
    }

    /**
     * Underlying voxel tracer used for occlusion checks.
     */
    public FrameGridRayTracer3 voxelTracer() {
        return voxelTracer;
    }

    /**
     * Trace first visible broad-phase hit up to {@code tMax}, clipped by first occupied voxel.
     */
    public TraceHit3<T> firstVisibleHit(
            FrameId sourceFrame,
            Ray sourceRay,
            double tMax,
            FrameBroadPhaseRayTracer3.NarrowPhase3<T> narrowPhase,
            Raycast.Occupancy occluder
    ) {
        if (narrowPhase == null) throw new NullPointerException("narrowPhase");
        if (occluder == null) throw new NullPointerException("occluder");
        double visibleTMax = visibleLimit(sourceFrame, sourceRay, tMax, occluder);
        return objectTracer.firstHit(sourceFrame, sourceRay, visibleTMax, narrowPhase);
    }

    /**
     * Trace visible broad-phase hits up to {@code maxHits}, clipped by first occupied voxel.
     */
    public List<TraceHit3<T>> visibleHits(
            FrameId sourceFrame,
            Ray sourceRay,
            double tMax,
            FrameBroadPhaseRayTracer3.NarrowPhase3<T> narrowPhase,
            Raycast.Occupancy occluder,
            int maxHits
    ) {
        if (narrowPhase == null) throw new NullPointerException("narrowPhase");
        if (occluder == null) throw new NullPointerException("occluder");
        if (maxHits <= 0) throw new IllegalArgumentException("maxHits must be > 0");

        double visibleTMax = visibleLimit(sourceFrame, sourceRay, tMax, occluder);
        List<TraceHit3<T>> hits = objectTracer.allHits(sourceFrame, sourceRay, visibleTMax, narrowPhase);
        if (hits.size() <= maxHits) return hits;
        return List.copyOf(hits.subList(0, maxHits));
    }

    /**
     * Trace first visible broad-phase hit along finite segment, clipped by first occupied voxel.
     */
    public TraceHit3<T> firstVisibleSegmentHit(
            FrameId sourceFrame,
            Segment3 sourceSegment,
            FrameBroadPhaseRayTracer3.NarrowPhase3<T> narrowPhase,
            Raycast.Occupancy occluder
    ) {
        if (sourceSegment == null) throw new NullPointerException("sourceSegment");
        Vector3 delta = sourceSegment.b().sub(sourceSegment.a());
        double length = delta.length();
        if (!Double.isFinite(length) || length <= 0.0) {
            throw new IllegalArgumentException("sourceSegment length must be finite and > 0");
        }
        return firstVisibleHit(sourceFrame, new Ray(sourceSegment.a(), delta), length, narrowPhase, occluder);
    }

    /**
     * Trace visible broad-phase hits along finite segment, clipped by first occupied voxel.
     */
    public List<TraceHit3<T>> visibleSegmentHits(
            FrameId sourceFrame,
            Segment3 sourceSegment,
            FrameBroadPhaseRayTracer3.NarrowPhase3<T> narrowPhase,
            Raycast.Occupancy occluder,
            int maxHits
    ) {
        if (sourceSegment == null) throw new NullPointerException("sourceSegment");
        Vector3 delta = sourceSegment.b().sub(sourceSegment.a());
        double length = delta.length();
        if (!Double.isFinite(length) || length <= 0.0) {
            throw new IllegalArgumentException("sourceSegment length must be finite and > 0");
        }
        return visibleHits(sourceFrame, new Ray(sourceSegment.a(), delta), length, narrowPhase, occluder, maxHits);
    }

    private double visibleLimit(FrameId sourceFrame, Ray sourceRay, double tMax, Raycast.Occupancy occluder) {
        GridRayHit3 voxelHit = voxelTracer.firstHit(sourceFrame, sourceRay, tMax, occluder);
        if (voxelHit == null) return tMax;
        return Math.min(tMax, voxelHit.tEnter());
    }
}
