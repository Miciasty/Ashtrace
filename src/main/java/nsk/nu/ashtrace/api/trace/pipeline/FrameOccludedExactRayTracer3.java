package nsk.nu.ashtrace.api.trace.pipeline;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashgrid.api.voxel.query.Raycast;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.grid.FrameGridSpaceMapper3;
import nsk.nu.ashtrace.api.broadphase.contracts.RayQueryableBroadPhase3;
import nsk.nu.ashtrace.api.trace.contracts.RayIntersector3;
import nsk.nu.ashtrace.api.trace.model.ExactTraceHit3;

import java.util.List;

/**
 * Provider-supplied shape intervals clipped at the first occupied grid cell.
 * Uses the same graph for both stages; keep frames, occupancy, index and provider geometry stable.
 * Voxel traversal is half-open at tMax; shape clipping includes contact at the occluder's entry.
 * Starting in an occupied callback cell clips to zero. At tMax=0 no voxel is visited, but shape
 * contact at the origin can match. An exit cut off by the wall has exitSurface=false.
 * Selection and costs follow FrameExactRayTracer3 plus voxel traversal and frame conversion.
 */
public final class FrameOccludedExactRayTracer3<T> {
    private final FrameExactRayTracer3<T> objectTracer;
    private final FrameGridRayTracer3 voxelTracer;

    /** Use unit world cells rooted at zero. */
    public FrameOccludedExactRayTracer3(
            FrameGraph3 frames, RayQueryableBroadPhase3<T> broadPhase, VoxelTraverser traverser
    ) {
        this(new FrameGridRayTracer3(frames, traverser), broadPhase);
    }

    /** Use an explicitly mapped grid, including its live or frozen frame graph. */
    public static <T> FrameOccludedExactRayTracer3<T> forGrid(
            FrameGridSpaceMapper3 grid, RayQueryableBroadPhase3<T> broadPhase, VoxelTraverser traverser
    ) {
        return new FrameOccludedExactRayTracer3<>(FrameGridRayTracer3.forGrid(grid, traverser), broadPhase);
    }

    private FrameOccludedExactRayTracer3(FrameGridRayTracer3 voxelTracer, RayQueryableBroadPhase3<T> broadPhase) {
        this.voxelTracer = voxelTracer;
        this.objectTracer = new FrameExactRayTracer3<>(voxelTracer.frames(), broadPhase);
    }

    public FrameExactRayTracer3<T> objectTracer() {
        return objectTracer;
    }

    public FrameGridRayTracer3 voxelTracer() {
        return voxelTracer;
    }

    /** First shape interval in the visible closed interval, or null. */
    public ExactTraceHit3<T> firstVisibleHit(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector, Raycast.Occupancy occluder
    ) {
        requireCallbacks(intersector, occluder);
        return objectTracer.firstHit(sourceFrame, sourceRay, visibleLimit(sourceFrame, sourceRay, tMax, occluder), intersector);
    }

    /** Interval with the greatest visible exit, then entry, or null. Its exit can be clipped by the wall. */
    public ExactTraceHit3<T> lastVisibleHit(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector, Raycast.Occupancy occluder
    ) {
        requireCallbacks(intersector, occluder);
        return objectTracer.lastHit(sourceFrame, sourceRay, visibleLimit(sourceFrame, sourceRay, tMax, occluder), intersector);
    }

    /** Whether any shape interval is present before or in contact with the occluder. */
    public boolean anyVisibleHit(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector, Raycast.Occupancy occluder
    ) {
        requireCallbacks(intersector, occluder);
        return objectTracer.anyHit(sourceFrame, sourceRay, visibleLimit(sourceFrame, sourceRay, tMax, occluder), intersector);
    }

    /** All visible intervals in entry/exit order. */
    public List<ExactTraceHit3<T>> visibleHits(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector, Raycast.Occupancy occluder
    ) {
        return visibleHits(sourceFrame, sourceRay, tMax, intersector, occluder, Integer.MAX_VALUE);
    }

    /** First maxHits visible intervals after exact sorting. */
    public List<ExactTraceHit3<T>> visibleHits(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector,
            Raycast.Occupancy occluder, int maxHits
    ) {
        return visibleHits(sourceFrame, sourceRay, tMax, intersector, occluder, maxHits, new TraceQueryBuffer3<>());
    }

    /** Visible intervals with reusable list capacity; do not share the buffer between concurrent calls. */
    public List<ExactTraceHit3<T>> visibleHits(
            FrameId sourceFrame, Ray sourceRay, double tMax, RayIntersector3<T> intersector,
            Raycast.Occupancy occluder, int maxHits, TraceQueryBuffer3<T> buffer
    ) {
        requireCallbacks(intersector, occluder);
        if (maxHits <= 0) throw new IllegalArgumentException("maxHits must be > 0");
        if (buffer == null) throw new NullPointerException("buffer");
        return objectTracer.allHits(sourceFrame, sourceRay, visibleLimit(sourceFrame, sourceRay, tMax, occluder),
                intersector, maxHits, buffer);
    }

    /** First interval on a finite nonzero segment, clipped by the grid. */
    public ExactTraceHit3<T> firstVisibleSegmentHit(
            FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector, Raycast.Occupancy occluder
    ) {
        double length = FrameExactRayTracer3.segmentLength(segment);
        return firstVisibleHit(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, intersector, occluder);
    }

    /** Last interval on a finite nonzero segment, clipped by the grid. */
    public ExactTraceHit3<T> lastVisibleSegmentHit(
            FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector, Raycast.Occupancy occluder
    ) {
        double length = FrameExactRayTracer3.segmentLength(segment);
        return lastVisibleHit(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, intersector, occluder);
    }

    /** Whether a finite nonzero segment has any visible shape interval. */
    public boolean anyVisibleSegmentHit(
            FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector, Raycast.Occupancy occluder
    ) {
        double length = FrameExactRayTracer3.segmentLength(segment);
        return anyVisibleHit(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, intersector, occluder);
    }

    /** First maxHits visible intervals on a finite nonzero segment. */
    public List<ExactTraceHit3<T>> visibleSegmentHits(
            FrameId sourceFrame, Segment3 segment, RayIntersector3<T> intersector, Raycast.Occupancy occluder, int maxHits
    ) {
        double length = FrameExactRayTracer3.segmentLength(segment);
        return visibleHits(sourceFrame, new Ray(segment.a(), segment.b().sub(segment.a())), length, intersector, occluder, maxHits);
    }

    private void requireCallbacks(RayIntersector3<T> intersector, Raycast.Occupancy occluder) {
        if (intersector == null) throw new NullPointerException("intersector");
        if (occluder == null) throw new NullPointerException("occluder");
    }

    private double visibleLimit(FrameId sourceFrame, Ray sourceRay, double tMax, Raycast.Occupancy occluder) {
        var hit = voxelTracer.firstHit(sourceFrame, sourceRay, tMax, occluder);
        return hit == null ? tMax : Math.min(tMax, hit.tEnter());
    }
}
