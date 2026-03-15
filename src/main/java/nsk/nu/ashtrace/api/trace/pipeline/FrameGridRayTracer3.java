package nsk.nu.ashtrace.api.trace.pipeline;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashgrid.api.voxel.query.Raycast;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.space.SpaceConverter3;
import nsk.nu.ashtrace.api.trace.model.GridRayHit3;

import java.util.ArrayList;
import java.util.List;

/**
 * Frame-aware voxel ray tracing pipeline built on Ashspace and Ashgrid.
 */
public final class FrameGridRayTracer3 {
    private final FrameGraph3 frames;
    private final SpaceConverter3 converter;
    private final VoxelTraverser traverser;
    private final Raycast raycast;

    public FrameGridRayTracer3(FrameGraph3 frames, VoxelTraverser traverser) {
        if (frames == null) throw new NullPointerException("frames");
        if (traverser == null) throw new NullPointerException("traverser");
        this.frames = frames;
        this.converter = new SpaceConverter3(frames);
        this.traverser = traverser;
        this.raycast = new Raycast(traverser);
    }

    /**
     * Underlying frame graph used for source-frame to world conversion.
     */
    public FrameGraph3 frames() {
        return frames;
    }

    /**
     * Underlying voxel traverser used for deterministic visit ordering.
     */
    public VoxelTraverser traverser() {
        return traverser;
    }

    /**
     * Trace first occupied voxel hit using a ray defined in {@code sourceFrame}.
     *
     * <p>If multiple cells begin at the same boundary, tie-breaking follows the traverser's visit order.</p>
     * <p>{@code tMax} is a finite world-space distance along the normalized world ray.</p>
     *
     * @return first hit, or {@code null} if no hit exists up to {@code tMax}
     */
    public GridRayHit3 firstHit(FrameId sourceFrame, Ray sourceRay, double tMax, Raycast.Occupancy occupancy) {
        if (sourceFrame == null) throw new NullPointerException("sourceFrame");
        if (sourceRay == null) throw new NullPointerException("sourceRay");
        if (occupancy == null) throw new NullPointerException("occupancy");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");

        Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
        Raycast.Hit hit = raycast.first(worldRay, tMax, occupancy);
        if (hit == null) return null;
        return new GridRayHit3(
                hit.x(),
                hit.y(),
                hit.z(),
                hit.tEnter(),
                hit.tExit(),
                worldRay.at(hit.tEnter())
        );
    }

    /**
     * Trace all occupied voxel hits using a ray defined in {@code sourceFrame}.
     * Hits are returned in deterministic traverser visit order.
     * {@code tMax} is a finite world-space distance along the normalized world ray.
     */
    public List<GridRayHit3> allHits(FrameId sourceFrame, Ray sourceRay, double tMax, Raycast.Occupancy occupancy) {
        return allHits(sourceFrame, sourceRay, tMax, occupancy, Integer.MAX_VALUE);
    }

    /**
     * Trace occupied voxel hits up to {@code maxHits} using a ray defined in {@code sourceFrame}.
     * Hits are returned in deterministic traverser visit order.
     * {@code tMax} is a finite world-space distance along the normalized world ray.
     */
    public List<GridRayHit3> allHits(
            FrameId sourceFrame,
            Ray sourceRay,
            double tMax,
            Raycast.Occupancy occupancy,
            int maxHits
    ) {
        if (sourceFrame == null) throw new NullPointerException("sourceFrame");
        if (sourceRay == null) throw new NullPointerException("sourceRay");
        if (occupancy == null) throw new NullPointerException("occupancy");
        if (!Double.isFinite(tMax) || tMax < 0.0) throw new IllegalArgumentException("tMax must be finite and >= 0");
        if (maxHits <= 0) throw new IllegalArgumentException("maxHits must be > 0");

        Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
        ArrayList<GridRayHit3> hits = new ArrayList<>();
        traverser.traverse(worldRay, tMax, (x, y, z, tEnter, tExit) -> {
            if (!occupancy.test(x, y, z)) return true;
            double enter = Math.max(0.0, tEnter);
            double exit = Math.max(enter, tExit);
            hits.add(new GridRayHit3(x, y, z, enter, exit, worldRay.at(enter)));
            return hits.size() < maxHits;
        });
        return List.copyOf(hits);
    }

    /**
     * Trace first occupied voxel hit along a finite segment defined in {@code sourceFrame}.
     *
     * @return first hit, or {@code null} if no hit exists within segment length
     */
    public GridRayHit3 firstSegmentHit(FrameId sourceFrame, Segment3 sourceSegment, Raycast.Occupancy occupancy) {
        if (sourceSegment == null) throw new NullPointerException("sourceSegment");
        Vector3 delta = sourceSegment.b().sub(sourceSegment.a());
        double length = delta.length();
        if (!Double.isFinite(length) || length <= 0.0) {
            throw new IllegalArgumentException("sourceSegment length must be finite and > 0");
        }
        return firstHit(sourceFrame, new Ray(sourceSegment.a(), delta), length, occupancy);
    }

    /**
     * Trace occupied voxel hits along a finite segment defined in {@code sourceFrame}.
     */
    public List<GridRayHit3> allSegmentHits(FrameId sourceFrame, Segment3 sourceSegment, Raycast.Occupancy occupancy) {
        return allSegmentHits(sourceFrame, sourceSegment, occupancy, Integer.MAX_VALUE);
    }

    /**
     * Trace occupied voxel hits along a finite segment defined in {@code sourceFrame}, up to {@code maxHits}.
     */
    public List<GridRayHit3> allSegmentHits(
            FrameId sourceFrame,
            Segment3 sourceSegment,
            Raycast.Occupancy occupancy,
            int maxHits
    ) {
        if (sourceSegment == null) throw new NullPointerException("sourceSegment");
        Vector3 delta = sourceSegment.b().sub(sourceSegment.a());
        double length = delta.length();
        if (!Double.isFinite(length) || length <= 0.0) {
            throw new IllegalArgumentException("sourceSegment length must be finite and > 0");
        }
        return allHits(sourceFrame, new Ray(sourceSegment.a(), delta), length, occupancy, maxHits);
    }
}
