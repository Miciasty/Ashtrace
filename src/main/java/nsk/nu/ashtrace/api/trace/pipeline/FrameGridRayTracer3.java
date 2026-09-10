package nsk.nu.ashtrace.api.trace.pipeline;

import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.geometry.Segment3;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashgrid.api.voxel.query.Raycast;
import nsk.nu.ashgrid.api.voxel.traversal.VoxelTraverser;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashspace.api.frame.FrameId;
import nsk.nu.ashspace.api.grid.FrameGridSpaceMapper3;
import nsk.nu.ashspace.api.space.SpaceConverter3;
import nsk.nu.ashtrace.api.trace.model.GridRayHit3;

import java.util.ArrayList;
import java.util.List;

/**
 * Frame-aware voxel ray tracing pipeline built on Ashspace and Ashgrid.
 * The original constructor uses unit world cells rooted at zero. The forGrid factory attaches
 * cells to an explicit grid frame, origin and cell size, using Ashspace conversion and Ashgrid traversal.
 * Results always use world-distance parameters and world points; integer coordinates belong to that grid.
 * The supplied traverser owns cell/tie ordering; the DDA provider visits [0,tMax), including
 * starting-cell and boundary tie visits, but visits nothing when tMax=0.
 * Zero-length segments are rejected. Keep frames, occupancy and traverser configuration stable
 * throughout a query, including callbacks; this tracer neither locks nor snapshots them.
 * Returned lists are immutable and hit coordinates/points do not track subsequent changes.
 */
public final class FrameGridRayTracer3 {
    private final FrameGraph3 frames;
    private final SpaceConverter3 converter;
    private final VoxelTraverser traverser;
    private final Raycast raycast;
    private final FrameGridSpaceMapper3 grid;

    public FrameGridRayTracer3(FrameGraph3 frames, VoxelTraverser traverser) {
        if (frames == null) throw new NullPointerException("frames");
        if (traverser == null) throw new NullPointerException("traverser");
        this.frames = frames;
        this.converter = new SpaceConverter3(frames);
        this.traverser = traverser;
        this.raycast = new Raycast(traverser);
        this.grid = null;
    }

    /**
     * Trace a grid attached to a frame. Retains the mapper and its live or frozen graph.
     * World distance is divided by cellSize for traversal and multiplied back for results.
     * The normalized cell ray and positive distance limit must remain representable; overflow,
     * loss of a nonzero origin offset to zero, or loss of a positive limit to zero is rejected.
     * Mapping does not add a boundary epsilon.
     */
    public static FrameGridRayTracer3 forGrid(FrameGridSpaceMapper3 grid, VoxelTraverser traverser) {
        return new FrameGridRayTracer3(grid, traverser);
    }

    private FrameGridRayTracer3(FrameGridSpaceMapper3 grid, VoxelTraverser traverser) {
        if (grid == null) throw new NullPointerException("grid");
        if (traverser == null) throw new NullPointerException("traverser");
        this.grid = grid;
        this.frames = grid.frames();
        this.converter = new SpaceConverter3(frames);
        this.traverser = traverser;
        this.raycast = new Raycast(traverser);
    }

    /** Explicit mapper, or null for the original unit-world constructor. */
    public FrameGridSpaceMapper3 grid() {
        return grid;
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

        QueryRay query = prepare(sourceFrame, sourceRay, tMax);
        Raycast.Hit hit = raycast.first(query.cellRay, query.cellLimit, occupancy);
        if (hit == null) return null;
        return hit(query, hit.x(), hit.y(), hit.z(), hit.tEnter(), hit.tExit());
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

        QueryRay query = prepare(sourceFrame, sourceRay, tMax);
        ArrayList<GridRayHit3> hits = new ArrayList<>();
        traverser.traverse(query.cellRay, query.cellLimit, (x, y, z, tEnter, tExit) -> {
            if (!occupancy.test(x, y, z)) return true;
            hits.add(hit(query, x, y, z, tEnter, tExit));
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

    private QueryRay prepare(FrameId sourceFrame, Ray sourceRay, double tMax) {
        Ray worldRay = converter.ray(sourceRay, sourceFrame, frames.root());
        if (grid == null) return new QueryRay(worldRay, worldRay, tMax, tMax, 1.0);
        Ray inGrid = converter.ray(sourceRay, sourceFrame, grid.gridFrame());
        Vector3 offset = inGrid.origin().sub(grid.gridOrigin());
        double size = grid.cellSize();
        Vector3 origin = new Vector3(cellCoordinate(offset.x(), size), cellCoordinate(offset.y(), size),
                cellCoordinate(offset.z(), size));
        double limit = tMax / size;
        if (!Double.isFinite(limit) || (tMax > 0.0 && limit == 0.0)) {
            throw new IllegalArgumentException("grid traversal distance is not representable");
        }
        return new QueryRay(worldRay, new Ray(origin, inGrid.direction()), limit, tMax, size);
    }

    private static double cellCoordinate(double offset, double size) {
        double coordinate = offset / size;
        if (!Double.isFinite(coordinate) || (offset != 0.0 && coordinate == 0.0)) {
            throw new IllegalArgumentException("grid origin coordinate is not representable");
        }
        return coordinate;
    }

    private GridRayHit3 hit(QueryRay query, int x, int y, int z, double enter, double exit) {
        double worldEnter = query.worldDistance(enter);
        double worldExit = query.worldDistance(exit);
        Vector3 point = query.worldRay.at(worldEnter);
        if (!Double.isFinite(point.x()) || !Double.isFinite(point.y()) || !Double.isFinite(point.z())) {
            throw new IllegalArgumentException("world hit point is not finite");
        }
        return new GridRayHit3(x, y, z, worldEnter, worldExit, point);
    }

    private record QueryRay(Ray worldRay, Ray cellRay, double cellLimit, double worldLimit, double cellSize) {
        private double worldDistance(double cellDistance) {
            if (!Double.isFinite(cellDistance) || cellDistance < 0.0 || cellDistance > cellLimit) {
                throw new IllegalArgumentException("traverser returned a parameter outside its interval");
            }
            // Preserve the requested endpoint exactly after the distance/unit conversion round trip.
            double worldDistance = cellDistance == cellLimit ? worldLimit : cellDistance * cellSize;
            if (!Double.isFinite(worldDistance)) throw new IllegalArgumentException("world distance is not finite");
            return Math.min(worldLimit, worldDistance);
        }
    }
}
