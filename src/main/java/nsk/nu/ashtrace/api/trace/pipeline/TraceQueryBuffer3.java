package nsk.nu.ashtrace.api.trace.pipeline;

import nsk.nu.ashtrace.api.broadphase.model.BroadPhaseRayHit3;
import nsk.nu.ashtrace.api.trace.model.ExactTraceHit3;

import java.util.ArrayList;

/**
 * Reusable candidate/result-list capacity for one query at a time. Not thread-safe.
 * Queries clear retained payload references on success or failure while keeping list capacity.
 * Returned immutable result lists are independent of later buffer reuse. This is not an
 * allocation-free contract: hits, sort workspace, callbacks and index queries may still allocate.
 * Reentrant use during a callback throws IllegalStateException.
 */
public final class TraceQueryBuffer3<T> {
    final ArrayList<BroadPhaseRayHit3<T>> candidates = new ArrayList<>();
    final ArrayList<ExactTraceHit3<T>> exactHits = new ArrayList<>();
    private boolean inUse;

    /** Release retained list capacity while no query is using this buffer. */
    public void trimToSize() {
        if (inUse) throw new IllegalStateException("buffer is in use");
        candidates.trimToSize();
        exactHits.trimToSize();
    }

    void begin() {
        if (inUse) throw new IllegalStateException("buffer is already in use");
        inUse = true;
    }

    void end() {
        candidates.clear();
        exactHits.clear();
        inUse = false;
    }
}
