package nsk.nu.ashtrace.api.broadphase.contracts;

/** Visits a closed AABB interval in world distances. Return false to request an immediate stop. */
@FunctionalInterface
public interface RayCandidateVisitor3<T> {
    boolean visit(T value, double tEnter, double tExit);
}
