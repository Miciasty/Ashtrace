package nsk.nu.ashtrace.benchmark.manual;

import com.sun.management.ThreadMXBean;
import nsk.nu.ashcore.api.geometry.AxisAlignedBox;
import nsk.nu.ashcore.api.geometry.Ray;
import nsk.nu.ashcore.api.math.Vector3;
import nsk.nu.ashspace.api.frame.FrameGraph3;
import nsk.nu.ashtrace.api.broadphase.model.AabbEntry3;
import nsk.nu.ashtrace.api.trace.pipeline.FrameBroadPhaseRayTracer3;
import nsk.nu.ashtrace.api.trace.pipeline.TraceQueryBuffer3;
import nsk.nu.ashtrace.implementation.broadphase.staticindex.LinearAabbBroadPhase3;

import java.lang.management.ManagementFactory;
import java.util.ArrayList;

/** Manual overlapping-bounds workload; timings are observations, not performance guarantees. */
public final class TraceWorkloadBenchmarkMain {
    private static volatile double result;

    private TraceWorkloadBenchmarkMain() {}

    public static void main(String[] args) {
        int count = 5_000;
        var entries = new ArrayList<AabbEntry3<Integer>>();
        for (int i = 0; i < count; i++) entries.add(new AabbEntry3<>(
                new AxisAlignedBox(new Vector3(2, 0, 0), new Vector3(5, 1, 1)), i));
        var frames = FrameGraph3.worldRoot();
        var tracer = new FrameBroadPhaseRayTracer3<>(frames, new LinearAabbBroadPhase3<>(entries));
        Ray ray = new Ray(new Vector3(0, 0.5, 0.5), new Vector3(1, 0, 0));
        measure("firstHit", () -> result = tracer.firstHit(frames.root(), ray, 10, (v, r, a, b) -> true).tEnter());
        var buffer = new TraceQueryBuffer3<Integer>();
        measure("firstHit reused buffer", () -> result = tracer.firstHit(frames.root(), ray, 10, (v, r, a, b) -> true, buffer).tEnter());
        measure("anyHit", () -> result = tracer.anyHit(frames.root(), ray, 10, (v, r, a, b) -> true) ? 1 : 0);
    }

    private static void measure(String name, Runnable query) {
        for (int i = 0; i < 100; i++) query.run();
        int iterations = 500;
        ThreadMXBean bean = (ThreadMXBean) ManagementFactory.getThreadMXBean();
        long thread = Thread.currentThread().threadId();
        boolean allocation = bean.isThreadAllocatedMemorySupported();
        if (allocation && !bean.isThreadAllocatedMemoryEnabled()) bean.setThreadAllocatedMemoryEnabled(true);
        long bytes = allocation ? bean.getThreadAllocatedBytes(thread) : 0;
        long start = System.nanoTime();
        for (int i = 0; i < iterations; i++) query.run();
        long elapsed = System.nanoTime() - start;
        long allocated = allocation ? bean.getThreadAllocatedBytes(thread) - bytes : -1;
        System.out.println(name + ": entries=5000 warmup=100 iterations=" + iterations
                + " ns/op=" + elapsed / iterations + " bytes/op=" + (allocation ? allocated / iterations : "unavailable")
                + " result=" + result);
    }
}
