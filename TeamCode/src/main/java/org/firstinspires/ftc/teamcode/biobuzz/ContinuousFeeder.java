package org.firstinspires.ftc.teamcode.biobuzz;

/**
 * AUTO feed: turns the windmill continuously (no stop on every pocket boundary, which left balls resting against
 * the flywheel) while the flywheel is at speed, pausing while it recovers after a ball. A stall backs off briefly.
 * Done after {@code totalTicks} of windmill travel. No FTC SDK dependency, so it is unit-tested on the laptop.
 */
public class ContinuousFeeder {

    static final double STALL_SEC = 0.6;
    static final int STALL_MIN_TICKS = 5;

    private final double totalTicks;
    private final double feedPower;
    private final double unjamPower;
    private final double unjamSec;

    private boolean started = false;
    private int startTicks;
    private int progressTicks;
    private double progressTime;
    private double unjamUntil = -1;
    private int stalls = 0;

    public ContinuousFeeder(double totalTicks, double feedPower, double unjamPower, double unjamSec) {
        this.totalTicks = totalTicks;
        this.feedPower = feedPower;
        this.unjamPower = unjamPower;
        this.unjamSec = unjamSec;
    }

    public double update(double t, int indexerTicks, boolean launcherReady) {
        if (!started) {
            started = true;
            startTicks = indexerTicks;
            progressTicks = indexerTicks;
            progressTime = t;
        }
        if (done(indexerTicks)) {
            return 0;
        }
        if (t < unjamUntil) {
            return -unjamPower;
        }
        if (!launcherReady) {
            progressTicks = indexerTicks;                       // waiting is not a stall
            progressTime = t;
            return 0;
        }
        if (indexerTicks - progressTicks >= STALL_MIN_TICKS) {
            progressTicks = indexerTicks;
            progressTime = t;
        } else if (t - progressTime > STALL_SEC) {
            stalls++;
            unjamUntil = t + unjamSec;
            progressTime = t + unjamSec;
            progressTicks = indexerTicks - 1000;                // count forward progress again after the back-off
            return -unjamPower;
        }
        return feedPower;
    }

    public boolean done(int indexerTicks) {
        return started && indexerTicks - startTicks >= totalTicks;
    }

    public double travelledPockets(int indexerTicks) {
        return started ? (indexerTicks - startTicks) / BioBuzzConstants.INDEXER_TICKS_PER_POCKET : 0;
    }

    public int stalls() {
        return stalls;
    }
}
