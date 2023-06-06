package main.java.obstacleavoiding.path.pid;

public class PIDPreset {
    private final double kP;
    private final double kI;
    private final double kD;
    private final double maxVel;
    private final double maxAccel;

    public PIDPreset(double kP, double kI, double kD, double maxVel, double maxAccel) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    public PIDPreset(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0);
    }


    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public double getMaxVel() {
        return maxVel;
    }

    public double getMaxAccel() {
        return maxAccel;
    }
}
