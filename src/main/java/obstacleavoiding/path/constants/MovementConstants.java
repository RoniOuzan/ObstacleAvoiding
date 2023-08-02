package obstacleavoiding.path.constants;

public class MovementConstants {
    private final double maxVel;
    private final double maxAccel;
    private final double maxOmegaVel;
    private final double maxOmegaAccel;

    public MovementConstants(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxOmegaVel = Math.toRadians(maxOmegaVel);
        this.maxOmegaAccel = Math.toRadians(maxOmegaAccel);
    }

    public double getMaxVel() {
        return maxVel;
    }

    public double getMaxAccel() {
        return maxAccel;
    }

    public double getMaxOmegaVel() {
        return maxOmegaVel;
    }

    public double getMaxOmegaAccel() {
        return maxOmegaAccel;
    }
}
