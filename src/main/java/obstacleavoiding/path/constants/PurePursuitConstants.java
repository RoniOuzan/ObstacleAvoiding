package obstacleavoiding.path.constants;

public class PurePursuitConstants {
    private final double maxDriftDistance;

    private final double driftPercentLinearity;
    private final double slowPercentLinearity;
    private final double turnVelocityLinearity;

    private final double finalSlowPercentLinearity;
    private final double rotationPercentLinearity;

    private final double deceleration;
    private final double minimumDriftVelocity;

    private final double distanceTolerance;
    private final double velocityTolerance;
    private final double rotationTolerance;

    /**
     * The values that the robot will calculate the velocity of the robot relative to the path.
     * <p>
     * The linearity values controls the linearity of some percentage of values, you can check this desmos link to see the linearity of each value.
     * <a href="https://www.desmos.com/calculator/hxebmqf9vb">Linearity Illustration</a>
     *
     * @param maxDriftDistance the distance from the current waypoint that the robot will start drift to the next waypoint.
     * @param driftPercentLinearity the linearity of the drift of the robot on turns (lower value means the robot will turn sooner than higher values)
     * @param turnVelocityLinearity the linearity of the calculated velocity at turns (Lower value means more velocity than higher values)
     * @param slowPercentLinearity the linearity of the velocity of the robot on turns (lower value means the robot will slow down sooner than higher values)
     * @param finalSlowPercentLinearity the linearity of the velocity of the robot when close to the final waypoint (lower value means the robot will slow down sooner than higher values)
     * @param rotationPercentLinearity the linearity of the angle of the robot in the path (higher value means the robot will do most of the turns at the end of the path)
     * @param minimumDriftVelocity the minimum velocity that the robot can turn easily.
     * @param distanceTolerance the distance tolerance from the final waypoint that it will be counted as finished
     * @param velocityTolerance the velocity tolerance at the final waypoint that it will be counted as finished
     * @param rotationTolerance the angle tolerance at the final waypoint that it will be counted as finished
     */
    public PurePursuitConstants(double maxDriftDistance, double driftPercentLinearity,
                    double turnVelocityLinearity, double slowPercentLinearity, 
                    double finalSlowPercentLinearity, double rotationPercentLinearity,
                    double deceleration, double minimumDriftVelocity, 
                    double distanceTolerance, double velocityTolerance, double rotationTolerance) {
        this.maxDriftDistance = maxDriftDistance;
        this.driftPercentLinearity = driftPercentLinearity;
        
        this.turnVelocityLinearity = turnVelocityLinearity;
        this.slowPercentLinearity = slowPercentLinearity;
        this.finalSlowPercentLinearity = finalSlowPercentLinearity;
        this.rotationPercentLinearity = rotationPercentLinearity;

        this.deceleration = deceleration;
        this.minimumDriftVelocity = minimumDriftVelocity;

        this.distanceTolerance = distanceTolerance;
        this.velocityTolerance = velocityTolerance;
        this.rotationTolerance = rotationTolerance;
    }

    public double getMaxDriftDistance() {
        return maxDriftDistance;
    }

    public double getDriftPercentLinearity() {
        return driftPercentLinearity;
    }

    public double getTurnVelocityLinearity() {
        return turnVelocityLinearity;
    }

    public double getSlowPercentLinearity() {
        return slowPercentLinearity;
    }

    public double getFinalSlowPercentLinearity() {
        return finalSlowPercentLinearity;
    }

    public double getRotationPercentLinearity() {
        return rotationPercentLinearity;
    }

    public double getDeceleration() {
        return deceleration;
    }

    public double getMinimumDriftVelocity() {
        return minimumDriftVelocity;
    }

    public double getDistanceTolerance() {
        return distanceTolerance;
    }

    public double getVelocityTolerance() {
        return velocityTolerance;
    }

    public double getRotationTolerance() {
        return rotationTolerance;
    }
}
