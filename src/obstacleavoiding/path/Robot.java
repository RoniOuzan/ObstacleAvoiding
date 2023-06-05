package obstacleavoiding.path;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.*;

public class Robot {
    private Pose2d position;
    private Pose2d velocity;

    private Pose2d lastVelocity = new Pose2d();

    private final Constants constants;

    private double period = 0;
    private long lastUpdate = 0;

    public Robot(Pose2d position, Constants constants) {
        this.position = position;
        this.velocity = new Pose2d();
        this.constants = constants;
    }

    public void drive(Pose2d velocity) {
        this.lastVelocity = this.velocity;
        this.period = (System.currentTimeMillis() - this.lastUpdate) / 1000d;

        double calculatedVelocity = Math.min(velocity.getTranslation().getNorm(), constants.maxVel - (MathUtil.inputModulus(velocity.getRotation().getRadians(), 0, 2 * Math.PI) / 4));
        velocity = new Pose2d(
                new Translation2d(calculatedVelocity, velocity.getTranslation().getAngle()),
                velocity.getRotation());

        this.position = new Pose2d(
                        this.position.getTranslation().plus(velocity.getTranslation().times(constants.period)),
                this.position.getRotation().rotateBy(Rotation2d.fromRadians(velocity.getRotation().getRadians() * constants.period)));
        this.velocity = velocity;

        this.lastUpdate = System.currentTimeMillis();
    }

    public void setAngle(double degrees) {
        this.position = new Pose2d(this.position.getTranslation(), Rotation2d.fromDegrees(degrees));
    }

    public double getAcceleration() {
        return (this.velocity.getTranslation().getNorm() - this.lastVelocity.getTranslation().getNorm()) / this.period;
    }

    public Pose2d getPosition() {
        return position;
    }

    public void setPosition(Pose2d position) {
        this.position = position;
    }

    public Pose2d getVelocity() {
        return velocity;
    }

    public Pose2d getRobotRelativeVelocity() {
        return new Pose2d(this.velocity.getTranslation().rotateBy(this.position.getRotation()), this.velocity.getRotation());
    }

    public record Constants(double maxVel, double period) {}
}
