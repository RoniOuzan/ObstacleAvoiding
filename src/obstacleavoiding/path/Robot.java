package obstacleavoiding.path;

import obstacleavoiding.math.geometry.*;

public class Robot {
    private Pose2d position;
    private Pose2d velocity;

    private Pose2d lastVelocity = new Pose2d();

    private final Constants constants;

    private double difference = 0;
    private long lastUpdate = 0;

    public Robot(Pose2d position, Constants constants) {
        this.position = position;
        this.velocity = new Pose2d();
        this.constants = constants;
    }

    public void drive(Pose2d velocity) {
        this.lastVelocity = this.velocity;
        this.difference = (System.currentTimeMillis() - this.lastUpdate) / 1000d;

        if (velocity.getTranslation().getNorm() > constants.maxVel) {
            velocity = new Pose2d(
                    new Translation2d(constants.maxVel, velocity.getTranslation().getAngle()),
                    velocity.getRotation());
        }

        this.position = new Pose2d(
                        this.position.getTranslation().plus(velocity.getTranslation().times(constants.period)),
                this.position.getRotation().rotateBy(Rotation2d.fromDegrees(velocity.getRotation().getDegrees() * constants.period)));
        this.velocity = velocity;

        this.lastUpdate = System.currentTimeMillis();
    }

    public void setAngle(double degrees) {
        this.position = new Pose2d(this.position.getTranslation(), Rotation2d.fromDegrees(degrees));
    }

    public double getAcceleration() {
        return (this.velocity.getTranslation().getNorm() - this.lastVelocity.getTranslation().getNorm()) / this.difference;
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
