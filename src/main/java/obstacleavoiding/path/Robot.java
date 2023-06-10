package obstacleavoiding.path;

import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;

public class Robot {
    private Pose2d position;
    private Pose2d velocity;

    private Pose2d lastVelocity = new Pose2d();

    private final Constants constants;

    public Robot(Pose2d position, Constants constants) {
        this.position = position;
        this.velocity = new Pose2d();
        this.constants = constants;
    }

    public void drive(Pose2d velocity, double period) {
        this.lastVelocity = this.velocity;

        double calculatedVelocity = Math.min(velocity.getTranslation().getNorm(), constants.maxVel);
        velocity = new Pose2d(
                new Translation2d(calculatedVelocity, velocity.getTranslation().getAngle()),
                velocity.getRotation());

        this.position = new Pose2d(
                        this.position.getTranslation().plus(velocity.getTranslation().times(period)),
                this.position.getRotation().rotateBy(Rotation2d.fromRadians(velocity.getRotation().getRadians() * period)));
        this.velocity = velocity;
    }

    public boolean isMoving() {
        return this.velocity.getTranslation().getNorm() > 0.01;
    }

    public boolean isRotating() {
        return Math.abs(this.velocity.getRotation().getDegrees()) > 0.01;
    }

    public void setAngle(double degrees) {
        this.position = new Pose2d(this.position.getTranslation(), Rotation2d.fromDegrees(degrees));
    }

    public double getAcceleration() {
        return (this.velocity.getTranslation().getNorm() - this.lastVelocity.getTranslation().getNorm()) / constants.period;
    }

    public Pose2d getPosition() {
        return position;
    }

    public Pose2d getVelocity() {
        return velocity;
    }

    public Constants getConstants() {
        return this.constants;
    }

    public record Constants(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel, double period) {}
}
