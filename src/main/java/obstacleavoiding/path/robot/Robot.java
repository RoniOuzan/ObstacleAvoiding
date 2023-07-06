package obstacleavoiding.path.robot;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.math.matrix.Matrix;

import java.util.Arrays;

public class Robot {
    private final int numModules;
    private final ModuleState[] modules;
    private final Translation2d centerOfRotation;
    private final double modulesRadius;
    private final Matrix inverseKinematics;

    private Pose2d position;
    private Pose2d velocity;

    private Pose2d lastVelocity = new Pose2d();

    private final Constants constants;

    public Robot(Pose2d position, Constants constants) {
        this.position = position;
        this.velocity = new Pose2d();
        this.constants = constants;

        this.numModules = constants.modulesLocation.length;
        this.modules = new ModuleState[numModules];
        this.inverseKinematics = new Matrix(numModules * 2, 3);
        for (int i = 0; i < numModules; i++) {
            this.modules[i] = new ModuleState(this.constants.modulesLocation[i], 0, 0);
            this.inverseKinematics.setRow(i * 2, 1, 0, -constants.modulesLocation[i].getY());
            this.inverseKinematics.setRow(i * 2 + 1, 0, 1, constants.modulesLocation[i].getX());
        }

        this.centerOfRotation = new Translation2d(
                Arrays.stream(this.modules).mapToDouble(m -> m.getLocation().getX()).average().orElse(0),
                Arrays.stream(this.modules).mapToDouble(m -> m.getLocation().getY()).average().orElse(0)
        );
        this.modulesRadius = Arrays.stream(this.modules).mapToDouble(m -> m.getLocation().getDistance(this.centerOfRotation)).average().orElse(0);
    }

    public void drive(Pose2d velocity, double period, boolean accelerate) {
        this.lastVelocity = this.velocity;

        this.setModuleStates(velocity, period, accelerate);

        velocity = new Pose2d(
                new Translation2d(
                        Arrays.stream(this.modules).mapToDouble(m -> m.getVector().rotateBy(this.position.getRotation()).getX()).average().orElse(0),
                        Arrays.stream(this.modules).mapToDouble(m -> m.getVector().rotateBy(this.position.getRotation()).getY()).average().orElse(0)
                ),
                Rotation2d.fromRadians(-Arrays.stream(this.modules).mapToDouble(m -> m.getVector().rotateBy(centerOfRotation.minus(m.getLocation()).getAngle().unaryMinus()).getY()).average().orElse(0))
        );

        this.position = new Pose2d(
                this.position.getTranslation().plus(velocity.getTranslation().times(period)),
                this.position.getRotation().rotateBy(Rotation2d.fromRadians(velocity.getRotation().getRadians() * period)));
        this.velocity = velocity;
    }

    private void setModuleStates(Pose2d velocity, double period, boolean accelerate) {
        Matrix vector = new Matrix(3, 1);
        vector.setColumn(0, velocity.getTranslation().rotateBy(this.position.getRotation().unaryMinus()).getX(),
                        velocity.getTranslation().rotateBy(this.position.getRotation().unaryMinus()).getY(),
                        velocity.getRotation().getRadians() / this.modulesRadius);

        Matrix chassisSpeedsVector = this.inverseKinematics.multiply(vector);

        for (int i = 0; i < numModules; i++) {
            Translation2d module = new Translation2d(chassisSpeedsVector.get(i * 2, 0), chassisSpeedsVector.get(i * 2 + 1, 0));
            if (accelerate && this.modules[i].getVector().getNorm() > 0.01) {
                this.modules[i].set(MathUtil.clampWithAccel(module.getX(), this.modules[i].getVector().getX(), this.constants.maxAccel, period),
                        MathUtil.clampWithAccel(module.getY(), this.modules[i].getVector().getY(), this.constants.maxAccel, period),
                        this.constants.maxVel);
            } else {
                this.modules[i].set(module, this.constants.maxVel);
            }
        }
    }

    public ModuleState[] getModules() {
        return modules;
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

    public record Constants(double maxVel, double maxAccel, double maxOmegaVel, double maxOmegaAccel, double period, Translation2d[] modulesLocation) {}
}
