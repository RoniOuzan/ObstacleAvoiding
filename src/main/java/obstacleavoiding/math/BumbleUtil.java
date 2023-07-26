package obstacleavoiding.math;

import obstacleavoiding.math.geometry.Pose2d;
import obstacleavoiding.math.geometry.Rotation2d;
import obstacleavoiding.math.geometry.Translation2d;

/**
 * Useful utility functions and variables.
 */
public class BumbleUtil {

    private BumbleUtil() {
    }

    /**
     * A general property that represent any use of an invalid value.
     */
    public static final double INVALID_VALUE = -3339;

    public static double deadband(double axisValue, double deadband, boolean toNormalize) {
        if (Math.abs(axisValue) < deadband) {
            return 0;
        } else {
            if (toNormalize) {
                return (1 / (1 - deadband)) * (Math.abs(axisValue) - deadband) * Math.signum(axisValue);
            } else {
                return axisValue;
            }
        }
    }

    public static double deadband(double axisValue, double deadband) {
        return deadband(axisValue, deadband, true);
    }

    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double bound90Degrees(double angleDegrees) {
        return MathUtil.inputModulus(angleDegrees, -90, 90);
    }

    public static double bound180Degrees(double angleDegrees) {
        return MathUtil.inputModulus(angleDegrees, -180, 180);
    }

    public static double boundPIRadians(double angleRadians) {
        return MathUtil.angleModulus(angleRadians);
    }

    public static double bound2PIRadians(double angleRadians) {
        return MathUtil.inputModulus(angleRadians, 0, Math.PI * 2);
    }

    public static double bound360Degrees(double angleDegrees) {
        return MathUtil.inputModulus(angleDegrees, 0, 360);
    }

    public static boolean inTolerance(double value, double target, double margin) {
        return Math.abs(value - target) <= margin;
    }

    public static Rotation2d bound360Degrees(Rotation2d rotation) {
        return Rotation2d.fromDegrees(bound360Degrees(rotation.getDegrees()));
    }

    /**
     * @param point point in an x,y plance
     * @param rotateAround a point in an x,y plane you want to rotate "point" around
     * @param angle angle anti-clockwise, to rotate the point
     * @return the returning point
     */
    public static Translation2d rotateAround(Translation2d point, Translation2d rotateAround, Rotation2d angle) {
        double x1 = point.getX() - rotateAround.getX();
        double y1 = point.getY() - rotateAround.getY();

        double x2 = x1 * angle.getCos() - y1 * angle.getSin();
        double y2 = x1 * angle.getSin() + y1 * angle.getCos();

        return new Translation2d(x2 + rotateAround.getX(), y2 + rotateAround.getY());
    }

    /**
     * @param point the point in the original coordinate system
     * @param center the center of the new coordinate system, placed in the old, the rotation2d pointing to the x axis direction
     * @param invertX if false, center = 0,0,0 will point right and up. if true center = 0,0,0 will point left and up
     * @return
     */
    public static Pose2d changeCoordinateSystem(Pose2d point, Pose2d center, boolean invertX) {
        Translation2d temp =  rotateAround(point.getTranslation(),center.getTranslation(),center.getRotation());
        if(invertX) {
            temp = new Translation2d(-temp.getX(), temp.getY());
        }
        return new Pose2d(temp.minus(center.getTranslation()), 
        point.getRotation().plus(center.getRotation()));
    }
}
