package obstacleavoiding.math;

import java.util.function.Function;

public final class MathUtil {
    private static final double DX = 0.0001;

    private MathUtil() {
        throw new AssertionError("utility class");
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    public static double clampWithAccel(double value, double last, double accel, double period) {
        return last + clamp(value - last, -accel * period, accel * period);
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
     *
     * @param value Value to clip.
     * @param deadband Range around zero.
     * @param maxMagnitude The maximum magnitude of the input. Can be infinite.
     * @return The value after the deadband is applied.
     */
    public static double applyDeadband(double value, double deadband, double maxMagnitude) {
        if (Math.abs(value) > deadband) {
            if (maxMagnitude / deadband > 1.0e12) {
                // If max magnitude is sufficiently large, the implementation encounters
                // roundoff error.  Implementing the limiting behavior directly avoids
                // the problem.
                return value > 0.0 ? value - deadband : value + deadband;
            }
            if (value > 0.0) {
                // Map deadband to 0 and map max to max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (deadband, 0) and (x₂, y₂) = (max, max).
                // x₁ = deadband
                // y₁ = 0
                // x₂ = max
                // y₂ = max
                //
                // y = (max - 0)/(max - deadband) (x - deadband) + 0
                // y = max/(max - deadband) (x - deadband)
                // y = max (x - deadband)/(max - deadband)
                return maxMagnitude * (value - deadband) / (maxMagnitude - deadband);
            } else {
                // Map -deadband to 0 and map -max to -max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (-deadband, 0) and (x₂, y₂) = (-max, -max).
                // x₁ = -deadband
                // y₁ = 0
                // x₂ = -max
                // y₂ = -max
                //
                // y = (-max - 0)/(-max + deadband) (x + deadband) + 0
                // y = max/(max - deadband) (x + deadband)
                // y = max (x + deadband)/(max - deadband)
                return maxMagnitude * (value + deadband) / (maxMagnitude - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value Value to clip.
     * @param deadband Range around zero.
     * @return The value after the deadband is applied.
     */
    public static double applyDeadband(double value, double deadband) {
        return applyDeadband(value, deadband, 1);
    }

    public static double deadband(double axisValue, double deadband, double maxMagnitude) {
        if (Math.abs(axisValue) < deadband)
            return 0;
        if (axisValue > 1) {
            return 1;
        } else {
            return (maxMagnitude / (maxMagnitude - deadband)) * (Math.abs(axisValue) - deadband) * Math.signum(axisValue);
        }
    }

    /**
     * Returns modulus of input.
     *
     * @param input Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Wraps an angle to the range -pi to pi radians.
     *
     * @param angleRadians Angle to wrap in radians.
     * @return The wrapped angle.
     */
    public static double angleModulus(double angleRadians) {
        return inputModulus(angleRadians, -Math.PI, Math.PI);
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue The value to end at.
     * @param t How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    public static double interpolate(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * MathUtil.clamp(t, 0, 1);
    }

    /**
     * Removes small numbers to make the value more readable or round it to a specific use
     *
     * @param value The value that we want to change
     * @param numbersAfterDot The amount of number after the dot to save
     * @return The new number after the changes
     */
    public static double limitDot(double value, int numbersAfterDot) {
        double power = Math.pow(10, numbersAfterDot);
        return Math.round(value * power) / power;
    }

    public static boolean inTolerance(double value, double target, double tolerance) {
        return Math.abs(target - value) <= tolerance;
    }

    public static boolean inRage(double value, double max, double min) {
        return value <= max && value >= min;
    }

    public static double calculateDerivative(double x, Function<Double, Double> function) {
        return (function.apply(x + DX) - function.apply(x - DX)) / (2 * DX);
    }

    public static double calculateIntegral(double a, double b, Function<Double, Double> function) {
        double sum = 0;
        for (double t = a; t < b; t += DX) {
            sum += (function.apply(t - DX) + function.apply(t + DX)) * (DX / 2d);
        }
        return sum;
    }
}

