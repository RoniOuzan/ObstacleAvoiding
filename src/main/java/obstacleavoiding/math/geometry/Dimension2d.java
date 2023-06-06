package obstacleavoiding.math.geometry;

import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.interpolation.Interpolatable;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

public class Dimension2d implements Interpolatable<Dimension2d> {
    private final int m_x;
    private final int m_y;

    /** Constructs a Translation2d with X and Y components equal to zero. */
    public Dimension2d() {
        this(0, 0);
    }

    /**
     * Constructs a Translation2d with the X and Y components equal to the provided values.
     *
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    public Dimension2d(int x, int y) {
        m_x = x;
        m_y = y;
    }

    /**
     * Calculates the distance between two translations in 2D space.
     *
     * <p>The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
     *
     * @param other The translation to compute the distance to.
     * @return The distance between the two translations.
     */
    public double getDistance(Dimension2d other) {
        return Math.hypot(other.m_x - m_x, other.m_y - m_y);
    }

    /**
     * Returns the X component of the translation.
     *
     * @return The X component of the translation.
     */
    public int getX() {
        return m_x;
    }

    /**
     * Returns the Y component of the translation.
     *
     * @return The Y component of the translation.
     */
    public int getY() {
        return m_y;
    }

    /**
     * Returns the norm, or distance from the origin to the translation.
     *
     * @return The norm of the translation.
     */
    public double getNorm() {
        return Math.hypot(m_x, m_y);
    }

    /**
     * Returns the angle this translation forms with the positive X axis.
     *
     * @return The angle of the translation
     */
    public Rotation2d getAngle() {
        return new Rotation2d(m_x, m_y);
    }

    /**
     * Applies a rotation to the translation in 2D space.
     *
     * <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
     * angle.
     *
     * <pre>
     * [x_new]   [other.cos, -other.sin][x]
     * [y_new] = [other.sin,  other.cos][y]
     * </pre>
     *
     * <p>For example, rotating a Translation2d of &lt;2, 0&gt; by 90 degrees will return a
     * Translation2d of &lt;0, 2&gt;.
     *
     * @param other The rotation to rotate the translation by.
     * @return The new rotated translation.
     */
    public Dimension2d rotateBy(Rotation2d other) {
        return new Dimension2d(
                (int) (m_x * other.getCos() - m_y * other.getSin()),
                (int) (m_x * other.getSin() + m_y * other.getCos())
        );
    }

    /**
     * Returns the sum of two translations in 2D space.
     *
     * <p>For example, Translation3d(1.0, 2.5) + Translation3d(2.0, 5.5) = Translation3d{3.0, 8.0).
     *
     * @param other The translation to add.
     * @return The sum of the translations.
     */
    public Dimension2d plus(Dimension2d other) {
        return new Dimension2d(m_x + other.m_x, m_y + other.m_y);
    }

    /**
     * Returns the difference between two translations.
     *
     * <p>For example, Translation2d(5.0, 4.0) - Translation2d(1.0, 2.0) = Translation2d(4.0, 2.0).
     *
     * @param other The translation to subtract.
     * @return The difference between the two translations.
     */
    public Dimension2d minus(Dimension2d other) {
        return new Dimension2d(m_x - other.m_x, m_y - other.m_y);
    }

    /**
     * Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees,
     * flipping the point over both axes, or negating all components of the translation.
     *
     * @return The inverse of the current translation.
     */
    public Dimension2d unaryMinus() {
        return new Dimension2d(-m_x, -m_y);
    }

    /**
     * Returns the translation multiplied by a scalar.
     *
     * <p>For example, Translation2d(2.0, 2.5) * 2 = Translation2d(4.0, 5.0).
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled translation.
     */
    public Dimension2d times(double scalar) {
        return new Dimension2d((int) (m_x * scalar), (int) (m_y * scalar));
    }

    /**
     * Returns the translation divided by a scalar.
     *
     * <p>For example, Translation3d(2.0, 2.5) / 2 = Translation3d(1.0, 1.25).
     *
     * @param scalar The scalar to multiply by.
     * @return The reference to the new mutated object.
     */
    public Dimension2d div(double scalar) {
        return new Dimension2d((int) (m_x / scalar), (int) (m_y / scalar));
    }

    /**
     * Returns the nearest Translation2d from a list of translations.
     *
     * @param translations The list of translations.
     * @return The nearest Translation2d from the list.
     */
    public Dimension2d nearest(List<Dimension2d> translations) {
        return Collections.min(translations, Comparator.comparing(this::getDistance));
    }

    @Override
    public String toString() {
        return String.format("Translation2d(X: %d, Y: %d)", m_x, m_y);
    }

    /**
     * Checks equality between this Translation2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Dimension2d) {
            return Math.abs(((Dimension2d) obj).m_x - m_x) < 1E-9
                    && Math.abs(((Dimension2d) obj).m_y - m_y) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_x, m_y);
    }

    @Override
    public Dimension2d interpolate(Dimension2d endValue, double t) {
        return new Dimension2d(
                (int) MathUtil.interpolate(this.getX(), endValue.getX(), t),
                (int) MathUtil.interpolate(this.getY(), endValue.getY(), t));
    }
}

