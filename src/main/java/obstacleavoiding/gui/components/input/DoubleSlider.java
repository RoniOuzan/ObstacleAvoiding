package obstacleavoiding.gui.components.input;

import obstacleavoiding.math.geometry.Dimension2d;

import java.awt.*;

public class DoubleSlider extends Slider {

    public DoubleSlider(Dimension2d size, Dimension2d location, String name, double value, double minimum, double maximum) {
        super(size, location, name, (int) (value * 10), (int) (minimum * 10), (int) (maximum * 10));
    }

    public double getCurrentValue() {
        return super.getValue() / 10d;
    }

    public void setCurrentValue(double value) {
        this.setValue((int) (value * 10));
    }

    @Override
    public DoubleSlider setBackgroundColor(Color color) {
        return (DoubleSlider) super.setBackgroundColor(color);
    }

    @Override
    public DoubleSlider setColor(Color color) {
        return (DoubleSlider) super.setColor(color);
    }
}
