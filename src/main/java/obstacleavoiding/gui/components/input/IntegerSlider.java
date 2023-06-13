package obstacleavoiding.gui.components.input;

import obstacleavoiding.math.geometry.Dimension2d;

import java.awt.*;

public class IntegerSlider extends Slider {
    public IntegerSlider(Dimension2d size, Dimension2d location, String name, int value, int minimum, int maximum) {
        super(size, location, name, value, minimum, maximum);
    }

    @Override
    public double getCurrentValue() {
        return super.getValue();
    }

    public void setCurrentValue(int value) {
        this.setValue(value);
    }

    @Override
    public IntegerSlider setBackgroundColor(Color color) {
        return (IntegerSlider) super.setBackgroundColor(color);
    }

    @Override
    public IntegerSlider setColor(Color color) {
        return (IntegerSlider) super.setColor(color);
    }
}
