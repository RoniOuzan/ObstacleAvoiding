package obstacleavoiding.gui.components.input;

import obstacleavoiding.math.geometry.Dimension2d;

import javax.swing.*;
import java.awt.*;

public class Slider extends JSlider implements InputComponent {
    public Slider(Dimension2d size, Dimension2d location, double value, double maximum, double minimum) {
        super((int) (minimum * 10), (int) (maximum * 10), (int) (value * 10));
        this.setSize(size.getX(), size.getY());
        this.setLocation(location.getX(), location.getY());
        this.setMajorTickSpacing(10);
        this.setMinorTickSpacing(1);
        this.setPaintTicks(false);
        this.setPaintLabels(false);
    }

    public double getCurrentValue() {
        return super.getValue() / 10d;
    }

    public void setCurrentValue(double value) {
        this.setValue((int) (value * 10));
    }

    public Slider setBackgroundColor(Color color) {
        this.setBackground(color);
        return this;
    }
}
