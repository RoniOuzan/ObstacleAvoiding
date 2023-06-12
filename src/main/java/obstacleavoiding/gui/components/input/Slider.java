package obstacleavoiding.gui.components.input;

import obstacleavoiding.gui.components.output.Text;
import obstacleavoiding.math.geometry.Dimension2d;

import javax.swing.*;
import java.awt.*;

public class Slider extends JSlider implements InputComponent {
    public Slider(Dimension2d size, Dimension2d location, double value, double maximum, double minimum) {
        super(JSlider.HORIZONTAL, (int) (minimum * 10), (int) (maximum * 10), (int) (value * 10));
        this.setSize(size.getX(), size.getY());
        this.setLocation(location.getX(), location.getY());
        this.setFont(new Font("Serif", Font.ITALIC, 15));
        this.setOrientation(JSlider.HORIZONTAL);
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

    public Slider setColor(Color color) {
        this.setForeground(color);
        return this;
    }
}
