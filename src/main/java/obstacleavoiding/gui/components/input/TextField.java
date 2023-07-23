package obstacleavoiding.gui.components.input;

import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.settings.Settings;

import javax.swing.*;

public class TextField<T> extends JFormattedTextField implements InputComponent {
    public TextField(Dimension2d size, Dimension2d location, T value) {
        this.setValue(value);
        this.setColumns(1);
        this.setSize(size.getX(), size.getY());
        this.setLocation(location.getX(), location.getY());
    }

    @Override
    public T getValue() {
        return (T) super.getValue();
    }

    @Override
    public void setLocation(int lastY) {
        this.setLocation(Settings.GAP, Settings.GAP + lastY);
    }
}
