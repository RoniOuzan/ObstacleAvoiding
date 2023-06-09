package obstacleavoiding.gui.input;

import obstacleavoiding.math.geometry.Dimension2d;

import javax.swing.*;

public class TextField<T> extends JFormattedTextField {
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
}
