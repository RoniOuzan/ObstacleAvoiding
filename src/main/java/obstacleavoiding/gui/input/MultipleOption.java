package obstacleavoiding.gui.input;

import obstacleavoiding.gui.components.input.InputComponent;
import obstacleavoiding.math.geometry.Dimension2d;

import javax.swing.*;

public class MultipleOption<T> extends JComboBox<T> implements InputComponent {

    private final T[] options;

    public MultipleOption(Dimension2d size, Dimension2d location, T defaultOption, T... options) {
        super(options);
        this.setSize(size.getX(), size.getY());
        this.setLocation(location.getX(), location.getY());
        this.setSelectedIndex(1);

        this.options = options;
    }

    public T getSelectedOption() {
        return options[this.getSelectedIndex()];
    }

    public void setSelectedOption(T option) {
        this.setSelectedItem(0);
    }
}
