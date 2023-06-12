package obstacleavoiding.gui.components.input;

import obstacleavoiding.math.geometry.Dimension2d;

import javax.swing.*;
import javax.swing.plaf.basic.BasicComboBoxUI;
import java.awt.*;

public class MultipleOption<T> extends JComboBox<T> implements InputComponent {

    private final T[] options;

    @SafeVarargs
    public MultipleOption(Dimension2d size, Dimension2d location, T defaultOption, T... options) {
        super(options);
        this.setSize(size.getX(), size.getY());
        this.setLocation(location.getX(), location.getY());
        this.setSelectedIndex(getIndexOfArray(defaultOption, options));
        this.setFocusable(true);
        this.setEditable(false);
        this.setAutoscrolls(false);

        this.options = options;
    }

    public T getSelectedOption() {
        return options[this.getSelectedIndex()];
    }

    public void setSelectedOption(T option) {
        this.setSelectedItem(0);
    }

    public MultipleOption<T> setBackgroundColor(Color color) {
        this.setBackground(color);
        return this;
    }

    public MultipleOption<T> setTextColor(Color color) {
        this.setForeground(color);
        return this;
    }

    private static int getIndexOfArray(Object value, Object[] values) {
        for (int i = 0; i < values.length; i++) {
            if (value.equals(values[i]))
                return i;
        }
        return 0;
    }
}
