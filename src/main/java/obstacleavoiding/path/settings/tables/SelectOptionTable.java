package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.input.MultipleOption;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;

import java.util.Arrays;
import java.util.List;

public class SelectOptionTable<T> extends TableType<T> {

    private static final int HEIGHT = 30;

    private final T[] options;
    private MultipleOption<T> multipleOption;

    public SelectOptionTable(String name, T defaultOption, T... options) {
        super(name, defaultOption);
        this.options = options;
    }

    @Override
    public int getLastY() {
        return this.multipleOption.getY() + this.multipleOption.getHeight();
    }

    @Override
    public T getValue() {
        return this.multipleOption.getSelectedOption();
    }

    @Override
    public void setValue(Object value) {
        this.multipleOption.setSelectedOption((T) value);
    }

    @Override
    public List<Component> getComponents(int lastY, int gap) {
        this.multipleOption = new MultipleOption<>(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * gap), HEIGHT),
                new Dimension2d(gap, lastY + gap),
                this.getDefaultValue(),
                this.options);

        return Arrays.asList(this.multipleOption);
    }
}
