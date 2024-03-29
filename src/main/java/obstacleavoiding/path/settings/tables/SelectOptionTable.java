package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.MultipleOption;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;
import obstacleavoiding.path.util.ValuesMode;

import java.awt.*;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class SelectOptionTable<T> extends TableType<T> {

    private static final int HEIGHT = 30;

    private final T[] options;
    private MultipleOption<T> multipleOption;

    private BiConsumer<T, T> onChange = (c, t) -> {};

    private T lastT = null;

    @SafeVarargs
    public SelectOptionTable(String name, ValuesMode mode, T defaultOption, T... options) {
        super(name, mode, defaultOption);
        this.options = options;
    }

    public SelectOptionTable<T> onChange(BiConsumer<T, T> onChange) {
        this.onChange = onChange;
        return this;
    }

    public boolean isJustChanged() {
        return lastT != null && !lastT.equals(this.getValue());
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
    public List<Component> getComponents() {
        this.multipleOption = new MultipleOption<>(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * Settings.GAP), HEIGHT),
                new Dimension2d(Settings.GAP, Settings.GAP),
                this.getDefaultValue(),
                this.options)
                .setBackgroundColor(Settings.BACKGROUND).setTextColor(Color.WHITE);
        this.multipleOption.setName(this.getName());

        return Arrays.asList(this.multipleOption);
    }

    @Override
    public void update() {
        T t = this.getValue();

        if (this.isJustChanged()) {
            this.onChange.accept(t, lastT);
        }

        lastT = t;
    }
}
