package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.DoubleSlider;
import obstacleavoiding.gui.components.input.IntegerSlider;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;
import obstacleavoiding.path.util.ValuesMode;

import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;

public class IntegerSliderTable extends TableType<Integer> {
    private static final int SLIDER_HEIGHT = 45;

    private final int minimum;
    private final int maximum;

    private IntegerSlider slider;

    private BiConsumer<Integer, Integer> onChange = (c, t) -> {};
    private int lastValue = 0;

    public IntegerSliderTable(String name, ValuesMode mode, int defaultValue, int minimum, int maximum) {
        super(name, mode, defaultValue);
        this.minimum = minimum;
        this.maximum = maximum;
    }

    public TableType<Integer> onChange(BiConsumer<Integer, Integer> onChange) {
        this.onChange = onChange;
        return this;
    }

    public boolean isJustChanged() {
        return lastValue != this.getValue();
    }

    @Override
    public int getLastY() {
        return this.slider.getY() + this.slider.getHeight();
    }

    @Override
    public Integer getValue() {
        return (int) this.slider.getCurrentValue();
    }

    @Override
    public void setValue(Object value) {
        this.slider.setCurrentValue((Integer) value);
    }

    @Override
    public List<Component> getComponents() {
        this.slider = new IntegerSlider(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * Settings.GAP), SLIDER_HEIGHT),
                new Dimension2d(Settings.GAP, Settings.GAP),
                this.getName(),
                this.getDefaultValue(), this.minimum, this.maximum)
                .setBackgroundColor(Settings.BACKGROUND).setColor(GUI.COLOR);

        return Collections.singletonList(this.slider);
    }

    @Override
    public void update() {
        int value = this.getValue();

        if (this.isJustChanged()) {
            this.onChange.accept(value, lastValue);
        }

        lastValue = value;
    }
}
