package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.DoubleSlider;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;
import obstacleavoiding.path.util.ValuesMode;

import java.util.Collections;
import java.util.List;
import java.util.function.BiConsumer;

public class DoubleSliderTable extends TableType<Double> {
    private static final int SLIDER_HEIGHT = 45;

    private final double minimum;
    private final double maximum;

    private DoubleSlider slider;

    private BiConsumer<Double, Double> onChange = (c, t) -> {};
    private double lastValue = 0;

    public DoubleSliderTable(String name, ValuesMode mode, double defaultValue, double minimum, double maximum) {
        super(name, mode, defaultValue);
        this.minimum = minimum;
        this.maximum = maximum;
    }

    public TableType<Double> onChange(BiConsumer<Double, Double> onChange) {
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
    public Double getValue() {
        return this.slider.getCurrentValue();
    }

    @Override
    public void setValue(Object value) {
        this.slider.setCurrentValue((Double) value);
    }

    @Override
    public List<Component> getComponents() {
        this.slider = new DoubleSlider(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * Settings.GAP), SLIDER_HEIGHT),
                new Dimension2d(Settings.GAP, Settings.GAP),
                this.getName(),
                this.getDefaultValue(), this.minimum, this.maximum)
                .setBackgroundColor(Settings.BACKGROUND).setColor(GUI.COLOR);

        return Collections.singletonList(this.slider);
    }

    @Override
    public void update() {
        double value = this.getValue();

        if (this.isJustChanged()) {
            this.onChange.accept(value, lastValue);
        }

        lastValue = value;
    }
}
