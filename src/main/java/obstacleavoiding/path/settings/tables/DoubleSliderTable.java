package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.DoubleSlider;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;
import obstacleavoiding.path.util.ValuesMode;

import java.util.Collections;
import java.util.List;

public class DoubleSliderTable extends TableType<Double> {
    private static final int SLIDER_HEIGHT = 45;

    private final double minimum;
    private final double maximum;

    private DoubleSlider slider;

    public DoubleSliderTable(String name, ValuesMode mode, double defaultValue, double minimum, double maximum) {
        super(name, mode, defaultValue);
        this.minimum = minimum;
        this.maximum = maximum;
    }

    @Override
    public int getLastY() {
        return this.slider.getY() + this.slider.getHeight();
    }

    @Override
    public Double getCurrentValue() {
        return this.slider.getCurrentValue();
    }

    @Override
    public void setCurrentValue(Object value) {
        this.slider.setCurrentValue((Double) value);
    }

    @Override
    public List<Component> getComponents(int lastY, int gap) {
        this.slider = new DoubleSlider(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * gap), SLIDER_HEIGHT),
                new Dimension2d(gap, lastY + gap),
                this.getName(),
                this.getDefaultValue(), this.minimum, this.maximum)
                .setBackgroundColor(Settings.BACKGROUND).setColor(GUI.COLOR);

        return Collections.singletonList(this.slider);
    }
}
