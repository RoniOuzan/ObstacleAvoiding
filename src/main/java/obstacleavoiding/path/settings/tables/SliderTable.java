package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.Slider;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;

import java.awt.*;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class SliderTable extends TableType<Double> {
    private static final int SLIDER_HEIGHT = 50;

    private final double minimum;
    private final double maximum;

    private Slider slider;

    public SliderTable(String name, double defaultValue, double minimum, double maximum) {
        super(name, defaultValue);
        this.minimum = minimum;
        this.maximum = maximum;
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
    public List<Component> getComponents(int lastY, int gap) {
        this.slider = new Slider(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * gap), SLIDER_HEIGHT),
                new Dimension2d(gap, lastY + gap),
                this.getName(),
                this.getDefaultValue(), this.maximum, this.minimum)
                .setBackgroundColor(Settings.BACKGROUND).setColor(new Color(245, 212, 9));

        return Collections.singletonList(this.slider);
    }
}
