package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.gui.components.input.DoubleSlider;
import obstacleavoiding.gui.components.input.IntegerSlider;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;

import java.util.Collections;
import java.util.List;

public class IntegerSliderTable extends TableType<Integer> {
    private static final int SLIDER_HEIGHT = 45;

    private final int minimum;
    private final int maximum;

    private IntegerSlider slider;

    public IntegerSliderTable(String name, int defaultValue, int minimum, int maximum) {
        super(name, defaultValue);
        this.minimum = minimum;
        this.maximum = maximum;
    }

    @Override
    public int getLastY() {
        return this.slider.getY() + this.slider.getHeight();
    }

    @Override
    public Integer getCurrentValue() {
        return (int) this.slider.getCurrentValue();
    }

    @Override
    public void setValue(Object value) {
        this.slider.setCurrentValue((Integer) value);
    }

    @Override
    public List<Component> getComponents(int lastY, int gap) {
        this.slider = new IntegerSlider(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * gap), SLIDER_HEIGHT),
                new Dimension2d(gap, lastY + gap),
                this.getName(),
                this.getDefaultValue(), this.minimum, this.maximum)
                .setBackgroundColor(Settings.BACKGROUND).setColor(GUI.COLOR);

        return Collections.singletonList(this.slider);
    }
}
