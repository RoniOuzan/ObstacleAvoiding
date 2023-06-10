package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.input.Slider;
import obstacleavoiding.gui.output.Text;
import obstacleavoiding.math.MathUtil;
import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.path.GUI;
import obstacleavoiding.path.settings.Settings;

import java.awt.*;
import java.util.Arrays;
import java.util.List;

public class SliderTable extends TableType<Double> {
    private static final int SLIDER_HEIGHT = 20;
    private static final int TEXT_SIZE = 14;

    private final double minimum;
    private final double maximum;

    private int minX;
    private int maxX;

    private Text nameText;
    private Slider slider;
    private Text minText;
    private Text maxText;
    private Text currentText;

    public SliderTable(String name, double defaultValue, double minimum, double maximum) {
        super(name, defaultValue);
        this.minimum = minimum;
        this.maximum = maximum;
    }

    @Override
    public int getLastY() {
        return this.currentText.getY() + this.currentText.getHeight();
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
        this.minX = gap;
        this.maxX = GUI.SETTINGS_WIDTH - gap - 15;

        this.nameText = new Text(this.getName() + ":",
                new Dimension2d(100, TEXT_SIZE),
                new Dimension2d(gap, lastY + gap))
                .setTextSize(TEXT_SIZE).setTextColor(Color.WHITE);

        this.slider = new Slider(
                new Dimension2d(GUI.SETTINGS_WIDTH - (2 * gap), SLIDER_HEIGHT),
                new Dimension2d(gap, this.nameText.getY() + TEXT_SIZE),
                this.getDefaultValue(), this.maximum, this.minimum).setBackgroundColor(Settings.BACKGROUND);

        this.currentText = new Text(
                doubleToString(this.getDefaultValue()),
                new Dimension2d(30, TEXT_SIZE),
                new Dimension2d((int) (MathUtil.deadband(this.getDefaultValue(), this.minimum, this.maximum) * (maxX - minX) + minX),
                        this.slider.getY() + this.slider.getHeight()))
                .setTextSize(TEXT_SIZE).setTextColor(Color.WHITE);

        this.minText = new Text(
                doubleToString(this.minimum),
                new Dimension2d(30, TEXT_SIZE),
                new Dimension2d(minX, this.slider.getY() + this.slider.getHeight()))
                .setTextSize(TEXT_SIZE).setTextColor(Color.WHITE);

        this.maxText = new Text(
                doubleToString(this.maximum),
                new Dimension2d(30, TEXT_SIZE),
                new Dimension2d(maxX, this.slider.getY() + this.slider.getHeight()))
                .setTextSize(TEXT_SIZE).setTextColor(Color.WHITE);

        return Arrays.asList(this.nameText, this.slider, this.currentText, this.minText, this.maxText);
    }

    @Override
    public void update() {
        this.currentText.setLocation((int) ((MathUtil.clamp(this.getValue() - this.minimum, 0, this.maximum) / (this.maximum - this.minimum)) * (maxX - minX) + minX),
                this.currentText.getY());
        this.currentText.setText(doubleToString(this.getValue()));
    }

    private static String doubleToString(double num) {
        if (num % 1 == 0)
            return Integer.toString((int) num);
        return Double.toString(num);
    }
}
