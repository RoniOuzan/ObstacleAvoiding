package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;
import obstacleavoiding.path.util.ValuesMode;

import java.util.List;
import java.util.function.Consumer;

public abstract class TableType<T> {
    private final String name;
    private T defaultValue;
    private final ValuesMode mode;

    private boolean changeable = true;

    private Consumer<T> valueParser = t -> {};

    public TableType(String name, ValuesMode mode, T defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
        this.mode = mode;
    }

    public TableType<T> setValueParser(Consumer<T> valueParser) {
        this.valueParser = valueParser;
        return this;
    }

    public boolean isChangeable() {
        return changeable;
    }

    public TableType<T> unchangeable() {
        this.changeable = false;
        return this;
    }

    public void setChangeable(boolean changeable) {
        this.changeable = changeable;
    }

    public void parse() {
        this.valueParser.accept(this.getValue());
    }

    public String getName() {
        return name;
    }

    public T getDefaultValue() {
        return defaultValue;
    }

    public ValuesMode getMode() {
        return mode;
    }

    public abstract int getLastY();

    public T getValue() {
        if (this.changeable) {
            return this.getCurrentValue();
        }
        return this.defaultValue;
    }

    public abstract T getCurrentValue();

    public void setValue(Object value) {
        if (this.changeable) {
            this.setCurrentValue(value);
        }
        this.defaultValue = (T) value;
    }

    public abstract void setCurrentValue(Object value);

    public abstract List<Component> getComponents(int lastY, int gap);

    public void update() {}
}
