package obstacleavoiding.path.settings.tables;

import obstacleavoiding.gui.components.Component;

import java.util.List;
import java.util.function.Consumer;

public abstract class TableType<T> {
    private final String name;
    private final T defaultValue;

    private boolean changeable = true;

    private Consumer<T> valueParser = t -> {};

    public TableType(String name, T defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
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

    public void parse() {
        this.valueParser.accept(this.getValue());
    }

    public String getName() {
        return name;
    }

    public T getDefaultValue() {
        return defaultValue;
    }

    public abstract int getLastY();

    public T getValue() {
        if (this.changeable) {
            return this.getCurrentValue();
        }
        return this.defaultValue;
    }

    public abstract T getCurrentValue();
    public abstract void setValue(Object value);

    public abstract List<Component> getComponents(int lastY, int gap);

    public void update() {}
}
