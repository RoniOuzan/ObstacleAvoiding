package obstacleavoiding.path.fields;

public enum Fields {
    CHARGED_UP(new ChargedUpField()),
    RAPID_REACT(new RapidReactField()),
    INFINITE_RECHARGE(new InfiniteRecharge())
    ;

    private final Field field;

    Fields(Field field) {
        this.field = field;
    }

    public Field getField() {
        return field;
    }
}
