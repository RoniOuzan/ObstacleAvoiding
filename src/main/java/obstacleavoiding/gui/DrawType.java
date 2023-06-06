package obstacleavoiding.gui;

public interface DrawType {
    default int convertXWithSize(double x, double size) {
        return (int) x;
    }
    default int convertYWithSize(double y, double size) {
        return (int) y;
    }
    default int convertWidth(double width) {
        return (int) width;
    }
    default int convertHeight(double height) {
        return (int) height;
    }
}
