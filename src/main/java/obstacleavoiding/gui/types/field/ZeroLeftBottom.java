package obstacleavoiding.gui.types.field;

import obstacleavoiding.gui.FieldType;
import obstacleavoiding.math.geometry.Dimension2d;

public interface ZeroLeftBottom extends FieldType {
    @Override
    default int convertY(double y, Dimension2d dimension) {
        return (int) (dimension.getY() - y);
    }

    @Override
    default double convertPixelsToX(double pixels, Dimension2d dimension) {
        return FieldType.super.convertPixelsToX(pixels, dimension);
    }

    @Override
    default double convertPixelsToY(double pixels, Dimension2d dimension) {
        return FieldType.super.convertPixelsToY(dimension.getY() - pixels, dimension);
    }

    default double convertPixelsToYFieldType(double pixels, Dimension2d dimension) {
        return FieldType.super.convertPixelsToY(pixels, dimension);
    }
}
