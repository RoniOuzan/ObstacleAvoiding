package obstacleavoiding.path.fields;

import obstacleavoiding.math.geometry.Translation2d;
import obstacleavoiding.path.obstacles.Obstacle;
import obstacleavoiding.path.util.Alliance;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class Field {
    protected final String name;
    
    protected final double width;
    protected final double height;

    protected final Image image;

    protected final FieldType fieldType;
    protected final List<Obstacle> obstacles;

    public Field(String name, double width, double height, FieldType fieldType) {
        this.name = name;
        this.width = width;
        this.height = height;
        this.image = new ImageIcon("src/main/java/obstacleavoiding/path/images/" + name + "Field.png").getImage();
        this.fieldType = fieldType;
        this.obstacles = new ArrayList<>(this.generateBlueObstacles());
        this.obstacles.addAll(this.obstacles.stream().map(o -> this.fieldType.convertToRedObstacle(o, this.getBounds())).toList());
        this.obstacles.addAll(this.generateGeneralObstacles());
        this.obstacles.addAll(Arrays.asList(
                        new Obstacle("RightX", Alliance.NONE,
                                new Translation2d(this.width, this.height),
                                new Translation2d(this.width + 0.1, this.height),
                                new Translation2d(this.width + 0.1, 0),
                                new Translation2d(this.width, 0)),
                        new Obstacle("LeftX", Alliance.NONE,
                                new Translation2d(0, this.height),
                                new Translation2d(-0.1, this.height),
                                new Translation2d(-0.1, -this.height),
                                new Translation2d(0, -this.height)),
                        new Obstacle("UpY", Alliance.NONE,
                                new Translation2d(0, this.height + 0.1),
                                new Translation2d(this.width, this.height + 0.1),
                                new Translation2d(this.width, this.height),
                                new Translation2d(0, this.height)),
                        new Obstacle("DownY", Alliance.NONE,
                                new Translation2d(0, -0.1),
                                new Translation2d(this.width, -0.1),
                                new Translation2d(this.width, 0),
                                new Translation2d(0, 0))
                        ));
    }

    public Field(String name, FieldType fieldType) {
        this(name, 16.54, 8.02, fieldType);
    }
    
    protected abstract List<Obstacle> generateBlueObstacles();
    protected abstract List<Obstacle> generateGeneralObstacles();

    public Translation2d getBounds() {
        return new Translation2d(this.width, this.height);
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }

    public Image getImage() {
        return image;
    }

    public List<Obstacle> getObstacles() {
        return obstacles;
    }
}
