package obstacleavoiding.gui;

import obstacleavoiding.math.geometry.Dimension2d;
import obstacleavoiding.math.geometry.Translation2d;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

@SuppressWarnings(value = "unused")
public abstract class Frame extends JFrame implements FieldType, DrawType {
    private final Panel panel;
    private final Dimension2d dimension;

    private double pixelsInOneUnit;

    private final List<JFrame> graphs = new ArrayList<>();

    public Frame(String title, Dimension2d frameSize, Color background, double pixelsInOneUnit) {
        super(title);
        this.dimension = frameSize;
        this.pixelsInOneUnit = pixelsInOneUnit;

        this.panel = new Panel();
        this.add(this.panel);

        this.addKeyListener(new KeyHandler());
        this.addMouseListener(new MouseHandler());
        this.addMouseMotionListener(new MouseMotionHandler());
        this.addMouseWheelListener(new MouseWheelHandler());

        this.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        this.setResizable(false);
        this.setVisible(true);
        this.setSize(frameSize.getX() + 15, frameSize.getY() + 40);
        this.setLocationRelativeTo(null);
        this.setBackground(background);
        this.repaint();
    }

    public Frame(String title, Dimension2d frameSize, double pixelsInOneUnit) {
        this(title, frameSize, Color.WHITE, pixelsInOneUnit);
    }

    public Frame(String title, Dimension2d frameSize) {
        this(title, frameSize, Color.WHITE, 1);
    }

    @Override
    public void repaint() {
        super.repaint();
        this.graphs.forEach(Component::repaint);
    }

    public Dimension2d convertDimension(Dimension2d dimension) {
        return new Dimension2d(convertX(dimension.getX(), this.dimension), convertY(dimension.getY(), this.dimension));
    }

    protected double convertUnits(double units) {
        return units * this.pixelsInOneUnit;
    }

    protected void setPixelsInOneUnit(double pixelsInOneUnit) {
        this.pixelsInOneUnit = pixelsInOneUnit;
    }

    @Override
    public double convertPixelsToUnits(double pixels) {
        return pixels / this.pixelsInOneUnit;
    }

    protected int convertXWithSize(double x, double size, Dimension2d dimension) {
        return convertX(convertXWithSize(x, size), dimension);
    }

    protected int convertYWithSize(double y, double size, Dimension2d dimension) {
        return convertY(convertYWithSize(y, size), dimension);
    }

    public void addGraph(String title, Supplier<List<Double>> values) {
        JFrame graph = Graph.createAndShowGui(title, values);
        graph.addKeyListener(new KeyHandler());
        graph.addMouseListener(new MouseHandler());
        graphs.add(graph);
    }

    public void clearFrame() {
        this.panel.graphics.clear();
        this.update();
    }

    public void update() {
//        this.repaint();
    }

    public Dimension2d getDimension() {
        return dimension;
    }

    public Translation2d getDimensionWithUnits() {
        return new Translation2d(convertPixelsToUnits(this.dimension.getX()), convertPixelsToUnits(this.dimension.getY()));
    }

    public void draw(double x, double y, Color color) {
        double X = convertUnits(x);
        double Y = convertUnits(y);

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.fillRect(convertX(X, this.dimension), convertY(Y, this.dimension), 1, 1);
        });
        this.update();
    }

    public void drawPolygon(Color color, Translation2d... translations) {
        int[] x = new int[translations.length];
        int[] y = new int[translations.length];
        for (int i = 0; i < translations.length; i++) {
            x[i] = convertX(convertUnits(translations[i].getX()), this.dimension);
            y[i] = convertY(convertUnits(translations[i].getY()), this.dimension);
        }

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.drawPolygon(x, y, translations.length);
        });
        this.update();
    }

    public void drawConnectedPoints(Color color, Translation2d... translations) {
        int[] x = new int[translations.length];
        int[] y = new int[translations.length];
        for (int i = 0; i < translations.length; i++) {
            x[i] = convertX(convertUnits(translations[i].getX()), this.dimension);
            y[i] = convertY(convertUnits(translations[i].getY()), this.dimension);
        }

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.drawPolyline(x, y, translations.length);
        });
        this.update();
    }

    public void drawConnectedPoints(Color color, List<? extends Translation2d> translations) {
        this.drawConnectedPoints(color, translations.toArray(new Translation2d[0]));
    }

    public void fillPolygon(Color color, Translation2d... translations) {
        int[] x = new int[translations.length];
        int[] y = new int[translations.length];
        for (int i = 0; i < translations.length; i++) {
            x[i] = convertX(convertUnits(translations[i].getX()), this.dimension);
            y[i] = convertY(convertUnits(translations[i].getY()), this.dimension);
        }

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.fillPolygon(x, y, translations.length);
        });
        this.update();
    }

    public void drawRect(double x, double y, int width, int height, Color color) {
        double X = convertUnits(x);
        double Y = convertUnits(y);

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.drawRect(convertXWithSize(X, width, dimension), convertYWithSize(Y, height, dimension), convertWidth(width), convertHeight(height));
        });
    }

    public void drawRect(Translation2d translation1, Translation2d translation2, Color color) {
        Translation2d newTranslation1 = new Translation2d(convertUnits(translation1.getX()), convertUnits(translation1.getY()));
        Translation2d newTranslation2 = new Translation2d(convertUnits(translation2.getX()), convertUnits(translation2.getY()));

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.drawRect(convertX(newTranslation1.getX(), this.dimension), convertY(newTranslation1.getY(), this.dimension),
                    convertWidth(newTranslation2.getX() - newTranslation1.getX()), convertHeight(newTranslation2.getY() - newTranslation1.getY()));
        });
        this.update();
    }

    public void fillRect(double x, double y, double width, double height, Color color) {
        double X = convertUnits(x);
        double Y = convertUnits(y);
        double newWidth = convertUnits(width);
        double newHeight = convertUnits(height);

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.fillRect(convertXWithSize(X, newWidth, dimension), convertYWithSize(Y, newHeight, dimension), convertWidth(newWidth), convertHeight(newHeight));
        });
        this.update();
    }

    public void fillRect(Translation2d translation1, Translation2d translation2, Color color) {
        Translation2d newTranslation1 = new Translation2d(convertUnits(translation1.getX()), convertUnits(translation1.getY()));
        Translation2d newTranslation2 = new Translation2d(convertUnits(translation2.getX()), convertUnits(translation2.getY()));

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.fillRect(convertX(newTranslation1.getX(), this.dimension), convertY(newTranslation1.getY(), this.dimension),
                    convertWidth(newTranslation2.getX() - newTranslation1.getX()), convertHeight(newTranslation2.getY() - newTranslation1.getY()));
        });
        this.update();
    }

    public void drawLine(Translation2d translation1, Translation2d translation2, double width, Color color) {
        width /= Math.sqrt(2);
        double max = translation1.getDistance(translation2) - width;
        for (double i = 0; i <= max; i += convertPixelsToUnits(1)) {
            this.fillPoint(translation1.getX() + ((translation2.getX() - translation1.getX()) * (i / max)),
                    translation1.getY() + ((translation2.getY() - translation1.getY()) * (i / max)),
                    width / 2,
                    color);
        }
        this.update();
    }

    public void drawLine(double x1, double y1, double x2, double y2, double width, Color color) {
        this.drawLine(new Translation2d(x1, y1), new Translation2d(x2, y2), width, color);
    }

    public void drawThinLine(Translation2d translation1, Translation2d translation2, Color color) {
        Translation2d newTranslation1 = new Translation2d(convertUnits(translation1.getX()), convertUnits(translation1.getY()));
        Translation2d newTranslation2 = new Translation2d(convertUnits(translation2.getX()), convertUnits(translation2.getY()));

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.drawLine(convertX(newTranslation1.getX(), dimension), convertY(newTranslation1.getY(), dimension),
                    convertX(newTranslation2.getX(), dimension), convertY(newTranslation2.getY(), dimension));
        });
        this.update();
    }

    public void drawThinLine(double x1, double y1, double x2, double y2, Color color) {
        this.drawThinLine(new Translation2d(x1, y1), new Translation2d(x2, y2), color);
    }

    public void drawPoint(double x, double y, double radius, Color color) {
        double X = convertUnits(x);
        double Y = convertUnits(y);
        double diameter = convertUnits(radius) * 2;

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.drawRoundRect(convertXWithSize(X, convertWidth(diameter), this.dimension),
                    convertYWithSize(Y, convertHeight(diameter), this.dimension),
                    (int) diameter,
                    (int) diameter,
                    (int) diameter,
                    (int) diameter);
        });
        this.update();
    }

    public void fillPoint(double x, double y, double radius, Color color) {
        double X = convertUnits(x);
        double Y = convertUnits(y);
        double diameter = convertUnits(radius) * 2;

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.fillRoundRect(convertXWithSize(X, convertWidth(diameter), this.dimension),
                    convertYWithSize(Y, convertHeight(diameter), this.dimension),
                    (int) (diameter),
                    (int) (diameter),
                    (int) (diameter),
                    (int) (diameter));
        });
        this.update();
    }

    public void write(double x, double y, String text, double size, Color color) {
        double X = convertUnits(x);
        double Y = convertUnits(y);
        double newSize = convertUnits(size);

        this.panel.graphics.add(g -> {
            g.setColor(color);
            g.setFont(new Font("ariel", Font.PLAIN, (int) newSize));
            g.drawString(text, convertX(X, this.dimension), convertY(Y, this.dimension));
        });
        this.update();
    }

    public void drawString(double x, double y, String text, double size, Color color) {
        this.write(x, y, text, size, color);
        this.update();
    }

    public void drawImage(Image image, double x, double y, double width, double height, double angle) {
        this.panel.graphics.add(g -> {
                ((Graphics2D) g).rotate(Math.toRadians(angle), convertX(convertUnits(x), dimension), convertY(convertUnits(y), dimension));
                g.drawImage(image,
                        convertX(convertUnits(x - (width / 2)), dimension),
                        convertY(convertUnits(y + (height / 2)), dimension),
                        (int) convertUnits(width), (int) convertUnits(height), (img, infoFlags, x1, y1, w, h) -> true);
                ((Graphics2D) g).rotate(Math.toRadians(-angle), convertX(convertUnits(x), dimension), convertY(convertUnits(y), dimension));
        });
    }

    public void drawImage(Image image, int x, int y, int width, int height) {
        this.panel.graphics.add(g -> {
            g.drawImage(image, x, y, width, height, (img, infoFlags, x1, y1, w, h) -> true);
        });
    }

    protected Translation2d getMouseTranslation(MouseEvent e) {
        return new Translation2d(convertPixelsToX(e.getX(), this.dimension), convertPixelsToY(e.getY(), this.dimension));
    }

    public void mousePressed(MouseEvent e) {}
    public void mouseReleased(MouseEvent e) {}
    public void mouseEntered(MouseEvent e) {}
    public void mouseExited(MouseEvent e) {}

    public void mouseDragged(MouseEvent e) {}
    public void mouseMoved(MouseEvent e) {}

    public void keyTyped(KeyEvent e) {}
    public void keyPressed(KeyEvent e) {}
    public void keyReleased(KeyEvent e) {}

    public void mouseWheelMoved(MouseWheelEvent e) {}

    private class MouseHandler implements MouseListener {
        @Override
        public void mouseClicked(MouseEvent e) {
        }

        @Override
        public void mousePressed(MouseEvent e) {
            Frame.this.mousePressed(e);
        }

        @Override
        public void mouseReleased(MouseEvent e) {
            Frame.this.mouseReleased(e);
        }

        @Override
        public void mouseEntered(MouseEvent e) {
            Frame.this.mouseEntered(e);
        }

        @Override
        public void mouseExited(MouseEvent e) {
            Frame.this.mouseExited(e);
        }
    }

    private class MouseMotionHandler implements MouseMotionListener {
        @Override
        public void mouseDragged(MouseEvent e) {
            Frame.this.mouseDragged(e);
        }

        @Override
        public void mouseMoved(MouseEvent e) {
            Frame.this.mouseMoved(e);
        }
    }

    private class MouseWheelHandler implements MouseWheelListener {
        @Override
        public void mouseWheelMoved(MouseWheelEvent e) {
            Frame.this.mouseWheelMoved(e);
        }
    }

    private class KeyHandler implements KeyListener {
        @Override
        public void keyTyped(KeyEvent e) {
            Frame.this.keyTyped(e);
        }

        @Override
        public void keyPressed(KeyEvent e) {
            Frame.this.keyPressed(e);
        }

        @Override
        public void keyReleased(KeyEvent e) {
            Frame.this.keyReleased(e);
        }
    }

    private static class Panel extends JPanel {
        private final List<Consumer<Graphics>> graphics;

        public Panel() {
            this.graphics = new ArrayList<>();
        }

        @Override
        protected void paintComponent(Graphics g) {
            try {
                this.graphics.forEach(c -> c.accept(g));
            } catch (ConcurrentModificationException ignored) {}
        }
    }
}
