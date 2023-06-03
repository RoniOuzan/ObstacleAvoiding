package obstacleavoiding.gui;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Graph extends JPanel {

    private static final int padding = 25;
    private static final int labelPadding = 25;
    private static final Color lineColor = new Color(44, 102, 230, 180);
    private static final Color pointColor = new Color(100, 100, 100, 180);
    private static final Color gridColor = new Color(200, 200, 200, 200);
    private static final Stroke GRAPH_STROKE = new BasicStroke(2f);
    private static final int pointWidth = 4;
    private static final int numberYDivisions = 10;
    private final Supplier<List<Double>> scores;

    private final double max;
    private final double min;

    public Graph(Supplier<List<Double>> scores, double max, double min) {
        this.scores = scores;
        this.max = max;
        this.min = min;

        this.setPreferredSize(new Dimension(800, 800));
        this.setLocation(0, 0);
        this.setVisible(true);
    }

    public Graph(Supplier<List<Double>> scores) {
        this(scores, 0, 0);
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        List<Double> scores = this.scores.get().stream().map(s -> {
            if (this.isAutoLimit())
                return s;
            if (s >= this.min)
                return s <= this.max ? s : this.max;
            return this.min;
        }).toList();

        double xScale = ((double) getWidth() - (2 * padding) - labelPadding) / (scores.size() - 1);
        double yScale = ((double) getHeight() - 2 * padding - labelPadding) / (getMaxScore() - getMinScore());

        List<Point> graphPoints = new ArrayList<>();
        for (int i = 0; i < scores.size(); i++) {
            int x1 = (int) (i * xScale + padding + labelPadding);
            int y1 = (int) ((getMaxScore() - scores.get(i)) * yScale + padding);
            graphPoints.add(new Point(x1, y1));
        }

        // draw white background
        g2.setColor(Color.WHITE);
        g2.fillRect(padding + labelPadding, padding, getWidth() - (2 * padding) - labelPadding, getHeight() - 2 * padding - labelPadding);
        g2.setColor(Color.BLACK);

        // create hatch marks and grid lines for y-axis.
        for (int i = 0; i < numberYDivisions + 1; i++) {
            int x0 = padding + labelPadding;
            int x1 = pointWidth + padding + labelPadding;
            int y0 = getHeight() - ((i * (getHeight() - padding * 2 - labelPadding)) / numberYDivisions + padding + labelPadding);
            if (scores.size() > 0) {
                g2.setColor(gridColor);
                g2.drawLine(padding + labelPadding + 1 + pointWidth, y0, getWidth() - padding, y0);
                g2.setColor(Color.BLACK);
                String yLabel = ((int) ((getMinScore() + (getMaxScore() - getMinScore()) * ((i * 1.0) / numberYDivisions)) * 10000)) / 10000.0 + "";
                FontMetrics metrics = g2.getFontMetrics();
                int labelWidth = metrics.stringWidth(yLabel);
                g2.drawString(yLabel, x0 - labelWidth - 5, y0 + (metrics.getHeight() / 2) - 3);
            }
            g2.drawLine(x0, y0, x1, y0);
        }

        // and for x-axis
        for (int i = 0; i < scores.size(); i++) {
            if (scores.size() > 1) {
                int x0 = i * (getWidth() - padding * 2 - labelPadding) / (scores.size() - 1) + padding + labelPadding;
                int y0 = getHeight() - padding - labelPadding;
                int y1 = y0 - pointWidth;
                if ((i % ((int) ((scores.size() / 20.0)) + 1)) == 0) {
                    g2.setColor(gridColor);
                    g2.drawLine(x0, getHeight() - padding - labelPadding - 1 - pointWidth, x0, padding);
                    g2.setColor(Color.BLACK);
                    String xLabel = i + "";
                    FontMetrics metrics = g2.getFontMetrics();
                    int labelWidth = metrics.stringWidth(xLabel);
                    g2.drawString(xLabel, x0 - labelWidth / 2, y0 + metrics.getHeight() + 3);
                }
                g2.drawLine(x0, y0, x0, y1);
            }
        }

        // create x and y axes
        g2.drawLine(padding + labelPadding, getHeight() - padding - labelPadding, padding + labelPadding, padding);
        g2.drawLine(padding + labelPadding, getHeight() - padding - labelPadding, getWidth() - padding, getHeight() - padding - labelPadding);

        Stroke oldStroke = g2.getStroke();
        g2.setColor(lineColor);
        g2.setStroke(GRAPH_STROKE);
        for (int i = 0; i < graphPoints.size() - 1; i++) {
            int x1 = graphPoints.get(i).x;
            int y1 = graphPoints.get(i).y;
            int x2 = graphPoints.get(i + 1).x;
            int y2 = graphPoints.get(i + 1).y;
            g2.drawLine(x1, y1, x2, y2);
        }

        g2.setStroke(oldStroke);
        g2.setColor(pointColor);
        for (Point graphPoint : graphPoints) {
            int x = graphPoint.x - pointWidth / 2;
            int y = graphPoint.y - pointWidth / 2;
            g2.fillOval(x, y, pointWidth, pointWidth);
        }
    }

    public boolean isAutoLimit() {
        return (this.min == 0 && this.max == 0) || (this.min >= this.max);
    }

    private double getMinScore() {
        if (this.isAutoLimit())
            return scores.get().stream().mapToDouble(s -> s).min().orElse(0);
        return this.min;
    }

    private double getMaxScore() {
        if (this.isAutoLimit()) {
            double maxScore = Double.MIN_VALUE;
            for (Double score : scores.get()) {
                maxScore = Math.max(maxScore, score);
            }
            return maxScore;
        }
        return this.max;
    }

    public List<Double> getScores() {
        return scores.get();
    }

    public static JFrame createAndShowGui(String title, Supplier<List<Double>> scores, double max, double min) {
        Graph mainPanel = new Graph(scores, max, min);
        mainPanel.setPreferredSize(new Dimension(800, 600));
        mainPanel.setLocation(0, 0);
        JFrame frame = new JFrame(title);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().add(mainPanel);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);

        return frame;
    }

    public static JFrame createAndShowGui(String title, Supplier<List<Double>> scores) {
        return createAndShowGui(title, scores, 0, 0);
    }
}
