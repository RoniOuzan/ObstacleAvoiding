package obstacleavoiding.math.matrix;

public class Matrix {
    private final double[][] matrix;

    public Matrix(int rows, int columns) {
        this.matrix = new double[rows][columns];
    }

    public Matrix(double[][] matrix) {
        this.matrix = matrix;
    }

    public int getRows() {
        return this.matrix.length;
    }

    public int getColumns() {
        return this.matrix[0].length;
    }

    public void setRow(int row, double... values) {
        if (this.getColumns() != values.length) return;

        this.matrix[row] = values;
    }

    public void setColumn(int column, double... values) {
        if (this.getRows() != values.length) return;

        for (int i = 0; i < this.getRows(); i++) {
            this.matrix[i][column] = values[i];
        }
    }

    public void set(int row, int column, double value) {
        this.matrix[row][column] = value;
    }

    public double get(int row, int column) {
        return this.matrix[row][column];
    }

    public Matrix plus(Matrix matrix) {
        if (this.getRows() != matrix.getRows() || this.getColumns() != matrix.getColumns())
            return new Matrix(this.matrix);

        Matrix result = new Matrix(this.getRows(), this.getColumns());
        for (int i = 0; i < result.getRows(); i++) {
            for (int j = 0; j < result.getColumns(); j++) {
                result.set(i, j, this.get(i, j) + matrix.get(i, j));
            }
        }
        return result;
    }

    public Matrix minus(Matrix matrix) {
        if (this.getRows() != matrix.getRows() || this.getColumns() != matrix.getColumns())
            return new Matrix(this.matrix);

        Matrix result = new Matrix(this.getRows(), this.getColumns());
        for (int i = 0; i < result.getRows(); i++) {
            for (int j = 0; j < result.getColumns(); j++) {
                result.set(i, j, this.get(i, j) - matrix.get(i, j));
            }
        }
        return result;
    }

    public Matrix times(int times) {
        Matrix result = new Matrix(this.getRows(), this.getColumns());
        for (int i = 0; i < result.getRows(); i++) {
            for (int j = 0; j < result.getColumns(); j++) {
                result.set(i, j, this.get(i, j) * times);
            }
        }
        return result;
    }

    public Matrix multiply(Matrix matrix) {
        if (this.getColumns() != matrix.getRows())
            return new Matrix(this.matrix);

        Matrix result = new Matrix(this.getRows(), matrix.getColumns());

        for (int i = 0; i < result.getRows(); i++) {
            for (int j = 0; j < result.getColumns(); j++) {
                double value = 0;

                for (int k = 0; k < matrix.getRows(); k++) {
                    value += this.get(i, k) * matrix.get(k, j);
                }

                result.set(i, j, value);
            }

        }
        return result;
    }

    @Override
    public String toString() {
        StringBuilder string = new StringBuilder();
        for (int i = 0; i < this.getRows(); i++) {
            for (int j = 0; j < this.getColumns(); j++) {
                string.append(this.get(i, j));
                if (j < this.getColumns() - 1)
                    string.append(", ");
            }
            string.append("\n");
        }
        return string.toString();
    }
}
