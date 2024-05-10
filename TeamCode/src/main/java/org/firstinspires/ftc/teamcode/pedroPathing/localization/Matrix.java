package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import java.util.Arrays;

/**
 * This is the Matrix class. This defines matrices, primarily for use in the localizers. However, if
 * matrices and matrix operations are necessary, this class as well as some operations in the
 * MathFunctions class can absolutely be used there as well. It's similar to Mats in OpenCV if you've
 * used them before, but with more limited functionality.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public class Matrix {
    private double[][] matrix;

    public Matrix() {
        matrix = new double[0][0];
    }

    public Matrix(int rows, int columns) {
        matrix = new double[rows][columns];
    }

    public Matrix(double[][] setMatrix) {
        setMatrix(setMatrix);
    }

    public Matrix(Matrix setMatrix) {
        setMatrix(setMatrix);
    }

    public static double[][] deepCopy(double[][] copyMatrix) {
        double[][] returnMatrix = new double[copyMatrix.length][copyMatrix[0].length];
        for (int i = 0; i < copyMatrix.length; i++) {
            returnMatrix[i] = Arrays.copyOf(copyMatrix[i], copyMatrix[i].length);
        }
        return returnMatrix;
    }

    public double[][] getMatrix() {
        return deepCopy(matrix);
    }

    public double[] get(int row) {
        return Arrays.copyOf(matrix[row], matrix[row].length);
    }

    public double get(int row, int column) {
        return get(row)[column];
    }

    public int getRows() {
        return matrix.length;
    }

    public int getColumns() {
        return matrix[0].length;
    }

    public boolean setMatrix(Matrix setMatrix) {
        return setMatrix(setMatrix.getMatrix());
    }

    public boolean setMatrix(double[][] setMatrix) {
        int columns = setMatrix[0].length;
        for (int i = 0; i < setMatrix.length; i++) {
            if (setMatrix[i].length != columns) {
                return false;
            }
        }
        matrix = deepCopy(setMatrix);
        return true;
    }

    public boolean set(int row, double[] input) {
        if (input.length != getColumns()) {
            return false;
        }
        matrix[row] = Arrays.copyOf(input, input.length);
        return true;
    }

    public boolean set(int row, int column, double input) {
        matrix[row][column] = input;
        return true;
    }

    public boolean add(Matrix input) {
        if (input.getRows() == getRows() && input.getColumns() == getColumns()) {
            for (int i = 0; i < getRows(); i++) {
                for (int j = 0; j < getColumns(); j++) {
                    set(i, j, get(i,j) + input.get(i,j));
                }
            }
            return true;
        }
        return false;
    }

    public boolean subtract(Matrix input) {
        if (input.getRows() == getRows() && input.getColumns() == getColumns()) {
            for (int i = 0; i < getRows(); i++) {
                for (int j = 0; j < getColumns(); j++) {
                    set(i, j, get(i,j) - input.get(i,j));
                }
            }
            return true;
        }
        return false;
    }

    public boolean scalarMultiply(double scalar) {
        for (int i = 0; i < getRows(); i++) {
            for (int j = 0; j < getColumns(); j++) {
                set(i, j, scalar * get(i,j));
            }
        }
        return true;
    }

    public boolean flipSigns() {
        return scalarMultiply(-1);
    }

    public boolean multiply(Matrix input) {
        if (getColumns() == input.getRows()) {
            Matrix product = new Matrix(getRows(), input.getColumns());
            for (int i = 0; i < product.getRows(); i++) {
                for (int j = 0; j < product.getColumns(); j++) {
                    double value = 0;
                    for (int k = 0; k < get(i).length; k++) {
                        value += get(i, k) * input.get(k, j);
                    }
                    product.set(i, j, value);
                }
            }
            setMatrix(product);
            return true;
        }
        return false;
    }

    public static Matrix multiply(Matrix one, Matrix two) {
        Matrix returnMatrix = new Matrix(one);
        if (returnMatrix.multiply(two)) {
            return returnMatrix;
        } else {
            return new Matrix();
        }
    }
}
