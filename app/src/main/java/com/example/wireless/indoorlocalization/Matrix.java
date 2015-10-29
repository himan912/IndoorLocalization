package com.example.wireless.indoorlocalization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * Created by Hyunmin Cho on 2015-10-21.
 */
public class Matrix {
    public static double[][] transposeMatrix(double[][] matrix) {
        double[][] ret_matrix = new double[matrix.length][matrix[0].length];

        for (int i = 0; i < matrix.length; i++) {
            for (int j = 0; j < matrix[i].length; j++) {
                ret_matrix[j][i] = matrix[i][j];
            }
        }

        return ret_matrix;
    }

    public static boolean isSquare(double[][] matrix) {
        if (matrix.length != matrix[0].length) {
            return false;
        }
        return true;
    }

    public static double[][] getInverseMatrix(double[][] matrix) {
        if (!isSquare(matrix)) return null;

        RealMatrix a = new Array2DRowRealMatrix(matrix);
        RealMatrix inv_a = new Array2DRowRealMatrix(a.getColumnDimension(), a.getRowDimension());
        double[] vector_i = new double[matrix.length];

        DecompositionSolver solver = new LUDecomposition(a).getSolver();

        for (int n = 0; n < matrix.length; n++) {
            for (int k = 0; k < vector_i.length; k++) {
                vector_i[k] = 0;
            }
            vector_i[n] = 1;

            RealVector x = solver.solve(new ArrayRealVector(vector_i));
            inv_a.setColumn(n, x.toArray());
        }

        return inv_a.getData();
    }

    public static double[][] matrixMultiplication(double[][] matrix1, double[][] matrix2) {
        if (matrix1[0].length != matrix2.length) {
            return null;
        }
        double[][] ret_matrix = new double[matrix1.length][matrix2[0].length];

        for (int i = 0; i < matrix1.length; i++) {
            for (int j = 0; j < matrix2[0].length; j++) {
                for (int k = 0; k < matrix2.length; k++) {
                    ret_matrix[i][j] += matrix1[i][k] * matrix2[k][j];
                }
            }
        }
        return ret_matrix;
    }

    public static int[][] getSubsets(int n) {
        int[][] subsets = new int[n][n - 1];

        subsets[0][n - 2] = n - 1;
        subsets[n - 1][0] = 2;

        for (int i = 1; i <= n - 1; i++) {
            for (int j = 1; j <= n - i; j++) {
                subsets[j - 1][i - 1] = i;
                subsets[n - j][n - 1 - i] = n - (i - 1);
            }
        }
        return subsets;
    }
}
