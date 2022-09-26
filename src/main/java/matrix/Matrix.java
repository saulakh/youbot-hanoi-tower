package matrix;

import java.util.Arrays;

public class Matrix {

    public static double[][] transposeMatrix(double[][] matrix) {
        int numberOfRows = matrix.length;
        int numberOfCols = matrix[0].length;
        double[][] matrixTranspose = new double[numberOfCols][numberOfRows];
        for (int i=0; i < numberOfCols; i++) {
            for (int j=0; j < numberOfRows; j++) {
                matrixTranspose[i][j] = matrix[j][i];
            }
        }
        return matrixTranspose;
    }

    public static double[][] matrixMultiplication(double[][] matrixOne, double[][] matrixTwo) {
        if (matrixOne[0].length != matrixTwo.length) {
            System.out.println("Matrix Multiplication Error: Inner dimensions do not match.");
            return null;
        }
        int innerDimension = matrixOne[0].length;
        double[][] matrixOutput = new double[matrixOne.length][matrixTwo[0].length];
        for (int row=0; row < matrixOne.length; row++) {
            for (int col=0; col < matrixTwo[0].length; col++) {
                for (int index=0; index < innerDimension; index++) {
                    matrixOutput[row][col] += matrixOne[row][index] * matrixTwo[index][col];
                }
            }
        }
        return matrixOutput;
    }

    public static double[][] matrixAddition(double[][] matrixOne, double[][] matrixTwo) {
        if (matrixOne.length != matrixTwo.length || matrixOne[0].length != matrixTwo[0].length) {
            System.out.println("Matrix Addition Error: Dimensions do not match.");
            return null;
        }
        double[][] matrixOutput = new double[matrixOne.length][matrixOne[0].length];
        for (int row=0; row < matrixOne.length; row++) {
            for (int col=0; col < matrixOne[0].length; col++) {
                matrixOutput[row][col] = matrixOne[row][col] + matrixTwo[row][col];
            }
        }
        return matrixOutput;
    }

    public static double[] flattenedMatrix(double[][] fullMatrix) {
        int totalNumberOfValues = fullMatrix.length * fullMatrix[0].length;
        double[] flattenedOutput = new double[totalNumberOfValues];
        int currentIndex = 0;
        for (double[] row : fullMatrix) {
            for (int j = 0; j < fullMatrix[0].length; j++) {
                flattenedOutput[currentIndex] = row[j];
                currentIndex += 1;
            }
        }
        return flattenedOutput;
    }

    public static double[][] scalarMultiplication(double[][] matrix, double scalarValue) {
        for (int row=0; row < matrix.length; row++) {
            for (int col=0; col < matrix[0].length; col++) {
                matrix[row][col] *= scalarValue;
            }
        }
        return matrix;
    }

    public static double[] rangeFromArray(double[] array, int rangeStart, int rangeEnd) {
        double[] outputArray = new double[rangeEnd - rangeStart];
        int currentIndex = 0;
        for (int i=rangeStart; i < rangeEnd; i++) {
            outputArray[currentIndex] = array[i];
            currentIndex += 1;
        }
        return outputArray;
    }

    public static double[][] transposeArray(double[] array) {
        double[][] arrayOutput = new double[array.length][1];
        for (int i=0; i < array.length; i++) {
            arrayOutput[i][0] = array[i];
        }
        return arrayOutput;
    }

    public static double[][] identityMatrix(int dimensions) {
        double[][] identityOutput = new double[dimensions][dimensions];
        for (int i=0; i < dimensions; i++) {
            identityOutput[i][i] = 1;
        }
        return identityOutput;
    }

    public static void printMatrix(double[][] matrix) {
        for (double[] row : matrix) {
            System.out.println(Arrays.toString(row));
        }
    }

}
