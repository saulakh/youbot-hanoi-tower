package libraries;

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

    public static double[][] rangeFromMatrix(double[][] matrix, int rowStart, int rowEnd, int colStart, int colEnd) {
        double[][] outputMatrix = new double[rowEnd - rowStart][colEnd - colStart];
        for (int i=0; i < rowEnd - rowStart; i++) {
            if (colEnd - colStart >= 0)
                System.arraycopy(matrix[rowStart + i], colStart, outputMatrix[i], 0, colEnd - colStart);
        }
        return outputMatrix;
    }

    public static void replaceRangeFromMatrix(double[][] sourceMatrix, double[][] destMatrix, int destRowStart, int destColStart) {
        if (destRowStart + sourceMatrix.length > destMatrix.length || destColStart + sourceMatrix[0].length > destMatrix[0].length) {
            System.out.println("Out of Range Error: Cannot replace range of values.");
        } else {
            for (int row=0; row < sourceMatrix.length; row++) {
                System.arraycopy(sourceMatrix[row], 0, destMatrix[destRowStart + row], destColStart, sourceMatrix[0].length);
            }
        }
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

    public static double[][] reshapeArray(double[] array, int rows, int cols) {
        int totalValues = rows * cols;
        if (totalValues != array.length) {
            System.out.println("Error reshaping: Dimensions do not match.");
            return null;
        }
        double[][] output = new double[rows][cols];
        for (int i=0; i < rows; i++) {
            System.arraycopy(array, i * cols, output[i], 0, cols);
        }
        return output;
    }

    public static double[] columnFromMatrix(double[][] matrix, int col) {
        double[] column = new double[matrix.length];
        for (int i=0; i < matrix.length; i++) {
            column[i] = matrix[i][col];
        }
        return column;
    }

    public static void replaceColumnValues(double[][] matrix, int col, double newVal) {
        for (int i=0; i < matrix.length; i++) {
            matrix[i][col] = newVal;
        }
    }

    public static double matrixDeterminant(double[][] matrix) {
        double determinant = 0;
        int dimension = matrix.length;
        if (matrix.length != matrix[0].length) {
            System.out.println("Error: Determinant is undefined for a non-square matrix.");
            return determinant;
        }
        if (dimension == 1) {
            determinant = matrix[0][0];
        }
        else if (dimension == 2) {
            determinant = (matrix[0][0] * matrix[1][1]) - (matrix[0][1] * matrix[1][0]);
        } else {
            for (int col=0; col < dimension; col++) {
                double value = Math.pow(-1, col) * matrix[0][col];
                determinant += value * matrixDeterminant(subMatrix(matrix, 0, col));
            }
        }
        return determinant;
    }

    public static double[][] subMatrix(double[][] matrix, int row, int col) {
        /*
        M_ij is a submatrix obtained by removing row i and column j from the matrix
         */
        int dimension = matrix.length;
        double[][] submatrix = new double[dimension-1][dimension-1];
        for (int i=0; i < submatrix.length; i++) {
            for (int j=0; j < submatrix.length; j++) {
                if (i >= row & j < col) {
                    submatrix[i][j] = matrix[i+1][j];
                } else if (j >= col & i < row) {
                    submatrix[i][j] = matrix[i][j+1];
                } else if (i >= row & j >= col) {
                    submatrix[i][j] = matrix[i+1][j+1];
                } else {
                    submatrix[i][j] = matrix[i][j];
                }
            }
        }
        return submatrix;
    }

    public static double[][] adjugateMatrix(double[][] matrix) {
        /*
        The adjugate matrix is defined by A_ij = (-1) ^ (i + j) * det(M_ij)
         */
        double[][] cofactor = new double[matrix.length][matrix[0].length];
        for (int i=0; i < matrix.length; i++) {
            for (int j=0; j < matrix[0].length; j++) {
                cofactor[i][j] = Math.pow(-1, (i + j)) * matrixDeterminant(subMatrix(matrix, i, j));
            }
        }
        // Adjugate matrix is the transpose of the cofactor matrix
        return transposeMatrix(cofactor);
    }

    public static double[][] inverseMatrix(double[][] matrix) {
        return scalarMultiplication(adjugateMatrix(matrix), 1 / matrixDeterminant(matrix));
    }

    public static double[][] pseudoInverse(double[][] matrix) {
        int m = matrix.length;
        int n = matrix[0].length;
        double[][] transpose = transposeMatrix(matrix);
        if (m > n) {
            double[][] inverseAtA = inverseMatrix(matrixMultiplication(transpose, matrix));
            return matrixMultiplication(inverseAtA, transpose);
        } else if (m < n) {
            double[][] inverseAAt = inverseMatrix(matrixMultiplication(matrix, transpose));
            return matrixMultiplication(transpose, inverseAAt);
        }
        return inverseMatrix(matrix);
    }

    public static double[][] pseudoInvTol(double[][] matrix, double tolerance) {
        /*
        Sets any values greater than the tolerance as zero, and returns the pseudo inverse of the matrix
         */
        double[][] pseudoInvTol = matrix.clone();
        for (int i=0; i < matrix.length; i++) {
            for (int j=0; j < matrix[0].length; j++) {
                if (Math.abs(matrix[i][j]) < tolerance) {
                    pseudoInvTol[i][j] = 0;
                }
            }
        }
        return pseudoInverse(pseudoInvTol);
    }

    public static double[][] scalarMultiplication(double[][] matrix, double scalarValue) {
        double[][] output = new double[matrix.length][matrix[0].length];
        for (int row=0; row < matrix.length; row++) {
            for (int col=0; col < matrix[0].length; col++) {
                output[row][col] = matrix[row][col] * scalarValue;
            }
        }
        return output;
    }

    public static double[] scalarArrayMultiplication(double[] array, double scalarValue) {
        for (int i=0; i < array.length; i++) {
            array[i] *= scalarValue;
        }
        return array;
    }

    public static void replaceRangeFromArray(double[] sourceArray, double[] destArray, int destStartPos) {
        if (destStartPos + sourceArray.length > destArray.length) {
            System.out.println("Out of Range Error: Cannot replace range of values.");
        } else {
            System.arraycopy(sourceArray, 0, destArray, destStartPos, sourceArray.length);
        }
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

    public static double[] arrayAddition(double[] arrayOne, double[] arrayTwo) {
        double[] output = new double[arrayOne.length];
        for (int i=0; i < arrayOne.length; i++) {
            output[i] = arrayOne[i] + arrayTwo[i];
        }
        return output;
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

    public static void nearZero(double[][] matrix) {
        /*
        Replaces any negative zeros in the matrix with 0.0 for unit tests
         */
        for (int i=0; i < matrix.length; i++) {
            for (int j=0; j < matrix[0].length; j++) {
                if (Math.abs(matrix[i][j]) <= 0.000001) {
                    matrix[i][j] = 0.0;
                }
            }
        }
    }

}