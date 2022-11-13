package libraries;

import java.util.Arrays;

public class Matrix {

    /**
     * Changes the row elements of a matrix into column elements and the column elements into row elements
     * @param matrix Input matrix (MxN)
     * @return NxM transpose of the input matrix
     */
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

    /**
     * Multiples two input matrices together. Number of columns in matrixOne must match number of rows in matrixTwo.
     * @param matrixOne First input matrix
     * @param matrixTwo Second input matrix
     * @return The matrix product of the inputs
     */
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

    /**
     * Adds two matrices of the same dimensions, element-wise.
     * @param matrixOne First input matrix (MxN)
     * @param matrixTwo Second input matrix (MxN)
     * @return MxN matrix of the sum of elements
     */
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

    /**
     * Returns a partial matrix with a range of values from the input matrix
     * @param matrix Input matrix
     * @param rowStart First row of values to include from input matrix (inclusive)
     * @param rowEnd Last row of values to include from input matrix (exclusive)
     * @param colStart First column of values to include from input matrix (inclusive)
     * @param colEnd Last column of values to include from input matrix (exclusive)
     * @return New matrix of specified range from input matrix
     */
    public static double[][] rangeFromMatrix(double[][] matrix, int rowStart, int rowEnd, int colStart, int colEnd) {
        double[][] outputMatrix = new double[rowEnd - rowStart][colEnd - colStart];
        for (int i=0; i < rowEnd - rowStart; i++) {
            if (colEnd - colStart >= 0)
                System.arraycopy(matrix[rowStart + i], colStart, outputMatrix[i], 0, colEnd - colStart);
        }
        return outputMatrix;
    }

    /**
     * Replaces a range of elements in a matrix with the values from another matrix. Modifies original matrix instead
     * of creating a new matrix.
     * @param sourceMatrix Matrix to copy elements from
     * @param destMatrix Matrix to copy elements into
     * @param destRowStart First row of destination matrix to copy elements into
     * @param destColStart First column of destination matrix to copy elements into
     */
    public static void replaceRangeFromMatrix(double[][] sourceMatrix, double[][] destMatrix, int destRowStart, int destColStart) {
        if (destRowStart + sourceMatrix.length > destMatrix.length || destColStart + sourceMatrix[0].length > destMatrix[0].length) {
            System.out.println("Out of Range Error: Cannot replace range of values.");
        } else {
            for (int row=0; row < sourceMatrix.length; row++) {
                System.arraycopy(sourceMatrix[row], 0, destMatrix[destRowStart + row], destColStart, sourceMatrix[0].length);
            }
        }
    }

    /**
     * Reshapes a matrix into a flattened 1-d array
     * @param fullMatrix Input matrix (MxN)
     * @return Flattened 1-d array of length M*N, with same values as original matrix rearranged in a new shape
     */
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

    /**
     * Reshapes the input array into a matrix with specified number of rows and columns
     * @param array Input array
     * @param rows Number of rows M for output matrix
     * @param cols Number of columns N for output matrix
     * @return MxN matrix with same values from the original array rearranged in a new shape
     */
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

    /**
     * Returns an array representing a column of a matrix
     * @param matrix Input matrix
     * @param col Index of column to return values from
     * @return A 1-d array of values from the specified column
     */
    public static double[] columnFromMatrix(double[][] matrix, int col) {
        double[] column = new double[matrix.length];
        for (int i=0; i < matrix.length; i++) {
            column[i] = matrix[i][col];
        }
        return column;
    }

    /**
     * Replaces each element in a column with a new value. Modifies original matrix instead of creating a new one
     * @param matrix Input matrix
     * @param col Index of column to replace
     * @param newVal Value to replace each element of the column with
     */
    public static void replaceColumnValues(double[][] matrix, int col, double newVal) {
        for (int i=0; i < matrix.length; i++) {
            matrix[i][col] = newVal;
        }
    }

    /**
     * Returns the determinant of a square matrix
     * @param matrix Input matrix
     * @return Determinant of the input matrix
     */
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

    /**
     * Returns the submatrix M_ij obtained by removing row i and column j from the matrix
     * @param matrix Input matrix
     * @param row Index i of the row to remove
     * @param col Index j of the column to remove
     * @return The corresponding submatrix
     */
    public static double[][] subMatrix(double[][] matrix, int row, int col) {
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

    /**
     * Returns the adjugate matrix A_ij = (-1) ^ (i + j) * det(M_ij), or the transpose of the cofactor matrix
     * @param matrix Input matrix
     * @return The corresponding adjugate matrix
     */
    public static double[][] adjugateMatrix(double[][] matrix) {
        double[][] cofactor = new double[matrix.length][matrix[0].length];
        for (int i=0; i < matrix.length; i++) {
            for (int j=0; j < matrix[0].length; j++) {
                cofactor[i][j] = Math.pow(-1, (i + j)) * matrixDeterminant(subMatrix(matrix, i, j));
            }
        }
        return transposeMatrix(cofactor);
    }

    /**
     * Returns the inverse of a matrix
     * @param matrix Input matrix
     * @return Inverse matrix
     */
    public static double[][] inverseMatrix(double[][] matrix) {
        return scalarMultiplication(adjugateMatrix(matrix), 1 / matrixDeterminant(matrix));
    }

    /**
     * Returns the Moore-Penrose solution for the pseudo inverse of a matrix A where A is not full rank
     * @param matrix Input matrix
     * @return Pseudo inverse of the input matrix
     */
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

    /**
     * Sets any values greater than the tolerance as zero, then returns the pseudo inverse of the matrix
     * @param matrix Input matrix
     * @param tolerance Max value of an element, to avoid unreasonably large values when taking the pseudo inverse
     * @return Pseudo inverse of input matrix, after setting values above tolerance to zero
     */
    public static double[][] pseudoInvTol(double[][] matrix, double tolerance) {
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

    /**
     * Multiplies each element of a matrix by a scalar value
     * @param matrix Input matrix
     * @param scalarValue Value to multiply each element by
     * @return New matrix of elements multiplied by scalar
     */
    public static double[][] scalarMultiplication(double[][] matrix, double scalarValue) {
        double[][] output = new double[matrix.length][matrix[0].length];
        for (int row=0; row < matrix.length; row++) {
            for (int col=0; col < matrix[0].length; col++) {
                output[row][col] = matrix[row][col] * scalarValue;
            }
        }
        return output;
    }

    /**
     * Multiples each element of an array by a scalar
     * @param array Input array
     * @param scalarValue Value to multiply each element by
     * @return New array of elements multiplied by scalar
     */
    public static double[] scalarArrayMultiplication(double[] array, double scalarValue) {
        for (int i=0; i < array.length; i++) {
            array[i] *= scalarValue;
        }
        return array;
    }

    /**
     * Replaces a range of elements in an array with the full length of another array. Modifies original array instead
     * of creating a new array
     * @param sourceArray Array to copy elements from
     * @param destArray Array to copy elements into
     * @param destStartPos First index of destination array to copy elements into
     */
    public static void replaceRangeFromArray(double[] sourceArray, double[] destArray, int destStartPos) {
        if (destStartPos + sourceArray.length > destArray.length) {
            System.out.println("Out of Range Error: Cannot replace range of values.");
        } else {
            System.arraycopy(sourceArray, 0, destArray, destStartPos, sourceArray.length);
        }
    }

    /**
     * Returns a range of elements from an array
     * @param array Input array
     * @param rangeStart First index of elements to include (inclusive)
     * @param rangeEnd Last index of elements to include (exclusive)
     * @return Selected range from input array
     */
    public static double[] rangeFromArray(double[] array, int rangeStart, int rangeEnd) {
        double[] outputArray = new double[rangeEnd - rangeStart];
        int currentIndex = 0;
        for (int i=rangeStart; i < rangeEnd; i++) {
            outputArray[currentIndex] = array[i];
            currentIndex += 1;
        }
        return outputArray;
    }

    /**
     * Represents a 1d array as a column
     * @param array 1xN input array
     * @return Nx1 representation of input array
     */
    public static double[][] transposeArray(double[] array) {
        double[][] arrayOutput = new double[array.length][1];
        for (int i=0; i < array.length; i++) {
            arrayOutput[i][0] = array[i];
        }
        return arrayOutput;
    }

    /**
     * Adds two arrays together element-wise
     * @param arrayOne The first 1xN input array
     * @param arrayTwo The second 1xN input array
     * @return 1xN array of the sum of elements in both arrays
     */
    public static double[] arrayAddition(double[] arrayOne, double[] arrayTwo) {
        if (arrayOne.length != arrayTwo.length) {
            System.out.println("Error: Dimensions do not match.");
            return null;
        }
        double[] output = new double[arrayOne.length];
        for (int i=0; i < arrayOne.length; i++) {
            output[i] = arrayOne[i] + arrayTwo[i];
        }
        return output;
    }

    /**
     * Returns an NxN identity matrix
     * @param dimensions number of rows and columns
     * @return The corresponding identity matrix
     */
    public static double[][] identityMatrix(int dimensions) {
        double[][] identityOutput = new double[dimensions][dimensions];
        for (int i=0; i < dimensions; i++) {
            identityOutput[i][i] = 1;
        }
        return identityOutput;
    }

    /**
     * Prints each row of the matrix to the console
     */
    public static void printMatrix(double[][] matrix) {
        for (double[] row : matrix) {
            System.out.println(Arrays.toString(row));
        }
    }

    /**
     * Replaces any negative zeros in the matrix with 0.0 for unit tests
     */
    public static void nearZero(double[][] matrix) {
        for (int i=0; i < matrix.length; i++) {
            for (int j=0; j < matrix[0].length; j++) {
                if (Math.abs(matrix[i][j]) <= 0.000001) {
                    matrix[i][j] = 0.0;
                }
            }
        }
    }

}
