package matrix;

import org.junit.Assert;
import org.junit.Test;

public class MatrixTests {

    Matrix matrix;

    @Test
    public void checkMatrixMultiplication() {
        double[][] inputOne = new double[][] {{5,2,6,1},{0,6,2,0},{3,8,1,4},{1,8,5,6}};
        double[][] inputTwo = new double[][] {{7,5,8,0},{1,8,2,6},{9,4,3,8},{5,3,7,9}};
        double[][] expected = new double[][] {{96,68,69,69},{24,56,18,52},{58,95,71,92},{90,107,81,142}};
        double[][] actual = Matrix.matrixMultiplication(inputOne, inputTwo);
        Assert.assertArrayEquals(expected, actual);
    }

    @Test
    public void checkReplaceRangeFromMatrix() {
        double[][] graspExample = Matrix.identityMatrix(4);
        double theta = 1;
        double[][] rotY = new double[][]{{Math.cos(theta), 0, Math.sin(theta)}, {0, 1, 0}, {-Math.sin(theta), 0, Math.cos(theta)}};
        Matrix.replaceRangeFromMatrix(rotY, graspExample, 0, 0);
        double[][] expected = new double[][] {{Math.cos(theta),0,Math.sin(theta),0},{0,1,0,0},{-Math.sin(theta),0,Math.cos(theta),0},{0,0,0,1}};
        Assert.assertArrayEquals(expected, graspExample);
    }

    @Test
    public void checkDeterminant3x3() {
        double expected = 23.0;
        double[][] testMatrix = new double[][] {{7,-4,2},{3,1,-5},{2,2,-5}};
        double actual = Matrix.matrixDeterminant(testMatrix);
        Assert.assertEquals(expected, actual, 0.1);
    }

    @Test
    public void checkDeterminant4x4() {
        double expected = -441;
        double[][] testMatrix = new double[][] {{0,-2,-5,8},{0,-6,3,1},{0,11,5,-3}, {-1,-2,-1,3}};
        double actual = Matrix.matrixDeterminant(testMatrix);
        Assert.assertEquals(expected, actual, 0.1);
    }

    @Test
    public void check3x3Submatrix() {
        double[][] testMatrix = new double[][] {{7,-4,2},{3,1,-5},{2,2,-5}};
        double[][] expected = new double[][] {{7,2},{2,-5}};
        double[][] actual = Matrix.subMatrix(testMatrix, 1, 1);
        Assert.assertArrayEquals(expected, actual);
    }

    @Test
    public void check4x4Inverse() {
        double[][] testMatrix = new double[][] {{1,1,1,-1},{1,1,-1,1},{1,-1,1,1},{-1,1,1,1}};
        double[][] expected = new double[][] {{0.25,0.25,0.25,-0.25},{0.25,0.25,-0.25,0.25},{0.25,-0.25,0.25,0.25},{-0.25,0.25,0.25,0.25}};
        double[][] actual = Matrix.inverseMatrix(testMatrix);
        Assert.assertArrayEquals(expected, actual);
    }
}
