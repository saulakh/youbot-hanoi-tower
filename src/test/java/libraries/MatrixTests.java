package libraries;

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
    public void checkRangeFromMatrix() {
        double[][] inputMatrix = new double[][] {{1,0,0,0},{0,0,-1,0},{0,1,0,3},{0,0,0,1}};
        double[][] expected = new double[][] {{1,0,0},{0,0,-1},{0,1,0}};
        double[][] actual = Matrix.rangeFromMatrix(inputMatrix, 0, 3, 0, 3);
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
    public void checkReshapeArray() {
        double[] input = {5,2,6,1,0,6,2,0,3,8,1,4,1,8,5,6};
        double[][] expected = new double[][] {{5,2,6,1},{0,6,2,0},{3,8,1,4},{1,8,5,6}};
        double[][] actual = Matrix.reshapeArray(input, 4, 4);
        Assert.assertArrayEquals(expected, actual);
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

    @Test
    public void checkPseudoInverse() {
        double[][] testMatrix = new double[][] {{0.1150, -1.1121, 0.2334, -0.2321},{1.2753, 1.0699, 0.2335, 1.0177},{0.3942, -1.0498, -0.0486, 0.3240}};
        double[][] expected = new double[][] {{0.7864, 0.5613, -0.2359},{-0.1709, 0.1768, -0.5890},{1.8576, 0.5363, -1.4498},{-1.2319, -0.0296, 1.2474}};
        double[][] actual = Matrix.pseudoInverse(testMatrix);
        checkMatrixWithDelta(expected, actual, 0.0001);
    }

    @Test
    public void checkJacobianPseudoInverse() {
        double[][] jacobian = new double[][] {{-0.98544973, 0, 0, 0, 0, 0.03039537, -0.03039537, -0.03039537, 0.03039537},
                {0,-1,-1,-1,0,0,0,0,0},
                {0.16996714,0,0,0,1,-0.00524249,0.00524249,0.00524249,-0.00524249},
                {0,-0.24000297,-0.21365806,-0.2176,0,0.00201836,0.00201836,0.00201836,0.00201836},
                {0.2206135,0,0,0,0,-0.01867964,0.01867964,-0.00507036,0.00507036},
                {0,-0.28768714,-0.13494244,0,0,0.01170222,0.01170222,0.01170222,0.01170222}};
        double[][] expected = new double[][] {{-1.01091819, 5.77279635E-15, 4.68102367E-10, -2.73082033E-14, -3.73964239E-07, 2.19715853E-15},
                {-3.40563123E-15, 6.56726842, 2.58128946E-15, -30.1804615, -1.57806135E-14, -1.04781488},
                {6.80941384E-15, -14.5624557, -5.80639387E-15, 66.9230499, 3.24687338E-14, -5.28942736},
                {-3.62417451E-15, 6.99518724, 2.92660210E-15, -36.7425884, -1.71460946E-14, 6.33724224},
                {0.172476722, -1.05375512E-15, 1, 4.66136683E-15, 4.61063717E-14, 2.16306429E-16},
                {-4.68189096, -1.61880885, 3.79409515E-09, 7.43937878, -21.0526346, -0.324961869},
                {4.68189096, -1.61880885, -3.79409644E-09, 7.43937878, 21.0526346, -0.324961869},
                {-4.74425144, -1.61880885, -3.79408687E-09, 7.43937878, -21.0526285, -0.324961869},
                {4.74425144, -1.61880885, 3.79408557E-09, 7.43937878, 21.0526285, -0.324961869}};
        double[][] actual = Matrix.pseudoInverse(jacobian);
        checkMatrixWithDelta(expected, actual, 0.0001);
    }

    public void checkMatrixWithDelta(double[][] expected, double[][] actual, double delta) {
        for (int i=0; i < actual.length; i++) {
            for (int j=0; j < actual[0].length; j++) {
                Assert.assertEquals(expected[i][j], actual[i][j], delta);
            }
        }
    }
}
