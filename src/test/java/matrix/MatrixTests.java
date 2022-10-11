package matrix;

import org.junit.Assert;
import org.junit.Test;

public class MatrixTests {

    Matrix matrix;

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
        double[][] testMatrix = new double[][] {{1,0,0,1},{0,1,0,0},{0,0,1,0.025},{0,0,0,1}};
        double[][] expected = new double[][] {{1,0,0,-1},{0,1,0,0},{0,0,1,-0.025},{0,0,0,1}};
        double[][] actual = Matrix.inverseMatrix(testMatrix);
    }
}
