package matrix;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class MatrixTests {

    Matrix matrix;
    double[][] graspExample;
    double theta;
    double[][] rotY;

    @Before
    public void setUp() {
        graspExample = Matrix.identityMatrix(4);
        theta = 1;
        rotY = new double[][]{{Math.cos(theta), 0, Math.sin(theta)}, {0, 1, 0}, {-Math.sin(theta), 0, Math.cos(theta)}};
    }

    @Test
    public void checkReplaceRangeFromMatrix() {
        Matrix.replaceRangeFromMatrix(rotY, graspExample, 0, 0);
        double[][] expected = new double[][] {{Math.cos(theta),0,Math.sin(theta),0},{0,1,0,0},{-Math.sin(theta),0,Math.cos(theta),0},{0,0,0,1}};
        Assert.assertArrayEquals(expected, graspExample);
    }
}
