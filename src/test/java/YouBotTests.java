import libraries.Matrix;
import libraries.Robotics;
import org.junit.Assert;
import org.junit.Test;

public class YouBotTests {

    int Tf = 2;
    double t = 0.6;

    @Test
    public void checkCubicTimeScaling() {
        double expected = 0.216;
        double actual = Robotics.cubicTimeScaling(Tf, t);
        Assert.assertEquals(expected, actual, 0.0001);
    }

    @Test
    public void checkQuinticTimeScaling() {
        double expected = 0.16308;
        double actual = Robotics.quinticTimeScaling(Tf, t);
        Assert.assertEquals(expected, actual, 0.0001);
    }

    @Test
    public void checkScrewTrajectory() {
        Tf = 5;
        int N = 4;
        int method = 3;
        double[][] xStart = new double[][] {{1,0,0,1},{0,1,0,0},{0,0,1,1},{0,0,0,1}};
        double[][] xEnd = new double[][] {{0,0,1,0.1},{1,0,0,0},{0,1,0,4.1},{0,0,0,1}};
        double[][][] expected = new double[][][] {{{1,0,0,1},{0,1,0,0},{0,0,1,1},{0,0,0,1}},
                {{0.904,-0.25,0.346,0.441},{0.346,0.904,-0.25,0.529},{-0.25,0.346,0.904,1.601},{0,0,0,1}},
                {{0.346,-0.25,0.904,-0.117},{0.904,0.346,-0.25,0.473},{-0.25,0.904,0.346,3.274},{0,0,0,1}},
                {{0,0,1,0.1},{1,0,0,0},{0,1,0,4.1},{0,0,0,1}}};
        double[][][] actual = Robotics.screwTrajectory(xStart, xEnd, Tf, N, method);
        for (int i=0; i < expected.length; i++) {
            Matrix.nearZero(actual[i]);
            checkMatrixWithDelta(expected[i], actual[i], 0.001);
        }
    }

    @Test
    public void checkAdjointMatrix() {
        double[][] inputMatrix = new double[][] {{1,0,0,0},{0,0,-1,0},{0,1,0,3},{0,0,0,1}};
        double[][] expected = new double[][] {{1,0,0,0,0,0},{0,0,-1,0,0,0},{0,1,0,0,0,0},{0,0,3,1,0,0},{3,0,0,0,0,-1},{0,0,0,0,1,0}};
        double[][] actual = Robotics.adjointMatrix(inputMatrix);
        Assert.assertArrayEquals(expected, actual);
    }

    @Test
    public void checkMatrixExp3() {
        double[][] inputMatrix = new double[][] {{0,-3,2},{3,0,-1},{-2,1,0}};
        double[][] expected = new double[][] {{-0.69492056,0.71352099,0.08929286},{-0.19200697,-0.30378504,0.93319235},{0.69297817,0.6313497,0.34810748}};
        double[][] actual = Robotics.matrixExp3(inputMatrix);
        checkMatrixWithDelta(expected, actual, 0.0001);
    }

    @Test
    public void checkMatrixLog3() {
        double[][] inputMatrix = new double[][] {{0,0,1},{1,0,0},{0,1,0}};
        double[][] expected = new double[][] {{0,-1.20919958,1.20919958},{1.20919958,0,-1.20919958},{-1.20919958,1.20919958,0}};
        double[][] actual = Robotics.matrixLog3(inputMatrix);
        checkMatrixWithDelta(expected, actual, 0.0001);
    }

    @Test
    public void checkMatrixExp6() {
        double[][] inputMatrix = new double[][] {{0,0,0,0},{0,0,-1.57079632,2.35619449},{0,1.57079632,0,2.35619449},{0,0,0,0}};
        double[][] expected = new double[][] {{1,0,0,0},{0,0,-1,0},{0,1,0,3},{0,0,0,1}};
        double[][] actual = Robotics.matrixExp6(inputMatrix);
        Matrix.nearZero(actual);
        checkMatrixWithDelta(expected, actual, 0.0001);
    }

    @Test
    public void checkMatrixLog6() {
        double[][] inputMatrix = new double[][] {{1,0,0,0},{0,0,-1,0},{0,1,0,3},{0,0,0,1}};
        double[][] expected = new double[][] {{0,0,0,0},{0,0,-1.57079632,2.35619449},{0,1.57079632,0,2.35619449},{0,0,0,0}};
        double[][] actual = Robotics.matrixLog6(inputMatrix);
        checkMatrixWithDelta(expected, actual, 0.0001);
    }

    @Test
    public void checkJacobianBody() {
        double[][] BList = Matrix.transposeMatrix(new double[][] {{0,0,1,0,0.2,0.2},{1,0,0,2,0,3},{0,1,0,0,2,1},{1,0,0,0.2,0.3,0.4}});
        double[] thetaList = new double[] {0.2,1.1,0.1,1.2};
        double[][] expected = new double[][] {{-0.04528405, 0.99500417, 0, 1},
                {0.74359313, 0.09304865, 0.36235775, 0},
                {-0.66709716, 0.03617541, -0.93203909, 0},
                {2.32586047, 1.66809, 0.56410831, 0.2},
                {-1.44321167, 2.94561275, 1.43306521, 0.3},
                {-2.06639565, 1.82881722, -1.58868628, 0.4}};
        double[][] actual = Robotics.jacobianBody(BList, thetaList);
        checkMatrixWithDelta(expected, actual, 0.0001);
    }

    @Test
    public void checkFKInBody() {
        double[][] M = new double[][] {{-1,0,0,0},{0,1,0,6},{0,0,-1,2},{0,0,0,1}};
        double[][] BList = Matrix.transposeMatrix(new double[][] {{0,0,-1,2,0,0},{0,0,0,0,1,0},{0,0,1,0,0,0.1}});
        double[] thetaList = new double[] {Math.PI / 2.0, 3, Math.PI};
        double[][] expected = new double[][] {{0,1,0,-5},{1,0,0,4},{0,0,-1,1.68584073},{0,0,0,1}};
        double[][] actual = Robotics.fkInBody(M, BList, thetaList);
        Matrix.nearZero(actual);
        checkMatrixWithDelta(expected, actual, 0.0001);
    }

    @Test
    public void checkTransToRot() {
        double[][] inputMatrix = new double[][] {{1,0,0,0},{0,0,-1,0},{0,1,0,3},{0,0,0,1}};
        double[][] expected = new double[][] {{1,0,0},{0,0,-1},{0,1,0}};
        double[][] actual = Robotics.transToRot(inputMatrix);
        Assert.assertArrayEquals(expected, actual);
    }

    @Test
    public void checkTransToPos() {
        double[][] inputMatrix = new double[][] {{1,0,0,0},{0,0,-1,0},{0,1,0,3},{0,0,0,1}};
        double[] expected = new double[] {0,0,3};
        double[] actual = Robotics.transToPos(inputMatrix);
        Assert.assertArrayEquals(expected, actual, 0.0001);
    }

    @Test
    public void checkTransInv() {
        double[][] inputMatrix = new double[][] {{1,0,0,0},{0,0,-1,0},{0,1,0,3},{0,0,0,1}};
        double[][] expected = new double[][] {{1,0,0,0},{0,0,1,-3},{0,-1,0,0},{0,0,0,1}};
        double[][] actual = Robotics.transInv(inputMatrix);
        Matrix.nearZero(actual);
        Assert.assertArrayEquals(expected, actual);
    }

    @Test
    public void checkVecToSo3() {
        double[] inputMatrix = new double[] {1,2,3};
        double[][] expected = new double[][] {{0,-3,2},{3,0,-1},{-2,1,0}};
        double[][] actual = Robotics.vecToSo3(inputMatrix);
        Assert.assertArrayEquals(expected, actual);
    }

    @Test
    public void checkSo3ToVec() {
        double[][] inputMatrix = new double[][] {{0,-3,2},{3,0,-1},{-2,1,0}};
        double[] expected = new double[] {1.0,2.0,3.0};
        double[] actual = Robotics.so3ToVec(inputMatrix);
        Assert.assertArrayEquals(expected, actual, 0.0001);
    }

    @Test
    public void checkVecToSE3() {
        double[] inputVector = new double[] {1,2,3,4,5,6};
        double[][] expected = new double[][] {{0,-3,2,4},{3,0,-1,5},{-2,1,0,6},{0,0,0,0}};
        double[][] actual = Robotics.vecToSE3(inputVector);
        Assert.assertArrayEquals(expected, actual);
    }

    @Test
    public void checkSE3ToVec() {
        double[][] inputMatrix = new double[][] {{0,-3,2,4},{3,0,-1,5},{-2,1,0,6},{0,0,0,0}};
        double[] expected = new double[] {1,2,3,4,5,6};
        double[] actual = Robotics.se3ToVec(inputMatrix);
        Assert.assertArrayEquals(expected, actual, 0.0001);
    }

    @Test
    public void checkRotToAxis3() {
        double[] inputVector = new double[] {1,2,3};
        double[] expected = {0.26726124,0.53452248,0.80178373};
        double[] actual = Robotics.rotToAxis3(inputVector);
        Assert.assertArrayEquals(expected, actual, 0.00001);
    }

    @Test
    public void checkRotToAng3() {
        double[] inputVector = new double[] {1,2,3};
        double expected = 3.7416573867739413;
        double actual = Robotics.rotToAng3(inputVector);
        Assert.assertEquals(expected, actual, 0.00001);
    }

    public void checkMatrixWithDelta(double[][] expected, double[][] actual, double delta) {
        for (int i=0; i < actual.length; i++) {
            for (int j=0; j < actual[0].length; j++) {
                Assert.assertEquals(expected[i][j], actual[i][j], delta);
            }
        }
    }
}
