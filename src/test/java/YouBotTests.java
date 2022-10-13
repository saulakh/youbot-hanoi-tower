import org.junit.Assert;
import org.junit.Test;

public class YouBotTests {

    YouBot robot = new YouBot();
    int Tf = 2;
    double t = 0.6;

    @Test
    public void checkCubicTimeScaling() {
        double expected = 0.216;
        double actual = robot.cubicTimeScaling(Tf, t);
        Assert.assertEquals(expected, actual, 0.0001);
    }

    @Test
    public void checkQuinticTimeScaling() {
        double expected = 0.16308;
        double actual = robot.quinticTimeScaling(Tf, t);
        Assert.assertEquals(expected, actual, 0.0001);
    }

    @Test
    public void checkScrewTrajectory() {
        Tf = 5;
        int N = 4;
        int method = 3;
        double[][] xStart = new double[][] {{1,0,0,1},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
        double[][] xEnd = new double[][] {{1,0,1,0.1},{1,0,0,0},{0,1,0,4.1},{0,0,0,1}};
        double[][][] expected = new double[][][] {{{1,0,0,1},{0,1,0,0},{0,0,1,0},{0,0,0,1}},
                {{0.904,-0.25,0.346,0.441},{0.346,0.904,-0.25,0.529},{-0.25,0.346,0.904,1.601},{0,0,0,1}},
                {{0.346,-0.25,0.904,-0.117},{0.904,0.346,-0.25,0.473},{-0.25,0.904,0.346,3.274},{0,0,0,1}},
                {{1,0,1,0.1},{1,0,0,0},{0,1,0,4.1},{0,0,0,1}}};
        double[][][] actual = robot.screwTrajectory(xStart, xEnd, Tf, N, method);
        Assert.assertArrayEquals(expected, actual);
    }
}
