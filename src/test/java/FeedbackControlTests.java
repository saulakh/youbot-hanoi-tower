import libraries.Matrix;
import org.junit.Assert;
import org.junit.Test;

public class FeedbackControlTests {

    YouBot robot = new YouBot();
    FeedbackControl feedback = new FeedbackControl();

    double[] testConfig = new double[] {0,0,0,0,0,0.2,-1.6,0,0,0,0,0,0};
    double[] thetaList = Matrix.rangeFromArray(testConfig, 3, 8);
    double[][] T0e = new double[][] {{0.16996714, 0, 0.98544973, 0.2206135},{0,1,0,0},{-0.98544973, 0, 0.16996714, 0.47129384},{0,0,0,1}};

    @Test
    public void checkJacobian() {
        double[][] expected = new double[][] {{-0.98544973, 0, 0, 0, 0, 0.03039537, -0.03039537, -0.03039537, 0.03039537},
                {0, -1, -1, -1, 0, 0, 0, 0, 0},
                {0.16996714, 0, 0, 0, 1, -0.00524249, 0.00524249, 0.00524249, -0.00524249},
                {0, -0.24000297, -0.21365806, -0.2176, 0, 0.00201836, 0.00201836, 0.00201836, 0.00201836},
                {0.2206135, 0, 0, 0, 0, -0.01867964, 0.01867964, -0.00507036, 0.00507036},
                {0, -0.28768714, -0.13494244, 0, 0, 0.01170222, 0.01170222, 0.01170222, 0.01170222}};
        double[][] actual = feedback.jacobian(T0e, robot.F, robot.BList, thetaList);
        checkMatrixWithDelta(expected, actual, 0.001);
    }

    @Test
    public void checkFeedbackControl() {
        double[] robotConfig = {0,0,0,0,0,0.2,-1.6,0,0,0,0,0,0};

        // Given Xd, Xd_next, X, Kp, Ki:
        double[][] Xd = {{0,0,1,0.5},{0,1,0,0},{-1,0,0,0.5},{0,0,0,1}};
        double[][] XdNext = {{0,0,1,0.6},{0,1,0,0},{-1,0,0,0.3},{0,0,0,1}};
        double[][] X = {{0.17,0,0.985,0.387},{0,1,0,0},{-0.985,0,0.17,0.57},{0,0,0,1}};

        // Expected value before recalculating controls:
        // double[] expected = {0, -653.192, 1399.261, -746.069, 0, 157.244, 157.244, 157.244, 157.244};
        // Updated value after recalculating controls:
        double[] expected = {0, -15, 15, -15, 0, 15, 15, 15, 15};
        double[] actual = feedback.feedbackControl(robot, X, Xd, XdNext, robotConfig);
        Assert.assertArrayEquals(expected, actual, 0.001);
    }

    public void checkMatrixWithDelta(double[][] expected, double[][] actual, double delta) {
        for (int i=0; i < actual.length; i++) {
            for (int j=0; j < actual[0].length; j++) {
                Assert.assertEquals(expected[i][j], actual[i][j], delta);
            }
        }
    }
}
