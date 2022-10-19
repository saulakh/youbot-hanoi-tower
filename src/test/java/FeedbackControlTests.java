import matrix.Matrix;
import org.junit.Assert;
import org.junit.Test;

public class FeedbackControlTests {

    YouBot robot = new YouBot();
    FeedbackControl feedback = new FeedbackControl();
    double radius = robot.WHEEL_RADIUS;
    double length = robot.FORWARD_BACKWARD_LENGTH;
    double width = robot.SIDE_TO_SIDE_WIDTH;

    double[] testConfig = new double[] {0,0,0,0,0,0.2,-1.6,0,0,0,0,0,0};
    double[] thetaList = Matrix.rangeFromArray(testConfig, 3, 8);
    double[][] F = Matrix.scalarMultiplication(new double[][] {{-1/(length+width),1/(length+width),1/(length+width),-1/(length+width)},{1,1,1,1}, {-1,1,-1,1}}, radius/4);
    double[][] T0e = new double[][] {{0.16996714, 0, 0.98544973, 0.2206135},{0,1,0,0},{-0.98544973, 0, 0.16996714, 0.47129384},{0,0,0,1}};

    @Test
    public void checkJacobian() {
        double[][] expected = new double[][] {{-0.98544973, 0, 0, 0, 0, 0.03039537, -0.03039537, -0.03039537, 0.03039537},
                {0, -1, -1, -1, 0, 0, 0, 0, 0},
                {0.16996714, 0, 0, 0, 1, -0.00524249, 0.00524249, 0.00524249, -0.00524249},
                {0, -0.24000297, -0.21365806, -0.2176, 0, 0.00201836, 0.00201836, 0.00201836, 0.00201836},
                {0.2206135, 0, 0, 0, 0, -0.01867964, 0.01867964, -0.00507036, 0.00507036},
                {0, -0.28768714, -0.13494244, 0, 0, 0.01170222, 0.01170222, 0.01170222, 0.01170222}};
        double[][] actual = feedback.jacobian(T0e, F, robot.BList, thetaList);
        checkMatrixWithDelta(expected, actual, 0.001);
    }

    @Test
    public void checkFeedbackControl() {
        double[] robotConfig = {0,0,0,0,0,0.2,-1.6,0,0,0,0,0,0};
        double[] errorIntegral = {0,0,0,0,0,0};
        double dT = 0.01;

        // Given Xd, Xd_next, X, Kp, Ki:
        double[][] Xd = {{0,0,1,0.5},{0,1,0,0},{-1,0,0,0.5},{0,0,0,1}};
        double[][] XdNext = {{0,0,1,0.6},{0,1,0,0},{-1,0,0,0.3},{0,0,0,1}};
        double[][] X = {{0.17,0,0.985,0.387},{0,1,0,0},{-0.985,0,0.17,0.57},{0,0,0,1}};
        double[][] Kp = new double[6][6];
        double[][] Ki = new double[6][6];

        double[] expected = {0, -653.192, 1399.261, -746.069, 0, 157.244, 157.244, 157.244, 157.244};
        double[] actual = feedback.feedbackControl(X, Xd, XdNext, Kp, Ki, dT, robotConfig, errorIntegral);
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
