import org.junit.Assert;
import org.junit.Test;

public class FeedbackControlTests {

    YouBot robot = new YouBot();
    FeedbackControl feedback = new FeedbackControl(robot);

    @Test
    public void checkJacobian() {
        double[][] expected = new double[][] {{-0.9995736030415051, 0.0, 0.0, 0.0, 0.0, 0.030831003989916554, -0.030831003989916554, -0.030831003989916554, 0.030831003989916554},
                {0.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                {-0.029199522301288888, 0.0, 0.0, 0.0, 1.0, 9.006346164358577E-4, -9.006346164358577E-4, -9.006346164358577E-4, 9.006346164358577E-4},
                {0.0, -0.24000297163886333, -0.213658064489326, -0.2176, 0.0, -3.467443273278053E-4, -3.467443273278053E-4, -3.467443273278053E-4, -3.467443273278053E-4},
                {0.281300962295066, 0.0, 0.0, 0.0, 0.0, -0.020551490720140023, 0.020551490720140023, -0.0031985092798599724, 0.0031985092798599724},
                {0.0, -0.28768714455881456, -0.13494243641060316, 0.0, 0.0, 0.011869936536117873, 0.011869936536117873, 0.011869936536117873, 0.011869936536117873}};
        double[][] actual = feedback.getFullJacobian();
        checkMatrixWithDelta(expected, actual, 0.001);
    }

    @Test
    public void checkFeedbackControl() {
        // Expected value before recalculating controls:
        // double[] expected = {0, -653.192, 1399.261, -746.069, 0, 157.244, 157.244, 157.244, 157.244};
        // Updated value after recalculating controls:
        double[] expected = {0, -15, 15, -15, 0, -15, -15, -15, -15};
        double[] actual = feedback.getControls();
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
