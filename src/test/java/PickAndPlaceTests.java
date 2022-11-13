import model.YouBot;
import org.junit.Assert;
import org.junit.Test;

public class PickAndPlaceTests {

    YouBot robot = new YouBot();

    @Test
    public void checkEndEffectorSe3() {
        double[] flattenedConfig = new double[] {0.52359878, -0.1, 0.1, 0.0, -0.2, 0.2, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double[][] expected = new double[][] {{-0.02528753, -0.5, 0.86565613, 0.2875472},
                {-0.01459976, 0.8660254, 0.4997868, 0.32375048},
                {-0.9995736, 0.0, -0.02919952, 0.5264565},
                {0.0, 0.0, 0.0, 1.0}};
        double[][] actual = robot.endEffectorSE3(flattenedConfig);
        checkMatrixWithDelta(expected, actual, 0.001);
    }

    public void checkMatrixWithDelta(double[][] expected, double[][] actual, double delta) {
        for (int i=0; i < actual.length; i++) {
            for (int j=0; j < actual[0].length; j++) {
                Assert.assertEquals(expected[i][j], actual[i][j], delta);
            }
        }
    }
}
