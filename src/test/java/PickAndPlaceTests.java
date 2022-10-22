import libraries.Matrix;
import org.junit.Assert;
import org.junit.Test;

public class PickAndPlaceTests {

    PickAndPlace run = new PickAndPlace();

    @Test
    public void checkEndEffectorSe3() {
        double[] flattenedConfig = new double[] {0.499452656, -0.145905873, -0.602820760, -2.07024898,
                -1.54002985, -0.00706004289, -0.809104594, 0.000000987275273,
                4.43125456, -21.2031906, 4.40442226, -20.4471803, 0.00};
        double[][] expected = new double[][] {{-1.05814893E-11, 1.00, 1.05814770E-11, -8.06084081E-06},
                {0.707106781, 1.49644860E-11, -0.707106781, -0.999975229},
                {-0.707106781, 0.00, -0.707106781, 0.100001684},
                {0.00, 0.00, 0.00, 1.00}};
        double[][] actual = run.endEffectorSE3(flattenedConfig);
        Matrix.printMatrix(actual);
        Assert.assertArrayEquals(expected, actual);
    }
}
