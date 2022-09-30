import org.junit.Assert;
import org.junit.Test;

import java.util.Arrays;

public class NextStateTest {

    NextState nextState;

    @Test
    public void checkConfig() {
        // Testing nextState method
        double[] expectedConfig = new double[] {0,0.00475,-4.34E-19,0.005,0.005,0.005,0.005,0.005,0.1,0.1,0.1,0.1,0};
        double[] initialConfig = new double[13];
        double[] controls = new double[] {0.5,0.5,0.5,0.5,0.5,10,10,10,10};
        double[] nextConfig = NextState.nextState(initialConfig, controls, 0.01, 10);
        Assert.assertArrayEquals(expectedConfig, nextConfig, 0.0001);
    }
}