import model.YouBot;
import org.junit.Assert;
import org.junit.Test;
import path.NextState;

public class NextStateTests {

    YouBot robot = new YouBot();
    NextState nextState = new NextState(0.01, 10, robot.F);

    @Test
    public void checkConfig() {
        // Testing nextState method
        double[] expectedConfig = new double[] {0,0.00475,-4.34E-19,0.005,0.005,0.005,0.005,0.005,0.1,0.1,0.1,0.1,0};
        double[] initialConfig = new double[13];
        double[] controls = new double[] {0.5,0.5,0.5,0.5,0.5,10,10,10,10};
        double[] nextConfig = nextState.getNextState(initialConfig, controls);
        Assert.assertArrayEquals(expectedConfig, nextConfig, 0.0001);
    }

    @Test
    public void checkNextStateCSV() {
        double[] config = {Math.PI/6,-0.1,0.1,0,-0.2,0.2,-1.6,0,0,0,0,0,0};
        double[] controls = new double[] {0.5,0.5,0.5,0.5,0.5,10,10,10,10};

        for (int i=0; i < 101; i++) {
            config = nextState.getNextState(config, controls);
        }

        double[] expected = {0.5235987750000001, 0.3154756876091014, 0.33987499975142077, 0.5050000000000003, 0.30500000000000027, 0.7050000000000004, -1.0950000000000109, 0.5050000000000003, 10.09999999999998, 10.09999999999998, 10.09999999999998, 10.09999999999998, 0.0};
        Assert.assertArrayEquals(expected, config, 0.0001);
    }
}