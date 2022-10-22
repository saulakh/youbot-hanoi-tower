import org.junit.Assert;
import org.junit.Test;

public class NextStateTests {

    NextState nextState;

    @Test
    public void checkConfig() {
        YouBot robot =  new YouBot();
        // Testing nextState method
        double[] expectedConfig = new double[] {0,0.00475,-4.34E-19,0.005,0.005,0.005,0.005,0.005,0.1,0.1,0.1,0.1,0};
        double[] initialConfig = new double[13];
        double[] controls = new double[] {0.5,0.5,0.5,0.5,0.5,10,10,10,10};
        double[] nextConfig = NextState.nextState(initialConfig, controls, robot.F, 0.01, 10);
        Assert.assertArrayEquals(expectedConfig, nextConfig, 0.0001);
    }

    @Test
    public void checkNextStateCSV() {
        YouBot robot =  new YouBot();
        String nextStateCSVPath = "nextState.csv";
        double[] config = robot.initialConfig;
        double[] controls = new double[] {0.5,0.5,0.5,0.5,0.5,10,10,10,10};

        CSV.clearCSVFile(nextStateCSVPath);

        for (int i=0; i < 101; i++) {
            config = NextState.nextState(config, controls, robot.F, robot.DELTA_T, robot.MAX_SPEED);
            CSV.writeToCSV(nextStateCSVPath, config);
        }

        double[] expected = {0.5235987750000001, 0.3154756876091014, 0.33987499975142077, 0.5050000000000003, 0.30500000000000027, 0.7050000000000004, -1.0950000000000109, 0.5050000000000003, 10.09999999999998, 10.09999999999998, 10.09999999999998, 10.09999999999998, 0.0};
        double[] actual = CSV.readFromCSV(nextStateCSVPath);
        Assert.assertArrayEquals(expected, actual, 0.0001);
    }
}