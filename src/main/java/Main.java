import matrix.Matrix;

import java.util.Arrays;

public class Main {

    public static void main(String[] args) {

        YouBot youBot = new YouBot();
        NextState nextState;

        // Testing eulerStep method in NextState
        double[] config = youBot.currentConfig;
        double[] angles = Matrix.rangeFromArray(config, 3, 12);
        double[] controls = new double[] {0.5,0.5,0.5,0.5,0.5,10,10,10,10};

        System.out.println("\nEuler Step: " + Arrays.toString(NextState.eulerStep(angles, controls, youBot.DELTA_T)));

        // Testing odometry method in NextState
        double[] chassisConfig = Arrays.copyOfRange(config,0,3);
        double[] deltaT = new double[] {1,1,1,1};
        double[] updatedChassis = NextState.odometry(chassisConfig, deltaT);
        System.out.println("\nOdometry: " + Arrays.toString(updatedChassis));

    }

}
