import matrix.Matrix;

import java.util.Arrays;

public class Main {

    public static void main(String[] args) {

        YouBot youBot = new YouBot();
        NextState nextState;

        // Testing nextState method
        double[] controls = new double[] {0.5,0.5,0.5,0.5,0.5,10,10,10,10};
        double[] nextConfig = NextState.nextState(youBot.currentConfig, controls, youBot.DELTA_T, youBot.MAX_SPEED);
        System.out.println("\nCurrent Config: " + Arrays.toString(youBot.currentConfig));
        System.out.println("\nNext Config: " + Arrays.toString(nextConfig));

    }

}
