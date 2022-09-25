import matrix.Matrix;

import java.util.Arrays;

public class Main {

    public static void main(String[] args) {

        YouBot youBot = new YouBot();
        NextState nextState;

        // Testing nextState Class and eulerStep method
        double[] config = youBot.currentConfig;
        double[] angles = Matrix.rangeFromArray(config, 3, 12);
        double[] controls = new double[] {0.5,0.5,0.5,0.5,0.5,10,10,10,10};

        System.out.println(Arrays.toString(NextState.eulerStep(angles, controls, youBot.DELTA_T)));

    }

}
