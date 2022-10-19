import matrix.Matrix;

import java.util.Arrays;

public class NextState {

    public static double[] eulerStep(double[] angles, double[] controls, double dT) {
        /*
         Inputs:
         - angles: includes 5 joint arm angles, and 4 wheel angles from currentConfig
         - controls: includes 5 joint speeds and 4 wheel speeds
         - dT: timestep Δt
         Output:
         - angles: updates to 5 new arm joint angles and 4 new wheel angles
         */
        for (int i=0; i < angles.length; i++) {
            angles[i] += controls[i] * dT;
        }
        return angles;
    }

    public static double[] odometry(double[] chassisConfig, double[] dTheta) {
        /*
         Inputs:
         - chassisConfig: current chassis state (phi,x,y)
         - dTheta: change in wheel angles (wheel1, wheel2, wheel3, wheel4)
         Output:
         - chassisConfig: new chassis state (phi,x,y)
         */
        YouBot youBot = new YouBot();

        // Body Twist
        double[][] Vb = Matrix.matrixMultiplication(youBot.F, Matrix.transposeArray(dTheta));
        assert Vb != null;
        double omegaBZ = Vb[0][0];
        double velBX = Vb[1][0];
        double velBY = Vb[2][0];
        double[][] deltaQb;

        // Set up deltaQb Matrix (Δ omegaBZ, Δ velBX, Δ velBY)
        if (omegaBZ < 0.0001) {
            deltaQb = Vb;
        } else {
            double deltaXb = (velBX * Math.sin(omegaBZ) + velBY * (Math.cos(omegaBZ) - 1)) / omegaBZ;
            double deltaYb = (velBY * Math.sin(omegaBZ) + velBX * (1 - Math.cos(omegaBZ))) / omegaBZ;
            deltaQb = new double[][] {{omegaBZ}, {deltaXb}, {deltaYb}};
        }

        // Set up deltaQ Matrix
        double phi = chassisConfig[0];
        double[][] deltaQ = Matrix.matrixMultiplication(new double[][] {{1,0,0},{0,Math.cos(phi),-Math.sin(phi)},{0,Math.sin(phi),Math.cos(phi)}}, deltaQb);

        // New Chassis state
        assert deltaQ != null;
        chassisConfig[0] += deltaQ[0][0];
        chassisConfig[1] += deltaQ[1][0];
        chassisConfig[2] += deltaQ[2][0];

        return chassisConfig;
    }

    public static void limitSpeeds(double[] controls, double maxSpeed) {
        for (int i=0; i < controls.length; i++) {
            if (controls[i] > maxSpeed) {
                controls[i] = maxSpeed;
            } else if (controls[i] < -maxSpeed) {
                controls[i] = -maxSpeed;
            }
        }
    }

    public static double[] nextState(double[] currentConfig, double[] controls, double dT, double maxSpeed) {
        /*
         Inputs:
         - currentConfig: 12-vector for current robot configuration (phi,x,y for chassis, 5 joint angles, and 4 wheel angles)
         - controls: 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
         - dT: timestep Δt
         - maxSpeed: limits the maximum angular speed of arm joints and wheels to [-max_speed rad/s, max_speed rad/s]
         Output:
         - currentConfig: 12-vector representing the configuration of the robot time Δt later
         */
        double[] newState = new double[13];
        double[] chassisConfig = Matrix.rangeFromArray(currentConfig, 0, 3);
        double[] angles = Matrix.rangeFromArray(currentConfig, 3, 12);

        limitSpeeds(controls, maxSpeed);
        double[] dTheta = Matrix.scalarArrayMultiplication(Arrays.copyOfRange(controls, 5, 9), dT);

        // Update configuration with odometry and eulerStep
        double[] newChassisConfig = odometry(chassisConfig, dTheta);
        double[] newAngles = eulerStep(angles, controls, dT);
        Matrix.replaceRangeFromArray(newChassisConfig, newState, 0);
        Matrix.replaceRangeFromArray(newAngles, newState, 3);

        return newState;
    }

}
