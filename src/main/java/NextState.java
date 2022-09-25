import matrix.Matrix;

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
         - dTheta: change in wheel angles
         Output:
         - chassisConfig: new chassis state (phi,x,y)
         */
        return chassisConfig;
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
        return currentConfig;
    }

}
