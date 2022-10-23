import libraries.Matrix;

import java.util.Arrays;

public class NextState {

    private final double dT;
    private final double maxSpeed;
    private final double[][] F;

    /**
     *
     * @param dT time step Δt
     * @param maxSpeed limits the maximum angular speed of arm joints and wheels to [-maxSpeed rad/s, maxSpeed rad/s]
     * @param FMatrix Pseudo inverse of H(θ) matrix, the chassis kinematic model of the robot
     */
    public NextState(double dT, double maxSpeed, double[][] FMatrix) {
        this.dT = dT;
        this.maxSpeed = maxSpeed;
        this.F = FMatrix;
    }

    /**
     * Returns the 5 joint angles and 4 wheel angles from the current robot configuration
     * @param config 12-vector for current robot configuration (phi,x,y for chassis, 5 joint angles, and 4 wheel angles)
     * @return angles : 9-vector of 5 joint arm angles, and 4 wheel angles from the input configuration
     */
    private double[] getAngles(double[] config) {

        return Matrix.rangeFromArray(config, 3, 12);
    }

    /**
     * Returns the new joint and wheel angles one time step later
     * @param angles includes 5 joint arm angles, and 4 wheel angles from the current configuration
     * @param controls 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
     * @return angles : a 9-vector of the 5 new arm joint angles and the 4 new wheel angles
     */
    private double[] eulerStep(double[] angles, double[] controls) {

        for (int i=0; i < angles.length; i++) {
            angles[i] += controls[i] * dT;
        }
        return angles;
    }

    /**
     * Returns the chassis state from the input configuration
     * @param config 12-vector for current robot configuration (phi,x,y for chassis, 5 joint angles, and 4 wheel angles)
     * @return chassisConfig : chassis state (phi, x, y)
     */
    private double[] getChassisConfig(double[] config) {

        return Matrix.rangeFromArray(config, 0, 3);
    }

    /**
     * Returns the change in wheel angles, calculated using controls * dT
     * @param controls 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
     * @return dTheta : change in wheel angles (wheel1, wheel2, wheel3, wheel4)
     */
    private double[] getDTheta(double[] controls) {

        return Matrix.scalarArrayMultiplication(Arrays.copyOfRange(controls, 5, 9), dT);
    }

    /**
     * Returns body twist Vb of the chassis, using Vb = FΔθ
     * @param F Pseudo inverse of H(θ) matrix, the chassis kinematic model of the robot
     * @param controls 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
     * @return bodyTwist Vb : (omegaBZ, Vbx, Vby)
     */
    private double[][] getBodyTwist(double[][] F, double[] controls) {

        double[] dTheta = getDTheta(controls);
        return Matrix.matrixMultiplication(F, Matrix.transposeArray(dTheta));
    }

    /**
     * Returns the change in chassis coordinates relative to the body frame
     * @param controls 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
     * @return deltaQb : (ΔOmegaBz, ΔVbx, ΔVby)
     */
    private double[][] getDeltaQbMatrix(double[] controls) {

        double[][] Vb = getBodyTwist(F, controls);
        double omegaBZ = Vb[0][0];
        double velBX = Vb[1][0];
        double velBY = Vb[2][0];
        double[][] deltaQb;

        if (omegaBZ < 0.0001) {
            deltaQb = Vb;
        } else {
            double deltaXb = (velBX * Math.sin(omegaBZ) + velBY * (Math.cos(omegaBZ) - 1)) / omegaBZ;
            double deltaYb = (velBY * Math.sin(omegaBZ) + velBX * (1 - Math.cos(omegaBZ))) / omegaBZ;
            deltaQb = new double[][] {{omegaBZ}, {deltaXb}, {deltaYb}};
        }
        return deltaQb;
    }

    /**
     * Returns the change in chassis coordinates (phi, x, y) relative to the fixed space frame
     * @param chassisConfig current chassis state (phi,x,y)
     * @param controls 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
     * @return deltaQMatrix : Transforms ΔQb in the body frame, to ΔQ in the fixed space frame using the chassis angle phi
     */
    private double[][] getDeltaQMatrix(double[] chassisConfig, double[] controls) {

        double phi = chassisConfig[0];
        double[][] deltaQb = getDeltaQbMatrix(controls);

        return Matrix.matrixMultiplication(new double[][] {{1,0,0},{0,Math.cos(phi),-Math.sin(phi)},{0,Math.sin(phi),Math.cos(phi)}}, deltaQb);
    }

    /**
     * Updates the chassis configuration, using Q(k+1) = Qk + ΔQ
     * @param chassisConfig current chassis state (phi,x,y)
     * @param controls 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
     * @return chassisConfig the updated chassis state (phi,x,y)
     */
    private double[] odometry(double[] chassisConfig, double[] controls) {

        double[][] deltaQ = getDeltaQMatrix(chassisConfig, controls);
        chassisConfig[0] += deltaQ[0][0];
        chassisConfig[1] += deltaQ[1][0];
        chassisConfig[2] += deltaQ[2][0];

        return chassisConfig;
    }

    /**
     * Limits the maximum angular speed for arm joints and wheels, and returns the updated 9-vector of controls
     * @param controls 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
     * @param maxSpeed maximum angular speed for the arm joints and wheels
     */
    private void limitSpeeds(double[] controls, double maxSpeed) {
        /*
        Limits the maximum angular speed for arm joints and wheels, and returns the updated 9-vector of controls
         */
        for (int i=0; i < controls.length; i++) {
            if (controls[i] > maxSpeed) {
                controls[i] = maxSpeed;
            } else if (controls[i] < -maxSpeed) {
                controls[i] = -maxSpeed;
            }
        }
    }

    /**
     * Given the current configuration and controls, this method returns the configuration one time step later
     * @param currentConfig 12-vector for current robot configuration (phi,x,y for chassis, 5 joint angles, and 4 wheel angles)
     * @param controls 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
     * @return newState 12-vector representing the configuration of the robot time Δt later
     */
    public double[] getNextState(double[] currentConfig, double[] controls) {

        double[] newState = new double[13];
        limitSpeeds(controls, maxSpeed);

        double[] newChassisConfig = odometry(getChassisConfig(currentConfig), controls);
        double[] newAngles = eulerStep(getAngles(currentConfig), controls);
        Matrix.replaceRangeFromArray(newChassisConfig, newState, 0);
        Matrix.replaceRangeFromArray(newAngles, newState, 3);

        return newState;
    }

}
