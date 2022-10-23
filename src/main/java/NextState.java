import libraries.Matrix;

import java.util.Arrays;

public class NextState {

    private final double dT;
    private final double maxSpeed;
    private final double[][] F;

    public NextState(double dT, double maxSpeed, double[][] FMatrix) {
        /*
        - dT: time step Δt
         - maxSpeed: limits the maximum angular speed of arm joints and wheels to [-max_speed rad/s, max_speed rad/s]
         - FMatrix: Pseudo inverse of H(0) matrix, the chassis kinematic model of the robot
         */
        this.dT = dT;
        this.maxSpeed = maxSpeed;
        this.F = FMatrix;
    }

    private double[] getAngles(double[] config) {
        /*
        Returns the 5 joint arm angles, and 4 wheel angles from the input configuration
         */
        return Matrix.rangeFromArray(config, 3, 12);
    }

    private double[] eulerStep(double[] angles, double[] controls) {
        /*
         Inputs:
         - angles: includes 5 joint arm angles, and 4 wheel angles from currentConfig
         - controls: includes 5 joint speeds and 4 wheel speeds
         Output:
         - angles: updates to 5 new arm joint angles and 4 new wheel angles
         */
        for (int i=0; i < angles.length; i++) {
            angles[i] += controls[i] * dT;
        }
        return angles;
    }

    private double[] getChassisConfig(double[] config) {
        /*
        Returns the chassis state (phi, x, y) from the input configuration
         */
        return Matrix.rangeFromArray(config, 0, 3);
    }

    private double[] getDTheta(double[] controls) {
        /*
        Returns the change in wheel angles (wheel1, wheel2, wheel3, wheel4), calculated from controls * dT
         */
        return Matrix.scalarArrayMultiplication(Arrays.copyOfRange(controls, 5, 9), dT);
    }

    private double[][] getBodyTwist(double[][] F, double[] controls) {
        /*
        Returns the body twist Vb = (omegaBZ, Vbx, Vby) of the chassis, where Vb = FΔ0
         */
        double[] dTheta = getDTheta(controls);
        return Matrix.matrixMultiplication(F, Matrix.transposeArray(dTheta));
    }

    private double[][] getDeltaQbMatrix(double[] controls) {
        /*
        Returns 1x3 matrix ΔQb = (ΔomegaBz, ΔVbx, ΔVby), for the change in coordinates relative to the body frame
         */
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

    private double[][] getDeltaQMatrix(double[] chassisConfig, double[] controls) {
        /*
        Transforms ΔQb in the body frame, to ΔQ in the fixed space frame using the chassis angle phi
         */
        double phi = chassisConfig[0];
        double[][] deltaQb = getDeltaQbMatrix(controls);

        return Matrix.matrixMultiplication(new double[][] {{1,0,0},{0,Math.cos(phi),-Math.sin(phi)},{0,Math.sin(phi),Math.cos(phi)}}, deltaQb);
    }

    private double[] odometry(double[] chassisConfig, double[] controls) {
        /*
         Inputs:
         - chassisConfig: current chassis state (phi,x,y)
         - dTheta: change in wheel angles (wheel1, wheel2, wheel3, wheel4)
         Output:
         - Returns the updated chassis state (phi,x,y), using Q(k+1) = Qk + ΔQ
         */
        double[][] deltaQ = getDeltaQMatrix(chassisConfig, controls);
        chassisConfig[0] += deltaQ[0][0];
        chassisConfig[1] += deltaQ[1][0];
        chassisConfig[2] += deltaQ[2][0];

        return chassisConfig;
    }

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

    public double[] getNextState(double[] currentConfig, double[] controls) {
        /*
         Inputs:
         - currentConfig: 12-vector for current robot configuration (phi,x,y for chassis, 5 joint angles, and 4 wheel angles)
         - controls: 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
         Output:
         - newState: 12-vector representing the configuration of the robot time Δt later
         */
        double[] newState = new double[13];
        limitSpeeds(controls, maxSpeed);

        // Update configuration with odometry and eulerStep
        double[] newChassisConfig = odometry(getChassisConfig(currentConfig), controls);
        double[] newAngles = eulerStep(getAngles(currentConfig), controls);
        Matrix.replaceRangeFromArray(newChassisConfig, newState, 0);
        Matrix.replaceRangeFromArray(newAngles, newState, 3);

        return newState;
    }

}
