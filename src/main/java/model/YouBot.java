package model;

import libraries.Matrix;
import libraries.Robotics;

public class YouBot {

    /**
     * Tb0 is the transformation matrix from the base frame of the arm to the mobile base / chassis frame
     * M0e is the end-effector configuration when the robot is in its zero configuration
     */

    public final double DELTA_T = 0.01; // seconds
    public final double MAX_SPEED = 25; // rad/s

    // Chassis dimensions
    public final double WHEEL_RADIUS = 0.0475; // meters
    public final double FORWARD_BACKWARD_LENGTH = 0.47;
    public final double SIDE_TO_SIDE_WIDTH = 0.30;

    // H(0) matrix: Chassis kinematic model
    public double[][] H = Matrix.scalarMultiplication(new double[][] {{-FORWARD_BACKWARD_LENGTH / 2 - SIDE_TO_SIDE_WIDTH / 2, 1, -1},
            {FORWARD_BACKWARD_LENGTH / 2 + SIDE_TO_SIDE_WIDTH / 2, 1, 1},
            {FORWARD_BACKWARD_LENGTH / 2 + SIDE_TO_SIDE_WIDTH / 2, 1, -1},
            {-FORWARD_BACKWARD_LENGTH / 2 - SIDE_TO_SIDE_WIDTH / 2, 1, 1}}, 1 / WHEEL_RADIUS);
    public double[][] F = Matrix.pseudoInverse(H);

    // Initial youBot configuration (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
    public double[] initialConfig = new double[] {0,-0.1,0.1,0,-0.2,0.2,-1.6,0,0,0,0,0,0};
    public double[] currentConfig = initialConfig;
    public double[] currentControls = new double[9]; // 5 joint speeds, 4 wheel speeds (rad/s)

    // Tb0: fixed offset from chassis frame to base frame of the arm
    public double[][] Tb0 = new double[][] {{1,0,0,0.1662},{0,1,0,0},{0,0,1,0.0026},{0,0,0,1}};
    // M0e: end-effector frame relative to base frame of the arm at home configuration, when all joint angles are zero
    public double[][] M0e = new double[][] {{1,0,0,0.033},{0,1,0,0},{0,0,1,0.6546},{0,0,0,1}};
    // BList: Screw axes for the five joints expressed in the end-effector frame
    public double[][] BList = Matrix.transposeMatrix(new double[][] {{0,0,1,0,0.033,0},{0,-1,0,-0.5076,0,0},{0,-1,0,-0.3526,0,0},{0,-1,0,-0.2176,0,0},{0,0,1,0,0,0}});

    // Kp and Ki gains
    public double KpGain = 10;
    public double KiGain = 8;
    public double[][] KpMatrix = Matrix.scalarMultiplication(Matrix.identityMatrix(6), KpGain);
    public double[][] KiMatrix = Matrix.scalarMultiplication(Matrix.identityMatrix(6), KiGain);
    public double[] errorIntegral = new double[6];

    // Initial X, Xd, and XdNext for feedforward control with initialConfig = {0,0,0,0,0,0.2,-1.6,0,0,0,0,0,0};
    public double[][] X = new double[][] {{0.17,0,0.985,0.387},{0,1,0,0},{-0.985,0,0.17,0.57},{0,0,0,1}}; // Tse, current actual end-effector config
    public double[][] Xd = new double[][] {{0,0,1,0.5},{0,1,0,0},{-1,0,0,0.5},{0,0,0,1}}; // current reference end-effector config
    public double[][] XdNext = new double[][] {{0,0,1,0.6},{0,1,0,0},{-1,0,0,0.3},{0,0,0,1}}; // end-effector reference config at the next timestep

    public YouBot() {}

    public double[][] spaceToChassis(double phi, double x, double y) {
        /*
        Transformation matrix from space frame to chassis frame
         */
        return new double[][] {{Math.cos(phi),-Math.sin(phi),0,x},{Math.sin(phi),Math.cos(phi),0,y},{0,0,1,0.0963},{0,0,0,1}};
    }

    public double[][] endEffectorSE3(double[] config) {
        /*
        Returns SE(3) transformation matrix (Tse) from 13-vector of current configuration
         */
        double[] chassisConfig = Matrix.rangeFromArray(config, 0, 3);
        double[] armConfig = Matrix.rangeFromArray(config, 3, 8);

        double[][] Tsb = spaceToChassis(chassisConfig[0], chassisConfig[1], chassisConfig[2]);
        double[][] T0e = Robotics.fkInBody(M0e, BList, armConfig);
        double[][] Ts0 = Matrix.matrixMultiplication(Tsb, Tb0);
        return Matrix.matrixMultiplication(Ts0, T0e);
    }

}
