package model;

import libraries.Matrix;

public class Cube {

    private final double[][] initialConfig;
    private final double[][] goalConfig;
    private final double theta;
    private final double graspHeight;

    public Cube(double[][] initialConfig, double[][] goalConfig) {
        this.initialConfig = initialConfig;
        this.goalConfig = goalConfig;
        this.theta = 3 * Math.PI / 4;
        this.graspHeight = 0.075;
    }

    public double[][] getInitialConfig() {
        return initialConfig;
    }

    public double[][] getGoalConfig() {
        return goalConfig;
    }

    public double[][] getGraspPosition(double[][] cubeConfig) {
        /*
        Returns end-effector grasp configuration, rotated about y-axis from cube position
         */
        double[][] grasp = Matrix.identityMatrix(4);
        // Rotate theta (in radians) about y-axis in space frame
        double[][] rotY = {{Math.cos(theta),0,Math.sin(theta)},{0,1,0},{-Math.sin(theta),0,Math.cos(theta)}};
        // Change end-effector orientation
        Matrix.replaceRangeFromMatrix(rotY, grasp, 0, 0);
        // Lower center of gripper to center height of cube
        grasp[2][3] -= 0.0215;
        return Matrix.matrixMultiplication(cubeConfig, grasp);
    }

    public double[][] getStandoffPosition(double[][] cubeConfig) {
        /*
        Returns the end-effector standoff configuration, relative to cube position
         */
        double[][] standoff = Matrix.identityMatrix(4);
        double[][] rotY = {{Math.cos(theta),0,Math.sin(theta)},{0,1,0},{-Math.sin(theta),0,Math.cos(theta)}};
        // Change end-effector orientation
        Matrix.replaceRangeFromMatrix(rotY, standoff, 0, 0);
        // Standoff at desired distance above cube (in meters)
        standoff[2][3] += graspHeight;
        return Matrix.matrixMultiplication(cubeConfig, standoff);
    }
}
