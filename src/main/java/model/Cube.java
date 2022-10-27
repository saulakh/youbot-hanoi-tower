package model;

import libraries.Robotics;

public class Cube {

    private final double[][] initialConfig;
    private final double[][] goalConfig;
    private final double theta;
    private final double standoffHeight;
    private final double graspHeight;

    public Cube(double[][] initialConfig, double[][] goalConfig) {
        this.initialConfig = initialConfig;
        this.goalConfig = goalConfig;
        this.theta = 3 * Math.PI / 4;
        this.standoffHeight = 0.075;
        this.graspHeight = -0.0215;
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
        double[][] newConfig = Robotics.rotateAboutY(cubeConfig, theta);
        newConfig[2][3] += graspHeight;
        return newConfig;
    }

    public double[][] getStandoffPosition(double[][] cubeConfig) {
        /*
        Returns the end-effector standoff configuration, relative to cube position
         */
        double[][] newConfig = Robotics.rotateAboutY(cubeConfig, theta);
        newConfig[2][3] += standoffHeight;
        return newConfig;
    }
}
