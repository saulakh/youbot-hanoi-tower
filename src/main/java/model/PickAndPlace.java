package model;

import libraries.Matrix;
import path.FeedbackControl;
import path.NextState;
import path.TrajectoryGeneration;
import util.CSV;

public class PickAndPlace {

    private final YouBot robot;
    private final NextState nextState;
    private final TrajectoryGeneration trajectory;
    private final FeedbackControl feedback;
    private final String youBotPath;

    public PickAndPlace(YouBot robot, Cube cube) {
        this.robot = robot;
        this.nextState = new NextState(robot.DELTA_T, robot.MAX_SPEED, robot.F);
        this.trajectory = new TrajectoryGeneration(robot, cube);
        this.feedback = new FeedbackControl(robot);
        this.youBotPath = "youBot.csv";
    }

    /**
     * Loops through end-effector trajectory, calculates controls for joints and wheels, and appends configs to youBot CSV
     */
    public void getConfigsFromTrajectory() {

        double[][][] trajMatrix = trajectory.getTrajectoryMatrix();

        for (int row=0; row < trajMatrix.length - 1; row++) {
            robot.Xd = trajectoryToSE3(trajMatrix[row][0]);
            robot.XdNext = trajectoryToSE3(trajMatrix[row+1][0]);

            // Get controls needed for path.NextState
            robot.currentControls = feedback.getControls();

            // Get next configuration and append to youBot CSV file
            Matrix.replaceRangeFromArray(nextState.getNextState(robot.currentConfig, robot.currentControls), robot.currentConfig, 0);
            robot.currentConfig[12] = trajMatrix[row][0][12];
            CSV.writeToCSV(youBotPath, robot.currentConfig);

            // Set next configuration as X for next iteration
            robot.X = robot.endEffectorSE3(robot.currentConfig);
        }
    }

    /**
     * Returns SE(3) transformation matrix from flattened trajectory
     * @param flattenedTrajectory Flattened SE(3) matrix representing the end-effector position in the world frame
     * @return SE(3) transformation matrix representing the same end-effector position
     */
    public double[][] trajectoryToSE3(double[] flattenedTrajectory) {

        double[] rot = Matrix.rangeFromArray(flattenedTrajectory, 0, 9);
        double[][] rotMatrix = Matrix.reshapeArray(rot, 3, 3);
        double[] pos = Matrix.rangeFromArray(flattenedTrajectory, 9, 12);

        double[][] se3Matrix = Matrix.identityMatrix(4);
        Matrix.replaceRangeFromMatrix(rotMatrix, se3Matrix, 0, 0);
        Matrix.replaceRangeFromMatrix(Matrix.transposeArray(pos), se3Matrix, 0, 3);
        return se3Matrix;
    }
}
