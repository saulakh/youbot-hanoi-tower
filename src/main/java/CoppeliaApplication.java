import libraries.Matrix;
import model.*;
import path.FeedbackControl;
import path.NextState;
import path.TrajectoryGeneration;
import util.CSV;

import java.util.ArrayList;
import java.util.List;

public class CoppeliaApplication {

    private final YouBot robot;
    private final Job hanoiTower;
    private final NextState nextState;
    private final TrajectoryGeneration trajectory;
    private final FeedbackControl feedback;

    private List<Task> taskList;
    private final String youBotPath;

    public static void main(String[] args) {

        CoppeliaApplication main = new CoppeliaApplication();

        CSV.clearCSVFile(main.youBotPath);
        main.processTaskList(main.hanoiTower);
    }

    public CoppeliaApplication() {
        this.robot = new YouBot();
        this.hanoiTower = new HanoiTower(robot);
        this.nextState = new NextState(robot.DELTA_T, robot.MAX_SPEED, robot.F);
        this.trajectory = new TrajectoryGeneration(robot);
        this.feedback = new FeedbackControl(robot);
        this.taskList = new ArrayList<>();
        this.youBotPath = "youBot.csv";
    }

    /**
     * Processes each task of a Job. Loads the trajectory of SE(3) end-effector positions, calculates the controls
     * needed to reach each position, and adds the robot configurations to the output CSV file for each time step
     */
    public void processTaskList(Job job) {
        taskList = job.getTaskList();

        for (Task task : taskList) {
            double[][] path = trajectory.getTrajectory(task);

            for (int row=0; row < path.length - 1; row++) {
                robot.Xd = trajectoryToSE3(path[row]);
                robot.XdNext = trajectoryToSE3(path[row+1]);

                // Get controls needed for NextState
                robot.currentControls = feedback.getControls();

                // Get next configuration and append to youBot CSV file
                Matrix.replaceRangeFromArray(nextState.getNextState(robot.currentConfig, robot.currentControls), robot.currentConfig, 0);
                robot.currentConfig[12] = path[row][12];
                CSV.writeToCSV(youBotPath, robot.currentConfig);

                // Set next configuration as X for next iteration
                robot.X = robot.endEffectorSE3(robot.currentConfig);
            }
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
