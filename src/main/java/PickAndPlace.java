import libraries.Matrix;
import model.Cube;

public class PickAndPlace {

    private final YouBot robot;
    private final NextState nextState;
    private final TrajectoryGeneration trajectory;
    private final FeedbackControl feedback;
    private final String youBotPath;

    public static void main(String[] args) {

        PickAndPlace main = new PickAndPlace();
        main.getConfigsFromTrajectory();
    }

    public PickAndPlace() {
        this.robot = new YouBot();
        this.nextState = new NextState(robot.DELTA_T, robot.MAX_SPEED, robot.F);
        Cube cube = new Cube(robot.cubeInitial, robot.cubeGoal);
        this.trajectory = new TrajectoryGeneration(robot.endEffectorSE3(robot.initialConfig), cube, robot.DELTA_T);
        this.feedback = new FeedbackControl(this.robot);
        this.youBotPath = "youBot.csv";
    }

    public void getConfigsFromTrajectory() {

        CSV.clearCSVFile(youBotPath);
        double[][][] trajMatrix = trajectory.getTrajectoryMatrix();

        // Loop through trajectory, calculate controls, and append configs to youBot CSV
        for (int row=0; row < trajMatrix.length - 1; row++) {
            robot.Xd = trajectoryToSE3(trajMatrix[row][0]);
            robot.XdNext = trajectoryToSE3(trajMatrix[row+1][0]);

            // Get controls needed for NextState
            robot.currentControls = feedback.feedbackControl(robot.X, robot.Xd, robot.XdNext, robot.currentConfig);
            double grip = trajMatrix[row][0][12];

            // Get next configuration and append to youBot CSV file
            Matrix.replaceRangeFromArray(nextState.getNextState(robot.currentConfig, robot.currentControls), robot.currentConfig, 0);
            robot.currentConfig[12] = grip;
            CSV.writeToCSV(youBotPath, robot.currentConfig);

            // Set next configuration as X for next iteration
            robot.X = robot.endEffectorSE3(robot.currentConfig);
        }
    }

    public double[][] trajectoryToSE3(double[] flattenedTrajectory) {
        /*
        Returns SE(3) transformation matrix from flattened trajectory
         */
        double[] rot = Matrix.rangeFromArray(flattenedTrajectory, 0, 9);
        double[][] rotMatrix = Matrix.reshapeArray(rot, 3, 3);
        double[] pos = Matrix.rangeFromArray(flattenedTrajectory, 9, 12);

        double[][] se3Matrix = Matrix.identityMatrix(4);
        Matrix.replaceRangeFromMatrix(rotMatrix, se3Matrix, 0, 0);
        Matrix.replaceRangeFromMatrix(Matrix.transposeArray(pos), se3Matrix, 0, 3);
        return se3Matrix;
    }
}
