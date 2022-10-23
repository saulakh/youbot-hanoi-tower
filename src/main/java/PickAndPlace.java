import libraries.Matrix;
import libraries.Robotics;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class PickAndPlace {

    YouBot robot = new YouBot();
    String trajectoryPath = "trajectory.csv";
    NextState nextState = new NextState(robot.DELTA_T, robot.MAX_SPEED, robot.F);
    TrajectoryGeneration trajectory = new TrajectoryGeneration();
    FeedbackControl feedback = new FeedbackControl();
    String youBotPath = "youBot.csv";

    public static void main(String[] args) {

        PickAndPlace main = new PickAndPlace();
        main.getConfigsFromTrajectory();
    }

    public PickAndPlace() {
    }

    public void getConfigsFromTrajectory() {

        // Generate trajectory file
        getTrajectory(robot, trajectoryPath, trajectory);
        // Clear youBot File
        CSV.clearCSVFile(youBotPath);

        // Load configurations from trajectory CSV and generate matrix of values
        File trajFile = new File(trajectoryPath);
        double[][][] trajMatrix = createTrajectoryMatrix(trajFile);

        // Loop through trajectory, calculate controls, and append configs to youBot CSV
        for (int row=0; row < trajMatrix.length - 1; row++) {
            robot.Xd = trajectoryToSE3(trajMatrix[row][0]);
            robot.XdNext = trajectoryToSE3(trajMatrix[row+1][0]);

            // Get controls needed for NextState
            robot.currentControls = feedback.feedbackControl(robot, robot.X, robot.Xd, robot.XdNext, robot.currentConfig);
            double grip = trajMatrix[row][0][12];

            // Get next configuration and append to youBot CSV file
            Matrix.replaceRangeFromArray(nextState.getNextState(robot.currentConfig, robot.currentControls), robot.currentConfig, 0);
            robot.currentConfig[12] = grip;
            CSV.writeToCSV(youBotPath, robot.currentConfig);

            // Set next configuration as X for next iteration
            robot.X = robot.endEffectorSE3(robot.currentConfig);
        }
    }

    public void getTrajectory(YouBot robot, String filePath, TrajectoryGeneration traj) {
        // Clear CSV file before running
        CSV.clearCSVFile(filePath);

        // Get SE(3) matrix for robot's initial position
        double[][] chassisSE3 = robot.spaceToChassis(robot.initialConfig[0],robot.initialConfig[1],robot.initialConfig[2]);
        double[][] T0e = Robotics.fkInBody(robot.M0e, robot.BList, robot.thetaList);
        double[][] robotInitial = Matrix.matrixMultiplication(chassisSE3, robot.Tb0);
        robotInitial = Matrix.matrixMultiplication(robotInitial, T0e);

        // Calculate full trajectory path
        traj.motionPlanning(robotInitial, robot.cubeInitial, robot.cubeGoal, robot.DELTA_T);
    }

    public double[][][] createTrajectoryMatrix(File trajectoryFile) {
        // TODO: don't hardcode the 1600 rows here
        double[][][] trajectoryMatrix = new double[1600][1][13];
        int index = 0;

        try(Scanner csvReader = new Scanner(trajectoryFile)) {
            while (csvReader.hasNextLine()) {
                trajectoryMatrix[index][0] = CSV.convertStringToDoubleArray(csvReader.nextLine().split(","));
                index++;
            }
        }
        catch (FileNotFoundException e) {
            System.out.println("Cannot open or read file.");
        }
        return trajectoryMatrix;
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
