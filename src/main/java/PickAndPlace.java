import matrix.Matrix;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class PickAndPlace {

    public static void main(String[] args) {
        YouBot robot = new YouBot();

        // Generate trajectory file
        TrajectoryGeneration trajectory = new TrajectoryGeneration();
        String trajectoryPath = "trajectory.csv";
        getTrajectory(robot, trajectoryPath, trajectory);

        // Load configurations from trajectory CSV and append configs to youBot CSV
        File trajFile = new File(trajectoryPath);
        double[] currentTraj = null;
        try(Scanner csvReader = new Scanner(trajFile)) {
            while (csvReader.hasNextLine()) {
                String currentConfig = csvReader.nextLine();
                String[] configStrings = currentConfig.split(",");
                currentTraj = CSV.convertStringToDoubleArray(configStrings);

                // Get X, Xd, XdNext from current trajectory

                // Get controls needed for NextState

                // Get next configuration and append to youBot CSV file

                // Set next configuration as X for next iteration

            }
        }
        catch (FileNotFoundException e) {
            System.out.println("Cannot open or read file.");
        }
    }

    public static void getTrajectory(YouBot robot, String filePath, TrajectoryGeneration traj) {
        // Clear CSV file before running
        CSV.clearCSVFile(filePath);

        // Convert initial config to SE(3) matrix
        double[][] chassisSE3 = traj.spaceToChassis(0,0,0);
        double[][] endEffectorSE3 = traj.chassisToEndEffector(chassisSE3);

        // Calculate full trajectory path
        traj.motionPlanning(endEffectorSE3, robot.cubeInitial, robot.cubeGoal);
    }

    public double[][] trajectoryToSE3(double[] flattenedTrajectory) {
        /*
        Returns SE(3) transformation matrix from 13-vector of current configuration
         */
        return Matrix.reshapeArray(Matrix.rangeFromArray(flattenedTrajectory, 0, 12), 3, 3);
    }
}
