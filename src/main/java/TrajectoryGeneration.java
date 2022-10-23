import libraries.Matrix;
import libraries.Robotics;
import model.Cube;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class TrajectoryGeneration {

    private final double[][] robotInitial;
    private final Cube cube;
    private final double[][] cubeInitial;
    private final double[][] cubeGoal;
    private final double dT;
    private final String trajectoryPath;

    /**
     *
     * @param robotInitial End-effector configuration at robot's initial position (SE3)
     * @param cube Cube with its initial and goal positions (SE3)
     * @param dT time step Δt (in seconds)
     */
    public TrajectoryGeneration(double[][] robotInitial, Cube cube, double dT) {
        this.robotInitial = robotInitial;
        this.cube = cube;
        this.cubeInitial = cube.getInitialConfig();
        this.cubeGoal = cube.getGoalConfig();
        this.dT = dT;
        this.trajectoryPath = "trajectory.csv";
    }

    /**
     * Appends flattened trajectory to CSV file
     * @param xStart Starting end-effector configuration of the robot (SE3)
     * @param xEnd Goal end-effector configuration of the robot (SE3)
     * @param gripState Gripper state for this section of the trajectory (0 = open, 1 = closed)
     * @param Tf total time of motion for this section (in seconds)
     */
    private void addToTrajectory(double[][] xStart, double[][] xEnd, int gripState, int Tf) {

        int N = (int)(Tf / dT);
        double[][][] trajectory = Robotics.screwTrajectory(xStart, xEnd, Tf, N, 3);

        // Extract rotation and position values from trajectory
        for (double[][] se3 : trajectory) {
            double[] rot = Matrix.flattenedMatrix(Robotics.transToRot(se3));
            double[] pos = Robotics.transToPos(se3);
            // End-effector config is [rot, pos, grip]
            double[] config = new double[13];
            Matrix.replaceRangeFromArray(rot, config, 0);
            Matrix.replaceRangeFromArray(pos, config, 9);
            config[12] = gripState;
            CSV.writeToCSV(trajectoryPath, config);
        }
    }

    /**
     * Appends full trajectory to CSV file, moving the cube from the initial to goal position
     * @return totalTime : Total time of motion (in seconds)
     */
    private int pickAndPlace() {

        int gripState = 0;
        int gripperTime = 1;
        int moveToGraspTime = 1;
        int moveToPositionTime = 5;
        int totalTime = 0;

        // 1) Move gripper to standoff configuration over initial cube location
        double[][] standoffInitial = cube.getStandoffPosition(cubeInitial);
        addToTrajectory(robotInitial, standoffInitial, gripState, moveToPositionTime);
        totalTime += moveToPositionTime;

        // 2) Move gripper down to initial grasp position
        double[][] graspInitial = cube.getGraspPosition(cubeInitial);
        addToTrajectory(standoffInitial, graspInitial, gripState, moveToGraspTime);
        totalTime += moveToGraspTime;

        // 3) Close gripper
        gripState = 1;
        addToTrajectory(graspInitial, graspInitial, gripState, gripperTime);
        totalTime += gripperTime;

        // 4) Move gripper back up to initial standoff configuration
        addToTrajectory(graspInitial, standoffInitial, gripState, moveToGraspTime);
        totalTime += moveToGraspTime;

        // 5) Move gripper to standoff configuration over goal cube location
        double[][] standoffFinal = cube.getStandoffPosition(cubeGoal);
        addToTrajectory(standoffInitial, standoffFinal, gripState, moveToPositionTime);
        totalTime += moveToPositionTime;

        // 6) Move gripper down to final grasp position
        double[][] graspFinal = cube.getGraspPosition(cubeGoal);
        addToTrajectory(standoffFinal, graspFinal, gripState, moveToGraspTime);
        totalTime += moveToGraspTime;

        // 7) Open Gripper
        gripState = 0;
        addToTrajectory(graspFinal, graspFinal, gripState, gripperTime);
        totalTime += gripperTime;

        // 8) Move gripper back to final standoff configuration
        addToTrajectory(graspFinal, standoffFinal, gripState, moveToGraspTime);
        totalTime += moveToGraspTime;

        return totalTime;
    }

    /**
     * Returns a matrix from the trajectory CSV file
     * @return trajectoryMatrix : matrix with flattened trajectories at each time step.
     * Each trajectory represents the (SE3) end-effector configuration relative to the space frame
     */
    public double[][][] getTrajectoryMatrix() {

        CSV.clearCSVFile(trajectoryPath);
        File trajectoryFile = new File(trajectoryPath);

        int timeOfMotion = pickAndPlace();
        int timeSteps = (int)(timeOfMotion / dT);
        double[][][] trajectoryMatrix = new double[timeSteps][1][13];
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
}
