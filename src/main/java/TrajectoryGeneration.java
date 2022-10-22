import libraries.Matrix;
import libraries.Robotics;

public class TrajectoryGeneration {

    public void getTrajectory(double[][] xStart, double[][] xEnd, int gripState, int Tf, double dT) {
        /*
         Inputs:
         - xStart: Starting end-effector Se3 configuration
         - xEnd: Goal end-effector Se3 configuration
         - gripState: Gripper state for this section of trajectory
         - Tf: total time of motion for this section (in seconds)
         - dT: timestep Δt
         Output:
         - Appends flattened trajectory to csv file
         */
        int N = (int)(Tf / dT);
        double[][][] trajectory = Robotics.screwTrajectory(xStart, xEnd, Tf, N, 3);
        String trajFilePath = "trajectory.csv";

        // Extract rotation and position values from trajectory
        for (double[][] se3 : trajectory) {
            double[] rot = Matrix.flattenedMatrix(Robotics.transToRot(se3));
            double[] pos = Robotics.transToPos(se3);
            // End-effector config is [rot, pos, grip]
            double[] config = new double[13];
            Matrix.replaceRangeFromArray(rot, config, 0);
            Matrix.replaceRangeFromArray(pos, config, 9);
            config[12] = gripState;
            CSV.writeToCSV(trajFilePath, config);
        }
    }

    public double[][] graspPosition(double[][] cubeConfig, double theta) {
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

    public double[][] standoffPosition(double[][] cubeConfig, double theta, double height) {
        /*
        Returns the end-effector standoff configuration, relative to cube position
         */
        double[][] standoff = Matrix.identityMatrix(4);
        double[][] rotY = {{Math.cos(theta),0,Math.sin(theta)},{0,1,0},{-Math.sin(theta),0,Math.cos(theta)}};
        // Change end-effector orientation
        Matrix.replaceRangeFromMatrix(rotY, standoff, 0, 0);
        // Standoff at desired distance above cube (in meters)
        standoff[2][3] += height;
        return Matrix.matrixMultiplication(cubeConfig, standoff);
    }

    public void motionPlanning(double[][] robotInitial, double[][] cubeInitial, double[][] cubeFinal, double dT) {
        /*
        Inputs:
        - robotInitial: End-effector configuration at robot's initial position
        - cubeInitial: Cube configuration at its initial position
        - cubeFinal: Cube configuration at its final position
        - dT: timestep Δt
        Ouput:
        - Appends full trajectory to csv file, moving the cube from inital to goal position
         */
        int gripState = 0;
        int gripperTime = 1;
        int lowerToGraspTime = 1;
        int moveToPositionTime = 5;

        // 1) Move gripper to standoff configuration over initial cube location
        double[][] standoffInitial = standoffPosition(cubeInitial, 3*Math.PI/4, 0.075);
        getTrajectory(robotInitial, standoffInitial, gripState, moveToPositionTime, dT);

        // 2) Move gripper down to initial grasp position
        double[][] graspInitial = graspPosition(cubeInitial, 3*Math.PI/4);
        getTrajectory(standoffInitial, graspInitial, gripState, lowerToGraspTime, dT);

        // 3) Close gripper
        gripState = 1;
        getTrajectory(graspInitial, graspInitial, gripState, gripperTime, dT);

        // 4) Move gripper back up to initial standoff configuration
        getTrajectory(graspInitial, standoffInitial, gripState, lowerToGraspTime, dT);

        // 5) Move gripper to standoff configuration over goal cube location
        double[][] standoffFinal = standoffPosition(cubeFinal, 3*Math.PI/4, 0.075);
        getTrajectory(standoffInitial, standoffFinal, gripState, moveToPositionTime, dT);

        // 6) Move gripper down to final grasp position
        double[][] graspFinal = graspPosition(cubeFinal, 3*Math.PI/4);
        getTrajectory(standoffFinal, graspFinal, gripState, lowerToGraspTime, dT);

        // 7) Open Gripper
        gripState = 0;
        getTrajectory(graspFinal, graspFinal, gripState, gripperTime, dT);

        // 8) Move gripper back to final standoff configuration
        getTrajectory(graspFinal, standoffFinal, gripState, lowerToGraspTime, dT);
    }
}
