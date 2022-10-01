import matrix.Matrix;

public class TrajectoryGeneration {

    public void getTrajectory(double[] xStart, double[] xEnd, int gripState, int Tf) {
        /*
         Inputs:
         - xStart: Starting end-effector configuration
         - xEnd: Goal end-effector configuration
         - gripState: Gripper state for this section of trajectory
         - Tf: total time of motion for this section (in seconds)
         Output:
         - configSe3: returns trajectory and appends values to csv file
         */
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

    public double[][] chassisToEndEffector(double[][] Tsb) {
        /*
        Input:
        - Tsb: Transformation matrix from space frame to chassis frame
        Output:
        - End effector configuration in the space frame, when all joint angles are zero
         */
        // Chassis frame to base frame of arm 0
        double[][] Tb0 = {{1,0,0,0.1662},{0,1,0,0},{0,0,1,0.0026},{0,0,0,1}};
        // End effector frame relative to base frame
        double[][] M0e = {{1,0,0,0.033},{0,1,0,0},{0,0,1,0.6546},{0,0,0,1}};
        // Initial configuration of end-effector
        double[][] Ts0 = Matrix.matrixMultiplication(Tsb, Tb0);
        assert Ts0 != null;
        return Matrix.matrixMultiplication(Ts0, M0e);
    }

    public double[][] spaceToChassis(double phi, double x, double y) {
        /*
        Transformation matrix from space frame to chassis frame
         */
        return new double[][] {{Math.cos(phi),-Math.sin(phi),0,x},{Math.sin(phi),Math.cos(phi),0,y},{0,0,1,0.0963},{0,0,0,1}};
    }

    public void motionPlanning(double[][] robotInitial, double[][] cubeInitial, double[][] cubeFinal) {
        /*
        Inputs:
        - robotInitial: End-effector configuration at robot's initial position
        - cubeInitial: Cube configuration at its initial position
        - cubeFinal: Cube configuration at its final position
        Ouput:
        - Appends full trajectory to csv file, moving the cube from inital to goal position
         */
    }
}
