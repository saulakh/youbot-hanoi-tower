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

    public void graspPosition() {
        /*
        Returns end-effector grasp configuration, rotated about y-axis from cube position
         */
    }

    public void standoffPosition() {
        /*
        Returns the end-effector standoff configuration, relative to cube position
         */
    }

    public void chassisToEndEffector() {
        /*
        Input:
        - Transformation matrix from space frame to chassis frame
        Output:
        - End effector configuration in the space frame, when all joint angles are zero
         */
    }

    public double[][] spaceToChassis(double phi, double x, double y) {
        /*
        Transformation matrix from space frame to chassis frame
         */
        return new double[][] {{Math.cos(phi),-Math.sin(phi),0,x},{Math.sin(phi),Math.cos(phi),0,y},{0,0,1,0.0963},{0,0,0,1}};
    }

    public void motionPlanning() {
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
