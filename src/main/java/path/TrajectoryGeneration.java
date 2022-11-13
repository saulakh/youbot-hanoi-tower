package path;

import libraries.Matrix;
import libraries.Robotics;
import model.Task;
import model.YouBot;

public class TrajectoryGeneration {

    private final double dT;

    public TrajectoryGeneration(YouBot robot) {
        this.dT = robot.DELTA_T;
    }

    /**
     * Computes a trajectory of N SE(3) matrices using the cubic time-scaling method, and flattens each waypoint to
     * (r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, grip). Each of the N rows represents a configuration
     * of the end-effector frame {e} relative to the space frame {s} at that instant in time.
     * @param task Task containing the robot's initial and goal position, gripper state, and time of motion
     * @return trajectoryOutput : N x 13 matrix containing the flattened trajectories at each time step
     */
    public double[][] getTrajectory(Task task) {

        int N = (int) (task.getTf() / dT);
        double[][][] trajectorySE3 = Robotics.screwTrajectory(task.getRobotInitial(), task.getRobotGoal(), task.getTf(), N, 3);

        double[][] trajectoryOutput = new double[N][13];
        int index = 0;

        // Extract rotation and position values from trajectory Se(3)
        for (double[][] se3 : trajectorySE3) {
            double[] rot = Matrix.flattenedMatrix(Robotics.transToRot(se3));
            double[] pos = Robotics.transToPos(se3);
            // End-effector config is [rot, pos, grip]
            Matrix.replaceRangeFromArray(rot, trajectoryOutput[index], 0);
            Matrix.replaceRangeFromArray(pos, trajectoryOutput[index], 9);
            trajectoryOutput[index][12] = task.getGripState();
            index++;
        }
        return trajectoryOutput;
    }
}
