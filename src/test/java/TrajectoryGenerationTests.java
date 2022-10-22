import libraries.Matrix;
import libraries.Robotics;
import org.junit.Assert;
import org.junit.Test;

public class TrajectoryGenerationTests {

    YouBot robot = new YouBot();
    TrajectoryGeneration traj = new TrajectoryGeneration();
    String trajFilePath = "trajectory.csv";

    @Test
    public void checkMotionPlanning() {
        // Clear CSV file before running
        CSV.clearCSVFile(trajFilePath);

        // Convert initial config to SE(3) matrix
        double[][] chassisSE3 = robot.spaceToChassis(0,0,0);
        double[][] T0e = Robotics.fkInBody(robot.M0e, robot.BList, robot.thetaList);
        double[][] robotInitial = Matrix.matrixMultiplication(chassisSE3, robot.Tb0);
        robotInitial = Matrix.matrixMultiplication(robotInitial, T0e);

        // Calculate full trajectory path
        traj.motionPlanning(robotInitial, robot.cubeInitial, robot.cubeGoal, robot.DELTA_T);

        double[] expected = {0.0, 1.0, 0.0, 0.7071067811865475, 0.0, -0.7071067811865476, -0.7071067811865476, 0.0, -0.7071067811865475, 0.0, -1.0, 0.1, 0.0};
        double[] actual = CSV.readFromCSV(trajFilePath);
        Assert.assertArrayEquals(expected, actual, 0.0001);
    }
}
