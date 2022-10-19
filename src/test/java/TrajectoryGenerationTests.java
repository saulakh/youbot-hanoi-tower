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
        double[][] chassisSE3 = traj.spaceToChassis(0,0,0);
        double[][] endEffectorSE3 = traj.chassisToEndEffector(chassisSE3);

        // Calculate full trajectory path
        traj.motionPlanning(endEffectorSE3, robot.cubeInitial, robot.cubeGoal);

        double[] expected = {0.0, 1.0, 0.0, 0.7071067811865475, 0.0, -0.7071067811865476, -0.7071067811865476, 0.0, -0.7071067811865475, 0.0, -1.0, 0.1, 0.0};
        double[] actual = CSV.readFromCSV(trajFilePath);
        Assert.assertArrayEquals(expected, actual, 0.0001);
    }
}
