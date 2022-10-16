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

        // Check values throughout the path

    }
}
