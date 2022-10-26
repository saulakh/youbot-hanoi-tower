import model.Cube;

public class CoppeliaApplication {

    private final String trajectoryPath;
    private final String youBotPath;

    public static void main(String[] args) {

        CoppeliaApplication main = new CoppeliaApplication();

        CSV.clearCSVFile(main.youBotPath);
        main.run();
    }

    public CoppeliaApplication() {
        this.trajectoryPath = "trajectory.csv";
        this.youBotPath = "youBot.csv";
    }

    public void run() {
        YouBot youBot = new YouBot();
        Cube cube1 = new Cube(youBot.cubeInitial, youBot.cubeGoal);
        Cube cube2 = new Cube(youBot.cube2Initial, youBot.cube2Goal);

        // TODO: Create List<PickAndPlace> or a way to run each motion
        CSV.clearCSVFile(trajectoryPath);
        PickAndPlace pickAndPlace1 = new PickAndPlace(youBot, cube1);
        pickAndPlace1.getConfigsFromTrajectory();

//        CSV.clearCSVFile(trajectoryPath);
//        PickAndPlace pickAndPlace2 = new PickAndPlace(youBot, cube2);
//        pickAndPlace2.getConfigsFromTrajectory();
    }
}
