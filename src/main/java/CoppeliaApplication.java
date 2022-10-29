import model.Cube;

import java.util.List;

public class CoppeliaApplication {

    private final String trajectoryPath;
    private final String youBotPath;

    public static void main(String[] args) {

        CoppeliaApplication main = new CoppeliaApplication();

        CSV.clearCSVFile(main.youBotPath);
        main.processTaskList();
    }

    public CoppeliaApplication() {
        this.trajectoryPath = "trajectory.csv";
        this.youBotPath = "youBot.csv";
    }

    public void processTaskList() {
        YouBot youBot = new YouBot();
        List<Cube> taskList = new HanoiTower().getTaskList();

        for (Cube cube : taskList) {
            CSV.clearCSVFile(trajectoryPath);
            PickAndPlace task = new PickAndPlace(youBot, cube);
            task.getConfigsFromTrajectory();
        }
    }
}
