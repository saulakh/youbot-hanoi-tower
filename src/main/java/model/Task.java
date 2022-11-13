package model;

public class Task {

    private final double[][] robotInitial;
    private final double[][] robotGoal;
    private final int gripState;
    private final int Tf;

    public Task(double[][] initial, double[][] goal, int gripState, int Tf) {
        this.robotInitial = initial;
        this.robotGoal = goal;
        this.gripState = gripState;
        this.Tf = Tf;
    }

    public double[][] getRobotInitial() {
        return robotInitial;
    }

    public double[][] getRobotGoal() {
        return robotGoal;
    }

    public int getGripState() {
        return gripState;
    }

    public int getTf() {
        return Tf;
    }

}
