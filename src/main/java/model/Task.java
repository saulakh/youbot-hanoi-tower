package model;

public class Task {

    private double[][] robotInitial;
    private final double[][] robotGoal;
    private final int gripState;
    private final double Tf;

    public Task(double[][] initial, double[][] goal, int gripState, double Tf) {
        this.robotInitial = initial;
        this.robotGoal = goal;
        this.gripState = gripState;
        this.Tf = Tf;
    }

    public double[][] getRobotInitial() {
        return robotInitial;
    }

    public void setRobotInitial(double[][] robotInitial) {
        this.robotInitial = robotInitial;
    }

    public double[][] getRobotGoal() {
        return robotGoal;
    }

    public int getGripState() {
        return gripState;
    }

    public double getTf() {
        return Tf;
    }

}
