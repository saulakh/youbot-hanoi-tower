package model;

import libraries.Robotics;

import java.util.List;

public class PickAndPlace implements Job {

    private final List<Task> taskList;
    private final double theta;
    private final double standoffHeight;
    private final double graspHeight;
    private double[][] currentPosition;

    public PickAndPlace(YouBot robot, List<Task> taskList) {
        this.taskList = taskList;
        this.theta = 3 * Math.PI / 4;
        this.standoffHeight = 0.075;
        this.graspHeight = -0.0215;
        this.currentPosition = robot.endEffectorSE3(robot.currentConfig);
    }

    /**
     * Returns end-effector grasp configuration, rotated about y-axis from cube position
     * @param cubeConfig SE(3) representation of cube position
     * @return SE(3) representation of grasp position for end-effector, oriented to grasp cube from above
     */
    public double[][] getGraspPosition(double[][] cubeConfig) {
        double[][] newConfig = Robotics.rotateAboutY(cubeConfig, theta);
        newConfig[2][3] += graspHeight;
        return newConfig;
    }

    /**
     * Returns the end-effector standoff configuration, relative to cube position
     * @param cubeConfig SE(3) representation of cube position
     * @return SE(3) representation of standoff position for end-effector before lowering down to grasp cube
     */
    public double[][] getStandoffPosition(double[][] cubeConfig) {
        double[][] newConfig = Robotics.rotateAboutY(cubeConfig, theta);
        newConfig[2][3] += standoffHeight;
        return newConfig;
    }

    /**
     * Adds a series of tasks to the robot's current task list, moving the cube from the initial to the goal position
     * @param cubeInitial SE(3) representation of initial cube position
     * @param cubeGoal SE(3) representation of goal cube position
     */
    public List<Task> addToTaskList(double[][] cubeInitial, double[][] cubeGoal) {

        int gripState = 0;
        int gripperTime = 1;
        int moveToGraspTime = 1;
        int moveToPositionTime = 5;

        // 1) Move gripper to standoff configuration over initial cube location
        double[][] standoffInitial = getStandoffPosition(cubeInitial);
        taskList.add(new Task(currentPosition, standoffInitial, gripState, moveToPositionTime));

        // 2) Move gripper down to initial grasp position
        double[][] graspInitial = getGraspPosition(cubeInitial);
        taskList.add(new Task(standoffInitial, graspInitial, gripState, moveToGraspTime));

        // 3) Close gripper
        gripState = 1;
        taskList.add(new Task(graspInitial, graspInitial, gripState, gripperTime));

        // 4) Move gripper back up to initial standoff configuration
        taskList.add(new Task(graspInitial, standoffInitial, gripState, moveToGraspTime));

        // 5) Move gripper to standoff configuration over goal cube location
        double[][] standoffFinal = getStandoffPosition(cubeGoal);
        taskList.add(new Task(standoffInitial, standoffFinal, gripState, moveToPositionTime));

        // 6) Move gripper down to final grasp position
        double[][] graspFinal = getGraspPosition(cubeGoal);
        taskList.add(new Task(standoffFinal, graspFinal, gripState, moveToGraspTime));

        // 7) Open Gripper
        gripState = 0;
        taskList.add(new Task(graspFinal, graspFinal, gripState, gripperTime));

        // 8) Move gripper back to final standoff configuration
        taskList.add(new Task(graspFinal, standoffFinal, gripState, moveToGraspTime));

        // Update robot's position for path planning (before robot config updates)
        currentPosition = standoffFinal;

        return taskList;
    }

    /**
     * Moves a cube from the start to the goal pad
     * @return taskList : Sequence of tasks for one pick and place job
     */
    @Override
    public List<Task> getTaskList() {
        double[][] cubeInitial = new double[][] {{1,0,0,1},{0,1,0,0},{0,0,1,0.025},{0,0,0,1}};
        double[][] cubeGoal = new double[][] {{0,1,0,0},{-1,0,0,-1},{0,0,1,0.025},{0,0,0,1}};
        return addToTaskList(cubeInitial, cubeGoal);
    }
}
