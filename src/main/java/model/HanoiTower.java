package model;

import libraries.Robotics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class HanoiTower implements Job {

    private final YouBot robot;
    private final Map<String, double[][]> positionMap;
    private List<Task> taskList;

    private final double[][] startPad = {{1,0,0,0.75},{0,1,0,0},{0,0,1,0.05},{0,0,0,1}};
    private double[][] goalPad = {{1,0,0,-0.75*Math.sin(Math.PI/6)},{0,1,0,-0.75*Math.cos(Math.PI/6)},{0,0,1,0.05},{0,0,0,1}};
    private double[][] extraPad = {{1,0,0,-0.75*Math.sin(Math.PI/6)},{0,1,0,0.75*Math.cos(Math.PI/6)},{0,0,1,0.05},{0,0,0,1}};

    public HanoiTower(YouBot robot) {
        this.robot = robot;
        this.positionMap = loadCubePositions();
        this.taskList = loadTaskList();
    }

    /**
     * Loads a map of possible cube positions for the Hanoi Tower
     * @return positionMap : Map of cube location names and the respective SE(3) position in each location
     */
    private Map<String, double[][]> loadCubePositions() {
        Map<String, double[][]> positionMap = new TreeMap<>();
        double cubeSize = 0.05;
        double middleShift = 0.04;
        double bottomShift = 0.08;
        double goalTheta = -2 * Math.PI / 3;
        double padTheta = 2 * Math.PI / 3;

        goalPad = Robotics.rotateAboutZ(goalPad, goalTheta);
        extraPad = Robotics.rotateAboutZ(extraPad, padTheta);

        // Start Pad
        positionMap.put("startBottomLeft", Robotics.translate(startPad, 0, bottomShift, 0));
        positionMap.put("startBottomMiddle", startPad);
        positionMap.put("startBottomRight", Robotics.translate(startPad, 0, -bottomShift, 0));
        positionMap.put("startMiddleLeft", Robotics.translate(startPad, 0, middleShift, cubeSize));
        positionMap.put("startMiddleRight", Robotics.translate(startPad, 0, -middleShift, cubeSize));
        positionMap.put("startTop", Robotics.translate(startPad, 0, 0, cubeSize * 2));
        // Extra Pad
        positionMap.put("padLeft", Robotics.translate(extraPad, 0, middleShift, 0));
        positionMap.put("padMiddle", Robotics.translate(extraPad, 0, 0, cubeSize));
        positionMap.put("padRight", Robotics.translate(extraPad, 0, -middleShift, 0));
        // Goal Pad
        positionMap.put("goalBottomLeft", Robotics.translate(goalPad, 0, bottomShift, 0));
        positionMap.put("goalBottomMiddle", goalPad);
        positionMap.put("goalBottomRight", Robotics.translate(goalPad, 0, -bottomShift, 0));
        positionMap.put("goalMiddleLeft", Robotics.translate(goalPad, 0, middleShift, cubeSize));
        positionMap.put("goalMiddleRight", Robotics.translate(goalPad, 0, -middleShift, cubeSize));
        positionMap.put("goalTop", Robotics.translate(goalPad, 0, 0, cubeSize * 2));
        return positionMap;
    }

    /**
     * Loads a taskList of pick and place operations to move the cubes from the start to goal pad, using the extra pad
     * as a placeholder for the Hanoi Tower challenge
     * @return taskList : A list of tasks required to move each cube for the Hanoi Tower
     */
    private List<Task> loadTaskList() {
        taskList = new ArrayList<>();
        PickAndPlace pickAndPlace = new PickAndPlace(robot, taskList);

        // Move top cube to goal pad
        pickAndPlace.addToTaskList(positionMap.get("startTop"), positionMap.get("goalBottomMiddle"));
        // Move middle row to extra pad
        pickAndPlace.addToTaskList(positionMap.get("startMiddleLeft"), positionMap.get("padLeft"));
        pickAndPlace.addToTaskList(positionMap.get("startMiddleRight"), positionMap.get("padRight"));
        // Move top cube to extra pad
        pickAndPlace.addToTaskList(positionMap.get("goalBottomMiddle"), positionMap.get("padMiddle"));
        // Move bottom row to goal pad
        pickAndPlace.addToTaskList(positionMap.get("startBottomLeft"), positionMap.get("goalBottomLeft"));
        pickAndPlace.addToTaskList(positionMap.get("startBottomMiddle"), positionMap.get("goalBottomMiddle"));
        pickAndPlace.addToTaskList(positionMap.get("startBottomRight"), positionMap.get("goalBottomRight"));
        // Move top cube to start pad
        pickAndPlace.addToTaskList(positionMap.get("padMiddle"), positionMap.get("startBottomMiddle"));
        // Move remaining cubes to goal pad
        pickAndPlace.addToTaskList(positionMap.get("padLeft"), positionMap.get("goalMiddleLeft"));
        pickAndPlace.addToTaskList(positionMap.get("padRight"), positionMap.get("goalMiddleRight"));
        pickAndPlace.addToTaskList(positionMap.get("startBottomMiddle"), positionMap.get("goalTop"));

        return taskList;
    }

    /**
     * Moves the tower of cubes from the start to the goal pad
     * @return taskList : Sequence of tasks for pick and place jobs for each cube
     */
    @Override
    public List<Task> getTaskList() {
        return loadTaskList();
    }
}
