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

    private final double[][] startPad = {{1,0,0,0.75},{0,1,0,0},{0,0,1,0.06},{0,0,0,1}};
    private final double[][] goalPad = {{0,1,0,-0.75*Math.sin(Math.PI/6)},{-1,0,0,-0.75*Math.cos(Math.PI/6)},{0,0,1,0.06},{0,0,0,1}};
    private final double[][] extraPad = {{0,1,0,-0.75*Math.sin(Math.PI/6)},{-1,0,0,0.75*Math.cos(Math.PI/6)},{0,0,1,0.06},{0,0,0,1}};

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
        // Start Pad
        positionMap.put("startBottomLeft", Robotics.translate(startPad, 0, 0.06667, 0));
        positionMap.put("startBottomMiddle", startPad);
        positionMap.put("startBottomRight", Robotics.translate(startPad, 0, -0.06667, 0));
        positionMap.put("startMiddleLeft", Robotics.translate(startPad, 0, 0.04167, 0.05));
        positionMap.put("startMiddleRight", Robotics.translate(startPad, 0, -0.04167, 0.05));
        positionMap.put("startTop", Robotics.translate(startPad, 0, 0, 0.10));
        // Extra Pad
        positionMap.put("padLeft", Robotics.translate(extraPad, 0.04167*Math.cos(Math.PI/3), -0.04167*Math.sin(Math.PI/3), 0));
        positionMap.put("padMiddle", Robotics.translate(extraPad, 0, 0, 0.05));
        positionMap.put("padRight", Robotics.translate(extraPad, -0.04167*Math.cos(Math.PI/3), 0.04167*Math.sin(Math.PI/3), 0));
        // Goal Pad
        positionMap.put("goalBottomLeft", Robotics.translate(goalPad, 0.06667*Math.cos(Math.PI/3), 0.06667*Math.sin(Math.PI/3), 0));
        positionMap.put("goalBottomMiddle", goalPad);
        positionMap.put("goalBottomRight", Robotics.translate(goalPad, -0.06667*Math.cos(Math.PI/3), -0.06667*Math.sin(Math.PI/3), 0));
        positionMap.put("goalMiddleLeft", Robotics.translate(goalPad, 0.04167*Math.cos(Math.PI/3), 0.04167*Math.sin(Math.PI/3), 0.05));
        positionMap.put("goalMiddleRight", Robotics.translate(goalPad, -0.04167*Math.cos(Math.PI/3), -0.04167*Math.sin(Math.PI/3), 0.05));
        positionMap.put("goalTop", Robotics.translate(goalPad, 0, 0, 0.10));
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
        double goalTheta = -Math.PI / 6;
        double padTheta = -5 * Math.PI / 6;

        // Move top cube to goal pad
        pickAndPlace.addToTaskList(positionMap.get("startTop"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), goalTheta));
        // Move middle row to extra pad
        pickAndPlace.addToTaskList(positionMap.get("startMiddleLeft"), Robotics.rotateAboutZ(positionMap.get("padLeft"), padTheta));
        pickAndPlace.addToTaskList(positionMap.get("startMiddleRight"), Robotics.rotateAboutZ(positionMap.get("padRight"), padTheta));
        // Move top cube to extra pad
        pickAndPlace.addToTaskList(Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), goalTheta), Robotics.rotateAboutZ(positionMap.get("padMiddle"), padTheta));
        // Move bottom row to goal pad
        pickAndPlace.addToTaskList(positionMap.get("startBottomLeft"), Robotics.rotateAboutZ(positionMap.get("goalBottomLeft"), goalTheta));
        pickAndPlace.addToTaskList(positionMap.get("startBottomMiddle"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), goalTheta));
        pickAndPlace.addToTaskList(positionMap.get("startBottomRight"), Robotics.rotateAboutZ(positionMap.get("goalBottomRight"), goalTheta));
        // Move top cube to start pad
        pickAndPlace.addToTaskList(Robotics.rotateAboutZ(positionMap.get("padMiddle"), padTheta), positionMap.get("startBottomMiddle"));
        // Move remaining cubes to goal pad
        pickAndPlace.addToTaskList(Robotics.rotateAboutZ(positionMap.get("padLeft"), padTheta), Robotics.rotateAboutZ(positionMap.get("goalMiddleLeft"), goalTheta));
        pickAndPlace.addToTaskList(Robotics.rotateAboutZ(positionMap.get("padRight"), padTheta), Robotics.rotateAboutZ(positionMap.get("goalMiddleRight"), goalTheta));
        pickAndPlace.addToTaskList(positionMap.get("startBottomMiddle"), Robotics.rotateAboutZ(positionMap.get("goalTop"), goalTheta));

        return taskList;
    }

    @Override
    public List<Task> getTaskList() {
        return loadTaskList();
    }
}
