import libraries.Robotics;
import model.Cube;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class HanoiTower implements TaskList {

    private final Map<String, double[][]> positionMap;
    private List<Cube> taskList;

    private final double[][] startPad = {{1,0,0,0.75},{0,1,0,0},{0,0,1,0.06},{0,0,0,1}};
    private final double[][] goalPad = {{0,1,0,-0.75*Math.sin(Math.PI/6)},{-1,0,0,-0.75*Math.cos(Math.PI/6)},{0,0,1,0.06},{0,0,0,1}};
    private final double[][] extraPad = {{0,1,0,-0.75*Math.sin(Math.PI/6)},{-1,0,0,0.75*Math.cos(Math.PI/6)},{0,0,1,0.06},{0,0,0,1}};

    public HanoiTower() {
        this.positionMap = loadCubePositions();
        this.taskList = loadTaskList();
    }

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

    private List<Cube> loadTaskList() {
        taskList = new ArrayList<>();
        double goalTheta = -Math.PI / 6;
        double padTheta = -5 * Math.PI / 6;

        // Move top cube to goal pad
        taskList.add(new Cube(positionMap.get("startTop"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), goalTheta)));
        // Move middle row to extra pad
        taskList.add(new Cube(positionMap.get("startMiddleLeft"), Robotics.rotateAboutZ(positionMap.get("padLeft"), padTheta)));

        taskList.add(new Cube(positionMap.get("startMiddleRight"), Robotics.rotateAboutZ(positionMap.get("padRight"), padTheta)));
        // Move top cube to extra pad
        taskList.add(new Cube(Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), goalTheta), Robotics.rotateAboutZ(positionMap.get("padMiddle"), padTheta)));
        // Move bottom row to goal pad
        taskList.add(new Cube(positionMap.get("startBottomLeft"), Robotics.rotateAboutZ(positionMap.get("goalBottomLeft"), goalTheta)));
        taskList.add(new Cube(positionMap.get("startBottomMiddle"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), goalTheta)));
        taskList.add(new Cube(positionMap.get("startBottomRight"), Robotics.rotateAboutZ(positionMap.get("goalBottomRight"), goalTheta)));
        // Move top cube to start pad
        taskList.add(new Cube(Robotics.rotateAboutZ(positionMap.get("padMiddle"), padTheta), positionMap.get("startBottomMiddle")));
        // Move remaining cubes to goal pad
        taskList.add(new Cube(Robotics.rotateAboutZ(positionMap.get("padLeft"), padTheta), Robotics.rotateAboutZ(positionMap.get("goalMiddleLeft"), goalTheta)));
        taskList.add(new Cube(Robotics.rotateAboutZ(positionMap.get("padRight"), padTheta), Robotics.rotateAboutZ(positionMap.get("goalMiddleRight"), goalTheta)));
        taskList.add(new Cube(positionMap.get("startBottomMiddle"), Robotics.rotateAboutZ(positionMap.get("goalTop"), goalTheta)));

        return taskList;
    }

    @Override
    public List<Cube> getTaskList() {
        return loadTaskList();
    }
}
