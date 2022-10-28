import libraries.Robotics;
import model.Cube;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class HanoiTower implements TaskList {

    private final Map<String, double[][]> positionMap;
    private List<Cube> taskList;

    public HanoiTower() {
        this.positionMap = loadCubePositions();
        this.taskList = loadTaskList();
    }

    private Map<String, double[][]> loadCubePositions() {
        Map<String, double[][]> positionMap = new TreeMap<>();
        // Start Pad
        positionMap.put("startBottomLeft", new double[][] {{1,0,0,0.75},{0,1,0,0.06667},{0,0,1,0.05},{0,0,0,1}});
        positionMap.put("startBottomMiddle", new double[][] {{1,0,0,0.75},{0,1,0,0},{0,0,1,0.05},{0,0,0,1}});
        positionMap.put("startBottomRight", new double[][] {{1,0,0,0.75},{0,1,0,-0.06667},{0,0,1,0.05},{0,0,0,1}});
        positionMap.put("startMiddleLeft", new double[][] {{1,0,0,0.75},{0,1,0,0.04167},{0,0,1,0.10},{0,0,0,1}});
        positionMap.put("startMiddleRight", new double[][] {{1,0,0,0.75},{0,1,0,-0.04167},{0,0,1,0.10},{0,0,0,1}});
        positionMap.put("startTop", new double[][] {{1,0,0,0.75},{0,1,0,0},{0,0,1,0.15},{0,0,0,1}});
        // YouBot Base
        // Extra Pad
        positionMap.put("padMiddle", new double[][] {{0,1,0,-0.75*Math.cos(Math.PI/3)},{-1,0,0,0.75*Math.sin(Math.PI/3)},{0,0,1,0.025},{0,0,0,1}});
        // Goal Pad
        positionMap.put("goalBottomMiddle", new double[][] {{0,1,0,-0.75*Math.cos(Math.PI/3)},{-1,0,0,-0.75*Math.sin(Math.PI/3)},{0,0,1,0.025},{0,0,0,1}});
        return positionMap;
    }

    private List<Cube> loadTaskList() {
        taskList = new ArrayList<>();
        taskList.add(new Cube(positionMap.get("startTop"), Robotics.rotateAboutZ(positionMap.get("padMiddle"),-5*Math.PI/6)));
        taskList.add(new Cube(positionMap.get("startMiddleLeft"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), -Math.PI/6)));
        taskList.add(new Cube(positionMap.get("startMiddleRight"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), -Math.PI/6)));
        taskList.add(new Cube(positionMap.get("startBottomLeft"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), -Math.PI/6)));
        taskList.add(new Cube(positionMap.get("startBottomMiddle"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), -Math.PI/6)));
        taskList.add(new Cube(positionMap.get("startBottomRight"), Robotics.rotateAboutZ(positionMap.get("goalBottomMiddle"), -Math.PI/6)));
        return taskList;
    }

    @Override
    public List<Cube> getTaskList() {
        return loadTaskList();
    }
}
