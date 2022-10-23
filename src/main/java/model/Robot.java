package model;

public class Robot {

    private double[] config;
    private double[] controls;
    private final double[][] chassisKinematicModel;

    public Robot(double[] config, double[] controls, double[][] chassisKinematicModel) {
        this.config = config;
        this.controls = controls;
        this.chassisKinematicModel = chassisKinematicModel;
    }

    public double[] getConfig() {
        return config;
    }

    public void setConfig(double[] config) {
        this.config = config;
    }

    public double[] getControls() {
        return controls;
    }

    public void setControls(double[] controls) {
        this.controls = controls;
    }

    public double[][] getChassisKinematicModel() {
        return chassisKinematicModel;
    }

    public static void limitSpeeds(double[] controls, double maxSpeed) {
        for (int i=0; i < controls.length; i++) {
            if (controls[i] > maxSpeed) {
                controls[i] = maxSpeed;
            } else if (controls[i] < -maxSpeed) {
                controls[i] = -maxSpeed;
            }
        }
    }
}
