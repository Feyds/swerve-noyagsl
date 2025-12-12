package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;

public class Driver {
    private final XboxController controller;

    public Driver(int port) {
        this.controller = new XboxController(port);
    }

    public double getForwardSpeed() {
        return -controller.getLeftY(); 
    }

    public double getStrafeSpeed() {
        return -controller.getLeftX();
    }

    public double getRotationSpeed() {
        return -controller.getRightX();
    }

    public boolean isResetGyroButtonPressed() {
        return controller.getAButton();
    }

    public boolean isSlowMode() {
        return controller.getLeftBumper();
    }
}