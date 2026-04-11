package frc.robot.libraries.control;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerIOXbox implements ControllerIO {
    private final int port;
    private final CommandXboxController xboxController; 

    public ControllerIOXbox(int port) {
        this.port = port;
        this.xboxController = new CommandXboxController(port);
    }

    @Override
    public double getLeftX() {
        return -this.xboxController.getLeftX();
    }

    @Override
    public double getLeftY() {
        return -this.xboxController.getLeftY();
    }

    @Override
    public double getRightX() {
        return -this.xboxController.getRightX();
    }

    @Override
    public double getRightY() {
        return -this.xboxController.getRightY();
    }

    @Override
    public Trigger leftStick() {
        return this.xboxController.leftStick();
    }

    @Override
    public Trigger rightStick() {
        return this.xboxController.rightStick();
    }

    @Override
    public Trigger leftTrigger() {
        return this.xboxController.leftTrigger();
    }

    @Override
    public Trigger rightTrigger() {
        return this.xboxController.rightTrigger();
    }

    @Override
    public Trigger leftBumper() {
        return this.xboxController.leftBumper();
    }

    @Override
    public Trigger rightBumper() {
        return this.xboxController.rightBumper();
    }

    @Override
    public Trigger upButton() {
        return this.xboxController.y();
    }

    @Override
    public Trigger rightButton() {
        return this.xboxController.b();
    }

    @Override
    public Trigger downButton() {
        return this.xboxController.a();
    }

    @Override
    public Trigger leftButton() {
        return this.xboxController.x();
    }

    @Override
    public Trigger povUpDirection() {
        return this.xboxController.povUp();
    }

    @Override
    public Trigger povRightDirection() {
        return this.xboxController.povRight();
    }

    @Override
    public Trigger povDownDirection() {
        return this.xboxController.povDown();
    }

    @Override
    public Trigger povLeftDirection() {
        return this.xboxController.povLeft();
    }

    @Override
    public Trigger menuButton() {
        return this.xboxController.start();
    }

    @Override
    public Trigger backButton() {
        return this.xboxController.back();
    }
}
