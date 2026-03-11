package frc.robot.libraries.control;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerIOPS5 implements ControllerIO {
    private final int port;
    private final CommandPS5Controller ps5Controller; 

    public ControllerIOPS5(int port) {
        this.port = port;
        this.ps5Controller = new CommandPS5Controller(port);
    }

    @Override
    public double getLeftX() {
        return -this.ps5Controller.getLeftX();
    }

    @Override
    public double getLeftY() {
        return -this.ps5Controller.getLeftY();
    }

    @Override
    public double getRightX() {
        return -this.ps5Controller.getRightX();
    }

    @Override
    public double getRightY() {
        return -this.ps5Controller.getRightY();
    }

    @Override
    public Trigger leftTrigger() {
        return this.ps5Controller.L2();
    }

    @Override
    public Trigger rightTrigger() {
        return this.ps5Controller.R2();
    }

    @Override
    public Trigger leftBumper() {
        return this.ps5Controller.L1();
    }

    @Override
    public Trigger rightBumper() {
        return this.ps5Controller.R1();
    }

    @Override
    public Trigger upButton() {
        return this.ps5Controller.triangle();
    }

    @Override
    public Trigger rightButton() {
        return this.ps5Controller.circle();
    }

    @Override
    public Trigger downButton() {
        return this.ps5Controller.cross();
    }

    @Override
    public Trigger leftButton() {
        return this.ps5Controller.square();
    }

    @Override
    public Trigger povUpDirection() {
        return this.ps5Controller.povUp();
    }

    @Override
    public Trigger povRightDirection() {
        return this.ps5Controller.povRight();
    }

    @Override
    public Trigger povDownDirection() {
        return this.ps5Controller.povDown();
    }

    @Override
    public Trigger povLeftDirection() {
        return this.ps5Controller.povLeft();
    }

    @Override
    public Trigger menuButton() {
        return this.ps5Controller.options();
    }

    @Override
    public Trigger backButton() {
        return this.ps5Controller.create();
    }
}

