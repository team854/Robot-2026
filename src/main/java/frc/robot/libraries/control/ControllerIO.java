package frc.robot.libraries.control;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public interface ControllerIO {
    default double getLeftX() {return 0;}
    default double getLeftY() {return 0;}

    default double getRightX() {return 0;}
    default double getRightY() {return 0;}

    default Trigger leftStick() {return new Trigger(() -> false);}
    default Trigger rightStick() {return new Trigger(() -> false);}

    default Trigger leftTrigger() {return new Trigger(() -> false);}
    default Trigger rightTrigger() {return new Trigger(() -> false);}

    default Trigger leftBumper() {return new Trigger(() -> false);}
    default Trigger rightBumper() {return new Trigger(() -> false);}

    default Trigger upButton() {return new Trigger(() -> false);}
    default Trigger rightButton() {return new Trigger(() -> false);}
    default Trigger downButton() {return new Trigger(() -> false);}
    default Trigger leftButton() {return new Trigger(() -> false);}

    default Trigger povUpDirection() {return new Trigger(() -> false);}
    default Trigger povRightDirection() {return new Trigger(() -> false);}
    default Trigger povDownDirection() {return new Trigger(() -> false);}
    default Trigger povLeftDirection() {return new Trigger(() -> false);}

    default Trigger menuButton() {return new Trigger(() -> false);}
    default Trigger backButton() {return new Trigger(() -> false);}

    default double[] curveStick(double axisX, double axisY) {
        double rawMagnitude = Math.hypot(axisX, axisY);
        double magnitude = Math.min(1.0, rawMagnitude);

        if (magnitude <= Constants.OperatorConstants.DEADBAND) {
            return new double[] {0, 0};
        }

        double mappedMagnitude = (magnitude - Constants.OperatorConstants.DEADBAND) / (1.0 - Constants.OperatorConstants.DEADBAND);

        double curvedMagnitude = Math.abs(Math.pow(mappedMagnitude, Constants.OperatorConstants.SWERVE_EXPONENT));

        return new double[] {(axisY / rawMagnitude) * curvedMagnitude, (axisX / rawMagnitude) * curvedMagnitude};
    }

    default double curveAxis(double axis) {
        if (Math.abs(axis) <= Constants.OperatorConstants.DEADBAND) {
                return 0;
            }

            double mappedTurn = (Math.abs(axis) - Constants.OperatorConstants.DEADBAND) / (1.0 - Constants.OperatorConstants.DEADBAND);

            double curvedTurn = Math.abs(Math.pow(mappedTurn, Constants.OperatorConstants.SWERVE_EXPONENT));

            return curvedTurn * Math.signum(axis);
    }

    default DoubleSupplier leftXCombinedSupplier() {
        return () -> {
            return curveStick(getLeftX(), getLeftY())[0];
        };
    }

    default DoubleSupplier leftYCombinedSupplier() {
        return () -> {
            return curveStick(getLeftX(), getLeftY())[1];
        };
    }

    default DoubleSupplier rightXCombinedSupplier() {
        return () -> {
            return curveStick(getRightX(), getRightY())[0];
        };
    }

    default DoubleSupplier rightYCombinedSupplier() {
        return () -> {
            return curveStick(getRightX(), getRightY())[1];
        };
    }

    default DoubleSupplier leftXSupplier() {
        return () -> {
            return curveAxis(getLeftX());
        };
    }

    default DoubleSupplier leftYSupplier() {
        return () -> {
            return curveAxis(getLeftY());
        };
    }

    default DoubleSupplier rightXSupplier() {
        return () -> {
            return curveAxis(getRightX());
        };
    }

    default DoubleSupplier rightYSupplier() {
        return () -> {
            return curveAxis(getRightY());
        };
    }
}
