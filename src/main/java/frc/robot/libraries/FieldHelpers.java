package frc.robot.libraries;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public final class FieldHelpers {
    public static Translation2d rotateBlueFieldCoordinates(Translation2d position) {
        double midpointX = (Constants.FieldConstants.FIELD_SIZE_X.in(Meter) / 2.0);
        double midpointY = (Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0);
        return new Translation2d((midpointX - position.getX()) + midpointX, (midpointY - position.getY()) + midpointY);
    }

    public static Translation3d rotateBlueFieldCoordinates(Translation3d position) {
        double midpointX = (Constants.FieldConstants.FIELD_SIZE_X.in(Meter) / 2.0);
        double midpointY = (Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0);
        return new Translation3d((midpointX - position.getX()) + midpointX, (midpointY - position.getY()) + midpointY, position.getZ());
    }

    public static Translation2d rotateBlueFieldCoordinates(Translation2d position, boolean rotate) {
        if (rotate) {
            double midpointX = (Constants.FieldConstants.FIELD_SIZE_X.in(Meter) / 2.0);
            double midpointY = (Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0);
            return new Translation2d((midpointX - position.getX()) + midpointX, (midpointY - position.getY()) + midpointY);
        } else {
            return position;
        }
    }

    public static Translation3d rotateBlueFieldCoordinates(Translation3d position, boolean rotate) {
        if (rotate) {
            double midpointX = (Constants.FieldConstants.FIELD_SIZE_X.in(Meter) / 2.0);
            double midpointY = (Constants.FieldConstants.FIELD_SIZE_Y.in(Meter) / 2.0);
            return new Translation3d((midpointX - position.getX()) + midpointX, (midpointY - position.getY()) + midpointY, position.getZ());
        } else {
            return position;
        }
    }
}
