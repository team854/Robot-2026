package frc.robot.subsystems.turret;

public interface TurretIO {
    default void setYawMotorVoltage(double voltage) {}
    default void setPitchMotorVoltage(double voltage) {}

    default double getYawRadians() {return 0;}
    default double getPitchRadians() {return 0;}

    default double getRawYawRadians() {return 0;}
    default double getRawAbsoluteYawRadians() {return 0;}

    default void setYawEncoderPosition(double position) {}

    default boolean getHomingSensor() {return false;}
    default boolean getHomingCounter() {return false;}
    default void resetHomingCounter() {}

    default void resetPitchPosition() {}

    default boolean checkCANError() {return false;}
}
