package frc.robot.subsystems.turret;

public interface TurretIO {
    default void setYawMotorVoltage(double voltage) {}
    default void setPitchMotorVoltage(double voltage) {}

    default double getYawRadians() {return 0;}
    default double getPitchRadians() {return 0;}
}
