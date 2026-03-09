package frc.robot.subsystems.climb;

public interface ClimbIO {
    default void setClimbMotorVoltage(double voltage) {}

    default boolean getClimbedSensor() {return false;}
}