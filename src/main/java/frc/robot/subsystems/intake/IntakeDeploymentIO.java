package frc.robot.subsystems.intake;

public interface IntakeDeploymentIO {
    default void setDeploymentMotorVoltage(double voltage) {}

    default boolean getRetractedSensor() {return true;}
    default boolean getDeployedSensor() {return false;}
}
