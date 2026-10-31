package frc.robot.subsystems.intake;

public interface IntakeDeploymentIO {
    default void setDeploymentMotor1Voltage(double voltage) {}
    default void setDeploymentMotor2Voltage(double voltage) {}

    default boolean getRetractedSensor() {return true;}
    default boolean getDeployedSensor() {return false;}

    default boolean checkCANError() {return false;}
}
