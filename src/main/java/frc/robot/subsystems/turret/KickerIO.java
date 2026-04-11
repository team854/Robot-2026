package frc.robot.subsystems.turret;

public interface KickerIO {
    default void setMotorVoltage(double voltage) {}

    default double getMotorCurrent() {return 0;}

    default boolean checkCANError() {return false;}
}
