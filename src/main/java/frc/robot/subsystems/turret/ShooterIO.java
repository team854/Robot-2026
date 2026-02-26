package frc.robot.subsystems.turret;

public interface ShooterIO {
    default void setMotorsVoltage(double voltage) {}

    default double getMotor1RPS() {return 0;}
    default double getMotor2RPS() {return 0;}
}
