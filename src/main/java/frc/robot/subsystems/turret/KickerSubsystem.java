package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMaxConfig kickerConfig = new SparkMaxConfig();
    private final SparkMax kickerMotor = new SparkMax(Constants.KickerConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

    public KickerSubsystem() {
        kickerConfig.idleMode(IdleMode.kCoast);
        kickerConfig.inverted(Constants.KickerConstants.KICKER_MOTOR_INVERTED);
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setThrottle(double throttle) {
        kickerMotor.set(throttle);
    }
}
