package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
    private SparkMaxConfig kickerConfig;
    private SparkMax kickerMotor;

    public KickerSubsystem() {
        if (Constants.KickerConstants.ENABLED) {
            kickerMotor = new SparkMax(Constants.KickerConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

            kickerConfig = new SparkMaxConfig();
            kickerConfig.idleMode(IdleMode.kCoast);
            kickerConfig.inverted(Constants.KickerConstants.KICKER_MOTOR_INVERTED);
            kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public void setThrottle(double throttle) {
        if (Constants.KickerConstants.ENABLED == false) {
            return;
        }

        kickerMotor.set(throttle);
    }
}
