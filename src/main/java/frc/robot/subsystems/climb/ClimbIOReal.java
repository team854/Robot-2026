package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ClimbIOReal implements ClimbIO {
    private final SparkMax climbMotor = new SparkMax(Constants.ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput climbedSensor;

    public ClimbIOReal() {
        climbedSensor = new DigitalInput(Constants.ClimbConstants.CLIMB_SENSOR_DIO);
    }

    @Override
    public void setClimbMotorVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }

    @Override
    public boolean getClimbedSensor() {
        return !climbedSensor.get();
    }
}
