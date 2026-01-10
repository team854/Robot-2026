// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.projectile.ProjectileSubsystem;
import frc.robot.subsystems.projectile.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

	//Establishes subsystems
	public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	public static final ProjectileSubsystem projectileSubsystem = new ProjectileSubsystem();
	public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	public static final TurretSubsystem turretSubsystem = new TurretSubsystem();
	public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

	public static final CommandXboxController driverController   = new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

	//Transforms controller input into swerve drive speeds
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
        () -> driverController.getLeftY() * -1,
        () -> driverController.getLeftX() * -1)
        .withControllerRotationAxis(
            () -> driverController.getRightX() * Constants.OperatorConstants.SWERVE_ROTATION_SCALE)
        .deadband(Constants.OperatorConstants.DEADBAND)
        .scaleTranslation(Constants.OperatorConstants.SWERVE_TRANSLATION_SCALE)

        .allianceRelativeControl(true);
	
	Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
