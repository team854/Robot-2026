// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.projectile.ProjectileSubsystem;
import frc.robot.subsystems.projectile.ProjectileSubsystem.TargetSolution;
import frc.robot.subsystems.turret.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

	// Establishes subsystems
	public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	public static final ProjectileSubsystem projectileSubsystem = new ProjectileSubsystem();
	public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	public static final TurretSubsystem turretSubsystem = new TurretSubsystem();
	public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

	public static final CommandXboxController driverController   = new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

	// Transforms controller input into swerve drive speeds
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

		//DO SANITY CHECKS OF THE MAGNUS EFFECT

		
		TargetSolution solution = projectileSubsystem.calculateLaunchAngleSimulation(
			MetersPerSecond.of(15),
			DegreesPerSecond.of(0),
			DegreesPerSecond.of(0),
			new Translation2d(0, 0),
			new Translation3d(2,0,1.9558),
			500,
			50
			
		);
		System.out.println(solution);
		System.out.println(solution.launchPitch().in(Degree)); // 56.08282080939181
		
		
	}

	private void configureBindings() {
		swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
