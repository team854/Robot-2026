// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.libraries.ProjectileSimulation;
import frc.robot.libraries.ProjectileSimulation.TargetErrorCode;
import frc.robot.libraries.ProjectileSimulation.TargetSolution;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.turret.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

	// Establishes subsystems
	public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	public static final TurretSubsystem turretSubsystem = new TurretSubsystem();
	public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
	public static final ProjectileSimulation projectileSimulation = new ProjectileSimulation();

	public static final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
	

	// Transforms controller input into swerve drive speeds
	static SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
        () -> driverController.getLeftY() * -1,
        () -> driverController.getLeftX() * -1)
        .withControllerRotationAxis(
            () -> driverController.getRightX() * Constants.OperatorConstants.SWERVE_ROTATION_SCALE)
        .deadband(Constants.OperatorConstants.DEADBAND)
        .scaleTranslation(Constants.OperatorConstants.SWERVE_TRANSLATION_SCALE)

        .allianceRelativeControl(true);
	
	public static final Command driveFieldOrientedAngularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
	public static final Command turretAutoAimCommand = new TurretAutoAimCommand();
	

	public RobotContainer() {
		configureBindings();

		//DO SANITY CHECKS OF THE MAGNUS EFFECT
		/*
		if (false) {
			TargetSolution solution = projectileSimulation.calculateLaunchAngleSimulation(
				projectileSimulation.convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5),
				DegreesPerSecond.of(0),
				new Translation2d(0, 2),
				new Translation3d(4.4,0,1.9558),
				Constants.FuelPhysicsConstants.MAX_STEPS,
				Constants.FuelPhysicsConstants.TPS
				
			);
			System.out.println(10.09999999999998 + " " + solution.toString());
		} else {
			for (double px = 2; px < 14; px+=0.1) {
				TargetSolution solution = projectileSimulation.calculateLaunchAngleSimulation(
					projectileSimulation.convertShooterSpeedToVelocity(Constants.ShooterConstants.SHOOTER_MAX_VELOCITY, Constants.ShooterConstants.SHOOTER_WHEEL_RADIUS, 0.5),
					DegreesPerSecond.of(0),
					new Translation2d(0, 0),
					new Translation3d(px,0,1.9558),
					Constants.FuelPhysicsConstants.MAX_STEPS,
					Constants.FuelPhysicsConstants.TPS
					
				);
				System.out.println(px + " " + solution.toString());
				
			}
			System.out.println("FEWF");
			
		}
		*/
		
	}

	private void configureBindings() {
		swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
		turretSubsystem.setDefaultCommand(turretAutoAimCommand);


	}

	public static boolean isBlueAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() != DriverStation.Alliance.Red : false;
    }

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
