// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // create a new swerve subsystem object
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  // create an object for our driver controller
  private final CommandPS5Controller driverController = new CommandPS5Controller(Constants.OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // set the default command for the drivebase to the drive command
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(OperatorConstants.TRANSLATION_SCALE)
                                                                .scaleRotation(OperatorConstants.ROTATION_SCALE)
                                                                .allianceRelativeControl(true);

  // For the right stick to correspond to the angle we want the robot to face instead of the speed of rotationa
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                                                                                             driverController::getRightY)
                                                                                             .headingWhile(true);
  
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveFieldOrientedDirectAngle     = drivebase.driveFieldOriented(driveDirectAngle);
  // create a new command that calls the driveCommand that we made in the swerveSubsystem, take in the joystick sticks with dead band and scale the rotation
  // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
  //     () -> MathUtil.applyDeadband(driverController.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
  //     () -> MathUtil.applyDeadband(driverController.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
  //     () -> driverController.getRightX() * -OperatorConstants.ROTATION_SCALE);

  // define what buttons do on the controller
  private void configureBindings() {
    driverController.button(1).onTrue(drivebase.zeroGyro()); //zero the gyro when square(?) is pressed
  }
}
