// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  // declare the configuration directory
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");

  // create a swerveDrive object but don't define it yet becasue it coomplains about not handling potential errors
  SwerveDrive swerveDrive;


  public SwerveSubsystem() {

    // Set Telemetry Verbosity (might want lower for comps as it can slow things down if it's too high, but for testing we don't care)
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // just put this code in a try/catch since java complains that there *might* be an error
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }


  // main command for driving the robot, takes in the x,y and rotation values from the controller and then scales them and passes them to the swerve drive object, also defines if it is field centric or not
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run( () -> {
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
            translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
            translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), OperatorConstants.TRANSLATION_SCALE),
        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
        OperatorConstants.FIELD_CENTRIC,
        false);
    });
  } 

  // command for zeroing the gyro, it needs disabling and re-enabling to start moving again after calling, might want to look into that
  public Command zeroGyro() {
    return run( () -> {
      swerveDrive.zeroGyro();
    });
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  
  // TODO: add remaining functions for pathplanner to get information and add pathplanner

}
