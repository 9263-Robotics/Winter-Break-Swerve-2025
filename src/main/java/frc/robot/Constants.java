// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Joystick Deadbands
    public static final double  DEADBAND           = 0.05;

    // Scales for movement and rotation (1 is full speed)
    public static final double  TRANSLATION_SCALE  = 0.5;
    public static final double  ROTATION_SCALE     = 0.6;

    // if we want field centric control or not (is forwards on the controller always away from  the driver station or is it forwards for the robot)
    public static final boolean FIELD_CENTRIC      = true;
  }


  // max speed of the robot in m/s use Units.feetToMeters to use feet
  public static final   double  MAX_SPEED = 4.5;

  
}
