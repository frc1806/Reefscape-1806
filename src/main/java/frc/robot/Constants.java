// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = Units.lbsToKilograms(130.839);
  //public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(7.5)), ROBOT_MASS);
  public static final Matter CHASSIS    = new Matter(new Translation3d(Units.inchesToMeters(.833), Units.inchesToMeters(.465), Units.inchesToMeters(7.77)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(15.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double PANIC_THRESHOLD_DEGREES = 45.0;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants
  {
    public static final double ELEVATOR_CASCADE_STAGES = 1.0;
    public static final double ELEVATOR_DRUM_DIAMETER = 1.0;
    public static final double ELEVATOR_GEAR_RATIO = 64.0/10.0;
    public static final double ELEVATOR_HEIGHT_TOLERANCE = 1.5; //inches
    public static final double ELEVATOR_SPEED_TOLERANCE = 10.0; //inches per second
    public static final double ELEVATOR_MIN_HEIGHT = 7.75; //TODO: SET
    public static final double ELEVATOR_START_HEIGHT = 21.0;
    public static final double ELEVATOR_MAX_HEIGHT = 50.0; //TODO: SET
    public static final double ELEVATOR_PARK_SERVO_BRAKE_ANGLE = 0.0;
    public static final double ELEVATOR_PARK_SERVO_DISENGAGE_ANGLE = 60.0;
    public static final double ELEVATOR_PARK_SERVO_ANGLE_TOLERANCE = 5.0;
    public static final double ELEVATOR_PARK_SERVO_LOCK_UNLOCK_TIME = 0.2;
    public static final double ELEVATOR_HEIGHT_FOR_SAFE_CLAW_MOVE = 25.0;
    public static final double ELEVATOR_HEIGHT_FOR_SAFE_OVER_THE_TOP = 30.0;
    public static final double CLAW_ANGLE_FOR_SAFE_ELEVATOR_MOVE = 0.0;
  }

  public static class TheClawConstants
  {
    public static final double MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR = 0.0;
    public static final double MAX_MOTION_MAX_ACCELERATION = 0.0;
    public static final double MAX_MOTION_MAX_VELOCITY = 0.0;
    public static final double MOVING_P_GAIN = 0.0;
    public static final double MOVING_I_GAIN = 0.0;
    public static final double MOVING_D_GAIN = 0.0;
    public static final double INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT = 0.0;
    public static final double INTAKE_ROLLER_STATOR_CURRENT_LIMIT = 0.0;
    public static final boolean IS_INTAKE_ROLLER_INVERTED = false;
    public static final int CLAW_ROTATION_CURRENT_LIMIT = 20;
    public static final boolean CLAW_INTAKE_ARM_INVERTED = false;
    public static final double ARM_GEAR_RATIO = (38.0/18.0) * 5.0 * 5.0 * 3.0;
    public static final double ARM_CENTER_OF_MASS_DISTANCE = Units.inchesToMeters(20.0);
    public static final double ARM_MASS = Units.lbsToKilograms(10);
    public static final double ANGLE_TOLERANCE = 1.5;
    public static final double OVER_THE_TOP_ANGLE = 160.0;

    public static final double THE_CLAW_ROLLER_IN_VOLTAGE = 12;
    public static final double THE_CLAW_ROLLER_OUT_VOLTAGE = -12;
  

  }

  public static class CoralClawConstants
  {
    public static class ClawRotationConstants
    {
      public static final double MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR = 0.0;
      public static final double MAX_MOTION_MAX_ACCELERATION = 0.0;
      public static final double MAX_MOTION_MAX_VELOCITY = 0.0;
      public static final double MOVING_P_GAIN = 0.0;
      public static final double MOVING_I_GAIN = 0.0;
      public static final double MOVING_D_GAIN = 0.0;
      public static final double ARM_GEAR_RATIO = 75.00; // 75:1
      public static final double ARM_CENTER_OF_MASS_DISTANCE = Units.inchesToMeters(Math.hypot(1.8, 2.345));
      public static final double ARM_MASS = Units.lbsToKilograms(6.358);
      public static final int CURRENT_LIMIT = 20;
    }

    public static class ClawOpenCloseConstants
    {
      public static final double MAXIMUM_ALLOWED_CLOSED_LOOP_ERROR = 0.0;
      public static final double MAX_MOTION_MAX_ACCELERATION = 0.0;
      public static final double MAX_MOTION_MAX_VELOCITY = 0.0;
      public static final double MOVING_P_GAIN = 0.0;
      public static final double MOVING_I_GAIN = 0.0;
      public static final double MOVING_D_GAIN = 0.0;
      public static final int CURRENT_LIMIT = 20;
      public static final double OPEN_ANGLE = 75;
      public static final double CLOSE_ANGLE = 70;
      public static final double ANGLE_TOLERANCE = 5.0;
    }
    public static final double THE_CLAW_ANGLE_TOLERANCE = 1.5;
    public static final double ROLLER_IN_VOLTAGE = 8.0;
    public static final double ROLLER_OUT_VOLTAGE = -4.0;
  }
}
