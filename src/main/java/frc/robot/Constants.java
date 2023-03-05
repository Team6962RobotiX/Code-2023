// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Enabled Systems
  public static final boolean ENABLE_DRIVE = true;
  public static final boolean ENABLE_BALANCE = true;
  public static final boolean ENABLE_ARM = true;
  public static final boolean ENABLE_CLAW = true;
  public static final boolean ENABLE_PNEUMATIC_CLAW = false;
  public static final boolean ENABLE_LIMELIGHT = false;


  // Drive Config
  public static final double DRIVE_POWER_LIMIT = 0.4; // Hard limit on power
  public static final double DRIVE_BASE_POWER = 0; // Motor power required to get the chassis moving
  public static final double DRIVE_TRACK_WIDTH = 0.5588; // Meters
  public static final double WHEEL_RADIUS = 8 / 100; // Meters
  public static final double GEARBOX_RATIO = 1 / 8.45;
  public static final double DRIVE_DISTANCE_PER_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS * GEARBOX_RATIO;
  public static final double DRIVE_KP = 0;
  public static final double DRIVE_KS = 0;
  public static final double DRIVE_KV = 0;
  public static final double DRIVE_KA = 0;
  public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(DRIVE_TRACK_WIDTH);

  // Joystick Dead-zones
  public static final double TWIST_DEADZONE = 0.2; // Joystick deadzone for turning
  public static final double STRAIGHT_DEADZONE = 0.1; // Joystick deadzone for turning
  public static final double THROTTLE_DEADZONE = 0.1; // Joystick deadzone for arm lifting


  // Channels
  public static final int CAN_LEFT_DRIVE_1 = 10;
  public static final int CAN_LEFT_DRIVE_2 = 28;
  public static final int CAN_RIGHT_DRIVE_1 = 7;
  public static final int CAN_RIGHT_DRIVE_2 = 27;
  public static final int CAN_ARM_LIFT_1 = 5;
  public static final int CAN_ARM_LIFT_2 = 15;
  public static final int CAN_ARM_EXTEND = 13;
  public static final int CAN_CLAW_GRAB = 16;

  public static final int DIO_ARM_LIFT_ENCODER = 0;
  public static final int DIO_CLAW_GRAB_MICRO_SWITCH = 1;

  public static final int USB_DRIVE_JOYSTICK = 0;
  public static final int USB_UTILITY_JOYSTICK = 1;


  // Balance Config
  public static final double BALANCE_LEVEL_ANGLE_RANGE = 2.5; // Angle range required to declare leveled
  public static final double BALANCE_ANGLE_POWER_MULTIPLE = 2; // Motor power multiple based on current angle
  public static final double BALANCE_BASE_POWER = 0.25; // Base balancing speed


  // Arm Positioning
  public static final double ARM_EXTEND_INCHES = 70; // Inches from pivot when fully extended
  public static final double ARM_RETRACT_INCHES = 40; // Inches from pivot when fully retracted
  public static final double ARM_HEIGHT_INCHES = 40; // Inches above ground from pivot
  public static final double ARM_LIFT_ENCODER_OFFSET = 242; // Offset so encoder reads 90 degrees when parallel to ground


  // Extension
  public static final double ARM_EXTEND_LIMIT = 36; // Arm extend limit (measured in encoder ticks)
  public static final double ARM_EXTEND_PADDING = 0.2; // Padding to prevent overshooting limits (measured in percent 0 - 1)
  public static final double ARM_EXTEND_POWER = 0.2; // Slowest speed arm will extend (0 - 1)
  public static final double ARM_EXTEND_MAX_POWER = 0.3; // Fastest speed arm will extend (0 - 1)


  // Lifting
  public static final double ARM_LIFT_MAX_POWER = 0.15; // Max arm lifting power
  public static final double ARM_LIFT_POWER_INCREMENT = 0.005; // Arm lifting power increment each tick
  public static final double ARM_LIFT_MIN_ANGLE = 28; // Min arm angle (degrees)
  public static final double ARM_LIFT_MAX_ANGLE = 118; // Max arm angle (degrees)
  public static final double ARM_LIFT_ANGLE_PRECISION = 1; // Degrees of precision


  // PID & FF Config
  public static final double ARM_EXTEND_KP = 0;
  public static final double ARM_EXTEND_KI = 0;
  public static final double ARM_EXTEND_KD = 0;
  
  // PID
  public static final double ARM_LIFT_KP = 0.02;
  public static final double ARM_LIFT_KI = 0;
  public static final double ARM_LIFT_KD = 0;

  // FF
  public static final double ARM_LIFT_KS = 0;
  public static final double ARM_LIFT_KG = 0;
  public static final double ARM_LIFT_KV = 0;


  // Claw Config
  public static final double CLAW_GRAB_LIMIT = -200; // Claw grab limit (measured in encoder ticks)
  public static final double CLAW_GRAB_PADDING = 0.2; // Padding to prevent overshooting limits (measured in percent 0 - 1)
  public static final double CLAW_GRAB_MIN_POWER = 0.05; // Slowest speed claw will grab (0 - 1)
  public static final double CLAW_GRAB_MAX_POWER = 0.4; // Fastest speed claw will grab (0 - 1)

  // Autonomous Config
  public static final double AUTONOMOUS_MAX_SPEED = 1; // m / s
  public static final double AUTONOMOUS_MAX_ACCELERATION = 1; // m / s^2
  public static final double AUTONOMOUS_RAMSETE_B = 0;
  public static final double AUTONOMOUS_RAMSETE_ZETA = 0;

  public static double mapPower(double power, double min, double max, double deadZone) {
    double sign = Math.signum(power);
    double absPower = Math.abs(power);

    if (absPower < deadZone) {
      return 0.0;
    } else {
      return mapNumber(absPower, deadZone, 1, min, max) * sign;
    }
  }

  public static double mapLimitedPower(int direction, double pos, double minPos, double maxPos, double minPower, double maxPower, double padding) {
    if (direction > 0) {
      if (pos > maxPos) {
        return 0.0;
      }
      if (pos > maxPos - padding) {
        return mapNumber(pos, maxPos - padding, maxPos, maxPower, minPower);
      } else {
        return maxPower;
      }
    } else if (direction < 0) {
      if (pos < minPos) {
        return 0.0;
      }
      if (pos < minPos + padding) {
        return -mapNumber(pos, minPos, minPos + padding, minPower, maxPower);
      } else {
        return -maxPower;
      }
    } else {
      return 0.0;
    }
  }

  public static double mapNumber(double x, double a, double b, double c, double d) {
    if (x < a) {
      return c;
    }
    if (x > b) {
      return d;
    }
    return (x - a) / (b - a) * (d - c) + c;
  }
}
