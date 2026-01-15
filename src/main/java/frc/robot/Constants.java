// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds
    // of the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    // CANCoder CAN IDs
    public static final int kFrontLeftTurningEncoderCanId = 10;
    public static final int kRearLeftTurningEncoderCanId = 12;
    public static final int kFrontRightTurningEncoderCanId = 14;
    public static final int kRearRightTurningEncoderCanId = 16;

    public static final RobotConfig kRobotConfig;

    static {
      try {
        kRobotConfig = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        throw new RuntimeException("Failed to load robot configuration", e);
      }
    }
  }

  public static final class ModuleConstants {
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDrivingMotorReduction = 22.0;
    public static final double kDriveWheelFreeSpeedRps = (NeoMotorConstants.kFreeSpeedRps
        * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kTurningMotorReduction = 22.0;

    // TODO: Check motor type
    public static final double kMaxSteerSpeedRadPerSec = 0.9
        * Units.rotationsToRadians(NeoMotorConstants.kFreeSpeedRps / kTurningMotorReduction);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final PIDConstants kTranslationConstants = new PIDConstants(1, 0, 0);
    public static final PIDConstants kRotationConstants = new PIDConstants(1, 0, 0);

    public static final PathFollowingController kPathFollowingController = new PPHolonomicDriveController(
        kTranslationConstants, kRotationConstants);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
    public static final double kFreeSpeedRps = kFreeSpeedRpm / 60;
  }
}
