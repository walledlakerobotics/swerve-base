// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.CANIDs;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds
    // of the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.47;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Odometry measurement standard deviations
    public static final Matrix<N4, N1> kOdometryStdDevs =
        VecBuilder.fill(0.001, 0.001, 0.001, 0.01);

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.125);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = CANIDs.frontLeftDrive(); // 8
    public static final int kRearLeftDrivingCanId = CANIDs.rearLeftDrive(); // 6
    public static final int kFrontRightDrivingCanId = CANIDs.frontRightDrive(); // 2
    public static final int kRearRightDrivingCanId = CANIDs.rearRightDrive(); // 4

    public static final int kFrontLeftTurningCanId = CANIDs.frontLeftTurning(); // 7
    public static final int kRearLeftTurningCanId = CANIDs.rearLeftTurning(); // 5
    public static final int kFrontRightTurningCanId = CANIDs.frontRightTurning(); // 1
    public static final int kRearRightTurningCanId = CANIDs.rearRightTurning(); // 3

    // Encoder CAN IDs
    public static final int kFrontLeftTurningEncoderId = CANIDs.frontLeftEncoder(); // 14
    public static final int kRearLeftTurningEncoderId = CANIDs.rearLeftEncoder(); // 13
    public static final int kFrontRightTurningEncoderId = CANIDs.frontRightEncoder(); // 12
    public static final int kRearRightTurningEncoderId = CANIDs.rearRightEncoder(); // 11

    public static final SwerveSetpointGenerator setpointGenerator =
        new SwerveSetpointGenerator(
            DriveConstants.kRobotConfig, ModuleConstants.kMaxSteerSpeedRadPerSec);

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
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDrivingMotorReduction = 6.75;
    public static final double kDriveFreeSpeedMetersPerSecond =
        (NeoMotorConstants.kFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kTurningMotorReduction = 150.0 / 7;

    public static final double kMaxSteerSpeedRadPerSec =
        0.9 * Units.rotationsToRadians(NeoMotorConstants.kFreeSpeedRps / kTurningMotorReduction);

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 35; // amps

    public static final boolean kDrivingMotorsInverted = true;
    public static final boolean kTurningMotorsInverted = true;
    public static final boolean kTurningEncoderInverted = false;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final PIDConstants kTranslationConstants = new PIDConstants(1, 0, 0);
    public static final PIDConstants kRotationConstants = new PIDConstants(1, 0, 0);

    public static final PathFollowingController kPathFollowingController =
        new PPHolonomicDriveController(kTranslationConstants, kRotationConstants);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
    public static final double kFreeSpeedRps = kFreeSpeedRpm / 60;
  }

  public static final class VisionConstants {
    public static final String[] kCameraNames = {};
    public static final Transform3d[] kRobotToCameraTransforms = {};
    public static final List<Matrix<N4, N1>> kVisionMeasurementStdDevs = List.of();

    public static final AprilTagFieldLayout kAprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  }
}
