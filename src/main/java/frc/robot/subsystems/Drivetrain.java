// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import java.util.function.DoubleSupplier;

/** Subsystem to control a swerve drivetrain. */
public class Drivetrain extends SubsystemBase {
  // Create SwerveModules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftTurningEncoderId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightTurningEncoderId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kRearLeftTurningEncoderId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kRearRightTurningEncoderId,
          DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Pose estimator class for tracking robot pose
  private final SwerveDrivePoseEstimator3d m_odometry =
      new SwerveDrivePoseEstimator3d(
          DriveConstants.kDriveKinematics,
          getGyroRotation3d(),
          getModulePositions(),
          Pose3d.kZero,
          DriveConstants.kOdometryStdDevs,
          new Matrix<>(Nat.N4(), Nat.N1()));

  // Vision class for managing PhotonVision cameras and pose estimates
  private final Vision m_vision = new Vision(m_odometry);

  // Setpoint for physically possible setpoint transitions
  private SwerveSetpoint m_previousSetpoint =
      new SwerveSetpoint(
          getChassisSpeeds(),
          getModuleStates(),
          DriveFeedforwards.zeros(DriveConstants.kRobotConfig.numModules));

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Report swerve drive to the HAL
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_Other);

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        this::drive,
        AutoConstants.kPathFollowingController,
        DriveConstants.kRobotConfig,
        () -> false,
        this);

    RobotModeTriggers.disabled()
        .and(() -> !DriverStation.isFMSAttached())
        .onTrue(setIdleMode(IdleMode.kCoast))
        .onFalse(setIdleMode(IdleMode.kBrake));
  }

  @Override
  public void periodic() {
    // Update vision and odometry in the periodic block
    m_vision.update();
    m_odometry.update(getGyroRotation3d(), getModulePositions());
  }

  /**
   * Gets the currently-estimated 3D pose of the robot.
   *
   * @return The current 3D pose of the robot.
   */
  public Pose3d getPose3d() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Gets the currently-estimated pose of the robot.
   *
   * @return The current pose of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition().toPose2d();
  }

  /**
   * Gets the current {@link ChassisSpeeds} of the robot.
   *
   * @return The current ChassisSpeeds of the robot.
   */
  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassisSpeeds =
        DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());

    chassisSpeeds.omegaRadiansPerSecond = getTurnRate().getRadians();
    return chassisSpeeds;
  }

  /**
   * Gets the current states of all the swerve modules.
   *
   * @return The current SwerveModuleStates.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
    };
  }

  /**
   * Gets the current positions of all the swerve modules.
   *
   * @return The current SwerveModulePositions.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetOdometry(new Pose3d(pose));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose3d pose) {
    m_odometry.resetPosition(getGyroRotation3d(), getModulePositions(), pose);
  }

  /**
   * Gets the field-relative heading of the robot.
   *
   * @return The robot's heading as a {@link Rotation2d}.
   */
  public Rotation2d getHeading() {
    return m_odometry.getEstimatedPosition().getRotation().toRotation2d();
  }

  /**
   * Gets the heading of the robot from the gyro directly. This may be offset from the
   * field-relative heading.
   *
   * @return The gyro heading as a {@link Rotation2d}.
   */
  public Rotation2d getGyroHeading() {
    return m_gyro.getRotation2d();
  }

  /**
   * Gets the field-relative 3D rotation of the robot.
   *
   * @return The robot's rotation as a {@link Rotation3d}.
   */
  public Rotation3d getRotation3d() {
    return m_odometry.getEstimatedPosition().getRotation();
  }

  /**
   * Gets the 3D rotation of the robot from the gyro directly. This may be offset from the
   * field-relative rotation.
   *
   * @return The gyro rotation as a {@link Rotation3d}.
   */
  public Rotation3d getGyroRotation3d() {
    return m_gyro.getRotation3d();
  }

  /**
   * Gets the turn rate of the robot.
   *
   * @return The turn rate of the robot as a {@link Rotation2d}.
   */
  public Rotation2d getTurnRate() {
    return Rotation2d.fromDegrees(-m_gyro.getRate());
  }

  /**
   * Method to drive the robot using speed info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward) in meters per second.
   * @param ySpeed Speed of the robot in the y direction (sideways) in meters per second.
   * @param rot Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds;

    if (fieldRelative) {
      Rotation2d headingForFieldRelative =
          DriverStation.isFMSAttached() ? getHeading() : getGyroHeading();
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, headingForFieldRelative);
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    drive(chassisSpeeds);
  }

  /**
   * Method to drive the robot using a robot-relative {@link ChassisSpeeds} object.
   *
   * @param chassisSpeeds The desired chassis speeds.
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    m_previousSetpoint =
        DriveConstants.setpointGenerator.generateSetpoint(
            m_previousSetpoint, chassisSpeeds, TimedRobot.kDefaultPeriod);

    setModuleStates(m_previousSetpoint.moduleStates());
  }

  /**
   * Creates a {@link Command} to drive the robot using joystick info. Handles deadbands and
   * inversion of joystick inputs.
   *
   * @param xSpeed Supplier of the speed of the robot in the x direction (forward) from -1 to 1.
   * @param ySpeed Supplier of the speed of the robot in the y direction (sideways) from -1 to 1.
   * @param rot Supplier of the angular rate of the robot from -1 to 1.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @return The command.
   */
  public Command drive(
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotSupplier,
      boolean fieldRelative) {
    return run(
        () -> {
          double xSpeed = xSpeedSupplier.getAsDouble();
          double ySpeed = ySpeedSupplier.getAsDouble();
          double rot = rotSupplier.getAsDouble();

          xSpeed = -MathUtil.applyDeadband(xSpeed, OIConstants.kDriveDeadband);
          ySpeed = -MathUtil.applyDeadband(ySpeed, OIConstants.kDriveDeadband);
          rot = -MathUtil.applyDeadband(rot, OIConstants.kDriveDeadband);

          // Convert the commanded speeds into the correct units for the drivetrain
          xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
          ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
          rot *= DriveConstants.kMaxAngularSpeed;

          drive(xSpeed, ySpeed, rot, fieldRelative);
        });
  }

  /**
   * Creates a {@link Command} that sets the wheels into an X formation to prevent movement.
   *
   * @return The command.
   */
  public Command setX() {
    return runOnce(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    Pose3d currentPose = getPose3d();

    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();

    resetOdometry(currentPose);
  }

  /**
   * Creates a {@link Command} that resets the field-relative controls.
   *
   * @return The command.
   */
  public Command resetFieldRelative() {
    return Commands.runOnce(
            () -> {
              Pose3d currentPose = getPose3d();
              m_gyro.reset();

              resetOdometry(currentPose);
            })
        .ignoringDisable(true);
  }

  /**
   * Creates a {@link Command} that sets the idle mode for all swerve modules.
   *
   * @param idleMode The desired idle mode (Brake or Coast).
   * @return The command.
   */
  public Command setIdleMode(IdleMode idleMode) {
    return Commands.runOnce(
            () -> {
              m_frontLeft.setIdleMode(idleMode);
              m_frontRight.setIdleMode(idleMode);
              m_rearLeft.setIdleMode(idleMode);
              m_rearRight.setIdleMode(idleMode);
            })
        .ignoringDisable(true);
  }
}
