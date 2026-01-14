// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;

public class Drivetrain extends SubsystemBase {
  // Create SwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftTurningEncoderCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightTurningEncoderCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId, DriveConstants.kRearLeftTurningEncoderCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId, DriveConstants.kRearRightTurningEncoderCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_UART);

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
      new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
          m_rearLeft.getPosition(), m_rearRight.getPosition() });

  private final SwerveSetpointGenerator m_setpointGenerator = new SwerveSetpointGenerator(
      DriveConstants.kRobotConfig, ModuleConstants.kMaxSteerSpeed);
  private SwerveSetpoint m_previousSetpoint;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Report swerve drive to the HAL
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_Other);

    AutoBuilder.configure(this::getPose, this::resetOdometry, this::getChassisSpeeds, this::drive,
        AutoConstants.kPathFollowingController, DriveConstants.kRobotConfig, () -> false, this);

    m_previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(),
        DriveFeedforwards.zeros(DriveConstants.kRobotConfig.numModules));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(),
        new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
            m_rearLeft.getPosition(), m_rearRight.getPosition() });
  }

  /**
   * Gets the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Gets the current {@link ChassisSpeeds} of the robot.
   * 
   * @return The current ChassisSpeeds of the robot.
   */
  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics
        .toChassisSpeeds(getModuleStates());

    chassisSpeeds.omegaRadiansPerSecond = getTurnRate().getRadians();
    return chassisSpeeds;
  }

  /**
   * Gets the current states of all the swerve modules.
   * 
   * @return The current SwerveModuleStates.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] { m_frontLeft.getState(), m_frontRight.getState(),
        m_rearLeft.getState(), m_rearRight.getState() };
  }

  /**
   * Gets the current positions of all the swerve modules.
   * 
   * @return The current SwerveModulePositions.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_rearLeft.getPosition(), m_rearRight.getPosition() };
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward) in
   *                      meters per second.
   * @param ySpeed        Speed of the robot in the y direction (sideways) in
   *                      meters per second.
   * @param rot           Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds;

    if (fieldRelative) {
      Rotation2d headingForFieldRelative = DriverStation.isFMSAttached() ? getHeading()
          : m_gyro.getRotation2d();
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
          headingForFieldRelative);
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    drive(chassisSpeeds);
  }

  /**
   * Method to drive the robot using a ChassisSpeeds object. This handles the
   * inversion of joystick inputs.
   *
   * @param chassisSpeeds The desired chassis speeds.
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    m_previousSetpoint = m_setpointGenerator.generateSetpoint(m_previousSetpoint, chassisSpeeds,
        TimedRobot.kDefaultPeriod);

    setModuleStates(m_previousSetpoint.moduleStates());
  }

  /**
   * Creates a {@link Command} to drive the robot using joystick info. Handles
   * deadbands and inversion of joystick inputs.
   *
   * @param xSpeed        Supplier of the speed of the robot in the x direction
   *                      (forward) from -1 to 1.
   * @param ySpeed        Supplier of the speed of the robot in the y direction
   *                      (sideways) from -1 to 1.
   * @param rot           Supplier of the angular rate of the robot from -1 to 1.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public Command drive(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
      DoubleSupplier rotSupplier, boolean fieldRelative) {
    return run(() -> {
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
   * Creates a {@link Command} that sets the wheels into an X formation to prevent
   * movement.
   * 
   * @return The command.
   */
  public Command setX() {
    return runOnce(() -> {
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
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    Pose2d currentPose = getPose();

    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();

    resetOdometry(currentPose);
  }

  /**
   * Creates a {@link Command} that resets the field relative controls.
   * 
   * @return The command.
   */
  public Command resetFieldRelative() {
    return runOnce(() -> {
      Pose2d currentPose = getPose();
      m_gyro.reset();

      resetOdometry(currentPose);
    });
  }

  /**
   * Gets the heading of the robot.
   *
   * @return the robot's heading as a {@link Rotation2d}.
   */
  public Rotation2d getHeading() {
    return m_odometry.getPoseMeters().getRotation();
  }

  /**
   * Gets the turn rate of the robot.
   *
   * @return The turn rate of the robot as a {@link Rotation2d}.
   */
  public Rotation2d getTurnRate() {
    return Rotation2d.fromDegrees(-m_gyro.getRate());
  }
}
