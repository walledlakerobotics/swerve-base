// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final CANcoder m_turningEncoder;

  private final RelativeEncoder m_turningFeedbackEncoder; // Forwards from CANCoder

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private Rotation2d m_chassisAngularOffset = Rotation2d.kZero;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, Rotation2d.kZero);

  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * CANcoder, and PID controller.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int turningEncoderId,
      double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningFeedbackEncoder = m_turningSpark.getEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.SwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.SwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningEncoder = new CANcoder(turningEncoderId);

    MagnetSensorConfigs turningEncoderConfig = new MagnetSensorConfigs();
    m_turningEncoder.getConfigurator().refresh(turningEncoderConfig);

    turningEncoderConfig.withSensorDirection(
        ModuleConstants.kTurningEncoderInverted ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive);

    m_turningEncoder.getConfigurator().apply(turningEncoderConfig);

    m_chassisAngularOffset = Rotation2d.fromRadians(chassisAngularOffset);
    m_desiredState.angle = getAngle();
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Gets the current angle of the module. Also ensures the turning feedback
   * encoder is correctly updated with values from the CANCoder.
   * 
   * @return The current angle of the module.
   */
  private Rotation2d getAngle() {
    double rotations = m_turningEncoder.getAbsolutePosition().getValueAsDouble();
    m_turningFeedbackEncoder.setPosition(rotations);

    return Rotation2d.fromRotations(rotations);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        getAngle().minus(m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(m_drivingEncoder.getPosition(),
        getAngle().minus(m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(m_chassisAngularOffset);

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(getAngle());

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond,
        ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRotations(),
        ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Sets the idle mode for both motors in the swerve module.
   * 
   * @param idleMode The desired idle mode (Brake or Coast).
   */
  public void setIdleMode(IdleMode idleMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(idleMode);

    m_drivingSpark.configure(config, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    m_turningSpark.configure(config, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
