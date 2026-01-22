package frc.robot;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

/** Contains configuration objects for various robot components. */
public final class Configs {
  public static final class SwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    public static final MagnetSensorConfigs turningEncoderConfig = new MagnetSensorConfigs();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 1.0 / ModuleConstants.kTurningMotorReduction;
      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
          .inverted(ModuleConstants.kDrivingMotorsInverted)
          .idleMode(ModuleConstants.kDrivingMotorIdleMode)
          .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.04, 0, 0)
          .outputRange(-1.0, 1.0)
          .feedForward
          .kS(0.0)
          .kV(drivingVelocityFeedForward);

      turningConfig
          .inverted(ModuleConstants.kTurningMotorsInverted)
          .idleMode(ModuleConstants.kTurningMotorIdleMode)
          .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
      turningConfig
          .encoder
          .positionConversionFactor(turningFactor) // rotations
          .velocityConversionFactor(turningFactor / 60.0); // rotations per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(1.0, 0, 0)
          .outputRange(-1.0, 1.0)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(-0.5, 0.5);

      turningEncoderConfig
          .withSensorDirection(
              ModuleConstants.kTurningEncoderInverted
                  ? SensorDirectionValue.Clockwise_Positive
                  : SensorDirectionValue.CounterClockwise_Positive)
          .withAbsoluteSensorDiscontinuityPoint(0.5);
    }
  }
}
