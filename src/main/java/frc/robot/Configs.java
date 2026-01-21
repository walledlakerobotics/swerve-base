package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

/** Contains configuration objects for various robot components. */
public final class Configs {
  public static final class SwerveModule {
    public static final TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    public static final MagnetSensorConfigs turningEncoderConfig = new MagnetSensorConfigs();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = 1 / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 1 / ModuleConstants.kTurningMotorReduction;
      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(
                      ModuleConstants.kDrivingMotorsInverted
                          ? InvertedValue.Clockwise_Positive
                          : InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(ModuleConstants.kDrivingMotorNeutralMode))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(ModuleConstants.kDrivingMotorSupplyLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(ModuleConstants.kDrivingMotorStatorLimit)
                  .withSupplyCurrentLimitEnable(true))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(drivingFactor))
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.04)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKV(drivingVelocityFeedForward));

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
          .pid(1, 0, 0)
          .outputRange(-1, 1)
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
