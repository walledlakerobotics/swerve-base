package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class SwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        public static final CANcoderConfiguration turningEncoderConfig = new CANcoderConfiguration();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 1 / ModuleConstants.kTurningMotorReduction;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage
                    / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig.inverted(ModuleConstants.kDrivingMotorsInverted)
                    .idleMode(ModuleConstants.kDrivingMotorIdleMode)
                    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            drivingConfig.encoder.positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0).outputRange(-1, 1).feedForward.kV(drivingVelocityFeedForward);

            turningConfig.inverted(ModuleConstants.kTurningMotorsInverted)
                    .idleMode(ModuleConstants.kTurningMotorIdleMode)
                    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

            turningConfig.encoder.positionConversionFactor(turningFactor) // rotations
                    .velocityConversionFactor(turningFactor / 60.0); // rotations per second

            turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(3.5, 0, 0.5).outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true).positionWrappingInputRange(-0.5, 0.5);

            MagnetSensorConfigs turningMagnetSensorConfig = new MagnetSensorConfigs();
            turningMagnetSensorConfig.withSensorDirection(ModuleConstants.kTurningEncoderInverted
                    ? SensorDirectionValue.Clockwise_Positive
                    : SensorDirectionValue.CounterClockwise_Positive);

            turningEncoderConfig.withMagnetSensor(turningMagnetSensorConfig);
        }
    }
}
