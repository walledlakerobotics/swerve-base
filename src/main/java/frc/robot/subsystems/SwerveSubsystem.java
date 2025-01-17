package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    SwerveDrive swerveDrive;
    public SwerveSubsystem(File dir){
        File swerveJsonDir = new File(Filesystem.getDeployDirectory(), "swerve");
        try{
            swerveDrive = new SwerveParser(dir).createSwerveDrive(SwerveConstants.maxSpeed, SwerveConstants.driveConversionFactor, SwerveConstants.angleConversionFactor);
        }catch(Exception e){
            throw new RuntimeException(e);
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        //swerveDrive.setCosineCompensator(false);  This line is for simulation testing
       
    }
    public void lockPose(){
        swerveDrive.lockPose();
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
    {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

    
    
}
