package frc.robot.commands.vision;

import frc.robot.subsystems.DriveSubsystem;

//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.OdometryUtils;
import frc.utils.SwerveUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.security.MessageDigestSpi;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

//Import this so you can make this class a command
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAim extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final VisionSubsystem m_visionSubsystem;
    final DriveSubsystem m_driveSubsystem;
    final PIDController distanceController = new PIDController(.2, 0, 0.05);

    boolean isAlignDistacne = false;
    boolean isAlignRotation = false;
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    //If you want to contoll whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;

    //Class Constructor
    public AutoAim(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;
        m_visionSubsystem = visionSubsystem;
        
        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.
        addRequirements(m_visionSubsystem, m_driveSubsystem);
    }



    /*Like Robot.java, there are a series of functions that you can override to give the command functionality. */
    

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    @Override
    public void initialize(){
        //m_chassisSubsystem.setBrakeMode();
        m_visionSubsystem.setPipeline(VisionConstants.kAprilTagPipeline);
        m_complete = false;
    }

    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute(){
        double y = m_visionSubsystem.getY();
        double x = m_visionSubsystem.getX();
        double targets = m_visionSubsystem.getTV();
        double forwardSpeed = 0;
        double rotate = 0;
        double angle = m_gyro.getAngle(); 
        double crabCrawl = 0;
        //SmartDashboard.putNumber("motor speed align targets", x); 

            //Move forwards/backwards
            Translation2d pos1 = m_driveSubsystem.getPose().getTranslation();
            Translation2d pos2 = new Translation2d(FieldConstants.kSpeakerX, FieldConstants.kSpeakerY);
            Rotation2d angleToTarget = OdometryUtils.anglePoseToPose(pos1, pos2);
            double distanceToTarget = OdometryUtils.getDistacnePosToPos(pos1, pos2);
            SmartDashboard.putNumber("Angle to Goal", angleToTarget.getDegrees());
            SmartDashboard.putNumber("Distance to Goal", distanceToTarget);
            SmartDashboard.putNumber("Targets", targets);

            /* 
                get shooter to be at the same angle
                1. Get absolute encoder value from the shooter 
                2. Move the shooter to meet that pot value, do this by using PID to go to the angleToTarget
                3. Shoot the note at the fastest possible speed, lets see how it works
             */
            double encoderValue = 0;
            double offset = 1; 
            double speedAngleChange = 0;
            double speedOfShot = 1;
            double maxDistanceShot = 20;

            if (distanceToTarget > maxDistanceShot){
                m_driveSubsystem.drive(0, distanceController.calculate(1), 0, false, true);
            }

            if (Math.abs(encoderValue-offset)>angleToTarget.getDegrees()){
                speedAngleChange = Math.abs(encoderValue-angleToTarget.getDegrees());
                speedAngleChange = distanceController.calculate(speedAngleChange);
            }



            
            //we need to find angle of shot, theta, then we need to find speed of shot. Are we just finding a vector?


            //P controller for distance (fred)
            //forwardSpeed = (currentPosition - desiredPosition) * Pconstant
        //     if ((distanceFromTarget < VisionConstants.kTopPoleDesiredDistance - VisionConstants.kDistanceTolerance) 
        //     || (distanceFromTarget > VisionConstants.kTopPoleDesiredDistance + VisionConstants.kDistanceTolerance)){
        //         forwardSpeed = (m_visionSubsystem.getReflectiveTapeDistance() - VisionConstants.kTopPoleDesiredDistance) * VisionConstants.kForwardSpeedPConstant;
        //     }
        //     else{
        //         forwardSpeed = 0;
        //     }

        //     //Change LED state
        //     if((distanceFromTarget > 39) && (distanceFromTarget < 41)){
        //         isAlignDistacne = true;
        //     }
        //     else{
        //         isAlignDistacne = false;
        //     }
        //     if ((x+1 < VisionConstants.kRotationTolerance)&&(x+1 > -VisionConstants.kRotationTolerance)){
        //         isAlignRotation = true;
        //     }
        //     else{
        //         isAlignRotation = false;
        //     }
        // }
        //PID
        forwardSpeed = distanceController.calculate(forwardSpeed);
        crabCrawl = distanceController.calculate(crabCrawl);
        SmartDashboard.putNumber("motor speed align forwardspeed", forwardSpeed);
        SmartDashboard.putNumber("motor speed align rotation", rotate);

       // m_driveSubsystem.drive(forwardSpeed*.2, crabCrawl*.2, rotate*.2, false, true);
  
    
        
    }

    /*This function is called once when the command ends.
     * A command ends either when you tell it to end with the "isFinished()" function below, or when it is interupted.
     * Whether a command is interrupted or not is determined by "boolean interrupted."
     * Things initialized in "initialize()" should be closed here.
     */
    @Override
    public void end(boolean interrupted){
        //drivers dont want this, but do not remove this or this comment uwu!
        //m_chassisSubsystem.drive(0, 0);

    }

    @Override
    public boolean isFinished(){
        return m_complete;
    }
}