package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class VisionSubsystem extends SubsystemBase {
    final public NetworkTableEntry ty;
    final public NetworkTableEntry tx; 
    //final public NetworkTableEntry tl;
    final public NetworkTableEntry tv;

    public VisionSubsystem(){
        //TODO: maybe this should be an instance variable?
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        ty = table.getEntry("ty");
        tx = table.getEntry("tx");
        //TODO: what is tl and why is it not configured properly?
        //tl = table.getEntry("ty");
        tv = table.getEntry("tv");
        setPipeline(VisionConstants.kDefaultPipeline);

        Shuffleboard.getTab("Vision").addInteger("Pipeline", () -> getPipeline());
        Shuffleboard.getTab("Vision").addDouble("Distance", () -> getReflectiveTapeDistance());
    }

    //tv = valid targets
    //tx horizontal offset from crosshair to target
    //ty vertical offset from crosshair to target
    //ta = target area 0% to 100%
    
    public void setPipeline(int pipeline){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }

    public int getPipeline(){
        return ((Double)NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getNumber(-1)).intValue();
    }

    /**
     * Returns the horizontal offset from the crosshair to the target. Returns 0 if the target can't be found.
     */
    public double getX(){
        return tx.getDouble(0.0);
    }

    /**
     * Returns the vertical offset from the crosshair to the target. Returns 0 if the target can't be found.
     */
    public double getY(){
        return ty.getDouble(0.0);
    }
    
    // public double getTL(){
    //     return tl.getDouble(0.0);
    // }
    
    /**
     * Returns the number of valid targets detected by the limelight. Returns 0 if no targets are found.
     */
    public int getTV(){
        return (int)tv.getDouble(0);
    }

    /**
     * Returns the distance between the robot and the reflective tape goal. Returns 0 if no targets can be found.
     */
    public double getReflectiveTapeDistance(){
        //Check for no targets
        if(getTV() == 0){
            return 0;
        }
        
        final double targetOffsetAngle_Vertical = getY();

        // distance from the target to the floor
        final double goalHeightInches;

        if(targetOffsetAngle_Vertical > 0){
            goalHeightInches = VisionConstants.kTopReflectiveTapeHeight;
        }
        else{
            goalHeightInches = VisionConstants.kBottomReflectiveTapeHeight;
        }

        final double angleToGoalDegrees = VisionConstants.kLimelightMountAngle + targetOffsetAngle_Vertical;
        final double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate distance
        return (goalHeightInches - VisionConstants.kLimelightLensHeight) / Math.tan(angleToGoalRadians);
    }


    @Override
    public void periodic(){
    }
}