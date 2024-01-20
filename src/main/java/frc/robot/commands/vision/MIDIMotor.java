package frc.robot.commands.vision;
import java.util.Timer;

import frc.robot.subsystems.DriveSubsystem;
//json imports
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import java.io.FileReader;
import java.io.IOException;

//Import subsystem(s) this command interacts with below

import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Path;

import javax.sound.midi.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Filesystem;





//Import this so you can make this class a command
import edu.wpi.first.wpilibj2.command.Command;

public class MIDIMotor extends Command {

    //Import any instance variables that are passed into the file below here, such as the subsystem(s) your command interacts with.
    final DriveSubsystem m_driveSubsystem;
    final PIDController distanceController = new PIDController(.2, 0, 0.05);

    //If you want to contoll whether or not the command has ended, you should store it in some sort of variable:
    private boolean m_complete = false;
    private static final int MOTOR_CAN_ID = 1; // Replace with your actual CAN ID
    private static final double MAX_SPEED = 1.0; // Maximum motor speed
    

    private CANSparkMax motor;
    //Class Constructor
    public MIDIMotor(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem){
        m_driveSubsystem = driveSubsystem;
        motor = new CANSparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
        //If your command interacts with any subsystem(s), you should pass them into "addRequirements()"
        //This function makes it so your command will only run once these subsystem(s) are free from other commands.
        //This is really important as it will stop scenarios where two commands try to controll a motor at the same time.

    }



    /*Like Robot.java, there are a series of functions that you can override to give the command functionality. */
    

    /*This function is called once when the command is schedueled.
     * If you are overriding "isFinished()", you should probably use this to set m_complete to false in case a command object is 
     * called a second time.
     */
    //When not overridden, this function is blank.
    String trajectoryJSON = "src/main/deploy/midi/grouped_notes_with_speed.json";
    @Override
    public void initialize(){
        //m_chassisSubsystem.setBrakeMode();
        m_complete = false;
        long currentTimeMillis = System.currentTimeMillis();
    }
    /*This function is called repeatedly when the schedueler's "run()" function is called.
     * Once you want the function to end, you should set m_complete to true.
     */
    @Override
    public void execute(){
        // Path to the JSON file
        String filePath = "src/main/deploy/midi/grouped_notes_with_speed.json";
        try {
            // Parse the JSON file
            JSONParser parser = new JSONParser();
            JSONArray jsonArray = (JSONArray) parser.parse(new FileReader(filePath));

            // Iterate through the array and print values
            //create something to track the ammount of time passed, and then when the note is at the certain time, run the motor until the note is no longer at that time
            for (Object arrayElement : jsonArray) {
                JSONObject jsonObject = (JSONObject) arrayElement;
                System.out.println("Note: " + jsonObject.get("Note") + ", Speed: " + jsonObject.get("Speed"));
                //motor.set(MAX_SPEED);
            }
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    
    
        
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