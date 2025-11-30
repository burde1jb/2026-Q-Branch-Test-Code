package frc.robot.commands;
//Import the controllers to be used (XBox or FLight Sticks)
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.*;

public class ClimberCommand extends Command {
    //Call up the Subsystems to be used in this Command
    ClimberSubsystem climberSubsystem;
    //Name the contorllers to be used
    CommandJoystick controller1;
    XboxController controller2;

    public ClimberCommand(
        //We called up the subsystem, but now we need to name the subsytem
        //This name is used only here in this command for the subsystem
            ClimberSubsystem climberSubsystem,
            CommandJoystick controller1,
            XboxController controller2) {
        //One more time, as this is command based coding, we name the subsystem with "this.subsystem"
        this.climberSubsystem = climberSubsystem;
        this.controller1 = controller1;
        this.controller2 = controller2;
        //You have to addRequirements so the system knows what to do when the controller is not doing anything
        //This "addRequirements" must be added for every Command
        //The place to define this is "RobotContainer" with the ".setDefaultCommand" lines
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
    //This is where to define what Commands to do when the controller inputs are either pressed or pass a threshold
    //The Commands are defined in the Subsystem portion
        if (controller2.getPOV() == 0){
            climberSubsystem.extend();
        }
        else if (controller2.getPOV() == 180){
            climberSubsystem.retract();
        }
        else{
           climberSubsystem.stop(); 
        }
    }}