package frc.robot.commands;
//Import the controllers to be used (XBox or Flight Sticks)
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class CoralIntakeCommand extends Command {
    //Call up the Subsystems to be used in this Command
    CoralIntakeSubsystem intakeSubsystem;
    //Name the contorllers to be used
    XboxController controller2;

    public CoralIntakeCommand(CoralIntakeSubsystem intakeSubsystem, XboxController controller2) {
        //One more time, as this is command based coding, we name the subsystem with "this.subsystem"
        this.intakeSubsystem = intakeSubsystem;
        this.controller2 = controller2;
        //You have to addRequirements so the system knows what to do when the controller is not doing anything
        //This "addRequirements" must be added for every Command
        //The place to define this is "RobotContainer" with the ".setDefaultCommand" lines
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
    //This is where to define what Commands to do when the controller inputs are either pressed or pass a threshold
    //The Commands are defined in the Subsystem portion
        if (controller2.getLeftBumperButton()){
            intakeSubsystem.limitSwitchOff(true);
            intakeSubsystem.intakeOn(false);
        }
        else if (controller2.getRightBumperButton()) {
            intakeSubsystem.limitSwitchOff(false);
            intakeSubsystem.intakeOn(true);
        }
        else if (controller2.getBackButton()){
            intakeSubsystem.limitSwitchOff(false);
            intakeSubsystem.intakeSlow(true);
        }
        else {
            intakeSubsystem.intakeOff();
        }
        
    }
}