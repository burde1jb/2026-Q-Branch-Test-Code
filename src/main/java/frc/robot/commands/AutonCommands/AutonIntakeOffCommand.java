package frc.robot.commands.AutonCommands;
//Import all subsystems to be used in this autonomous command
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class AutonIntakeOffCommand extends Command {
    //Call up the Subsystems to be used in this Command
    CoralIntakeSubsystem intakeSubsystem;

    public AutonIntakeOffCommand(CoralIntakeSubsystem intakeSubsystem) {
        //One more time, as this is command based coding, we name the subsystem with "this.subsystem"
        this.intakeSubsystem = intakeSubsystem;
        //You have to addRequirements so the system knows what to do when the controller is not doing anything
        //This "addRequirements" must be added for every Command
        //The place to define this is "RobotContainer" with the ".setDefaultCommand" lines
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        //This is where to define what Commands to do
        //The Commands are defined in the Subsystem portion
        intakeSubsystem.intakeOff();
    }
}
