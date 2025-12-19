package frc.robot.commands.AutonCommands;
//Import all subsystems to be used in this autonomous command
//We are using time as a metric in autonomous so we need to define a timer
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class AutonTimedIntakeCommandReverse extends Command {
    //Call up the Subsystems to be used in this Command
    CoralIntakeSubsystem intakeSubsystem;
    //We are using time as a metric in autonomous so we need to define a timer
    Timer timer;

    public AutonTimedIntakeCommandReverse(CoralIntakeSubsystem intakeSubsystem) {
        //One more time, as this is command based coding, we name the subsystem with "this.subsystem"
        this.intakeSubsystem = intakeSubsystem;
        timer = new Timer();
        //You have to addRequirements so the system knows what to do when the controller is not doing anything
        //This "addRequirements" must be added for every Command
        //The place to define this is "RobotContainer" with the ".setDefaultCommand" lines
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //Robots are dumb and they need something to tell the program to move on to the next command
        //Here we need to tell the robot that the command is not yet finished
        //We also need to tell the robot to start a timer
        timer.restart();
    }

    @Override
    public void execute() {
        //This is where to define what Commands to do
        //The Commands are defined in the Subsystem portion
        intakeSubsystem.intakeOn(false);
    }

    @Override
    public boolean isFinished() {
        //This is where we define how much time is allowed for the command
        //Otherwise if the encoder does not reach the desired goal or sensor gets tripped, then the
        //robot will continue the action and NOT move on to the next action
        //This value will need to be adjusted through testing. You want as little time as possible
        //while giving enough wiggle room time in case a smal mechanical issue happens which slows the action.
        if (timer.get() >= 0.3) {
            intakeSubsystem.intakeOff();
            return true;
        }
        return false;
    }
}
