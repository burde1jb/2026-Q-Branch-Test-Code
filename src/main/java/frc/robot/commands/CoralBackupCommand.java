package frc.robot.commands;
//Import the controllers to be used (XBox or FLight Sticks)
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralBackupCommand extends Command {
    //Call up the Subsystems to be used in this Command
    CoralIntakeSubsystem intakeSubsystem;
    //Name the contorllers to be used
    XboxController controller2;

    public CoralBackupCommand(CoralIntakeSubsystem intakeSubsystem) {
        //One more time, as this is command based coding, we name the subsystem with "this.subsystem"
        this.intakeSubsystem = intakeSubsystem;
        //You have to addRequirements so the system knows what to do when the controller is not doing anything
        //This "addRequirements" must be added for every Command
        //The place to define this is "RobotContainer" with the ".setDefaultCommand" lines
        addRequirements(intakeSubsystem);
    }

    // Turn on the intake-slow command
    @Override
    public void initialize() {
        intakeSubsystem.intakeSlow(true);
    }

    // Command ends when beam is no longer broken
    @Override
    public boolean isFinished() {
        return !intakeSubsystem.isBeamBroken();
    }

    // This is called after isFinished() returns true
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeOff();
    }
}
