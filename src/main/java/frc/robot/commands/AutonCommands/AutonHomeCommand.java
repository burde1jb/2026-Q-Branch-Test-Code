package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
// import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutonHomeCommand extends Command {
    Timer timer;
    // ExtendoSubsystem extendoSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    boolean isItFinished;
    boolean extendoFinished;
    boolean elevatorFinished;

    public AutonHomeCommand(ElevatorSubsystem elevatorSubsystem) {
        timer = new Timer();
        // this.extendoSubsystem = extendoSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        // addRequirements(extendoSubsystem);
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        isItFinished = false;
        // extendoFinished = false;
        elevatorFinished = false;
        timer.restart();

    }

    @Override
    public void execute() {
        // if (!extendoFinished && extendoSubsystem.wentTo(RobotConstants.ExtendoRetract) || timer.get() > 2.5) {
        //     extendoSubsystem.stop();
        //     extendoFinished = true;
        // }
        if (!elevatorFinished && elevatorSubsystem.lcwentTo(RobotConstants.lcHomeValue) || timer.get() > 3.5) {
            elevatorSubsystem.stop();
            elevatorFinished = true;
        }
        if (elevatorFinished) {
            isItFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isItFinished;
    }
}