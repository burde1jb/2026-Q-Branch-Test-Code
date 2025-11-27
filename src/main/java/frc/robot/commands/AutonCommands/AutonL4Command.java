package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
// import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonL4Command extends Command {
    // ExtendoSubsystem extendoSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    boolean isItFinished;
    // boolean extendoFinished;
    boolean elevatorFinished;
    Timer timer;

    public AutonL4Command(ElevatorSubsystem elevatorSubsystem) {
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
        elevatorSubsystem.lcwentTo(RobotConstants.lcL4Value);
        // if (!extendoFinished && extendoSubsystem.wentTo(RobotConstants.ExtendoExtendL4) || timer.get() > 2.5) {
        //     extendoSubsystem.stop();
        //     extendoFinished = true;
        // }
        // if (elevatorSubsystem.lcwentTo(RobotConstants.lcL4Value) || timer.get() > 2.5) {
        //     elevatorSubsystem.stop();
        //     elevatorFinished = true;
        // }
        // if (elevatorFinished) {
        //     isItFinished = true;
        // }
    }

    @Override
    public boolean isFinished() {
        if (elevatorSubsystem.lcwentTo(RobotConstants.lcL4Value) || timer.get() >= 2.5) {
            elevatorSubsystem.stop();
            return true;
        }
        return false;
        // return isItFinished;
    }
}