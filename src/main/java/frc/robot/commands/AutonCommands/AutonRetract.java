package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.ExtendoSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutonRetract extends Command {
    ExtendoSubsystem extendoSubsystem;
    boolean isItFinished;
    boolean extendoFinished;
    Timer timer;

    public AutonRetract(ExtendoSubsystem extendoSubsystem) {
        timer = new Timer();
        this.extendoSubsystem = extendoSubsystem;
        addRequirements(extendoSubsystem);
    }

    @Override
    public void initialize() {
        isItFinished = false;
        extendoFinished = false;
        timer.restart();
        

    }

    @Override
    public void execute() {
        extendoSubsystem.RetractAuton(true);
        // if (!extendoFinished && extendoSubsystem.wentTo(RobotConstants.ExtendoRetract) || timer.get() > 1.0) {
        //     extendoSubsystem.stop();
        //     extendoFinished = true;
        // }
        // if (extendoFinished) {
        //     isItFinished = true;
        // }
    }

    @Override
    public boolean isFinished() {
        if (timer.get() >= 1.5) {
            extendoSubsystem.stop();
            return true;
        }
        return false;
        // return isItFinished;
    }
}