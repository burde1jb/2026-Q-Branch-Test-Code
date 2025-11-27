package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralBackupCommand extends Command {
        CoralIntakeSubsystem intakeSubsystem;
    XboxController controller2;

    public CoralBackupCommand(CoralIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
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
