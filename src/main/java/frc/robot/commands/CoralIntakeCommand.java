package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class CoralIntakeCommand extends Command {
    CoralIntakeSubsystem intakeSubsystem;
    XboxController controller2;

    public CoralIntakeCommand(CoralIntakeSubsystem intakeSubsystem, XboxController controller2) {
        this.intakeSubsystem = intakeSubsystem;
        this.controller2 = controller2;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
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
        // if ((controller2.getRightBumperButton() && !intakeSubsystem.beambreak.isPressed()) || (controller2.getLeftBumper() && intakeSubsystem.beambreak.isPressed())) {
        //     intakeSubsystem.intakeOn(true);
        // } else if (controller2.getBackButton()) {
        //     intakeSubsystem.intakeSlow(true);
        // } else if (controller2.getStartButton()) {
        //     intakeSubsystem.intakeSlow(false);
        // } else {
        //     intakeSubsystem.intakeOff();
        // }
    }
}