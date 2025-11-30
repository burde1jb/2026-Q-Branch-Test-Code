package frc.robot.commands;
//Import the controllers to be used (XBox or Flight Sticks)
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.*;

public class AlgaeIntakeCommand extends Command {
    //Call up the Subsystems to be used in this Command
    AlgaeIntakeSubsystem intakeSubsystem;
    AlgaeWristSubsystem wristSubsystem;
    //Name the contorllers to be used
    XboxController controller2;

    public AlgaeIntakeCommand(
        //We called up the subsystem, but now we need to name the subsytem
        //This name is used only here in this command for the subsystem
            AlgaeIntakeSubsystem intakeSubsystem,
            AlgaeWristSubsystem wristSubsystem,
            XboxController controller2) {
        //One more time, as this is command based coding, we name the subsystem with "this.subsystem"
        this.intakeSubsystem = intakeSubsystem;
        this.wristSubsystem = wristSubsystem;
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
        if (controller2.getLeftTriggerAxis() > 0.2) {
            intakeSubsystem.AlgaeIntakeOn(true);
        } else if (controller2.getRightTriggerAxis() > 0.2 && controller2.getLeftTriggerAxis() > 0.2) {
            intakeSubsystem.AlgaeIntakeSlow(false);
        } else if (controller2.getRightTriggerAxis() > 0.2) {
            intakeSubsystem.AlgaeIntakeOn(false);
        } else {
            intakeSubsystem.AlgaeIntakeOff();
        }

        if (controller2.getRightY() > 0.2) {
            wristSubsystem.goTo(RobotConstants.AlgaeWristRetractgoal);
        } else if (controller2.getRightY() < -0.2) {
            wristSubsystem.goTo(RobotConstants.AlgaeWristExtendgoal);
        } else {
            wristSubsystem.stop();
        }
    }
}