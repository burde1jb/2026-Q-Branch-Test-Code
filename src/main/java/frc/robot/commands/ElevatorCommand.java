package frc.robot.commands;
//Import the controllers to be used (XBox or Flight Sticks)
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.AlgaeWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    //Call up the Subsystems to be used in this Command
    ElevatorSubsystem elevatorSubsystem;
    ExtendoSubsystem extendoSubsystem;
    AlgaeWristSubsystem wristSubsystem;
    //Name the contorllers to be used
    XboxController controller2;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            XboxController controller2) {
        //One more time, as this is command based coding, we name the subsystem with "this.subsystem"
        this.elevatorSubsystem = elevatorSubsystem;
        this.extendoSubsystem = extendoSubsystem;
        this.controller2 = controller2;
        //You have to addRequirements so the system knows what to do when the controller is not doing anything
        //This "addRequirements" must be added for every Command
        //The place to define this is "RobotContainer" with the ".setDefaultCommand" lines
        addRequirements(elevatorSubsystem);
        addRequirements(extendoSubsystem);
    }

    @Override
    public void execute() {
    //This is where to define what Commands to do when the controller inputs are either pressed or pass a threshold
    //The Commands are defined in the Subsystem portion
        if (controller2.getAButton()) { //Home Value
            elevatorSubsystem.lcgoToHome(RobotConstants.lcHomeValue);
            extendoSubsystem.goTo(RobotConstants.ExtendoRetract);
        } else if (controller2.getXButton()) { //L2 Value
            elevatorSubsystem.lcgoTo(RobotConstants.lcL2Value);
            extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
        } else if (controller2.getYButton()) { //L4 Value
            elevatorSubsystem.lcgoTo(RobotConstants.lcL4Value);
            extendoSubsystem.goToL4(RobotConstants.ExtendoExtendL4);
        } else if (controller2.getBButton()) { //L3 Value
            elevatorSubsystem.lcgoTo(RobotConstants.lcL3Value);
            extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
        } else if (controller2.getLeftY() < -0.2) {
            elevatorSubsystem.goUp();
        } else if (controller2.getLeftY() > 0.2) {
            elevatorSubsystem.goDown();
        } else if (controller2.getLeftX() > 0.2) {
            extendoSubsystem.Extend();
        } else if (controller2.getLeftX() < -0.2) {
            extendoSubsystem.Retract();
        } else if (controller2.getStartButton()) { // Barge Algae Value
            elevatorSubsystem.lcgoTo(RobotConstants.lcL4Value);
            extendoSubsystem.goTo(RobotConstants.ExtendoBarge);
        } else {
            elevatorSubsystem.stop();
            extendoSubsystem.stop();
        }
    }
}