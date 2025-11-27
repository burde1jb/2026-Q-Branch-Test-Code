package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.AlgaeWristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ExtendoSubsystem extendoSubsystem;
    AlgaeWristSubsystem wristSubsystem;
    XboxController controller2;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            XboxController controller2) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.extendoSubsystem = extendoSubsystem;
        this.controller2 = controller2;
        addRequirements(elevatorSubsystem);
        addRequirements(extendoSubsystem);
    }

    @Override
    public void execute() {
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
        // if (controller2.getLeftY() < -0.2) {
        //     elevatorSubsystem.goUp();
        // } else if (controller2.getLeftY() > 0.2) {
        //     elevatorSubsystem.goDown();
        // } else if (controller2.getLeftX() > 0.2) {
        //     extendoSubsystem.Extend();
        // } else if (controller2.getLeftX() < -0.2) {
        //     extendoSubsystem.Retract();
        // } else if (controller2.getAButton()) { // Bottom value
        //     elevatorSubsystem.lcgoTo(RobotConstants.lcHomeValue);
        //     extendoSubsystem.goTo(RobotConstants.ExtendoRetract);
        // } else if (controller2.getXButton()) { // L2 value
        //     elevatorSubsystem.lcgoTo(RobotConstants.lcL2Value);
        //     extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
        // } else if (controller2.getYButton()) { // L4 value
        //     elevatorSubsystem.lcgoTo(RobotConstants.lcL4Value);
        //     extendoSubsystem.goTo(RobotConstants.ExtendoExtendL4);
        // } else if (controller2.getBButton()) { // L3 value
        //     elevatorSubsystem.lcgoTo(RobotConstants.lcL3Value);
        //     extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
        // } else if (controller2.getStartButton()) { // Climb Value
        //     // elevatorSubsystem.lcgoTo(RobotConstants.lcClimbValue);
        //     extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
        // } else {
        //     elevatorSubsystem.stop();
        //     extendoSubsystem.stop();
        // }
    }
}