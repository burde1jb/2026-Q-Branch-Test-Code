// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.IntSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.AutonCommands.AutonHomeCommand;
import frc.robot.commands.AutonCommands.AutonL1Command;
import frc.robot.commands.AutonCommands.AutonL4Command;
import frc.robot.commands.AutonCommands.AutonTimedIntakeCommand;
import frc.robot.commands.AutonCommands.AutonTimedIntakeCommandShort;
import frc.robot.commands.AutonCommands.AutonIntakeOffCommand;
import frc.robot.commands.AutonCommands.AutonTimedIntakeCommandReverse;
import frc.robot.commands.AutonCommands.AutonExtend;
import frc.robot.commands.AutonCommands.AutonRetract;

import frc.robot.commands.ClimberCommand;
// import frc.robot.commands.VisionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AlgaeWristSubsystem;
import frc.robot.subsystems.AprilTagManager;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class RobotContainer {
  // kSpeedAt12Volts desired top speed
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(3.0).in(RadiansPerSecond);
  private final CommandJoystick joystick = new CommandJoystick(0);
  private final XboxController xboxController = new XboxController(1);
  public static final SwerveDrivetrainSubsystem commandSwerveDrivetrain = TunerConstants.createDrivetrain();
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CoralIntakeSubsystem intakeSubsystem;
  private final AlgaeWristSubsystem wristSubsystem;
  private final ExtendoSubsystem extendoSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final SendableChooser<Command> autoChooser;
  private final ClimberSubsystem climberSubsystem;
  private final AprilTagManager ATMan;

  public RobotContainer() {
    this.intakeSubsystem = new CoralIntakeSubsystem();
    this.wristSubsystem = new AlgaeWristSubsystem();
    this.visionSubsystem = new VisionSubsystem();
    this.extendoSubsystem = new ExtendoSubsystem();
    this.ledSubsystem = new LEDSubsystem();
    this.elevatorSubsystem = new ElevatorSubsystem();
    this.algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    this.climberSubsystem = new ClimberSubsystem();
    ATMan = new AprilTagManager(commandSwerveDrivetrain); 

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    commandSwerveDrivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        // Drive forward with negative Y (forward)
        commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(joystick.getRawAxis(4) * MaxSpeed)
            // Drive left with negative X (left)
            .withVelocityY(-joystick.getRawAxis(3) * MaxSpeed)
            // Drive counterclockwise with negative X (left)
            .withRotationalRate(-joystick.getRawAxis(0) * MaxAngularRate)));

    NamedCommands.registerCommand("AutonHomeCommand", new AutonHomeCommand(elevatorSubsystem));
    NamedCommands.registerCommand("AutonL1Command", new AutonL1Command(extendoSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("AutonL4Command", new AutonL4Command(elevatorSubsystem));
    NamedCommands.registerCommand("AutonTimedIntakeCommand", new AutonTimedIntakeCommand(intakeSubsystem));
    NamedCommands.registerCommand("AutonTimedIntakeCommandReverse", new AutonTimedIntakeCommandReverse(intakeSubsystem));
    NamedCommands.registerCommand("AutonTimedIntakeCommandShort", new AutonTimedIntakeCommandShort(intakeSubsystem));
    NamedCommands.registerCommand("AutonIntakeOffCommand", new AutonIntakeOffCommand(intakeSubsystem));
    NamedCommands.registerCommand("AutonIntakeOnCommand", new AutonIntakeOffCommand(intakeSubsystem));
    NamedCommands.registerCommand("AutonIntakeOn", AutonIntakeOn());
    NamedCommands.registerCommand("AutonIntakeOff", AutonIntakeOff());
    NamedCommands.registerCommand("AutonExtend", new AutonExtend(extendoSubsystem));
    NamedCommands.registerCommand("AutonRetract", new AutonRetract(extendoSubsystem));



    autoChooser = AutoBuilder.buildAutoChooser("Straight 1 Coral");
    SmartDashboard.putData("Auto Mode", autoChooser);

    algaeIntakeSubsystem
        .setDefaultCommand(new AlgaeIntakeCommand(algaeIntakeSubsystem, wristSubsystem, xboxController));
    elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, extendoSubsystem, xboxController));
    intakeSubsystem.setDefaultCommand(new CoralIntakeCommand(intakeSubsystem, xboxController));
    ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem, xboxController, intakeSubsystem, elevatorSubsystem, extendoSubsystem));
    // visionSubsystem.setDefaultCommand(new AlignCommand(commandSwerveDrivetrain, visionSubsystem,6));
    climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem, joystick, xboxController));

    configureBindings();
  }

     private final IntSupplier OptionalButtonSupplier = ()-> {
        if(joystick.button(12).getAsBoolean())//button 12 is the bottom right 3 position switch. 
        {
            return 2;//2 is the right side option on the reef
        }
            //default option is that x is not pressed and we score left side.
        else return 0;//0 is the left side option on reef
        
    };

  public Command alignReefForCoral()
  {
      return new ConditionalCommand( ATMan.C_ReefLeftSelectCommand(), ATMan.C_ReefRightSelectCommand(),()->{return OptionalButtonSupplier.getAsInt() == 0;}).asProxy();//.until(MantaState.getLimeLightBypassed)
  }

  private void configureBindings() {
    // joystick.button(13).whileTrue(commandSwerveDrivetrain.applyRequest(() -> brake));
    // joystick.button(14).whileTrue(commandSwerveDrivetrain.applyRequest(
    //     () -> point.withModuleDirection(new Rotation2d(-joystick.getRawAxis(3), -joystick.getRawAxis(4)))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.button(1).and(joystick.button(12)).whileTrue(commandSwerveDrivetrain.sysIdDynamic(Direction.kForward));
    // joystick.button(1).and(joystick.button(11)).whileTrue(commandSwerveDrivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.button(3).and(joystick.button(12)).whileTrue(commandSwerveDrivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.button(3).and(joystick.button(11)).whileTrue(commandSwerveDrivetrain.sysIdQuasistatic(Direction.kReverse));

    //joystick.button(1).whileTrue(new AlignCommand(commandSwerveDrivetrain, visionSubsystem));
    joystick.button(1).whileTrue(alignReefForCoral());
    
    // reset the field-centric heading on left bumper press
    joystick.button(13).onTrue(commandSwerveDrivetrain.runOnce(() -> commandSwerveDrivetrain.seedFieldCentric()));

    commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  public Command AutonIntakeOn() {
    return new InstantCommand(() -> {
      intakeSubsystem.intakeOn(true);
    });
  }

  public Command AutonIntakeOff() {
    return new InstantCommand(() -> {
      intakeSubsystem.intakeOff();
    });
  }
}
