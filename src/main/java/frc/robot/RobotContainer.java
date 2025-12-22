// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.ExtendoSubsystemDRY;
import frc.robot.subsystems.ExtendoSubsystemPID1;

public class RobotContainer {
  // kSpeedAt12Volts desired top speed
  //private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // 3/4 of a rotation per second max angular velocity
  //private double MaxAngularRate = RotationsPerSecond.of(3.0).in(RadiansPerSecond);
  //private final CommandJoystick joystick = new CommandJoystick(0);
  private final XboxController xboxController = new XboxController(1);
  private final CommandXboxController CommandXbox = new CommandXboxController(1);
  // public static final SwerveDrivetrainSubsystem commandSwerveDrivetrain = TunerConstants.createDrivetrain();
  // /* Setting up bindings for necessary control of the swerve drive platform */
  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(MaxSpeed * 0.1)
  //     .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);
  // private final CoralIntakeSubsystem intakeSubsystem;
  // private final AlgaeWristSubsystem wristSubsystem;
  
  //CHOOSE ONE
      //private final ExtendoSubsystem extendoSubsystem = new ExtendoSubsystem();;
      //private final ExtendoSubsystemDRY extendoSubsystem = new ExtendoSubsystemDRY();
      private final ExtendoSubsystemPID1 extendoSubsystem = new ExtendoSubsystemPID1();

  // private final LEDSubsystem ledSubsystem;
  // private final ElevatorSubsystem elevatorSubsystem;
  // private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  // private final VisionSubsystem visionSubsystem;
  // private final SendableChooser<Command> autoChooser;
  // private final ClimberSubsystem climberSubsystem;
  // private final AprilTagManager ATMan;

  public RobotContainer() {
    // this.intakeSubsystem = new CoralIntakeSubsystem();
    // this.wristSubsystem = new AlgaeWristSubsystem();
    // this.visionSubsystem = new VisionSubsystem();
    // this.extendoSubsystem = new ExtendoSubsystem();
    // this.ledSubsystem = new LEDSubsystem();
    // this.elevatorSubsystem = new ElevatorSubsystem();
    // this.algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    // this.climberSubsystem = new ClimberSubsystem();
    // ATMan = new AprilTagManager(commandSwerveDrivetrain); 

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // commandSwerveDrivetrain.setDefaultCommand(
    //     // Drivetrain will execute this command periodically
    //     // Drive forward with negative Y (forward)
    //     commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(joystick.getRawAxis(4) * MaxSpeed)
    //         // Drive left with negative X (left)
    //         .withVelocityY(-joystick.getRawAxis(3) * MaxSpeed)
    //         // Drive counterclockwise with negative X (left)
    //         .withRotationalRate(-joystick.getRawAxis(0) * MaxAngularRate)));

    // NamedCommands.registerCommand("AutonHomeCommand", new AutonHomeCommand(elevatorSubsystem));
    // NamedCommands.registerCommand("AutonL1Command", new AutonL1Command(extendoSubsystem, elevatorSubsystem));
    // NamedCommands.registerCommand("AutonL4Command", new AutonL4Command(elevatorSubsystem));
    // NamedCommands.registerCommand("AutonTimedIntakeCommand", new AutonTimedIntakeCommand(intakeSubsystem));
    // NamedCommands.registerCommand("AutonTimedIntakeCommandReverse", new AutonTimedIntakeCommandReverse(intakeSubsystem));
    // NamedCommands.registerCommand("AutonTimedIntakeCommandShort", new AutonTimedIntakeCommandShort(intakeSubsystem));
    // NamedCommands.registerCommand("AutonIntakeOffCommand", new AutonIntakeOffCommand(intakeSubsystem));
    // NamedCommands.registerCommand("AutonIntakeOnCommand", new AutonIntakeOffCommand(intakeSubsystem));
    // NamedCommands.registerCommand("AutonIntakeOn", AutonIntakeOn());
    // NamedCommands.registerCommand("AutonIntakeOff", AutonIntakeOff());
    // NamedCommands.registerCommand("AutonExtend", new AutonExtend(extendoSubsystem));
    // NamedCommands.registerCommand("AutonRetract", new AutonRetract(extendoSubsystem));



    // autoChooser = AutoBuilder.buildAutoChooser("Straight 1 Coral");
    // SmartDashboard.putData("Auto Mode", autoChooser);

    // algaeIntakeSubsystem
    //     .setDefaultCommand(new AlgaeIntakeCommand(algaeIntakeSubsystem, wristSubsystem, xboxController));
    // elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, extendoSubsystem, xboxController));
    // intakeSubsystem.setDefaultCommand(new CoralIntakeCommand(intakeSubsystem, xboxController));
    // ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem, xboxController, intakeSubsystem, elevatorSubsystem, extendoSubsystem));
    // // visionSubsystem.setDefaultCommand(new AlignCommand(commandSwerveDrivetrain, visionSubsystem,6));
    // climberSubsystem.setDefaultCommand(new ClimberCommand(climberSubsystem, joystick, xboxController));

    configureBindings();
  }

    //  private final IntSupplier OptionalButtonSupplier = ()-> {
    //     if(joystick.button(12).getAsBoolean())//button 12 is the bottom right 3 position switch. 
    //     {
    //         return 2;//2 is the right side option on the reef
    //     }
    //         //default option is that x is not pressed and we score left side.
    //     else return 0;//0 is the left side option on reef
        
    // };

  // public Command alignReefForCoral()
  // {
  //     return new ConditionalCommand( ATMan.C_ReefLeftSelectCommand(), ATMan.C_ReefRightSelectCommand(),()->{return OptionalButtonSupplier.getAsInt() == 0;}).asProxy();//.until(MantaState.getLimeLightBypassed)
  // }
  double JoystickAxisDeadZone = 0.2;//REMOVING MAGIC NUMBER AND GIVING IT A NAME
  int LeftXAxis = 0; //REMOVING MAGIC NUMBER AND GIVING IT A NAME
  
  private void ConfigExtendoPID()
  {
    //Extendo subsystem PID version
      CommandXbox.a().onTrue(extendoSubsystem.goTo(RobotConstants.ExtendoRetract));
      CommandXbox.x().onTrue(extendoSubsystem.goTo(.4).andThen(extendoSubsystem.goTo(.7)).andThen(extendoSubsystem.goTo(.35)));
      CommandXbox.y().onTrue(extendoSubsystem.goTo(RobotConstants.ExtendoExtendL4));
      CommandXbox.b().onTrue(extendoSubsystem.goTo(RobotConstants.ExtendoExtend));
      CommandXbox.axisMagnitudeGreaterThan(LeftXAxis, JoystickAxisDeadZone).whileTrue((new InstantCommand(()->{extendoSubsystem.Move(CommandXbox.getRawAxis(LeftXAxis));}).repeatedly()));
      CommandXbox.start().onTrue(extendoSubsystem.goTo(RobotConstants.ExtendoBarge));
      //no stop command here becuase pid goes to a location and never "stops" it just only uses the force it needs (some exceptions exist for park commands or locking positions)
  }
  private void ConfigExtendo()
  {
      //THE ORIGINAL WAY THE BUTTONS ARE CHECKED EACH LOOP (NOT USED THIS VERSION)
      //extendoSubsystem.setDefaultCommand(new ElevatorCommand(extendoSubsystem, xboxController));
      //OR
      //Extendo subsystem and DRY version.
      // CommandXbox.a().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoRetract);}).repeatedly());
      // CommandXbox.x().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoExtend);}).repeatedly());
      // CommandXbox.y().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoExtendL4);}).repeatedly());
      // CommandXbox.b().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoExtend);}).repeatedly());
      // //CommandXbox.axisMagnitudeGreaterThan(LeftXAxis, JoystickAxisDeadZone).whileTrue((new InstantCommand(()->{extendoSubsystem.Move(CommandXbox.getRawAxis(LeftXAxis));}).repeatedly()));
      // CommandXbox.axisGreaterThan(LeftXAxis, JoystickAxisDeadZone).whileTrue((new InstantCommand(()->{extendoSubsystem.Extend();}).repeatedly()));
      // CommandXbox.axisLessThan(LeftXAxis, -JoystickAxisDeadZone).whileTrue((new InstantCommand(()->{extendoSubsystem.Retract();}).repeatedly()));
      // CommandXbox.start().whileTrue(new InstantCommand(()->{extendoSubsystem.goTo(RobotConstants.ExtendoBarge);}).repeatedly());
      // CommandXbox.a().negate()
      //   .and(CommandXbox.b().negate())
      //   .and(CommandXbox.x().negate())
      //   .and(CommandXbox.y().negate())
      //   .and(CommandXbox.start().negate())
      //   .and(CommandXbox.axisMagnitudeGreaterThan(LeftXAxis, JoystickAxisDeadZone).negate())
      //   .onTrue(new InstantCommand(()->{extendoSubsystem.stop();}));
  }
  public void ConfigExtendoOriginal()
  {

    //this is a snippet found in Elevatorcommand.java
    // if (controller2.getAButton()) { //Home Value
    //         //elevatorSubsystem.lcgoToHome(RobotConstants.lcHomeValue);
    //         extendoSubsystem.goTo(RobotConstants.ExtendoRetract);
    //     } else if (controller2.getXButton()) { //L2 Value
    //         //elevatorSubsystem.lcgoTo(RobotConstants.lcL2Value);
    //         extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
    //     } else if (controller2.getYButton()) { //L4 Value
    //         //elevatorSubsystem.lcgoTo(RobotConstants.lcL4Value);
    //         extendoSubsystem.goToL4(RobotConstants.ExtendoExtendL4);
    //     } else if (controller2.getBButton()) { //L3 Value
    //         //elevatorSubsystem.lcgoTo(RobotConstants.lcL3Value);
    //         extendoSubsystem.goTo(RobotConstants.ExtendoExtend);
    //     } else if (controller2.getLeftY() < -0.2) {
    //         //elevatorSubsystem.goUp();
    //     } else if (controller2.getLeftY() > 0.2) {
    //         //elevatorSubsystem.goDown();
    //     } else if (controller2.getLeftX() > 0.2) {
    //         extendoSubsystem.Extend();
    //     } else if (controller2.getLeftX() < -0.2) {
    //         extendoSubsystem.Retract();
    //     } else if (controller2.getStartButton()) { // Barge Algae Value
    //         //elevatorSubsystem.lcgoTo(RobotConstants.lcL4Value);
    //         extendoSubsystem.goTo(RobotConstants.ExtendoBarge);
    //     } else {
    //         //elevatorSubsystem.stop();
    //         extendoSubsystem.stop();
    //     }
  }

  private void configureBindings() {

    //ConfigExtendo();
    ConfigExtendoPID();   

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
    // joystick.button(1).whileTrue(alignReefForCoral());
    
    // // reset the field-centric heading on left bumper press
    // joystick.button(13).onTrue(commandSwerveDrivetrain.runOnce(() -> commandSwerveDrivetrain.seedFieldCentric()));

    // commandSwerveDrivetrain.registerTelemetry(logger::telemeterize);
  }

  // public Command getAutonomousCommand() {
  //   /* First put the drivetrain into auto run mode, then run the auto */
  //   return autoChooser.getSelected();
  // }

  // public Command AutonIntakeOn() {
  //   return new InstantCommand(() -> {
  //     intakeSubsystem.intakeOn(true);
  //   });
  // }

  // public Command AutonIntakeOff() {
  //   return new InstantCommand(() -> {
  //     intakeSubsystem.intakeOff();
  //   });
  // }
}
