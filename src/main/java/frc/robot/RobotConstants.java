package frc.robot;

import edu.wpi.first.units.measure.measure.Angle;
import edu.wpi.first.units.measure.measure.Distance;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class RobotConstants {
  // AprilTag Constants
    public final static double ReefOffset = 0.15;
    public final static double ReefOffsetLeft = - 0.05;

  // Elevator Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor for moving
    // elevator up and down
    public final static int ElevatorMotorCANid = 53;
    public final static int ElevatorMotor2CANid = 54;
    public final static int ElevatorEncoderDIOidA = 5;
    public final static int ElevatorEncoderDIOidB = 6;

    // Elevator Values for encoders - Offsets used to make end values between 0 and
    // 1
    public final static double ElevatorEncoderOffset = 0.00;
    public final static double ElevatorRangeOffset = 50.00;

    // Elevator PID Values for Motors
    public final static double ElevatorMotorP = 0;
    public final static double ElevatorMotorI = 0;
    public final static double ElevatorMotorD = 0;
    public final static double ElevatorMotorFF = 0;

    // Elevator Power values for motors
    public final static double ElevatorUpSpeed = -1.00;
    public final static double ElevatorDownSpeed = 0.70;
    public final static double ElevatorOffSpeed = 0.00;
    public final static double ElevatorDownSlowSpeed = 0.40;

    // Elevator Values for encoders - Offsets used to make end values between 0 and
    // 1

    public final static double HomeValue = 0.0;
    public final static double HomeValueExtreme = -50.0;
    public final static double ExtendoClearValue = 80.0;
    public final static double L1Value = 200.0;
    public final static double L1HighExtreme = 220.0;
    public final static double L2Value = 1000.0;
    public final static double L2HighExtreme = 1100.0;
    public final static double L3Value = 2000.0;
    public final static double L3HighExtreme = 2100.0;
    public final static double L4Value = 4000.0;
    public final static double L4HighExtreme = 4100.0;

     // Elevator Values for encoders - Offsets used to make end values between 0 and
    // 1

    public final static double lcHomeValue = 35.0;
    public final static double lcL2Value = 95.0;
    public final static double lcL3Value = 270.0;
    public final static double lcL4Value = 560.0;
    public final static double lcBargeValue = 560.0;
    public final static double lcrangeOffset = 60;
    public final static double lcSlowZone = 30.0;
    public final static double lcClimbValue = 380.0;
    public final static double lcrangeOffsetHome = 35;

    // Algae Wrist Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor moving the
    // Algae Intake in/out of robot
    public final static int AlgaeWristmotorCANid = 23;
    public final static int AlgaeWristEncoderDIOid = 1;
    public final static int AlagaeWristQuadEncoder1 = 8;
    public final static int AlagaeWristQuadEncoder2 = 9;

    // Algae Wrist PID Values for Motors
    public final static double AlgaeWristmotorP = 0;
    public final static double AlgaeWristmotorI = 0;
    public final static double AlgaeWristmotorD = 0;
    public final static double AlgaeWristmotorFF = 0;

    // Algae Wrist Values for encoders - Offsets used to make end values between 0
    // and 1
    public final static double AlgaeWristencoderOffset = 0.00;
    public final static double AlgaeWristrangeOffset = 0.03;
    public final static double AlgaeWristExtendgoal = 0.449;
    public final static double AlgaeWristRetractgoal = 0.670;

    // Algae Wrist Power values for motors
    public final static double AlgaeWristExtendpower = 0.40;
    public final static double AlgaeWristRetractpower = -0.40;

    // Algae Intake Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor Spinning
    // Wheels on Algae Intake
    public final static int AlgaeIntakeCANid = 33;

    // Algae Intake PID Values for Motors
    public final static double AlgaeIntakemotorP = 0;
    public final static double AlgaeIntakemotorI = 0;
    public final static double AlgaeIntakemotorD = 0;
    public final static double AlgaeIntakemotorFF = 0;

    // Algae Intake Power values for motors
    public final static double AlgaeIntakeOnspeed = -0.50;
    public final static double AlgaeIntakeOutspeed = 0.50;
    public final static double AlgaeIntakeSlowspeed = -0.2;

    // Extendo Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - REV Linear Actuator
    public final static int ExtendoMotorCANid = 29;
    public final static int ExtendoEncoderDIOidA = 3;
    public final static int ExtendoEncoderDIOidB = 4;

    // Extendo Values for encoders - Offsets used to make end values between 0 and 1
    public final static double ExtendoEncoderOffset = 0.15;
    public final static double ExtendoRangeOffset = 0.03;
    public final static double ExtendoBarge = 0.515;
    public final static double ExtendoExtend = 0.723;
    public final static double ExtendoExtendL4 = 0.730;
    public final static double ExtendoExtendAuton = 0.730;
    public final static double ExtendoRetract = 0.155;

    // Extendo PID Values for Motors
    public final static double ExtendoMotorP = 0.10;
    public final static double ExtendoMotorI = 0;
    public final static double ExtendoMotorD = 0.01;
    public final static double ExtendoMotorFF = 0;

    // Extendo Power values for motors
    public final static double ExtendoExtendSpeed = 0.600;
    public final static double ExtendoL4ExtendSpeed = 0.50;
    public final static double ExtendoRetractSpeed = -0.700;
    public final static double ExtendoOffSpeed = 0.00;

    // Coral Intake Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor Spinning
    // Wheels on Algae Intake
    public final static int CoralIntakeCANid = 22;
    public final static int CoralIntakeSensorDIOid = 7;

    // Algae Intake PID Values for Motors
    public final static double CoralIntakemotorP = 0;
    public final static double CoralIntakemotorI = 0;
    public final static double CoralIntakemotorD = 0;
    public final static double CoralIntakemotorFF = 0;

    // Algae Intake Power values for motors
    public final static double CoralIntakeOnspeed = -0.9;
    public final static double CoralIntakeOnspeedAuton = -0.90;
    public final static double CoralIntakeOutspeed = 0.90;
    public final static double CoralIntakeSlowspeed = 0.15;

        // Climber Constants
    // CAN, PWM, DIO values for motors, encoders, and sensors - Motor moving the
    // Climber in/out of robot
    public final static int ClimbermotorCANid = 24;
    public final static int ClimberEncoderDIOid = 1;
    public final static int ClimberQuadEncoder1 = 8;
    public final static int ClimberQuadEncoder2 = 9;

    // Climber PID Values for Motors
    public final static double ClimbermotorP = 0;
    public final static double ClimbermotorI = 0;
    public final static double ClimbermotorD = 0;
    public final static double ClimbermotorFF = 0;

    // Climber Values for encoders - Offsets used to make end values between 0
    // and 1
    public final static double ClimberencoderOffset = 0.00;
    public final static double ClimberrangeOffset = 0.03;
    public final static double ClimberExtendgoal = 0.469;
    public final static double ClimberRetractgoal = 0.670;
    public final static double ClimberClimbgoal = 0;
    public final static double ClimberReleasegoal = 0;

    // Climber Power values for motors
    public final static double ClimberExtendpower = 0.40;
    public final static double ClimberRetractpower = -0.40;
    public final static double ClimberClimbpower = 0.50;
    public final static double ClimberReleasepower = -0.50;

    // LED Values
    public final static double LEDintakesensor = 0.81;
    public final static double LEDintakeReady = 0.75;
    public final static double LEDdefault = 0.57;

    public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
    public static final Distance LIMELIGHT_LENS_HEIGHT = Distance.ofBaseUnits(8, Inches);
    public static final Angle LIMELIGHT_ANGLE = Angle.ofBaseUnits(0, Degrees);

    public static final Distance REEF_APRILTAG_HEIGHT = Distance.ofBaseUnits(6.875, Inches);
    public static final Distance PROCCESSOR_APRILTAG_HEIGHT = Distance.ofBaseUnits(45.875, Inches);
    public static final Distance CORAL_APRILTAG_HEIGHT = Distance.ofBaseUnits(53.25, Inches);
  }
}
