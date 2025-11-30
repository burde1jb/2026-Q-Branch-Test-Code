package frc.robot.subsystems;
//Import all libraries needed to call up the systems used
//Here we call up Spark Flex motor controller and an Absolute Encoder
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class AlgaeWristSubsystem extends SubsystemBase {
    //Name the motors and sensors
    private SparkFlex wristMotor;
    private AbsoluteEncoder wristEncoder;
    //Define the set values to be used with the sensor - Keep the actual value in the RobotConstants
    private final double rangeOffset = RobotConstants.AlgaeWristrangeOffset;
    private final double encoderOffset = RobotConstants.AlgaeWristencoderOffset;

    public AlgaeWristSubsystem() {
        //Define the motor with the CAN ID value and type of motor (brushed or brushless)
        wristMotor = new SparkFlex(RobotConstants.AlgaeWristmotorCANid,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        //Define the sensor - the REV Encoder can be either Absolute (0 to 1) or Quadrature (Unlimited)
        wristEncoder = wristMotor.getAbsoluteEncoder();
    }

//Set the commands we want to use in the COMMAND portion
//This command allows the motor to run until we reach the goal of the Encoder
    public void goTo(double encoderGoal) {
        if ((wristEncoder.getPosition()) < (encoderGoal - rangeOffset)) {
            this.retract();
        } else if ((wristEncoder.getPosition()) > (encoderGoal + rangeOffset)) {
            this.extend();
        } else {
            this.stop();
        }
    }

//This command is to be used in the AUTONOMOUS period because this returns only TRUE or FALSE values
    public boolean wentTo(double encoderGoal, double extremaValue) {
        if ((wristEncoder.getPosition() + encoderOffset) % 1 < (encoderGoal - rangeOffset + encoderOffset) % 1) {
            this.retract();
            return false;
        } else if ((wristEncoder.getPosition() + encoderOffset) % 1 > (encoderGoal + rangeOffset + encoderOffset) % 1) {
            this.extend();
            return false;
        } else {
            this.stop();
            return true;
        }
    }

//Here we name the commands without the use of the Encoder
    public void extend() {
        wristMotor.set(RobotConstants.AlgaeWristExtendpower);
    }

    public void retract() {
        wristMotor.set(RobotConstants.AlgaeWristRetractpower);
    }

    public void stop() {
        wristMotor.stopMotor();
    }

//This is for AUTONOMOUS and only a check - no effect on what the motor does
    public boolean encoderCheck(double distance) {
        if (wristEncoder.getPosition() == distance) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
    //This puts a number on the Smart Dashboard, the big screen which pops up with the FRC Driver Station
        SmartDashboard.putNumber("Algae Wrist Encoder", (wristEncoder.getPosition()));
    }
}