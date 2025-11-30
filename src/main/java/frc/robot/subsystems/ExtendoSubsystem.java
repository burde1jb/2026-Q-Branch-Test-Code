package frc.robot.subsystems;
//Import all libraries needed to call up the systems used
//Here we call up Spark Flex motor controller and an Absolute Encoder
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.AbsoluteEncoder;
//The generic "DigitalInput" is used when plugging in a sensor like a magnetic limit switch
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

import au.grapplerobotics.LaserCan;

public class ExtendoSubsystem extends SubsystemBase {
    //Name the motors and sensors
    SparkFlex ExtendoMotor;
    AbsoluteEncoder ExtendoEncoder;
    SparkFlexConfig ExtendoMotorConfig;
     private LaserCan lc;
    public DigitalInput sensor;
    public SparkLimitSwitch magLimitSwitch;
    public LimitSwitchConfig magLimitSwitchConfig;
    //Define the set values to be used with the sensor - Keep the actual value in the RobotConstants
    private final double rangeOffset = RobotConstants.ExtendoRangeOffset;
    private final double encoderOffset = RobotConstants.ExtendoEncoderOffset;

    public ExtendoSubsystem() {
        //Define the motor with the CAN ID value and type of motor (brushed or brushless)
        ExtendoMotor = new SparkFlex(RobotConstants.ExtendoMotorCANid, MotorType.kBrushless);
        //Define the sensor - the REV Encoder can be either Absolute (0 to 1) or Quadrature (Unlimited)
        ExtendoEncoder = ExtendoMotor.getAbsoluteEncoder();
        ExtendoMotorConfig = new SparkFlexConfig();
        ExtendoMotorConfig.limitSwitch
                .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
                //OLD way of doing the same thing as above - Deprecated for 2026
                //.forwardLimitSwitchEnabled(true);
        lc = new LaserCan(0);

    }

    public double getEncoderMeasurement() {
        var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;
        return (position);
    }

//Set the commands we want to use in the COMMAND portion
//This command allows the motor to run until we reach the goal of the Encoder
    public void goTo(double degrees) {

        var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;

        if (position < degrees) {
            ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
        } else if ((position  - (2 * rangeOffset)) > (degrees)) {
            ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        }
        else {
            this.stop();
        }
    }

    public void goToL4(double degrees) {

        var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;

        if (position < degrees) {
            ExtendoMotor.set(RobotConstants.ExtendoL4ExtendSpeed);
        } else if ((position  - (2 * rangeOffset)) > (degrees)) {
            ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        }
        else {
            this.stop();
        }
    }

//This command is to be used in the AUTONOMOUS period because this returns only TRUE or FALSE values
    public boolean wentTo(double degrees) {
        LaserCan.Measurement measurement = lc.getMeasurement();

        var position = (ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1;

        if (position < (degrees + encoderOffset)) {
            this.Extend();
            // ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
            return false;
        } else if ((position - (2 * rangeOffset)) > (degrees)) {
            this.Retract();
            // ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
            return false;
        } else {
            this.stop();
            return true;
        }
    }

    public void Extend() {
        if ((ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1 < (RobotConstants.ExtendoExtendL4)) {
        ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
        }
        else {
            this.stop();
        }
        } 

    public void ExtendAuton(boolean forward) {
            if (((ExtendoEncoder.getPosition() + rangeOffset + encoderOffset) % 1 < (RobotConstants.ExtendoExtendAuton)) && forward) {
                ExtendoMotor.set(RobotConstants.ExtendoExtendSpeed);
            }
            else {
                ExtendoMotor.set(0);
            }
        }

    public void Retract() {
        if ((ExtendoEncoder.getPosition() - rangeOffset + encoderOffset) % 1 > (RobotConstants.ExtendoRetract)){
        ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        }
        else {
            this.stop();
        }
    } 

    public void RetractAuton(boolean forward) {
        if (((ExtendoEncoder.getPosition() - rangeOffset + encoderOffset) % 1 > (RobotConstants.ExtendoRetract)) && forward) {
             ExtendoMotor.set(RobotConstants.ExtendoRetractSpeed);
        }
        else {
            ExtendoMotor.set(0);
        }
    }

    public void stop() {
        ExtendoMotor.stopMotor();
    }

//This is for AUTONOMOUS and only a check - no effect on what the motor does
    public boolean encoderCheck(double distance) {
        if (ExtendoEncoder.getPosition() == distance) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
    //This puts a number on the Smart Dashboard, the big screen which pops up with the FRC Driver Station
        SmartDashboard.putNumber("Extendo Encoder", (ExtendoEncoder.getPosition() + encoderOffset) % 1);
    }
}