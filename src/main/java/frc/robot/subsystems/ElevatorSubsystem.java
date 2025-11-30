package frc.robot.subsystems;
//Import all libraries needed to call up the systems used
//Here we call up Spark Flex motor controller
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
//Importing from Grapple Robotics due to using their LaserCAN device
import au.grapplerobotics.LaserCan;
//Import for the REV Encoder and using a RELATIVE (Quadrature) Encoder
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class ElevatorSubsystem extends SubsystemBase {
    //Name the motors and sensors
    SparkFlex ElevatorMotor;
    RelativeEncoder ElevatorEncoder;
    //Define the set values to be used with the sensor - Keep the actual value in the RobotConstants
    private final double rangeOffset = RobotConstants.ElevatorRangeOffset;
    private final double lcrangeOffset = RobotConstants.lcrangeOffset;
    private final double lcrangeOffsetHome = RobotConstants.lcrangeOffsetHome;
    SparkFlexConfig ElevatorMotorConfig;
    private LaserCan lc;

    public ElevatorSubsystem() {
        //Define the motor with the CAN ID value and type of motor (brushed or brushless)
        ElevatorMotor = new SparkFlex(RobotConstants.ElevatorMotorCANid,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        //Define the sensor - the REV Encoder can be either Absolute (0 to 1) or Quadrature (Unlimited)
        ElevatorEncoder = ElevatorMotor.getExternalEncoder();
        //Define the LaserCAN distance reader - only need the CAN id
        lc = new LaserCan(0);

        ElevatorMotorConfig = new SparkFlexConfig();
    }

//Set the commands we want to use in the COMMAND portion
//This command allows the motor to run until we reach the goal of the Encoder
    public void goTo(double encoderGoal) {
        if ((ElevatorEncoder.getPosition()) < (encoderGoal - rangeOffset)) {
            this.goUp();
        } else if ((ElevatorEncoder.getPosition()) > (encoderGoal + rangeOffset)) {
            this.goDown();
        } else {
            this.stop();
        }
    }

//This command allows the motor to run until we reach the goal of the LaserCAN Distance Sensor
//You can use either the encoder above or the laser below to determine distance
    public void lcgoTo(double lcGoal) {

        LaserCan.Measurement measurement = lc.getMeasurement();
        //You have to remember to program what the robot is to do IF the LaserCAN comes unplugged
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            System.out.println("The target is " + measurement.distance_mm + "mm away!");
        } else {
            System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
            // You can still use distance_mm in here, if you're ok tolerating a clamped
            // value or an unreliable measurement.
        }
        if (measurement != null){
        if ((measurement.distance_mm) < (lcGoal - lcrangeOffset)) {
            this.goUp();
        } else if ((measurement.distance_mm) >= (lcGoal + lcrangeOffset) && (measurement.distance_mm) >= RobotConstants.lcSlowZone) {
            this.goDown();
        } else if ((measurement.distance_mm) >= (lcGoal + lcrangeOffset) && (measurement.distance_mm) < RobotConstants.lcSlowZone) {
         this.goDownSlow();
        } else {
            this.stop();
        }}
        else {
            this.stop();
        }
        }

        public void lcgoToHome(double lcGoal) {

            LaserCan.Measurement measurement = lc.getMeasurement();
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                System.out.println("The target is " + measurement.distance_mm + "mm away!");
            } else {
                System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
                // You can still use distance_mm in here, if you're ok tolerating a clamped
                // value or an unreliable measurement.
            }
            if (measurement != null){
            if ((measurement.distance_mm) < (lcGoal - lcrangeOffsetHome)) {
                this.goUp();
            } else if ((measurement.distance_mm) >= (lcGoal + lcrangeOffsetHome) && (measurement.distance_mm) >= RobotConstants.lcSlowZone) {
                this.goDown();
            } else if ((measurement.distance_mm) >= (lcGoal + lcrangeOffsetHome) && (measurement.distance_mm) < RobotConstants.lcSlowZone) {
             this.goDownSlow();
            } else {
                this.stop();
            }}
            else {
                this.stop();
            }
            }
        
//This command is to be used in the AUTONOMOUS period because this returns only TRUE or FALSE values
    public boolean wentTo(double encoderGoal) {
        if ((ElevatorEncoder.getPosition()) < (encoderGoal - rangeOffset)) {
            this.goUp();
            return false;
        } else if ((ElevatorEncoder.getPosition()) > (encoderGoal + rangeOffset)) {
            this.goDown();
            return false;
        } else {
            this.stop();
            return true;
        }
    }

//This command is to be used in the AUTONOMOUS period because this returns only TRUE or FALSE values
    public boolean lcwentTo(double lcGoal) {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            System.out.println("The target is " + measurement.distance_mm + "mm away!");
        } else {
            System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
            // You can still use distance_mm in here, if you're ok tolerating a clamped
            // value or an unreliable measurement.
        }
        if ((measurement != null)){
        if ((measurement.distance_mm) < (lcGoal - lcrangeOffset)) {
            this.goUp();
            return false;
        } else if ((measurement.distance_mm) >= (lcGoal + lcrangeOffset) && (measurement.distance_mm) >= RobotConstants.lcSlowZone) {
            this.goDown();
            return false;
        } else if ((measurement.distance_mm) < (lcGoal + lcrangeOffset) && (measurement.distance_mm) < RobotConstants.lcSlowZone) {
         this.goDownSlow();
         return false;
        } else {
            this.stop();
            return true;
        }
    }
        else {
            this.stop();
            return false;
        }
        }

//Here we name the commands without the use of sensors (Encoder or LaserCAN distance reader)
    public void goUp() {
        ElevatorMotor.set(RobotConstants.ElevatorUpSpeed);
    }

    public void goDown() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if(measurement != null){
        if ((measurement.distance_mm) >= RobotConstants.lcSlowZone)
        ElevatorMotor.set(RobotConstants.ElevatorDownSpeed);
        else
        ElevatorMotor.set(RobotConstants.ElevatorDownSlowSpeed);
    }
    else
    ElevatorMotor.set(RobotConstants.ElevatorDownSpeed);
}

    public void goDownSlow() {
        ElevatorMotor.set(RobotConstants.ElevatorDownSpeed * 0.33);
    }

    public void stop() {
        ElevatorMotor.stopMotor();
    }

//This is for AUTONOMOUS and only a check - no effect on what the motor does
    public boolean encoderCheck(double distance) {
        if (ElevatorEncoder.getPosition() == distance) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        //You need the following code so the LaserCAN continues to read values
        LaserCan.Measurement measurement = lc.getMeasurement();
        //This puts a number on the Smart Dashboard, the big screen which pops up with the FRC Driver Station
        SmartDashboard.putNumber("Elevator Encoder", (ElevatorEncoder.getPosition()));
        SmartDashboard.putNumber("Elevatore Distance mm", (measurement.distance_mm));
    }
}