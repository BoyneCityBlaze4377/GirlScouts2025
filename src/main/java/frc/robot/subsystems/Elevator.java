package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.AutoAimConstants.Position;

public class Elevator extends SubsystemBase {
  private final SparkMax elevatorMotor;
  private final PIDController elevatorController;
  private final SparkMaxConfig elevatorMotorConfig;
  private final RelativeEncoder elevatorEncoder;

  private final double dH;

  private boolean locked, atPos;
  private double elevatorSpeed;
  private String positionStatusString;
  private Position currentPosition;

  private final GenericEntry elevatorHeight, elevatorSpeedSender, upperLimit, lowerLimit, 
                             positionStatusSender, lockedSender, atPositionSender;

  /** Creates a new Elevator. */
  public Elevator() {
    /** Motor Stuff */
    elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);
    elevatorMotorConfig = new SparkMaxConfig();
    elevatorEncoder = elevatorMotor.getEncoder();
    configMotorControllerDefaults();

    /** PIDController */
    elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    elevatorController.setTolerance(ElevatorConstants.kTolerance);

    /** Initializing other stuff */
    dH = ElevatorConstants.startingHeight - elevatorEncoder.getPosition();

    positionStatusString = "At floor";
    currentPosition = Position.floor;
    elevatorSpeed = 0;
    atPos = false;
    locked = false;

    /** DashBoard initialization */
    elevatorHeight = IOConstants.TeleopTab.add("Elevator Height", 0)
                                         .withWidget("Number Bar")
                                         .withProperties(Map.of("min_value", 0,
                                                                "max_value", 220,
                                                                "divisions", 11,
                                                                "orientation", "vertical"))
                                         .getEntry();
    elevatorSpeedSender = IOConstants.DiagnosticTab.add("Elevator Speed", 0)
                                                   .withWidget("Number Slider")
                                                   .withProperties(Map.of("min_value", -1, "max_value", 1))
                                                   .getEntry();
    upperLimit = IOConstants.DiagnosticTab.add("At Upper Limit?", false)
                                          .withWidget("Boolean Box").getEntry();
    lowerLimit = IOConstants.DiagnosticTab.add("At Lower Limit?", true)
                                          .withWidget("Boolean Box").getEntry();
    positionStatusSender = IOConstants.TeleopTab.add("Position", positionStatusString)
                                               .withWidget("Text Display").getEntry();
    lockedSender = IOConstants.DiagnosticTab.add("Elevator locked", false)
                                            .withWidget("Boolean Box").getEntry();
    atPositionSender = IOConstants.TeleopTab.add("At Position?", atPos)
                                            .withWidget("Boolean Box").getEntry();
  }

  @Override
  public void periodic() {
    /** Move the elevator unless at a limit */
    // if (atUpperLimit()) {
    //   elevatorMotor.set(-ElevatorConstants.correctionSpeed);
    // } else if (atLowerLimit()) {
    //   elevatorMotor.set(ElevatorConstants.correctionSpeed);
    // } else {
    //   elevatorMotor.set(elevatorSpeed);
    // }
    
    /** DashBoard updates */
    elevatorHeight.setDouble(getEncoderVal());
    elevatorSpeedSender.setDouble(elevatorSpeed);
    upperLimit.setBoolean(atUpperLimit());
    lowerLimit.setBoolean(atLowerLimit());
    positionStatusSender.setString(positionStatusString);
    lockedSender.setBoolean(locked);
    atPositionSender.setBoolean(atPos);
  }

  /** 
   * Set the speed for the elevator to move at
   * 
   * @param speed The speed for the elevator to move at
   */
  public void set(double speed) {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = speed;
    locked = false;
  }

  /** Move the elevator up */
  public void up() {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = ElevatorConstants.upSpeed;
    locked = false;
  }

  /** Move the elevator down */
  public void down() {
    if (!atLowerLimit() && !atUpperLimit()) elevatorSpeed = ElevatorConstants.downSpeed;
    locked = false;
  }

  /** Lock the elevator */
  public void lockElevator() {
    elevatorSpeed = ElevatorConstants.lockSpeed;
    locked = true;
  }

  /** Stop the elevator */
  public void stop() {
    elevatorSpeed = 0;
    locked = false;
  }

  /** @return The value of the elevator's encoder */
  public double getEncoderVal() {
    return elevatorEncoder.getPosition() + dH;
  }
  
  /** Reset the position of the elevator's encoder */
  public void zeroEncoder() {
    elevatorEncoder.setPosition(0);
  }
  
  /** Set the default configuration of the elevator's motor */
  private void configMotorControllerDefaults() {
    elevatorMotorConfig.inverted(false);
    elevatorMotorConfig.idleMode(IdleMode.kBrake);
    elevatorMotorConfig.voltageCompensation(ElevatorConstants.voltageComp);
    elevatorMotorConfig.encoder.positionConversionFactor(ElevatorConstants.conversionFactor);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  /** Set the setpoint of the elevator's PIDController
   * 
   * @param setPoint The desired height of the elevator
   */
  public void setSetpoint(double setPoint) {
    elevatorController.setSetpoint(setPoint);
  }

  /** Moves the elevator based on PID output */
  public void PIDMove() {
    elevatorSpeed = MathUtil.clamp(elevatorController.calculate(getEncoderVal()), 
                    ElevatorConstants.maxDownSpeed, ElevatorConstants.maxUpSpeed);
    locked = false;
  }

  /** @return Whether or not the elevator is at its PID setpoint */
  public boolean atSetpoint() {
    return elevatorController.atSetpoint();
  }

  /** @return Whether or not the elevator is at its upper limit */
  public boolean atUpperLimit() {
    return getEncoderVal() >= ElevatorConstants.upperLimit;
  }

    /** @return Whether or not the elevator is at its lower limit */
  public boolean atLowerLimit() {
    return getEncoderVal() <= ElevatorConstants.lowerLimit;
  }

  /** 
   * Set the value of the elevator's position String to go on the DashBoard
   * 
   * @param positionString The String with the desired message to be posted
   */
  public void setPositionString(String positionString) {
    positionStatusString = positionString;
  }
  
  /**
   * Set the current/desired position of the elevator
   * 
   * @param position  The current/desired position
   */
  public void setCurrentPosition(Position position) {
    currentPosition = position;
  }

  /** @return The current/desired position of the elevator */
  public Position getCurrentPosition() {
    return currentPosition;
  }

  /** 
   * Set whether or not the elevator is at its desired position
   * 
   * @param atPosition Whether or not the elevator is at its desired position
   */
  public void setAtPos(boolean atPosition) {
    atPos = atPosition;
  }
}
