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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AffectorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SensorConstants;

public class CoralAffector extends SubsystemBase {
  private final SparkMax coralAffector, coralWrist;
  private final SparkMaxConfig coralAffectorConfig, coralWristConfig;
  private final RelativeEncoder wristEncoder;

  private final DigitalInput coralDetector, wristResetter;
  private final GenericEntry wristValSender, hasCoralSender, lockedSender, wristSpeedSender;
  private final PIDController wristController;

  private final double dA;

  private boolean locked, override;
  private double wristSpeed;

  /** Creates a new CoralAffector. */
  public CoralAffector() {
    /** Motors */
    coralAffector = new SparkMax(AffectorConstants.coralAffectorID, MotorType.kBrushless);
    coralWrist = new SparkMax(AffectorConstants.coralWristID, MotorType.kBrushless);

    coralAffectorConfig = new SparkMaxConfig();
    coralWristConfig = new SparkMaxConfig();

    wristEncoder = coralWrist.getEncoder();

    configMotorControllerDefaults();

    /** PIDController */
    wristController = new PIDController(AffectorConstants.coralWristKP, 
                                        AffectorConstants.coralWristKI, 
                                        AffectorConstants.coralWristKD);
    wristController.setTolerance(AffectorConstants.coralWristKTolerance);

    coralDetector = new DigitalInput(SensorConstants.coralBreakID);
    wristResetter = new DigitalInput(SensorConstants.wristResetterID);

    /** DashBoard Initialization */
    wristValSender = IOConstants.TeleopTab.add("Wrist Encoder Degrees", getWristDegrees())
                                              .withWidget("Radial Gauge")
                                              .withProperties(Map.of("start_angle", 180, "end_angle", 0,
                                                                     "min_value", 0, "max_value", 180, 
                                                                     "number_of_labels", 4, "show_pointer", false))
                                              .getEntry();
    hasCoralSender = IOConstants.TeleopTab.add("HasCoral", hasCoral()).withWidget("Boolean Box").getEntry();
    lockedSender = IOConstants.DiagnosticTab.add("CoralWrist Locked", false)
                                            .withWidget("Boolean Box").getEntry();
    wristSpeedSender = IOConstants.DiagnosticTab.add("CoralWrist Speed", wristSpeed).getEntry();

    /** Other Initializations */                                        
    dA = AffectorConstants.startingAngle - wristEncoder.getPosition() * AffectorConstants.coralWristConversionFactor;
    wristSpeed = 0;
    override = false;
  }
  
  @Override
  public void periodic() {
    /** Move Wrist */
    //coralWrist.set(wristSpeed);

    /** DashBoard Updates */
    wristValSender.setDouble(getWristDegrees());
    hasCoralSender.setBoolean(hasCoral());
    lockedSender.setBoolean(locked);
    wristSpeedSender.setDouble(wristSpeed);

    if (wristResetter.get() == false) {
      resetWristEncoder();
    }

    SmartDashboard.putBoolean("Wrist Reset", wristResetter.get());
  }

  public void collect() {
    coralAffector.set(AffectorConstants.coralAffectorInSpeed);
  }

  /** Spit coral out */
  public void eject() {
    if (getWristDegrees() > AffectorConstants.wristScoringThreshold) {
      coralAffector.set(AffectorConstants.coralAffectorOutSpeed);
    } else {
      coralAffector.set(-AffectorConstants.coralAffectorOutSpeed);
    }
  }

  /** Stop the affector motor */
  public void stopAffector() {
    coralAffector.set(0);
  }

  /**
   * Set the setpoint of the wrist's PIDController
   * 
   * @param setPoint
   */
  public void setSetpoint(double setPoint) {
    wristController.setSetpoint(setPoint);// + (setPoint == AffectorConstants.coralWristDefaultPos ? 0 : 3));
  }

  /** Move the wrist based on PID output */
  public void PIDMoveWrist() {
    wristSpeed = (MathUtil.clamp(wristController.calculate(getWristDegrees()), 
                  AffectorConstants.maxCoralWristDownSpeed, AffectorConstants.maxCoralWristUpSpeed));
    locked = false;
  }

  /** @return Whether or not the wrist is at its desired angle */
  public boolean atSetpoint() {
    return wristController.atSetpoint();
  }

  /** 
   * Set the speed of the wrist's motor to override set positions
   * 
   * @param speed The speed to move the wrist at
   */
  public void overrideWrist(double speed) {
    wristSpeed = speed;
    locked = false;
  }

  /** Stop the wrist */
  public void stopWrist() {
    wristSpeed = 0;
    locked = false;
  }

  public void lockWrist() {
    wristSpeed = .05;
  }

  /** @return The value, in degrees-ish, of the wrist's encoder */
  public double getWristDegrees() {
    return wristEncoder.getPosition() * AffectorConstants.coralWristConversionFactor + dA;
  }
  
  /** Set the position of the wrist's encoder to zero */
  public void resetWristEncoder() {
    wristEncoder.setPosition(AffectorConstants.coralWristDefaultPos);
  }

  /** Set the default configuration of the CoralAffector's motor controllers */
  private void configMotorControllerDefaults() {
    coralAffectorConfig.inverted(true);
    coralAffectorConfig.idleMode(IdleMode.kCoast);

    coralWristConfig.inverted(false);
    coralWristConfig.idleMode(IdleMode.kBrake);
    coralWristConfig.voltageCompensation(AffectorConstants.voltageComp);

    configMotorControllers();
  }

  /** Update the motors controllers to their respective configurations */
  private void configMotorControllers() {
    coralAffector.configure(coralAffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralWrist.configure(coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** @return Whether or not the CoralAffector is holding a piece of coral */
  public boolean hasCoral() {
    return !coralDetector.get();
  }

  public void setIsOverride(boolean isOverride) {
    override = isOverride;
  }
}
