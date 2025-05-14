package frc.robot.subsystems;

import java.io.IOException;
import java.util.Map;
import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.Lib.AdvancedPose2D;
import frc.Lib.Elastic;
import frc.Lib.LimelightHelpers;
import frc.Lib.Elastic.Notification;
import frc.Lib.Elastic.Notification.NotificationLevel;
import frc.Lib.LimelightHelpers.PoseEstimate;
import frc.Lib.Terms.*;
import frc.Lib.Term;

import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.*;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SwerveConstants;

import choreo.trajectory.SwerveSample;

public class DriveTrain extends SubsystemBase {
  private final SwerveModule m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private final AHRS m_gyro;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d estimateField;
  private AprilTagFieldLayout fieldLayout;

  private final Elevator m_elevator;

  private final GenericEntry robotHeading, xSpeedSender, 
                             ySpeedSender, omegaSender, matchTime,
                             desPoseSender, desAlignSender, desStationSender,
                             atDesPose, orientationSender;

  private final PIDController xController = new PIDController(AutoAimConstants.transkP,
                                                              AutoAimConstants.transkI,
                                                              AutoAimConstants.transkD);
  private final PIDController yController = new PIDController(AutoAimConstants.transkP,
                                                              AutoAimConstants.transkI,
                                                              AutoAimConstants.transkD);
  private final PIDController headingController = new PIDController(AutoAimConstants.turnkP,
                                                                    AutoAimConstants.turnkI,
                                                                    AutoAimConstants.turnkD);

  private final String cameraName;
  private Alliance m_alliance;

  private AdvancedPose2D desiredPose, initialPose = AutonConstants.initialPoseRedBack;
  private Alignment desiredAlignment;
  private ReefStation estimatedStation;

  private boolean fieldOrientation = true, isLocked = false, slow = false, 
                  isBrake = true, autonInRange = false, useScalers = false, 
                  straightDriveBackwards = false, isBlue = true, notified = false;

  private double tx, ty, ta, tID, speedScaler, heading, x, y, omega,  elevatorHeight;
  private int periodicTimer = 1;
    
  /** Creates a new DriveTrain. */
  public DriveTrain(Elevator elevator, String limelightName) {
    /* Swerve Modules */
    m_frontLeft = new SwerveModule("frontLeft", ModuleConstants.frontLeftDriveMotorPort, 
                                                     ModuleConstants.frontLeftTurningMotorPort,
                                                     ModuleConstants.frontLeftTurningEncoderPort, 
                                                     ModuleConstants.frontLeftDriveMotorReversed,
                                                     ModuleConstants.frontLeftTurningMotorReversed, 
                                                     ModuleConstants.frontLeftAnalogEncoderOffset, 
                                                     ModuleConstants.frontLeftAbsReversed);

    m_frontRight = new SwerveModule("frontRight", ModuleConstants.frontRightDriveMotorPort, 
                                                       ModuleConstants.frontRightTurningMotorPort,
                                                       ModuleConstants.frontRightTurningEncoderPort, 
                                                       ModuleConstants.frontRightDriveMotorReversed,
                                                       ModuleConstants.frontRightTurningMotorReversed, 
                                                       ModuleConstants.frontRightAnalogEncoderOffset, 
                                                       ModuleConstants.frontRightAbsReversed);

    m_backLeft = new SwerveModule("backLeft", ModuleConstants.backLeftDriveMotorPort, 
                                                   ModuleConstants.backLeftTurningMotorPort,
                                                   ModuleConstants.backLeftTurningEncoderPort, 
                                                   ModuleConstants.backLeftDriveMotorReversed,
                                                   ModuleConstants.backLeftTurningMotorReversed, 
                                                   ModuleConstants.backLeftAnalogEncoderOffset, 
                                                   ModuleConstants.backLeftAbsReversed);

    m_backRight = new SwerveModule("backRight", ModuleConstants.backRightDriveMotorPort, 
                                                     ModuleConstants.backRightTurningMotorPort,
                                                     ModuleConstants.backRightTurningEncoderPort, 
                                                     ModuleConstants.backRightDriveMotorReversed,
                                                     ModuleConstants.backRightTurningMotorReversed, 
                                                     ModuleConstants.backRightAnalogEncoderOffset, 
                                                     ModuleConstants.backRightAbsReversed);

    brakeAll();
    resetEncoders();

    // DriveTrain GyroScope
    m_gyro = new AHRS(NavXComType.kUSB1);
    m_gyro.setAngleAdjustment(initialPose.getRotation().getDegrees());
    heading = -MathUtil.inputModulus(m_gyro.getYaw() - initialPose.getHeadingDegrees(), -180, 180);

    /* Pose Estimation */
    estimateField = new Field2d();
    poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.driveKinematics, 
                                                 getHeading(), 
                                                 getSwerveModulePositions(), 
                                                 initialPose,
                                                 AutoAimConstants.poseEstimateOdometryStdDev,
                                                 AutoAimConstants.poseEstimateVisionStdDev);   
    setInitialPose(initialPose);
    setInitialPose(initialPose);
    setInitialPose(initialPose);
    estimateField.setRobotPose(initialPose);

    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      fieldLayout.setOrigin(new Pose3d());
    } catch (IOException e) {}

    /** AutoAim */
    // Determine desired ReefStation
    estimatedStation = isBlue ? AutoAimConstants.blueReefStationFromAngle.get(getEstimatedStationAngle()) 
                              : AutoAimConstants.redReefStationFromAngle.get(getEstimatedStationAngle());
    // Calculate desired pose
    desiredPose = isBlue ? AutoAimConstants.redReef.get(estimatedStation) 
                         : AutoAimConstants.redReef.get(estimatedStation);
    desiredAlignment = Alignment.left;

    // BezierPath bezier = new BezierPath(FieldConstants.blueReefCenterPos,
    //                                    FieldConstants.blueReefCenterPos,
    //                                    FieldConstants.blueCenterStartAlgae.horizontallyFlip());
    // ArrayList<AdvancedPose2D> path = bezier.outputPath(50);

    // for (int i = 0; i < path.toArray().length; i++) {
    //   estimateField.getObject("Point" + i).setPose(path.get(i));
    // }

    // for (int k = 0; k < bezier.getControlPoints().length; k++) {
    //   estimateField.getObject("Control" + k).setPose(bezier.getControlPoints()[k]);
    // }

    /* DashBoard Initialization */
    robotHeading = IOConstants.TeleopTab.add("Robot Heading", heading)
                                        .withWidget("Gyro")
                                        .withProperties(Map.of("counter_clockwise_positive", true))
                                        .getEntry();
    xSpeedSender = IOConstants.TeleopTab.add("xSpeed", 0)
                                        .withWidget("Number Slider")
                                        .withProperties(Map.of("min_value", -1, "max_value", 1))
                                        .getEntry();
    ySpeedSender = IOConstants.TeleopTab.add("ySpeed", 0)
                                        .withWidget("Number Slider")
                                        .withProperties(Map.of("min_value", -1, "max_value", 1))
                                        .getEntry();
    omegaSender = IOConstants.TeleopTab.add("rot", 0)
                                       .withWidget("Number Slider")
                                       .withProperties(Map.of("min_value", -1, "max_value", 1))
                                       .getEntry();
    matchTime = IOConstants.TeleopTab.add("Match Time", 15)
                                     .withWidget("Match Time")
                                     .withProperties(Map.of("red_start_time", 10, "yellow_start_time", 20))
                                     .getEntry();
    desPoseSender = IOConstants.DiagnosticTab.add("Desired Pose", initialPose.toString())
                                             .withWidget("Text Display")
                                             .getEntry();
    desAlignSender = IOConstants.DiagnosticTab.add("Desired Alignment", desiredAlignment.toString())
                                              .withWidget("Text Display")
                                              .getEntry();
    desStationSender = IOConstants.DiagnosticTab.add("Estimated Reefstation", estimatedStation.toString())
                                                .withWidget("Text Display")
                                                .getEntry();
    atDesPose = IOConstants.TeleopTab.add("At Desired Pose", false)
                                     .withWidget("Boolean Box")
                                     .getEntry();
    orientationSender = IOConstants.TeleopTab.add("Field Oriented?", true)
                                             .withWidget("Boolean Box")
                                             .getEntry();
    SmartDashboard.putData("Field Position", estimateField);

    //SwerveDrive Widget
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_backLeft.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> m_backRight.getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> m_gyro.getRotation2d().getRadians(), null);
      }
    });

    /* PID Controllers */
    xController.setTolerance(AutoAimConstants.transkTolerance);
    yController.setTolerance(AutoAimConstants.transkTolerance);
    headingController.setTolerance(AutoAimConstants.turnkTolerance);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    /* LimeLight Initialization */
    cameraName = limelightName;

    LimelightHelpers.SetRobotOrientation(cameraName, initialPose.getRotation().getDegrees(), 
                                         0, 0, 0, 0, 0);

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camera_robotspace_set")
    //                 .setDoubleArray(SensorConstants.limelightRobotSpacePose);

    fieldOrientation = true;
    isLocked = false;
    useScalers = false;
    slow = false;

    m_elevator = elevator;
    elevatorHeight = m_elevator.getEncoderVal();

    x = 0;
    y = 0;
    omega = 0;

    speedScaler = DriveConstants.speedScaler;

    m_alliance = Alliance.Blue;

    SwerveSample s;
    
  }

  @Override
  public void periodic() {
    if (periodicTimer >= 10) {
      m_frontLeft.update();
      m_frontRight.update();
      m_backLeft.update();
      m_backRight.update();

      periodicTimer = 0;
    }

    heading = -MathUtil.inputModulus(m_gyro.getYaw() - initialPose.getHeadingDegrees(), -180, 180);

    /* Pose Estimation */
    poseEstimator.update(getHeading(), getSwerveModulePositions());
    if (getPoseEstimate().get().tagCount >= 1) {
      poseEstimator.addVisionMeasurement(getPoseEstimate().get().pose, 
                                         getPoseEstimate().get().timestampSeconds);
    }

    // Field Displaying
    estimateField.setRobotPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), getHeading()));
    estimateField.getObject("desired").setPose(desiredPose.withReefAlignment(desiredAlignment, false));
    // estimateField.getObject("heading").setPose(FieldConstants.fieldLength / 2, FieldConstants.fieldWidth / 2, getHeading());

    /** Dashboard Posting */
    robotHeading.setDouble(getHeading().getDegrees());
    desPoseSender.setString(desiredPose.toString());
    desAlignSender.setString(desiredAlignment.toString());
    desStationSender.setString(estimatedStation.toString());
    atDesPose.setBoolean(atSetpoints());
    matchTime.setDouble(DriverStation.getMatchTime());
    orientationSender.setBoolean(fieldOrientation);

    // Tell if gyro disconnects
    if (!m_gyro.isConnected() && !notified) {
      Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "NAVX", "GYRO DISCONNECTED")
                                               .withDisplaySeconds(10));
      notified = true;
    } else if (notified && m_gyro.isConnected()) {
      Elastic.sendNotification(new Notification(NotificationLevel.INFO, "NAVX", "GYRO RECONNECTED")
                                               .withDisplaySeconds(5));
      notified = false;
    }

    straightDriveBackwards = desiredAlignment == Alignment.center;

    /* LIMELIGHT */
    NetworkTable table = NetworkTableInstance.getDefault().getTable(SensorConstants.limeLightName);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);
    tID = table.getEntry("fID").getDouble(0);

    LimelightHelpers.SetRobotOrientation(cameraName, getHeading().getDegrees(), 
                                         0, 0, 0, 0, 0);

    // Determine desired ReefStation
    estimatedStation = isBlue ? AutoAimConstants.blueReefStationFromAngle.get(getEstimatedStationAngle()) 
                              : AutoAimConstants.redReefStationFromAngle.get(getEstimatedStationAngle());
    // Calculate desired pose
    desiredPose = isBlue ? AutoAimConstants.blueReef.get(estimatedStation) 
                         : AutoAimConstants.redReef.get(estimatedStation);
    
    // Update random stuff
    autonInRange = Math.hypot(xController.getError(), yController.getError()) <= AutonConstants.inRangeThreshold;

    isBrake = m_frontLeft.getIdleMode() == IdleMode.kBrake;

    isBlue = m_alliance == Alliance.Blue;

    elevatorHeight = m_elevator.getEncoderVal();

    // Drive Robot
    rawDrive(x , y, omega, fieldOrientation, useScalers);

    periodicTimer++;

    SmartDashboard.putBoolean("INRANGE", autonInRange);
    SmartDashboard.putBoolean("ATSET", atSetpoints());

    if (DriverStation.isTeleop()) setPoseSetpoints(desiredPose.withReefAlignment(desiredAlignment, false));
  }

  /**
   * The function that sets the raw speeds of the {@link DriveTrain}
   * 
   * @param xSpeed Speed on the x-axis in Meters per Second
   * @param ySpeed Speed on the y-axis in Meters per Second
   * @param omega Rotational speed in Radians per Second
   * @param fieldRelative Whether to drive field oriented or not
   * @param scale Whether to use Elevator Height Scalers
   */
  private void rawDrive(double xSpeed, double ySpeed, double omega, boolean fieldRelative, boolean scale) {
    if (scale) {
      xSpeed *= calcTransHeightScaler(elevatorHeight);
      ySpeed *= calcTransHeightScaler(elevatorHeight);
      omega *= calcRotHeightScaler(elevatorHeight);
    }

    var swerveModuleStates = SwerveConstants.driveKinematics.toSwerveModuleStates(fieldOrientation
                           ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, m_gyro.getRotation2d()) 
                           : new ChassisSpeeds(xSpeed, ySpeed, omega));

    setModuleStates(swerveModuleStates, (xSpeed == 0 && ySpeed == 0 && omega == 0));

    xSpeedSender.setDouble(xSpeed);
    ySpeedSender.setDouble(ySpeed);
    omegaSender.setDouble(-omega);
  }


  /**
   * Drive the robot back and forth
   * 
   * @param xSpeed The speed at which to drive
   */
  public void straightDrive(double xSpeed) {
    x = xSpeed * (straightDriveBackwards ? -1 : 1) * (isBlue ? 1 : -1);
    y = 0;
    omega = 0;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void teleopDrive(double xSpeed, double ySpeed, double rot) {
    rot = Math.pow(rot, 3);

    x = xSpeed * DriveConstants.maxSpeedMetersPerSecond * speedScaler * (fieldOrientation ? (isBlue ? 1 : -1) : 1);
    y = ySpeed * DriveConstants.maxSpeedMetersPerSecond * speedScaler * (fieldOrientation ? (isBlue ? 1 : -1) : 1);
    omega = rot * DriveConstants.maxRotationSpeedRadiansPerSecond * speedScaler;

    setUseScalers(true);
  }

  /**
   * Drive the robot autonomously.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param scale Whether to use Height Speed Scalers
   */
  public void autonDrive(double xSpeed, double ySpeed, double rot, boolean scale) {
    brakeAll();
    x = xSpeed;
    y = ySpeed;
    omega = rot;
    setUseScalers(scale);
  }

  /**
   * Drive based on a ChassisSpeeds object
   * 
   * @param speeds The desired ChassisSpeeds of the {@link DriveTrain}
   */
  public void chassisSpeedDrive(ChassisSpeeds speeds) {
    brakeAll();
    x = speeds.vxMetersPerSecond;
    y = speeds.vyMetersPerSecond;
    omega = speeds.omegaRadiansPerSecond;
    setUseScalers(true);
  }

  public void choreoDrive(SwerveSample sample) {
      // Generate the next speeds for the robot
      ChassisSpeeds speeds = new ChassisSpeeds(
          sample.vx + xController.calculate(getPose().getX(), sample.x),
          sample.vy + yController.calculate(getPose().getY(), sample.y),
          sample.omega + headingController.calculate(getPose().getRotation().getRadians(), sample.heading)
      );





      
      // Apply the generated speeds
      chassisSpeedDrive(speeds);
      setOrientation(true);
  }

  /**
   * Set the setpoints of all drive PIDControllers
   * 
   * @param xSetpoint The setpoint for the controller of the x-coordinate
   * @param ySetpoint The setpoint for the controller of the y-coordinate
   * @param headingSetpoint The setpoint for the controller of the robot's heading
   */
  public void setPIDSetpoints(double xSetpoint, double ySetpoint, double headingSetpoint) {
    xController.setSetpoint(xSetpoint);
    yController.setSetpoint(ySetpoint);
    headingController.setSetpoint(headingSetpoint);
  }

  /**
   * Set the setpoints of all drive PIDControllers from a desired position on the field
   * 
   * @param pose The desired position
   */
  public void setPoseSetpoints(AdvancedPose2D pose) {
    setPIDSetpoints(pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  /** Set the setpoints of all drive PIDControllers based on the currently determined AutoAimDrive pose */
  public void setAutoAimDriveSetpoints() {
    setPoseSetpoints(getAutoAimPose());
  }

  /** Drive the robot based on PIDController outputs */
  public void PIDDrive() {
    x = MathUtil.clamp(xController.calculate(getPose().getX()), -AutoAimConstants.maxPIDDriveSpeed, 
                                                                 AutoAimConstants.maxPIDDriveSpeed);
    y = MathUtil.clamp(yController.calculate(getPose().getY()), -AutoAimConstants.maxPIDDriveSpeed, 
                                                                 AutoAimConstants.maxPIDDriveSpeed);
    omega = MathUtil.clamp(headingController.calculate(getHeading().getRadians()),
                                                      -AutoAimConstants.maxPIDRot, 
                                                       AutoAimConstants.maxPIDRot);

    setUseScalers(false);
  }

  /** @return Whether or not the robot is at its desired position based on PIDController setpoints and tolerances */
  public boolean atSetpoints() {
    return xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isNeutral) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0], isNeutral);
    m_frontRight.setDesiredState(desiredStates[1], isNeutral);
    m_backLeft.setDesiredState(desiredStates[2], isNeutral);
    m_backRight.setDesiredState(desiredStates[3], isNeutral);
  }

  /** @return An array of the modules' positions */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(),
                                       m_backLeft.getPosition(), m_backRight.getPosition()};
  }

  /**
   * Sets the positions of the SwerveModules' angle motors.
   *  
   * @param desiredStates The state containing the desired angle for the angle motor;
   */
  public void setModuleAngles(SwerveModuleState[] desiredStates) {
    m_frontLeft.setAngle(desiredStates[0], false);
    m_frontRight.setAngle(desiredStates[1], false);
    m_backLeft.setAngle(desiredStates[2], false);
    m_backRight.setAngle(desiredStates[3], false);
  }

  /** Sets the pose of the robot to be locked: all modules' angles form an X */
  public void lockPose() {
    m_frontLeft.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
    m_frontRight.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
    m_backLeft.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
    m_backRight.setLockedState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
  }

  /** @return The current ChassisSpeeds of the {@link DriveTrain} */
  public ChassisSpeeds getChassisSpeeds() {
    return fieldOrientation ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, m_gyro.getRotation2d()) 
                            : new ChassisSpeeds(-x, -y, omega);
  }

  /** 
   * Sets whether or not the {@link DriveTrain} is in the Locked Pose. 
   *
   * @param locked Whether or not the robot is in the Locked Pose.
   */
  public void setLocked(boolean locked) {
    isLocked = locked;
  }

  /**
   * Scales the max speed of the robot.
   * 
   * @param scaler What to multiply the SpeedScaler by.
   */
  public void scaleSpeedScaler(double scaler) {
    speedScaler *= scaler;
  }

  /** @return The current SpeedScaler of the {@link DriveTrain} */
  public synchronized double getSpeedScaler() {
    return speedScaler;
  }

  /**
   * Sets whether or not the robot is in SlowMode.
   * 
   * @param isSlow True for slow, false for regular.
   */
  public void setIsSlow(boolean isSlow) {
    slow = isSlow;
  }

  /**
   * Set the angle offset of the drive gyroscope
   * 
   * @param offsetDeg The desired offset for the gyro
   */
  public void setOffset(double offsetDeg) {
    m_gyro.setAngleAdjustment(offsetDeg);
  }

  /** @return The pose */
  public synchronized Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

   /** Switches the idle modes of all modlues' drive motors */
   public void switchBrake() {
    if (isBrake) {
      coastAll();
    } else {
      brakeAll();
    }
  }

  /** Sets all idle modes to Brake */
  public void brakeAll() {
    m_frontLeft.brake();
    m_frontRight.brake();
    m_backLeft.brake();
    m_backRight.brake();
  }

  /** Sets all idle modes to Coast */
  public void coastAll() {
    m_frontLeft.coast();
    m_frontRight.coast();
    m_backLeft.coast();
    m_backRight.coast();
  }

  /** @return The current IdleMode of the {@link DriveTrain} */
  public synchronized IdleMode getIdleMode() {
    return m_frontLeft.getIdleMode();
  }

  /**
   * Sets the IdleMode of the {@link DriveTrain}
   * 
   * @param mode The desired IdleMode for the {@link DriveTrain}
   */
  public synchronized void setIdleMode(IdleMode mode) {
    m_frontLeft.setIdleMode(mode);
    m_frontRight.setIdleMode(mode);
    m_backLeft.setIdleMode(mode);
    m_backRight.setIdleMode(mode);
  }

  /** Stops drive motors for all modules */
  public void stop() {
    x = 0;
    y = 0;
    omega = 0;
  }

  /** Reset encoders of all modules */
  public synchronized void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Switches whether or not the robot drives field oriented */
  public synchronized void switchOrientation() {
    if (fieldOrientation) {
      fieldOrientation = false;
    } else {
      fieldOrientation = true;
    }
  }

  /**
   * Sets whether or not the robot should drive Field Oriented.
   * 
   * @param isFieldOriented True for Field Oriented, false for Robot Oriented.
   */
  public synchronized void setOrientation(boolean isFieldOriented) {
    fieldOrientation = isFieldOriented;
  }

  /** @return Whether or not the {@link DriveTrain} is field oriented */
  public synchronized boolean isFieldOriented() {
    return fieldOrientation;
  }

  /**
   * Set whether or not to use the Height Speed Scalers
   * 
   * @param scale
   */
  public synchronized void setUseScalers(boolean scale) {
    useScalers = scale;
  }

  /**
   * Sets the offset of the gyro.
   * 
   * @param offsetDegrees The number of degrees to offset by.
   */
  public void setGyroOffset(double offsetDegrees) {
    m_gyro.setAngleAdjustment(offsetDegrees);
  }

  /** Zeroes the heading of the robot */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** @return The filtered robot heading as a {@link Rotation2d} */
  public synchronized Rotation2d getHeading() {
    return Rotation2d.fromDegrees(heading);
  }

  /** @return The unfiltered heading of the robot */
  public synchronized double getRawHeading() {
    return m_gyro.getAngle();
  }

   /** @return The roll of the gyro */
   public double getRoll(){
    return m_gyro.getRoll();
  }
  
  /** @return The pitch of the gyro */
  public double getPitch(){
    return m_gyro.getPitch();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  /** @return The average distance of all modules */
  public double getAverageDistance(){
    return (Math.abs(m_frontLeft.getDistance()) + Math.abs(m_frontRight.getDistance()) +
            Math.abs(m_backLeft.getDistance()) + Math.abs(m_backRight.getDistance())) / 4;
    
  }

  /** @return Whether or not the robot is close enough to bring up the elevator in auton */
  public boolean getInRange() {
    return autonInRange;
  }

  /** Set whether or not the robot is close enough to bring up the elevator in auton */
  public void setInRange(boolean isInRange) {
    autonInRange = isInRange;
  }

  /**
   * Calculate the amount by which to scale the {@link DriveTrain}'s translational speed based on the elevator's height
   * 
   * @param height The current height of the elevator
   * @return The calculated translational speed scaler
   */
  public double calcTransHeightScaler(double height) {
    return ((DriveConstants.minDriveSpeed - DriveConstants.maxDriveSpeed) / 
             Math.pow(ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit, 2)) * 
             Math.pow(height - ElevatorConstants.lowerLimit, 2) + DriveConstants.maxDriveSpeed;
  }

  /**
   * Calculate the amount by which to scale the {@link DriveTrain}'s rotational speed based on the elevator's height
   * 
   * @param height The current height of the elevator
   * @return The calculated rotational speed scaler
   */
  public double calcRotHeightScaler(double height) {
    return ((DriveConstants.minRotSpeed - DriveConstants.maxRotSpeed) / 
             Math.pow(ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit, 2)) * 
             Math.pow(height - ElevatorConstants.lowerLimit, 2) + DriveConstants.maxRotSpeed;
  }

  /**
   * Determine the estimated {@link ReefStation} angle of the {@link DriveTrain} for AutoAim
   * 
   * @return The angle of the estimated {@link ReefStation} of the {@link DriveTrain}
   */
  public double getEstimatedStationAngle() {
    double prevError = 180;
    double selectedAngle = 0;

    for (int i = 0; i < AutoAimConstants.reefStationAngles.length; i++) {
      double error = Math.abs(getHeading().getDegrees() - AutoAimConstants.reefStationAngles[i]);
      if (error < prevError){
        prevError = error;
        selectedAngle = MathUtil.inputModulus(AutoAimConstants.reefStationAngles[i] + (desiredAlignment == Alignment.center ? 180 : 0),
                                              -180, 180);
      }
    }
    return selectedAngle;
  }

  /**
   * Get the Vision Estimated Position of the robot
   * 
   * @return The Vision Estimated Position of the Robot
   */
  public Optional<PoseEstimate> getPoseEstimate() {
    return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName));
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    return ta;
  }

  public double getTargetID() {
    return tID;
  }

  /**
   * Set the desired position of the {@link DriveTrain}
   * 
   * @param pose The desired position
   */
  public synchronized void setDesiredPose(AdvancedPose2D pose) {
    desiredPose = pose;
  }

  /**
   * Set the desired alignment of the desired pose for the {@link DriveTrain}
   * 
   * @param alignment
   */
  public synchronized void setDesiredAlignment(Alignment alignment) {
    desiredAlignment = alignment;
  }

  /** @return The desired position of the {@link DriveTrain} */
  public synchronized AdvancedPose2D getDesiredPose() {
    return desiredPose;
  }

  /** @return The desired alignment of the desired pose for the {@link DriveTrain} */
  public synchronized Alignment getDesiredAlignment() {
    return desiredAlignment;
  }

  
  /** @return The current AutoAimPose of the robot */
  public synchronized AdvancedPose2D getAutoAimPose() {
    return desiredPose.withReefAlignment(desiredAlignment, false);
  }

  /**
   * Set the initial pose of the {@link DriveTrain}
   * 
   * @param pose The initial pose to be set
   */
  public synchronized void setInitialPose(AdvancedPose2D pose) {
    m_gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    m_gyro.reset();
    m_gyro.zeroYaw();
    initialPose = pose;
    heading = -MathUtil.inputModulus(m_gyro.getYaw() - initialPose.getHeadingDegrees(), -180, 180);
    poseEstimator.resetPose(new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(heading)));
  }

  public void resetPose() {
    poseEstimator.resetPose(initialPose);
  }

  /** @return The desired ReefStation of the robot */
  public synchronized ReefStation getDesiredStation() {
    return estimatedStation;
  }

  /**
   * Set the Alliance for the match
   * 
   * @param alliance The Alliance to be set
   */
  public synchronized void setAlliance(Alliance alliance) {
    m_alliance = alliance;
    isBlue = (m_alliance == Alliance.Blue);
  }

  /** @return The Alliance for the current match */
  public synchronized Alliance getAlliance() {
    return m_alliance;
  }

  /** @return The height of the elevator */
  public Position elevatorPos() {
    return m_elevator.getCurrentPosition();
  }

  public void setToVisionPos() {
    if (getPoseEstimate().get().tagCount >= 1) {
    poseEstimator.resetTranslation(getPoseEstimate().get().pose.getTranslation());
    }
  }
}