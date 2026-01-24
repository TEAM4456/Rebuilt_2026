// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ==== Liam has attempted to mark up this file. Pray for me! ====

// Imports
package frc.robot.Subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

// Imports from our code
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;

// Class declaration
public class Swerve extends SubsystemBase {
  
  // Class variables
  public Vision photonVision = new Vision(); // Represents the robot camera
  private final AHRS m_gyro; //Represents the robot gyroscope, Check to make sure SPI switch is flicked "ON" on the NavX
  public Field2d field; // Represents the compitition field
  private SwerveModule[] mSwerveMods; // Represents an array of the individual swerve modules in each corner of the robot
  private SwerveDrivePoseEstimator swerveOdometry; // Represents a combo of vision measurements and swerve drive velocity measurements
  public RobotConfig config; // 
  
  // Class constructor passing parameter "v"
  public Swerve(Vision v) {

    // Class variables getting assigned values
    photonVision = v;
    m_gyro = new AHRS(NavXComType.kMXP_SPI); 
    field = new Field2d(); 
    mSwerveMods = new SwerveModule[] { 
        new SwerveModule(0, Constants.Swerve.Mod0.constants), // Module values are pulled from our own Constants file
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    // These variables are declared for the swerveOdometry assignment statment below, but I don't know what they do
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
    var visionStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(10));

    // FROM CLASS SwerveDrivePoseEstimator DAN_F
    swerveOdometry = new SwerveDrivePoseEstimator(
        // The formating looks strange, but these are being passed as attributes for the assignment of swerveOdometry
        Constants.Swerve.swerveKinematics,
        getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    // Methods that are run every time and object is created, NOT VARIABLES!!!!!
    zeroHeading();
    Timer.delay(1.0);
    resetModulesToAbsolute();
    SmartDashboard.putData("Field", field);
    // .configFactoryDefault();    Sets somthing to the factory default, not sure what?
    
    // info on how to set up the SwerveDrivePoseEstimator can be found here: file:///C:/Users/Public/wpilib/2025/documentation/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator.html

    // This was here before, not going to bother trying to understand it
    /*
    AutoBuilder.configureHolonomic(//FROM CLASS AUTOBUILDER DAN_F
    this::getPose, // Robot pose supplier
    this::resetPose, // Method to reset odometry (will be called if your auto has
    a starting pose)
    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
    RELATIVE
    this::driveRobotRelative, // Method that will drive the robot given ROBOT
    RELATIVE ChassisSpeeds
    new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
    likely live in your Constants class
    new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI,
    Constants.Swerve.driveKD), // Translation PID constants
    new PIDConstants(5, Constants.Swerve.angleKI, Constants.Swerve.angleKD), //
    Rotation PID constants
    4, // Max module speed, in m/s
    0.4, // Drive base radius in meters. Distance from robot center to furthest
    module.
    new ReplanningConfig() //NO DECLARATION DAN_F Default path replanning config.
    /ee the API for the options here
    ),
    () -> {
    // Boolean supplier that controls when the path will be mirrored for the red
    alliance
    // This will flip the path being followed to the red side of the field.
    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    Load the RobotConfig from the GUI settings. You should probably store this in your Constants file
    */

    // This is weird, but I think it tests for a whether an error occurs in the "try" statment. If yes, the catch statment is executed and prints a log of recent robot actions
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    
    // Yay, lambda statements, not going to try writing any comments for this method
    // Need to setup in FRC PathPlanner App (GUI) before you can deploy code to robot.  Navigate to Settings in app once you open Robot Project
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI,Constants.Swerve.driveKD), // Translation PID constants
              new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI,Constants.Swerve.angleKD) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance(); // Represents which alliance we are
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }
  // Constructor ends
  
  /** Drive method passing parameters "translation", "rotation", and "isOpenLoop".
  * The method declares the array "swerveModuleStates" and sets its values to the result
  * of some code I don't know why we preform at the momment */
  public void drive(Translation2d translation, double rotation, /* boolean fieldRelative, */ boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    
    // A for each loop that sets the desired state of each swerve module on the robot
    for (SwerveModule tempMod : mSwerveMods) {
      tempMod.setDesiredState(swerveModuleStates[tempMod.moduleNumber], isOpenLoop);
    }
  }

  // Some commented code that was here before
  /*
   * Drive with field relative boolean
   * public void drive(
   * Translation2d translation, double rotation, boolean fieldRelative, boolean
   * isOpenLoop) {
   * SwerveModuleState[] swerveModuleStates =
   * Constants.Swerve.swerveKinematics.toSwerveModuleStates(
   * fieldRelative
   * ? ChassisSpeeds.fromFieldRelativeSpeeds(
   * translation.getX(), translation.getY(), rotation, getYaw())
   * : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
   * SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
   * Constants.Swerve.maxSpeed);
   * 
   * for (SwerveModule mod : mSwerveMods) {
   * mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
   * }
   * }
   */

  /** setModuleStates passing array parameter "desiredStates"
  * The method sets the moduleStates of each swerve module on the robot
  * Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule tempMod : mSwerveMods) {
      tempMod.setDesiredState(desiredStates[tempMod.moduleNumber], false);
    }
  }

  /** Returns the estimated pose of the robot */
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /** Sets the robot to a desired pose */
  public void resetPose(Pose2d pose) {
  swerveOdometry.resetPosition(getRotation2d(), getModulePositions(),
  pose);
  }

  /** Returns the robot relative speeds */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
    return chassisSpeeds;
  }

  /** Sets the robot to the desired speed */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  /** Sets each swerve module to a desired state */
  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(targetStates[mod.moduleNumber], true);
    }
  }

  // public void resetOdometry(Pose2d pose) {
  // swerveOdometry.resetPosition(getRotation2d(), getModulePositions(),
  // pose);//NO DECLARATION FOR SWERVEODOMETRY
  // }

  /** Returns an array of the states of the swerve modules */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /** Sets the gyro heading to zero */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** Apparently also sets the gyro heading to zero */
  public void zeroHeadingAdjust() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }

  /*
   * I think this was added by AJ because he couldn't solve a problem with
   * pathplanner
   * Probably a better solution, commenting out until we know we need it -B.McC
   */
  // public void setHeading(){
  // m_gyro.setAngleAdjustment(180);
  // }

  /** Returns the current gyro z axis heading */
  public double getHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360);
  }

  /** Returns the current gyro rotation around the x axis*/
  public double getGyroRoll() {
    return m_gyro.getRoll();
  }

  /*
   * TODO: Test this to see if it is giving correct values on SmartDashboard (called in periodic)
   * If incorrect values, try switching to the commented out code below this that calls from getHeading
  */

  /** Returns the gyro angle as a getRotation 2d object */
  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  /*
   * public Rotation2d getRotation2d() {
   * return Rotation2d.fromDegrees(getHeading());
   * }
   */

  /*
   * public Rotation2d getYaw() {
   * return (Constants.Swerve.invertGyro)
   * ? Rotation2d.fromDegrees(360 - gyro.getYaw())
   * : Rotation2d.fromDegrees(gyro.getYaw());
   * }
   */

  /** Returns an array of the positions of each swerve module */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /** Turns each swerve module heading to 0, probably can get rid of this and make it directly use "resetToAbsolute" */
  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
      System.out.println("Modules Reset to Absolute");
    }
  }

  /* TODO: code that will only allow position based on AprilTag if we get good results
  public boolean isEstimateValid(Optional<EstimatedRobotPose>
  visionEstimateFront){ 
  if(visionEstimateFront == null){
  return false;
  }
  //reject poses that aren't inside the field
  boolean isInField = visionEstimateFront.get().estimatedPose.getX() > 0 &&
  visionEstimateFront.get().estimatedPose.getX() < FieldConstants.FIELD_LENGTH
  && visionEstimateFront.get().estimatedPose.getY() > 0 &&
  visionEstimateFront.get().estimatedPose.getY() < FieldConstants.FIELD_WIDTH;
  //reject tags > 5m away
  boolean isCloseEnough = visionEstimateFront.get().targetsUsed. < 5;
  //reject pose estimates with no tags in view
  boolean canSeeTag = visionEstimateFront.get().targetsUsed. .tagCount > 0;
  return canSeeTag && isInField && isCloseEnough;
  }
  

  private boolean isOdometryValid(){ 
  giving data
  for(SwerveModule module: swerveModules){
  if(!module.isEncoderDataValid()){
  return false;
  }
  }
  return true;
  }
  */

  /** Checks if the robot thinks it's outside the field and moves it back within field boundraies if it is */
  private void keepOdometryOnField() {
    Pose2d pose = getPose();
    double x = pose.getX();
    double y = pose.getY();
    Rotation2d heading = pose.getRotation();
    boolean isInField = true;

    if (x < 0) {
      x = 0;
      isInField = false;
    }
    if (y < 0) {
      y = 0;
      isInField = false;
    }
    if (x > FieldConstants.FIELD_LENGTH) {
      x = FieldConstants.FIELD_LENGTH;
      isInField = false;
    }
    if (y > FieldConstants.FIELD_WIDTH) {
      y = FieldConstants.FIELD_WIDTH;
      isInField = false;
    }
    if (!isInField) {
      swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), new Pose2d(x, y, heading));
    }
  }

  // COMMENTING ON THIS METHOD IS NOT COMPLETE
  @Override
  public void periodic() {
    swerveOdometry.update(getRotation2d(), getModulePositions());

    // Correct pose estimate with vision measurements
    SmartDashboard.putNumber("poseY", getPose().getY());
    SmartDashboard.putNumber("poseX", getPose().getX());
    SmartDashboard.putNumber("NAVX Heading", this.getRotation2d().getDegrees());

    Optional<EstimatedRobotPose> visionEstimate = photonVision.getEstimatedPose();// FROM VISION.JAVA

    if (visionEstimate.isPresent()) {
      swerveOdometry.addVisionMeasurement(visionEstimate.get().estimatedPose.toPose2d(),
          visionEstimate.get().timestampSeconds);

      SmartDashboard.putNumber("Front Estimate X", visionEstimate.get().estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Front Estimate Y", visionEstimate.get().estimatedPose.toPose2d().getY());

    }
    SmartDashboard.putBoolean("Front Estimate Present", visionEstimate.isPresent());
    SmartDashboard.putBoolean("Front AprilTag Present", visionEstimate.isPresent());

    //keepOdometryOnField();
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANCoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANCoder * 360)", mod.getCanCoder360().getDegrees());
    }

  }
}
