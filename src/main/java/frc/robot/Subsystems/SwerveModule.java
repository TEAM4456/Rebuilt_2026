// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Liam has decided to attempt to comment this file as well!
package frc.robot.Subsystems;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
//Possible redundant import: DO NOT DELETE. Check for instaces in other parts of code.
//import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Imports from our code
import frc.lib.configs.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;
import frc.robot.Robot;

// Class declaration
public class SwerveModule {
  
  // Class variables
  public int moduleNumber; // Represents which of the fours robot modules it is
  private Rotation2d lastAngle; // Represents which angle the motor is currently at
  private Rotation2d angleOffset;

  private SparkMax driveMotor; // Represents the drive motor of the swerve module
  private RelativeEncoder driveEncoder; // Represents the encoder of the drive motor
  private final SparkClosedLoopController driveController; // Represents the loop controller of the drive motor
  
  private SparkMax turnMotor; // Represents the turn motor of the swerve module
  private RelativeEncoder turnEncoder; // Represents the encoder of the turn motor
  private final SparkClosedLoopController angleController; // Represents the loop controller of the turn motor
  
  private SparkMaxConfig driveConfig; // Represents the config of the drive motor
  private SparkMaxConfig turnConfig; // Represents the config of the turn motor

  private CANcoder absoluteEncoder; //Changed the name from angleEncoder to clarify the Absolute CANcoder
  // Loooong declaration statment, but this is still just a class variable
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  // Class constructor passing variables moduleNumber and moduleConstants
  public SwerveModule(int modNum, SwerveModuleConstants moduleConstants) {

    // Class variables getting assigned values
    moduleNumber = modNum;
    lastAngle = getState().angle;
    angleOffset = moduleConstants.angleOffset;

    /* CANcoder (absolute angle) Encoder Config */
    absoluteEncoder = new CANcoder(moduleConstants.canCoderID);
    configAbsoluteEncoder();

    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    
    turnMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    turnEncoder = turnMotor.getEncoder(); //This is the encoder for the angle motor
    angleController = turnMotor.getClosedLoopController(); //This is the PID controller for the angle motor

    // Methods that are run every time an object is created, NOT VARIABLES!!!!!
    configDriveMotor();
    configTurnMotor();
  }
  // End constructor

  /** Sets the turn motor to the zero position based on its starting offset */
  public void resetToAbsolute() {
    double zeroPosition = (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360) - angleOffset.getDegrees(); //returns the zero position based on offset
    turnEncoder.setPosition(zeroPosition); //Turns the wheels to the zero position based on the offset
  }

  /** Sets the swerve module turn and drive motors to a desired state */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  /** Sets the absolute encoder to a default configuration. Called during creation of a swerveModule object */
  private void configAbsoluteEncoder() {
    //absoluteEncoder.optimizeBusUtilization();
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive) 
        .withAbsoluteSensorDiscontinuityPoint(1); //I think 1 is 360 unsigned, but check documentation -B.Mcc
    absoluteEncoder.getConfigurator().apply(canCoderConfig);

    /* Speed up signals to an appropriate rate */
    BaseStatusSignal.setUpdateFrequencyForAll(100, absoluteEncoder.getPosition());
  }

  /* ORIGINAL METHOD */
  // private void configTurnMotor() {
  //   //turnMotor.restoreFactoryDefaults();
  //   //CANSparkMaxUtil.setCANSparkMaxBusUsage(turnMotor, Usage.kPositionOnly);
  //   angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
  //   integratedConfig.inverted(Constants.Swerve.angleInvert);
  //   integratedConfig.idleMode(Constants.Swerve.angleNeutralMode);
  //   integratedConfig.encoder.positionConversionFactor(360/(150/7));
  //   integratedConfig.closedLoop.pid.(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
  //   angleController.setFF(Constants.Swerve.angleKFF);
  //   angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
  //   angleMotor.configure(integratedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //   resetToAbsolute();
  // }

  /** Sets the turn motor to a default configuration. Called during creation of a swerveModule object */
  private void configTurnMotor() {
    turnConfig = new SparkMaxConfig();
    turnConfig
      .inverted(Constants.Swerve.angleInvert)
      .idleMode(Constants.Swerve.angleNeutralMode)
      .smartCurrentLimit(20) //limits current to 20 amps
      .voltageCompensation(12.0); //provides consistent power even if voltage drops
    turnConfig.encoder
      .positionConversionFactor(360/(12.8)); //changed from (360/(150/7)) to (360/12.8) to reflect steering gear ratio of MK4c
      //.velocityConversionFactor(1000);
    turnConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD)
      .positionWrappingEnabled(true) // TODO test if this turns to nearest angle
      .positionWrappingInputRange(0, 360); //should allow motor to move in direction of nearest angle (changed to 0-360 by CB)
    
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /* ORIGINAL METHOD */ 
  // private void configDriveMotor() {
  //   driveMotor.restoreFactoryDefaults();
  //   CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
  //   driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
  //   driveMotor.setInverted(Constants.Swerve.driveInvert);
  //   driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
  //   driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
  //   driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
  //   driveController.pid(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
  //   driveController.setFF(Constants.Swerve.angleKFF);
  //   driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
  //   driveMotor.configure(integratedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //   driveEncoder.setPosition(0.0);
  // }

  /** Sets the drive motor to a default configuration. Called during creation of a swerveModule object */
  private void configDriveMotor() {
    driveConfig = new SparkMaxConfig();
    driveConfig
      .inverted(Constants.Swerve.driveInvert)
      .idleMode(Constants.Swerve.driveNeutralMode)
      .smartCurrentLimit(40) //limits current to 40 amps
      .voltageCompensation(12.0); //provides consistent power even if voltage drops
    driveConfig.encoder
      .positionConversionFactor(Constants.Swerve.driveConversionPositionFactor)
      .velocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);    
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Sets the speed of the drive motor in a swerve module */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    }
    else {
      driveController.setSetpoint(
        desiredState.speedMetersPerSecond,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }
  /** Sets the angle of the turn motor in a swerve module */
  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }
  /** Returns current angle of the turn motor in a swerve module */
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getPosition());
  }

  /** Returns the Can Coder angle reading in 360 degrees */
  public Rotation2d getCanCoder360() {
    return Rotation2d.fromDegrees((absoluteEncoder.getAbsolutePosition().getValueAsDouble()) * 360); //gets Cancoder value 0-1 as double * 360 for degrees
  }

  /** Returns the Can Coder angle reading */
  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees((absoluteEncoder.getAbsolutePosition().getValueAsDouble())); //gets Cancoder value 0-1 as double * 360 for degrees
  }

  /** Returns the current state of the swerve module */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  /** Returns the current position of the swerve module */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }


}
