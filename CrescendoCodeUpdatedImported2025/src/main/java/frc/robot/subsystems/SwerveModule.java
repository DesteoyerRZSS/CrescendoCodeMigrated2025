package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.config.SwerveModuleConstants;
import frc.robot.lib.math.OnboardModuleState;
import frc.robot.lib.util.CANCoderUtil;
import frc.robot.lib.util.CANCoderUtil.CCUsage;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  // private CANSparkMax angleMotor;
  // private CANSparkMax driveMotor;
  private SparkMax angleMotor;
  private SparkMax driveMotor;
  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    //newencoder = new CANcoderConfigurator(new DeviceIdentifier(moduleConstants.cancoderID, null, null));//new CANcoderConfigurator(moduleConstants.cancoderID);
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    System.out.println(moduleNumber + "set to "+ absolutePosition);

    integratedAngleEncoder.setPosition(360-getCanCoder().getDegrees()+angleOffset.getDegrees());
    
  }
  private double optimizeAngle(Rotation2d currentAngle, Rotation2d targetAngle){
    if(Math.abs(targetAngle.minus(currentAngle).getDegrees()) > 180){
      if((targetAngle.minus(currentAngle).getDegrees()) < 0){
        return ((targetAngle.minus(currentAngle).getDegrees()) + 360);
      }
      else{
        return ((targetAngle.minus(currentAngle).getDegrees()) - 360);
      }
    }else{
      return targetAngle.minus(currentAngle).getDegrees();
    }
    
  }

  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(new CANcoderConfiguration());//configFactoryDefault();
    
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCancoderConfig);
    //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCancoderConfig);
  }

  private void configAngleMotor() {
    SparkMaxConfig angleconfig = new SparkMaxConfig();
    angleconfig.idleMode(Constants.Swerve.angleNeutralMode)
    .inverted(Constants.Swerve.angleInvert)
    .smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
    .voltageCompensation(Constants.Swerve.voltageComp)
    .smartCurrentLimit(30);
    angleconfig.closedLoop
    .pidf(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD, Constants.Swerve.angleKFF);
    angleconfig.encoder
    .positionConversionFactor(Constants.Swerve.angleConversionFactor);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, angleconfig, Usage.kPositionOnly);
    angleMotor.configure(angleconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.idleMode(Constants.Swerve.driveNeutralMode)
    .inverted(Constants.Swerve.driveInvert)
    .smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
    .voltageCompensation(Constants.Swerve.voltageComp)
    .smartCurrentLimit(50);
    driveConfig.encoder
    .velocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor)
    .positionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveConfig.closedLoop
    .pidf(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD, Constants.Swerve.angleKFF);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, driveConfig, Usage.kAll);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }
  public SwerveModulePosition getPoset() {
    return new SwerveModulePosition(driveEncoder.getPosition()/*Math.PI*0.1016*2*/, getAngle());
  }
  public Double getPosets(){
    return driveEncoder.getPosition();
  }
}
