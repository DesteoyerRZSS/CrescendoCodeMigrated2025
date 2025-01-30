package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.intake.intakeArm;
import frc.robot.Constants.Swerve.intake.intakeMotor;

public class Intake extends SubsystemBase{
    
    private SparkMax lolIntakeArm;
    private RelativeEncoder intakeArmEncoder;
    private SparkClosedLoopController intakeArmPID;
    private Spark intakeMotorL;
    private Spark intakeMotorR;

    public Intake(){

        lolIntakeArm = new SparkMax(
            intakeArm.rotMotorID,
            MotorType.kBrushless
        );

        intakeMotorL = new Spark(
            intakeMotor.IDL
        );
        intakeMotorR = new Spark(
            intakeMotor.IDR
        );
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.closedLoop.pid(Constants.Swerve.intake.intakeArm.kP, Constants.Swerve.intake.intakeArm.kI, Constants.Swerve.intake.intakeArm.kD);
        config.encoder.positionConversionFactor(360.0/intakeArm.gearRatio).velocityConversionFactor((360 / intakeArm.gearRatio) / 60.0);
        //Initializes encoder for intake's arm
        intakeArmEncoder = lolIntakeArm.getEncoder();
        
        

        // // Set Conversion factor for Encoder -> degrees
        // intakeArmEncoder.setPositionConversionFactor(
        //     360.0/intakeArm.gearRatio //TODO
        // ); 
        // // Set Velocity Conversion factor -> degrees/second
        // intakeArmEncoder.setVelocityConversionFactor(
        //     (360 / intakeArm.gearRatio) / 60.0  //TODO
        // );
        // Set the position to zero
        intakeArmEncoder.setPosition(0);

        //Initialize arm PID
        intakeArmPID = lolIntakeArm.getClosedLoopController();

        lolIntakeArm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public Command Out(){
         return run(()->{
            intakeMotorL.set(-1); 
            intakeMotorR.set(1);
         });
    }
    public Command In(){
        return run(()->{
            intakeMotorL.set(1);
            intakeMotorR.set(-1);
            System.out.println("running intake");
        });
    }
    public Command Stop(){
        return run(()->{
            intakeMotorL.stopMotor();
            intakeMotorR.stopMotor();
        });
    }

    public Rotation2d getPos(){
        Rotation2d posIntakeArm = Rotation2d.fromDegrees(intakeArmEncoder.getPosition()); 
        return posIntakeArm;
    }
    public Rotation2d getErrors(Rotation2d goal){
        Rotation2d currPos =getPos();
        double error = currPos.getDegrees() - goal.getDegrees();
        return Rotation2d.fromDegrees(error);
    }
    // Run PID
    public void GoTo(Rotation2d goal){
        intakeArmPID.setReference(
            goal.getDegrees(), 
            ControlType.kPosition, ClosedLoopSlot.kSlot0
        );
        
        
    }
    // Convert PID method into a runable command
    // if button is pressed, go down to goal, until within setpoint or when button stopped pressing, when button at setpoint, start intake and go until 
    public Command moveTo(Rotation2d goal, boolean intake){
        
        //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Goal", goal.getDegrees());
           
        }).andThen(run(
            () -> GoTo(goal)
        ).withTimeout(1)/*until(
            ()->(
                Math.abs(getErrors(goal).getDegrees()) < Constants.Swerve.intake.tolerance 
                && Math.abs(getErrors(goal).getDegrees()) < Constants.Swerve.intake.tolerance
            )
        ).andThen((intake)? In(): Stop())*/
        );
    }
    public void periodic(){
        SmartDashboard.putNumber("IntakePos", getPos().getDegrees());
    }
}
