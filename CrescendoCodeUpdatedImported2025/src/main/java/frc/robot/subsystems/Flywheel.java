package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase{
    private SparkMax top;
    private SparkMax bott;
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottEncoder;
    private SparkClosedLoopController FWtopPID;
    private SparkClosedLoopController FWbottPID;
    public Flywheel() {
        // initialies all the variables and constants 
        top = new SparkMax(frc.robot.Constants.Swerve.flywheel.FWtop.FWid, MotorType.kBrushless );
        bott = new SparkMax(frc.robot.Constants.Swerve.flywheel.FWbott.FWid, MotorType.kBrushless );
        SparkMaxConfig topconfig = new SparkMaxConfig();
        topconfig.idleMode(IdleMode.kCoast).inverted(false);
        topconfig.encoder.velocityConversionFactor((Math.PI * Constants.Swerve.flywheel.FWDiameter * 0.0254 / Constants.Swerve.flywheel.FWtop.gearRatio) / 60.0 );
        topconfig.closedLoop.pid(Constants.Swerve.flywheel.FWtop.kP, Constants.Swerve.flywheel.FWtop.kI, Constants.Swerve.flywheel.FWtop.kD);
        top.configure(topconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig bottConfig = new SparkMaxConfig();
        bottConfig.idleMode(IdleMode.kCoast).inverted(true);
        bottConfig.encoder.velocityConversionFactor((Math.PI * Constants.Swerve.flywheel.FWDiameter * 0.0254 / Constants.Swerve.flywheel.FWbott.gearRatio) / 60.0);
        bottConfig.closedLoop.pid(Constants.Swerve.flywheel.FWbott.kP, Constants.Swerve.flywheel.FWbott.kI, Constants.Swerve.flywheel.FWbott.kD);
        bott.configure(bottConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        topEncoder = top.getEncoder();
        topEncoder.setPosition(0);
        bottEncoder = bott.getEncoder();
        bottEncoder.setPosition(0);
        
        FWtopPID = top.getClosedLoopController();

        FWbottPID = bott.getClosedLoopController();
    }

    public double[] getVelocity(){
        double topVelocity = topEncoder.getVelocity(); //RPM
        double bottVelocity = bottEncoder.getVelocity();
        double[] combinedVelocity = 
        new double[] {topVelocity , bottVelocity};
        return combinedVelocity;
    }
    public double[] getErrors(double[] goal){
        double[] currVelocity = new double[] {getVelocity()[0], getVelocity()[1]};
        double[] error = new double[] {
            goal[0] - currVelocity[0] , goal[1] - currVelocity[1]
        };
        return error;
    }
    public void GoTo(double topGoal, double bottGoal){
        FWtopPID.setReference(
            topGoal, 
            ControlType.kVelocity, ClosedLoopSlot.kSlot0, new SimpleMotorFeedforward(0, 0.1).calculate(topGoal)
        );
        FWbottPID.setReference(
            bottGoal, 
            ControlType.kVelocity, ClosedLoopSlot.kSlot0, new SimpleMotorFeedforward(0, 0.1).calculate(bottGoal)
        );
    }
    private boolean atSetpoint = false;

    /*public Command moveTo(BooleanSupplier amp, BooleanSupplier speaker, DoubleSupplier trigger){
        final double[] v = new double[]{(amp.getAsBoolean())? flywheel.AMP: flywheel.SPEAKER, (amp.getAsBoolean())? flywheel.AMP: flywheel.SPEAKER};
        SmartDashboard.putNumber("trigger value", trigger.getAsDouble());
        System.out.println(trigger.getAsDouble());
        if (trigger.getAsDouble() > 0.2){
            System.out.println("running flywheel");
            return runOnce(()->{
                SmartDashboard.putNumber("Velocity Goal", v[0]);
                //System.out.println(v[0]);
                }).andThen(run(
                    () -> GoTo(
                        v[0], v[1]
                    )
                ).until(
                    ()->(
                        Math.abs(getErrors(v)[0]) < Constants.Swerve.flywheel.tolerance 
                        && Math.abs(getErrors(v)[1]) < Constants.Swerve.flywheel.tolerance
                    )).andThen(runOnce(()->{
                        atSetpoint = true;
                    }))
            );
        }
        else{
            return runOnce(()->{});
        }
        
    }*/
    public Command stop(){
        return run(()->{top.set(0); bott.set(0);});
    }
    public Command ampShot(){
        return run(()->{top.setVoltage(1.70); bott.setVoltage(1.70);});
    }
    /*public Command speakerShot(){
        return run(()->{top.set(0.4); bott.set(0.4);});
    }*/

    public Command moveTo(double vtop, double vbott, boolean idleState, BooleanSupplier trigger, boolean note){
        double[] v  =new double[]{vtop, vbott};
        //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Shoulder Goal", v[0]);
            SmartDashboard.putNumber("Joint Goal", v[1]);
        }).andThen(run(
            () -> {GoTo(v[0], v[1]);
            double[] getVelocity = this.getVelocity();
            System.out.println(getVelocity);
            }

        )/* .until(
            ()->(
                (Math.abs(getErrors(v)[0]) < Constants.Swerve.flywheel.tolerance 
                && Math.abs(getErrors(v)[1]) < Constants.Swerve.flywheel.tolerance) && !trigger.getAsBoolean()
            )).andThen(runOnce(()->{
                if (idleState){
                    atSetpoint = false;
                }else{
                    atSetpoint = true;
                }
            }))*/
        );
    }
    public Command moveTill(double vtop, double vbott, boolean idleState){
        double[] v  =new double[]{vtop, vbott};
        //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Shoulder Goal", v[0]);
            SmartDashboard.putNumber("Joint Goal", v[1]);
        }).andThen(run(
            () -> {GoTo(v[0], v[1]);
            double[] getVelocity = this.getVelocity();
            System.out.println(getVelocity);
            }

        ) .until(
            ()->(
                (Math.abs(getErrors(v)[0]) < Constants.Swerve.flywheel.tolerance 
                && Math.abs(getErrors(v)[1]) < Constants.Swerve.flywheel.tolerance) 
            )).andThen(runOnce(()->{
                if (idleState){
                    atSetpoint = false;
                }else{
                    atSetpoint = true;
                }
            }))
        );
    }
    public void periodic(){
        SmartDashboard.putBoolean("Ready To Shoot?", atSetpoint);
        SmartDashboard.putNumber("lywhell Vellovity", getVelocity()[0]);
    }

}
