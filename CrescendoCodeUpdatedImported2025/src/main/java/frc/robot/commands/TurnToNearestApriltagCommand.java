package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;

public class TurnToNearestApriltagCommand extends Command{
    private final Swerve s_Swerve;
    private final Supplier<Pose2d> poseProvider;
    private final PIDController moveThetaController = new PIDController(0.5, 0, 0); //kp used to be 0.07
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private boolean isDone;
    private double theta;
    private long curr_tag_in_view;
    public TurnToNearestApriltagCommand(
        Swerve s_Swerve,
        Supplier<Pose2d> poseProvider) {
        this.s_Swerve= s_Swerve;
        this.poseProvider = poseProvider;
        // moveThetaController.setTolerance(0.05);
        moveThetaController.enableContinuousInput(-180, 180);
        addRequirements(s_Swerve);
    }
    
    @Override
    public void initialize() {
        s_Swerve.togglePreciseTargeting(true);
        // var roboPose = poseProvider.get();
        var robot_rot = LimelightHelpers.getBotPose3d_TargetSpace("limelight").getRotation().toRotation2d().getDegrees();
        // curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        // End if currtaginview is not valid
        if (curr_tag_in_view < 0){
            System.out.println("No apriltag");
            isDone = true;
        }
        else{
            
            // theta = layout.getTagPose((int)(curr_tag_in_view)).get().toPose2d().getRotation().getDegrees() - 180;
            theta =0.1;
            isDone = false;
        }
        
        
    }
  
    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robot_rot = s_Swerve.optimizeAngle(Rotation2d.fromDegrees(LimelightHelpers.getBotPose3d_TargetSpace("limelight").getRotation().toRotation2d().getDegrees()), Rotation2d.fromDegrees(theta));
        if (isDone){
            return;
        }
        // double delt = theta - robotPose2d.getRotation().getDegrees();

        double delt = s_Swerve.optimizeAngle(robotPose2d.getRotation(), Rotation2d.fromDegrees(theta));
        System.out.println("ID: "+ curr_tag_in_view + " diff t: " + robot_rot);
        // Output Volts is capped at 2 to prevent brownout
        double thetaOutput = Math.min(moveThetaController.calculate(robot_rot), 7);
        System.out.println("Theta output: " + thetaOutput);
        s_Swerve.drive(new Translation2d(0, 0), thetaOutput, true, true);
        if (Math.abs(robot_rot) < 1){
            isDone = true;
        }
        else{
            isDone = false;
        }
        
    }
    @Override
    public boolean isFinished(){
        return isDone;
    }
    @Override
    public void end(boolean interrupted) {
        s_Swerve.stop();
        s_Swerve.togglePreciseTargeting(false);
    }
    
}

