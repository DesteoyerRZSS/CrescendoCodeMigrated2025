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

public class moveToNearestApriltagTargetR extends Command{
    private final Swerve s_Swerve;
    private final Supplier<Pose2d> poseProvider;
    private final PIDController moveXController = new PIDController(2, 0, 0); //kp used to be 0.07
    private final PIDController moveYController = new PIDController(2, 0, 0);
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private boolean isDone;
    private double diff_x;
    private double diff_y;
    private long curr_tag_in_view;
    public moveToNearestApriltagTargetR(
        Swerve s_Swerve,
        Supplier<Pose2d> poseProvider, 
        double offset) {
        this.s_Swerve= s_Swerve;
        this.poseProvider = poseProvider;
        addRequirements(s_Swerve);
    }
    
    @Override
    public void initialize() {
        s_Swerve.togglePreciseTargeting(true);
        // var roboPose = poseProvider.get();
       
        // curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        // End if currtaginview is not valid
        if (curr_tag_in_view < 0){
            System.out.println("No apriltag");
            isDone = true;
        }
        else{
            
            // theta = layout.getTagPose((int)(curr_tag_in_view)).get().toPose2d().getRotation().getDegrees() - 180;
            diff_x = LimelightHelpers.getBotPose3d_TargetSpace("limelight").toPose2d().getX();
            diff_y = LimelightHelpers.getBotPose3d_TargetSpace("limelight").toPose2d().getY();
            isDone = false;
        }
        
        
    }
  
    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        // var robot_rot = s_Swerve.optimizeAngle(Rotation2d.fromDegrees(LimelightHelpers.getBotPose3d_TargetSpace("limelight").getRotation().toRotation2d().getDegrees()), Rotation2d.fromDegrees(theta));
        var diff_x = LimelightHelpers.getBotPose3d_TargetSpace("limelight").toPose2d().getX();
        var diff_y = LimelightHelpers.getBotPose3d_TargetSpace("limelight").toPose2d().getY();
        if (isDone){
            return;
        }

        // double delt = theta - robotPose2d.getRotation().getDegrees();

        System.out.println("ID: "+ curr_tag_in_view + " diff t: " + diff_x + " " + diff_y);
        // Output Volts is capped at 2 to prevent brownout
        double xOutput = Math.min(moveXController.calculate(-1*diff_x), 2);
        double yOutput = Math.min(moveYController.calculate(-1*diff_y), 2);
        s_Swerve.drive(new Translation2d(xOutput, yOutput), 0, true, true);
        if (Math.abs(diff_x) < 0.05 && Math.abs(diff_y) < 0.05){
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

