package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;

public class moveToOffset extends Command{
    private final Swerve s_Swerve;
    private final Supplier<Pose2d> poseProvider;
    private final PIDController moveXController = new PIDController(2, 0, 0);
    private final PIDController moveYController = new PIDController(2, 0, 0);
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private boolean isDone;
    private double x;
    private double y;
    private long curr_tag_in_view;
    public moveToOffset(
        Swerve s_Swerve,
        Supplier<Pose2d> poseProvider) {
        this.s_Swerve= s_Swerve;
        this.poseProvider = poseProvider;
        // this.amount_offset = amount_offset;
        // moveXController.setTolerance(0.05);
        // moveYController.setTolerance(0.05);
        addRequirements(s_Swerve);
    }
    
    @Override
    public void initialize() {
        isDone = false;
        s_Swerve.togglePreciseTargeting(true);
        curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);

        if (curr_tag_in_view < 0){
            isDone = true;
            System.out.println("No apriltag");
        }
        else{
            var roboPose = poseProvider.get();
            Pose2d tag_pose = new Pose2d();
            if (curr_tag_in_view > 0){
                tag_pose = layout.getTagPose((int)(curr_tag_in_view)).get().toPose2d();
            }
            System.out.println("x_diff"+ (roboPose.getX() - tag_pose.getX()) + "y_dff" + (roboPose.getY() - tag_pose.getY()) + "angle" + roboPose.getRotation().minus(tag_pose.getRotation()).getDegrees());

            double diff_x = LimelightHelpers.getBotPose3d_TargetSpace("limelight").toPose2d().getX();
            double diff_y = LimelightHelpers.getBotPose3d_TargetSpace("limelight").toPose2d().getY();
            System.out.println("HEEEYEEYYEYEYE: " + diff_x + "and"+ diff_y);
            x = roboPose.getX() - diff_x;//amount_offset[0];
            y = roboPose.getY() - diff_y + 0.5;//amount_offset[1];
        }
        
        
    }
  
    @Override
    public void execute() {
        System.out.println("Executed main loop: "+ isDone);
        var robotPose2d = poseProvider.get();
        if (isDone){
            return;
        }
        double delx = x - robotPose2d.getX();
        double dely = y - robotPose2d.getY();
        System.out.println("ID: "+ curr_tag_in_view + " diff x: " + delx + " diff y: " + dely);
        // Output Volts is capped at 2 to prevent brownout
        double xOutput = Math.min(moveXController.calculate(-1*delx), 1);
        double yOutput = Math.min(moveYController.calculate(-1*dely), 1);
        // s_Swerve.drive(new Translation2d(xOutput, yOutput), 0, true, true);
        if (Math.abs(delx) < 0.05 && Math.abs(dely) < 0.05){
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
    }
    
}

