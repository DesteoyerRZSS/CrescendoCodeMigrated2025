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
import frc.robot.subsystems.*;

public class moveToNearestApriltagCommand extends Command{
    private final Swerve s_Swerve;
    private final Supplier<Pose2d> poseProvider;
    private final PIDController moveXController = new PIDController(0.5, 0, 0);
    private final PIDController moveYController = new PIDController(0.5, 0, 0);
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    private final double amount_offset;
    private boolean isDone;
    private double x;
    private double y;
    private long curr_tag_in_view;
    public moveToNearestApriltagCommand(
        Swerve s_Swerve,
        Supplier<Pose2d> poseProvider,
        double amount_offset) {
        this.s_Swerve= s_Swerve;
        this.poseProvider = poseProvider;
        this.amount_offset = amount_offset;
        moveXController.setTolerance(0.05);
        moveYController.setTolerance(0.05);
        addRequirements(s_Swerve);
    }
    
    @Override
    public void initialize() {
        s_Swerve.togglePreciseTargeting(true);
        var roboPose = poseProvider.get();
        curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        // End if currtaginview is not valid
        if (curr_tag_in_view < 0){
            System.out.println("No apriltag");
            isDone = true;
        }
        else{
            double[] xy_tag = s_Swerve.get_xy_from_tag(layout.getTagPose((int)(curr_tag_in_view)).get(), amount_offset);
            x = xy_tag[0];
            y = xy_tag[1];
            isDone = false;
        }
        
    }
  
    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        if (isDone){
            return;
        }
        double delx = x - robotPose2d.getX();
        double dely = y - robotPose2d.getY();
        System.out.println("ID: "+ curr_tag_in_view + " diff x: " + delx + " diff y: " + dely);
        // Output Volts is capped at 2 to prevent brownout
        double xOutput = Math.min(moveXController.calculate(-1*delx), 2);
        double yOutput = Math.min(moveYController.calculate(-1*dely), 2);
        s_Swerve.drive(new Translation2d(xOutput, yOutput), 0, true, true);
        
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

