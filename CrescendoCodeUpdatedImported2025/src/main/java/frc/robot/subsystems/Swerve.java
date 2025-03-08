package frc.robot.subsystems;

import java.io.UncheckedIOException;
import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;
import frc.robot.LimelightHelpers;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro1;
  //ADXRS450_Gyro gyro;
  // private SwerveDriveOdometry swerveOdometry;
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveModule[] mSwerveMods;
  // private PhotonCamera cam;
  private Field2d field;
  private AprilTagFieldLayout layout;
  private NetworkTable table;
  private boolean preciseTargeting;
  private long curr_tag_in_view;
  private long is_tag_present;
  public Swerve(/*PhotonCamera cam*/) {
    curr_tag_in_view = -1;
    // this.cam = cam;
    gyro1 = new Pigeon2(5);
    gyro1.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };
    // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
    poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d());
    preciseTargeting = false;
    try {
      layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
      
    } catch (UncheckedIOException e) {
      System.out.println("April Tag Field Layout not Found");
    }
    // AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    field = new Field2d();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("TODO"); //TODO
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);// TODO
                                                                                                              // NEED TO
                                                                                                              // WORK
                                                                                                              // ONnnjnijni
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void XLock() {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    desiredStates[0] = new SwerveModuleState(0, new Rotation2d(Mod0.angleOffset.getDegrees() + 45));
    desiredStates[1] = new SwerveModuleState(0, new Rotation2d(Mod1.angleOffset.getDegrees() - 45));
    desiredStates[2] = new SwerveModuleState(0, new Rotation2d(Mod2.angleOffset.getDegrees() - 45));
    desiredStates[3] = new SwerveModuleState(0, new Rotation2d(Mod3.angleOffset.getDegrees() + 45));
    setModuleStates(desiredStates);
  }
  
  public void resetWheels() {
    SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    desiredStates[0] = new SwerveModuleState(0, new Rotation2d(Mod0.angleOffset.getDegrees()));
    desiredStates[1] = new SwerveModuleState(0, new Rotation2d(Mod1.angleOffset.getDegrees()));
    desiredStates[2] = new SwerveModuleState(0, new Rotation2d(Mod2.angleOffset.getDegrees()));
    desiredStates[3] = new SwerveModuleState(0, new Rotation2d(Mod3.angleOffset.getDegrees()));
    setModuleStates(desiredStates);
  }
  
  public void stop() {
    setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
  }

  public void setAbsolute() {
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
    // Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  //moves the robot's x and y coordinates to the location specified

  public Command moveTo(Pose2d end){
    preciseTargeting = true;
    Pose2d xy1 = getPose();
    Pose2d xy2 = end;

    PIDController moveXController = new PIDController(2, 0, 0);
    PIDController moveYController = new PIDController(2, 0, 0);

    double xDiff = xy2.getX() - xy1.getX();
    double yDiff = xy2.getY() - xy1.getY();

    

    return run(
      () -> {
        System.out.println("current x = " + getPose().getX());
        System.out.println(xy2.getX() - getPose().getX());
        double xOutput = moveXController.calculate(-1*xy2.getX() + getPose().getX());
        double yOutput = moveYController.calculate(-1*xy2.getY() + getPose().getY());
        drive(new Translation2d(xOutput, yOutput), 0, false, true);

      }
    ).until(
      () -> (
        (((Math.abs((xy2.getX() - getPose().getX()))) < 0.05) && ((Math.abs((xy2.getY() - getPose().getY()))) < 0.05))
      )).andThen(()->{preciseTargeting = false;});

    }
  
  // aims torwards an angle theta displaced from the heading of the robot
  public Command turnToAngle(Rotation2d theta){

    preciseTargeting = true;

    Rotation2d target = theta;

    PIDController turnController = new PIDController(0.255, 0.0001, 0);

    double totalDiff = optimizeAngle(getYaw(), target);

    return run(
      () -> {
        System.out.println("current angle = " + getYaw().getDegrees());
        System.out.println(target.getDegrees() - getYaw().getDegrees());
        double output = turnController.calculate(optimizeAngle(getYaw(), target));
        drive(new Translation2d(0, 0), output, false, true);

      }
    ).until(
      ()->(
          (Math.abs(optimizeAngle(getYaw(), target)/totalDiff) < 0.05) 
      )).andThen(()->{preciseTargeting = false;});
  }
  public Command turnToAngle_nearest_apriltag(){

    preciseTargeting = true;
    long curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
    Pose3d target;
    Rotation2d target_rotation;
    Rotation2d turn_rotation;
    double x, y;
    System.out.println(curr_tag_in_view);
    if (curr_tag_in_view >= 0){
      System.out.println("tag update:" + preciseTargeting);
      target = layout.getTagPose((int)(curr_tag_in_view)).get();
      target_rotation = target.getRotation().toRotation2d().plus(Rotation2d.fromDegrees(0));
      turn_rotation = target.getRotation().toRotation2d().plus(Rotation2d.fromDegrees(-135));

    }
    else{
      return runOnce(()->{System.out.println("no apriltags detected");});
    }

    PIDController turnController = new PIDController(0.255, 0.0001, 0);

    double totalDiff = optimizeAngle(getYaw(), turn_rotation);
    
    return run(
      () -> {
        System.out.println("curr pose: " + getYaw().getDegrees() + " desired: " + turn_rotation.getDegrees() );
        System.out.println(turn_rotation.getDegrees() - getYaw().getDegrees());
        double output = turnController.calculate(optimizeAngle(getYaw(), turn_rotation));
        output = Math.min(output, 7);
        System.out.print(output);
      
        drive(new Translation2d(0,0), output, false, true);
        

      }
    ).until(
      ()->(
          (Math.abs(optimizeAngle(getYaw(), turn_rotation)/totalDiff) < 0.05) 
      )).andThen(()->{preciseTargeting = false;});
  }
  public Pose2d get_xy_from_tag(Pose3d layoutpose3d, double offset){
    Rotation2d target_rotation = layoutpose3d.getRotation().toRotation2d().plus(Rotation2d.fromDegrees(0));
    double offset_x = offset * target_rotation.getCos();
    double offset_y = offset * target_rotation.getCos();
    // offset_x += 0.5 * target_rotation.plus(Rotation2d.fromDegrees(-90)).getCos();
    offset_y += 0.5 * target_rotation.plus(Rotation2d.fromDegrees(-90)).getSin();
    double x = layoutpose3d.getX() + offset_x;
    double y = layoutpose3d.getY() + offset_y;
    return new Pose2d(x, y, target_rotation);
  }
  public void togglePreciseTargeting(boolean val){
    preciseTargeting = val;
  }
  
  public double optimizeAngle(Rotation2d currentAngle, Rotation2d targetAngle){
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

  public void resetOdometry(Pose2d pose) {
    //swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public void zeroGyro() {
    gyro1.reset();
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPoset();
    }
    return positions;
  }
  
  public Pose2d getPose() {
    //return swerveOdometry.getPoseMeters();
    return poseEstimator.getEstimatedPosition();

  }

  public ChassisSpeeds getSpeeds() {

    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-1*gyro1.getYaw().getValueAsDouble() * (Constants.Swerve.invertGyro ? 1 : -1));

  }

  public void update_odometry_and_pose(boolean tag_update){
    // swerveOdometry.update(getYaw(), getPositions());
    tag_update = true; //TODO remove later
    poseEstimator.update(getYaw(), getPositions());
    
    
    if (tag_update){
      boolean doRejectUpdate = false;
      
      //Pose2d tag2dpose = layout.getTagPose((int)(curr_tag_in_view)).get().toPose2d();
      LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      //field.getObject("traj").setPoses(tag2dpose);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(-1*gyro1.getYaw().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    } 
    
  }
  
  @Override
  public void periodic() {
    // if (preciseTargeting == false){System.out.println("tag update:" + preciseTargeting);}
    // boolean highAccuracy = SmartDashboard.getBoolean("highAccuracyTargeting", false);
    update_odometry_and_pose(preciseTargeting);
    // if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1) > 0){
    //   System.out.println("Rotation relative: " + LimelightHelpers.getBotPose3d_TargetSpace("limelight").getRotation().toRotation2d().getDegrees());
    // }
    // else{
    //   System.out.println("no tag");
    // }


    // long curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0);
    // Pose3d target;
    // Rotation2d target_rotation;
    // if (curr_tag_in_view >= 0){
    //   target = layout.getTagPose((int)(curr_tag_in_view)).get();
    //   target_rotation = target.getRotation().toRotation2d().plus(Rotation2d.fromDegrees(0));
    //   double offset_x = 1 * target_rotation.getCos();
    //   double offset_y = 1 * target_rotation.getCos();
    //   if (true){
    //     double x = target.getX() + offset_x;
    //     double y = target.getY() + offset_y;
    //     System.out.println("before: "+ target.getX() +"," + target.getY());
    //     System.out.println("after: "+ x +"," + y);
    //   }
    // }
    // System.out.println(getPose())
    curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
    is_tag_present = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0);
    // System.out.println(curr_tag_in_view);
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Mod" + mod.moduleNumber + " DrivePos", mod.getPosets());
    }
  }
}
