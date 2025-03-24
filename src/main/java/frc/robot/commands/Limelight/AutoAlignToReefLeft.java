// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.LLSettings1;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/* This command works flawlessly, DON'T TOUCH THIS CODE PLEASE!*/
public class AutoAlignToReefLeft extends Command {

  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = -1;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double maxDtSpeed;
  private double maxAngularRate;

  public AutoAlignToReefLeft(boolean isRightScore, CommandSwerveDrivetrain drivebase, 
  double maxDtSpeed, double maxAngularRate) {
    xController = new PIDController(.658, 0.0, 0);  // Vertical movement
    yController = new PIDController(0.43, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(.048, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.maxDtSpeed = maxDtSpeed;
    this.maxAngularRate = maxAngularRate;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    if(Limelight.autoEstimateDistance() > 25) {
      rotController.setP(0.041);
    } else if(Limelight.autoEstimateDistance() < 25) {
      rotController.setP(0.041);
    } else if(Limelight.autoEstimateDistance() <= 18) {
      rotController.setP(0.007);
    } 

    LimelightHelpers.setPipelineIndex("limelight", 3);
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(LLSettings1.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(LLSettings1.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(LLSettings1.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(LLSettings1.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? LLSettings1.Y_SETPOINT_REEF_ALIGNMENT : -LLSettings1.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(LLSettings1.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
      SmartDashboard.putNumber("x", postions[2]);
      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.setControl(
        robotCentric
          .withVelocityX(xSpeed * (maxDtSpeed /2)) // forward backward
          .withVelocityY(ySpeed *(maxDtSpeed /1)) // left right
          .withRotationalRate(rotValue *(maxAngularRate /4)));

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      System.out.println("NO ATAG");
      
      drivebase.setControl(
        robotCentric
          .withVelocityX(0) // forward backward
          .withVelocityY(0) // left right
          .withRotationalRate(0));
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("ALIGNED");
    drivebase.setControl(
        robotCentric
          .withVelocityX(0) // forward backward
          .withVelocityY(0) // left right
          .withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(LLSettings1.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(LLSettings1.POSE_VALIDATION_TIME);
  }
}
