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
import frc.robot.Constants.LLSettings2;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/* Align's to the reef's right side, and as of 3/20/25, 
 the second limelight has been installed but we haven't grabbed the setpoints yet */
public class AutoAlignToReefRight extends Command {

  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = -1;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double maxDtSpeed;
  private double maxAngularRate;

  public AutoAlignToReefRight(boolean isRightScore, CommandSwerveDrivetrain drivebase, 
  double maxDtSpeed, double maxAngularRate) {
    xController = new PIDController(.4, 0.0, 0);  // Vertical movement
    yController = new PIDController(0.4, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(.05, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.maxDtSpeed = maxDtSpeed;
    this.maxAngularRate = maxAngularRate;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    if(Limelight.autoEstimateDistance() > 25) {
      rotController.setP(0.05);
    } else if(Limelight.autoEstimateDistance() < 25) {
      rotController.setP(0.03);
    } else if(Limelight.autoEstimateDistance() <= 18) {
      rotController.setP(0.005);
    } 

    LimelightHelpers.setPipelineIndex("limelight-two", 0);
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(LLSettings2.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(LLSettings2.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(LLSettings2.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(LLSettings2.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? LLSettings2.Y_SETPOINT_REEF_ALIGNMENT : -LLSettings2.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(LLSettings2.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-two");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-two") && LimelightHelpers.getFiducialID("limelight-two") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-two");
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
      
      drivebase.applyRequest( () -> {
        return robotCentric
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0);
      });
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    //System.out.println("ALIGNED");
    drivebase.applyRequest( () -> {
      return robotCentric
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0);
    });
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(LLSettings2.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(LLSettings2.POSE_VALIDATION_TIME);
  }
}
