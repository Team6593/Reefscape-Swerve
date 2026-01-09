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

/* Note: Don't use this command, it's janky, the code is messy, and it only works 80 percent of the time*/
public class AutoAlignToReef extends Command {

  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = -1;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double maxDtSpeed;
  private double maxAngularRate;

  public AutoAlignToReef(boolean isRightScore, CommandSwerveDrivetrain drivebase, 
  double maxDtSpeed, double maxAngularRate) {
    xController = new PIDController(.7, 0.0, 0);  // Vertical movement
    yController = new PIDController(0.5, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(.07, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.maxDtSpeed = maxDtSpeed;
    this.maxAngularRate = maxAngularRate;
    addRequirements(drivebase);
  }

  // copypasted from RobotContainer because im lazy -MQ
  double aim() {
    double kP = -.014; // -0.0095
    double targetingAngularVelocity = (LimelightHelpers.getTX("limelight") - LLSettings1.TX_VALUE) * kP;
    targetingAngularVelocity *= maxAngularRate;
    return targetingAngularVelocity;
  }

  double distance() {
    double kP = -0.05; // -0.09 before, -0.05 was too slow
    double target = 5.53; // goal is 6 inches (aka bumper to reef)
    double current = Limelight.autoEstimateDistance();
    double error = target - current;
    double result = error * kP;
    return result;
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight", 3);
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(LLSettings1.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(LLSettings1.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(LLSettings1.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(LLSettings1.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? LLSettings1.TX_VALUE : -LLSettings1.TX_VALUE);
    yController.setTolerance(LLSettings1.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight");
  }

  @Override
  public void execute() {
    //System.out.println("RUNNING");
    if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
      this.dontSeeTagTimer.reset();
      //System.out.println("SAW ATAG");

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
      SmartDashboard.putNumber("x", postions[2]);
      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      double currentTx = LimelightHelpers.getTX("limelight");
      double error = LLSettings1.TX_VALUE - currentTx;
      double result = error * 0.009;
      // System.out.println("DRIVING TO ATAG");
      // System.out.println("VALUES ---");
      // System.out.println(xSpeed);
      // System.out.println(ySpeed);
      // System.out.println("---");
      double rotation = aim();

      // null check if tag isn't visible
      double forwardspeed = distance();

      drivebase.setControl(
        robotCentric
          .withVelocityX(forwardspeed * (maxDtSpeed /8)) // forward backward
          .withVelocityY(result *(maxDtSpeed /2)) // left right
          .withRotationalRate(rotValue *(maxAngularRate /4)));

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      System.out.println("NO ATAG");
      //drivebase.drive(new Translation2d(), 0, false);
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
    return this.dontSeeTagTimer.hasElapsed(LLSettings1.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(LLSettings1.POSE_VALIDATION_TIME);
  }
}
