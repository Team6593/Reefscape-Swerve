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
import frc.robot.Constants.LLSettings;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToReef extends Command {

  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = -1;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double maxDtSpeed;

  public AutoAlignToReef(boolean isRightScore, CommandSwerveDrivetrain drivebase, double maxDtSpeed) {
    xController = new PIDController(.5, 0.0, 0);  // Vertical movement
    yController = new PIDController(.5, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(.5, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.maxDtSpeed = maxDtSpeed;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(LLSettings.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(LLSettings.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(LLSettings.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(LLSettings.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? LLSettings.Y_SETPOINT_REEF_ALIGNMENT : -LLSettings.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(LLSettings.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight");
  }

  @Override
  public void execute() {
    System.out.println("RUNNING");
    if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
      this.dontSeeTagTimer.reset();
      System.out.println("SAW ATAG");

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);
      // drivebase.applyRequest( () -> {
        // System.out.println("DRIVING TO ATAG");
        // System.out.println("VALUES ---");
        // System.out.println(xSpeed);
        // System.out.println(ySpeed);
        // System.out.println("---");
      //   return robotCentric
      //     .withVelocityX(xSpeed * (maxDtSpeed))
      //     .withVelocityY(ySpeed * (maxDtSpeed))
      //     .withRotationalRate(0);
      // });

      System.out.println("DRIVING TO ATAG");
      System.out.println("VALUES ---");
      System.out.println(xSpeed);
      System.out.println(ySpeed);
      System.out.println("---");

      drivebase.setControl(
        robotCentric
          .withVelocityX(xSpeed *(maxDtSpeed /2))
          .withVelocityY(ySpeed *(maxDtSpeed /2))
          .withRotationalRate(0));

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
    System.out.println("ALIGNED");
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
    return this.dontSeeTagTimer.hasElapsed(LLSettings.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(LLSettings.POSE_VALIDATION_TIME);
  }
}
