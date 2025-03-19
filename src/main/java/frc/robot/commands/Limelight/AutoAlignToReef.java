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
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

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
  private double maxAngularRate;

  public AutoAlignToReef(boolean isRightScore, CommandSwerveDrivetrain drivebase, 
  double maxDtSpeed, double maxAngularRate) {
    xController = new PIDController(.7, 0.0, 0);  // Vertical movement
    yController = new PIDController(0.5, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(.05, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    this.maxDtSpeed = maxDtSpeed;
    this.maxAngularRate = maxAngularRate;
    addRequirements(drivebase);
  }

  // copypasted from RobotContainer because im lazy -MQ
  double aim() {
    //System.out.println("AIMING");
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = -.0095;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = (LimelightHelpers.getTX("limelight") - LLSettings.TX_VALUE) * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= maxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    //targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
}

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight", 3);
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(LLSettings.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(LLSettings.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(LLSettings.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(LLSettings.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? LLSettings.TX_VALUE : -LLSettings.TX_VALUE);
    yController.setTolerance(LLSettings.Y_TOLERANCE_REEF_ALIGNMENT);

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
      double error = LLSettings.TX_VALUE - currentTx;
      double result = error * 0.009;
      // System.out.println("DRIVING TO ATAG");
      // System.out.println("VALUES ---");
      // System.out.println(xSpeed);
      // System.out.println(ySpeed);
      // System.out.println("---");
      double rotation = aim();

      drivebase.setControl(
        robotCentric
          .withVelocityX(xSpeed * (maxDtSpeed /2)) // forward backward
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
    return this.dontSeeTagTimer.hasElapsed(LLSettings.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(LLSettings.POSE_VALIDATION_TIME);
  }
}
