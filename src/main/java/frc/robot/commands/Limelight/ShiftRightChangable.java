// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShiftRightChangable extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double targetPose;
  private double initialPoseX;
  private double initialPoseY;
  private boolean isY;
  private double maxSpeed;
  private boolean finished = false;
  
  /** Creates a new ShiftRight. */
  public ShiftRightChangable(CommandSwerveDrivetrain drivetrain, double maxSpeed) {
    this.drivetrain = drivetrain;
    this.maxSpeed = maxSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Double[] yIDs = {7.0, 10.0, 18.0, 21.0};
    Double tagID = LimelightHelpers.getFiducialID("limelight");
    List<Double> yIDsList = Arrays.asList(yIDs);
    if (yIDsList.contains(tagID)) {
      isY = true;
    } else if (!yIDsList.contains(tagID)) {
      isY = false;
    }

    if (isY) {
      initialPoseY = drivetrain.getState().Pose.getY();
      targetPose = initialPoseY + 0.40; // .29 before
    } else if (!isY) {
      initialPoseX = drivetrain.getState().Pose.getX();
      targetPose = initialPoseX - 0.33; // .29 before
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isY) {
      System.out.println("CURRENT POSEY");
      double currentPose = drivetrain.getState().Pose.getY();
      System.out.println(currentPose);
      if(currentPose < targetPose) {
        System.out.println("LESS THAN VALUE");
        finished = false;
        drivetrain.setControl(
          robotCentric
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0)
        );
      } else if(currentPose > targetPose) {
        System.out.println("GREATER THAN VALUE");
        finished = false;
        drivetrain.setControl(
          robotCentric
          .withVelocityX(0)
          .withVelocityY(-.12 * maxSpeed)
          .withRotationalRate(0)
        );
      } else {
        drivetrain.setControl(
          robotCentric
            .withVelocityX(0) // forward backward
            .withVelocityY(0) // left right
            .withRotationalRate(0));
        finished = true;
      }
    } else if (!isY) {
      System.out.println("CURRENT POSEX");
      double currentPose = drivetrain.getState().Pose.getX();
      System.out.println(currentPose);
      if(currentPose < targetPose) {
        System.out.println("LESS THAN VALUE");
        finished = false;
        drivetrain.setControl(
          robotCentric
          .withVelocityX(0)
          .withVelocityY(-.12 * maxSpeed)
          .withRotationalRate(0)
        );
      } else if(currentPose > targetPose) {
        System.out.println("GREATER THAN VALUE");
        finished = false;
        drivetrain.setControl(
          robotCentric
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0)
        );
      } else {
        drivetrain.setControl(
          robotCentric
            .withVelocityX(0) // forward backward
            .withVelocityY(0) // left right
            .withRotationalRate(0));
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("INIT POSE");
    System.out.println(initialPoseX);
    System.out.println("TARGET POSE");
    System.out.println(targetPose);
    System.out.println("END POSE");
    System.out.println(drivetrain.getState().Pose.getX());
    drivetrain.setControl(
        robotCentric
          .withVelocityX(0) // forward backward
          .withVelocityY(0) // left right
          .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
