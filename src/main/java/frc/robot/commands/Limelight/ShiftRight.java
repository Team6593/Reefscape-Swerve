// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShiftRight extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double targetPosey;
  private double initialPoseY;
  private double maxSpeed;
  private boolean finished = false;
  
  /** Creates a new ShiftRight. */
  public ShiftRight(CommandSwerveDrivetrain drivetrain, double maxSpeed) {
    this.drivetrain = drivetrain;
    this.maxSpeed = maxSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPoseY = drivetrain.getState().Pose.getY();
    targetPosey = initialPoseY + 0.35;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.getState().Pose.getY() > targetPosey) {
      // when the robot gets very close to the target, it will oscillate and "jitter"
      // which is expected behaviour when using bang-bang control
      // so we'll just end the command here and stop moving the bot
      System.out.println("POSITIVE VALUE");
      finished = true;
      drivetrain.setControl(
        robotCentric
        .withVelocityX(0)
        .withVelocityY(0.0 * maxSpeed)
        .withRotationalRate(0)
      );
    } else if(drivetrain.getState().Pose.getY() < targetPosey) {
      System.out.println("NEGATIVE VALUE");
      finished = false;
      drivetrain.setControl(
        robotCentric
        .withVelocityX(0)
        .withVelocityY(-0.05 * maxSpeed)
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
