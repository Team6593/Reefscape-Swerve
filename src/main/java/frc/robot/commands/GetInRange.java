// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.OtherConstants.LimelightConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetInRange extends Command {

  private Limelight limelight;
  private RobotContainer robotContainer;
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new GetInRange. */
  public GetInRange(Limelight limelight, CommandSwerveDrivetrain drivetrain) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;

    addRequirements(limelight, drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Limelight.estimateDistance(LimelightConstants.mountAngleDegrees, LimelightConstants.lensHeightInches, LimelightConstants.goalHeightInches) > 10) {
      forwardStraight.withVelocityX(.5);
    } else {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forwardStraight.withVelocityX(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
