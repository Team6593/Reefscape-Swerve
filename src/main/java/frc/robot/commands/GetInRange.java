// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.OtherConstants.LimelightConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetInRange extends Command {

  private Limelight limelight;
  private RobotContainer robotContainer;
  private CommandSwerveDrivetrain drivetrain;

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
      drivetrain.sysIdDynamic(Direction.kForward);
    } else {
      robotContainer.stopMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotContainer.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
