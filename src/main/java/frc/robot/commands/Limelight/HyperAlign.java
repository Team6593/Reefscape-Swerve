// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HyperAlign extends Command {

  private final CommandSwerveDrivetrain swerve;

  private final double PX = 0.5;
  private final double PY = 0.05;
  private final double PTheta = 0.05;
  private final String llname = "limelight";

  private final SwerveRequest.RobotCentric robotCentric;

  /** Creates a new HyperAlign. */
  public HyperAlign(CommandSwerveDrivetrain swerve, SwerveRequest.RobotCentric robotCentric) {
    this.robotCentric = robotCentric;
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(llname, 3);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = LimelightHelpers.getTX(llname);
    double ty = LimelightHelpers.getTY(llname);

    double vx = ty * PX;
    double vy = -tx * PY * 0;
    double omega = tx * PTheta * 0;

    swerve.setControl(robotCentric
    .withVelocityX(vx)
    .withVelocityY(vy)
    .withRotationalRate(omega));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setControl(robotCentric
    .withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(LimelightHelpers.getTX(llname)) < 1.0 && Math.abs(LimelightHelpers.getTY(llname)) < 1.0;
  }
}
