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
  private boolean done = false;
  private double targetPosition;
  
  /** Creates a new ShiftRight. */
  public ShiftRight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble() - 3;
    System.out.println("BEGAN SHIFT TO RIGHT BRANCH.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("SHIFTING TO RIGHT.");
    if (drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble() == targetPosition) {
      done = true;
    } else { 
      drivetrain.applyRequest( () -> {
        return robotCentric
          .withVelocityX(0)
          .withVelocityY(.2)
          .withRotationalRate(0);
      });
    }
    // if (drivetrain.getState().Pose.getY() == targetPose) {
    //   done = true;
    // } else {
    //   drivetrain.applyRequest( () -> {
    //     return robotCentric
    //       .withVelocityY(.2);
    //   });
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FINISHED SHIFT TO RIGHT BRANCH.");
    drivetrain.applyRequest( () -> {
      return robotCentric
        .withVelocityX(0)
        .withVelocityY(.0)
        .withRotationalRate(0);
    });
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
