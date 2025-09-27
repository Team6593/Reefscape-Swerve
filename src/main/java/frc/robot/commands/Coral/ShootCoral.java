// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoral extends Command {

  private Coral coral;

  /** Creates a new ShootCoral. */
  public ShootCoral(Coral coral) {
    this.coral = coral;

    addRequirements(coral);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(coral.isOnL4()) {
      coral.shootCoral(-.675);
      System.out.println("L4");
    } else {
      System.out.println("NOT L4");
      coral.shootCoral(-.75);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
