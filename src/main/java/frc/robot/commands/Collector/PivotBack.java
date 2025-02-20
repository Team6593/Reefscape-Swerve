// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Collector;

import frc.robot.subsystems.Collector;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotBack extends Command {

  private Collector collector;

  private boolean done = false;

  /** Creates a new PivotBack. */
  public PivotBack(Collector collector) {
    this.collector = collector;

    addRequirements(collector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.pivotBack();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((Math.round(collector.pivotEncoder.getPosition() * 100) / 100) == 1) {
      done = true;
    }
    System.out.println("PIVOTING BACK");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
