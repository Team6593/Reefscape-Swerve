// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorIO;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorIOHome extends Command {

  private ElevatorIO elevator;

  /** Creates a new ElevatorIOHome. */
  public ElevatorIOHome(ElevatorIO elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.limitSwitch.get()) {
      elevator.rawSpeed(-.2);
    } else {
      elevator.stop();
      elevator.resetEncoder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !elevator.limitSwitch.get();
  }
}
