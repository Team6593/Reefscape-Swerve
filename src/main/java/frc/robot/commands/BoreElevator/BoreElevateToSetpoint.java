// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BoreElevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BoreElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BoreElevateToSetpoint extends Command {

  private BoreElevator elevator;
  private double setpoint;
  private double speed;
  private boolean done = false;
  private double position;

  /** Creates a new BoreElevateToSetpoint. 
   * @param BoreElevator - Elevator subsystem, MUST BE THE ONE WITH THE BORE.
   * @param setpoint - Desired setpoint.
   * @param speed - Desired speed.
  */
  public BoreElevateToSetpoint(BoreElevator elevator, double setpoint, double speed) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    this.speed = speed;

    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = elevator.getEncoderReading();
    
    if (Math.round(position) == setpoint) {
      done = true;
    }

    if (position > setpoint) {
      elevator.elevate(-speed);
    } else if (position == setpoint) {
      done = true;
    } else if (setpoint > position) {
      elevator.elevate(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
