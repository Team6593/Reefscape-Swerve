// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L4 extends Command {
  
  Elevator elevator;
  Coral coral;
  boolean done = false;

  /** Creates a new L1. */
  public L4(Elevator elevator, Coral coral) {
    this.elevator = elevator;
    this.coral = coral;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //elevator.changeToCoastMode();
    System.out.println("L4 INIT");
    elevator.revampedElevate(-19); // -92 IS THE REAL SETPOINT, -92.62278747558594, OLD -88; -95; -96
    coral.setL4(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("L4 RUN");
    // if(elevator.getRightEncoderReading() == -230) {
    //   done = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("L4 END");
    //elevator.changeToBrakeMode();
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
