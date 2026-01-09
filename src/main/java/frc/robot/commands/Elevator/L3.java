// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3 extends Command {
  
  Elevator elevator;
  Coral coral;
  boolean done = false;

  /** Creates a new L1. */
  public L3(Elevator elevator, Coral coral) {
    this.elevator = elevator;
    this.coral = coral;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //elevator.changeToCoastMode();
    System.out.println("L3 INIT");
    elevator.revampedElevate(-7); // (-36 / 25) * 15
    coral.setL4(false);

    // newTicks = oldTicks * (newRatio/oldRatio)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("L3 Run");
    
    // if(elevator.getRightEncoderReading() == -83) {
    //   done = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //elevator.changeToBrakeMode();
    System.out.println("l3 end");
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
