// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorAlgae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorAlgae;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorAlgaeToAngle extends Command {

  private ElevatorAlgae elevatorAlgaeMech = new ElevatorAlgae();
  private double setpoint;

  /** Creates a new ElevatorAlgaeToAngle. 
   * @param elevatorAlgaeMech - Elevator Algae Mechanism subsystem.
   * @param setpoint - COUNTER CLOCKWISE. SETPOINTS SHOULD BE -45 AND -90
  */
  public ElevatorAlgaeToAngle(ElevatorAlgae elevatorAlgaeMech, double setpoint) {
    this.elevatorAlgaeMech = elevatorAlgaeMech;
    this.setpoint = setpoint;

    addRequirements(elevatorAlgaeMech);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorAlgaeMech.moveToSetpoint(setpoint);
    System.out.println("ELEVATOR ALGAE MECH BEGINNING MOVEMENT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("ELEVATOR ALGAE MECH MOVING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ELEVATOR ALGAE MECH ENDING MOVEMENT");
    elevatorAlgaeMech.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
