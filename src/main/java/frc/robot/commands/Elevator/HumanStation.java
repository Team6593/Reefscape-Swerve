// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Elevator;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Elevator;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class HumanStation extends Command {
//   private Elevator elevator;
//   /** Command that elevates the robot to human/coral station.
//    * @param Elevator - Elevator object of the subsytem
//   */
//   public HumanStation(Elevator elevator) {
//     this.elevator = elevator;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(elevator);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     elevator.goToSetpoint(21.8);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}


//   // 214
//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
