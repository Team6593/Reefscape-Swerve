// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Elevator;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Elevator;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class Elevate extends Command {

//   public Elevator elevator;
//   private double speed;

//   /** 
//    * Command that elevates the robot.
//    * @param Elevator - Elevator object for the subsystem
//    * @param Speed - -1 to 1, - goes down and + goes up
//    * */
//   public Elevate(Elevator elevator, double speed) {
//     this.elevator = elevator;
//     this.speed = speed;

//     addRequirements(elevator);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     elevator.reachGoal(speed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     elevator.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
