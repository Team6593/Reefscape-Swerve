// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Elevator;

// import frc.robot.subsystems.Coral;
// import frc.robot.subsystems.Elevator;
// import edu.wpi.first.wpilibj2.command.Command;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ElevatorToZero extends Command {

//   private Elevator elevator;
//   private Coral coral;
//   private double speed;

//   /** Creates a new ElevatorToZero. */
//   public ElevatorToZero(Elevator elevator, Coral coral, double speed) {
//     this.elevator = elevator;
//     this.speed = speed;
//     this.coral = coral;

//     addRequirements(elevator);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     //elevator.changeToBrakeMode();
//     coral.setL4(false);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (elevator.limitSwitch.get()) {
//       elevator.reachGoal(-speed);
//     } else {
//       elevator.stop();
//       elevator.resetEncoder();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     elevator.stop();
//     elevator.resetEncoder();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return !elevator.limitSwitch.get();
//   }
// }
