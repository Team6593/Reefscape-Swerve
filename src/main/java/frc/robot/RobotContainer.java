// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.Coral.IntakeWithoutBrake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.KrakenElevator;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StopAll;
import frc.robot.commands.Climber.ClimberPivot;
import frc.robot.commands.Climber.WinchOnly;
import frc.robot.commands.Collector.IntakeAlgae;
import frc.robot.commands.Collector.IntakeAndPivot;
import frc.robot.commands.Collector.IntakeUntilSwitch;
import frc.robot.commands.Collector.MovePivot;
import frc.robot.commands.Collector.PivotBack;
import frc.robot.commands.Collector.PivotToSetpoint;
import frc.robot.commands.Collector.SpitAlgae;
import frc.robot.commands.Coral.IntakeCoral;
import frc.robot.commands.Coral.ShootCoral;
import frc.robot.commands.Coral.ShootWithoutBrake;
import frc.robot.commands.Elevator.Elevate;
import frc.robot.commands.Elevator.ElevatorToZero;

import frc.robot.commands.Elevator.ElevatorBrake;
import frc.robot.commands.Elevator.HumanStation;
import frc.robot.commands.Elevator.L0;
import frc.robot.commands.Elevator.L1;
import frc.robot.commands.Elevator.L2;
import frc.robot.commands.Elevator.L3;
import frc.robot.commands.Elevator.L4;
import frc.robot.commands.Elevator.StopElevator;
import frc.robot.commands.KrakenElevator.KrakenElevate;
import frc.robot.commands.Limelight.GetInRange;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // private final Coral outtake = new Coral();

    private final Elevator elevator = new Elevator();

    //private final KrakenElevator krakenElevator = new KrakenElevator();

    private final Climber climber = new Climber();

    public final Coral coral = new Coral();

    //private final Climber climber = new Climber();

    private final Collector collector = new Collector();

    private final Camera camera = new Camera();

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandJoystick buttonBoard  = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register Commands before AutoBuilder is initialized!
        //NamedCommands.registerCommand("Intake Coral", new IntakeCoral(outtake).withTimeout(2));
        //NamedCommands.registerCommand("Shoot Coral", new ShootCoral(outtake).withTimeout(1));
        NamedCommands.registerCommand("L4", new L4(elevator, coral).withTimeout(2.5));
        NamedCommands.registerCommand("Score", new ShootCoral(coral).withTimeout(1));
        NamedCommands.registerCommand("Home", new ElevatorToZero(elevator, -.90));
        NamedCommands.registerCommand("Grab", new IntakeCoral(coral));
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        camera.streamVideo();

        configureBindings();
    }


    public void stopClimber() {
        climber.stopClimber();
    }

    public void stopElevator() {
        elevator.stop();
    }

    // public void stopKrakenElevator() {
    //     krakenElevator.stop();
    // }

    public void stopCollector() {
        collector.stop();
    }

    double aim() {
        //System.out.println("AIMING");
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .0095;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= MaxAngularRate;

        //invert since tx is positive when the target is to the right of the crosshair
        //targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    /**
     * effectively the same as aim(), except used for y velocity of swerve
     * @return y velocity
     */
    double slide() {
        double kP = -0.009;
        double targetingYVelocity = LimelightHelpers.getTX("limelight") * kP;
        return targetingYVelocity *= MaxSpeed;
    }

    double range() {
        double kP = -.1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= MaxSpeed;
        //targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    double blob() {
        double blobArea = LimelightHelpers.getTA("limelight");
        //double target = 27.771;
        double target = 35;
        double kP = 0.01;
        System.out.println(blobArea);

        if (Limelight.hasValidTargets() == 1) {
            double error = target - blobArea;
            return kP * error * MaxSpeed;
        } else {
            return 0;
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double deadband = 0;
                double multiplier = -.7;
                double rotationalMultiplier = -1;

                double velocityX = joystick.getLeftY() * multiplier;
                double velocityY = joystick.getLeftX() * multiplier;
                double rotationalRate = joystick.getRightX() * rotationalMultiplier;
    
                // Apply deadband to velocityX
                if (Math.abs(velocityX) < deadband) {
                    velocityX = 0.0;
                } else {
                    velocityX = (velocityX - Math.signum(velocityX) * deadband) / (1 - deadband);
                }
    
                // Apply deadband to velocityY
                if (Math.abs(velocityY) < deadband) {
                    velocityY = 0.0;
                } else {
                    velocityY = (velocityY - Math.signum(velocityY) * deadband) / (1 - deadband);
                }
    
                // Apply deadband to rotationalRate
                if (Math.abs(rotationalRate) < deadband) {
                    rotationalRate = 0.0;
                } else {
                    rotationalRate = (rotationalRate - Math.signum(rotationalRate) * deadband) / (1 - deadband);
                }
                
                return drive
                    .withVelocityX(velocityX * MaxSpeed)
                    .withVelocityY(velocityY * MaxSpeed)
                    .withRotationalRate(rotationalRate * MaxAngularRate);
            })
        );

        //.x().whileTrue(new IntakeWithoutBrake(coral, .2));

        // joystick.y().whileTrue(new ElevatorCoast(elevator));
        // joystick.y().whileFalse(new ElevatorBrake(elevator));
        //joystick.a().whileTrue(new Elevate(elevator, .2));
        //joystick.y().whileTrue(new Elevate(elevator, -.2));

        // joystick.y().whileTrue(drivetrain.applyRequest(() -> {
        //     final double rotation = aim();
        //     final double forward = range();
        //     return forwardStraight.withVelocityX(forward)
        //         .withVelocityY(0)
        //         .withRotationalRate(rotation);
        // })
        // );

        // joystick.y().whileTrue(drivetrain.applyRequest(() -> {
        //     final double rotation = aim();
        //     double forwardspeed = 0;
        //     if(Limelight.hasValidTargets() == 1) {
        //         forwardspeed = .2;
        //     } else {
        //         forwardspeed = 0;
        //     }
        //     //final double forward = range();
        //     return forwardStraight.withVelocityX(forwardspeed)
        //         .withVelocityY(0)
        //         .withRotationalRate(rotation);
        // })
        // );

        joystick.povRight().whileTrue(drivetrain.applyRequest( () -> {
            final double yvel = slide();
            final double xvel = blob();
            //System.out.println("ALIGNING");
            // when we have both left and right align, 
            // then we'll put a line of code to set which pipeline to use
            return forwardStraight.withVelocityX(xvel)
                .withVelocityY(yvel)
                .withRotationalRate(0);
        }));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        buttonBoard.button(OperatorConstants.L1).onTrue(new ElevatorToZero(elevator, -.90));
        buttonBoard.button(OperatorConstants.L3).onTrue(new L4(elevator, coral).withTimeout(2.5));
        buttonBoard.button(OperatorConstants.L2).onTrue(new L3(elevator, coral).withTimeout(2.5));
        //buttonBoard.button(OperatorConstants.L2).onTrue(new L2(elevator));

        // buttonBoard.button(OperatorConstants.L1).onTrue(new WinchOnly(climber, -.2));
        // buttonBoard.button(OperatorConstants.L2).onTrue(new WinchOnly(climber, .2));
        //buttonBoard.button(7).onTrue(new IntakeViaSensor(coral, .5));                      
        buttonBoard.button(5).onTrue(new SpitAlgae(collector).withTimeout(.50));
        buttonBoard.button(6).onTrue(new PivotBack(collector));
        //buttonBoard.button(8).onTrue(new IntakeWithoutBrake(coral, .5));
        buttonBoard.button(10).onTrue(new StopAll(collector, coral, elevator));
        buttonBoard.button(4).onTrue(new IntakeAndPivot(collector, .7)
            .until( () -> !collector.hasAlgae())
            .andThen(new PivotBack(collector)));
        buttonBoard.button(9).onTrue(new PivotToSetpoint(collector));

        //joystick.y().whileTrue(new ClimberPivot(climber, .8));
        //joystick.a().whileTrue(new ClimberPivot(climber, -.8));
        
        joystick.a().whileTrue(new Elevate(elevator, .3));
        joystick.y().whileTrue(new Elevate(elevator, -.3));
        
        joystick.x().whileTrue(new WinchOnly(climber, .5));
        joystick.b().whileTrue(new WinchOnly(climber, -.5));

        // Algae Commands 
        // joystick.x().onTrue(new IntakeAndPivot(collector, .7)
        //     .until( () -> !collector.hasAlgae())
        //     .andThen(new PivotBack(collector)));
        // joystick.b().onTrue(new SpitAlgae(collector).withTimeout(.50));


        //joystick.a().whileTrue(new KrakenElevate(krakenElevator, -.1));
        //joystick.y().whileTrue(new KrakenElevate(krakenElevator, .1));

        //joystick.b().whileTrue(new WinchOnly(climber, .6));

        buttonBoard.button(7).onTrue(new IntakeCoral(coral));
        buttonBoard.button(8).onTrue(new ShootCoral(coral).withTimeout(1).andThen(new ElevatorToZero(elevator, -.9)));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
