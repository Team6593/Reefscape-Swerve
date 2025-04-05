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
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.commands.Coral.ManuallyIntakeCoral;
import frc.robot.commands.Coral.ReverseCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.KrakenElevator;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.LLSettings1;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShiftBack;
import frc.robot.commands.StopAll;
import frc.robot.commands.Climber.ClimberPivot;
import frc.robot.commands.Climber.ClimberPivotToSetpoint;
import frc.robot.commands.Climber.PivotAndWinch;
import frc.robot.commands.Climber.PivotOnly;
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
import frc.robot.commands.Limelight.AutoAlignToReef;
import frc.robot.commands.Limelight.AutoAlignToReefLeft;
import frc.robot.commands.Limelight.AutoAlignToReefRight;
import frc.robot.commands.Limelight.AutoAlignToStation;
import frc.robot.commands.Limelight.GetInRange;
import frc.robot.commands.Limelight.ShiftRight;
import frc.robot.commands.Limelight.ShiftRightChangable;
import frc.robot.commands.Limelight.ShiftRightX;

public class RobotContainer {

    private PIDController limelightPID = new PIDController(.02, 0, 0);

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

    private final Camera camera = new Camera(0, "Camera 1");
    private final Camera camera2 = new Camera(1, "Camera 2");

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandJoystick buttonBoard  = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final Limelight limelight = new Limelight();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Register Commands before AutoBuilder is initialized!
        //NamedCommands.registerCommand("Intake Coral", new IntakeCoral(outtake).withTimeout(2));
        //NamedCommands.registerCommand("Shoot Coral", new ShootCoral(outtake).withTimeout(1));
        NamedCommands.registerCommand("L4", new L4(elevator, coral).withTimeout(1.5));
        NamedCommands.registerCommand("Score", new ShootCoral(coral)
            .withTimeout(.5));
        NamedCommands.registerCommand("Left Align", new AutoAlignToReefLeft(false, drivetrain, 
        MaxSpeed, MaxAngularRate)
            .withTimeout(1.8));
        NamedCommands.registerCommand("Extended Left Align", new AutoAlignToReefLeft(false, drivetrain, 
        MaxSpeed, MaxAngularRate)
            .withTimeout(2.2));
        NamedCommands.registerCommand("Right Align", (new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
            .withTimeout(2)
            .andThen(new ShiftRight(drivetrain, MaxSpeed))
            .withTimeout(2.75)));
        NamedCommands.registerCommand("Right Align X", (new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
            .withTimeout(2)
            .andThen(new ShiftRightX(drivetrain, MaxSpeed))
            .withTimeout(2.75)));
        NamedCommands.registerCommand("Left Align w/ Stop", new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
            .withTimeout(1.8)
            .andThen(stopDrivetrain()));
        NamedCommands.registerCommand("Stop Drivetrain", stopDrivetrain().withTimeout(.1));
        NamedCommands.registerCommand("Home", new ElevatorToZero(elevator, coral, -.90));
        NamedCommands.registerCommand("Grab", new IntakeCoral(coral));
        NamedCommands.registerCommand("Field Centric", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        NamedCommands.registerCommand("Station Align", new AutoAlignToStation(false, drivetrain, MaxSpeed, MaxAngularRate)
                                                            .withTimeout(.5));

        // Events
        new EventTrigger("Left Align Event")
        .onTrue(new  AutoAlignToReefLeft(false, drivetrain, 
        MaxSpeed, MaxAngularRate));

        new EventTrigger("Right Align Event")
        .onTrue((new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
        .andThen(new ShiftRight(drivetrain, MaxSpeed))));

        new EventTrigger("Scoring Event").onTrue(new L4(elevator, coral)
            .withTimeout(1.5)
            .andThen(new ShootCoral(coral))
            .withTimeout(2)
            .andThen(new ShiftBack(drivetrain, MaxSpeed))
            .withTimeout(2.25)
            .andThen(new ElevatorToZero(elevator, coral, -.90)));

        new EventTrigger("Left Align Score")
                        .onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
                        .andThen(new L4(elevator, coral)
                        .withTimeout(1.5)
                        .andThen(new ShootCoral(coral))
                        .withTimeout(2)
                        .andThen(new ShiftBack(drivetrain, MaxSpeed))
                        .withTimeout(2.25)
                        .andThen(new ElevatorToZero(elevator, coral, -.90))));


        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        camera.streamVideo();
        camera2.streamVideo();

        configureBindings();
    }


    public Command stopDrivetrain() {
        return drivetrain.applyRequest( () -> {
            return drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0);
        });
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
        //LimelightHelpers.setPipelineIndex("limelight", pipeline);
        double blobArea = Math.round(LimelightHelpers.getTA("limelight"));
        //double target = 27.771;
        double target = 27;
        if(Limelight.hasValidTargets() == 1) {
            if(target > blobArea) {
                return 0.05 * MaxSpeed;
            } else {
                return 0;
            }
        } else {return 0;}
    }

    void setRightAlignPipeline() {
        LimelightHelpers.setPipelineIndex("limelight", 1);
    }

    void setLeftAlignPipeline() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    double getPipeline() {
        return LimelightHelpers.getCurrentPipelineIndex("limelight");
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double deadband = 0;
                double multiplier = -1;
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
        
        // Note: The timeouts dont mean the actual robot will be moving for this long, this is to let the
        // scheduler know when the command has "ended"
        // Appropriately change time such that it finishes quickly whilst also running for long enough
        // If the time out is too long, the driver will be unable to do anything for a few seconds
        // joystick.povLeft().onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
        //     .withTimeout(2));
        
        // joystick.povRight().onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
        //     .withTimeout(2).andThen(new ShiftRight(drivetrain, MaxSpeed)).withTimeout(2.75));
        
        // joystick.povDown().onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
        //     .withTimeout(2).andThen(new ShiftRightX(drivetrain, MaxSpeed)).withTimeout(2.75));
        

        // joystick.povDown().onTrue(new AutoAlignToReefRight(false, drivetrain, MaxSpeed, MaxAngularRate)
        //     .withTimeout(3));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        // Elevator
        buttonBoard.button(OperatorConstants.HOME).onTrue(new ElevatorToZero(elevator, coral, -.90));
        buttonBoard.button(OperatorConstants.L4)
            .onTrue(new L4(elevator, coral)
            .withTimeout(1.5)
            .andThen(new ShootCoral(coral))
            .withTimeout(1.75)
            .andThen(new ElevatorToZero(elevator, coral, -.90)));
        //buttonBoard.button(OperatorConstants.L3).onTrue(new L3(elevator, coral).withTimeout(2.5));
        buttonBoard.button(OperatorConstants.L3)
            .onTrue(new L3(elevator, coral)
            .withTimeout(.75)
            .andThen(new ShootCoral(coral))
            .withTimeout(1)
            .andThen(new ElevatorToZero(elevator, coral, -.90)));

        // Algae
        buttonBoard.button(OperatorConstants.algaeOut).onTrue(new SpitAlgae(collector).withTimeout(.50));
        buttonBoard.button(OperatorConstants.algaeBack).onTrue(new PivotBack(collector));
        buttonBoard.button(OperatorConstants.algaePivot)
            .onTrue(new PivotToSetpoint(collector)
            .withTimeout(.5)
            .andThen(new ClimberPivotToSetpoint(climber, 7.1)));
        //buttonBoard.button(8).onTrue(new IntakeWithoutBrake(coral, .5));

        buttonBoard.button(OperatorConstants.StopAll).onTrue(new StopAll(collector, coral, elevator));
        buttonBoard.button(OperatorConstants.algaeIn).onTrue(new IntakeAndPivot(collector, .7)
            .until( () -> !collector.hasAlgae())
            .andThen(new PivotBack(collector)));
        

        //joystick.y().whileTrue(new ClimberPivot(climber, .8));
        //joystick.a().whileTrue(new ClimberPivot(climber, -.8));
        
        // joystick.a().whileTrue(new Elevate(elevator, .5));
        // joystick.y().whileTrue(new Elevate(elevator, -.5));

        // joystick.y().onTrue(new IntakeAndPivot(collector, .7)
        //     .until( () -> !collector.hasAlgae())
        //     .andThen(new PivotBack(collector)));
        // joystick.a().onTrue(new SpitAlgae(collector).withTimeout(.50));
        
        joystick.x().onTrue(new IntakeAndPivot(collector, .7)
            .until( () -> !collector.hasAlgae())
            .andThen(new PivotBack(collector)));
        joystick.a().onTrue(new SpitAlgae(collector).withTimeout(.50));
        joystick.y()
            .onTrue(new PivotToSetpoint(collector)
            .withTimeout(.5)
            .andThen(new ClimberPivotToSetpoint(climber, 5.6)));
        joystick.b().whileTrue(new WinchOnly(climber, -1.0));



        //joystick.button(7).onTrue(new StopAll(collector, coral, elevator));
        //joystick.button(8).onTrue(new StopAll(collector, coral, elevator));

        // CLIMBER
        //joystick.povUp().whileTrue(new PivotOnly(climber, .1));
        //joystick.povRight().whileTrue(new WinchOnly(climber, 1.0));
        joystick.povDown().whileTrue(new WinchOnly(climber, -1.0));
        //joystick.povLeft().onTrue(new ClimberPivotToSetpoint(climber, 5.65));

        // joystick.povLeft().onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
        //     .withTimeout(2));
        // joystick.povRight().onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
        //     .withTimeout(2).andThen(new ShiftRight(drivetrain, MaxSpeed)).withTimeout(3));
        joystick.povUp().whileTrue(new WinchOnly(climber, 1.0));
        joystick.rightBumper().whileTrue(new PivotOnly(climber, .1));
        joystick.povRight().whileTrue(new PivotOnly(climber, -.1));

        // changes based off tag
        // joystick.povUp().onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
        //     .withTimeout(2).andThen(new ShiftRightChangable(drivetrain, MaxSpeed)).withTimeout(2.75));

        // joystick.povLeft().onTrue(new AutoAlignToStation(false, drivetrain, MaxSpeed, MaxAngularRate)
        //     .withTimeout(.5));

        buttonBoard.button(OperatorConstants.intakeCoral).onTrue(new IntakeCoral(coral));
        buttonBoard.button(OperatorConstants.shootCoral).onTrue(new ShootCoral(coral).withTimeout(1).andThen(new ElevatorToZero(elevator, coral, -.90)));

        buttonBoard.button(OperatorConstants.leftAlign).onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
            .withTimeout(2));
        buttonBoard.button(OperatorConstants.rightAlign).onTrue(new AutoAlignToReefLeft(false, drivetrain, MaxSpeed, MaxAngularRate)
            .withTimeout(2).andThen(new ShiftRight(drivetrain, MaxSpeed)).withTimeout(2.75));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.rightBumper().whileTrue(new ReverseCoral(coral));
        
        //joystick.rightBumper().whileTrue(new ManuallyIntakeCoral(coral, .15));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void fieldCentricTheRobot() {
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
    }

    public Command getAutonomousCommand() {
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
