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
import frc.robot.Constants.LLSettings;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StopAll;
import frc.robot.commands.Climber.ClimberPivot;
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
import frc.robot.commands.Limelight.GetInRange;

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
        NamedCommands.registerCommand("L4", new L4(elevator, coral).withTimeout(2.5));
        NamedCommands.registerCommand("Score", new ShootCoral(coral).withTimeout(1));
        

        // LEFT ALIGN ONLY 
        // NamedCommands.registerCommand("leftalign", drivetrain.applyRequest( () -> {
        //     if(getPipeline() == 1) {
        //         setLeftAlignPipeline();
        //     }
        //     final double yvel = slide();
        //     final double xvel = blob();
        //     // System.out.println("ALIGNING");
        //     // when we have both left and right align, 
        //     // then we'll put a line of code to set which pipeline to use
        //     return forwardStraight.withVelocityX(xvel)
        //         .withVelocityY(yvel)
        //         .withRotationalRate(0);
        // }).withTimeout(2));
        
        NamedCommands.registerCommand("Home", new ElevatorToZero(elevator, -.90));
        NamedCommands.registerCommand("Grab", new IntakeCoral(coral));
        NamedCommands.registerCommand("Field Centric", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        camera.streamVideo();
        camera2.streamVideo();

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

    private PIDController xController = new PIDController(1.5, 0.0, 0);  // Vertical movement
    private PIDController yController = new PIDController(1.5, 0.0, 0);  // Horitontal movement
    private PIDController rotController = new PIDController(.009, 0, 0);  // Rotation
    boolean isConfigured = false;
    double tagID = -1;

    void setupAlign() {
        rotController.setSetpoint(LLSettings.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(LLSettings.ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(LLSettings.X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(LLSettings.X_TOLERANCE_REEF_ALIGNMENT);
    
        yController.setSetpoint(LLSettings.Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(LLSettings.Y_TOLERANCE_REEF_ALIGNMENT);
        tagID = LimelightHelpers.getFiducialID("limelight");
        isConfigured = true;
    }

    double[] runAlign() {
        double[] values = {0, 0, 0};
        if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getFiducialID("limelight") == tagID) {
            //this.dontSeeTagTimer.reset();
      
            System.out.println("SAW ATAG");
      
            double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
            SmartDashboard.putNumber("x", postions[2]);
      
            double xSpeed = xController.calculate(postions[2]);
            SmartDashboard.putNumber("xspeed", xSpeed);
            double ySpeed = -yController.calculate(postions[0]);
            double rotValue = rotController.calculate(postions[4]);

            values[0] = xSpeed;
            values[1] = ySpeed;
            values[2] = rotValue;
            // drive
            //   .withVelocityX(xSpeed)
            //   .withVelocityY(ySpeed)
            //   .withRotationalRate(rotValue);
            
      
            if (!rotController.atSetpoint() ||
                !yController.atSetpoint() ||
                !xController.atSetpoint()) {
              //stopTimer.reset();
            }
          } else {
            System.out.println("OUT OF SIGHT");
            values[0] = 0;
            values[1] = 0;
            values[2] = 0;
        //    drive
        //     .withVelocityX(0)
        //     .withVelocityY(0)
        //     .withRotationalRate(0);
          }
          
          System.out.println("VALUES ----");
            System.out.println(values[0]);
            System.out.println(values[1]);
            System.out.println(values[2]);

        SmartDashboard.putNumberArray("PID ALIGN VALUES", values);
          return values;
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

        joystick.povRight().onTrue(new AutoAlignToReef(false, drivetrain, MaxSpeed)
            .withTimeout(2.5));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        // Elevator
        buttonBoard.button(OperatorConstants.HOME).onTrue(new ElevatorToZero(elevator, -.9));
        buttonBoard.button(OperatorConstants.L4).onTrue(new L4(elevator, coral).withTimeout(2.5));
        buttonBoard.button(OperatorConstants.L3).onTrue(new L3(elevator, coral).withTimeout(2.5));
        
        // Algae
        buttonBoard.button(OperatorConstants.algaeOut).onTrue(new SpitAlgae(collector).withTimeout(.50));
        buttonBoard.button(OperatorConstants.algaeBack).onTrue(new PivotBack(collector));
        buttonBoard.button(OperatorConstants.algaePivot).onTrue(new PivotToSetpoint(collector));
        //buttonBoard.button(8).onTrue(new IntakeWithoutBrake(coral, .5));

        buttonBoard.button(OperatorConstants.StopAll).onTrue(new StopAll(collector, coral, elevator));
        buttonBoard.button(OperatorConstants.algaeIn).onTrue(new IntakeAndPivot(collector, .7)
            .until( () -> !collector.hasAlgae())
            .andThen(new PivotBack(collector)));
        

        //joystick.y().whileTrue(new ClimberPivot(climber, .8));
        //joystick.a().whileTrue(new ClimberPivot(climber, -.8));
        
        joystick.a().whileTrue(new Elevate(elevator, .5));
        joystick.y().whileTrue(new Elevate(elevator, -.5));

        // joystick.y().onTrue(new IntakeAndPivot(collector, .7)
        //     .until( () -> !collector.hasAlgae())
        //     .andThen(new PivotBack(collector)));
        // joystick.a().onTrue(new SpitAlgae(collector).withTimeout(.50));
        
        //joystick.button(7).onTrue(new StopAll(collector, coral, elevator));
        //joystick.button(8).onTrue(new StopAll(collector, coral, elevator));


        // NAFI'S CODE
        // joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> {
        //     if (LimelightHelpers.getCurrentPipelineIndex("limelight") != 4) 
        //         LimelightHelpers.setPipelineIndex("limelight", 4);
        //     double adjust = 0;
        //     final double kP = -0.058;
        //     final double steerAdjust = -0.06;
        //     final double strafeAdjust = -0.07;
        //     double tx = LimelightHelpers.getTX("limelight");            

        //     if (LimelightHelpers.getCurrentPipelineIndex("limelight") != 4) 
        //         LimelightHelpers.setPipelineIndex("limelight", 4);

        //     if (LimelightHelpers.getTV("limelight") == true) {
        //         double error = 7.5 - Limelight.autoEstimateDistance();
        //         adjust = kP * error;

        //         System.out.println("Driving Towards Target");
        //         System.out.println("Driving Towards Target");
        //         SmartDashboard.putBoolean("AprilTag Driving", true);
                

        //         return forwardStraight.
        //             withVelocityX(adjust)
        //             .withVelocityY(tx * strafeAdjust)
        //             .withRotationalRate(tx * steerAdjust);

        //     }   
            
        //     return forwardStraight.
        //             withVelocityX(0)
        //             .withVelocityY(0)
        //             .withRotationalRate(0);
        // }));

        // // nafi
        // joystick.povRight().whileTrue(drivetrain.applyRequest(() -> {
        //     SmartDashboard.putBoolean("Is Pressed", true);
        //     double adjust = 0;
        //     final double kP = -0.058;
        //     final double steerAdjust = -0.06;
        //     final double strafeAdjust = -0.05;
        //     double tx = LimelightHelpers.getTX("limelight");            

        //     if (LimelightHelpers.getCurrentPipelineIndex("limelight") != 4) 
        //         LimelightHelpers.setPipelineIndex("limelight", 4);

        //     if (LimelightHelpers.getTV("limelight") == true) {
        //         double error = 7.5 - Limelight.autoEstimateDistance();
        //         adjust = kP * error;

        //         System.out.println("Driving Towards Target");
        //         System.out.println("Driving Towards Target");
        //         SmartDashboard.putBoolean("AprilTag Driving", true);

        //         SmartDashboard.putNumber("adjust", adjust);
        //         SmartDashboard.putNumber("strafe adjust", tx * strafeAdjust);
        //         SmartDashboard.putNumber("steer adjust", tx * steerAdjust);
                

        //         return forwardStraight.
        //             withVelocityX(adjust)
        //             .withVelocityY(tx * strafeAdjust)
        //             .withRotationalRate(tx * steerAdjust);

        //     }   
            
        //     return forwardStraight.
        //             withVelocityX(0)
        //             .withVelocityY(0)
        //             .withRotationalRate(0);
        // }));

        // joystick.povDown().whileTrue(drivetrain.applyRequest(() -> {
        //     double adjust = -0.06;

        //     if (LimelightHelpers.getTV("limelight") == true) {
        //         return drive.
        //                 withVelocityY(LimelightHelpers.getTX("limelight") * adjust);
        //     }
        //     return drive.withVelocityY(0);
        // }));

        // joystick.povUp().whileTrue(drivetrain.applyRequest(() -> {
        //     double steerAdjust = -0.1;

        //     if (LimelightHelpers.getTV("limelight") == true) {
        //         double tx = LimelightHelpers.getTX("limelight");

        //         return drive.
        //             withRotationalRate(tx * steerAdjust);
        //     }

        //     return drive.withRotationalRate(0);
        // }));

        // CLIMBER //
        // joystick.povUp().whileTrue(new PivotOnly(climber, .1));
        // joystick.povRight().whileTrue(new WinchOnly(climber, 1.0));
        // joystick.povDown().whileTrue(new WinchOnly(climber, -1.0));

        buttonBoard.button(OperatorConstants.intakeCoral).onTrue(new IntakeCoral(coral));
        buttonBoard.button(OperatorConstants.shootCoral).onTrue(new ShootCoral(coral).withTimeout(1).andThen(new ElevatorToZero(elevator, -.9)));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.rightBumper().whileTrue(new ReverseCoral(coral));
        
        //joystick.rightBumper().whileTrue(new ManuallyIntakeCoral(coral, .15));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
