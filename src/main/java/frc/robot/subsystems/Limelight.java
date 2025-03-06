// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.LimelightConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  static NetworkTableEntry tx = table.getEntry("tx");
  static NetworkTableEntry ty = table.getEntry("ty");
  static NetworkTableEntry ta = table.getEntry("ta");

  /** Creates a new Limelight. */
  public Limelight() {
    setToAprilTagPipeline();
    System.out.println("Limelight initialized!");
  }

  /**
   * changes pipeline to 0 (apriltag pipeline)
   */
  public static void setToAprilTagPipeline() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  public static void update() {
    // This method will be called once per scheduler run

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);
    SmartDashboard.putNumber("ta", area);

    System.out.println("x:"+ x);
    System.out.println("y:"+ y);
    System.out.println("area:"+ area);


    SmartDashboard.putNumber("Distance,", estimateDistance(LimelightConstants.mountAngleDegrees, LimelightConstants.lensHeightInches, LimelightConstants.goalHeightInches));

  }

  /**
   * Gets the distance from the limelight to the apriltag
   * @param limelightMountAngleDegrees
   * @param goalHeightInches
   * @param limelightLensHeightInches
   * @return distanceFromLimelightToGoalInches
   * 
   */
  public static double getDistanceFromAprilTag(double limelightMountAngleDegrees, 
  double goalHeightInches, double limelightLensHeightInches) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
    / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  /**
   * Think getDistanceFromAprilTag, but for autonomous
   * @return distanceFromLimelightToGoalInches
   */
  public double autoEstimateDistance() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical
    double limelightMountAngleDegrees = 0.0; // grab later

    // distance from the center of the limelight lens to the floor
    double limelightLensHeightInches = 0.0; // grab later

    // distance from the targets center to the floor 
    double goalHeightInches = 12; // grab later

    double angelToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angelToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
     / Math.tan(angleToGoalRadians);

    // return distance
    return distanceFromLimelightToGoalInches;
  }

  /**
   * 
   * @return 1 if there is a valid target, 0 if not
   */
  public static double hasValidTargets() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }

  /**
   * 
   * @return vertical offset of crosshair to target
   */
  public static double getVerticalOffset() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  /**
   * 
   * @return vertical offset of crosshair to target
   */
  public static double getHorizontalOffset() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  /**
   * 
   * @return AprilTag ID as double array
   */
  public static double[] getTagID() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]);
  }

  /**
   * Distance estimation
   * @param mountAngleDegrees - Degrees backwards the mount is from being perfectly vertical
   * @param lensHeightInches - Height from ground to camera lens
   * @param goalHeightInches - Height from ground to apriltag
   * @return distance
   */
  public static double estimateDistance(double mountAngleDegrees, double lensHeightInches, double goalheightInches) {
    double verticalOffset = getVerticalOffset();
    double angleToGoalDegrees = mountAngleDegrees + verticalOffset;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);// was 3.14159
    double distance = Math.abs((goalheightInches - lensHeightInches) / Math.tan(angleToGoalRadians));
    return distance; 
  }

}
