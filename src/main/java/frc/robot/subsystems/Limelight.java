// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {
    setToAprilTagPipeline();
  }

  /**
   * changes pipeline to 0 (apriltag pipeline)
   */
  public static void setToAprilTagPipeline() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

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
    double goalHeightInches = 0; // grab later

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
   * @param mountAngleDegrees
   * @param lensHeightInches
   * @param goalHeightInches
   * @return distance
   */
  public static double estimateDistance(double mountAngleDegrees, double lensHeightInches, 
  double goalHeightInches) {
    double verticalOffset = getVerticalOffset();
    double angleToGoalDegrees = mountAngleDegrees + verticalOffset;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180);
    double distance = (goalHeightInches - lensHeightInches) / Math.tan(angleToGoalRadians);

    return distance;
  }

}
