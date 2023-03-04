// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import java.util.Collections;
//import java.util.List;
//import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTableEntry;

public class limelight3 extends SubsystemBase {
  public double[] cameratotarget;
  Pose3d cameraPose;

  double targetX;
  double targetY;

  double targetYaw;
  double targetPitch;
  double targetRoll;

  int targetId;

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  public limelight3() {}

  public double getTX() {
    targetX = limelightTable.getEntry("tx").getDouble(0);
    return (targetX);
  }

  public double getTY() { //up/down, ty without *-1 is negative up/positive down
    targetY = -1 * limelightTable.getEntry("ty").getDouble(0);
    return (targetY);
  }

  public double getYaw(){ //for rotation
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, roll, pitch, yaw
    targetYaw = cameratotarget[5];
    return (targetYaw);
  }

  public double getPitch(){ //for rotation
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, roll, pitch, yaw
    targetPitch = cameratotarget[4];
    return (targetPitch);
  }

  public double getRoll(){ //for rotation
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, roll, pitch, yaw
    targetRoll = cameratotarget[3];
    return (targetRoll);
  }

  public double getZ(){ //for rotation
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, roll, pitch, yaw
    return (cameratotarget[2]);
  }

  public double getY(){ //for rotation
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, roll, pitch, yaw
    return (cameratotarget[1]);
  }

  public double getX(){ //for rotation
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, roll, pitch, yaw
    return (cameratotarget[0]);
  }

  public double[] getCamtoTarget() {
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, roll, pitch, yaw
    return (cameratotarget);
  }

  public int getId() {
    targetId = (int) limelightTable.getEntry("tid").getInteger(-1);
    return (targetId);
  }

  public void ledOn() {
    limelightTable.getEntry("ledMode").setNumber(1);
  }

  public void ledOff() {
    limelightTable.getEntry("ledMode").setNumber(0);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
