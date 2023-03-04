// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 
package frc.robot.commands;

import frc.robot.subsystems.limelight3;
import frc.robot.subsystems.SwerveSubsystem;
//import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

//import javax.xml.catalog.GroupEntry.PreferType;

//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants.Limelight3Constants;;

public class MoveToTag extends CommandBase {
  private final limelight3 camera;
  private final SwerveSubsystem swerve;

  private PIDController xcontroller;
  private PIDController ycontroller;
  private PIDController rcontroller;

  private final double goalxdist;
  private final double goalydist;
  private final double goalr;

  double initialydistance;
  double initialxdistance;
  double initialr;

  double yangledif;
  double xangle;

  public MoveToTag(limelight3 camera, SwerveSubsystem swerve, double goalxdist, double goalydist, double goalr) {
    this.camera = camera;
    this.swerve = swerve;
    this.goalxdist = goalxdist;
    this.goalydist = goalydist;
    this.goalr = goalr;

    addRequirements(camera);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    

    initialr = camera.getPitch();
    initialxdistance = camera.getX();
    initialydistance = camera.getZ();

    double xpercentdiff = 1;
    double ypercentdiff = 1;

    /*if(initialxdistance>initialydistance){
      ypercentdiff = initialydistance/(initialxdistance*1.2);
    }else{
      xpercentdiff = initialxdistance/(initialydistance*1.2);
    }*/

    xcontroller = new PIDController(Math.abs(xpercentdiff)*Limelight3Constants.kPx, Limelight3Constants.kIx, Limelight3Constants.kDx);
    ycontroller = new PIDController(Math.abs(ypercentdiff)*Limelight3Constants.kPy, Limelight3Constants.kIy, Limelight3Constants.kDy);
    rcontroller = new PIDController(Limelight3Constants.kPr, Limelight3Constants.kIr, Limelight3Constants.kDr);

    xcontroller.setTolerance(Limelight3Constants.xtolerance);
    ycontroller.setTolerance(Limelight3Constants.ytolerance);
    rcontroller.setTolerance(Limelight3Constants.rtolerance);
  }

  @Override
  public void execute() {    


    //y, forward/backward distance from tagpose_cameraspace, meters
    double currentydistance = camera.getZ(); //? getY?

    //x, left/right distance from tagpose_cameraspace, meters
    double currentxdistance = camera.getX();

    //r, rotation of tag around z-axis (up/down) from tagpose_cameraspace, in degrees
    double currentr = camera.getPitch();
    

    double xspeed = -1 * MathUtil.clamp((xcontroller.calculate(currentxdistance, goalxdist)), -Limelight3Constants.xclamp, Limelight3Constants.xclamp);
    double yspeed = -0.3 * MathUtil.clamp((ycontroller.calculate(currentydistance, goalydist)), -Limelight3Constants.yclamp, Limelight3Constants.yclamp);
    double rspeed = -0.1 * MathUtil.clamp((rcontroller.calculate(currentr, goalr)), -Limelight3Constants.rclamp, Limelight3Constants.rclamp); //only works with clockwise rotation for some reason

      //^ mecanum drive, +/- xspeed, rspeed might be issue

    if(camera.getId() == -1){
        xspeed = 0;
        yspeed = 0;
        rspeed = -rspeed;
    }

      //rspeed = 0;
    //System.out.println(xspeed); //xspeed negative when counterclockwise
    System.out.println(rspeed); //rotation negative when counterclockwise

    //stopping individually since command only ends with all 3
    if (xcontroller.atSetpoint()){
      xspeed = 0;
    }
    if (ycontroller.atSetpoint()){
      yspeed = 0;
    }
    if (rcontroller.atSetpoint()){
      rspeed = 0;
    }
    //if (ycontroller.atSetpoint()){
      //yspeed = 0;
    //}
    //if (rcontroller.atSetpoint()){
      //rspeed = 0;
    //}

    //stopping abrupt movement at end
    if (Math.abs(xspeed) < 0.02){
      xspeed = 0;
    }
    if (Math.abs(yspeed) < 0.02){
      yspeed = 0;
    }
    if (Math.abs(rspeed)< 0.02){
      rspeed = 0;
    }

    swerve.drive(xspeed, yspeed, rspeed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return (xcontroller.atSetpoint() && ycontroller.atSetpoint() && rcontroller.atSetpoint()); //was accidentally running without stopping rcontroller in church 2/26
  }
}
