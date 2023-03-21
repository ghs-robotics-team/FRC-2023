// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.helper.SetPoints;
import frc.robot.subsystems.DriveTrain;

public class LimelightCommand extends CommandBase {
  //5 front profiles
  //2 back profiles
  /** Creates a new LimelightCommand. */
  private NetworkTable limelightFront = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTable limelightBack = NetworkTableInstance.getDefault().getTable("limelight-back");
  private DriveTrain drive;
  private ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new Constraints(0.1,0.1));
  private ShuffleboardTab tab = Shuffleboard.getTab("PID");
  private GenericEntry p = tab.add("Rotate P",0).getEntry();
  private GenericEntry i = tab.add("Rotate I",0).getEntry();
  private GenericEntry d = tab.add("Rotate D",0).getEntry();
  private GenericEntry maxSpeedEntry = tab.add("Max Rotation Speed",0).getEntry();
  private GenericEntry maxAccelEntry = tab.add("Max Rotation Accel",0).getEntry();
  public LimelightCommand(DriveTrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setPID(p.getDouble(0), i.getDouble(0), d.getDouble(0));
    pid.setConstraints(new Constraints(maxSpeedEntry.getDouble(0), maxAccelEntry.getDouble(0)));
    if(OperatorConstants.armSetPoint == SetPoints.PlaceHigh){
      //Align High
      if(SetPoints.cubeMode){
        //Front April Tag
        if(limelightFront.getEntry("pipeline").getDouble(0) != 0){
          limelightFront.getEntry("pipeline").setDouble(0);
        }
      }else{
        //Front Top Tape
        if(limelightFront.getEntry("pipeline").getDouble(0) != 3){
          limelightFront.getEntry("pipeline").setDouble(3);
        }else{
          double speed = pid.calculate(limelightFront.getEntry("tx").getDouble(0));
          speed = Math.copySign(Math.min(Math.abs(speed),maxSpeedEntry.getDouble(0)), speed);
          drive.tankdrive(-speed, speed);
        }
      }
    }
    if(OperatorConstants.armSetPoint == SetPoints.PlaceMid){
      //Align Mid
      if(SetPoints.cubeMode){
        //Front April Tag
        if(limelightFront.getEntry("pipeline").getDouble(0) != 0){
          limelightFront.getEntry("pipeline").setDouble(0);
        }
      }else{
        //Front Bottom Tape
        if(limelightFront.getEntry("pipeline").getDouble(0) != 4){
          limelightFront.getEntry("pipeline").setDouble(4);
        }else{
          double speed = pid.calculate(limelightFront.getEntry("tx").getDouble(0));
          speed = Math.copySign(Math.min(Math.abs(speed),maxSpeedEntry.getDouble(0)), speed);
          drive.tankdrive(-speed, speed);
        }
      }
    }
    if(OperatorConstants.armSetPoint == SetPoints.GrabGround){
      //Align Grab Ground Front
      if(SetPoints.cubeMode){
        //Front Cube
        if(limelightFront.getEntry("pipeline").getDouble(0) != 1){
          limelightFront.getEntry("pipeline").setDouble(1);
        }else{
          double speed = pid.calculate(limelightFront.getEntry("tx").getDouble(0));
          speed = Math.copySign(Math.min(Math.abs(speed),maxSpeedEntry.getDouble(0)), speed);
          drive.tankdrive(-speed, speed);
        }
      }else{
        //Front Cone
        if(limelightFront.getEntry("pipeline").getDouble(0) != 2){
          limelightFront.getEntry("pipeline").setDouble(2);
        }else{
          double speed = pid.calculate(limelightFront.getEntry("tx").getDouble(0));
          speed = Math.copySign(Math.min(Math.abs(speed),maxSpeedEntry.getDouble(0)), speed);
          drive.tankdrive(-speed, speed);
        }
      }
    }
    if(OperatorConstants.armSetPoint == SetPoints.GrabIntake){
      //Align Grab Ground Back
      if(SetPoints.cubeMode){
        //Back Cube
      }else{
        //Back Cone
      }
    }
    if(OperatorConstants.armSetPoint == SetPoints.GrabSubstation){
      //Align Grab Substation
      if(SetPoints.cubeMode){
        //Front Cube
        if(limelightFront.getEntry("pipeline").getDouble(0) != 1){
          limelightFront.getEntry("pipeline").setDouble(1);
        }else{
          double speed = pid.calculate(limelightFront.getEntry("tx").getDouble(0));
          speed = Math.copySign(Math.min(Math.abs(speed),maxSpeedEntry.getDouble(0)), speed);
          drive.tankdrive(-speed, speed);
        }
      }else{
        //Front Cone
        if(limelightFront.getEntry("pipeline").getDouble(0) != 2){
          limelightFront.getEntry("pipeline").setDouble(2);
        }else{
          double speed = pid.calculate(limelightFront.getEntry("tx").getDouble(0));
          speed = Math.copySign(Math.min(Math.abs(speed),maxSpeedEntry.getDouble(0)), speed);
          drive.tankdrive(-speed, speed);
        }
      }
    }
    if(OperatorConstants.armSetPoint == SetPoints.Home){
      if(limelightFront.getEntry("pipeline").getDouble(0) != 5){
        limelightFront.getEntry("pipeline").setDouble(5);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
