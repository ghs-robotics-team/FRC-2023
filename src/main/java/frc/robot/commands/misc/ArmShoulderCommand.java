// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmShoulder;

public class ArmShoulderCommand extends CommandBase {
  /** Creates a new ArmPivotCommand. */
  private ShuffleboardTab tab = Shuffleboard.getTab("PID");
  private GenericEntry p = tab.add("Shoulder P",7).getEntry();
  private GenericEntry i = tab.add("Shoulder I",0).getEntry();
  private GenericEntry d = tab.add("Shoulder D",0).getEntry();
  private GenericEntry maxSpeedEntry = tab.add("Max Speed",0.6).getEntry();
  private GenericEntry maxAccelEntry = tab.add("Max Accel",1).getEntry();
  private double maxSpeed = 0.15;
  private ArmShoulder subsystem;
  private double angleToTickFactor = OperatorConstants.AngleToTickShoulder;
  private ProfiledPIDController pid = new ProfiledPIDController(50, 2, 0, new Constraints(0.5,1));
  public ArmShoulderCommand(ArmShoulder subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(OperatorConstants.ShoulderTargetAngle > -0.15){
      OperatorConstants.ShoulderTargetAngle = -0.15;
    }
    pid.setConstraints(new Constraints(maxSpeed, maxAccelEntry.getDouble(0.01)));
    pid.setP(p.getDouble(0));
    pid.setI(i.getDouble(0));
    pid.setD(d.getDouble(0));
    maxSpeed = maxSpeedEntry.getDouble(0);
    double targetPos = OperatorConstants.ShoulderTargetAngle;
    double speed = pid.calculate(subsystem.getPos()/angleToTickFactor, targetPos);
    subsystem.setSpeed(Math.max(Math.min(speed,maxSpeed),-maxSpeed));
    if(Math.abs(targetPos-subsystem.getPos()/angleToTickFactor) < 2*Math.PI/360){
      OperatorConstants.ShoulderCorrect = true;
    }else{
      OperatorConstants.ShoulderCorrect = false;
    }
    SmartDashboard.putNumber("Shoulder Target", targetPos);
    SmartDashboard.putNumber("Shoulder Position", subsystem.getPos()/angleToTickFactor);
    SmartDashboard.putNumber("Shoulder PID Speed Output", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
