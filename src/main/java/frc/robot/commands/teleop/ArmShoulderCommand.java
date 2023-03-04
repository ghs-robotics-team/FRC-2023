// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

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
  private GenericEntry p = tab.add("Shoulder P",1).getEntry();
  private GenericEntry i = tab.add("Shoulder I",1).getEntry();
  private GenericEntry d = tab.add("Shoulder D",1).getEntry();
  private ArmShoulder subsystem;
  private double angleToTickFactor = 42*30*72/22;
  private ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new Constraints(0.2,0.01));
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
    pid.setP(p.getDouble(0));
    pid.setI(i.getDouble(0));
    pid.setD(d.getDouble(0));
    double targetPos = OperatorConstants.ShoulderTargetAngle*angleToTickFactor;
    double speed = pid.calculate(subsystem.getPos(), targetPos);
    subsystem.setSpeed(Math.max(Math.min(speed,0.2),-0.2));
    if(Math.abs(targetPos-subsystem.getPos()) < 2*Math.PI/360*angleToTickFactor){
      OperatorConstants.ShoulderCorrect = true;
    }else{
      OperatorConstants.ShoulderCorrect = false;
    }
    SmartDashboard.putNumber("Shoulder Target", targetPos);
    SmartDashboard.putNumber("Shoulder Position", subsystem.getPos());
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
