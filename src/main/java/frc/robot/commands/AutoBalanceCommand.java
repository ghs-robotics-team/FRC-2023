// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.PID;
import frc.robot.subsystems.DriveTrain;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  PID controller = new PID (0,0,0);
  DriveTrain DT = null;
  GenericEntry pEntry = Shuffleboard.getTab("AutoMenu").add("p",0).withWidget(BuiltInWidgets.kTextView).getEntry();
  GenericEntry iEntry = Shuffleboard.getTab("AutoMenu").add("i",0).withWidget(BuiltInWidgets.kTextView).getEntry();
  GenericEntry dEntry = Shuffleboard.getTab("AutoMenu").add("d",0).withWidget(BuiltInWidgets.kTextView).getEntry();
  public AutoBalanceCommand(DriveTrain DT) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DT = DT;
    addRequirements(DT);
    gyro.calibrate();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.kp = pEntry.getDouble(0);
    controller.ki = iEntry.getDouble(0);
    controller.kd = dEntry.getDouble(0);
    float pitch = gyro.getRoll();
    SmartDashboard.putNumber("Pitch", pitch);
    double speed = Math.max(controller.step(pitch), 0.3);
    DT.tankdrive(speed, speed);
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
