// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.DriveTrain;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  DriveTrain DT = null;
  Balance balance = null;
  GenericEntry pEntry = Shuffleboard.getTab("AutoMenu").add("p",0).withWidget(BuiltInWidgets.kTextView).getEntry();
  GenericEntry iEntry = Shuffleboard.getTab("AutoMenu").add("i",0).withWidget(BuiltInWidgets.kTextView).getEntry();
  GenericEntry dEntry = Shuffleboard.getTab("AutoMenu").add("d",0).withWidget(BuiltInWidgets.kTextView).getEntry();
  boolean calibrated = false;
  SlewRateLimiter filter = new SlewRateLimiter(0.5);
  public AutoBalanceCommand(DriveTrain DT, Balance balance ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DT = DT;
    this.balance = balance;
    addRequirements(DT,balance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!calibrated){
      balance.reset(); 
      calibrated = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = gyro.getRoll() - balance.zero;
    SmartDashboard.putNumber("Pitch", pitch);
    double speed = filter.calculate(Math.min(Math.max(balance.out, -0.3), 0.3));
    SmartDashboard.putNumber("PID output", speed);
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
