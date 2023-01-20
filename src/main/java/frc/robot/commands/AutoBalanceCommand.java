// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  DriveTrain DT = null;
  public AutoBalanceCommand(DriveTrain DT) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DT = DT;
    addRequirements(DT);
    gyro.calibrate();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    float pitch = gyro.getRoll();
    SmartDashboard.putNumber("Pitch", pitch);
    if (pitch > 0){
      DT.tankdrive(-0.1,-0.1);
    }
    else{
      DT.tankdrive(0.1, 0.1);
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
