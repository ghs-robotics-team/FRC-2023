// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmPivot;

public class AutoAlignArmToHeightCommand extends CommandBase {
  /** Creates a new AutoAlignRobotToTapeCommand. */
  NetworkTableEntry error = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");//Vertical offset in degrees between the crosshair and the vision tape.

  ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.3, 0.1));

  ArmPivot armPivot = null;
  public AutoAlignArmToHeightCommand(ArmPivot armPivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armPivot = armPivot;
    addRequirements(armPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armPivot.turning(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setGoal(0);
    double speed = controller.calculate(error.getDouble(0));
    armPivot.turning(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armPivot.turning(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
