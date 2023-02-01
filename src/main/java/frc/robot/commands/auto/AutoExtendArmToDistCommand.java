// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtension;

public class AutoExtendArmToDistCommand extends CommandBase {
  NetworkTableEntry ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta");//Percent of the image the tape takes up

  ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.3, 0.1));

  //Test to get the ta values at a bunch of distances, then test to get the corresponding encoder value that the arm motor needs to be at to be at the perfect distance
  //Interpolate from this array to get the target point
  double[] distances = new double[]{0};
  double[] encoderVals = new double[]{0};

  ArmExtension extension = null;
  /** Creates a new AutoExtendArmToDistCommand. */
  public AutoExtendArmToDistCommand(ArmExtension extension) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extension);
    this.extension = extension;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extension.extendArm(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setGoal(getGoal(ta.getDouble(0)));
    controller.calculate(extension.getEncoderValue());
  }

  public double getGoal(double area){
    for(int i = 0; i < distances.length-1; i++){
      if(distances[i] <= area && distances[i+1] > area){
        double p = (area-distances[i])/(distances[i+1]-distances[i]);
        return p*(encoderVals[i+1]-encoderVals[i]) + encoderVals[i];
      }
    }
    return 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extension.extendArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
