// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private TalonFX frontLeft = new TalonFX(0);
  private TalonFX frontRight = new TalonFX(2);
  private TalonFX backLeft = new TalonFX(1);
  private TalonFX backRight = new TalonFX(3);
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20));//Might also be 10
  private DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();
  private double gearRatio = 1 / 10.71;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
    SlotConfiguration pid = new SlotConfiguration();
    pid.kP = 0.10988;
    pid.kF = 0.1079;
    frontRight.configureSlot(pid);
    frontLeft.configureSlot(pid);
    backRight.configureSlot(pid);
    backLeft.configureSlot(pid);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), ticksToMeters(frontLeft), ticksToMeters(frontRight));
  }
  public void tankdrive(double left, double right){
    frontLeft.set(TalonFXControlMode.PercentOutput,left);
    frontRight.set(TalonFXControlMode.PercentOutput,right);
  }
  public void tankdriveVelocity(double rightVelocity, double leftVelocity){
    //input velocity is in m/s
    //it needs to be converted to tps for control mode velocity
    double r = Units.inchesToMeters(3);
    double leftRotations = (leftVelocity)/(2*r*Math.PI);
    double rightRotations = (rightVelocity)/(2*r*Math.PI);

    leftRotations/=10;
    rightRotations/=10;

    SmartDashboard.putNumber("LeftTargetVelocity",leftRotations*2048);
    SmartDashboard.putNumber("RightTargetVelocity",rightRotations*2048);
    SmartDashboard.putNumber("LeftVelocity", frontLeft.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RightVelocity", frontRight.getSelectedSensorVelocity());

    frontLeft.set(TalonFXControlMode.Velocity, leftRotations*2048);
    frontRight.set(TalonFXControlMode.Velocity, rightRotations*2048);
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  public void resetPose(Pose2d pose){
    if(pose.getX() == 1.8 && pose.getY() == 0.5){
      pose = new Pose2d(1.8,0.5,new Rotation2d(Math.PI));
    }
    odometry.resetPosition(gyro.getRotation2d(), ticksToMeters(frontLeft), ticksToMeters(frontRight), pose);
  }
  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }
  public double ticksToMeters(TalonFX talon){
    double r = Units.inchesToMeters(3);
    //arc length = rad*r;
    double rad = talon.getSelectedSensorPosition()/2048.0 * 2 * Math.PI * gearRatio;
    return rad*r;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), -ticksToMeters(frontLeft), -ticksToMeters(frontRight));
    field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData("Field",field);
  }
}
