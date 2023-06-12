// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.helper.SetPoints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double ShoulderP = 0;
    public static final double ShoulderI = 0;
    public static final double ShoulderD = 0;
    public static final double ElbowP = 0;
    public static final double ElbowI = 0;
    public static final double ElbowD = 0;
    public static boolean ShoulderCorrect = false;
    public static boolean ElbowCorrect = false;
    public static double ShoulderTargetAngle = 0;
    public static boolean StopMovingShoulder = false;
    public static boolean StopMovingElbow = false;
    public static double ElbowTargetAngle = 0;
    //ticks = angle * rotations/rad * ticks/rotation * outersprocket/innersprocket * gearboxratio
    public static double AngleToTickShoulder = (1/(2*Math.PI))*42*(72/22)*30;
    public static double AngleToTickElbow = (1/(2*Math.PI))*2048*(22/18)*100;
    public static SetPoints armSetPoint = SetPoints.Home;
  }
}
