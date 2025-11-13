// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;


/** Add your docs here. */
public class SwerveModuleConfig 
{
    public int moduleNumber = -1;
    public double gearRatio = 6.75;
    public double wheelDiameterMeters = 0.1016;
    public Translation2d position = new Translation2d(0,0);
    //public double tickPerMeter = 1000;
    public double rotationsPerMeter = gearRatio / (wheelDiameterMeters * Math.PI * 1.06); // 1.06 is the measure correction factor while driving
    public double radiansPerRotation = (150.0 / 7.0) / (Math.PI * 2);
    //public double tickPerRadian = 1000;
    //public double steerRotationOffset = 0;
    public double steerCurrentLimit = 20;
    public double driveCurrentLimit = 35;
    public double steerCurrentThreshold = 12;
    public double driveCurrentThreshold = 22;
    public double steerCurrentThresholdTime = 0.1;
    public double driveCurrentThresholdTime = 0.25;
    public double steerP = 0;
    public double steerI = 0;
    public double steerD = 0;
    public double steerV = 0;
    public double driveP = 0;
    public double driveI = 0;
    public double driveD = 0;
    public double driveV = 0;
    public double driveA = 0;
    public double driveMaxIntegrator = 400.0;
    public double steerMaxIntegrator = 400.0;    


    /**SwerveModuleConfig contstructor sets PIDF values and current limits
     * 
     */
    public SwerveModuleConfig()
    {
        driveP = 0.3; 
        driveI = 0.0;
        driveD = 0.001;
        driveV = 0.12;
        driveA = 0.05;
        driveMaxIntegrator = 400.0;
        driveCurrentLimit = 30;
        driveCurrentThreshold = 35;


        //tickPerRadian = Preferences.getDouble("Drive.Steer.TicksPerRadian", 4096.0/(2*Math.PI)); // 4,096 ticks per rotation, converted to radians
        steerP = 30.0; 
        steerI = 4.0;
        steerD = 1.0;
        steerV = 0.12;
        steerMaxIntegrator = 400.0;
        steerCurrentLimit = 12;
        steerCurrentThreshold = 15;
    }
}