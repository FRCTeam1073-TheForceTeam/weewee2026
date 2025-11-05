// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.PubSubOption;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase
{
  private final String kCANbus = "CANivore";
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private SwerveModule[] modules;
  private SwerveModulePosition[] modulePositions;
  private ChassisSpeeds targetChassisSpeeds;
  private Pigeon2 pigeon2;
  private boolean debug = false;

  private double maximumLinearSpeed = 1.0;
  private boolean parkingBrakeOn = false;

  /** Creates a new DriveSubsystem. */
  public Drivetrain()
  {
    super.setSubsystem("Drivetrain");

    pigeon2 = new Pigeon2(13, kCANbus);
    var error = pigeon2.getConfigurator().apply(new Pigeon2Configuration());
    if (!error.isOK()) 
    {
      System.out.println(String.format("Drivetrain: PIGEON IMU ERROR: %s", error.toString()));
    }
    error = pigeon2.setYaw(0);

    // Make space for four swerve modules:
    modules = new SwerveModule[4];
    modulePositions = new SwerveModulePosition[4];

    //front left
    SwerveModuleIDConfig moduleIDConfig = new SwerveModuleIDConfig(5, 6, 1);

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 0;
    moduleConfig.position = new Translation2d(0.289, 0.289);

    modules[0] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[0] = new SwerveModulePosition();

    //front right
    moduleIDConfig = new SwerveModuleIDConfig(7, 8, 2);

    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 1;
    moduleConfig.position = new Translation2d(0.289, -0.289);

    modules[1] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[1] = new SwerveModulePosition();

    //back left
    moduleIDConfig = new SwerveModuleIDConfig(9, 10, 3);

    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 2;
    moduleConfig.position = new Translation2d(-0.289, 0.289);

    modules[2] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[2] = new SwerveModulePosition();

    //back right
    moduleIDConfig = new SwerveModuleIDConfig(11, 12, 4);
    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 3;
    moduleConfig.position = new Translation2d(-0.289, -0.289);

    modules[3] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[3] = new SwerveModulePosition();

    // Create our kinematics class
    kinematics = new SwerveDriveKinematics(
      modules[0].position,
      modules[1].position,
      modules[2].position,
      modules[3].position
    );

    // Create odometry:
    modules[0].samplePosition(modulePositions[0]);
    modules[1].samplePosition(modulePositions[1]);
    modules[2].samplePosition(modulePositions[2]);
    modules[3].samplePosition(modulePositions[3]);
    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getGyroHeadingDegrees()), modulePositions, new Pose2d(0,0,new Rotation2d(0)));

    // Configure maximum linear speed for limiting:
    maximumLinearSpeed = 3.5;
    // Initial chassis speeds are zero:
    targetChassisSpeeds = new ChassisSpeeds(0,0,0);


    // Add each module as a child for debugging:
    // for (int mod = 0; mod < 4; ++mod) {
    //   addChild(String.format("Module[%d]", mod), modules[mod]);
    // }
  }

  // Returns target x velocity (for sendable)
  double getTargetVx() {
    return targetChassisSpeeds.vxMetersPerSecond;
  }

  // Returns target y velocity (for sendable)
  double getTargetVy() {
    return targetChassisSpeeds.vyMetersPerSecond;
  }

  // Returns target angular velocity (for sendable)
  double getTargetOmega() {
    return targetChassisSpeeds.omegaRadiansPerSecond;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    //TODO: take out sendable stuff from the code
    // builder.setSmartDashboardType("Drivetrain");
    builder.addBooleanProperty("ParkingBrake", this::getParkingBrake, null);
    builder.addDoubleProperty("Odo X", this::getOdometryX, null);
    builder.addDoubleProperty("Odo Y", this::getOdometryY, null);
    builder.addDoubleProperty("Odo Theta(RAD)", this::getOdometryThetaRadians, null);
    builder.addDoubleProperty("Odo Gyro Heading(DEG)", this::getGyroHeadingDegrees, null);
    builder.addDoubleProperty("Odo Gyro Wrapped Heading", this::getWrappedGyroHeadingDegrees, null);
    builder.addDoubleProperty("Target Vx", this::getTargetVx, null);
    builder.addDoubleProperty("Target Vy", this::getTargetVy, null);
    builder.addDoubleProperty("Target Omega", this::getTargetOmega, null);
    builder.addDoubleProperty("Pitch", this::getPitch, null);
    builder.addDoubleProperty("Roll", this::getRoll, null);
  }

  // @Override
  // public boolean updateDiagnostics() {
  //   String result = new String();
  //   boolean isOK = true;

  //   // Run the diagnostics for each ofthe modules and return value if something is wrong:
  //   for (int mod = 0; mod < 4; ++mod) {
  //     if (!modules[mod].updateDiagnostics())
  //       return setDiagnosticsFeedback(modules[mod].getDiagnosticsDetails(), false);
  //   }

  //   StatusCode error = pigeon2.clearStickyFaults(0.5);
  //   if (error != StatusCode.OK) {
  //      return setDiagnosticsFeedback("Pigeon 2 Diagnostics Error", false);
  //   }

  //   return setDiagnosticsFeedback(result, isOK);
  // }

  public void setDebugMode(boolean debug) 
  {
    this.debug = debug;
  }

  //Returns IMU heading in degrees
  public double getGyroHeadingDegrees() 
  {
    return pigeon2.getYaw().refresh().getValueAsDouble();
  }

  public double getGyroHeadingRadians()
  {
    return getGyroHeadingDegrees() * Math.PI / 180.0;
  }

  // Wraps the heading in degrees:
  public double getWrappedGyroHeadingDegrees()
  {
    return MathUtils.wrapAngleDegrees(getGyroHeadingDegrees());
  }

  public double getWrappedGyroHeadingRadians()
  {
    return MathUtils.wrapAngleRadians(getGyroHeadingDegrees() * Math.PI / 180);
  }

  public double getPitch()
  {
    return pigeon2.getPitch().getValueAsDouble();
  }

  public double getRoll()
  {
    return pigeon2.getRoll().getValueAsDouble();
  }

  // Reset IMU heading to zero degrees
  public void zeroHeading() 
  {
    // TODO:Change value to whatever value you need it to be
    pigeon2.setYaw(0);
  }

  // Set the commanded chassis speeds for the drive subsystem.
  public void setTargetChassisSpeeds(ChassisSpeeds speeds)
  {
    targetChassisSpeeds = speeds;
  }

  // Return the measured chassis speeds for the drive subsystem.
  public ChassisSpeeds getChassisSpeeds()
  {
    SwerveModuleState[] wheelStates = new SwerveModuleState[4];
    wheelStates[0] = new SwerveModuleState();
    wheelStates[1] = new SwerveModuleState();
    wheelStates[2] = new SwerveModuleState();
    wheelStates[3] = new SwerveModuleState();

    wheelStates[0].speedMetersPerSecond = modules[0].getDriveVelocity();
    wheelStates[1].speedMetersPerSecond = modules[1].getDriveVelocity();
    wheelStates[2].speedMetersPerSecond = modules[2].getDriveVelocity();
    wheelStates[3].speedMetersPerSecond = modules[3].getDriveVelocity();

    wheelStates[0].angle = Rotation2d.fromRotations(modules[0].getSteerRotations());
    wheelStates[1].angle = Rotation2d.fromRotations(modules[1].getSteerRotations());
    wheelStates[2].angle = Rotation2d.fromRotations(modules[2].getSteerRotations());
    wheelStates[3].angle = Rotation2d.fromRotations(modules[3].getSteerRotations());

    return kinematics.toChassisSpeeds(wheelStates);
  }

  // puts the motors in brake mode
  public void setBrakes(boolean brakeOn)
  {
    modules[0].setDriveMotorBraking(brakeOn);
    modules[1].setDriveMotorBraking(brakeOn);
    modules[2].setDriveMotorBraking(brakeOn);
    modules[3].setDriveMotorBraking(brakeOn);
  }

  // returns the wheel positions
  public SwerveDriveKinematics getKinematics()
  {
    return kinematics;
  }

  public void updateOdometry() 
  {
    modules[0].samplePosition(modulePositions[0]);
    modules[1].samplePosition(modulePositions[1]);
    modules[2].samplePosition(modulePositions[2]);
    modules[3].samplePosition(modulePositions[3]);
    
    odometry.update(Rotation2d.fromDegrees(getGyroHeadingDegrees()), modulePositions);
  }

  public SwerveModulePosition[] getSwerveModulePositions()
  {
    return modulePositions;
  }

  public void resetOdometry(Pose2d where)
  {
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroHeadingDegrees()), modulePositions, where);
  }

  public Pose2d getOdometry()
  {
    return new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), Rotation2d.fromRadians(getOdometryThetaRadians()));
  }

  public double getOdometryX(){
    return odometry.getPoseMeters().getX();
  }

  public double getOdometryY(){
    return odometry.getPoseMeters().getY();
  }

  public double getOdometryThetaRadians()
  {
    return odometry.getPoseMeters().getRotation().getRadians();
  }

  public Pose3d get3dOdometry()
  {
    // return odometry position as a pose 3d
    Pose2d odo = getOdometry();
    // TODO: use internal roll and pitch methods later
    return new Pose3d(odo.getX(), odo.getY(), 0.0, new Rotation3d(getRoll(), getPitch(), getOdometryThetaRadians()));
  }
  
  @Override
  public void periodic()
  {

    if (!debug && !parkingBrakeOn) //disables motors when parking brakes are active
    {
      // This method will be called once per scheduler run
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetChassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, maximumLinearSpeed);

      states[0] = SwerveModuleState.optimize(states[0], Rotation2d.fromRotations(modules[0].getSteerRotations()));
      states[1] = SwerveModuleState.optimize(states[1], Rotation2d.fromRotations(modules[1].getSteerRotations()));
      states[2] = SwerveModuleState.optimize(states[2], Rotation2d.fromRotations(modules[2].getSteerRotations()));
      states[3] = SwerveModuleState.optimize(states[3], Rotation2d.fromRotations(modules[3].getSteerRotations()));

      modules[0].setCommand(states[0].angle.getRotations(), states[0].speedMetersPerSecond);
      modules[1].setCommand(states[1].angle.getRotations(), states[1].speedMetersPerSecond);
      modules[2].setCommand(states[2].angle.getRotations(), states[2].speedMetersPerSecond);
      modules[3].setCommand(states[3].angle.getRotations(), states[3].speedMetersPerSecond);
    }
   
    updateOdometry();
  }

  // rotates all the wheels to be facing inwards and stops the motors to hold position
  public void parkingBrake(boolean parkingBrakeOn) 
  {
    this.parkingBrakeOn = parkingBrakeOn;
    if (parkingBrakeOn)
    {
      targetChassisSpeeds.vxMetersPerSecond = 0.0;
      targetChassisSpeeds.vyMetersPerSecond = 0.0;
      targetChassisSpeeds.omegaRadiansPerSecond = 0.0;
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetChassisSpeeds);

      // Set angles for locked parking position:
      states[0].angle = Rotation2d.fromRadians(Math.PI/4.0);
      states[1].angle = Rotation2d.fromRadians(-Math.PI/4.0);
      states[2].angle = Rotation2d.fromRadians(-Math.PI/4.0);
      states[3].angle = Rotation2d.fromRadians(Math.PI/4.0);


      // Run the optimizer for the states:
      states[0] = SwerveModuleState.optimize(states[0], Rotation2d.fromRotations(modules[0].getSteerRotations()));
      states[1] = SwerveModuleState.optimize(states[1], Rotation2d.fromRotations(modules[1].getSteerRotations()));
      states[2] = SwerveModuleState.optimize(states[2], Rotation2d.fromRotations(modules[2].getSteerRotations()));
      states[3] = SwerveModuleState.optimize(states[3], Rotation2d.fromRotations(modules[3].getSteerRotations()));

      
      modules[0].setCommand(states[0].angle.getRotations(), states[0].speedMetersPerSecond);
      modules[1].setCommand(states[1].angle.getRotations(), states[1].speedMetersPerSecond);
      modules[2].setCommand(states[2].angle.getRotations(), states[2].speedMetersPerSecond);
      modules[3].setCommand(states[3].angle.getRotations(), states[3].speedMetersPerSecond);
    }
  }

  public boolean getParkingBrake()
  {
    return parkingBrakeOn;
  }

  public void setDebugSpeed(double speed) // sets the speed directly
  {
    modules[0].setDriveVelocity(speed);
    modules[1].setDriveVelocity(speed);
    modules[2].setDriveVelocity(speed);
    modules[3].setDriveVelocity(speed);
  }

  public void setDebugAngle(double power) // sets the angle directly
  {
    modules[0].setDebugRotate(power);
    modules[1].setDebugRotate(power);
    modules[2].setDebugRotate(power);
    modules[3].setDebugRotate(power);
  }

  public void setDebugDrivePower(double power) // sets the power directly
  {
    modules[0].setDebugTranslate(power);
    modules[1].setDebugTranslate(power);
    modules[2].setDebugTranslate(power);
    modules[3].setDebugTranslate(power);
  }

  public SwerveModule[] getModules() {
    return modules;
  }

  public double getAverageLoad() {
    return (modules[0].getLoad() + modules[1].getLoad() + modules[2].getLoad() + modules[3].getLoad()) / 4;
  }

}
