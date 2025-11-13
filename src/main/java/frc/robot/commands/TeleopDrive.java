// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SwerveModule;

public class TeleopDrive extends Command 
{
  double angleTolerance = 0.05;
  ChassisSpeeds chassisSpeeds;
  Pose2d Rotation;
  Pose2d robotRotation;
  Drivetrain drivetrain;
  OI m_OI;
  private boolean fieldCentric;
  private boolean parked = false;
  ChassisSpeeds speeds;
  double last_error = 0; //for snap-to-positions derivative
  double last_time = 0; //for snap-to-positions derivative
  boolean lastParkingBreakButton = false;
  boolean lastFieldCentricButton = true;
  boolean pointAtTarget;

  PIDController snapPidProfile;

  // Teleop drive velocity scaling:
  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second
  private double mult1;
  private double mult2;
  private double leftX;
  private double leftY;
  private double rightX;
  private double vx;
  private double vy;
  private double w;
  private double allianceSign = 1; // this is handled by setting the odometry orientation for each alliance

  double frontLeftTorque;
  double frontRightTorque;
  double backLeftTorque;
  double backRightTorque;
  double avgTorque;

  double torqueGate = 65; 

  /** Creates a new Teleop. */
  public TeleopDrive(Drivetrain drivetrain, OI oi){
    this.drivetrain = drivetrain;
    m_OI = oi;
    fieldCentric = true;
    pointAtTarget = false;
    snapPidProfile = new PIDController(
      0.05, 
      0.0, 
      0.0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    System.out.println("TeleopDrive: Init");
    super.initialize();
    if (DriverStation.getAlliance().get() == Alliance.Red)
    {
      allianceSign = -1;
    }
    else
    {
      allianceSign = 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    leftY = -m_OI.getDriverTranslateY();
    leftX = -m_OI.getDriverTranslateX();
    rightX = -m_OI.getDriverRotate();

    SmartDashboard.putBoolean("Parking Brake", parked);

    if(m_OI.getDriverRightBumper() && lastFieldCentricButton == false){
      fieldCentric = !fieldCentric;
    }
    lastFieldCentricButton = m_OI.getDriverRightBumper();

    if(m_OI.getDriverLeftBumper() && lastParkingBreakButton == false)
    {
      parked = !parked;
    }
    // TODO: get parking brake to work - right now only wheel in swerve mod 0 works
    lastParkingBreakButton = m_OI.getDriverLeftBumper();
    if(parked && !drivetrain.getParkingBrake())
    {
      drivetrain.parkingBrake(true);
    }
    if(!parked && drivetrain.getParkingBrake())
    {
      drivetrain.parkingBrake(false);
    }
    else 
    {
      var dPadUp = m_OI.getDriverDPadUp(); 
      var dPadDown = m_OI.getDriverDPadDown();
      var dPadLeft = m_OI.getDriverDPadLeft();
      var dPadRight = m_OI.getDriverDPadRight();

      if(!dPadUp && !dPadDown && !dPadLeft && !dPadRight) {

      //multiples the angle by a number from 1 to the square root of 30:
        mult1 = 1.0 + (m_OI.getDriverLeftTrigger() * ((Math.sqrt(36)) - 1));
        mult2 = 1.0 + (m_OI.getDriverRightTrigger() * ((Math.sqrt(36)) - 1));

        //sets deadzones on the controller to extend to .05:
        if(Math.abs(leftY) < .15) {leftY = 0;}
        if(Math.abs(leftX) < .15) {leftX = 0;}
        if(Math.abs(rightX) < .15) {rightX = 0;}

        vx = MathUtil.clamp((allianceSign * leftY * maximumLinearVelocity / 25 ) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        vy = MathUtil.clamp((allianceSign * leftX * maximumLinearVelocity / 25 ) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
        w = MathUtil.clamp((rightX * maximumRotationVelocity / 25) * mult1 * mult2, -maximumRotationVelocity, maximumRotationVelocity);

        SmartDashboard.putNumber("TeleopDrive/vx", vx); 
          drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(vx, vy, w));
      }
      else {
        //robot centric creep
        ChassisSpeeds creepSpeeds = new ChassisSpeeds();

        if(dPadUp) {
          creepSpeeds.vxMetersPerSecond = 0.2;
        }
        if(dPadDown) {
          creepSpeeds.vxMetersPerSecond = -0.2;
        }
        if(dPadRight) {
          creepSpeeds.vyMetersPerSecond = -0.2;
        }
        if(dPadLeft) {
          creepSpeeds.vyMetersPerSecond = 0.2;
        }
        drivetrain.setTargetChassisSpeeds(creepSpeeds);

      }
      

      avgTorque = drivetrain.getAverageLoad();

      //if above the torque gate rumble the contorller
      if(avgTorque >= torqueGate) {
        m_OI.rumble();
      }
      else if(avgTorque < torqueGate) {
       m_OI.stopRumble();
      }

      SmartDashboard.putNumber("Avg Torque", avgTorque);
    }

    
    // Allow driver to zero the drive subsystem heading for field-centric control.
    // if(m_OI.getDriverViewButton())
    // {
    //   drivetrain.zeroHeading();
    // }


    //TODO: we should test resetting odometry to see if it works
    // if(m_OI.getDriverMenuButton()){
    //   Rotation2d zeroRotate = new Rotation2d();
    //   Pose2d zero = new Pose2d(0.0, 0.0, zeroRotate);
    //   drivetrain.resetOdometry(zero);
    //   localizer.resetOrientation();
    // }


    SmartDashboard.putBoolean("Field Centric ", fieldCentric);

    super.execute();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    if (interrupted) {
      System.out.println("TeleopDrive: Interrupted!");
    }
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
