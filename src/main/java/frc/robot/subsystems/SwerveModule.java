// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Swerve module class one for each swerve module. 
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
 *  
 * THIS CODE ONLY WORKS IF YOU SET THE CANCODERS MAGNETIC OFFSET TO BOOT TO ABSOLUTE POSITION
 * 
 * OTHERWISE THE WHEELS WILL NOT INITIALIZE IN THE CORRECT POSITIONS
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
*/
public class SwerveModule extends SubsystemBase implements Sendable
{
    private SwerveModuleConfig cfg;
    private SwerveModuleIDConfig idcfg;
    private TalonFX steerMotor, driveMotor;
    private CANcoder steerEncoder;
    public Translation2d position;
    public VelocityVoltage driveVelocityVoltage;
    public PositionVoltage steerPositionVoltage;
    private double targetSteerRotations = 0.0;
    private double targetDriveVelocity = 0.0;
    private double targetDriveVelocityRotations = 0.0;
    private double steerVelocity;
    private final String kCANbus = "rio";


    /** Constructs a swerve module class. Initializes drive and steer motors
     * 
     * @param cfg swerve module configuration values for this module
     * @param ids Can Ids for this module
     */
    public SwerveModule(SwerveModuleConfig cfg, SwerveModuleIDConfig ids)
    {
        this.position = cfg.position;
        this.cfg = cfg;
        this.idcfg = ids;

        setName(String.format("SwerveModule[%d]", cfg.moduleNumber));

        steerMotor = new TalonFX(ids.steerMotorID, kCANbus);
        driveMotor = new TalonFX(ids.driveMotorID, kCANbus);
        steerEncoder = new CANcoder(ids.steerEncoderID, kCANbus);
    
        driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);
        steerPositionVoltage = new PositionVoltage(0).withSlot(0);
        configureHardware();
    }

    // Sample a SwerveModulePosition object from the state of this module.
    public void samplePosition(SwerveModulePosition position)
    {
        position.angle = Rotation2d.fromRotations(getSteerRotations());
        position.distanceMeters = getDrivePosition();
    }

    // Return steering sensor angle in rotations. 0 = dead ahead on robot.
    public double getSteerRotations()
    {
        steerEncoder.getAbsolutePosition().refresh();
        return ((steerEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    // Return drive position in meters.
    public double getDrivePosition()
    {   //*NOTE */ Set Alpha to 1 if you are recalibrating
        
        //double alpha = 4.87 / 5.356; (old)
        //double alpha = 1/524 / 1.6296 (new)
        double alpha = 0.95598 * 0.9352;
        return alpha * (driveMotor.getRotorPosition().getValueAsDouble() / cfg.rotationsPerMeter);
    }

    // Return drive velocity in meters/second.
    public double getDriveVelocity()
    { 
        return driveMotor.getRotorVelocity().getValueAsDouble() / (cfg.rotationsPerMeter);
    }
    
    public double getTargetSteerRotations() {
        return targetSteerRotations;
    }

    public double getTargetDriveVelocity()  {
        return targetDriveVelocity;
    }

    //debug only
    public double getTargetDriveVelocityRotations(){
        return targetDriveVelocityRotations;
    }

    //debug only
    public double getDriveVelocityRotations(){
        return getDriveVelocity() * cfg.rotationsPerMeter;
    }

    //debug only
    public double getVelocityError(){
        return Math.abs(getTargetDriveVelocity()) - Math.abs(getDriveVelocity());
    }

    //*Wrapping code from sds example swerve library
    public void setCommand(double steerRotations, double driveVelocity){
        targetSteerRotations = steerRotations;
        targetDriveVelocity = driveVelocity;

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double steerMotorError = steerRotations - getSteerRotations();
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }
        driveVelocity *= cosineScalar;

        // /* Back out the expected shimmy the drive motor will see */
        // /* Find the angular rate to determine what to back out */
        // double azimuthTurnRps = m_steerVelocity.getValue();
        // /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        // double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;
        // velocityToSet -= driveRateBackOut;
        
        //alpha is a multilier based on the speed of steer velocity to offset the coupling in the wheel speed when turning
        double alpha = 0.25;
        // //multiply the alpha and steer velocity and then add it to drive velocity in order to offset
        double driveOffset = getSteerVelocity() * alpha;
        driveVelocity += driveOffset;

        setDriveVelocity(driveVelocity);
        setSteerRotations(steerRotations);
    }

    // Sets the velocity for the drive motor(s in meters per second.
    public void setDriveVelocity(double driveVelocity)
    {
        //line below is Debug only
        targetDriveVelocityRotations = driveVelocity * cfg.rotationsPerMeter;
        driveMotor.setControl(driveVelocityVoltage.withVelocity((driveVelocity * cfg.rotationsPerMeter)));
    }

    // setSteerAngle in radians
    public void setSteerRotations(double steerRotations)
    {
        steerMotor.setControl(steerPositionVoltage.withPosition(steerRotations));
    }

    /**Sets motors in the module to brake or coast mode
     * 
     * @param brake a boolean to indicate if motors should be in brake mode or not
     */
    public void setDriveMotorBraking(boolean brake)
    {
        if(brake)
        {
            steerMotor.setNeutralMode(NeutralModeValue.Brake);
            driveMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        else
        {
            steerMotor.setNeutralMode(NeutralModeValue.Coast);
            driveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    // configures motors with PIDF values, if it is inverted or not, current limits, etc.
    public void configureHardware()
    {
        // Default configurations:
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();

        var error = steerMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.5);
        if (!error.isOK()) 
        {
            System.err.print(String.format("Module %d STEER MOTOR ERROR: %s", cfg.moduleNumber, error.toString()));
        }

        error = driveMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.5);
        if (!error.isOK()) 
        {
            System.err.println(String.format("Module %d DRIVE MOTOR ERROR: %s", cfg.moduleNumber, error.toString()));
        }


        // Set control direction of motors:
        steerMotor.setInverted(true);
        driveMotor.setInverted(false);


        // Default to brakes off:
        steerMotor.setNeutralMode(NeutralModeValue.Coast);
        driveMotor.setNeutralMode(NeutralModeValue.Coast);

        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(cfg.steerCurrentLimit));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(cfg.steerCurrentThreshold));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLowerTime(cfg.steerCurrentThresholdTime));  

        CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
        driveMotor.getConfigurator().apply(driveCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true));
        driveMotor.getConfigurator().apply(driveCurrentLimitsConfigs.withSupplyCurrentLimit(cfg.driveCurrentLimit));
        driveMotor.getConfigurator().apply(driveCurrentLimitsConfigs.withSupplyCurrentLimit(cfg.driveCurrentThreshold));
        driveMotor.getConfigurator().apply(driveCurrentLimitsConfigs.withSupplyCurrentLowerTime(cfg.driveCurrentThresholdTime));
        
        MagnetSensorConfigs mgSenseCfg = new MagnetSensorConfigs();
        // mgSenseCfg.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // mgSenseCfg.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // mgSenseCfg.MagnetOffset = cfg.steerRotationOffset; 

        error = steerEncoder.getConfigurator().refresh(mgSenseCfg, 0.5);
        if (!error.isOK()) {
            System.err.println(String.format("ERROR: SwerveModule %d steerEncoder response: %s ", cfg.moduleNumber, error.getDescription()));
        }
        // System.out.println(String.format("SwerveModule %d Magnet AbsoluteSensorRange: %s", cfg.moduleNumber, mgSenseCfg.AbsoluteSensorRange));
        // System.out.println(String.format("SwerveModule %d Magnet SensorDirection: %d",  cfg.moduleNumber, mgSenseCfg.SensorDirection));
        System.out.println(String.format("SwerveModule %d Magnet MagnetOffset: %f", cfg.moduleNumber, mgSenseCfg.MagnetOffset));

        
        steerConfigs.Feedback.FeedbackRemoteSensorID = idcfg.steerEncoderID;
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfigs.Feedback.RotorToSensorRatio = (150.0 / 7.0);
        steerConfigs.Feedback.SensorToMechanismRatio = 1.0;  // This should be used for remote CANCoder with continuous wrap.
        steerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    
        error = steerMotor.getConfigurator().apply(steerConfigs);
        if (!error.isOK())
        {
            System.err.println(String.format("SwerveModule %d configSelectedFeedbackSensor failed: %s ", cfg.moduleNumber, error.getDescription()));
        }

        driveConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.getConfigurator().apply(driveConfigs);
        driveMotor.setPosition(0);

        // PID Loop settings for steering position control:
        var steerMotorClosedLoopConfig = new Slot0Configs();
        steerMotorClosedLoopConfig.withKP(cfg.steerP); 
        steerMotorClosedLoopConfig.withKI(cfg.steerI); 
        steerMotorClosedLoopConfig.withKD(cfg.steerD); 
        steerMotorClosedLoopConfig.withKV(cfg.steerV); 
        error = steerMotor.getConfigurator().apply(steerMotorClosedLoopConfig, 0.5);
        if (!error.isOK()) {
            System.err.println(String.format("SwerveModule %d Steer Motor Configuration Error: %s", cfg.moduleNumber, error.getDescription()));
        }
        
        // PID Loop settings for drive velocity control:
        var driveMotorClosedLoopConfig = new Slot0Configs();
        driveMotorClosedLoopConfig.withKP(cfg.driveP);
        driveMotorClosedLoopConfig.withKI(cfg.driveI);
        driveMotorClosedLoopConfig.withKD(cfg.driveD);
        driveMotorClosedLoopConfig.withKV(cfg.driveV);
        driveMotorClosedLoopConfig.withKA(cfg.driveA);

        error = driveMotor.getConfigurator().apply(driveMotorClosedLoopConfig, 0.5);
        if (error.isOK()) {
            System.err.println(String.format("SwerveModule %d Drive Motor Configuration Error: %s", cfg.moduleNumber, error.getDescription()));
        }

        System.out.println(String.format("SwerveModule %d configured.", cfg.moduleNumber));
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // Removed for elims:
        // builder.setSmartDashboardType(String.format("SwerveModule[%d]", cfg.moduleNumber));
        // builder.addDoubleProperty(String.format("Target Steer R %d", cfg.moduleNumber), this::getTargetSteerRotations, null);
        // builder.addDoubleProperty(String.format("Target Drive V %d", cfg.moduleNumber), this::getTargetDriveVelocity, null);
        // builder.addDoubleProperty(String.format("Steer R %d", cfg.moduleNumber), this::getSteerRotations, null);
        // builder.addDoubleProperty(String.format("Drive V %d", cfg.moduleNumber), this::getDriveVelocity, null);
        // builder.addDoubleProperty(String.format("Drive Position %d", cfg.moduleNumber), this::getDrivePosition, null);
        // builder.addDoubleProperty(String.format("Target Drive V Rotations %d", cfg.moduleNumber), this::getTargetDriveVelocityRotations, null);
        // builder.addDoubleProperty(String.format("Drive V Rotations %d", cfg.moduleNumber), this::getDriveVelocityRotations, null);
        // builder.addDoubleProperty(String.format("Drive V Error %d", cfg.moduleNumber), this::getVelocityError, null);
    //   steerEncoder.initSendable(builder);
    //   steerMotor.initSendable(builder);
    //   driveMotor.initSendable(builder);
    }

    /**
     * Gets the name of this Subsystem.
     *
     * @return Name
     */
    public String getName() {
        return SendableRegistry.getName(this);
    }

    /**
     * Sets the name of this Subsystem.
     *
     * @param name name
     */
    public void setName(String name) {
        SendableRegistry.setName(this, name);
    }

    /**Sets the percent output velocity to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugTranslate(double power)
    {
        driveMotor.setControl(new DutyCycleOut(power));
    }

    /**Sets the percent output velocity of wheel angle to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugRotate(double power)
    {
        steerMotor.setControl(new DutyCycleOut(power));
    }

    public double getLoad() {
        return Math.abs(driveMotor.getTorqueCurrent(true).getValueAsDouble());
    }

    public double getSteerVelocity(){
        steerEncoder.getVelocity().refresh();
        return steerEncoder.getVelocity().getValueAsDouble();
    }
}
