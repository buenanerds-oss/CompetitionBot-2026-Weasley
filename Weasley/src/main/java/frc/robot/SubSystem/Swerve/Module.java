package frc.robot.SubSystem.Swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotMap;
import frc.robot.SubSystem.Logging.GroupLogger;
import frc.robot.SubSystem.Logging.NerdLog;
import frc.robot.Util.BasicUtil;


public class Module extends ModuleMotorConfig implements ModuleIO {

    
    boolean EmergencyStop = false;
    double turnAmps;
    double turnVolts;
    double driveAmps;
    double driveVolts;
    SwerveModuleState currentSwerveState;
    SwerveModulePosition currentswervePosition;
    SwerveModuleState desiredState;

    double turnLocation;
    int index;
    SparkMax turnMotor;
    SparkMax driveMotor;
    PIDController drivePID;
    PIDController turnPID;
    AnalogEncoder AbsEncoder;
    SparkClosedLoopController turnSparkPID;
    SparkClosedLoopController driveSparkPID;
    boolean atSetpoint = false;

    /**
     * one indivual SwerveModule, 4 of these makes the swerve drive. starts count @ 0
     * @param index - which module is this?
     * @param turnMotor - turn encoder
     * @param driveMotor - drive encoder
     */
    public Module(int index, SparkMax turnMotor, SparkMax driveMotor) {
        this.index = index;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.AbsEncoder = RobotMap.swerveAbsEncoders[index];


        //from the ModuleMotorConfig Class
        configureTurnMotor(turnMotor);
        configureDriveMotor(driveMotor);

        this.turnSparkPID = turnMotor.getClosedLoopController();
        this.driveSparkPID = driveMotor.getClosedLoopController();
        this.currentSwerveState = new SwerveModuleState(driveMotor.getEncoder().getVelocity() * SwerveConstants.wheelRadiusMeters, //SpeedRadPS * RadiusMeters = velocityMetersPerSecond
         new Rotation2d(turnMotor.getEncoder().getPosition()));
        this.currentswervePosition = new SwerveModulePosition(driveMotor.getEncoder().getPosition() * SwerveConstants.wheelRadiusMeters, new Rotation2d(turnMotor.getEncoder().getPosition()));
       

        //prevents initialization issues
        turnAmps = 0;
        turnVolts = 0;
        driveAmps = 0; 
        driveVolts = 0;
        desiredState = new SwerveModuleState();

        //PID initialization:
        drivePID = new PIDController(ModuleMotorConfig.DRIVE_P, ModuleMotorConfig.DRIVE_I, ModuleMotorConfig.DRIVE_D);
        turnPID = new PIDController(ModuleMotorConfig.TURN_P, ModuleMotorConfig.TURN_I, ModuleMotorConfig.TURN_D);
        //turnFeedforward = new SimpleMotorFeedforward(,);

        turnPID.enableContinuousInput(0, 2* Math.PI);
        turnPID.setTolerance(0.2);
        
    }
    @Override
    public void setDesiredSwerveState(SwerveModuleState desiredState) {
        desiredState.optimize(new Rotation2d(currentSwerveState.angle.getRadians()));//currentSwerveState.angle); // modules doesn;t rotate more than 180DEG most of the time
        //desiredState.cosineScale(new Rotation2d(currentSwerveState.angle.getRadians()));//currentSwerveState.angle); // Smoother driving
        this.desiredState = desiredState;

        //Force Stops everything if we're asking for more resources than what we are supposed to
        if (turnMotor.getOutputCurrent() >= SwerveConstants.turnMaxAmps || driveMotor.getOutputCurrent() >= SwerveConstants.driveMaxAmps || // current
         turnMotor.getAppliedOutput() * turnMotor.getBusVoltage() >= SwerveConstants.turnMaxVolts || driveMotor.getAppliedOutput() * driveMotor.getBusVoltage() >= SwerveConstants.driveMaxVolts//voltage
        ) EmergencyStop = true;

        //turning
        //if we are not at the position AND we aren't in emergencyStop, keep running PID.
        //if (!BasicUtil.numIsInBallparkOf(currentSwerveState.angle.getRadians(), desiredState.angle.getRadians(), SwerveConstants.turnAccuracyToleranceRAD)) {
           turnPID.setSetpoint(desiredState.angle.getRadians());
           turnMotor.setVoltage(turnPID.calculate(AbsEncoder.get()));
           //turnSparkPID.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition);
          // atSetpoint = false;
       // }
        /*else {
            turnMotor.setVoltage(0);
            turnPID.reset();
            atSetpoint = true;
        } */
            //turnMotor.setVoltage(0);} // if we don't need it to move, stop giving it the voltage to move

        desiredState.speedMetersPerSecond *= Math.cos(turnPID.getError()); 
        //drive
        //the equation in the getter of the velocity converts from RPM to RADPM to MPS
      
        //driveSparkPID.setSetpoint(desiredState.speedMetersPerSecond/ SwerveConstants.wheelRadiusMeters, ControlType.kVelocity);
        driveMotor.setVoltage(-desiredState.speedMetersPerSecond*3); // was *5
        /*
        if (turnPID.atSetpoint()) { // only drive if at setpoint
        driveMotor.setVoltage(-desiredState.speedMetersPerSecond*3); // was *5
        GroupLogger.LogBooleanGroup("modules at set", true, index, 4);
        }
        else  {
            GroupLogger.LogBooleanGroup("modules at set", false, index, 4);
            driveMotor.setVoltage(0.00);
           } */

        //record changes:
        turnAmps = turnMotor.getOutputCurrent();
            turnVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
            driveAmps =  driveMotor.getOutputCurrent();
            driveVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        currentSwerveState = new SwerveModuleState(driveMotor.getEncoder().getVelocity() * SwerveConstants.wheelRadiusMeters, 
        new Rotation2d(AbsEncoder.get()));
        currentswervePosition = new SwerveModulePosition(driveMotor.getEncoder().getPosition() * SwerveConstants.wheelRadiusMeters, new Rotation2d(AbsEncoder.get()));
        
    }

    @Override
    public void forceSetVoltage(double turnVolts, double driveVolts) {
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);
    }

    @Override
    public void changeModuleTurnPID(String valueToChange, double IncrementAmount) {
        forceSetVoltage(0, 0);
        changeTurnPID(valueToChange, IncrementAmount);
        turnPID = new PIDController(ModuleMotorConfig.TURN_P,
         ModuleMotorConfig.TURN_I,
         ModuleMotorConfig.TURN_D);
    }

    @Override
    public void changeModuleDrivePID(String valueToChange, double IncrementAmount) {
        forceSetVoltage(0, 0);
        changeDrivePID(valueToChange, IncrementAmount);
        drivePID = new PIDController(ModuleMotorConfig.DRIVE_P,
         ModuleMotorConfig.DRIVE_I,
         ModuleMotorConfig.DRIVE_D);
    }

    @Override
    public void periodic() {
        GroupLogger.logStructGroup("Swerve Module States", currentSwerveState, SwerveModuleState.struct, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn Amps", turnAmps, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn volts", turnVolts, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module drive Amps", driveAmps, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module drive Volts", driveVolts, index, 4);
        GroupLogger.LogBooleanGroup("Emergency Stopped?", EmergencyStop, index, 4);
        GroupLogger.logStructGroup("Swerve Module States", currentSwerveState, SwerveModuleState.struct, index, 4);
        GroupLogger.logStructGroup("swerve module Positions", currentswervePosition, SwerveModulePosition.struct, index, 4);
        GroupLogger.LogBooleanGroup("Module Turn @ setpoint", turnPID.atSetpoint(), index, 4);
        GroupLogger.logStructGroup("desired state", desiredState, SwerveModuleState.struct, index, 4);
        if (RobotState.isDisabled()) EmergencyStop = false;
        GroupLogger.logDoubleGroup("Turn Error", turnPID.getError(), index, 4);

        
    }

    @Override
    public SwerveModulePosition getmodulePosition() {
        return currentswervePosition;
         
    }
    


    

    

}
