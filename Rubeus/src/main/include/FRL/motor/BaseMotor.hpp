#pragma once
/* By Tyler Clarke. Base class for polymorphing highly compatible motors */


/**
 @author Tyler Clarke and Luke White
 @version 1.0
 
 * Base class for polymorphic motors.

 * All virtual functions are pure; meant to be implemented by code like {@link SparkMotor}.

 * Some "normal" functions call the virtual functions. These should never be overridden!
 */
class BaseMotor {
public:
    bool inversionState = false;
    /**
     * Set the motor to a percentage value. Range (-1, 1) of max power.
     @param percent The percentage of total power to run at.
     */
    virtual void SetPercent(double percent) = 0;
    /**
     * Set the invertion state of the motor
     */
    void SetInverted(bool invert){
        inversionState = invert;
        _setInverted(invert);
    }

    void SetInverted(){
        inversionState = !inversionState;
        _setInverted(inversionState);
    }
    /**
     * INTERNAL: This actually sends the relevant commands to the motor
     @param invert Whether or not to invert
     */
    virtual void _setInverted(bool invert) = 0;
    /**
     * Set the PIDF "P" coefficient. Manufacturer software differences mean that this shouldn't be used! Instead, use #PIDController.
     @param kP The P coefficient.
     */
    virtual void SetP(double kP) = 0;
    /**
     * Set the PIDF "I" coefficient. Manufacturer software differences mean that this shouldn't be used! Instead, use #PIDController.
     @param kI The I coefficient.
     */
    virtual void SetI(double kI) = 0;
    /**
     * Set the PIDF "D" coefficient. Manufacturer software differences mean that this shouldn't be used! Instead, use #PIDController.
     @param kD The D coefficient
     */
    virtual void SetD(double kD) = 0;
    /**
     * Set the PIDF "F" coefficient. Manufacturer software differences mean that this shouldn't be used! Instead, use #PIDController.
     @param kF The F coefficient
     */
    virtual void SetF(double kF) = 0;
    /**
     * Set the PIDF output range. Manufacturer software differences mean that this shouldn't be used! Instead, use #PIDController.
     @param kPeakOF    The maximum forward output
     @param kPeakOR    The maximum reverse output
     @param kNominalOF The "nominal" forward output
     @param kNominalOR the "nominal" reverse output
     */
    virtual void SetOutputRange(double kPeakOF, double kPeakOR, double kNominalOF, double kNominalOR) = 0;
    /**
     * Return the encoder position of the motor.
     */
    virtual double GetPosition() = 0;
    /**
     * Return the encoder velocity of the motor.
     */
    virtual double GetVelocity() = 0;
    /**
     * Set the position of the motor using PID. Manufacturer software differences mean that this shouldn't be used! Instead, use #PIDController.
     @param position The position to move to
     */
    virtual void SetPositionPID(double position) = 0;
    /**
     * Set the speed of the motor using PID. Manufacturer software differences mean that this shouldn't be used! Instead, use #PIDController.
     @param speed The speed to ramp to
     */
    virtual void SetSpeedPID(double speed) = 0;
    /**
     * Return whether the motor is at the 0 state. Don't override this function in subclasses!
     */
    bool IsAtZero() {
        return GetPosition() == 0;
    }

    virtual void ConfigIdleToBrake() = 0;

    virtual double GetCurrent() = 0;
};
