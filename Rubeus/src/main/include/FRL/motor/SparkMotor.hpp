/* Spark MAX motor; can be used as a BaseMotor. */


#include <rev/CANSparkMax.h> /* Requires REVLib */
#include "BaseMotor.hpp"


struct _SparkMotorEncoderControlContainer{
    rev::SparkMaxRelativeEncoder encoder;
    rev::SparkMaxPIDController pid;
};


/**
 @author Tyler Clarke and Luke White
 @version 1.0

 * Motor wrapper for Spark Max.
 */
class SparkMotor : public BaseMotor {
public:
    _SparkMotorEncoderControlContainer* controls;
    rev::CANSparkMax* spark;

    /**
     * Construct a spark motor. Creates a new rev::CANSparkMax.
     @param canID The CAN id of the Spark Max it's wrapping.
     */
    SparkMotor(int canID){
        spark = new rev::CANSparkMax (canID, rev::CANSparkMax::MotorType::kBrushless);
        controls = new _SparkMotorEncoderControlContainer {spark -> GetEncoder(), spark -> GetPIDController()};
    }

    void _setInverted(bool invert) {
        spark -> SetInverted(invert);
    }
    
    void SetPercent(double percent){
        spark -> Set(percent);
    }

    void SetP(double kP){
        controls -> pid.SetP(kP);
    }

    void SetI(double kI){
        controls -> pid.SetI(kI);
    }

    void SetD(double kD){
        controls -> pid.SetD(kD);
    }

    void SetF(double kF){
        controls -> pid.SetFF(kF);
    }

    void SetOutputRange(double kPeakOF, double kPeakOR, double kNominalOF = 0, double kNominalOR = 0){
        controls -> pid.SetOutputRange(kPeakOR, kPeakOF); // Ignore nominal values; this is Spark.
    }
        
    double GetPosition() {
        return controls -> encoder.GetPosition();
    }
    
    double GetVelocity() {
        return controls -> encoder.GetVelocity();
    }
        
    void SetPositionPID(double position){
        controls -> pid.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
    }

    void SetSpeedPID(double speed){
        controls -> pid.SetReference(speed, rev::CANSparkMax::ControlType::kVelocity);
    }

    void ConfigIdleToBrake() {
        spark -> SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }

    double GetCurrent() {
        return spark -> GetOutputCurrent();
    }
};
