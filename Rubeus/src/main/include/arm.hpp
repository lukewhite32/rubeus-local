#include <FRL/motor/BaseMotor.hpp>
#include <frc/AnalogInput.h>
#include <FRL/motor/PIDController.hpp>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <FRL/motor/CurrentWatcher.hpp>

const double shoulderBarLengthCM = 91.44; // Length of the forearm in centimeters
const double elbowBarLengthCM = 91.44; // What can I call it? Anti-forearm? Arm? Elbow-y bit? Yeah. Elbow-y bit. This is the elbow-y bit length in centimeters.

const double shoulderDefaultAngle = 80; // I calculated. At displacement x 5, displacement y is 30. So it's atan(30/5). Which is about 80.5 degrees.
const double elbowDefaultAngle = 280; // Reflect the angle of the shoulder about the x axis
// TODO: use sane units (radians).

enum GrabMode {
    OFF,
    INTAKE,
    BARF
};


struct ArmPosition {
    float x;
    float y;
};

const vector lowPole { 130, 76.36 };
const vector highPole { 130, 106.84 };
const vector home { 35, 0 };

struct ArmInfo {
/*
    Arm math structure, to clean up.
    Generates values for the encoders, and stores all the intermediate calculations.
    Units: Degrees (needs to be radians).

    This is virtual math only; the hardware translation layer (class Arm) should convert to encoder ticks.
    Usage:
    armInfo.goal = ...; // goal position
    armInfo.curHeadX = ...; // current head x position
    armInfo.curHeadY = ...; // current head y position
    armInfo.Update(); // convert goal pos to angles
    // RestrictOutputs goes here, because RestrictOutputs is a post-processing thing
*/
    double omega; // Elbow composites - omega = true goal angle, theta = goal dist from shoulder
    double theta;
    double y; // Angle between the appropriate vertical line and the shoulder
    double a; // Angle between the appropriate vertical line and the elbow
    double n; // Real (goal) angle of the shoulder
    double f; // Angle between goal-vector and shoulder
    double x; // Angle of the goal-vector
    double s;


    vector goal; // The program sets these and uses gamma and n.
    double curHeadX;
    double curHeadY;
    // Current head position is needed for Restrict...() functions.

    double GetTheta(){
        double base = goal.magnitude() / 2;
        double thetaOverTwo = asin(base / shoulderBarLengthCM) * 180/PI;
        return smartLoop(thetaOverTwo * 2, 360);
        // It's an isoscelese triangle. That means we can divide it up into two right triangles about the base and theta; each one has an angle of F
        // and an angle of theta/2, and the base is the original base/2. Also, each one has a hypotenuse of shoulderBarLengthCM (or elbowBarLengthCM; they're the same),
        // Sin is defined as opposite/hypotenuse (SOHCAHTOA!), so to get that top angle (which is, remember, theta/2), we use asin(base/shoulderBarLengthCM). The
        // resulting angle is exactly half of theta, so we multiply by two and convert to degrees.
    }

    void Update (){ // Do the calculations to set *every* variable
        x = smartLoop(goal.angle() * 180/PI, 360);
        theta = GetTheta();
        f = smartLoop((180 - theta) / 2, 360); // F is an angle in an isoscelese triangle. It is equal to d. The equation for any triangle is,
        // f + d + theta = 180; since this is an isoscelese triangle, we know that f == d, and f * 2 + theta = 180. Solve for f,
        // we get f = (180 - theta) / 2.
        n = smartLoop(f + x, 360); // n = f + x. Check the chart.
        y = smartLoop(90 - n, 360); // Yep.
        a = smartLoop(theta - y, 360); // Check the chart.
        omega = smartLoop(270 + a - 10, 360); // Check das chart.
        s = smartLoop(180 - a, 360); // Note: s and gamma are related, but 360 - s != gamma.
    }

    void RestrictOutputs(double shoulderMax, double shoulderMin, double elbowMax, double elbowMin){
        assert(shoulderMax > shoulderMin); // C++ feature: assert evaluates an expression, and if it's false throws an error.
        assert(elbowMax > elbowMin); // Insane values for shoulder and elbow mins/maxs could throw off the entire program.
        if (omega > elbowMax){
            omega = elbowMax;
        }
        else if (omega < elbowMin){
            omega = elbowMin;
        }
        if (n > shoulderMax){
            n = shoulderMax;
        }
        else if (n < shoulderMin){
            n = shoulderMin;
        }
    }
};


template <int elbowID, int shoulderID, int boopID, int elbowLimitswitchID, int shoulderLimitswitchID>
class Arm {
public:
    long elbowDefaultEncoderTicks = 0;
    long shoulderDefaultEncoderTicks = 0;

    bool handState = false;
    BaseMotor* shoulder;
    BaseMotor* elbow;
    BaseMotor* hand;
    PIDController<BaseMotor>* elbowController;
    PIDController<BaseMotor>* shoulderController;
    CurrentWatcher* shoulderWatcher;
    CurrentWatcher* elbowWatcher;
    std::vector<ArmPosition> stack;
    vector goalPos;
    ArmInfo info;
    bool retract = false;
    bool sweeping = false;
    frc::DigitalInput elbowLimitSwitch { elbowLimitswitchID };
    frc::DigitalInput shoulderLimitSwitch { shoulderLimitswitchID };

    Arm(BaseMotor* s, BaseMotor* e, BaseMotor* h){
        shoulder = s;
        elbow = e;
        hand = h;
        elbowController = new PIDController(e);
        elbowController -> constants.P = 0.005;
        //elbowController -> constants.D = 0.0045;
        elbowController -> constants.MinOutput = -0.25;
        elbowController -> constants.MaxOutput = 0.25;
        shoulderController = new PIDController(s);
        shoulderController -> constants.P = 0.0025;
        shoulderController -> constants.I = 0;
        //shoulderController -> constants.D = 0.0045;
        shoulderController -> constants.MinOutput = -0.15;
        shoulderController -> constants.MaxOutput = 0.15;
        elbowController -> SetCircumference(4096);
        shoulderController -> SetCircumference(4096);
        shoulder -> ConfigIdleToBrake();
        elbow -> ConfigIdleToBrake();
        shoulderWatcher = new CurrentWatcher { shoulder, 35, 2 };
        elbowWatcher = new CurrentWatcher { elbow, 3, 2 };
    }

    frc::AnalogInput elbowEncoder { elbowID };
    frc::AnalogInput shoulderEncoder { shoulderID };
    frc::DigitalInput boop { boopID };

    void test(){
        //goalX += 0.002;
        //vector goal = { 60, 5 };
        //frc::SmartDashboard::PutNumber("Goal X", goal.x);
        //shoulderController -> SetPosition(halfPos);
        frc::SmartDashboard::PutNumber("Shoulder real", shoulderEncoder.GetValue());
        frc::SmartDashboard::PutNumber("Shoulder nice", GetShoulderPos());
        frc::SmartDashboard::PutNumber("Elbow real", elbowEncoder.GetValue());
        frc::SmartDashboard::PutNumber("Elbow nice", GetElbowPos());
        ArmPosition p = GetArmPosition();
        frc::SmartDashboard::PutNumber("Head X", p.x);
        frc::SmartDashboard::PutNumber("Head Y", p.y);
        frc::SmartDashboard::PutNumber("Shoulder Current", shoulder -> GetCurrent());
        frc::SmartDashboard::PutNumber("Elbow Current", elbow -> GetCurrent());
        frc::SmartDashboard::PutBoolean("Elbow Danger", !elbowWatcher -> isEndangered);
        frc::SmartDashboard::PutBoolean("Shoulder Danger", !shoulderWatcher -> isEndangered);
        //armGoToPos(lowPole);
        //frc::SmartDashboard::PutNumber("Shoulder goal", GetShoulderGoalFrom(goal));
        //frc::SmartDashboard::PutNumber("Elbow goal", GetElbowGoalFrom(goal));
        //frc::SmartDashboard::PutNumber("Shoulder goal ticks", ShoulderAngleToEncoderTicks(GetShoulderGoalFrom(goal)));
        //frc::SmartDashboard::PutNumber("Elbow goal ticks", ElbowAngleToEncoderTicks(GetElbowGoalFrom(goal), GetShoulderGoalFrom(goal)));
    }

    GrabMode grabMode;
    
    void goToHome(bool triggerSol = false) {
        armGoToPos({35, 0});
    }

    void goToPickup() {
        armGoToPos({100, 0});
    }

    void goToLowPole() {
        armGoToPos({lowPole});
    }

    void goToHighPole() {
        armGoToPos({highPole});
    }

    bool checkSwitches() {
        frc::SmartDashboard::PutBoolean("elbow switch", !elbowLimitSwitch.Get()); // elbow is Normally Open because c'est messed up
        frc::SmartDashboard::PutBoolean("shoulder switch", shoulderLimitSwitch.Get()); // shoulder is, as proper, Normally Closed
        bool zero = true;
        if (shoulderLimitSwitch.Get()){
            shoulderDefaultEncoderTicks = shoulderEncoder.GetValue();
        }
        else {
            zero = false;
        }
        if (!elbowLimitSwitch.Get()){
            elbowDefaultEncoderTicks = elbowEncoder.GetValue();
        }
        else {
            zero = false;
        }
        return zero;
    }

    void armGoToPos(vector pos) {
        goalPos = pos;
    }

    int GetNormalizedShoulder(){
        return smartLoop(shoulderDefaultEncoderTicks - shoulderEncoder.GetValue());
    }

    int GetNormalizedElbow(){
        return smartLoop(elbowDefaultEncoderTicks - elbowEncoder.GetValue());
    }

    double GetShoulderPos(){ // Get the shoulder angle in degrees relative to the ground
        return smartLoop(shoulderDefaultAngle - (GetNormalizedShoulder() * 360/4096), 360); // Todo: USE DANG RADIANS
    }

    double GetElbowPos(){ // Ditto
        return smartLoop(elbowDefaultAngle + 10 + (GetNormalizedElbow() * 360/4096) - (90 - GetShoulderPos()), 360);
    }

    // TODO: use radians
    // Actually todo: use sane code instead of this garbage

    double GetShoulderRadians(){
        return GetShoulderPos() * PI/180;
    }

    double GetElbowRadians(){
        return GetElbowPos() * PI/180;
    }

    ArmPosition GetArmPosition(){
        ArmPosition ret;
        double shoulderRads = GetShoulderRadians();
        double elbowRads = GetElbowRadians();
        double elbowX = cos(elbowRads) * elbowBarLengthCM;
        double elbowY = sin(elbowRads) * elbowBarLengthCM;
        double shoulderX = cos(shoulderRads) * shoulderBarLengthCM;
        double shoulderY = sin(shoulderRads) * shoulderBarLengthCM;
        ret.x = shoulderX + elbowX;
        ret.y = shoulderY + elbowY;
        return ret;
    }

    double ElbowAngleToEncoderTicks(double ang, double shoulder){
        return smartLoop(elbowDefaultEncoderTicks - (ang - elbowDefaultAngle + (90 - shoulder)) * 4096/360);
    }

    double ShoulderAngleToEncoderTicks(double ang) {
        return smartLoop((ang - shoulderDefaultAngle) * 4096/360 + shoulderDefaultEncoderTicks); 
    }

    double setX = 35;
    void setRetract(bool s = true) {
        retract = s;
    }

    void armPickup(bool triggerSol = true) {
        goToPickup();
        /*if (!retract) {
            if (!sweeping) {
                armGoToPos({setX, -10});
                if (atGoal()) {
                    sweeping = true;
                }
            }
            else {
                armGoToPos({setX, -10});
                hand -> SetPercent(.35);
                setX += .005;
                if (Has() || setX > 150) {
                    retract = true;
                }
            }
        }*/
    }

    double sAng, eAng;

    bool atGoal(){
        return (std::abs(shoulderEncoder.GetValue() - sAng) < 15) && (std::abs(elbowEncoder.GetValue() - eAng) < 15);
    }

    bool zeroed = false;

    void Update(){
        shoulderWatcher -> Update();
        elbowWatcher -> Update();
        if (!zeroed){
            AuxSetPercent(0.2, 0.1);
            zeroed = checkSwitches();
            return;
        }
        if (retract){
            goToHome();
            if (atGoal()){
                retract = false;
            }
        }
        checkSwitches();
        if (elbowWatcher -> isEndangered || shoulderWatcher -> isEndangered){
            AuxSetPercent(0, 0);
            return;
        }
        ArmPosition pos = GetArmPosition();
        info.goal = goalPos;
        info.curHeadX = pos.x;
        info.curHeadY = pos.y;
        info.Update();
        //info.RestrictOutputs(shoulderDefaultAngle, 0, 360, elbowDefaultAngle);
        if (pos.x > 25 && pos.x < 55) { // Shoulder has to go up if it's finna clear the bumper
            info.n = 80;
        }
        sAng = ShoulderAngleToEncoderTicks(info.n);
        eAng = ElbowAngleToEncoderTicks(info.omega, info.n);
        shoulderController -> SetPosition(sAng);
        elbowController -> SetPosition(eAng);
        frc::SmartDashboard::PutNumber("Shoulder goal", sAng);
        frc::SmartDashboard::PutNumber("Elbow goal", eAng);
        frc::SmartDashboard::PutNumber("Head Goal X", goalPos.x);
        frc::SmartDashboard::PutNumber("Head Goal Y", goalPos.y);

        shoulderController -> Update(shoulderEncoder.GetValue());
        elbowController -> Update(elbowEncoder.GetValue());

        grabMode = OFF; // ain't sticky - don't want breakies
    }

    bool Has(){
        return !boop.Get();
    }

    void SetGrab(GrabMode mode){
        grabMode = mode;
    }

    bool shoulderAtLimit(){ // These do *not* return the state of the limit switch; they return whether or not the respective motor is at its limit. Thus they also include watchers in their math.
        return shoulderLimitSwitch.Get() || shoulderWatcher -> isEndangered;
    }

    bool elbowAtLimit(){
        return !elbowLimitSwitch.Get() || elbowWatcher -> isEndangered;
    }

    void Zero(){
        zeroed = false;
    }

    void AuxSetPercent(double s, double e){
        shoulderWatcher -> Update();
        elbowWatcher -> Update();
        if ((shoulderAtLimit() && (s > 0)) || shoulderWatcher -> isEndangered){
            s = 0;
        }
        if ((elbowAtLimit() && (e > 0)) || elbowWatcher -> isEndangered){
            e = 0;
        }
        shoulder -> SetPercent(s);
        elbow -> SetPercent(e);
    }
};