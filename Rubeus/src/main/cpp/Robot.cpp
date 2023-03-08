/* By Tyler Clarke
	This is an experiment with c++20 features and new paradigms to make FRC robot code cleaner.
	The idea: structuring an FRC robot like a real C++ program instead of like Java gone even wronger. Craszy.
*/

#define PI 3.141592

#include <iostream>
#include <fstream>
#include <FRL/bases/AwesomeRobotBase.hpp>
#include <FRL/motor/SparkMotor.hpp>
#include <FRL/swerve/SwerveModule.hpp>
#include <constants.h>
#include <frc/XboxController.h>
#include <frc/GenericHID.h>
#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <arm.hpp>
#include <photonlib/PhotonCamera.h>
#include <FRL/util/vector.hpp>

#include "controls.hpp"
#include "Positionizer.hpp"
#include "apriltags.h"

const vector blue_mid_ramp {12.8, -1.9};

SwerveModule frontLeftSwerve (
	new SparkMotor(FRONT_LEFT_SPEED),
	new SparkMotor(FRONT_LEFT_DIREC), 
	FRONT_LEFT_CANCODER,
	1, 
	-1024 + FRONT_LEFT_OFFSET
);

SwerveModule frontRightSwerve (
	new SparkMotor(FRONT_RIGHT_SPEED),
	new SparkMotor(FRONT_RIGHT_DIREC),
	FRONT_RIGHT_CANCODER,
	2, 
	1024 + FRONT_RIGHT_OFFSET
);

SwerveModule mainSwerve (
	new SparkMotor(BACK_LEFT_SPEED),
	new SparkMotor(BACK_LEFT_DIREC),
	BACK_LEFT_CANCODER,
	4, 
	1024 + BACK_LEFT_OFFSET
);

SwerveModule backRightSwerve (
	new SparkMotor(BACK_RIGHT_SPEED),
	new SparkMotor(BACK_RIGHT_DIREC),
	BACK_RIGHT_CANCODER,
	3, 
	-1024 + BACK_RIGHT_OFFSET
);

Arm <1, 0, 2, 1, 0> arm {
	new SparkMotor(ARM_SHOULDER),
	new SparkMotor(ARM_ELBOW),
	new SparkMotor(ARM_HAND)
};

frc::DoubleSolenoid armSol { frc::PneumaticsModuleType::CTREPCM, 0, 1 };

AHRS navx {frc::SPI::Port::kMXP}; // Well, obviously, the navx
double navxOffset = 0;


Odometry <NUM_APRILTAGS, apriltags, &navx> odometry ("OV5647"); /* This is what we call misusing templates and doing a bad job of it */
frc::Compressor compressor {frc::PneumaticsModuleType::CTREPCM};

Controls <5, 4, 3> controls;

long navxHeading(){
	return navx.GetFusedHeading() - navxOffset;
}

long navxHeadingToEncoderTicks(){
	return navxHeading() * 4096/360;
}

void zeroNavx(){
	navxOffset = navx.GetFusedHeading();
}

bool onRamp = false;
double approachethAngle = 0;

void autoRamp() {
    if (!onRamp) {
        mainSwerve.SetDirection(90 * (4096/360));
        frc::SmartDashboard::PutNumber("Navx heading", navxHeadingToEncoderTicks());
        approachethAngle = smartLoop(navxHeadingToEncoderTicks() + -180, 360);
        mainSwerve.SetPercent(.27);
        if (navx.GetRoll() * -1 > 10) {
            onRamp = true;
        }
    }
    else {
        mainSwerve.SetDirection(90 * (4096/360));
        float speed = navx.GetRoll() * -1 * .017;
        frc::SmartDashboard::PutNumber("Ramp Load Speed", speed);
        mainSwerve.SetPercent(speed);
    }
}


bool driveTo(vector goal){
    auto pos = odometry.Update();
    vector translation = { pos.y - goal.y, pos.x - goal.x };
    translation.speedLimit(0.2);
    translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));
    translation = translation.flip();
    mainSwerve.SetToVector(translation, { 0, 0 });
    return translation.magnitude() < 0.05;
}

enum MacroMode {
    TERMINATOR,
    ARM_TYPE,
    DRIVE_TYPE,
    RAMP_TYPE,
    BARF_TYPE
};



struct MacroOp {
    MacroMode type;
    vector pos = {};
};

class MacroController {
    MacroOp* mnm = 0;
    size_t sP;
    bool rampA = false;
public:
    MacroController(bool rampAfter) {
        rampA = rampAfter;
    } 
    void operator=(MacroOp m[]){
        mnm = m;
        sP = 0;
    }

    void Update(){
        if (mnm == 0){
            return;
        }
        MacroOp thing = mnm[sP];
        switch (thing.type) {
            case TERMINATOR:
                mnm = 0;
                sP = 0;
                std::cout << "Macro unloaded" << std::endl;
                arm.AuxSetPercent(0, 0);
                break;
            case ARM_TYPE:
                onRamp = false;
                arm.armGoToPos(thing.pos);
                if (arm.atGoal()) {
                    sP ++;
                    std::cout << "There! " << std::endl;
                }
                break;
            case DRIVE_TYPE:
                onRamp = false;
                if (driveTo(thing.pos)) {
                    sP ++;
                }
                break;
            case RAMP_TYPE:
                autoRamp();
                break;
            case BARF_TYPE:
                arm.SetGrab(BARF);
                if (!arm.Has()) {
                    sP ++;
                }
                break;
        }
    }
};
MacroOp autoMacro[] {
    {
        DRIVE_TYPE,
        blue_mid_ramp
    },
    {
        RAMP_TYPE
    },
    {
        TERMINATOR
    }
};
frc::GenericHID buttonboard {5};
class TeleopMode : public RobotMode {
public:
	ArmPosition p { 120, 120 };

	Position2D goal { 2, 0 };

	void Start(){
		zeroNavx();
        compressor.EnableDigital();
        macros = autoMacro;
	}

    vector g = { 45, -5 };
    MacroController macros {true};

    void armAux(){ // Arm auxiliary mode
        if (controls.GetButton(ELBOW_CONTROL)){
            arm.AuxSetPercent(0, controls.LeftY());//g.x += controls.LeftY() * 5;
        }
        else if (controls.GetButton(SHOULDER_CONTROL)) {
            arm.AuxSetPercent(controls.LeftY(), 0);//g.y += controls.LeftY() * 5;
        }
        else{
            arm.AuxSetPercent(0, 0);
        }
        arm.test();
        arm.checkSwitches(); // Always check switches. Friggin' always.
        controls.update();
    }

	void Synchronous(){
		Position2D pos = odometry.Update();
		frc::SmartDashboard::PutNumber("Odometry nearest angle", odometry.NearestAngle() * 180/PI);
		frc::SmartDashboard::PutNumber("Odometry X", pos.x);
		frc::SmartDashboard::PutNumber("Odometry Y", pos.y);
		frc::SmartDashboard::PutNumber("Odometry Quality", odometry.Quality());
        /*if (owner != 0){
            if (!owner -> Execute()){
                owner = 0;
            }
            return; // lock control of the robot into a macro
        }*/
		vector translation {controls.LeftX(), controls.LeftY()};
		vector rotation;
		rotation.setMandA(controls.RightX(), PI/4);

		float limit = controls.GetSpeedLimit();
		frc::SmartDashboard::PutNumber("Speed limit", limit);

		rotation.dead(0.2);
		translation.dead(0.12);

		rotation.speedLimit(limit);
		translation.speedLimit(limit);

		translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));

		if (controls.GetButton(ZERO_NAVX)){
			zeroNavx();
		}

		if (controls.GetButton(ARM_INTAKE)){
			arm.SetGrab(INTAKE);
		}

		if (controls.GetButton(ARM_BARF)){
			arm.SetGrab(BARF);
		}

		if (controls.GetButton(ELBOW_CONTROL)){
            arm.AuxSetPercent(0, controls.LeftY());
        }
        else if (controls.GetButton(SHOULDER_CONTROL)) {
            arm.AuxSetPercent(controls.LeftY(), 0);
        }
        else{
            mainSwerve.SetToVector(translation, rotation.flip());
            arm.Update();
        }

        if (controls.GetKey()){
            armSol.Set(frc::DoubleSolenoid::Value::kForward);
        }
        else {
            armSol.Set(frc::DoubleSolenoid::Value::kReverse);
        }

        if (controls.GetButtonReleased(ARM_PICKUP)){
            arm.setRetract();
        }

        if (controls.GetButtonReleased(ZERO)){
            arm.Zero();
        }

        frc::SmartDashboard::PutNumber("Maura is a WOMAN", controls.GetOption());

        if (controls.GetButton(ARM_PICKUP)) { // like a woman, this code doesn't work
            arm.goToPickup();
            arm.SetGrab(INTAKE);
        }
        else if (controls.GetOption() == 1) {
            arm.goToHighPole();
        }
        else if (controls.GetOption() == 2) {
            arm.goToLowPole();
        }
        else {
            arm.goToHome();
        }
        arm.test();
		// Should run periodically no matter what - it cleans up after itself
        mainSwerve.ApplySpeed();
        controls.update();
	}
};



class AutonomousMode : public RobotMode {
	vector rotation;
	vector translation;
	PIDController <vector> autoRotationController { &rotation };
public:
	AutonomousMode(){
		autoRotationController.SetCircumference(360);
		autoRotationController.constants.MinOutput = -0.1;
		autoRotationController.constants.MaxOutput = 0.1;
		autoRotationController.constants.P = -0.002;
	}

	void Start(){

	}

    vector goal;

	void Synchronous(){
        goal = {1, 0};
        auto pos = odometry.Update();
        translation = { pos.y - goal.y, pos.x - goal.x };
        translation.speedLimit(0.2);
        translation.setAngle(smartLoop(PI - translation.angle() + (navxHeading() * PI/180), PI * 2));
        translation = translation.flip();
        autoRotationController.SetPosition(0);//odometry.NearestAngle() * 180/PI); // Always rotate to the nearest apriltag
        autoRotationController.Update(navxHeading());
        rotation.setAngle(PI/4); // Standard rotation.
		mainSwerve.SetToVector(translation, rotation);
		mainSwerve.ApplySpeed();
	}
};


class TestMode : public RobotMode {

};


class DisabledMode : public RobotMode {

};


#ifndef RUNNING_FRC_TESTS // I'm afraid to remove this.
int main() {
    compressor.Disable();
	mainSwerve.Link(&backRightSwerve); // Weird, right? This can in fact be used here.
	backRightSwerve.Link(&frontRightSwerve);
	frontRightSwerve.Link(&frontLeftSwerve);
	mainSwerve.SetLockTime(1); // Time before the swerve drive locks, in seconds
	// As it turns out, int main actually still exists and even works here in FRC. I'm tempted to boil it down further and get rid of that stupid StartRobot function (replace it with something custom inside AwesomeRobot).
	return frc::StartRobot<AwesomeRobot<TeleopMode, AutonomousMode, TestMode, DisabledMode>>(); // Look, the standard library does these nested templates more than I do.
}
#endif
