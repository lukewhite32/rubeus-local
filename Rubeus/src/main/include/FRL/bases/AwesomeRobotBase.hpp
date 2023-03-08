/* By Tyler Clarke
    More modern version of ModularRobot based on C++20. Experimental, but stable enough for usage at the moment.
*/

#include <frc/RobotBase.h> // todo: trim down includes
#include <vector>
#include <pthread.h>
#include <atomic>
#include <string>
#include <frc/Timer.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <frc/internal/DriverStationModeThread.h>
#include <frc/DriverStation.h>


/**
 * @author Tyler Clarke
 * @version 1.0
 * 
 * Base class for all robot operation modes.
 * Subclass this for autonomous, teleop, test, and disabled modes in a robot;
 * AwesomeRobot uses templating magic to use your code.
*/

class RobotMode {
public:
    /**
     * Speed in hertz that the thread updates at.
     */
    float hz = 50;
    
    /**
     * Whether or not the thread is enabled.
 
     * This is a formality only; it is unlikely to ever find use.
     */
    std::atomic <bool> noThread = false; // Set to true if thread creation failed. Should be checked occasionally.

    /**
     * Override this as your initial setup.
     * Called once when the robot program starts.
     */
    virtual void Init(){

    }

    /**
     * Override this as your setup function.
    
     * Called every time the robot is enabled to the mode - e.g. going to teleoperated from autonomous, or enabling teleoperated.
     */
    virtual void Start() {

    }
    
    /**
     * Override this for a synchronized mainloop.
    
     * Called every *synchronous* tick - equivalent to the Loop functions in ModularRobot.
     */
    virtual void Synchronous() {

    }

    /**
     * Override this for code that needs to run asynchronously at hz.
     * Called asynchronously at a frequency - safe to have infinite loops, because it's killed on end
     * MEMORY SAFETY NOTE: BECAUSE THIS CAN BE CANCELED AT ANY TIME, NEVER ALLOCATE NEW MEMORY INSIDE IT! A MEMORY LEAK WILL OCCUR!
     * Even if memory is freed in End(), being killed during a malloc() can leave the internal linked-list of free memory blocks in an "inconsistent state",
     * which could potentially crash the entire process - maybe even force a kernel panic on the RoboRio.
     * Lots of bad things happen when you play with memory inside threads, no matter what.
     * BUG NOTE: There is a chance the CTRE and Rev libraries will allocate memory on callug notes to their libraries.
     * If they do, we might start seeing a stream of seemingly random segfaults (or even kernel panics).
     * That is probably caused by bad Care Of Memory.
     * Okayish safety solution: Use Start() and the constructor and the default constructors to allocate everything, and do nothing but call safe or safe-ish functions here.
     * Good safety solution: don't ever directly call library functions in this thread, and instead use std::atomics to store things like motor speeds, to be updated synchronously.
     * It is safe to use infinite while loops in here! The thread handling intentionally permits it.
     * DEPRECATED: pthread_cancel doesn't work properly on the RoboRIO so threads are deprecated for now.
     */
    virtual void Thread() {

    }

    /**
     * Override to clean-up.
     * Called when the current mode exits - e.g. going from autonomous to teleoperated, or disabling the robot.
     */
    virtual void End() {

    }
};


/**
 * @author Tyler Clarke
 * Concept for robot modes. It's a c++20 feature that allows us to template in the mode types, initialize them, and call functions on them.
 * Really, really powerful. I recommend reading up on it.
 */
template <typename T>
concept RobotModeType = std::is_base_of <RobotMode, T>::value;

/**
 * @author Tyler Clarke
 * @version 1.0
 * Templated class that manages threads, calls the right mode at the right time, and etc.
 */
template <RobotModeType TeleopModeType, RobotModeType AutonomousModeType, RobotModeType TestModeType, RobotModeType DisabledModeType>
class AwesomeRobot : public frc::RobotBase {
    /**
     * From WPILib. This variable controls whether or not to exit. It's a threading thing.
     */
    std::atomic<bool> m_exit = false;

    /**
     * Heap allocated teleop mode object.
     * Stack allocation *would* have been better, but I really wanted to be able to say "activeMode == disabled" without having to take a reference every time, and activeMode does in fact have to be a pointer.
     */
    TeleopModeType* teleop = new TeleopModeType;
    /**
     * Heap allocated autonomous mode object
     * See {@link teleop}
     */
    AutonomousModeType* autonomous = new AutonomousModeType;
    /**
     * Heap allocated test mode object.
     * See {@link teleop}
     */
    TestModeType* test = new TestModeType;
    /**
     * Heap allocated disabled mode object.
     * See {@link teleop}
     */
    DisabledModeType* disabled = new DisabledModeType;

    /**
     * Store a pointer to the active mode. If this is 0, no mode is active. This is default, safe, and checked for; it will only be 0 until the robot turns on and does the disabled check.
     */
    RobotMode* activeMode = 0;

    /**
     * Internal FRC stuff
     */
    frc::internal::DriverStationModeThread modeThread;

    /**
     * The actual thread function
     * DEPRECATED: pthread_cancel doesn't work properly on the RoboRIO so threads are deprecated for now.
     @param _robot A void* pointer to the AwesomeRobot object. Necessary because this is static.
     */
    static void* thread_function(void* _robot){
        AwesomeRobot* robot = (AwesomeRobot*)_robot;
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL); // Set how the thread should handle being cancelled
        // The Asynchronous cancel type means it can be canceled at any time; the alternative "deferred" means it waits until a library function checks if there's a queued cancel request.
        // We want to be able to run infinite loops in here, so asynchronous is the best and only option.
        double lastTime = (double)frc::Timer::GetFPGATimestamp();
        float frameSecs = 1/robot -> activeMode -> hz; // Maintain this many seconds per frame
        while (true){ // Frequency loop
            // note that it's totally safe to run infinite loops, use delays, and etc in your Thread() code! This is just a frequency controller.
            robot -> activeMode -> Thread();
            double timeElapsed = (double)frc::Timer::GetFPGATimestamp() - lastTime; // Seconds the frame took
            if (timeElapsed < frameSecs){ // If it took too long, don't even bother waiting
                sleep((frameSecs - timeElapsed)); // Wait to fill in the missed 1/hz seconds.
            }
        }
    }

    /**
     * Switch the mode of the robot
     @param toMode The mode to switch control to.
     */
    void switchMode(RobotMode* toMode){
        if (activeMode){ // If the pointer is actually set and isn't initial 0
            // It is, however, guaranteed that the thread will be exited as soon as pthread_join returns
            // NULL means don't bother with the return value of the pthread's function
            activeMode -> End(); // End() may do cleanup work concerning threads (there are situations where this is acceptable, such  as when memory has been allocated by Start specifically for the usage of that thread),
            // so it's important that the thread really be exited by the time End() runs.
        }
        activeMode = toMode;
        toMode -> Start();
        // Start the thread for that mode
    }
public:

    /**
     * Called by WPILib internal; overloaded here. Has a mainloop that runs while m_exit is false.
     */
    void StartCompetition(){
        wpi::Event event{false, false};
        frc::DriverStation::ProvideRefreshedDataEventHandle(event.GetHandle());

        HAL_ObserveUserProgramStarting();

        teleop -> Init();
        autonomous -> Init();
        disabled -> Init();
        test -> Init();

        while (!m_exit){
            modeThread.InDisabled(false);
            modeThread.InAutonomous(false);
            modeThread.InTeleop(false);
            modeThread.InTest(false);
            if (IsDisabled()){ // Disabled tasks
                modeThread.InDisabled(true);
                if (activeMode != disabled){
                    switchMode(disabled);
                    std::cout << "Switched to Disabled Mode" << std::endl;
                }
            }
            else if (IsAutonomous()){ // Autonomous tasks
                modeThread.InAutonomous(true);
                if (activeMode != autonomous){
                    switchMode(autonomous);
                    std::cout << "Switched to Autonomous Mode" << std::endl;
                }
            }
            else if (IsTest()){ // Test tasks
                modeThread.InTest(true);
                if (activeMode != test){
                    switchMode(test);
                    std::cout << "Switched to Test Mode" << std::endl;
                }
            }
            else{ // Teleop tasks
                modeThread.InTeleop(true);
                if (activeMode != teleop){
                    switchMode(teleop);
                    std::cout << "Switched to Teleop Mode" << std::endl;
                }
            }
            activeMode -> Synchronous(); // Synchronous looping
        }
    }

    /**
     * Called by FRC internal; sets {@link m_exit} to true.
     */
    void EndCompetition() {
        m_exit = true;
    }
};
