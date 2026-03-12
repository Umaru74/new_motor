// "MotorManager" class handles the configuration of physical motor connectors (M-0 through M-3)
// These motor connectors are represented and controlled by instances of the "MotorDriver" class

// "Step & Direction" mode needs to be set on MSP software
// HLFB_MODE_STATIC required when MSP HLFB is configured for
//"In Range - Position" or "In Range - Velocity"
// Set the Input Resolution 

#include "ClearCore.h"
#define motor1 ConnectorM0

#define MOTOR_COUNT 1

#define SerialPort ConnectorUsb

// Group connectors into an array for the [i] loops, this is for 1 motor, M-0 
MotorDriver *motors[] = {&motor1};

// Hardware Constants
const int32_t encoderResolution = 8000; // Counts per revolution (set to match MSP) , 8000 RPM CAP for step/dir mode
const int32_t velocityRPM = 2000; // Target RPM 
const int32_t velocityMAX = 2000; // max RPM
const uint32_t accelerationLimit = 300; // RPM / sec^2
const uint32_t decelerationLimit = 300; // RPM / sec^2
const int32_t homingVelocity = -300; // move negative toward hard stop
const int32_t exitHomingMove = 200; // Distance to move away to exit homing mode
const int32_t baudRate = 115200; // Communication speed for USB, baudRate needs to match with the host computer, monitor

bool isRotating = false; // Global flag to see if motors are at the desired speed (not ramping)

// Define phase offsets in degree, corresponding to motor 0, 1 (+ve goes CCW , -ve goes CW)
const int32_t offsets[] = {-90};

// Define specific motor location to move to (such as when to close the valve for emergency)
const int32_t targets[] = {180}; // in degree with respect to the motor 0, initial location

// Function: Check if the motor is stationary or rotating 
bool AreMotorsStationary() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // StepsActive is true if the motor is currently executing a move pulses [1]
        if (motors[i]->StatusReg().bit.StepsActive) {
            return false; 
        }
    }
    return true; 
}

// Function: Synchronize each motor individually with user confirmation
void ManualSyncZero() {
    SerialPort.SendLine("\n--- SEQUENTIAL MANUAL SYNC START ---");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        // 1. Prompt User for the specific motor
        SerialPort.Send("ACTION REQUIRED: Prepare Motor ");
        SerialPort.Send(i);
        // Set the homing position to 0 in MSP
        SerialPort.SendLine(" (Lock rotor and click Ctrl+0 in MSP).");
        SerialPort.SendLine("Press 'Y' when this motor is ready to sync...");

        // 2. Wait for user confirmation ('Y') for this specific motor
        char response = 0;
        while (response != 'Y' && response != 'y') {
            if (SerialPort.AvailableForRead() > 0) {
                response = SerialPort.CharGet(); // Capture individual response 
            }
        }

        // 3. Enable the motor and wait for HLFB (All Systems Go / In-Range)
        // Enabling energizes the coils to hold the physical position 
        motors[i]->EnableRequest(true);
        
        // 4: Wait for motor to finish enabling sequence
        while (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            // Safety: Abort if an alert (like a tracking error) occurs 
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Alert on Motor "); 
                SerialPort.Send(i);
                SerialPort.SendLine("! Homing aborted.");
                return;
            }
            Delay_ms(1);
        }

        // 5. Set the Logical Zero in ClearCore code
        // This ensures the current shaft orientation is treated as "0" for absolute moves
        motors[i]->PositionRefSet(0);
        Delay_ms(10);

        // 6: Disble the motor
        motors[i]->EnableRequest(false);

        SerialPort.Send("Motor ");
        SerialPort.Send(i);
        SerialPort.SendLine(" SUCCESS: Enabled and Zeroed.\n");
    }

    SerialPort.SendLine("--- ALL MOTORS SYNCED ---");
    SerialPort.SendLine("CRITICAL: Remove all locking rods before commanding 'R' (Run).");
}

// function to set phase/angle differences
void SetPhaseAngles(const int32_t angles[]){
    // Safety Check: Ensure no moves are in progress before starting phase alignment
    if (!AreMotorsStationary()) {
        SerialPort.SendLine("ERROR: Cannot set phase while motors are running! Command 'S' first.");
        return; 
    }
    SerialPort.SendLine("Applying phase offsets...");
    for (int i = 0; i < MOTOR_COUNT; i++){
        // Convert degrees to counts 
        int32_t distance = (angles[i] * encoderResolution) / 360;
        // Move(): move to the absolute position (0 ~ 8000)
        motors[i]->Move(distance, StepGenerator::MOVE_TARGET_ABSOLUTE);

    }
    Delay_ms(1);
    // Wait for all to finish
    for (int i = 0; i <MOTOR_COUNT; i++){
        while(!motors[i]->StepsComplete()){
            Delay_ms(1);
        }
        //motors[i]->PositionRefSet(0); // Make the offset the new "Logical 0"
        Delay_ms(10);
    }
    SerialPort.SendLine("Phase lock established");
}

// function: Start rotation at RPM
void StartRotation(int32_t rpm){
    // Convert RPM to steps/sec: (RPM/60)*8000
    int32_t velocityStepsPerSec = (encoderResolution * rpm ) / 60;

    SerialPort.Send("Starting synchronized rotation at RPM: ");
    SerialPort.SendLine(rpm);

    // Command the velocity move to all four motors synchronously
    for (int i = 0; i < MOTOR_COUNT; i++){
         motors[i]-> MoveVelocity(velocityStepsPerSec);
    }

    // Wait for the step command to ramp up to the commanded velocity
    SerialPort.SendLine("Ramping to speed...");
    uint32_t startTime = Milliseconds();

    bool allAtSpeed = false;
    while (!allAtSpeed) {
        allAtSpeed = true;

        for (int i = 0; i < MOTOR_COUNT; i++) {

            // Check for motor alerts
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Motor Alert During Ramping: Motor ");
                SerialPort.SendLine(i);
                StopAll();
                return;
            }

            // Check if motor reached cruise velocity
            if (!motors[i]->CruiseVelocityReached()) {
                allAtSpeed = false;
            }
        }

        // Time Out Protection: 15s
        if (Milliseconds() - startTime > 15000) {
            SerialPort.SendLine("Error: Motors failed to reach target speed!");
            StopAll();
            return;
        }

        Delay_ms(1); // reduce CPU usage
    }
    isRotating = true;
    SerialPort.Send("All motors have hit the desired RPM:");
    SerialPort.SendLine(rpm);
}

// function: Stop all motors smoothly
void StopAll() {
    SerialPort.SendLine("Initiating controlled stop for all motors...");

    // Convert acceleration limit (RPM/s^2) to pulses / s^2
    int32_t decelStepsPerSec_2 = (encoderResolution * decelerationLimit ) / 60;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        // "MoveStopDecel": Interrupts any current move and commands the motor to stop. 
        motors[i]->MoveStopDecel(decelStepsPerSec_2);
    }

    // Wait for all motors to ramp down and reach zero velocity
    bool allStopped = false;

    while (!allStopped) {
        allStopped = true; 
        for (int i = 0; i < MOTOR_COUNT; i++) {
            // Check alerts during stopping
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Motor Alert While Stopping: Motor ");
                SerialPort.SendLine(i);
                return;
            }
            // Check if motor stops
            if (!motors[i]->StepsComplete()) {
                allStopped = false;
            }
        }
        Delay_ms(1);
    }
    isRotating = false;
    SerialPort.SendLine("All motors have come to a stop.");
}

// Function: Smoothly decelerate from 2000 RPM to a specific absolute count
void MoveMotorsToTargets(const int32_t targets[]) {
    if(!isRotating){
        SerialPort.SendLine("Moving motors to individual absolute targets...");

        for (int i = 0; i < MOTOR_COUNT; i++) {
            // Always check for alerts before commanding a move 
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Motor "); 
                SerialPort.Send(i); 
                SerialPort.SendLine(" in alert. Skipping.");
                continue;
            }
            int32_t target_angle = (targets[i] * encoderResolution) / 360;
            // Use MOVE_TARGET_ABSOLUTE to move to the exact count relative to zero
            motors[i]->Move(target_angle, StepGenerator::MOVE_TARGET_ABSOLUTE);     // Need to convert targets degree to counts 
        }

        uint32_t startTime = Milliseconds();

        // Wait for all motors to physically arrive at their targets
        for (int i = 0; i < MOTOR_COUNT; i++) {
            while (!motors[i]->StepsComplete()) {
                if (Milliseconds() - startTime > 15000){
                    SerialPort.SendLine("Error: Motor move timed out");
                    break;
                }
                Delay_ms(1);
            }
        }

        SerialPort.SendLine("All motors arrived at target counts.");
    }
    if(isRotating){
        SerialPort.SendLine("Stop Motors before moving to targets");
        return;
    }
}

void EnableAllMotors(){
    SerialPort.SendLine("Enabling all motors...");
    // "EnableRequest": Request all motors to enable
    for (int i = 0; i < MOTOR_COUNT; i++){
        motors[i]->EnableRequest(true);
    }

    uint32_t startTime = Milliseconds();

    // Wait for all motors to physically energize and clear alerts
    bool allReady = false;
    // Wait for all motors to finish enabling
    // while (!allReady) {
    //     allReady = true; 
    //     for (int i = 0; i < MOTOR_COUNT; i++) {
    //         // If HLFB is still de-asserted, we are NOT ready yet
    //         if (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
    //             allReady = false;
    //         }

    //         // CRITICAL: Only return an error if an alert is ACTUALLY present
    //         if (motors[i]->StatusReg().bit.AlertsPresent) {
    //             SerialPort.Send("Motor Fault Detected: ");
    //             SerialPort.SendLine(i);
    //             return; // Hardware failure (e.g., no DC power or tracking error)
    //         }
    //     }

    //     if (Milliseconds() - startTime > 15000) {
    //         SerialPort.SendLine("Timeout enabling motors!");
    //         return;
    //     }
    //     Delay_ms(1);
    // }

    // "ValidateMove": verify that the motor is in a good state before sending a move command 
    for (int i =0; i < MOTOR_COUNT; i++){
        if (!motors[i]->ValidateMove(false)){
            SerialPort.SendLine("Error: A motor failed validation after enabling");
            return;
        }
    }

    SerialPort.SendLine("All motors are enabled, and ready to go");
}

void DisableAllMotors() {
    SerialPort.SendLine("Disabling all motors...");

    // Check if all motors are stationary
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // StepsComplete() returns true if the trajectory generator is idle 
        // VelocityRefCommanded() == 0 ensures the target speed is zero
        if (!motors[i]->StepsComplete() || motors[i]->VelocityRefCommanded() != 0) {
            SerialPort.SendLine("ERROR: Motion detected! Cannot disable motors while moving.");
            SerialPort.SendLine("Stop the moving motors first before disabling...");
            return; // Exit the function immediately
        }
    } 

    // Send the disable request to all motors
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // Passing false de-asserts the Enable signal and removes coil power 
        motors[i]->EnableRequest(false);
    }

    // Wait for the hardware to confirm the motors are disabled
    bool allDisabled = false;
    while (!allDisabled) {
        allDisabled = true; 
        for (int i = 0; i < MOTOR_COUNT; i++) {
            // Check if internal hardware state is MOTOR_DISABLED 
            if (motors[i]->StatusReg().bit.ReadyState != MotorDriver::MOTOR_DISABLED) {
                allDisabled = false;
            }
        }
        Delay_ms(1);
    }

    SerialPort.SendLine("All motors are disabled and de-energized.");
}

// HLFB mode needs to be set for "In Range-Velocity" on MSP for each motor 
bool allMotorsInRange(){
    for (int i = 0; i < MOTOR_COUNT; i++){
        // HlfbState() returns HLFB_ASSERTED when the motor is physically
        // within the MSP-defined speed window 
        if (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED){
            SerialPort.Send("Motor out of range: ");
            SerialPort.SendLine(i);
            return false; // At least one motor has drifted or is still ramping
        }
    }
    return true; // All motors are physically at 2000 RPM (within tolerance)
}

void setup() {
    // Set the input clocking rate for the MotorDriver connectors as a group, they cannot be individually set. 
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

    // Put all motors into Step and Direction mode. They cannot be individually set. 
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Convert acceleration limit (RPM/s^2) to pulses / s^2
    int32_t accelStepsPerSec_2 = (encoderResolution * accelerationLimit ) / 60;
    int32_t velmaxStepsPerSec_2 = (encoderResolution * velocityMAX) / 60;

    for (int i = 0; i < MOTOR_COUNT; i++){
        motors[i]->AccelMax(accelStepsPerSec_2);
        motors[i]->VelMax(velmaxStepsPerSec_2);

        // HLFB_Mode_Static is required for In-Range, ASG, and Servo On Modes
        motors[i]->HlfbMode(MotorDriver::HLFB_MODE_STATIC);
    }

    uint32_t start = Milliseconds();

    // Set the mode to USB Serial
    SerialPort.Mode(Connector::USB_CDC);
    // Set the communication speed (Baud Rate)
    SerialPort.Speed(baudRate); // 9600 or 115200 / 
    SerialPort.PortOpen();
    while(!SerialPort){
        if (Milliseconds() - start > 5000){
            SerialPort.SendLine("Communication between the motors and controller was not established within 5 seconds");
            break;
        };
    }
    SerialPort.SendLine("Commands: 'M'= Manual Homing, 'D'= Disable All the motors, 'E'= Enable All the motors, 'P'= Set Phase Angles, 'R'= Start Rotation, 'S'= Stop Rotation, 'G'= Move motors to targets");
}

void loop() {
    // 1: Listen to Serial Commands to trigger functions
    if (SerialPort.AvailableForRead() > 0){
        char cmd = SerialPort.CharGet();
        switch(cmd){
            case 'M': case 'm': 
                SerialPort.SendLine("----Manual Synchronization Command Initialized----");
                ManualSyncZero(); 
                break;
            case 'D': case 'd': 
                SerialPort.SendLine("----Disable All Motors Command Initialized----");
                DisableAllMotors(); 
                break;
            case 'E': case 'e': 
                SerialPort.SendLine("----Enable All Motors Command Initialized----");
                EnableAllMotors(); 
                break;
            case 'P': case 'p': 
                SerialPort.SendLine("----Set Phase Offsets Command Initialized----");
                SetPhaseAngles(offsets); 
                break;
            case 'R': case 'r': 
                SerialPort.SendLine("----Start Rotation Command Initialized----");
                StartRotation(velocityRPM); 
                break;
            case 'S': case 's': 
                SerialPort.SendLine("----Stop All Motors Command Initialized----");
                StopAll();
                break;
            case 'G': case 'g': 
                SerialPort.SendLine("----Move Motors To Targets Command Initialized----");
                MoveMotorsToTargets(targets); 
                break;
        }
    }

    // 2: Continuous Monitoring (Only when motors are at the desired speed)
    if (isRotating){
        if (!allMotorsInRange()){
            SerialPort.SendLine("CRITICAL: Synchronization lost or motor stalled!");
            StopAll(); // safety action, stop the motors when they are not synchronizing 
            isRotating = false;
        }
    }
}

