// "MotorManager" class handles the configuration of physical motor connectors (M-0 through M-3)
// These motor connectors are represented and controlled by instances of the "MotorDriver" class

// "Step & Direction" mode needs to be set on MSP software
// HLFB_MODE_STATIC required when MSP HLFB is configured for
//"In Range - Position" or "In Range - Velocity"
// Set the Input Resolution 

#include "ClearCore.h"
#define motor1 ConnectorM0
#define motor2 ConnectorM1
#define motor3 ConnectorM2
#define motor4 ConnectorM3

int MOTOR_COUNT = 0; // Run-time motor count 


#define SerialPort ConnectorUsb

// --- Full pool of all 4 connectors ---
MotorDriver *motorPool[] = {&motor1, &motor2, &motor3, &motor4};

// --- Active motors array (pointer into motorPool, sized at runtime) ---
// We declare the max size statically; only [0..MOTOR_COUNT-1] are used.
MotorDriver *motors[4] = {nullptr, nullptr, nullptr, nullptr};

// Hardware Constants
const int32_t encoderResolution = 8000; // Counts per revolution (set to match MSP) , 8000 RPM CAP for step/dir mode
const int32_t velocityRPM = 1000; // Target RPM 
const int32_t velocityMAX = 1000; // max RPM
const uint32_t accelerationLimit = 300; // RPM / sec^2
const uint32_t decelerationLimit = 300; // RPM / sec^2
const int32_t homingVelocity = 300; // move negative toward hard stop
const int32_t exitHomingMove = 200; // Distance to move away to exit homing mode
const int32_t baudRate = 115200; // Communication speed for USB, baudRate needs to match with the host computer, monitor

bool isRotating = false; // Global flag to see if motors are at the desired speed (not ramping)

// Define phase offsets in degree, corresponding to motor 0, 1 (+ve goes CCW , -ve goes CW)
const int32_t offsets[] = {0, 90, 180, 270};

// Define specific motor location to move to (such as when to close the valve for emergency)
const int32_t targets[] = {0, 0, 0, 0}; // in degree with respect to the motor 0, initial location


// -----------------------------------------------------------------------
// Startup: Ask the user how many motors to activate (1-4)
// -----------------------------------------------------------------------
void PromptMotorCount() {
    SerialPort.SendLine("==============================================");
    SerialPort.SendLine("  ClearCore Motor Controller - Startup");
    SerialPort.SendLine("==============================================");
    SerialPort.SendLine("How many motors will you be testing? (Enter 1, 2, 3, or 4)");

    char response = 0;
    while (true) {
        if (SerialPort.AvailableForRead() > 0) {
            response = SerialPort.CharGet();

            // Accept '1' through '4' only
            if (response >= '1' && response <= '4') {
                MOTOR_COUNT = response - '0'; // Convert ASCII char to integer
                break;
            } else {
                SerialPort.Send("Invalid input '");
                SerialPort.Send(response);
                SerialPort.SendLine("'. Please enter 1, 2, 3, or 4.");
            }
        }
        Delay_ms(10);
    }

    // Populate the active motors[] array from the pool
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i] = motorPool[i];
    }

    SerialPort.Send("Motor count set to: ");
    SerialPort.SendLine(MOTOR_COUNT);
    SerialPort.Send("Active connectors: M0");
    for (int i = 1; i < MOTOR_COUNT; i++) {
        SerialPort.Send(", M");
        SerialPort.Send(i);
    }
    SerialPort.SendLine("");
}

// -----------------------------------------------------------------------
// Configure acceleration, velocity limits for each active motor
// -----------------------------------------------------------------------
void ConfigureMotors() {
    int32_t accelStepsPerSec_2  = (encoderResolution * accelerationLimit) / 60;
    int32_t velmaxStepsPerSec   = (encoderResolution * velocityMAX) / 60;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->AccelMax(accelStepsPerSec_2);
        motors[i]->VelMax(velmaxStepsPerSec);
        motors[i]->HlfbMode(MotorDriver::HLFB_MODE_STATIC);

        SerialPort.Send("  Motor M");
        SerialPort.Send(i);
        SerialPort.Send(" configured: accel=");
        SerialPort.Send(accelerationLimit);
        SerialPort.Send(" RPM/s^2, velMax=");
        SerialPort.Send(velocityMAX);
        SerialPort.SendLine(" RPM");
    }
}

// -----------------------------------------------------------------------
// Check if all active motors are stationary
// -----------------------------------------------------------------------
bool AreMotorsStationary() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i]->StatusReg().bit.StepsActive) {
            return false;
        }
    }
    return true;
}

// -----------------------------------------------------------------------
// Manual sync: step through each active motor individually
// -----------------------------------------------------------------------
void ManualSyncZero() {
    SerialPort.SendLine("\n--- SEQUENTIAL MANUAL SYNC START ---");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        SerialPort.Send("ACTION REQUIRED: Prepare Motor M");
        SerialPort.Send(i);
        SerialPort.SendLine(" (Lock rotor and click Ctrl+0 in MSP).");
        SerialPort.SendLine("Press 'Y' when this motor is ready to sync...");

        char response = 0;
        while (response != 'Y' && response != 'y') {
            if (SerialPort.AvailableForRead() > 0) {
                response = SerialPort.CharGet();
            }
        }

        motors[i]->EnableRequest(true);

        while (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Alert on Motor M");
                SerialPort.Send(i);
                SerialPort.SendLine("! Homing aborted.");
                return;
            }
            Delay_ms(1);
        }

        motors[i]->PositionRefSet(0);
        Delay_ms(10);
        motors[i]->EnableRequest(false);

        SerialPort.Send("Motor M");
        SerialPort.Send(i);
        SerialPort.SendLine(" SUCCESS: Enabled and Zeroed.\n");
    }

    SerialPort.SendLine("--- ALL MOTORS SYNCED ---");
    SerialPort.SendLine("CRITICAL: Remove all locking rods before commanding 'R' (Run).");
}

// -----------------------------------------------------------------------
// Set phase/angle offsets for all active motors
// -----------------------------------------------------------------------
void SetPhaseAngles(const int32_t angles[]) {
    if (!AreMotorsStationary()) {
        SerialPort.SendLine("ERROR: Cannot set phase while motors are running! Command 'S' first.");
        return;
    }
    SerialPort.SendLine("Applying phase offsets...");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        int32_t distance = (angles[i] * encoderResolution) / 360;
        motors[i]->Move(distance, StepGenerator::MOVE_TARGET_ABSOLUTE);
    }
    Delay_ms(1);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        while (!motors[i]->StepsComplete()) {
            Delay_ms(1);
        }
        Delay_ms(10);
    }
    SerialPort.SendLine("Phase lock established.");
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

// -----------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------
void setup() {
    // Set USB serial
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    SerialPort.PortOpen();

    uint32_t start = Milliseconds();
    while (!SerialPort) {
        if (Milliseconds() - start > 5000) {
            break; // Continue even if no terminal is connected
        }
    }
    Delay_ms(200); // Short settle time for the terminal to connect

    // Step 1: Ask how many motors
    PromptMotorCount();

    // Step 2: Apply global mode settings (must be done before per-motor config)
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Step 3: Configure only the active motors
    SerialPort.SendLine("Configuring active motors...");
    ConfigureMotors();

    SerialPort.SendLine("\nCommands: 'M'=Manual Homing, 'D'=Disable, 'E'=Enable, 'P'=Set Phase, 'R'=Start Rotation, 'S'=Stop, 'G'=Move to Targets");
    SerialPort.SendLine("----------------------------------------------");
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
