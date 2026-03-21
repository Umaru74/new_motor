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

// Run-time motor count
int MOTOR_COUNT = 0;

#define SerialPort ConnectorUsb

// --- Full pool of all 4 connectors ---
MotorDriver *motorPool[] = {&motor1, &motor2, &motor3, &motor4};

// --- Active motors array (pointer into motorPool, sized at runtime) ---
MotorDriver *motors[4] = {nullptr, nullptr, nullptr, nullptr};

// Hardware Constants
const int32_t encoderResolution = 8000; // Counts per revolution (set to match MSP) , 8000 resolution CAP for step/dir mode
const int32_t velocityRPM = 100; // Target RPM 
const int32_t velocityMAX = 100; // max RPM
const uint32_t accelerationLimit = 50; // RPM / sec^2
const uint32_t decelerationLimit = 50; // RPM / sec^2
const int32_t baudRate = 115200; // Communication speed for USB, baudRate needs to match with the host computer, monitor
bool motorsRunning = false;

// Define phase offsets in degree, corresponding to motor 0, 1 (+ve goes CCW , -ve goes CW)
const float offsets[] = {0, 90, 180, 270};

// Define specific motor location to move to (such as when to close the valve for emergency)
// const float homes[] = {0, 0, 0, 0}; // in degree with respect to the motor 0, initial location

// Forward declarations
void PrintAlerts();
void DisableAllMotors();
void EnableAllMotors();
void StopAll();

// Define system states
enum SystemState {
    STATE_IDLE,        // Motors enabled, not yet synced
    STATE_SYNCED,      // Manual sync done, ready for phase
    STATE_PHASE_SET,   // Phase angles applied, ready to run
    STATE_RUNNING,     // Motors spinning
    STATE_HOMED        // Motors homed, back to idle
};

SystemState systemState = STATE_IDLE;

// -----------------------------------------------------------------------
// Startup: Ask the user how many motors to activate (1-4), used in setup()
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
// Configure acceleration, velocity limits for each active motor, HLFB mode
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
bool AllMotorsStationary() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i]->StatusReg().bit.ReadyState == MotorDriver::MOTOR_DISABLED) {
            continue; // Disabled = stationary by definition
        }
        if (!motors[i]->StepsComplete() || 
            motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            return false;
        }
    }
    return true;
}

// -----------------------------------------------------------------------
// Manual sync: step through each active motor individually
// -----------------------------------------------------------------------
void ManualSyncZero() {
    if (systemState != STATE_IDLE && systemState != STATE_HOMED){
        SerialPort.SendLine("ERROR: Must be in IDLE or HOMED state to sync");
        return;
    }
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

        motors[i]->EnableRequest(true); // moves to the original home position set on MSP ?

        while (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Alert on Motor M");
                SerialPort.Send(i);
                SerialPort.SendLine("! Homing aborted.");
                PrintAlerts();
                return;
            }
            Delay_ms(1);
        }

        motors[i]->PositionRefSet(0);
        Delay_ms(10);
        //motors[i]->EnableRequest(false);

        SerialPort.Send("Motor M");
        SerialPort.Send(i);
        SerialPort.SendLine(" SUCCESS: Enabled and Zeroed.\n");
    }
    systemState = STATE_SYNCED;
    SerialPort.SendLine("--- ALL MOTORS SYNCED ---");
    SerialPort.SendLine("CRITICAL: Remove all locking rods before commanding 'R' (Run).");
}

// -----------------------------------------------------------------------
// Set phase/angle offsets for all active motors, positionRefSet(0) 
// -----------------------------------------------------------------------
void SetPhaseAngles(const float angles[]) {
    if (systemState != STATE_SYNCED) {
        SerialPort.SendLine("ERROR: Must run Manual Sync 'M' before setting phase.");
        return;
    }

    if (!AllMotorsStationary()) {
        SerialPort.SendLine("ERROR: Cannot set phase while motors are running! Command 'S' first.");
        return;
    }
    SerialPort.SendLine("Applying phase offsets...");

    // Command all motors simultaneously
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // Use float math before casting to int32_t at the end
        int32_t distance = (int32_t)((angles[i] * encoderResolution) / 360.0f);

        SerialPort.Send("Motor ");
        SerialPort.Send(i);
        SerialPort.Send(" target steps: ");
        SerialPort.Send(distance);
        SerialPort.Send(" pulse steps / ");
        SerialPort.Send(angles[i]);
        SerialPort.SendLine(" degrees");

        motors[i]->Move(distance, StepGenerator::MOVE_TARGET_ABSOLUTE);
    }
    while (!AllMotorsStationary()){
        Delay_ms(1);
    }
    systemState = STATE_PHASE_SET;
    SerialPort.SendLine("Phase lock established.");
}

// function: Start rotation at RPM
void StartRotation(int32_t rpm){
    if (systemState != STATE_PHASE_SET && systemState != STATE_SYNCED) {
        SerialPort.SendLine("ERROR: Must sync 'M' before running.");
        return;
    }
    // Convert RPM to steps/sec: (RPM/60)*8000
    int32_t velocityStepsPerSec = (encoderResolution * rpm ) / 60;

    SerialPort.Send("Starting synchronized rotation at RPM: ");
    SerialPort.SendLine(rpm);

    // Command the velocity move to all four motors synchronously
    for (int i = 0; i < MOTOR_COUNT; i++){
         motors[i]-> MoveVelocity(velocityStepsPerSec);
    }
    // set running state before ramp loop so StopAll() works if alert fires 
    motorsRunning = true;
    systemState = STATE_RUNNING;

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
                PrintAlerts();
                StopAll();
                return;
            }

            // Check if motor reached cruise velocity
            if (!motors[i]->CruiseVelocityReached()) {
                allAtSpeed = false;
            }
        }

        // Time Out Protection: 15s
        if (Milliseconds() - startTime > 30000) {
            SerialPort.SendLine("Error: Motors failed to reach target speed!");
            StopAll();
            return;
        }

        Delay_ms(1); // reduce CPU usage
    }
    SerialPort.Send("All motors have hit the desired RPM:");
    SerialPort.SendLine(rpm);
}

// function: Stop all motors smoothly
void StopAll() {
    if (systemState != STATE_RUNNING) {
        SerialPort.SendLine("ERROR: Motors are not running.");
        return;
    }
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
                PrintAlerts();
                systemState = STATE_IDLE;
                motorsRunning = false;
                return;
            }
            // Check if motor stops
            if (!motors[i]->StepsComplete()) {
                allStopped = false;
            }
        }
        Delay_ms(1);
    }
    systemState = STATE_IDLE;
    motorsRunning = false;
    SerialPort.SendLine("All motors have come to a stop.");
}

// Function: Move the motors to the homing position set by PositionRefSet(0)
void HomeAllMotors() {
    if (systemState != STATE_IDLE) {
        SerialPort.SendLine("ERROR: Stop motors before homing.");
        return;
    }
    SerialPort.SendLine("Homing all motors via MSP...");

    // Disable first to trigger MSP homing on re-enable
    DisableAllMotors();
    Delay_ms(500); // Let motors fully de-energize

    // Re-enable triggers MSP homing routine (CW to home)
    EnableAllMotors();

    // Now software zero matches physical home
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->PositionRefSet(0);
    }
    systemState = STATE_HOMED;
    SerialPort.SendLine("All motors homed and zeroed.");
}


//-------------------------------
// EnableAllMotors(): function to enable all the motors, timeout after 10seconds
//-------------------------------
void EnableAllMotors() {
    SerialPort.SendLine("Enabling all motors...");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->EnableRequest(true);
    }

    SerialPort.SendLine("Waiting for HLFB...");

    uint32_t startTime = Milliseconds();
    uint32_t lastStatusTime = Milliseconds();

    // Wait for ALL motors in parallel
    while (!AllMotorsStationary()) {
        // Print status every 1000ms
        if (Milliseconds() - lastStatusTime > 1000) {
            SerialPort.SendLine("Still waiting for HLFB...");
            lastStatusTime = Milliseconds(); // Reset status timer
        }

        // Timeout after 10 seconds
        if (Milliseconds() - startTime > 10000) {
            SerialPort.SendLine("ERROR: Timeout waiting for motors to enable.");
            return;
        }

        // Check for alerts on any motor
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("ERROR: Alert on motor ");
                SerialPort.SendLine(i);
                return;
            }
        }

        Delay_ms(1);
    }

    SerialPort.SendLine("All motors enabled and ready.");
}

void DisableAllMotors() {
    SerialPort.SendLine("Disabling all motors...");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!motors[i]->StepsComplete() || motors[i]->VelocityRefCommanded() != 0) {
            SerialPort.SendLine("ERROR: Motion detected! Cannot disable motors while moving.");
            SerialPort.SendLine("Stop the moving motors first before disabling.");
            return;
        }
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->EnableRequest(false);
    }

    // Wait for hardware confirmation with timeout
    uint32_t startTime = Milliseconds();
    bool allDisabled = false;

    while (!allDisabled) {
        if (Milliseconds() - startTime > 2000) {
            SerialPort.SendLine("ERROR: Timeout waiting for motors to disable.");
            return;
        }

        allDisabled = true;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (motors[i]->StatusReg().bit.ReadyState != MotorDriver::MOTOR_DISABLED) {
                allDisabled = false;
            }
        }
        Delay_ms(1);
    }

    SerialPort.SendLine("All motors disabled and de-energized.");
}

// -----------------------------------------------------------------------
// HandleAlerts(): Clear motor faults by cycling enable and clearing registers
// -----------------------------------------------------------------------
void HandleAlerts() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i]->AlertReg().bit.MotorFaulted) {
            SerialPort.Send("Fault detected on motor ");
            SerialPort.SendLine(i);

            // Cycle enable to clear physical fault
            motors[i]->EnableRequest(false);
            Delay_ms(100);  // Give motor time to fully de-energize
            motors[i]->EnableRequest(true);

            // Wait for motor to recover before clearing alerts
            uint32_t startTime = Milliseconds();
            while (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
                if (Milliseconds() - startTime > 5000) {
                    SerialPort.Send("ERROR: Motor ");
                    SerialPort.Send(i);
                    SerialPort.SendLine(" did not recover after fault clear.");
                    return;
                }
                Delay_ms(1);
            }

            // Now safe to clear alerts
            motors[i]->ClearAlerts();

            // Verify alerts actually cleared
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("WARNING: Motor ");
                SerialPort.Send(i);
                SerialPort.SendLine(" still has alerts after clear attempt.");
            } else {
                SerialPort.Send("Motor ");
                SerialPort.Send(i);
                SerialPort.SendLine(" fault cleared successfully.");
            }
        }
    }
}

// -----------------------------------------------------------------------
// PrintAlerts(): Prints all active alerts for diagnostic purposes
// -----------------------------------------------------------------------
void PrintAlerts() {
    bool anyAlerts = false;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i]->StatusReg().bit.AlertsPresent) {
            anyAlerts = true;
            SerialPort.Send("Alerts on Motor ");
            SerialPort.SendLine(i);

            if (motors[i]->AlertReg().bit.MotionCanceledInAlert) 
                SerialPort.SendLine("  - MotionCanceledInAlert");
            if (motors[i]->AlertReg().bit.MotionCanceledPositiveLimit) 
                SerialPort.SendLine("  - MotionCanceledPositiveLimit");
            if (motors[i]->AlertReg().bit.MotionCanceledNegativeLimit) 
                SerialPort.SendLine("  - MotionCanceledNegativeLimit");
            if (motors[i]->AlertReg().bit.MotionCanceledSensorEStop) 
                SerialPort.SendLine("  - MotionCanceledSensorEStop");
            if (motors[i]->AlertReg().bit.MotionCanceledMotorDisabled) 
                SerialPort.SendLine("  - MotionCanceledMotorDisabled");
            if (motors[i]->AlertReg().bit.MotorFaulted) 
                SerialPort.SendLine("  - MotorFaulted (Shutdown)");
        }
    }

    if (!anyAlerts) {
        SerialPort.SendLine("No alerts present on any motor.");
    }
}


// HLFB mode needs to be set for "In Range-Velocity" on MSP for each motor 
bool AllMotorsInRange(){
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
// setup(): Runs when the controller is powered up
// -----------------------------------------------------------------------
void setup() {
    // Set USB serial
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    SerialPort.PortOpen();

    uint32_t start = Milliseconds();
    uint32_t timeout = 5000;
    while (!SerialPort) {
        if (Milliseconds() - start > timeout) {
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

    SerialPort.SendLine("Commands: 'M'=Sync, 'P'=Phase, 'R'=Run, 'S'=Stop, 'G'=Home, 'E'=Enable, 'D'=Disable, 'A'=Alerts, 'H'=Handle Alerts, '?'=Status");
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
                HomeAllMotors(); 
                break;
            case 'A': case 'a':
                SerialPort.SendLine("----Print Alerts Command Initialized----");
                PrintAlerts();
                break;
            case 'H': case 'h':
                SerialPort.SendLine("----Handle Alerts Command Initialized----");
                HandleAlerts();
                break;    
            case '?':
                switch (systemState) {
                    case STATE_IDLE:      SerialPort.SendLine("State: IDLE - Command 'M' to sync"); break;
                    case STATE_SYNCED:    SerialPort.SendLine("State: SYNCED - Command 'P' to set phase"); break;
                    case STATE_PHASE_SET: SerialPort.SendLine("State: PHASE SET - Command 'R' to run"); break;
                    case STATE_RUNNING:   SerialPort.SendLine("State: RUNNING - Command 'S' to stop"); break;
                    case STATE_HOMED:     SerialPort.SendLine("State: HOMED - Command 'M' to sync"); break;
                }
                break;
            }
    }

    // 2: Continuous Monitoring (Only when motors are at the desired speed)
    if (!AllMotorsStationary() && motorsRunning){
        if (!AllMotorsInRange()){
            SerialPort.SendLine("CRITICAL: Synchronization lost or motor stalled!");
            PrintAlerts();
            StopAll(); // safety action, stop the motors when they are not synchronizing 
        }
    }
}
