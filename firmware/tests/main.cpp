#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include <chrono>
#include <thread>
#include <functional>

using namespace std;

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    #include "../src/thermistors.h"
    #include "../src/board_id.h"
    #include "../src/hall.h"
    #include "../src/fault_handler.h"

    // variable declaration
    extern volatile int16_t fetTestCurrentMeasurement;
    extern volatile int16_t fetTestMeasurementCounter;
    extern volatile uint16_t adc_bus_current_offset;

    void TC4_TimerCallbackRegister( TC_TIMER_CALLBACK callback, uintptr_t context ) { }
    void ADC_CALIB_IBUS_ISR(ADC_STATUS status, uintptr_t context);

    void turnOn12VSwRail() { }
    void turnOn3v3SwRail() { }
    void turnOff12VSwRail() { }
    void turnOff3v3SwRail() { }

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

/*
  This helper class is used to simulate input pin reads, by specifying the sequence
  of values returned on every read.
*/
class PortPinsMock
{
    std::vector<bool> port_pin_reads{};
    int port_pin_idx = 0;

public:
    bool getNextPin()
    {
        if (!port_pin_reads.empty()&&(port_pin_idx < port_pin_reads.size()))
        {
            return port_pin_reads[port_pin_idx++];
        }
        else
        {
            return false;
        }
    }
    void addPinVals(int vals, int count)
    {

        for (int i=0;i<count;i++)
        {
            port_pin_reads.push_back(vals % 2);
            vals = vals/2;
        }
    }
    void reset()
    {
        port_pin_idx = 0;
        port_pin_reads.clear();
    }
};

// Add here the function to make STUB *void
// add single *.c file and *.h to create STUB and avoid the dependencies

/// This is an instance of the pin simulating class used for the Hall sensor faults.
PortPinsMock hallPins;

/// This function is called when the PORT_PinRead() macro is called in unit test.
bool ut_port_pin_read(PORT_PIN p)
{
    return hallPins.getNextPin();
}

bool x2cscope_update_flag = false;
bool pdec_hallstop_flag = false;

bool ut_variant_heavy_value = false;
bool ut_variant_lite_value = false;
bool ut_variant_heavy_get (void) { return ut_variant_heavy_value; }
bool ut_variant_lite_get (void) { return ut_variant_lite_value; }

bool ut_enter_low_power_mode = false;

/// This auxiliary variable stores the fault status when the mock X2CScope_Update
/// function is called inside the fault_handler code.
drive_fault_status_t    ut_drive_fault_status;

/// This auxiliary variable stores the fault status when the mock PDEC_HALLStop
/// function is called inside the fault_handler code. This second variable is
/// used to identify cases where both mock functions are called.
drive_fault_status_t    ut2_drive_fault_status;

/// Auxiliary variable that is returned when the Motor_Enable_Get macro is called
uint8_t ut_motor_enable_get = 1;

/// Auxiliary variable that is returned when the Motor_Direction_Get macro is called
uint8_t ut_motor_direction_get = 0;

/// This function object can be used to store a lambda that will be executed inside
/// the X2CScope_Update mock function to add custom code, e.g. updating variables
/// that will clear a fault condition, so the captive loop is exited.
std::function<void(void)> scope_update_cb = [](){};

void X2CScope_Init(void) { }
void X2CScope_Update(void) {
    x2cscope_update_flag = true;
    ut_drive_fault_status = drive_fault_status;
    scope_update_cb();
}

void X2CScope_Communicate(void) {  }

// managed by fault_handler.h these are "STUB" functions
uint32_t TC0_Timer32bitCounterGet( void ) { return 0; }
void TC2_CaptureStart( void ) { }
void TC4_TimerStart(void) { }
void TC4_TimerStop(void) {  }
void PDEC_HALLStop(void)
{
    pdec_hallstop_flag = true;
    ut2_drive_fault_status = drive_fault_status;
}

void fh_ShortedFetISRTasks(void) {  }

void TCC1_PWMStop(void) {  }
bool TCC1_PWMPatternSet(uint8_t pattern_enable, uint8_t pattern_output) { return 0; }
void TCC1_PWMForceUpdate(void) {  }
void TCC1_PWMStart(void) {  }

void TC0_TimerStart(void) {  }
void TC0_TimerStop(void) {  }
void TC0_Timer32bitCounterSet( uint32_t count ) {  }
uint32_t TC2_Capture32bitChannel0Get( void ) {return 0;}

void PDEC_HALLStart(void) {  }
void PDEC_HALLCallbackRegister(PDEC_HALL_CALLBACK callback, uintptr_t context) {  }
uint8_t PDEC_HALLPatternGet(void) { return 0; }

void ADC0_Disable(void) { }
void ADC0_Enable(void) { }
void ADC0_ChannelSelect(ADC_POSINPUT positiveInput, ADC_NEGINPUT negativeInput) { }
void ADC0_CallbackRegister( ADC_CALLBACK callback, uintptr_t context ) { }
uint16_t ADC0_ConversionResultGet(void) { return 0; }
void ADC1_ChannelSelect( ADC_POSINPUT positiveInput, ADC_NEGINPUT negativeInput ) { }
uint16_t ADC1_ConversionResultGet( void ) { return 0; }
void ADC0_InterruptsClear (ADC_STATUS interruptMask) { }
void ADC0_ConversionStart (void) { }

uint16_t thermistorGetFetTemp1(uint16_t AdcVal) { return 0; }
uint16_t thermistorGetFetTemp2(uint16_t AdcVal) { return 0; }

void WDT_Clear(void) { }
void WDT_Enable( void ) { }
void WDT_Disable( void ) { }

void PM_StandbyModeEnter( void ) { ut_enter_low_power_mode = true;}

void mcLib_ClarkeTransform(mcParam_ABC *abcParam, mcParam_AlphaBeta *alphabetaParam) { }
void mcLib_ParkTransform(mcParam_AlphaBeta *alphabetaParam , mcParam_SinCos *scParam, mcParam_DQ *dqParam) { }
void mcLib_InitPI( mcParam_PIController *pParam) { }
void mcLib_CalcPI( mcParam_PIController *pParam) { }
void mcLib_SinCosGen(mcParam_SinCos *scParam) { }
void mcLib_InvParkTransform(mcParam_DQ *dqParam, mcParam_SinCos *scParam,mcParam_AlphaBeta *alphabetaParam) { }
void mcLib_SVPWMGen(mcParam_AlphaBeta *alphabetaParam, mcParam_SVPWM *svParam) { }


/*  Check Temperature Failure for FET1
    Triggering default setting a temperature above the TEMPERATURE_MAX_THRESHOLD
    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update()

    The funtion X2CScope_Update() is used to avoid the infinite loop by setting
    the temperature inside valid range (through the scope_update_cb).
*/
TEST_CASE( "FET_1 Temperature Failure", "[fet_temp][faults]" )
{
    drive_fault_status.faultCode = NO_FAULT;
    drive_fault_status.faultCount = 0;

    // set the temperature above the TEMPERATURE_MAX_THRESHOLD
    fetTemp1 = FET_TEMPERATURE_MAX_THRESHOLD_DEGC + 1.0;

    // code executed inside X2CScope_Update() function
    scope_update_cb =
        [](){
            fetTemp1 = FET_TEMPERATURE_RECOVERY_DEGC-1.0;
        };

    // Function under test
    fh_FetTemperatureTest();    // wait to execute X2CScope_Update()

    CHECK( drive_fault_status.faultCount > 0); // to check if the counter is increased
    CHECK( ut_drive_fault_status.faultCode == FET_TEMPERATURE_FAULT);

    // to check is returning to IDLE and RECOVERY
    CHECK( drive_fault_status.faultCode == IDLE );
}


/*  Check Temperature Failure for FET2
    Triggering default setting a temperature above the TEMPERATURE_MAX_THRESHOLD
    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update()

    The funtion X2CScope_Update() is used to avoid the infinite loop by setting
    the temperature inside valid range (through the scope_update_cb).
*/
TEST_CASE( "FET_2 Temperature Failure", "[fet_temp][faults]" )
{
    drive_fault_status.faultCode = NO_FAULT;
    drive_fault_status.faultCount = 0;

    // set the temperature above the TEMPERATURE_MAX_THRESHOLD
    fetTemp2 = FET_TEMPERATURE_MAX_THRESHOLD_DEGC + 1.0;

    // code executed inside X2CScope_Update() function
    scope_update_cb =
        [](){
            fetTemp2 = FET_TEMPERATURE_RECOVERY_DEGC-1.0;
        };

    // Function under test
    fh_FetTemperatureTest();    // wait to execute X2CScope_Update()

    CHECK( drive_fault_status.faultCount > 0); // to check if the counter is increased
    CHECK( ut_drive_fault_status.faultCode == FET_TEMPERATURE_FAULT);

    // to check is returning to IDLE and RECOVERY
    CHECK( drive_fault_status.faultCode == IDLE );
}


/*  Check Temperature Failure for FET1 and FET2
    Triggering default setting a temperature above the TEMPERATURE_MAX_THRESHOLD
    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update()

    The funtion X2CScope_Update() is used to avoid the infinite loop by setting
    the temperature inside valid range (through the scope_update_cb).
*/
TEST_CASE( "FET_1_2 Temperature Failure", "[fet_temp][faults]" )
{
    drive_fault_status.faultCode = NO_FAULT;
    drive_fault_status.faultCount = 0;

    // set the temperature above the TEMPERATURE_MAX_THRESHOLD
    fetTemp1 = FET_TEMPERATURE_MAX_THRESHOLD_DEGC + 1.0;
    fetTemp2 = FET_TEMPERATURE_MAX_THRESHOLD_DEGC + 1.0;

    // code executed inside X2CScope_Update() function
    scope_update_cb =
        [](){
            fetTemp1 = FET_TEMPERATURE_RECOVERY_DEGC-1.0;
            fetTemp2 = FET_TEMPERATURE_RECOVERY_DEGC-1.0;
        };

    // Function under test
    fh_FetTemperatureTest();    // wait to execute X2CScope_Update()

    CHECK( drive_fault_status.faultCount > 0); // to check if the counter is increased
    CHECK( ut_drive_fault_status.faultCode == FET_TEMPERATURE_FAULT);

    // to check is returning to IDLE and RECOVERY
    CHECK( drive_fault_status.faultCode == IDLE );
}


/*  Check Normal Temperature for FET1 and FET2
    Setting the temperature bellow TEMPERATURE_MAX_THRESHOLD so no fault should happen.
*/
TEST_CASE( "FET Temperature Inside Threshold Temperature", "[fet_temp][faults]" )
{
    drive_fault_status.faultCode = NO_FAULT;
    drive_fault_status.faultCount = 0;

    // set the temperature bellow the TEMPERATURE_MAX_THRESHOLD
    fetTemp1 = FET_TEMPERATURE_MAX_THRESHOLD_DEGC - 1.0;
    fetTemp2 = FET_TEMPERATURE_MAX_THRESHOLD_DEGC - 1.0;

    // Function under test
    fh_FetTemperatureTest();    // wait to execute X2CScope_Update()

    CHECK(drive_fault_status.faultCount == 0);  // to check if the counter is not increased
    CHECK( drive_fault_status.faultCode == NO_FAULT );
}


/*  Check Undervoltage Fault on system

    There are two entities to test: in the upper limit or above, and bellow limit.

    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update()

    The funtion X2CScope_Update() is used to avoid the infinite loop by setting
    the temperature inside valid range (through the scope_update_cb).
*/
TEST_CASE( "Undervoltage Fault", "[undervoltage][faults]" )
{
    drive_fault_status.faultCode = NO_FAULT;
    drive_fault_status.faultCount = 0;
    x2cscope_update_flag = false;
    ut_drive_fault_status.faultCode = NO_FAULT;

    // check if fault is not triggered for voltage above or upper threshold
    printf("\n UNDERVOLTAGE FAULT\n");
    busVoltage = DCBUS_UNDERVOLTAGE_THRESHOLD_VOLTS;
    printf("     busVoltage = %f\n", busVoltage);

    // code executed inside X2CScope_Update() function
    scope_update_cb =
        [](){
            busVoltage = DCBUS_UNDERVOLTAGE_RECOVERY_VOLTS + 2.0;
        };

    // Function under test
    fh_BusUnderVoltageTest();   // wait to execute X2CScope_Update()

    // check any fault condition not changed
    CHECK( drive_fault_status.faultCount == 0);
    CHECK( drive_fault_status.faultCode == NO_FAULT );

    // check if fault is triggered by voltage bellow threshold
    busVoltage = DCBUS_UNDERVOLTAGE_THRESHOLD_VOLTS-1.0;
    printf("     busVoltage = %f\n", busVoltage);

    // Function under test
    fh_BusUnderVoltageTest();   // wait to execute X2CScope_Update()

    CHECK( x2cscope_update_flag );  // this function was modified to avoid infinite while loop
    CHECK( drive_fault_status.faultCount > 0);

    // this is the request by Olivia R.
    printf("     BUS_UNDERVOLTAGE_FAULT\n");
    CHECK( ut_drive_fault_status.faultCode == BUS_UNDERVOLTAGE_FAULT );
}


/*  Check ID Resistor Fault on system

    Due the complexity of possible combinations: motors and controllers. The if
    statement; catch all non allowed matches, raising the flag for ID_RESISTOR_FAULT

    There are two tables to use, because there is a flag error condition: adc_motor_id_error_f
    one table is to be use when the flag is false/no-set, and other when is true/set.

*/
TEST_CASE( "ID Resistor Fault", "[id_resistor][faults]" )
{
                                        // Truth table when adc_motor_id_error_f = false
    std::vector<std::vector<int>> invalid_combination_table_false{{true,true,true,true},
                                                                  {true,false,true,true},
                                                                  {true,false,false,true},
                                                                  {true,true,true,false}};

                                        // Truth table when adc_motor_id_error_f = true
    std::vector<std::vector<int>> invalid_combination_table_true {{true,true,true,true},
                                                                  {true,true,true,true},
                                                                  {true,false,true,true},
                                                                  {true,true,true,true}};

    // for adc_motor
    drive_fault_status.faultCode = NO_FAULT;
    ut2_drive_fault_status.faultCode = NO_FAULT;
    drive_fault_status.faultCount = 0;
    adc_motor_id_error_f=false;
    // for Board
    for( Board_ID_num=INVALID; Board_ID_num<MAX_MOTOR_NUM; Board_ID_num++)
    {
        // for Motor
        for ( Motor_ID_num=INVALID; Motor_ID_num<MAX_MOTOR_NUM; Motor_ID_num++)
        {
            pdec_hallstop_flag = false;
            bi_check_id_error();

            if(pdec_hallstop_flag == false )
            {
                printf("Motor_ID_num=%u, Board_ID_num=%u\n", Motor_ID_num, Board_ID_num);
            }
            CHECK( pdec_hallstop_flag == invalid_combination_table_false[Motor_ID_num][Board_ID_num] );  // this function was modified to avoid infinite while loop
            CHECK( ut2_drive_fault_status.faultCode == ID_RESISTOR_FAULT);
        } // end for Motor
    } // end for Board

    // for adc_motor
    drive_fault_status.faultCode = NO_FAULT;
    ut2_drive_fault_status.faultCode = NO_FAULT;
    drive_fault_status.faultCount = 0;
    adc_motor_id_error_f=true;
    // for Board
    for( Board_ID_num=INVALID; Board_ID_num<MAX_MOTOR_NUM; Board_ID_num++)
    {
        // for Motor
        for ( Motor_ID_num=INVALID; Motor_ID_num<MAX_MOTOR_NUM; Motor_ID_num++)
        {
            pdec_hallstop_flag = false;
            bi_check_id_error();

            if(pdec_hallstop_flag == false )
            {
                printf("Motor_ID_num=%u, Board_ID_num=%u\n", Motor_ID_num, Board_ID_num);
            }
            CHECK( pdec_hallstop_flag == invalid_combination_table_true[Motor_ID_num][Board_ID_num] );  // this function was modified to avoid infinite while loop
            CHECK( ut2_drive_fault_status.faultCode == ID_RESISTOR_FAULT);
        } // end for Motor
    } // end for Board
}


/*  Check Motor_ID and Board_ID
    Only a few variants can be allowed.

    ut_variant_heavy_value is a MOCK used instead of function Variant_Heavy_Get()
    ut_variant_lite_value  is a MOCK used instead of function Variant_Lite_Get()
*/
TEST_CASE( "Motor_ID and Board_ID", "[board_id][motor_id][faults]" )
{

    ut_variant_heavy_value = true;
    ut_variant_lite_value = false;
    // Function under test
    bi_determine_board_id();
    CHECK( Board_ID_num == SPARTAN_HEAVY );

    ut_variant_heavy_value = false;
    ut_variant_lite_value = true;
    // Function under test
    bi_determine_board_id();
    CHECK( Board_ID_num == SPARTAN_LITE );

    ut_variant_heavy_value = false;
    ut_variant_lite_value = false;
    // Function under test
    bi_determine_board_id();
    CHECK( Board_ID_num == INVALID );

    ut_variant_heavy_value = true;
    ut_variant_lite_value = true;
    // Function under test
    bi_determine_board_id();
    CHECK( Board_ID_num == INVALID );
}


extern uint16_t calibration_sample_count;
extern uint32_t adc_1_sum;

/*  Check OpAmp Fault on system

    The current offset readings from ADC compare the measurement between range of offset values:
    PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS < adc_0_offset < PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS
    PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS < adc_1_offset < PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS
    BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS < adc_bus_current_offset < BUS_CURNT_OFFSET_MIN_THRESHOLD_COUNTS
    if adc_0_offset, or adc_1_offset, adc_bus_current_offset are inside of these boundaries,
    the routine raises the flag OPAMP_FAULT.
*/
TEST_CASE( "OpAmp Fault", "[opamp][faults]" )
{
    ADC_STATUS status{};
    uintptr_t context = 0;

    // set values inside of boundaries
    adc_0_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_1_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // set the value outside of boundaries
    adc_0_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS -1.0;

    calibration_sample_count = 4097;
    
    // auxiliary condition
    adc_1_sum = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    adc_1_sum = adc_1_sum<<12;

    // Function under test
    ADC_CALIB_IBUS_ISR(status, context);

    // Verify the fault is present
    CHECK( ut2_drive_fault_status.faultCode == OPAMP_FAULT );

    // set values inside of boundaries
    adc_0_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_1_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // set the value outside of boundaries
    adc_1_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS -1.0;
    
    // auxiliary condition
    adc_1_sum = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    adc_1_sum = adc_1_sum<<12;

    // Function under test
    ADC_CALIB_IBUS_ISR(status, context);

    // Verify the fault is present
    CHECK( ut2_drive_fault_status.faultCode == OPAMP_FAULT );

    // set values inside of boundaries
    adc_0_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_1_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // set the value outside of boundaries
    adc_0_offset = PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS + 1.0;

    // auxiliary condition
    adc_1_sum = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    adc_1_sum = adc_1_sum<<12;

    // Function under test
    ADC_CALIB_IBUS_ISR(status, context);

    // Verify the fault is present
    CHECK( ut2_drive_fault_status.faultCode == OPAMP_FAULT );

    // set values inside of boundaries
    adc_0_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_1_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // set the value outside of boundaries
    adc_1_offset = PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS + 1.0;

    // auxiliary condition
    adc_1_sum = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    adc_1_sum = adc_1_sum<<12;

    // Function under test
    ADC_CALIB_IBUS_ISR(status, context);

    // Verify the fault is present
    CHECK( ut2_drive_fault_status.faultCode == OPAMP_FAULT );

    // set values inside of boundaries
    adc_0_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_1_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // set the value outside of boundaries
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS + 1.0;

    // auxiliary condition
    adc_1_sum = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS + 1.0;
    adc_1_sum = adc_1_sum<<12;

    // Function under test
    ADC_CALIB_IBUS_ISR(status, context);

    // Verify the fault is present
    CHECK( ut2_drive_fault_status.faultCode == OPAMP_FAULT );

    // set values inside of boundaries
    adc_0_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_1_offset = PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS + 1.0;
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // set the value outside of boundaries
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS + 1.0;

    // auxiliary condition
    adc_1_sum = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS + 1.0;
    adc_1_sum = adc_1_sum<<12;

    // Function under test
    ADC_CALIB_IBUS_ISR(status, context);

    // Verify the fault is present
    CHECK( ut2_drive_fault_status.faultCode == OPAMP_FAULT );

}

/*  Check OpAmp No Fault on system

    The current offset readings from ADC compare the measurement between range of offset values:
    PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS < adc_0_offset < PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS
    PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS < adc_1_offset < PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS
    BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS < adc_bus_current_offset < BUS_CURNT_OFFSET_MIN_THRESHOLD_COUNTS
    if adc_0_offset, or adc_1_offset, adc_bus_current_offset are inside of boundaries,
    the program continue reading, otherwise; raises the flag for OPAMP_FAULT.
    Here the no fault result is checked.
*/
TEST_CASE( "OpAmp NO Fault", "[opamp][faults]" )
{
    ADC_STATUS status{};
    uintptr_t context = 0;

    // setting values below max thresholds
    adc_0_offset = PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1;
    adc_1_offset = PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1;
    adc_bus_current_offset = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    calibration_sample_count = 4097;
    adc_1_sum = BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS - 1.0;
    adc_1_sum = adc_1_sum<<12;

    // Function under test
    ADC_CALIB_IBUS_ISR(status, context);

    // Verify no faults
    CHECK( ut2_drive_fault_status.faultCode == NO_FAULT );

}


/*  Check FET High Side Shorted Fault for U phase
    Triggering a the fault with a Current ADC reading above the SHORTED_FET_CURRENT_THRESHOLD_COUNTS
    threshold.
    ut_drive_fault_status is used to store the intermmediate condition due is updated into the
    PDEC_HALLStop(), this function is used in this case, because there fault handling code consists
    of an infinite loop, that is being disabled to allow the unit test keep going, and
    this prevents the same X2CScope_Update() function to be reused in this case.
*/
TEST_CASE( "FET Shorted Fault High side U phase", "[fet][faults]" )
{
    adc_0_offset = 1;
    adc_1_offset = 1;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // A secondary execution thread is created to exercise the internal logic inside the function
    // that expects the intervention of an ADC interrupt.
    std::thread async_task([]() {
        using namespace std::this_thread;
        auto constexpr delay = std::chrono::milliseconds(30);

        // this is where the current value above the threshold is set
        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS + 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 7;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_1_offset;
        fetTestMeasurementCounter = 1;
        sleep_for(delay);

        for (int i=0;i<4;i++)
        {
            fetTestMeasurementCounter = 2;
            sleep_for(delay);
        }

    });

    fh_shortedFetTest();

    async_task.join();

    CHECK( pdec_hallstop_flag == true );
    CHECK( drive_fault_status.faultCount == 1);
    CHECK( ut2_drive_fault_status.faultCode == HIGH_SIDE_FET_FAULT );

}


/*  Check FET High Side Shorted Fault for V/W phases
    Triggering a the fault with a Current ADC reading above the SHORTED_FET_CURRENT_THRESHOLD_COUNTS
    threshold.
    ut_drive_fault_status is used to store the intermmediate condition due is updated into the
    PDEC_HALLStop(), this function is used in this case, because there fault handling code consists
    of an infinite loop, that is being disabled to allow the unit test keep going, and
    this prevents the same X2CScope_Update() function to be reused in this case.
*/
TEST_CASE( "FET Shorted Fault High side V/W phase", "[fet][faults]" )
{
    adc_0_offset = 1;
    adc_1_offset = 1;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // A secondary execution thread is created to exercise the internal logic inside the function
    // that expects the intervention of an ADC interrupt.
    std::thread async_task([]() {
        using namespace std::this_thread;
        auto constexpr delay = std::chrono::milliseconds(30);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 7;
        sleep_for(delay);

        // this is where the current value above the threshold is set
        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS + 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_1_offset;
        fetTestMeasurementCounter = 1;
        sleep_for(delay);

        for (int i=0;i<4;i++)
        {
            fetTestMeasurementCounter = 2;
            sleep_for(delay);
        }

    });

    fh_shortedFetTest();

    async_task.join();

    CHECK( pdec_hallstop_flag == true );
    CHECK( drive_fault_status.faultCount == 1);
    CHECK( ut2_drive_fault_status.faultCode == HIGH_SIDE_FET_FAULT );

}


/*  Check FET Low Side Shorted Fault for U phase
    Triggering a the fault with a Current ADC reading above the SHORTED_FET_CURRENT_THRESHOLD_COUNTS
    threshold.
    ut_drive_fault_status is used to store the intermmediate condition due is updated into the
    PDEC_HALLStop(), this function is used in this case, because there fault handling code consists
    of an infinite loop, that is being disabled to allow the unit test keep going, and
    this prevents the same X2CScope_Update() function to be reused in this case.
*/
TEST_CASE( "FET Shorted Fault Low side U phase", "[fet][faults]" )
{
    adc_0_offset = 1;
    adc_1_offset = 1;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // A secondary execution thread is created to exercise the internal logic inside the function
    // that expects the intervention of an ADC interrupt.
    std::thread async_task([]() {
        using namespace std::this_thread;
        auto constexpr delay = std::chrono::milliseconds(30);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 7;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        // this is where the current value above the threshold is set
        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS + 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_1_offset;
        fetTestMeasurementCounter = 1;
        sleep_for(delay);

        for (int i=0;i<4;i++)
        {
            fetTestMeasurementCounter = 2;
            sleep_for(delay);
        }

    });

    fh_shortedFetTest();

    async_task.join();

    CHECK( pdec_hallstop_flag == true );
    CHECK( drive_fault_status.faultCount == 1);
    CHECK( ut2_drive_fault_status.faultCode == LOW_SIDE_FET_FAULT );

}


/*  Check FET Low Side Shorted Fault for V/W phases
    Triggering a the fault with a Current ADC reading rate of change larger than 5 ADC counts.
    ut_drive_fault_status is used to store the intermmediate condition due is updated into the
    PDEC_HALLStop(), this function is used in this case, because there fault handling code consists
    of an infinite loop, that is being disabled to allow the unit test keep going, and
    this prevents the same X2CScope_Update() function to be reused in this case.
*/
TEST_CASE( "FET Shorted Fault Low side V/W phase", "[fet][faults]" )
{
    adc_0_offset = 1;
    adc_1_offset = 1;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // A secondary execution thread is created to exercise the internal logic inside the function
    // that expects the intervention of an ADC interrupt.
    std::thread async_task([]() {
        using namespace std::this_thread;
        auto constexpr delay = std::chrono::milliseconds(30);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 7;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestMeasurementCounter = 1;
        sleep_for(delay);

        for (int i=0;i<4;i++)
        {
            // in this case a minimal change in the current value is expected after each reading
            fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 6.0*i) + adc_1_offset;
            sleep_for(delay);
            fetTestMeasurementCounter = 2;
            sleep_for(delay);
        }

    });

    fh_shortedFetTest();

    async_task.join();

    CHECK( pdec_hallstop_flag == true );
    CHECK( drive_fault_status.faultCount == 1);
    CHECK( ut2_drive_fault_status.faultCode == LOW_SIDE_FET_FAULT );

}

/*  Check for no FET Shorted Faults when all readings are in range.
    Make sure no faults are triggered when Current ADC readings are inside the valid threhold, and
    the rate of change is lower or equal than 5 ADC counts.
    ut_drive_fault_status is used to store the intermmediate condition due is updated into the
    PDEC_HALLStop(), this function is used in this case, because there fault handling code consists
    of an infinite loop, that is being disabled to allow the unit test keep going, and
    this prevents the same X2CScope_Update() function to be reused in this case.
*/
TEST_CASE( "FET Shorted Fault NO faults", "[fet][faults]" )
{
    adc_0_offset = 1;
    adc_1_offset = 1;
    pdec_hallstop_flag = false;
    drive_fault_status.faultCount = 0;
    ut2_drive_fault_status.faultCode = NO_FAULT;

    // A secondary execution thread is created to exercise the internal logic inside the function
    // that expects the intervention of an ADC interrupt.
    std::thread async_task([]() {
        using namespace std::this_thread;
        auto constexpr delay = std::chrono::milliseconds(30);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 7;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 1.0) + adc_0_offset;
        sleep_for(delay);
        fetTestMeasurementCounter = 4;
        sleep_for(delay);

        fetTestMeasurementCounter = 1;
        sleep_for(delay);

        for (int i=0;i<4;i++)
        {
            fetTestCurrentMeasurement = (SHORTED_FET_CURRENT_THRESHOLD_COUNTS - 4.0*i) + adc_1_offset;
            sleep_for(delay);
            fetTestMeasurementCounter = 2;
            sleep_for(delay);
        }

    });

    fh_shortedFetTest();

    async_task.join();

    CHECK( pdec_hallstop_flag == false );
    CHECK( drive_fault_status.faultCount == 0);
    CHECK( ut2_drive_fault_status.faultCode == NO_FAULT );

}


/*  Check HALL Sensor fault

    The hall sensor fault is generated by an invalid HALL code (0 or 7) read from
    digital pins. The pin reading function was replaced by a mock version that returns
    specified values.

    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update()

    The function X2CScope_Update()  is used to avoid the infinite loop by setting
    the Hall value to a valid one (through the scope_update_cb).
*/
TEST_CASE( "HALL sensor init fault", "[hall][faults]" )
{
    drive_fault_status.faultCount = 0;
    drive_fault_status.faultCode = NO_FAULT;
    ut_drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    // testing invalid value 0
    hallPins.reset();
    hallPins.addPinVals(0,3); // value read inside hall_sensor_init()
    hallPins.addPinVals(0,3); // value read inside CheckIsHallValueValid() in the fault Handler

    // code executed inside X2CScope_Update() function
    scope_update_cb =
        []() {
            hallPins.reset();
            hallPins.addPinVals(1,3); // valid value to exit fault handler
        };

    // force set adc interrupt flag
    ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_RESRDY_Msk;

    // function under test
    hall_sensor_init();

    CHECK(ut_drive_fault_status.faultCount == 1);
    CHECK(ut_drive_fault_status.faultCode == HALL_SENSOR_FAULT);

    drive_fault_status.faultCount = 0;
    drive_fault_status.faultCode = NO_FAULT;
    ut_drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    // testing invalid value 7
    hallPins.reset();
    hallPins.addPinVals(7,3); // value read inside hall_sensor_init()
    hallPins.addPinVals(7,3); // value read inside CheckIsHallValueValid() in the fault Handler

    hall_sensor_init();

    CHECK(ut_drive_fault_status.faultCount == 1);
    CHECK(ut_drive_fault_status.faultCode == HALL_SENSOR_FAULT);

}


/*  Check HALL Sensor for No fault

    The hall sensor fault is generated by an invalid HALL code (0 or 7) read from
    digital pins. The pin reading function was replaced by a mock version that returns
    specified values.

    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update(). In this case this function is not expected to be called.
*/
TEST_CASE( "HALL sensor no fault", "[hall][faults]" )
{
    drive_fault_status.faultCount = 0;
    drive_fault_status.faultCode = NO_FAULT;
    ut_drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    // force set adc interrupt flag
    ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_RESRDY_Msk;

    // testing valid Hall values
    for (int i=1;i<7;i++)
    {
        hallPins.reset();
        hallPins.addPinVals(i,3); // value read inside hall_sensor_init()
        hallPins.addPinVals(0,3); // value read inside CheckIsHallValueValid() in the fault Handler

        // function under test
        hall_sensor_init();

        CHECK(ut_drive_fault_status.faultCount == 0);
        CHECK(ut_drive_fault_status.faultCode == NO_FAULT);
    }
}


extern motor_status_t mcApp_motorState;
extern mcParam_PIController mcApp_Speed_PIParam;
extern mcParam_DQ mcApp_I_DQParam;
extern uint16_t timer_stall_detect_count;


/*  Check Motor Stall fault

    For the motor stall fault to be triggered the following condition needs to be observed
    for an number of times (TIMER_STALL_DETECT_PERIOD):
    I_DQParam.q > MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO

    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update().

    To exit the capturing loop, the Motor_Enable_Get() macro must return 1, this is
    controlled using the ut_motor_enable_get variable.
*/
TEST_CASE( "Motor Stall fault", "[stall][faults]" )
{
    drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    Motor_ID_num = SPARTAN_HEAVY;

    // setting conditions to trigger fault
    mcApp_motorState.motorDirection = 0;
    mcApp_motorState.focStart = 1;
    mcApp_Speed_PIParam.qInMeas = STALL_DETECT_RPM_THRESHOLD - 1;
    mcApp_I_DQParam.q = (motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO) + 1;

    timer_stall_detect_count = TIMER_STALL_DETECT_PERIOD;

    ut_motor_enable_get = 0;

    // code executed inside X2CScope_Update() function
    scope_update_cb =
        [](){
            ut_motor_enable_get = 1;
        };

    // function under test
    mcApp_StallDetection();

    CHECK(drive_fault_status.faultCount == 1);
    CHECK(ut_drive_fault_status.faultCode == MOTOR_STALL);

    // motor direction 1

    drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    // setting conditions to trigger fault
    mcApp_motorState.motorDirection = 1;
    mcApp_motorState.focStart = 1;
    mcApp_Speed_PIParam.qInMeas = -STALL_DETECT_RPM_THRESHOLD + 1;
    mcApp_I_DQParam.q = -(motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO) - 1;

    timer_stall_detect_count = TIMER_STALL_DETECT_PERIOD;

    ut_motor_enable_get = 0;
    ut_motor_direction_get = 1;

    // function under test
    mcApp_StallDetection();

    CHECK(drive_fault_status.faultCount == 1);
    CHECK(ut_drive_fault_status.faultCode == MOTOR_STALL);

}


/*  Check Motor Stall NO fault (counts)

    For the motor stall fault to be triggered the following condition needs to be observed
    for an number of times (TIMER_STALL_DETECT_PERIOD):
    I_DQParam.q > MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO

    The fault should not happen if the expected count (TIMER_STALL_DETECT_PERIOD) is
    not reached.

    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update().
*/
TEST_CASE( "Motor Stall NO fault, counts", "[stall][faults]" )
{
    drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    Motor_ID_num = SPARTAN_HEAVY;

    // setting conditions to trigger fault
    mcApp_motorState.motorDirection = 0;
    mcApp_motorState.focStart = 1;
    mcApp_Speed_PIParam.qInMeas = STALL_DETECT_RPM_THRESHOLD - 1;
    mcApp_I_DQParam.q = (motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO) + 1;

    timer_stall_detect_count = TIMER_STALL_DETECT_PERIOD - 2;

    ut_motor_enable_get = 0;

    // code executed inside X2CScope_Update() function
    scope_update_cb =
        [](){
            ut_motor_enable_get = 1;
        };

    // function under test
    mcApp_StallDetection();

    CHECK(drive_fault_status.faultCount == 0);
    CHECK(ut_drive_fault_status.faultCode == NO_FAULT);

    // motor direction 1

    drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    // setting conditions to trigger fault
    mcApp_motorState.motorDirection = 1;
    mcApp_motorState.focStart = 1;
    mcApp_Speed_PIParam.qInMeas = -STALL_DETECT_RPM_THRESHOLD + 1;
    mcApp_I_DQParam.q = -(motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO) - 1;

    timer_stall_detect_count = TIMER_STALL_DETECT_PERIOD - 2;

    ut_motor_enable_get = 0;
    ut_motor_direction_get = 1;

    // function under test
    mcApp_StallDetection();

    CHECK(drive_fault_status.faultCount == 0);
    CHECK(ut_drive_fault_status.faultCode == NO_FAULT);

}


/*  Check Motor Stall NO fault (current)

    For the motor stall fault to be triggered the following condition needs to be observed
    for an number of times (TIMER_STALL_DETECT_PERIOD):
    I_DQParam.q > MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO

    The fault should not happen if the I_DQParam.q is not greater than the expected value.

    ut_drive_fault_status is used to store the intermmediate condition due is
    updated into X2CScope_Update().
*/
TEST_CASE( "Motor Stall NO fault, current", "[stall][faults]" )
{
    drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    Motor_ID_num = SPARTAN_HEAVY;

    // setting conditions to trigger fault
    mcApp_motorState.motorDirection = 0;
    mcApp_motorState.focStart = 1;
    mcApp_Speed_PIParam.qInMeas = STALL_DETECT_RPM_THRESHOLD - 1;
    mcApp_I_DQParam.q = (motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO);

    timer_stall_detect_count = TIMER_STALL_DETECT_PERIOD;

    ut_motor_enable_get = 0;

    // code executed inside X2CScope_Update() function
    scope_update_cb =
        [](){
            ut_motor_enable_get = 1;
        };

    // function under test
    mcApp_StallDetection();

    CHECK(drive_fault_status.faultCount == 0);
    CHECK(ut_drive_fault_status.faultCode == NO_FAULT);

    // motor direction 1

    drive_fault_status.faultCount = 0;
    ut_drive_fault_status.faultCode = NO_FAULT;

    // setting conditions to trigger fault
    mcApp_motorState.motorDirection = 1;
    mcApp_motorState.focStart = 1;
    mcApp_Speed_PIParam.qInMeas = -STALL_DETECT_RPM_THRESHOLD + 1;
    mcApp_I_DQParam.q = -(motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO);

    timer_stall_detect_count = TIMER_STALL_DETECT_PERIOD;

    ut_motor_enable_get = 0;
    ut_motor_direction_get = 1;

    // function under test
    mcApp_StallDetection();

    CHECK(drive_fault_status.faultCount == 0);
    CHECK(ut_drive_fault_status.faultCode == NO_FAULT);

}


/* Enter Low Power Mode

   To enter low power mode a minimum amount of time needs to pass under certain conditions.
   To measure this, a function  mcApp_decideLowPowerMode() is executed periodically and
   an internal counter is incremented until reaching the required count value:
     LOW_POWER_MODE_DELAY_TIME_COUNTS
   So, the test call the function this number of times to make the internal logic to reach
   the low power mode branch.
*/
TEST_CASE( "Enter Low Power mode", "[low power]" )
{
    // clear flag that indicates low power mode was entered
    ut_enter_low_power_mode = false;

    // conditions to allow low power mode
    ut_motor_enable_get = 1;
    mcApp_motorState.inverterRunning = 0;

    // increment low power mode counter to the limit before entering low power mode
    for (int i=0; i<=LOW_POWER_MODE_DELAY_TIME_COUNTS; i++)
    {
        // function under test
        mcApp_decideLowPowerMode();
    }

    // check flag to make sure the low power mode was not entered prematurely
    CHECK(ut_enter_low_power_mode == false);

    // execute one more time to actualy enter low power mode
    mcApp_decideLowPowerMode();

    // check the flag to confirm low power mode has been reached
    CHECK(ut_enter_low_power_mode == true);
}


/* Low Power Mode, Conditions not met

   To enter low power mode a minimum amount of time needs to pass under certain conditions.
   To measure this, a function  mcApp_decideLowPowerMode() is executed periodically and
   an internal counter is incremented until reaching the required count value:
     LOW_POWER_MODE_DELAY_TIME_COUNTS
   So, the test call the function this number of times to make the internal logic to reach
   the low power mode branch, but as the conditions are not correct, it should not enter.
*/
TEST_CASE( "Low Power mode, conditions not met", "[low power][negative]" )
{
    // clear flag that indicates low power mode was entered
    ut_enter_low_power_mode = false;

    // conditions to prevent low power mode
    ut_motor_enable_get = 0;
    mcApp_motorState.inverterRunning = 0;

    // increment low power mode counter beyond the limit
    for (int i=0; i<=LOW_POWER_MODE_DELAY_TIME_COUNTS + 1; i++)
    {
        // function under test
        mcApp_decideLowPowerMode();
    }

    // check the flag to confirm low power mode has NOT been reached
    CHECK(ut_enter_low_power_mode == false);

    // clear flag that indicates low power mode was entered
    ut_enter_low_power_mode = false;

    // conditions to prevent low power mode
    ut_motor_enable_get = 1;
    mcApp_motorState.inverterRunning = 1;

    // increment low power mode counter beyond the limit
    for (int i=0; i<=LOW_POWER_MODE_DELAY_TIME_COUNTS + 1; i++)
    {
        // function under test
        mcApp_decideLowPowerMode();
    }

    // check the flag to confirm low power mode has NOT been reached
    CHECK(ut_enter_low_power_mode == false);
}
