/* ************************************************************************** */
/** HALL.C

  @Company
    Chamberlain Group Inc

  @File Name
    hall.c

  @Summary
    Source file to handle hall sensors

 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "hall.h"
#include "board_id.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

float tblHALLangle[8u] = {
    RL_DUMM,  /* 0: dummy */
    RL_FIVE_OVER_THREE_PI,  /* 1: 300 degree */
    RL_ONE_OVER_THREE_PI,  /* 2: 60 degree */
    0.0f,  /* 3: 0 degree */
    RL_PI,  /* 4: 180 degree */
    RL_FOUR_OVER_THREE_PI,  /* 5: 240 degree */
    RL_TWO_OVER_THREE_PI,  /* 6: 120 degree */
    RL_DUMM  /* 7: dummy */
};

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

static inline void dirHALLget(tagHALLdata * ptData)
{
    uint16_t tmp1;
    
    tmp1 = ptData->combHALLvalue;
    switch(tmp1){
        case RL_30_HALL_JUMP_CW:
        case RL_90_HALL_JUMP_CW:            
        case RL_150_HALL_JUMP_CW:            
        case RL_210_HALL_JUMP_CW:            
        case RL_270_HALL_JUMP_CW:            
        case RL_330_HALL_JUMP_CW:
            ptData->dir = 1;  /* Rotor angle is increasing. */
            break;
        case RL_30_HALL_JUMP_CCW:
        case RL_90_HALL_JUMP_CCW:
        case RL_150_HALL_JUMP_CCW:
        case RL_210_HALL_JUMP_CCW:
        case RL_270_HALL_JUMP_CCW:
        case RL_330_HALL_JUMP_CCW:
            ptData->dir = -1;  /* Rotor angle is decreasing. */        
            break;
        default:
            break;
    }
}

static inline void nextAngleHALLcal(tagHALLdata * ptData)
{
    float tmp;
    
    tmp = ptData->angleHALLjump + ((float)(ptData->dir) * RL_ONE_OVER_THREE_PI);
    
    if(RL_TWO_PI < tmp){
        ptData->nextAngleHALLjump = tmp - RL_TWO_PI;
    } else if(0.0f > tmp){
        ptData->nextAngleHALLjump = tmp + RL_TWO_PI;
    } else{
        ptData->nextAngleHALLjump = tmp;
    }
}

static inline float angleHALLjumpGet(tagHALLdata * ptData)
{
    uint16_t tmp1;
    float tmp2 = 0.0f;
    
    tmp1 = ptData->combHALLvalue;
    switch(tmp1){
        case RL_30_HALL_JUMP_CW:
        case RL_30_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_ONE_OVER_SIX_PI;
            break;
        case RL_90_HALL_JUMP_CW:
        case RL_90_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_ONE_OVER_TWO_PI;
            break;
        case RL_150_HALL_JUMP_CW:
        case RL_150_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_FIVE_OVER_SIX_PI;
            break;
        case RL_210_HALL_JUMP_CW:
        case RL_210_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_SEVEN_OVER_SIX_PI;
            break;
        case RL_270_HALL_JUMP_CW:
        case RL_270_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_THREE_OVER_TWO_PI;
            break;
        case RL_330_HALL_JUMP_CW:
        case RL_330_HALL_JUMP_CCW:
            (ptData->numEffectiveHALLjump)++;
            tmp2 = RL_ELEVEN_OVER_SIX_PI;
            break;
        default:
            ptData->numEffectiveHALLjump = 0u;  /* not an effective HALL jump */
            break;
    }
    
    if(0x1000u < ptData->numEffectiveHALLjump){
        ptData->numEffectiveHALLjump = 0x1000u;
    }
    
    return tmp2;
}

static inline void velocityCalOneJump(tagHALLdata * ptData)
{
    if(0u == ptData->tcHALLisr){
        return;
    }
    
    ptData->eleVelocity = ptData->dAngleHALLjump / (float)ptData->tcHALLisr * RL_TC0_FREQ;
}

static inline void velocityCalSixJump(tagHALLdata * ptData)
{
    if(0u == ptData->tcSum){
        return;
    }
    
    ptData->eleVelocity = (float)(ptData->dir) * RL_2PI_TC0_FREQ / (float)ptData->tcSum;  
}

static inline void velocityManipulate(tagHALLdata * ptData)
{
    /* Calculate absolute value. */
    if(0.0f > ptData->eleVelocity){
        ptData->absEleVel = -1.0f * ptData->eleVelocity;
    } else{
        ptData->absEleVel = ptData->eleVelocity;
    }   
}

static inline void PORT_HALLcodeGet(tagHALLdata * ptData)
{
    ptData->HALLa = PORT_PinRead(PORT_PIN_PA24);
    ptData->HALLb = PORT_PinRead(PORT_PIN_PA25);
    ptData->HALLc = PORT_PinRead(PORT_PIN_PB22);
    ptData->HALLvalue = ((ptData->HALLc << 2) | (ptData->HALLb << 1) | ptData->HALLa) & 0x7u;
}

void resetHALLdata(tagHALLdata * ptData)
{
    uint16_t cnt;
    tagState * ptStt = &(ptData->sttData);    
 
    NVIC_DisableIRQ(PDEC_OTHER_IRQn);
    
    for(cnt = 0u; cnt<TC_HALL_FIFO_LEN; cnt++){
        ptData->tcFIFO[cnt] = 0ul;
    }
    ptData->cntTCfifo = 0u;
    ptData->tcSum = 0u;
    PORT_HALLcodeGet(&HALLdata);
    ptData->angleHALL = tblHALLangle[ptData->HALLvalue];
    ptData->estimatedRotorAngle = ptData->angleHALL;
    ptData->dir = 0u;    
    ptData->numEffectiveHALLjump = 0u;
    ptData->tcHALLisr = 0lu;
    ptData->eleVelocity = 0.0f;
    ptData->offsetAngleBound = 0.0f;
    ptData->offsetAngle = 0.0f;
    ptData->eleVelocity = 0.0f;
    ptData->eleVelocityFil = 0.0f;
    TC0_TimerStop();  
    TC0_Timer32bitCounterSet(0lu);
    ptStt->state = STT_HALL_ISR_INI;
    ptStt->isSttChanged = 0u;
    ptStt->cnt32_1 = 0u;
    
    NVIC_EnableIRQ(PDEC_OTHER_IRQn);
}

static inline  void tcFIFOmaintain(tagHALLdata * ptData)
{
    uint16_t * cnt = &(ptData->cntTCfifo);
    
    ptData->tcSum += ptData->tcHALLisr;
    (*cnt)++;
    if(TC_HALL_FIFO_LEN <= *cnt){
        *cnt = 0u;
    }
    ptData->tcSum -= ptData->tcFIFO[*cnt];
    ptData->tcFIFO[*cnt] = ptData->tcHALLisr;
}

void latchHALLdata(tagHALLdata * ptData)
{
    ptData->angleHALLjumpLth = ptData->angleHALLjump;
    ptData->eleVelocityLth = ptData->eleVelocity;
    ptData->dirLth = ptData->dir;
    ptData->stateLth = ptData->sttData.state;
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

// Returns 1 if hall value is valid, returns 0 if hall value is invalid
uint8_t CheckIsHallValueValid(void)
{
    tagHALLdata HALLpremotiondata;
    PORT_HALLcodeGet(&HALLpremotiondata);
    uint8_t hallValue = HALLpremotiondata.HALLvalue;

    //invalid hall codes are 0 and 7 (1-6 are good)
    if ((0 == hallValue) || (7 == hallValue))
    {
        return 0;
    }
    else
    {
        return 1;
    }
    
    
}

inline static void HALL_jump_ISR (PDEC_HALL_STATUS status, uintptr_t context)
{
    uint32_t capture_val;
    float tmp1;
    tagState * ptStt = &(HALLdata.sttData);
#if 0
    HALLdata.tcHALLisr = TC0_Timer32bitCounterGet();
#else    
    capture_val = TC2_Capture32bitChannel0Get();
    HALLdata.tcHALLisr = capture_val;
#endif    
    /* To make sure MC_ISR reads TC0 counter value as after TC0 reset.
     * MC_ISR may read counter value as before TC0 reset, if without manually counter reset. */
    TC0_Timer32bitCounterSet(0u);  
    TC0_TimerStart();
    tcFIFOmaintain(&HALLdata);
    HALLdata.lastHALLvalue = HALLdata.HALLvalue;
#if 1  /* Result from PDEC maybe the last HALL value, not the current one. */
    HALLdata.HALLvalue = PDEC_HALLPatternGet();
#else
    PORT_HALLcodeGet(&HALLdata);
#endif
    if (HALLdata.HALLvalue != 0 && HALLdata.HALLvalue != 7)
    {
        HALLdata.combHALLvalue = (HALLdata.lastHALLvalue << 8) + (HALLdata.HALLvalue);        
        tmp1 = angleHALLjumpGet(&HALLdata);
        if(0u != HALLdata.numEffectiveHALLjump)
        {
            HALLdata.lastAngleHALLjump = HALLdata.angleHALLjump;
            HALLdata.angleHALLjump = tmp1;  
            tmp1 = HALLdata.angleHALLjump - HALLdata.lastAngleHALLjump;
            if(-RL_PI > tmp1){
                HALLdata.dAngleHALLjump = tmp1 + RL_TWO_PI;
            }else if(RL_PI < tmp1){
                HALLdata.dAngleHALLjump = tmp1 - RL_TWO_PI;
            }else{
                HALLdata.dAngleHALLjump = tmp1;
            }       
        }    

        switch(ptStt->state){
            case STT_HALL_ISR_INI: 
                HALLdata.angleHALL = tblHALLangle[HALLdata.HALLvalue];
                ptStt->state = STT_HALL_ISR_1ST_JUMP;
                break;
            case STT_HALL_ISR_1ST_JUMP:
                if(2u <= HALLdata.numEffectiveHALLjump){
                    HALLdata.angleHALL = tblHALLangle[HALLdata.HALLvalue];
                    dirHALLget(&HALLdata);
                    HALLdata.offsetAngleBound = (float)(HALLdata.dir) * RL_ONE_OVER_THREE_PI;
                    velocityCalOneJump(&HALLdata);
                    velocityManipulate(&HALLdata);
                    ptStt->state = STT_HALL_ISR_ONE_JUMP_LOW_SPEED;
                }            
                break;
            case STT_HALL_ISR_ONE_JUMP_LOW_SPEED:  /* Calculate velocity by neighbored HALL jumps. */
                HALLdata.angleHALL = tblHALLangle[HALLdata.HALLvalue];
                dirHALLget(&HALLdata);
                HALLdata.offsetAngleBound = (float)(HALLdata.dir) * RL_ONE_OVER_THREE_PI;
                velocityCalOneJump(&HALLdata);
                velocityManipulate(&HALLdata);
                if((7u < HALLdata.numEffectiveHALLjump) && (motorParams[Motor_ID_num].CTC_ELE_VEL_1_HIGH_BOUND < HALLdata.absEleVel)){
                    ptStt->state = STT_HALL_ISR_ONE_JUMP_HIGH_SPEED;
                }
            break;
        case STT_HALL_ISR_ONE_JUMP_HIGH_SPEED:  /* Calculate velocity by neighbored HALL jumps. */
            dirHALLget(&HALLdata);
            HALLdata.angleHALL = tblHALLangle[HALLdata.HALLvalue];
            HALLdata.offsetAngleBound = (float)(HALLdata.dir) * RL_ONE_OVER_THREE_PI;
            velocityCalOneJump(&HALLdata);
            velocityManipulate(&HALLdata);
            if(motorParams[Motor_ID_num].CTC_ELE_VEL_2_HIGH_BOUND < HALLdata.absEleVel){
                ptStt->state = STT_HALL_ISR_SIX_JUMP;
            } else if(motorParams[Motor_ID_num].CTC_ELE_VEL_1_LOW_BOUND > HALLdata.absEleVel){
                ptStt->state = STT_HALL_ISR_ONE_JUMP_LOW_SPEED;
            }
            break;            
        case STT_HALL_ISR_SIX_JUMP:
            HALLdata.angleHALL = tblHALLangle[HALLdata.HALLvalue];
            velocityCalSixJump(&HALLdata); 
            velocityManipulate(&HALLdata);
            if(motorParams[Motor_ID_num].CTC_ELE_VEL_2_LOW_BOUND > HALLdata.absEleVel){
                ptStt->state = STT_HALL_ISR_ONE_JUMP_HIGH_SPEED;
            }
            break;
        default:
            break;
        }
    }
    else    // else Hall value is invalid (0 or 7)
    {
        fh_faultHandler(HALL_SENSOR_FAULT);
    }
    HALLdata.test32_1++;
    
}

void hall_sensor_init(void)
{
    PDEC_HALLCallbackRegister((PDEC_HALL_CALLBACK)HALL_jump_ISR, (uintptr_t)NULL);
    PDEC_HALLStart();  /* Start PDEC in HALL or QDEC mode. */
    // Check for a valid hall sensor code
    PORT_HALLcodeGet(&HALLdata);
    if ( (0 == HALLdata.HALLvalue) || (7 == HALLdata.HALLvalue) )
    //invalid hall codes are 0 and 7 (1-6 are good)
    {
        fh_faultHandler(HALL_SENSOR_FAULT);
    }
}


/* *****************************************************************************
 End of File
 */
