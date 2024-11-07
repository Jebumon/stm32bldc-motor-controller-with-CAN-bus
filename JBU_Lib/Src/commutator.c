/**************************************************************************//**
 * @file     commutation.c
 * @brief    Commutation sequence source file.
 * @version  V1.0
 * @date     10. April 2024
 * @developer Jebumon K Thomas
 ******************************************************************************/


#include "stdio.h"
#include "main.h"
//#include "..\..\JBU_Lib\Inc\commutator.h"

//TOTAL HALL STEPS FOR ONE ROTATION = 90.

//#define TIMER_OUTPUT_ACTIVE_LOW

const _Bool CommuteMatrix[8][6] = {//UH , UL - VH , VL - WH , WL
																    { 0 , 0  ,  0 , 0  , 0  , 0},
																		{ 0 , 1  ,  1 , 0  , 0  , 0},//1
																		{ 0 , 0  ,  0 , 1  , 1  , 0},//2
																		{ 0 , 1  ,  0 , 0  , 1  , 0},//3
																		{ 1 , 0  ,  0 , 0  , 0  , 1},//4
																		{ 0 , 0  ,  1 , 0  , 0  , 1},//5
																		{ 1 , 0  ,  0 , 1  , 0  , 0},//6
																		{ 0 , 0  ,  0 , 0  , 0  , 0}     

	
																/*  { 0 , 0  ,  0 , 0  , 0  , 0},
																		{ 1	, 0	 ,  0	, 1  , 0  ,	0},	
																		{ 1	, 0  ,  0 ,	0	 , 0	, 1},
																		{ 0	, 0	 ,  1	, 0	 , 0	, 1},
																		{ 0	, 1	 ,  1	, 0	 , 0	, 0},
																		{ 0	, 1	 ,  0	, 0	 , 1	, 0},
																		{ 0	, 0	 ,  0	, 1	 , 1	, 0},
																		{ 0 , 0  ,  0 , 0  , 0  , 0}, */
	
															  /*  { 0 , 0  ,  0 , 0  , 0  , 0},
																		{ 1	, 0	 ,  0	, 0  , 0  ,	1},	
																		{ 0	, 1	 ,  1	, 0	 , 0	, 0},
																		{ 0	, 0	 ,  1	, 0	 , 0	, 1},
																		{ 0	, 1	 ,  0	, 0	 , 1	, 0},
																		{ 1	, 0	 ,  0	, 1	 , 0	, 0},
																		{ 0	, 1	 ,  0	, 0	 , 0	, 1},
																		{ 0 , 0  ,  0 , 0  , 0  , 0}, */
																		
	
																  };
															

	void Commutation(int commutePosition, TIM_HandleTypeDef htim, int direction, int dutyCycle)
	{
		TIM_HandleTypeDef htim1 = htim;
		
		_Bool UH = CommuteMatrix[commutePosition][0];
		_Bool UL = CommuteMatrix[commutePosition][1];
		_Bool VH = CommuteMatrix[commutePosition][2];
		_Bool VL = CommuteMatrix[commutePosition][3];
		_Bool WH = CommuteMatrix[commutePosition][4];
		_Bool WL = CommuteMatrix[commutePosition][5];
		
#ifdef TIMER_OUTPUT_ACTIVE_LOW  //Output polarity of Timer channels
		#define UHP  1
		#define ULP  1
		#define VHP  1
		#define VLP  1
		#define WHP  1
		#define WLP  1
		
		#else
		#define UHP  0
		#define ULP  0
		#define VHP  0
		#define VLP  0
		#define WHP  0
		#define WLP  0
		
#endif

		
		
		//WH == 1 ? TIM1->CCMR1 = (0x01<<4) : WL == 1 ? TIM1->CCMR1 = (0x06<<4) : (0x01<<4);  //+Ve clamping for upper mosfet & PWM for Lower mosfet
		//VH == 1 ? TIM1->CCMR1 = (0x01<<12): VL == 1 ? TIM1->CCMR1 = (0x06<<12): (0x01<<12); //+Ve clamping for upper mosfet & PWM for Lower mosfet
		//UH == 1 ? TIM1->CCMR2 = (0x01<<4) : UL == 1 ? TIM1->CCMR2 = (0x06<<4) : (0x01<<4);  //+Ve clamping for upper mosfet & PWM for Lower mosfet
		

		
		uint16_t tempOutput = 0;
		tempOutput = (ULP<<11)|(UL<<10)|(UHP<<9)|(UH<<8)|(VLP<<7)|(VL<<6)|(VHP<<5)|(VH<<4)|(WLP<<3)|(WL<<2)|(WHP<<1)|WH;
		TIM1->CCER = tempOutput;
		
		//WH == 1 ? (Phase_U = (0xFFFF)) : (Phase_U = (dutyCycle));  //+Ve clamping for upper mosfet & PWM for Lower mosfet
		//VH == 1 ? (Phase_V = (0xFFFF)) : (Phase_V = (dutyCycle));  //+Ve clamping for upper mosfet & PWM for Lower mosfet
		//UH == 1 ? (Phase_W = (0xFFFF)) : (Phase_W = (dutyCycle));  //+Ve clamping for upper mosfet & PWM for Lower mosfet
		
		//if(UH == 1) {Phase_U = 0xFFFF;} else if (UL == 1){ Phase_U = dutyCycle;} //else {Phase_U = 0;} ;
		//if(VH == 1) {Phase_V = 0xFFFF;} else if (VL == 1){ Phase_V = dutyCycle;} //else {Phase_V = 0;} ;
		//if(WH == 1) {Phase_W = 0xFFFF;} else if (WL == 1){ Phase_W = dutyCycle;} //else {Phase_W = 0;} ;
		
	  Phase_W = (UH * 0xFFFF * (dutyCycle/dutyCycle) ) + (UL * dutyCycle);
		Phase_U = (VH * 0xFFFF * (dutyCycle/dutyCycle) ) + (VL * dutyCycle);
		Phase_V = (WH * 0xFFFF * (dutyCycle/dutyCycle) ) + (WL * dutyCycle);
		
	/*Phase_U = (UH * 0xFFFF * (dutyCycle/dutyCycle) ) + (UL * dutyCycle);
		Phase_V = (VH * 0xFFFF * (dutyCycle/dutyCycle) ) + (VL * dutyCycle);
		Phase_W = (WH * 0xFFFF * (dutyCycle/dutyCycle) ) + (WL * dutyCycle);*/
	}
	
/*void Commutation(int hallPosition, TIM_HandleTypeDef htim, int direction, int dutyCycle)
	{
		TIM_HandleTypeDef htim1 = htim;
		
		if(direction == 2){
			switch(hallPosition){
			
			//U = channel3
			//V = channel2
			//W = channel1
				case 1: //10 00 01
				//TIM1->CR2 |= 0 << 0;
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //UH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); //WL
				Phase_U = dutyCycle;
				//Phase_V = ZERO_DUTY;
				//Phase_W = MAX_DUTY; //For completely on lower mosfet.
				
			  break;
			
				case 2: //00 01 10
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //WH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);//VL
				//Phase_U = ZERO_DUTY;
				//Phase_V = MAX_DUTY;   //For completely on lower mosfet.
				Phase_W = dutyCycle;
				
				break;
			
				case 3: //10 01 00
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //UH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);//VL
				Phase_U = dutyCycle;
				//Phase_V = MAX_DUTY;   //For completely on lower mosfet.
				//Phase_W = ZERO_DUTY;
				
				break;
			
				case 4: //01 10 00
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //VH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);//UL
				//Phase_U = MAX_DUTY;   //For completely on lower mosfet.
				Phase_V = dutyCycle;
				//Phase_W = ZERO_DUTY;
				
				break;
			
				case 5: //00 10 01
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //VH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);//WL
				//Phase_U = ZERO_DUTY;
				Phase_V = dutyCycle;
				//Phase_W = MAX_DUTY; //For completely on lower mosfet.
				
				break;
			
				case 6: //01 00 10
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //WH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);//UL
				//Phase_U = MAX_DUTY;   //For completely on lower mosfet.
				//Phase_V = ZERO_DUTY;
				Phase_W = dutyCycle;
				
				break;
			
			default:
				break;
		}
	}
		else if(direction == 1)
			{
				switch(hallPosition){
			
			//U = channel3
			//V = channel2
			//W = channel1
				case 1: //01 00 10
				//TIM1->CR2 |= 0 << 0;
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //WH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); //UL
				//Phase_U = MAX_DUTY;   //For completely on lower mosfet.
				//Phase_V = ZERO_DUTY;
				Phase_W = dutyCycle;
				
			  break;
			
				case 2: //00 10 01
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //VH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);//WL
				//Phase_U = ZERO_DUTY;
				Phase_V = dutyCycle;
				//Phase_W = MAX_DUTY;   //For completely on lower mosfet.
				
				break;
			
				case 3: //01 10 00
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //VH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);//UL
				//Phase_U = MAX_DUTY;   //For completely on lower mosfet.
				Phase_V = dutyCycle;
				//Phase_W = ZERO_DUTY;
				
				break;
			
				case 4: //10 01 00
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //UH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);//VL
				Phase_U = dutyCycle;
				//Phase_V = MAX_DUTY;  //For completely on lower mosfet.
				//Phase_W = ZERO_DUTY;
				
				break;
			
				case 5: //00 01 10
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //WH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);//VL
				//Phase_U = ZERO_DUTY;
				//Phase_V = MAX_DUTY;  //For completely on lower mosfet.
				Phase_W = dutyCycle;
				
				break;
			
				case 6: //10 00 01
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); 
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
				
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //UH
				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);//WL
				Phase_U = dutyCycle;
				//Phase_V = ZERO_DUTY;
				//Phase_W = MAX_DUTY; //For completely on lower mosfet.
				
				break;
			
			default:
				break;
		}
				
		}
} */
	
