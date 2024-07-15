USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING Siemens.Simatic.S71500.Clocks;
USING ENABLEPROGRAM;
USING ENABLEAX_FUNCTIONS;
USING  _TYPE;
USING _FC_FB;
USING _STATE_CONST;
USING _CONST_VAR_DEF;


NAMESPACE Retain_Namespase

/**************************************************************************************
 * AT CPU SHUTDOWN, IT PERFORMS THE SAVING INTO THE RETAIN OF THE AXIS QUOTES MODULE. *
 **************************************************************************************/

// PROGRAM SaveRetain
    
//     VAR_EXTERNAL
// 		ProTube                     :Produzione;
// 		System_Timer                : LDATE_AND_TIME; // Timer di sistema
// 		pm                      	:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
// 		ControlledStop				:BOOL;	//True se il Simotion è stato spento in modo controllato
// 		Axes                        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
// 		///ENABLEAX
// 		//ABSOLUTELY DO NOT MODIFY THE RETAIN IN THIS MODULE
// 		OldPosAxe                   :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF LREAL;
// 		Presence4RFORM              :SINT;
// 		O_EmergencyFromSimotion		:BOOL;	//Only to be able to compile machines without safety PLCs
		
	

//     END_VAR

//     VAR_EXTERNAL CONSTANT
// 		STATE_CNC_SHUTDOWN			:INT;	//257//Gruppo 16#100 Macchina spenta;

// 		U4 							:INT;  //Mobile slide
// 		U2 							:INT;  //Core
// 		T1							:INT;	//Axis table 1
// 		T2							:INT; 	//Axis table 2
// 		A1      					:INT;  //Rotation 4RTL/"Rotazione 4RC
// 		A2      					:INT;  //Rotation 4RC
// 		B1      					:INT;  //Rotation 4RIPC
// 		B2      					:INT;  //Rotation 4RIPC
// 		V2    						:INT;  //Flow-Drill
//     END_VAR

//     VAR
		
// 		dret					:DINT;
// 		SettingsTO				:FB_SettingsTO;
// 		PosAx					: REF_TO TO_PositioningAxis;
// 		S_TEST : LREAL;			//TEST_LUIGI
// 	END_VAR

//     VAR_TEMP
// 		i   					:INT;
// 	END_VAR

	// ;
	// Protube.machinestate.MachineState:=STATE_CNC_SHUTDOWN; ?????-
	// Protube.machinestate.ByteHight:=SHR(TO_WORD(STATE_CNC_SHUTDOWN) & WORD#16#FF00, UINT#8);
	// Protube.machinestate.ByteLow:= TO_WORD(STATE_CNC_SHUTDOWN); //&16#00FF;
	// Protube.machinestate.datetime:=System_Timer;
	// ControlledStop:=TRUE;
	// PosAx := AsPositioningAxisRef(Axes[0].PosAx); ?????-
	// S_TEST := PosAx^.ActualPosition;//TEST_LUIGI
	// FOR i :=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1 DO ?????-
	// 	// PosAx := AsPositioningAxisRef(Axes[0].PosAx); ?????-
	// 	IF CheckIfTOactive(Axis:=i)=TRUE THEN ;
	// 		// IF PosAx^.ActualPosition <= pm[i].Ma_Pos_Limit_Plus AND ?????-
    //         //     // PosAx^.ActualPosition >= pm[i].Ma_Pos_Limit_Minus THEN ;
	// 		// 	// //Storing axis positions only when they are not outside 
	// 		// 	// //the expected limits is an attempt to avoid storing situations 
	// 		// 	// //where the active positions at that time are incorrect.
	// 		// 	// OldPosAxe[0]:=PosAx^.ActualPosition; // error in TIA 
	//     	// END_IF;  
    // 	END_IF;      
	// END_FOR;
	// (*{
	// #ifdef EMULAZIONE
	// }
	// 	{
	// 	#ifdef SMART_SMART_A
	// 	}
	// 		{
	// 		#ifdef BASCULANTE_ELETTRICO
	// 		}*)
	// 			//To enable software emulation, U4 must be enabled.
	// 			SettingsTO(Axis:=U4, State:=TRUE);
	// 			pm[U4].ma_Asse_Abilitato:=TRUE;
	// 		(*{
	// 		#endif	//Fine BASCULANTE_ELETTRICO
	// 		}
	// 	{
	// 	#endif	//Fine SMART_SMART_A
	// 	}
	// {
	// #endif	//Fine EMULAZIONE
	// }
	// {
	// #ifdef ELECTMRL
	// }
	// 	{
	// 	#ifdef TAVOLI_ELETTRICI
	// 	}*)	
	// 		// SettingsTO(Axis:=T1, State:=TRUE); ?????-
	// 		// pm[T1].ma_Asse_Abilitato:=TRUE;
	// 		// SettingsTO(Axis:=T2, State:=TRUE);
	// 		// pm[T2].ma_Asse_Abilitato:=TRUE;
	// 	(*{
	// 	#endif //FINE TAVOLI_ELETTRICI
	// 	}
	// {
	// #endif //FINE ELECTMRL
	// }
	// {
	// #ifdef RUNNER
	// }*)
	// 		// This is used to ensure that BlmMotion can complete the WRITE TO CN command, 
	// 		// as the TO objects of A1, B1, and B2 are destroyed when they are not enabled 
	// 		// and there would be no structure to complete the data reception from BlmMotion, 
	// 		// which sends the parameters in any case.
	// 		// It will be the subsequent restart of Simotion that, depending on the parameter pm[x].ma_asse_installato, 
	// 		// the TO will be readjusted to the piece transferred from VGP.
	// 	// IF Presence4RFORM>0 THEN  ?????-
	// 	// 	(*{
	// 	// 	#ifndef PLAUNO_LIGHT
	// 	// 	}*)
	// 	// 		SettingsTO(Axis:=A1, State:=TRUE);
	// 	// 		pm[A1].ma_Asse_Abilitato:=TRUE;
	// 	// 		(*{
	// 	// 		#ifdef FOUR_RUNNER_PLAUNO
	// 	// 		}*)
	// 	// 			SettingsTO(Axis:=A2, State:=TRUE);
	// 	// 			pm[A2].ma_Asse_Abilitato:=TRUE;
	// 	// 		(*{
	// 	// 		#endif	//Fine FOUR_RUNNER_PLAUNO
	// 	// 		}
	// 	// 	{
	// 	// 	#endif //FINE NO PLAUNO_LIGHT
	// 	// 	}
	// 	// 	;*)
	// 	// END_IF;
	// 	(*{
	// 	#ifdef B1_B2
	// 	}*)
	// 		SettingsTO(Axis:=B1, State:=TRUE);
	// 		pm[B1].ma_Asse_Abilitato:=TRUE;
	// 		SettingsTO(Axis:=B2, State:=TRUE);
	// 		pm[B2].ma_Asse_Abilitato:=TRUE;
	// 	(*{
	// 	#endif	//Fine B1_B2
	// 	}
	// 	{
	// 	#ifdef FOUR_RUNNER
	// 	}
	// 		//The TO is reconstructed, otherwise, upon the next transition to RUN, 
	// 		the error of the technological object not being closed appears. !!!!*)
	// 		SettingsTO(Axis:=V2, State:=TRUE);
	// 	(*{
	// 	#endif	//Fine FOUR_RUNNER
	// 	}
	// {
	// #endif	//Fine RUNNER
	// }
	// {
	// #ifdef PROFINET
	// }*)
	// 	O_EmergencyFromSimotion:=TRUE;
	// 	//dret:=_waitTime(T#300ms);
	// (*{
	// #endif	//Fine PROFINET
	// }*)
// END_PROGRAM
END_NAMESPACE

