USING _FC_FC;
USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING Simatic.Ax.LAcycCom;
USING System.BitAccess;
USING System.Math;
USING Siemens.Simatic.S71500.Clocks;
USING System.DateTime;
USING ENABLEPROGRAM;
USING EnableAxFunction;
USING  _TYPE;
USING _CONST_VAR_DEF;
USING _STATE_CONST;
USING _FC_IO_FC;

NAMESPACE ENABLEAX_FUNCTIONS

/*******************************************************************************************************************
 * RETURNS TRUE IF THE TECHNOLOGICAL OBJECT POSASSE IS ACTIVE. THE FUNCTION IS ACTUALLY OPERATIONAL ONLY IF WE ARE IN SIMULATION, *
 * AS TO AVOID LOCKING THE BACKGROUND AND IPOS IS INTERROGATED FOR ITS STATE WITH IMMEDIATELY, SO THE OUTCOME OF THE RESPONSE CANNOT *
 * BE CONSIDERED SECURE. IN THE REALITY OF MACHINERY,  ACCESSING A DISABLED IPOS SHOULD NOT OCCUR; IF IT HAPPENS, THE PROBLEM MUST BE *
 *                                          SOLVED AS INDICATED EARLIER.                                           *
 *******************************************************************************************************************/

FUNCTION  CheckIfTOactive : BOOL


	VAR_External 
		pm                      : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
	END_VAR


	VAR_INPUT
		Axis	:INT;
	END_VAR
    
	(*{
	#ifdef EMULAZIONE
	}*)

		IF Axis>=_FIRST_AXIS#FIRST_AXIS AND Axis < _CONST_VAR_DEF._MAXIS#MAXAXIS THEN
			IF Pm[0].ma_Asse_Abilitato=TRUE THEN
				CheckIfTOactive :=TRUE;
			ELSE
				CheckIfTOactive:=FALSE;
			END_IF;
		END_IF;
	(*{
	#else	//Fine EMULAZIONE
	}*)
	CheckIfTOactive:=TRUE;
	(*{
	#endif	//Fine NO EMULAZIONE
	}*)
END_FUNCTION	

FUNCTION reset_trace

	VAR_EXTERNAL
		ElencoTrace                     :ARRAY[0..MAXTRACE-1] OF Trace;
		i_sys_Trace_ON              	:BOOL;  //True per far visualizzare la nuova trace inserita, sarà BlmMottion a metterlo a FALSE
		i_sys_current_trace 			:DINT;			// TRACE attiva
		i_sys_current_trace_axis 		:INT;  // Parametro della trace attiva
		i_sys_trace_number  			:DINT;
	END_VAR

	VAR_EXTERNAL CONSTANT
		MAXTRACE                        :INT;
		TRC_NOTRACE                     :INT;   //Cancellazione trace
	END_VAR

	VAR_INPUT
		Code		:DINT;  
	END_VAR

	VAR
		i			:INT;
		Count		:INT;
		Trovata		:BOOL;
	END_VAR

	FOR i:=0 TO MAXTRACE-1 DO
		//Scorre la tabella alla ricerca di una trace inserita con lo stesso codice 
		IF ElencoTrace[i].codice=Code THEN
			i_sys_Trace_ON:=TRUE;	//Dopo cancellazione abilitazione per BlmMotion per lettura nuova lettura della tabella
			ElencoTrace[i].asse:=0;
			ElencoTrace[i].dataora:=DT#2024-01-01-00:00:00.000;//DT#1980-01-23-13:14:33.123; //DATE_AND_TIME#0001-01-01-0:0:0.0;
			ElencoTrace[i].stato:=SINT#0;
			ElencoTrace[i].codice:=0;
		END_IF;
	END_FOR;
	Count:=0;
	Trovata:=FALSE;
	FOR i:=0 TO MAXTRACE-1 DO
		IF ElencoTrace[i].stato=1 THEN
			IF Trovata=FALSE THEN
				//Dopo aver eliminato l^eventuale trace attiva cerca la prima trace in elenco da visualizzare
				Trovata:=TRUE;
				i_sys_current_trace:=ElencoTrace[i].codice;
				i_sys_current_trace_axis:=ElencoTrace[i].asse;
			END_IF;
			//Conteggio delle trace attive
			Count:=Count+1;
		END_IF;
	END_FOR;
	i_sys_trace_Number:=Count;
	IF i_sys_trace_Number=0 THEN
		//Azzeramento trace se nella tabella non ne è presente nessuna
		i_Sys_Current_Trace:=TRC_NOTRACE;
		i_sys_current_trace_axis:=TRC_NOTRACE;
	END_IF;
END_FUNCTION

FUNCTION init_trace

	VAR_EXTERNAL
		ElencoTrace                     :ARRAY[0..MAXTRACE-1] OF Trace;
		i_sys_Trace_ON              	:BOOL;  //True per far visualizzare la nuova trace inserita, sarà BlmMottion a metterlo a FALSE
		i_sys_current_trace 			:DINT;			// TRACE attiva
		i_sys_current_trace_axis 		:INT;  // Parametro della trace attiva
		i_sys_trace_number  			:DINT;
	END_VAR

	VAR_EXTERNAL CONSTANT
		MAXTRACE                        :INT;
		TRC_NOTRACE                     :INT;   //Cancellazione trace
	END_VAR

	VAR
		i:INT;
	END_VAR   
	// Inizializza la coda errori
	i_Sys_Current_Trace:=TRC_NOTRACE;
	i_sys_current_trace_axis:=TRC_NOTRACE;
	i_sys_Trace_number:=0;
	FOR i:=0 TO MAXTRACE-1 DO
		ElencoTrace[i].asse:=0;
		ElencoTrace[i].dataora:=DT#2001-01-01-0:0:0.0;
		//ElencoTrace[i].p_dint:=0;
		ElencoTrace[i].stato:=SINT#0;
		ElencoTrace[i].codice:=0;
	END_FOR;	
END_FUNCTION

FUNCTION init_error

	VAR_EXTERNAL
		i_sys_error                     :DINT;	// codice errore
		i_sys_error_number				:DINT;	//Numero di errori presenti nel sistema
		AlarmsQueue                    : ARRAY[0..MAXALARMS_LIFO-1] OF Alarms;
		i_sys_error_blm			       :Alarms;
	END_VAR

	VAR_EXTERNAL CONSTANT
		MAXALARMS_LIFO                  :INT;
		ERR_NOERROR						:SINT;
	END_VAR

	VAR
		i	:INT;
	END_VAR;
	//Inizializza la coda errori
	i_sys_error:=ERR_NOERROR;
	i_sys_error_number:=0;
	FOR i:=0 TO MAXALARMS_LIFO-1 DO
		AlarmsQueue[i].Code:=ERR_NOERROR;
		AlarmsQueue[i].Code_export:=ERR_NOERROR;
		AlarmsQueue[i].TypeAxis:=ERR_NOERROR;
		AlarmsQueue[i].Status:=SINT#0; // attivo
	END_FOR;
	i_sys_error_blm.Code:=ERR_NOERROR;
	i_sys_error_blm.Code_export:=ERR_NOERROR;	
	i_sys_error_blm.TypeAxis:=ERR_NOERROR;
	i_sys_error:=ERR_NOERROR;
END_FUNCTION

FUNCTION Push_error:INT
	// Inserisce un errore nella coda errori
	VAR_EXTERNAL
		TimerSystem                     :ARRAY[0..MAX_TIMER_SYSTEM-1] OF Timer;
		i_sys_error                    	:DINT;	// codice errore
		AlarmsQueue                    	: ARRAY[0..MAXALARMS_LIFO-1] OF Alarms;
		i_sys_error_number	           	:DINT;	//Numero di errori presenti nel sistema
		i_sys_error_blm			        :Alarms;
		System_Timer                    : LDATE_AND_TIME; // Timer di sistema
		HistoricalAlarms                :ARRAY[0..MAXALARMS_FIFO-1] OF Alarms;
		HistoricalIN                    :INT; // Puntatore al primo allarme nello storico
		HistoricalOUT                   :INT; //Puntatore all^ultimo allarme nello storico
		i_sys_activedevice              : INT;  //Dispositivo selezionato per movimenti in JOG
												// da 0..31 asse
												// da 32 a 64 dispositivo IO
												// -1 = INDEFINITO
		i_sys_OPMODE            		: INT;	// MODO OPERATIVO
												// 0 = JOG
												// 1 = REFERENCE
												// 2 = MDA
												// 3 = AUTO
		i_sys_Error_ON              	:BOOL;  //Indica a VGP che c^è un cambiamento nella situazione dei messaggi di errore
		RejectTypeBerg					:BOOL;	//True=esegue scarto spezzone tubo secondo specifiche cliente Berg SW00001253
		i_sys_swOVR          			: LREAL ;  // OVERRIDE SOFTWARE - VALORE 0 - 100
	END_VAR

	VAR_EXTERNAL CONSTANT
		MAX_TIMER_SYSTEM 	                :USINT;
		T_POST_MORTEM						:INT;
		MAXALARMS_LIFO                      :INT;
		MAXALARMS_FIFO                     	:INT;
	END_VAR

	VAR_INPUT
		Code					:DINT; //Codice errore
		Asse					:INT :=-1;
		P_DINT				:ARRAY[1..5] OF DINT :=[0,0,0,0,0];
		P_UDINT				:ARRAY[1..5] OF UDINT:=[UDINT#0,UDINT#0,UDINT#0,UDINT#0,UDINT#0];
		P_LREAL				:ARRAY[1..5] OF LREAL:=[0,0,0,0,0];
		Arresto				:BOOL:=TRUE;
		Attesa				:BOOL:=TRUE;
		ErrorGroup			:DWORD:=DWORD#0;
		TypAxe	  			:INT:=0;   //0=posAxis, 1=followingObjectType, 2=camType, 3=pathObjectTpye
		SoloMessaggio		:BOOL:=FALSE;	
	END_VAR

	VAR
		i						:INT;
		AddCode			:INT;
		AllarmePres	:BOOL:=FALSE;
	END_VAR
	IF Code<>i_sys_error THEN
		TimerSystem[T_POST_MORTEM].Value:=1000;
		TimerSystem[T_POST_MORTEM].Status:=SINT#1;
	END_IF;
	IF i_sys_activedevice<>SF#C_RESET_SIMOTION AND i_sys_OPMODE<>PI_COMMANDS_CODE#REFMODE THEN
		//In caso di errore si resetta il dispositivo/comando attivo, solo non
		//è il comando di reset del Simotion, oppure non siamo in referent point, così
		//da poter fare l^index degli assi anche con errori attivi
		i_sys_activedevice:=MAX_NUMBER_STATES_FLASHING_4COLOR#UNDEF_DEVICE;
	END_IF;
	FOR i:=0 TO MAXALARMS_LIFO-1 DO 
		IF code=AlarmsQueue[i].Code THEN
			AllarmePres:=TRUE;
		END_IF;
	END_FOR;
	//Inserisce l^errore solo se differente dall^ultimo inserito	   
	//IF Code<>i_sys_error AND i_sys_error_number<MAXALLARMI_LIFO THEN
	IF NOT(AllarmePres) THEN
		// ferma tutti gli assi e passa in manuale
		AddCode:=0; 
		IF errorgroup <> DWORD#0 THEN
			FOR i:=0 TO 23 DO
				IF getbit(errorgroup,TO_USINT(i))=TRUE THEN
					AddCode:=i;
					EXIT;
				END_IF;			   
			END_FOR;  
		END_IF;
		i_sys_Error_ON:=TRUE;
		AlarmsQueue[i_sys_error_number].Code:=Code+AddCode;
		IF Code+AddCode>=_ERR#ERR_FIRST_ERROR_SIEMENS AND Code+AddCode<_ERR#ERR_FIRST_BLM THEN
			AlarmsQueue[i_sys_error_number].Code_export:=TypAxe*DINT#1000000+Code+AddCode;
		ELSE
			AlarmsQueue [i_sys_error_number].Code_export:=ERROR_TYPE#ERROR_BLM*DINT#1000000+Code+AddCode;
		END_IF;
		AlarmsQueue[i_sys_error_number].TypeAxis:=TypAxe;
		AlarmsQueue[i_sys_error_number].Axis:=TO_SINT(Asse);
		FOR i:=1 TO 5 DO
			AlarmsQueue[i_sys_error_number].P_DINT[i]:=P_DINT[i];
			AlarmsQueue[i_sys_error_number].P_UDINT[i]:=P_UDINT[i];
			AlarmsQueue[i_sys_error_number].P_LREAL[i]:=P_LREAL[i];
		END_FOR;
		GetSystemDateTime(value => System_Timer); //EX RCT 
		//System_Timer(read:=TRUE); 
		AlarmsQueue[i_sys_error_number].DateTime:=TO_DT(System_Timer);
		AlarmsQueue[i_sys_error_number].Status:=SINT#1; // attivo
		IF Code>=_ERR#ERR_FIRST_BLM OR Code<_ERR#ERR_FIRST_ERROR_SIEMENS THEN
			i_sys_error_blm:=AlarmsQueue[i_sys_error_number];
		END_IF;	
		IF SoloMessaggio=FALSE THEN
			i_sys_error_number:=i_sys_error_number+1;
			(*{
			#ifdef FOUR_RUNNER_PLAUNO
			}*)
			RejectTypeBerg:=TRUE;
			(*{
			#endif FOUR_RUNNER_PLAUNO
			}*)
		END_IF;
		// Inserisce nello storico allarmi
		HistoricalAlarms[HistoricalIN].Code:=Code+AddCode;
		HistoricalAlarms[HistoricalIN].TypeAxis:=TypAxe;
		HistoricalAlarms[HistoricalIN].Axis:=TO_SINT(Asse);
		FOR i:=1 TO 5 DO
			HistoricalAlarms[HistoricalIN].P_DINT[i]:=P_DINT[i];
			HistoricalAlarms[HistoricalIN].P_UDINT[i]:=P_UDINT[i];
			HistoricalAlarms[HistoricalIN].P_LREAL[i]:=P_LREAL[i];
		END_FOR;
		i_sys_error:=Code;
		//System_Timer(read:=TRUE);
		HistoricalAlarms[HistoricalIN].DateTime:=TO_DT(System_Timer);
		HistoricalAlarms[HistoricalIN].Status:=SINT#1; // attivo
		HistoricalIN:=((HistoricalIN+1) MOD MAXALARMS_FIFO);
		IF HistoricalIN=HistoricalOUT THEN
			// Sposta il puntatore della FIFO (ossia cancella l^allarme più vecchio)
			HistoricalOUT:=((HistoricalIN+1) MOD MAXALARMS_FIFO);
		END_IF;
		IF Arresto=TRUE THEN
			Abort_All();
		END_IF;
		i_sys_swOVR:=CONSTANTS_WELD_SEARCH#OVERRIDE_STANDARD;
	END_IF;
	Push_error:=0;
	//Fine:;
END_FUNCTION

FUNCTION Abort_All

	VAR_EXTERNAL
		i_sys_OPMODE                    : INT;
		i_sys_CHANSTATUS                : INT;	// STATO DEL SISTEMA
                                                            // 0 = Canale in RESET - IN JOG è sempre 0
                                                            // 1 = Canale attivo - IN REF, MDA e AUTO attivi
                                                            // 2 = Canale interrotto
		TaskCyclesActive                :BOOL;
		mcpo_ncstop  					:BOOL;
		mcpo_ncstart 					:BOOL;  
	END_VAR
	
	VAR 
		dret  :DINT;
		dwret :DWORD;
	END_VAR;
	// IF i_sys_OPMODE=PI_COMMANDS_CODE#AUTOMODE THEN %%%%%
	// 	dwret:=_suspendTaskId(_task.motiontask_AUTO);
	// END_IF;	
	// IF i_sys_OPMODE=PI_COMMANDS_CODE#MDAMODE THEN
	// 	dwret:=_suspendTaskId(_task.motiontask_MDA);
	// END_IF;
	// IF _getStateOfTaskId(_task.MotionTask_EliminaCodaComandi)=TASK_STATE_STOPPED THEN
	// 	dwret:=_restartTaskId(_task.MotionTask_EliminaCodaComandi);
	// END_IF;
	// IF i_sys_OPMODE=PI_COMMANDS_CODE#AUTOMODE THEN
	// 	dwret:=_restartTaskId(_task.motiontask_AUTO);
	// END_IF;	
	// IF i_sys_OPMODE=PI_COMMANDS_CODE#MDAMODE THEN
	// 	dwret:=_restartTaskId(_task.motiontask_MDA);
	// END_IF;	
	// dwret:=_resetTaskId(_task.motiontask_CICLI);
	// dwret:=_resetTaskId(_task.motiontask_Weld_Marker);
	(*{
	#ifdef DH4010	
	}*)
		// dwret:=_resetTaskId(_task.MotionTask_AutoSx); %%%%
		// dwret:=_resetTaskId(_task.MotionTask_AutoDx);
	(*{
	#endif DH4010	
	}*)
	mcpo_NCSTART:=FALSE;
	mcpo_NCSTOP:=FALSE;
	i_sys_CHANSTATUS:=PI_COMMANDS_CODE#CHANNELRESET;  
	TaskCyclesActive:=FALSE;
END_FUNCTION

FUNCTION VerificationSimulationAxes:BOOL

	VAR_EXTERNAL
	    pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;

  	END_VAR
  
  VAR_EXTERNAL CONSTANT
	X1 								:INT;    //Cart X
  END_VAR

  VAR_INPUT
    Axis                : DB_ANY;  
  END_VAR

	VAR
		PosAxes    : REF_TO TO_PositioningAxis;

	END_VAR
;
	//PosAxes:=AsPositioningAxisRef(Axis);
	
	 //VerificationSimulationAxes:=PosAxes^.StatusWord.%X25; ///Lock at FB_SetDatiAsse*)
  //VerificaSimulazioneAssi:=pm[X1].MA_STATO_ENCODER=SIMULATION;
END_FUNCTION 

FUNCTION_BLOCK FB_ResetMotionResetAxis

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
		pm                      	    :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
		PercentageTorqueSet          	:ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF LREAL;
	END_VAR

	VAR_EXTERNAL CONSTANT
		MY_BIG_VALUE_REAL                   	:REAL;
	END_VAR

	VAR_INPUT
		Asse		:INT;
		Attesa	:BOOL:=TRUE;
	END_VAR
	
	VAR
		
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
		ResetAxisBLM		:FB_ResetAxisBLM;
		PosAx            : REF_TO TO_PositioningAxis;
	END_VAR

	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF pm[Asse].ma_asse_abilitato=TRUE THEN
				IF (ResetAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
					ResetAxisBLM(Asse, Attesa);
				ELSE
					ResetAxisBLM(Asse, Attesa);
					//dret:=_resetMotionBuffer(axis:=Axes[Asse].PosAx);		
					PercentageTorqueSet[Asse]:=MY_BIG_VALUE_REAL;
					PosAx:=AsPositioningAxisRef(Axes[Asse].PosAx);
					IF PosAx^.StatusWord.%X25 AND pm[Asse].ma_asse_abilitato=TRUE THEN ;
						//Con asse in simulazione dopo il reset si deve ripristinare la simulazione
						(*dret:=_enableAxisSimulation(axis:=Axes[Asse].PosAx, 
																				nextCommand:=IMMEDIATELY, 
																				commandId:=_getCommandId());*)
					END_IF;
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
			GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_ResetAxisBLM

	VAR_EXTERNAL
		UsoPLCOpen						:BOOL;
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
		mcReset							:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1-1] OF MC_Reset;
	END_VAR

	VAR_INPUT
		Asse	:INT;
		Attesa:BOOL;
	END_VAR

	VAR
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
		//PosAxes : REF_TO TO_PositioningAxis;

/************
 * PLC_OPEN *
 ************/

		//mcResetAxisInfo :  MC_Reset;
	END_VAR

	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT: 
		(*IF UsoPLCOpen THEN*)
			Axes[Asse].MC_Axes.reset_axes :=TRUE;
		(*END_IF;*)
		
		IF Axes[Asse].MC_Axes.State_reset.Done  THEN
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
		END_IF;
		
		// 	IF UsoPLCOpen THEN
		// 		//PosAxes:=AsPositioningAxisRef(Axes[Asse].PosAx);
		// 	Axes[Asse].mcResetAxisInfo.Execute:=TRUE;

		// 	END_IF;
		// 	IF (Attesa) THEN
		// 		CURRENT_STATE:=_FB_ResetAxisBLM#STATE_RESET_AXIS_BLM__WAIT;
		// 	ELSE
		// 		CURRENT_STATE:=GENERAL_STATE#STATE_END;
		// 	END_IF;
			
		// _FB_ResetAxisBLM#STATE_RESET_AXIS_BLM__WAIT:
		// 	IF (UsoPLCOpen) THEN
		// 		IF Axes[Asse].mcResetAxisInfo.done=TRUE THEN //IF mcReset[Asse].done=TRUE THEN 
		// 			CURRENT_STATE:=GENERAL_STATE#STATE_END;
		// 		END_IF;
		// 	END_IF;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;

/************
 * PLC_OPEN *
 ************/

	//mcResetAxisInfo (Axis:=PosAxes^);
END_FUNCTION_BLOCK

/*****************************************************
 * REMOVES THE^LAST ERROR ENTERED IN THE ERROR QUEUE *
 *****************************************************/

FUNCTION pop_error

	VAR_EXTERNAL
		i_sys_error_number	           :DINT;
		AlarmsQueue                    : ARRAY[0..MAXALARMS_LIFO-1] OF Alarms;
		i_sys_error                    :DINT;
		i_sys_error_blm			       :Alarms;
		System_Timer                   : LDATE_AND_TIME; // Timer di sistema
		
	END_VAR

	VAR_EXTERNAL CONSTANT
		MAXALARMS_LIFO                      :INT;
		ERR_NOERROR							:SINT;
		ERR_FIRST_BLM                       :DINT; // Primo codice di errore BLM
		ERR_FIRST_ERROR_SIEMENS			    :DINT;
	END_VAR

	VAR
		tempcode		:DINT;  
		iret				:INT;
		TempTipoAsse:INT;
	END_VAR		
	IF i_sys_error_number>0 THEN 
		i_sys_error_number:=i_sys_error_number-1;
		tempcode:=AlarmsQueue[i_sys_error_number].Code;
		TempTipoAsse:=AlarmsQueue[i_sys_error_number]. TypeAxis;
		AlarmsQueue[i_sys_error_number].Code:=ERR_NOERROR;
		AlarmsQueue[i_sys_error_number].Code_export:=ERR_NOERROR;
		AlarmsQueue[i_sys_error_number]. TypeAxis:=ERR_NOERROR;
		//System_Timer(read:=TRUE);
		AlarmsQueue[i_sys_error_number].DateTime:=TO_DT(System_Timer);
		AlarmsQueue[i_sys_error_number].Status:=SINT#0; // attivo
		IF i_sys_error_number>0 THEN
			i_sys_error:=AlarmsQueue[i_sys_error_number-1].Code;
			IF i_sys_error>=ERR_FIRST_BLM OR i_sys_error<ERR_FIRST_ERROR_SIEMENS THEN
				i_sys_error_blm.Code:=i_sys_error;
			ELSE
				i_sys_error_blm.Code:=ERR_NOERROR;
				i_sys_error_blm.Code_export:=ERR_NOERROR;
				i_sys_error_blm. TypeAxis:=ERR_NOERROR;
			END_IF;	 
		ELSE
			i_sys_error_blm.Code:=ERR_NOERROR;
			i_sys_error_blm.Code_export:=ERR_NOERROR;
			i_sys_error_blm. TypeAxis:=ERR_NOERROR;
			i_sys_error:=ERR_NOERROR;
		END_IF;
	ELSE
		//sbianca la coda allarmi
		init_error();
	END_IF;
END_FUNCTION		

FUNCTION_BLOCK FB_Disable_Axis_In_Torque

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
		PercentageTorqueSet          	:ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF LREAL;
		AxisInLimitationCouple		 	:ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;
		pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
	END_VAR

	VAR_EXTERNAL CONSTANT
		MY_BIG_VALUE_REAL                   	:REAL;
		INACTIVE_DEACTIVATABLE_AXIS         	:SINT;
	END_VAR

	VAR_INPUT
		NumeroAsse	:DINT;
		Verifica		:BOOL;
	END_VAR
	VAR
		dret				:DINT;
		Asse				:REF_TO TO_PositioningAxis;
		
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
	END_VAR
	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF CheckIfTOactive(Axis:=TO_INT(NumeroAsse))=TRUE THEN
				Asse:=AsPositioningAxisRef(Axes[NumeroAsse].PosAx);
				PercentageTorqueSet[NumeroAsse]:=MY_BIG_VALUE_REAL;
				IF Asse^.VirtualAxis.Mode = UDINT#1 THEN
					IF pm[NumeroAsse].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS THEN ;
						(*IF Asse.torquelimitingcommand.state=ACTIVE THEN
							dret:=_disableTorqueLimiting(axis:=Asse, 
																					 nextCommand:=IMMEDIATELY,
																					 commandId:=_getCommandId());
							IF (Verifica) THEN
								CURRENT_STATE:=_FB_Disable_Axis_In_Torque#STATE_DISABLE_AXIS_IN_TORQUE_WAIT;
							ELSE
								CURRENT_STATE:=GENERAL_STATE#STATE_END;
							END_IF;
						ELSE
							CURRENT_STATE:=GENERAL_STATE#STATE_END;
						END_IF;*)
					ELSE
						AxisInLimitationCouple[NumeroAsse]:=FALSE;
						CURRENT_STATE:=GENERAL_STATE#STATE_END;
					END_IF;
				ELSE
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
			_FB_Disable_Axis_In_Torque#STATE_DISABLE_AXIS_IN_TORQUE_WAIT:
			(*IF Asse.torquelimitingcommand.state<>ACTIVE THEN 
				AxisInLimitationCouple[NumeroAsse]:=FALSE;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
		GENERAL_STATE#STATE_END:*)
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK Homing

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_INPUT
		Asse	:INT;
		Quota	:LREAL;
		Modo	:EnumHomingMode;
	END_VAR
	

	Axes[Asse].MC_Axes.home_axis_param.Mode:=Modo;
		
		//PLCOpen
		Axes[Asse].MC_Axes.home_axis_param.position:=Quota;
		Axes[Asse].MC_Axes.home_axis_param.Mode:=Modo;
		Axes[Asse].MC_Axes.home_axes:=TRUE;

END_FUNCTION_BLOCK

FUNCTION GetActiveEncoder:INT

		VAR_EXTERNAL
			Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
		END_VAR

		VAR_INPUT
			Asse	:INT;
		END_VAR
		
		VAR
			PosAx		:REF_TO TO_PositioningAxis;
			i			:INT;
		END_VAR
		PosAx:= AsPositioningAxisRef(Axes[Asse].PosAx); 
		FOR i:=1 TO 
			TO_INT(PosAx^.Sensor[4].Existent =truE) DO
			IF PosAx^.StatusSensor[i].control=true THEN
				GetActiveEncoder:=i;
			END_IF;
		END_FOR;
END_FUNCTION

FUNCTION TestAxesY2:BOOL
	VAR_EXTERNAL
		i_sys_PresenceMatrix			          :BOOL;
	END_VAR

	IF i_sys_PresenceMatrix=TRUE THEN
		TestAxesY2:=TRUE;
	ELSE
		TestAxesY2:=FALSE;
	END_IF;
END_FUNCTION

/***********************************************************************************************
 * RETURNS TRUE IF NUMSTATI IS GREATER THAN 0. FOR DH4010 BOTH NUMSTATI IN THE TWO TABLES MUST *
 *                                    BE GREATER THAN ZERO                                     *
 ***********************************************************************************************/

FUNCTION TestNumStati:BOOL

	VAR_EXTERNAL
		Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
	END_VAR

	VAR_EXTERNAL CONSTANT

	END_VAR

	VAR_INPUT
		_Program	:INT;
	END_VAR
	(*{
	#ifdef DH4010
	}*)

		TestNumStati:=Programmi[_Program].Numstati[TABLE#TABLE_RX]>0 AND //IFDEF non e giusto, varificare
						Programmi[_Program].Numstati[TABLE#TABLE_LX ]>0;
	(*{
	#else	//Fine DH4010
	}*)
		(*TestNumStati:=Programmi[_Program].Numstati>0;*)
	(*{
	#endif	//Fine NO DH4010
	}*)
END_FUNCTION

/*********************************************************************************
 * BASED ON THE POSITION OF THE MASTER, IT CALCULATES THE CORRESPONDING POSITION *
 *                                 OF THE SLAVE                                  *
 *********************************************************************************/

FUNCTION_BLOCK ReadPosSlave
	
	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_INPUT
		
		NomeMaster: DB_ANY;
		Asse			:INT;
		PosMaster	    :LREAL;

		Modo			:_EnumCamPositionMode;
	END_VAR

	VAR_OUTPUT
		PosSlave : LREAL;
	END_VAR
	VAR
		ActiveMaster : REF_TO TO_SynchronousAxis;
		_getcamfollowingvalue : MC_GetCamFollowingValue; 
		cam :REF_TO TO_Cam;
		SelMaster : REF_TO TO_SynchronousAxis;
	END_VAR;

	ActiveMaster:= AsSynchronousAxisRef(Axes[Asse].PosAx);
	SelMaster:= AsSynchronousAxisRef(NomeMaster);
	
		IF ActiveMaster^.StatusSynchronizedMotion.ActualMaster=SelMaster^.StatusSynchronizedMotion.ActualMaster THEN //Assi[Asse].sincroasse.activemaster=NomeMaster
			If Modo = _EnumCamPositionMode#BASIC THEN
				_getcamfollowingvalue.MasterOffset:=0;
			ELSE
				_getcamfollowingvalue.MasterOffset:=0;
			END_if;
			cam:= AsCamRef(Axes[Asse].Camma);                  
			_getcamfollowingvalue(Execute := TRUE, 
								LeadingValue := PosMaster, 
								MasterOffset := 0, 
								SlaveOffset := 0, 
								MasterScaling := 0, 
								SlaveScaling :=0 ,
								//  Done => _done, 
								//  Busy => _busy, 
								//  Error => , 
								//  ErrorId => , 
								Value => PosSlave, 
								//  FirstDerivative => ,
								//   SecondDerivative => , 
								Cam := cam^ );
		ELSE
			//Asse senza camma, si restituisce l^attuale posizione
			PosSlave:=ActiveMaster^.actualPosition;
		END_IF;
 
	
END_FUNCTION_BLOCK

//Esegue la disabilitazione della forzatura dell^ingresso ripristinandone il valore reale
FUNCTION DisAbilitaForzaturaInput

	VAR_EXTERNAL
		IAddress                        : ARRAY [0..MAXDIGITALINPUT] OF Digital_Input;
	END_VAR

	VAR_EXTERNAL CONSTANT
		MAXDIGITALINPUT                     :INT;
	END_VAR

	VAR_INPUT
		Indirizzo	:INT;
	END_VAR
	Iaddress[Indirizzo].enable_forzatura:=FALSE;
END_FUNCTION	

/************************************************************************************************************
 *        FOR SMART'S ELECTRIC ROCKER WHEN THE JOG IS ACTIVE, THE B2 AXIS SHOULD NOT BE RESET IF THE        *
 *                ELEVATION IS WITHIN ONE OF THE TWO THRUST ZONES, OTHERWISE THE COMMAND TO                 *
 * POSITIONING TO GO ON THE EXTREMES TRIGGERED BY THE BACKGROUND IN THE EMULATIONPNEUMATICOVERHEAD FUNCTION *
 ************************************************************************************************************/

FUNCTION TestAsseDaResettare:BOOL
	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
		cfg                             :_TYPE.Config_Struct;
		i_sys_OPMODE                    : INT;	// MODO OPERATIVO
		(*{
		#ifdef BASCULANTE_ELETTRICO
		}*)
		  i_sys_coppiaB2          :USINT;  //Percentuale coppia spinta B2 sugli estremi
		  BasculantePosCarm7_DX   :REAL;
		  BasculantePosCarm7_SX   :REAL;
		(*{
		#else  //Fine BASCULANTE_ELETTRICO
		}*)
	END_VAR

	VAR_EXTERNAL CONSTANT
		JOGMODE                         	:INT;
		B2      							:INT;  //Rotation 4RIPC
		TILTING_POS_CURVATURE               :REAL;
		TILTING_DIST_POS_PUSH_JOG      :REAL;
		OVERHEAD_LEFT_SIDE      		    :SINT;
		
	END_VAR

	VAR_INPUT
		Asse	:INT;
	END_VAR

	VAR
		AxesS								:REF_TO TO_SynchronousAxis;	

	END_VAR
	;
	// (*{
	// #ifdef BASCULANTE_ELETTRICO
	// }*)
	// 	TestAsseDaResettare:=TRUE;	
	// 	IF Asse=B2 AND i_sys_OPMODE=JOGMODE THEN
	// 		IF cfg.i_sys_latobasculante=OVERHEAD_LEFT_SIDE THEN
	// 			//BASCULANTE LATO SX
	// 			AxesS:=AsSynchronousAxisRef(Axes[B2].PosAx);
	// 			IF AxesS^.actualPosition>TILTING_POS_CURVATURE-TILTING_DIST_POS_PUSH_JOG OR
	// 				AxesS^.actualPosition<BasculantePosCarm7_SX+TILTING_DIST_POS_PUSH_JOG THEN
	// 				TestAsseDaResettare:=FALSE;	
	// 			END_IF;
	// 		ELSE
	// 			//BASCULANTE LATO DX
	// 			IF AxesS^.actualPosition<TILTING_POS_CURVATURE+TILTING_DIST_POS_PUSH_JOG OR
	// 				AxesS^.actualPosition>BasculantePosCarm7_DX-TILTING_DIST_POS_PUSH_JOG THEN
	// 				TestAsseDaResettare:=FALSE;	
	// 			END_IF;
	// 		END_IF;
	// 	END_IF;		
	// (*{
	// #else	//Fine BASCULANTE_ELETTRICO
	// }*)
	// 	TestAsseDaResettare:=TRUE;	
	// (*{
	// #endif	//FineBASCULANTE_ELETTRICO
	// }*)
END_FUNCTION

FUNCTION StopAxis

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_INPUT
		Asse:INT;
		Acceleration: LREAL:= -1;
	END_VAR
	
	(*IF NOT (UsoPLCOpen) THEN
		IF (Acceleration>-1 AND Deceleration>-1) THEN
			dret:=_stop(axis:=Assi[Asse].PosAx, 
								stopMode:=STOP_AND_ABORT,
								stopSpecification:=ALL_AXIS_MOTION, 
								movingMode:=POSITION_CONTROLLED, 
								stopId:=_getCommandId(), 
								velocityProfile:=TRAPEZOIDAL, 
								mergeMode:=IMMEDIATELY, 
								nextCommand:=IMMEDIATELY,
								commandId:=_getCommandId(),
								PositiveAccel:=Acceleration,
								NegativeAccel:=Deceleration);
		ELSE
			dret:=_stop(axis:=Assi[Asse].PosAx, 
					stopMode:=STOP_AND_ABORT,
					stopSpecification:=ALL_AXIS_MOTION, 
					movingMode:=POSITION_CONTROLLED, 
					stopId:=_getCommandId(), 
					velocityProfile:=TRAPEZOIDAL, 
					mergeMode:=IMMEDIATELY, 
					nextCommand:=IMMEDIATELY,
					commandId:=_getCommandId());			
		END_IF;
	ELSE
		//PLCOpen*)
		// Assi[Asse].mcStopAxis.Acceleration:=Acceleration;
		// Assi[Asse].mcStopAxis.Execute:=TRUE;
		Axes[Asse].MC_Axes.stop_axes := TRUE;  
		Axes[Asse].MC_Axes.stop_Axis_Param.Deceleration :=Acceleration;
		//Axes[Asse].mcStopAxis.Execute:=TRUE;
	//END_IF;
END_FUNCTION	

FUNCTION_BLOCK FB_StopMaster

	VAR_EXTERNAL
		AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
	END_VAR

	VAR_EXTERNAL CONSTANT

		FIRST_AXIS_MASTER				        :INT; 
	  
	  END_VAR 

	VAR_INPUT
	NomeMaster					:INT;
    AccelerazioneMaster	:LREAL;
	
	END_VAR
	VAR
		dret					:DINT;
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
		//thisCommand		:commandIDType;
	END_VAR
	VAR_OUTPUT
		OutputState		:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:

			//thisCommand:=_getCommandID();
			CURRENT_STATE:=_FB_StopMaster#STATE_STOP_MASTER_STOP;

		_FB_StopMaster#STATE_STOP_MASTER_STOP:
	
		AssiMaster[NomeMaster].MC_Axes.stop_axes := TRUE;
		AssiMaster[NomeMaster].MC_Axes.stop_Axis_Param.Deceleration := AccelerazioneMaster;
			(*dret:=_stop(axis:=NomeMaster, 
									positiveAccelType:=DIRECT,
									positiveAccel:=AccelerazioneMaster,
									negativeAccelType:=DIRECT,
									negativeAccel:=AccelerazioneMaster,
									stopMode:=STOP_AND_ABORT, 
									stopSpecification:=BASIC_MOTION, 
									movingMode:=POSITION_CONTROLLED, 
									stopId:=_getCommandId(), 
									mergeMode:=IMMEDIATELY, 
									nextCommand:=IMMEDIATELY, 
									commandId:=thisCommand);*)	
			IF AssiMaster[NomeMaster].MC_Axes.State_stop.Done THEN
				AssiMaster[NomeMaster].MC_Axes.stop_axes := FALSE;
			// 	IF NOT (dret=7001 OR dret=7002) THEN
			// 		CURRENT_STATE:=STATE_END;
			// 	END_IF;
			// ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
				
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_ResetAssiMaster

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT
		X1 								:INT;    //Cart X
		X2      						:INT; 
	END_VAR

	VAR

		dret				:DINT;
		//StatoBuffer	:		StructRetMotionBuffer;
		
		CURRENT_STATE				:INT:=GENERAL_STATE#STATE_INIT;
		ResetMasterAxisBLM	:FB_ResetMasterAxisBLM;
	END_VAR
	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			// StatoBuffer:=_getstateofmotionbuffer(axis:=Axes[X1].masteraxis);  // primo di utilisare varifica tempo per reset
			// IF StatoBuffer.motionBufferState<>EMPTY THEN
			// 	dret:=_resetMotionBuffer(axis:=Axes[X1].masteraxis); 
			// END_IF;
			CURRENT_STATE:=_FB_ResetAssiMaster#STATE_RESET_ASSI_MASTER_RESET_MASTER_1;
			
		_FB_ResetAssiMaster#STATE_RESET_ASSI_MASTER_RESET_MASTER_1:
			IF (ResetMasterAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetMasterAxisBLM(Axis:=Axes[X1].masteraxis(* , attesa:=IMMEDIATELY*));
			ELSE
				ResetMasterAxisBLM(Axis:=Axes[X1].masteraxis (* , attesa:=IMMEDIATELY*));
				(*{#ifdef DH4010}*)
				// 	CURRENT_STATE:=_FB_ResetAssiMaster#STATE_RESET_ASSI_MASTER_RESET_BUFFER_2;
				// (*{#else}*)
				// 	CURRENT_STATE:=GENERAL_STATE#STATE_END;
				(*{#endif}*)
			END_IF;
			
		// _FB_ResetAssiMaster#STATE_RESET_ASSI_MASTER_RESET_BUFFER_2:  // primo di utilisare varifica tempo per reset
		// 	(*{#ifdef DH4010}*)
		// 		StatoBuffer:=_getstateofmotionbuffer(axis:=Axes[X2].masteraxis);
		// 		IF StatoBuffer.motionBufferState<>EMPTY THEN
		// 			dret:=_resetMotionBuffer(axis:=Axes[X2].masteraxis); 
		// 		END_IF;
		// 		CURRENT_STATE:=_FB_ResetAssiMaster#STATE_RESET_ASSI_MASTER_RESET_MASTER_2;
		// 	(*{#endif};*)
			
		_FB_ResetAssiMaster#STATE_RESET_ASSI_MASTER_RESET_MASTER_2:
			(*{#ifdef DH4010}*)
				IF (ResetMasterAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
					ResetMasterAxisBLM(Axis:=Axes[X2].masteraxis (* , attesa:=IMMEDIATELY*));
				ELSE
					ResetMasterAxisBLM(Axis:=Axes[X2].masteraxis (* , attesa:=IMMEDIATELY*));
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			(*{#endif};*)
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
			
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK	

FUNCTION GetMasterAxisFromTO:INT

	VAR_EXTERNAL
		AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
	END_VAR

	VAR_EXTERNAL CONSTANT
		FIRST_AXIS_MASTER				:INT;
	END_VAR

	VAR_INPUT
		Axis	: DB_ANY;
	END_VAR
	
	VAR
		PosAx       : REF_TO TO_SynchronousAxis;
		LocAxis 	: REF_TO TO_SynchronousAxis;
		i			:INT;
	END_VAR

	GetMasterAxisFromTO:=-1;
	LocAxis := AsSynchronousAxisRef(Axis);
	FOR i:=0 TO _MAXAXIS_MASTER#MAXAXIS_MASTER-1 DO
	posax:= AsSynchronousAxisRef(AssiMaster[i].PosAx);	
		IF (PosAx=LocAxis) THEN
			GetMasterAxisFromTO:=i;
		END_IF;
	END_FOR;
END_FUNCTION

FUNCTION_BLOCK FB_ResetMasterAxisBLM

	VAR_EXTERNAL 
		AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
		mcReset_Master					:ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MC_Reset;
		
	END_VAR

	VAR_EXTERNAL CONSTANT
		FIRST_AXIS_MASTER				:INT;
	END_VAR

	VAR_INPUT
		Axis: DB_ANY;//REF_TO TO_SynchronousAxis;
		//Attesa: EnumNextCommandReset;
	END_VAR
	VAR
		CURRENT_STATE:INT:=GENERAL_STATE#STATE_INIT;
		dret:LREAL;
	END_VAR
	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			AssiMaster[GetMasterAxisFromTO(Axis)].MC_Axes.reset_axes :=TRUE;
			IF AssiMaster[GetMasterAxisFromTO(Axis)].MC_Axes.State_reset.Done THEN
				CURRENT_STATE:=_FB_ResetMasterAxisBLM#STATE_RESET_MASTER_AXIS_BLM_WAIT_PLCOPEN;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;

		_FB_ResetMasterAxisBLM#STATE_RESET_MASTER_AXIS_BLM_WAIT_PLCOPEN:
			IF (mcReset_Master[GetMasterAxisFromTO(Axis)].done) THEN
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;

			(*IF UsoPLCOpen THEN
				AssiMaster[GetMasterAxisFromTO(Asse)].mcResetAxisInfo.Execute:=TRUE;
				IF Attesa=WHEN_COMMAND_DONE THEN
					CURRENT_STATE:=_FB_ResetMasterAxisBLM#STATE_RESET_MASTER_AXIS_BLM_WAIT_PLCOPEN;
				ELSE
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			ELSE
				CURRENT_STATE:=_FB_ResetMasterAxisBLM#STATE_RESET_MASTER_AXIS_BLM_NO_PLCOPEN;
			END_IF;
			
		_FB_ResetMasterAxisBLM#STATE_RESET_MASTER_AXIS_BLM_WAIT_PLCOPEN:
			IF (mcReset_Master[GetMasterAxisFromTO(Asse)].done) THEN
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;*)
			
		(*STATE_RESET_MASTER_AXIS_BLM_NO_PLCOPEN:
			dret:=_resetAxis(axis:=Asse, nextCommand:=IMMEDIATELY);
			IF attesa=WHEN_COMMAND_DONE THEN
				IF NOT (dret=7001 OR dret=7002) THEN
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;*)
			
			GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK


FUNCTION TestMasterLavoro:BOOL

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT
		X1:INT;
		X2:INT;
		POSITION_EXTRA_MASTER   :REAL;
	END_VAR

	VAR
		masteraxis1 : REF_TO TO_PositioningAxis;
		masteraxis2 : REF_TO TO_PositioningAxis;
	END_VAR
	;
	{::ifdef DH4010}
		masteraxis1:=AsPositioningAxisRef(Axes[X1].masteraxis);
		masteraxis2:=AsPositioningAxisRef(Axes[X2].masteraxis); ///// ?????-
		TestMasterLavoro:=masteraxis1^.actualPosition>-POSITION_EXTRA_MASTER OR
											masteraxis2^.actualPosition>-POSITION_EXTRA_MASTER;
	{::else}	//Fine DH4010
		masteraxis1:=AsPositioningAxisRef(Axes[X1].masteraxis);
		TestMasterLavoro:=masteraxis1^.actualPosition>-POSITION_EXTRA_MASTER;
	{::endif}	//Fine NO DH4010

END_FUNCTION	

/***********************************************************************************************
 * RETURNS THE VALUE OF THE MASTER AXIS OVERRIDE (THE LESSER OF THE TWO IN THE CASE OF DH4010) *
 ***********************************************************************************************/

FUNCTION GetMasterOverride:LREAL
	
	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT
		X1:INT;
		X2:INT;
	END_VAR

	VAR
		masteraxisX1 : REF_TO TO_PositioningAxis;
		masteraxisX2 : REF_TO TO_PositioningAxis;

	END_VAR

	// {
	// #ifdef DH4010
	// }
		masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis);
		masteraxisX2:= AsPositioningAxisRef(Axes[X1].masteraxis);
		// GetMasterOverride:=MIN(masteraxisX1^._Override.velocity, masteraxisX2^._Override.velocity); ****
	// {
	// #else	//Fine DH4010
	// }
		// GetMasterOverride:=masteraxisX1^._Override.velocity;
	// {
	// #endif	//Fine NO DH4010
	// }
END_FUNCTION	

/***********************************************************************************************************************
 * RETURNS TRUE IF THE SPEED OF THE MASTER AXIS (OF AT LEAST ONE OF THE TWO IN THE CASE OF THE DH4010) IS GREATER THAN 0 *
 *                                             AT THE CAM START DIMENSION                                              *
 ***********************************************************************************************************************/

FUNCTION TestMasterVeloc:BOOL

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT
		X1:INT;
		X2:INT;
	END_VAR

	VAR
		masteraxisX1 : REF_TO TO_PositioningAxis;
		masteraxisX2 : REF_TO TO_PositioningAxis;

	END_VAR
	;
	// {
	// #ifdef DH4010 %%%%
	// }
		// masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis);
		// masteraxisX2:= AsPositioningAxisRef(Axes[X1].masteraxis);
		// TestMasterVeloc:=masteraxisX1^.motionstatedata.commandvelocity>0.005 OR
		// 				masteraxisX2^.motionstatedata.commandvelocity>0.005;
	// {
	// #else	//Fine DH4010
	// }
		// TestMasterVeloc:=masteraxisX1^.motionstatedata.commandvelocity>0.005;
	// {
	// #endif	//Fine NO DH4010
	// }
END_FUNCTION 

/***********************************************************************************************
 * RETURNS THE VALUE OF THE MASTER AXIS OVERRIDE (THE LESSER OF THE TWO IN THE CASE OF DH4010) *
 ***********************************************************************************************/
 
FUNCTION GetMasterMaxAcceleration:LREAL

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT
		X1:INT;
		X2:INT;
	END_VAR

	VAR
		masteraxisX1 : REF_TO TO_PositioningAxis;
		masteraxisX2 : REF_TO TO_PositioningAxis;

	END_VAR
	;

	// {
	// #ifdef DH4010 ****
	// }
	// 	masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis);
	// 	masteraxisX2:= AsPositioningAxisRef(Axes[X1].masteraxis);
	// 	// GetMasterMaxAcceleration:=MAX(masteraxisX1^.DynamicLimits.MaxAcceleration, ****
	// 									// masteraxisX2^.DynamicLimits.MaxAcceleration);
	// 	// GetMasterMaxAcceleration:=MAX(masteraxisX1^.activeconfigdata.TypeOfAxis.MaxAcceleration.Maximum, ****
	// 	// 															Assi[X2].masteraxis.activeconfigdata.TypeOfAxis.MaxAcceleration.Maximum);
	// // {
	// // #else	//Fine DH4010
	// // }
	// 	GetMasterMaxAcceleration:=masteraxisX1^.DynamicLimits.MaxAcceleration;
	// {
	// #endif	//Fine NO DH4010
	// }
END_FUNCTION

/**********************************************************
 * SET THE ACCELERATION OVERRIDE VALUE OF THE MASTER AXIS *
 **********************************************************/

FUNCTION SetMasterMaxAcceleration

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT
		X1:INT;
		X2:INT;
	END_VAR

	VAR
		masteraxisX1 : REF_TO TO_PositioningAxis;
		masteraxisX2 : REF_TO TO_PositioningAxis;

	END_VAR

	VAR_INPUT
		Value	:LREAL;
	END_VAR
	;
	// {
	// #ifdef DH4010 ****
	// }
	// 	masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis);
	// 	masteraxisX2:= AsPositioningAxisRef(Axes[X1].masteraxis);
	// 	masteraxisX1^.DynamicLimits.MaxAcceleration:=Value;
	// 	masteraxisX2^.DynamicLimits.MaxAcceleration:=Value;
	// // {
	// // #else	//Fine DH4010
	// // }
	// 	masteraxisX1^.DynamicLimits.MaxAcceleration:=Value;
	// // {
	// // #endif	//Fine NO DH4010
	// }
END_FUNCTION	

/********************************************
 * SET THE MASTER AXIS SPEED OVERRIDE VALUE *
 ********************************************/

FUNCTION SetMasterVelocity
	
	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT
		X1:INT;
		X2:INT;
	END_VAR

	VAR
		masteraxisX1 : REF_TO TO_PositioningAxis;
		masteraxisX2 : REF_TO TO_PositioningAxis;

	END_VAR

	VAR_INPUT
		Value	:LREAL;
	END_VAR
	;
	{::ifdef DH4010}
		masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis); 
		masteraxisX2:= AsPositioningAxisRef(Axes[X2].masteraxis);
		masteraxisX1^._Override.velocity:=Value;
		masteraxisX2^._Override.velocity:=Value;
	{::else}	//Fine DH4010
		masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis); 
		masteraxisX1^._Override.velocity:=Value;
	{::endif}	//Fine NO DH4010
	
END_FUNCTION

//Imposta la posizione dell^asse master nella variabile PosMaster nella struttura Programmi

FUNCTION SetPosizioneMaster

	VAR_EXTERNAL
		Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT
		X1:INT;
		X2:INT;
	END_VAR

	VAR
		masteraxisX1 : REF_TO TO_PositioningAxis;
		masteraxisX2 : REF_TO TO_PositioningAxis;

	END_VAR

	VAR_INPUT
		Programma	:INT;
	END_VAR
	;
	{::ifdef DH4010}
		masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis);
		masteraxisX2:= AsPositioningAxisRef(Axes[X1].masteraxis);
		Programmi[Programma].PosMasterSx:=masteraxisX1^.actualPosition;
		Programmi[Programma].PosMasterDx:=masteraxisX2^.actualPosition;
	{::else}	//Fine DH4010
		masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis);
		Programmi[Programma].PosMaster:=masteraxisX1^.actualPosition;
	{::endif}	//Fine NO DH4010
	
END_FUNCTION	

//Restituisce true se l^oggetto tecnologico PosAsse è attivo.
//La FUNCTION è realmente funzionante solo se siamo in simulazione, in quanto per evitare
//di bloccare il background e l^ipo viene interrogato lo stato del TO con IMMEDIATELY, quindi
//non posso considerare l^esito della risposta sicuro. Nella realtà delle macchine il caso di
//accedere ad un TO disattivato non dovrebbe accadere, se servisse si dovrà risolvere il problema
//prima indicato. 
FUNCTION TestToAttivo:BOOL

	VAR_EXTERNAL
		pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters; 
	END_VAR

	VAR_INPUT
		Asse	:INT;
	END_VAR

	{::ifdef EMULAZIONE}
		IF Asse>=_FIRST_AXIS#FIRST_AXIS AND Asse<_MAXIS#MAXAXIS THEN
			IF Pm[Asse].ma_Asse_Abilitato=TRUE THEN
				TestToAttivo:=TRUE;
				(*		
				Stato:=_getstateofto(to_instance:=Assi[Asse].PosAx,
														 reqactdeactgetstatemode:=REQUEST_TRUE,
														 commandid:=_getCommandId(),
														 nextcommand:=IMMEDIATELY);
				IF Stato.commandidstate=ACTIVE THEN
					TestToAttivo:=TRUE;
				ELSE
					TestToAttivo:=FALSE;
				END_IF;
				*)
			ELSE
				TestToAttivo:=FALSE;
			END_IF;
		END_IF;
	{::else}	//Fine EMULAZIONE
	
		TestToAttivo:=TRUE;
	{::endif}	//Fine NO EMULAZIONE
	
END_FUNCTION

//Resituisce la posizione dell^asse master. 
FUNCTION PosizioneMaster:LREAL

		VAR_EXTERNAL
			Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
		END_VAR

		VAR_EXTERNAL CONSTANT
			X1 :INT;
			X2 :INT;
		END_VAR

		VAR_INPUT
			Tabella	:SINT;
		END_VAR

		VAR
			masteraxisX1 : REF_TO TO_PositioningAxis;
			masteraxisX2 : REF_TO TO_PositioningAxis;
		END_VAR

		masteraxisX1:= AsPositioningAxisRef(Axes[X1].masteraxis);
		masteraxisX2:= AsPositioningAxisRef(Axes[X2].masteraxis);
		IF Tabella=TABLE#TABLE_LX THEN
			PosizioneMaster:=masteraxisX1^.actualPosition;
		ELSE
			PosizioneMaster:=masteraxisX2^.actualPosition;
		END_IF;
	// {
	// #else	//Fine DH4010
	// }
		PosizioneMaster:=masteraxisX1^.actualPosition;
	// {
	// #endif	//Fine NO DH4010
	// }
END_FUNCTION	

FUNCTION_BLOCK FB_EnableAsseInCoppia

	VAR_INPUT
		Coppia	 	:LREAL;
		NumeroAsse:DINT;
		Attesa		:BOOL:=TRUE;
	END_VAR
	VAR
		dret		  :DINT;
		AppoCoppia:LREAL;
		//Asse		  :PosAxis;
		
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
	END_VAR
	VAR_OUTPUT
		OutputState		:INT;
	END_VAR
	;
	// CASE CURRENT_STATE OF
	// 	STATE_INIT:
	// 		Asse:=Assi[NumeroAsse].PosAx;
	// 		AppoCoppia:=MIN(Coppia, 100);
	// 		// {#ifdef SLITTA_MOBILE_DX}
	// 			IF NumeroAsse=U4 THEN
	// 				//Eliminazione scalatura camma asse slitta mobile solo se l^asse parte
	// 				//da indietro
	// 				IF Assi[NumeroAsse].PosAx.positioningState.actualPosition<1 THEN
	// 					ResetScalaturaParzialeCamma(Asse:=NumeroAsse);
	// 				END_IF;
	// 			END_IF;
	// 		// {#endif}	//Fine SLITTA_MOBILE_DX		
			
	// 		// {#ifdef SLITTA_MOBILE_SX}
	// 			IF NumeroAsse=U5 THEN
	// 				IF Assi[NumeroAsse].PosAx.positioningState.actualPosition<1 THEN
	// 					ResetScalaturaParzialeCamma(Asse:=NumeroAsse);
	// 				END_IF;
	// 			END_IF;
	// 		// {#endif}	//Fine SLITTA_MOBILE_SX
			
	// 		IF Assi[NumeroAsse].posax.typeofaxis<>VIRTUAL_AXIS AND pm[NumeroAsse].ma_asse_installato<>ASSE_DISATTIVABILE_DISATTIVO THEN
	// 			IF Asse.torquelimitingcommand.state=INACTIVE OR PercentualeCoppiaImpostata[NumeroAsse]<>AppoCoppia THEN
	// 				//Esecuzione del comando solo se l^asse non è già in limitazione, oppure la limitazione
	// 				//richiesta è diversa da quella attiva
	// 				PercentualeCoppiaImpostata[NumeroAsse]:=AppoCoppia;
	// 				NmCoppiaImpostata[NumeroAsse]:=AppoCoppia/100*pm[NumeroAsse].az_r1538;
	// 				dret:=_enableTorqueLimiting(axis:=Asse, 
	// 																		torqueLimitType:=DIRECT, 
	// 																		torqueLimit:=NmCoppiaImpostata[NumeroAsse], 
	// 																		nextCommand:=IMMEDIATELY, 
	// 																		commandId:=_getCommandId(),
	// 																		torqueLimitUnit:=TORQUE );
	// 				IF Attesa=TRUE THEN
	// 					CURRENT_STATE:=STATE_ENABLE_ASSE_IN_COPPIA_WAIT;
	// 				ELSE
	// 					AsseInLimitazioneCoppia[NumeroAsse]:=TRUE;
	// 					CURRENT_STATE:=STATE_END;
	// 				END_IF;
	// 			ELSE
	// 				AsseInLimitazioneCoppia[NumeroAsse]:=TRUE;
	// 				CURRENT_STATE:=STATE_END;
	// 			END_IF;
	// 		ELSE
	// 			CURRENT_STATE:=STATE_END;
	// 		END_IF;
			
	// 	STATE_ENABLE_ASSE_IN_COPPIA_WAIT:
	// 		IF Asse.torquelimitingcommand.state=INACTIVE THEN
	// 			// {#ifdef PROFINET}
	// 				IF I_Barriera=FALSE THEN
	// 					AsseInLimitazioneCoppia[NumeroAsse]:=TRUE;
	// 					CURRENT_STATE:=STATE_END;
	// 				END_IF;
	// 			// {#endif}
	// 		ELSE
	// 			AsseInLimitazioneCoppia[NumeroAsse]:=TRUE;
	// 			CURRENT_STATE:=STATE_END;
	// 		END_IF;
			
	// 	STATE_END:
	// 		CURRENT_STATE:=STATE_INIT;
			
	// END_CASE;
	// OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK	

FUNCTION_BLOCK FB_DisableAsseInCoppia
	VAR_INPUT
		NumeroAsse	:DINT;
		Verifica		:BOOL;
	END_VAR
	VAR
		dret				:DINT;
		//Asse				:PosAxis;
		
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
	END_VAR
	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	;
	// CASE CURRENT_STATE OF
	// 	GENERAL_STATE#STATE_INIT:
	// 		IF TestToAttivo(Asse:=TO_INT(NumeroAsse))=TRUE THEN
	// 			Asse:=Assi[NumeroAsse].PosAx;
	// 			PercentualeCoppiaImpostata[NumeroAsse]:=MY_BIG_VALUE_REAL;
	// 			IF Assi[NumeroAsse].posax.typeofaxis<>VIRTUAL_AXIS THEN
	// 				IF pm[NumeroAsse].ma_asse_installato<>ASSE_DISATTIVABILE_DISATTIVO THEN
	// 					IF Asse.torquelimitingcommand.state=ACTIVE THEN
	// 						dret:=_disableTorqueLimiting(axis:=Asse, 
	// 																				 nextCommand:=IMMEDIATELY,
	// 																				 commandId:=_getCommandId());
	// 						IF (Verifica) THEN
	// 							CURRENT_STATE:=STATE_DISABLE_ASSE_IN_COPPIA_WAIT;
	// 						ELSE
	// 							CURRENT_STATE:=GENERAL_STATE#STATE_END;
	// 						END_IF;
	// 					ELSE
	// 						CURRENT_STATE:=GENERAL_STATE#STATE_END;
	// 					END_IF;
	// 				ELSE
	// 					AsseInLimitazioneCoppia[NumeroAsse]:=FALSE;
	// 					CURRENT_STATE:=GENERAL_STATE#STATE_END;
	// 				END_IF;
	// 			ELSE
	// 				CURRENT_STATE:=GENERAL_STATE#STATE_END;
	// 			END_IF;
	// 		ELSE
	// 			CURRENT_STATE:=GENERAL_STATE#STATE_END;
	// 		END_IF;
			
	// 	STATE_DISABLE_ASSE_IN_COPPIA_WAIT:
	// 		IF Asse.torquelimitingcommand.state<>ACTIVE THEN
	// 			AsseInLimitazioneCoppia[NumeroAsse]:=FALSE;
	// 			CURRENT_STATE:=GENERAL_STATE#STATE_END;
	// 		END_IF;
			
	// 	GENERAL_STATE#STATE_END:
	// 		CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	
	// END_CASE;
	// OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK	

FUNCTION_BLOCK FB_ResetAllCamme

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_EXTERNAL CONSTANT

		TEMPO_ARRESTO_BARRIERA  :REAL;

	END_VAR

	VAR_INPUT
		//ProssimoComando	:EnumNextCommandEnable:=WHEN_MOTION_DONE;
	END_VAR
	VAR
					
		PosAx 						: REF_TO TO_SynchronousAxis;
		dret						:DINT;
		i							:INT;
		
		CURRENT_STATE		:INT:=GENERAL_STATE#STATE_INIT;
		CURRENT_AXIS		:INT;
		DisabilitaCamma	:FB_DisabilitaCamma;
		StopAssiMaster	:FB_StopAssiMaster;
	END_VAR
	VAR_OUTPUT
		OutputState			:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
			CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_STOP_MASTER;
		
			_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_STOP_MASTER:
			IF (StopAssiMaster.outputState<>GENERAL_STATE#STATE_END) THEN
				StopAssiMaster(AccelerazioneMaster:=1/TEMPO_ARRESTO_BARRIERA (* , ProssimoComando:=ProssimoComando*));
			ELSE
				StopAssiMaster(AccelerazioneMaster:=1/TEMPO_ARRESTO_BARRIERA (* , ProssimoComando:=ProssimoComando*));
				CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_LOOP_ASSI;
			END_IF;
					
			_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_LOOP_ASSI:
			CURRENT_AXIS:=CURRENT_AXIS+1;
			IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
				Axes[CURRENT_AXIS].StatoCamming:=FALSE;
				IF TestToAttivo(Asse:=CURRENT_AXIS) THEN
					CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_DISABILITA_CAMMA;
				END_IF;		
			ELSE
				CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
				CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_WAIT_ASSI_ACTIVE;
			END_IF;
			 
			_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_DISABILITA_CAMMA:
			IF (DisabilitaCamma.OutputState<>GENERAL_STATE#STATE_END) THEN
				//Disabilitazione stato asse in camma
				DisabilitaCamma(Asse:=CURRENT_AXIS, (*ProssimoComando:=ProssimoComando,*) SgancioCamma:=TRUE, EsegueReset:=FALSE);
			ELSE
				DisabilitaCamma(Asse:=CURRENT_AXIS, (*ProssimoComando:=ProssimoComando,*) SgancioCamma:=TRUE, EsegueReset:=FALSE);
				PosAx := AsSynchronousAxisRef(Axes[CURRENT_AXIS].SincroAsse);
				IF PosAx^.StatusSynchronizedMotion.ActualMaster <> UINT#0 (*TO#NIL*)  THEN 
				//IF Assi[CURRENT_AXIS].SincroAsse.activeMaster<> NULL(*TO#NIL*)  THEN  
					// IF Axes[CURRENT_AXIS].camma.followingRangeSettings.scaleRange[0].scale<>1 THEN %%%%%
					// 	//Disabilitazione delle eventuali scalature delle camme
					// 	ResetScalaturaParzialeCamma(Asse:=CURRENT_AXIS);
					// 	CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_LOOP_ASSI;
					// ELSE
						CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_LOOP_ASSI;
					// END_IF;
				ELSE
					CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_LOOP_ASSI;
				END_IF;
			END_IF;
			
			_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_WAIT_ASSI_ACTIVE:
			CURRENT_AXIS:=CURRENT_AXIS+1;
			IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
				IF TestToAttivo(Asse:=CURRENT_AXIS) THEN
					CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_WAIT_ASSE;
				END_IF;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
			_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_WAIT_ASSE:
			PosAx := AsSynchronousAxisRef(Axes[CURRENT_AXIS].SincroAsse);
			IF NOT (PosAx^.StatusSynchronizedMotion.FunctionState <> 0 (*AND ProssimoComando=WHEN_MOTION_DONE*)) THEN
			//IF NOT (Axes[CURRENT_AXIS].SincroAsse.state<>INACTIVE AND ProssimoComando=WHEN_MOTION_DONE) THEN
				CURRENT_STATE:=_FB_ResetAllCamme#STATE_RESET_ALL_CAMME_WAIT_ASSI_ACTIVE;
			END_IF;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK   

/***************************************************
 * PARTIALLY ELIMINATES CAM SCALING (START TO END) *
 ***************************************************/
FUNCTION ResetScalaturaParzialeCamma

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR
	VAR_INPUT
		Asse				:DINT;
	END_VAR
	VAR
		PosAx 			:REF_TO TO_SynchronousAxis;
		cam 			:REF_TO TO_Cam;
		dret			:DINT;
	END_VAR
	;
	// IF TestToAttivo(Asse:=TO_INT(Asse))=TRUE THEN
	// 	PosAx:=AsSynchronousAxisRef(Axes[Asse].sincroasse);
	// 	IF PosAx^.StatusSynchronizedMotion.ActualMaster<> UINT#0 (*TO#NIL*) THEN
	// 		IF Axes[Asse].Camma.followingrangesettings.scale<>1 OR
	// 			 Axes[Asse].Camma.followingrangesettings.scalerange[0].scale<>1 OR 
	// 			 Axes[Asse].Camma.followingrangesettings.scalerange[1].scale<>1 THEN
	// 			dret:=_setCamScale(cam:=Axes[Asse].Camma,
	// 												 scalingRange:=FOLLOWING_RANGE,
	// 												 scalingSpecification:=SPECIFIC_PART_OF_RANGE, 
	// 												 specificRangeNumber:=1, 
	// 												 specificRangeStartPointType:=DIRECT, 
	// 												 specificRangeStartPoint:=-MY_BIG_VALUE_REAL,	   
	// 												 specificRangeEndPointType:=DIRECT, 
	// 												 specificRangeEndPoint:=MY_BIG_VALUE_REAL, 
	// 												 scaleValue:=1);
	// 		END_IF;
	// 	END_IF;
	// END_IF;
END_FUNCTION	

FUNCTION_BLOCK FB_DisabilitaCamma

	VAR_EXTERNAL
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
		UsoPLCOpen						:BOOL;
	END_VAR

	VAR_INPUT
		Asse						:INT;
		ProssimoComando		:EnumNextCommandEnable;
		SgancioCamma		:BOOL;	//TRUE se l^asse deve essere sganciato dalla camma
		EsegueReset			:BOOL:=TRUE;
	END_VAR
	VAR
		dret						:DINT;
		PosAx : REF_TO TO_SynchronousAxis;
		CURRENT_STATE					:INT:=GENERAL_STATE#STATE_INIT;
		ResetMotionResetAxis	:FB_ResetMotionResetAxis;
		Attesa:BOOL;
	END_VAR
	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
		PosAx:=AsSynchronousAxisRef(Axes[Asse].sincroasse);
			IF TestToAttivo(Asse:=Asse)=TRUE AND PosAx^.StatusSynchronizedMotion.FunctionState<>0 THEN //Axes[Asse].SincroAsse.state<>INACTIVE THEN
				IF (EsegueReset) THEN
					IF ProssimoComando=EnumNextCommandEnable#IMMEDIATELY THEN
						Attesa:=FALSE;
					ELSE
						Attesa:=TRUE;
					END_IF;
					CURRENT_STATE:=_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_RESET_MOTION;
				ELSE	
					CURRENT_STATE:=_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_DISABLE_CAMMING;
				END_IF;
			ELSE
				CURRENT_STATE:=_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_SGANCIO_CAMMA;
			END_IF;
			
			_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_RESET_MOTION:
			IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetMotionResetAxis(Asse:=Asse, Attesa:=Attesa);
			ELSE
				ResetMotionResetAxis(Asse:=Asse, Attesa:=Attesa);
				IF (UsoPLCOpen) THEN
					//se si usano i blocchi PLCOpen, non è sufficiente eseguire il reset. 
					//bisogna eseguire esplicitamente la disabilitazione della camma
					CURRENT_STATE:=_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_DISABLE_CAMMING;
				ELSE
					CURRENT_STATE:=_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_SGANCIO_CAMMA;
				END_IF;
			END_IF;
			
			_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_DISABLE_CAMMING:
			Axes[Asse].MC_Axes.cam_out_param.SyncProfileReference := 0;
			//Axes[Asse].mcCamOutInfo.SyncOutDirection:= ????
			//dret:=_disableCamming(followingObject:=Axes[Asse].SincroAsse, 
											// syncOffMode:=IMMEDIATELY,
											// syncProfileReference:=RELATE_SYNC_PROFILE_TO_TIME, 
											// synchronizingDirection:=USER_DEFAULT, 
											// mergeMode:=IMMEDIATELY, 
											// nextCommand:=ProssimoComando,
											// commandId:=_getCommandId());
				IF Axes[Asse].MC_Axes.State_cam_out.Busy THEN								
				//IF (ProssimoComando<>EnumNextCommandEnable#IMMEDIATELY) THEN
				//IF NOT (dret=7001 OR dret=7002) THEN
					CURRENT_STATE:=_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_SGANCIO_CAMMA;
				// END_IF;
				ELSE
					CURRENT_STATE:=_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_SGANCIO_CAMMA;
				END_IF;
			
			_FB_DisabilitaCamma#STATE_DISABILITA_CAMMA_SGANCIO_CAMMA:
			IF SgancioCamma=TRUE THEN
				//Serve per gestire la rimessa in camma dell^asse a seguito della ripartenza dell^asse dopo
				//una violazione della barriera di sicurezza
				Axes[Asse].statocamming:=FALSE;
			END_IF;
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
			
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK  

FUNCTION ResetOggettiVettoriali

	VAR_EXTERNAL CONSTANT
		KinAxesY1_U4  														:UINT;
		KinAxesQ1_V1_Z1                   									:UINT;
		AxisKinematics                 										 :ARRAY[_FIRST_KIN_AXES#FIRST_KIN_AXES.._MAX_KIN_AXES#MAX_KIN_AXES-1] OF Kinematics;
	END_VAR

	VAR
		dret					:DINT;
		KinAxesY1_U4_def     : DB_ANY;
		Y1_U4             	: REF_TO TO_Kinematics;
		KinAxesQ1_V1_Z1_def     : DB_ANY;
		Q1_V1_Z1             	: REF_TO TO_Kinematics;
	END_VAR

	VAR_OUTPUT
		done					: BOOL;
	END_VAR
	// {
	// #ifndef ORBITALE %%%%
	// }
	// 	IF X1_U2.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=X1_U2, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[X1].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[U2].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// {
	// #endif	//Fine NO ORBITALE
	// }
	// {
	// #ifdef INTERPOLA_Y1_X1
	// }
	// 	IF Y1_X1.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=Y1_X1, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[X1].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[Y1].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// {
	// #endif	//Fine INTERPOLA_Y1_X1
	// }
	// {
	// #ifdef INTERPOLA_X3_X6
	// }
	// 	IF X3_X6.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=X3_X6, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[X3].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[X6].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// {
	// #endif	//Fine INTERPOLA_X3_X6
	// }
	// {
	// #ifdef INTERPOLA_X2_X6
	// }
	// 	IF X2_X6.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=X2_X6, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[X2].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[X6].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// {
	// #endif	//Fine INTERPOLA_X2_X6
	// }
	// {
	// #ifdef INTERPOLA_X1_Y1
	// }
	// 	IF X1_Y1.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=X1_Y1, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[X1].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[Y1].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// {
	// #endif	//Fine INTERPOLA_X1_Y1
	// }
	// {
	// #ifdef INTERPOLA_X2_Y2
	// }
	// 	IF X2_Y2.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=X2_Y2, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[X2].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[Y2].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// {
	// #endif	//Fine INTERPOLA_X2_Y2
	// }
	// {::ifdef SLITTA_MOBILE_DX}
		Y1_U4:= AsKinematicsRef(KinAxesY1_U4_def);
		Done := FALSE; 
		IF Y1_U4^.StatusWord.%X8 THEN //linear command active
		//IF Y1_U4.linearpathcommand.state=ACTIVE THEN
		AxisKinematics[KinAxesY1_U4_def].mc_GroupStop.Execute:= TRUE;
		AxisKinematics[KinAxesY1_U4_def].mc_GroupStop.Mode := 0;
		// 	dret:=_stopPath(pathobject:=Y1_U4, 
		// 									stopmode:=STOP_AND_ABORT, 
		// 									velocityProfile:=USER_DEFAULT, 
		// 									mergeMode:=IMMEDIATELY, 
		// 									nextCommand:=IMMEDIATELY, 
		// 									commandId:=_getCommandId());
		// 	dret:=_waitTime(T#0ms);
		// 	dret:=_resetMotionBuffer(axis:=Assi[Y1].PosAx);
		// 	dret:=_resetMotionBuffer(axis:=Assi[U4].PosAx);
		// 	dret:=_waitTime(T#0ms);
	ELSIF AxisKinematics[KinAxesY1_U4_def].mc_GroupStop.Done THEN
		AxisKinematics[KinAxesY1_U4_def].mc_GroupStop.Execute:= FALSE;
		Done:= TRUE; 

END_IF;
	// {::endif}	//Fine SLITTA_MOBILE_DX
	
	// {
	// #ifdef SLITTA_MOBILE_SX
	// }
	// 	IF Y1_U5.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=Y1_U5, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[Y1].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[U5].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// {
	// #endif	//Fine SLITTA_MOBILE_SX
	// }
	// {
	// #ifndef ETURN
	// }
	// 	{
	// 	#ifndef ORBITALE
	// 	}
	// 		{
	// 		#ifndef ELECTMRL
	// 		}
	// 			IF W3_W4_Q1.linearpathcommand.state=ACTIVE THEN
	// 				dret:=_stopPath(pathobject:=W3_W4_Q1, 
	// 												stopmode:=STOP_AND_ABORT, 
	// 												velocityProfile:=USER_DEFAULT, 
	// 												mergeMode:=IMMEDIATELY, 
	// 												nextCommand:=IMMEDIATELY, 
	// 												commandId:=_getCommandId());
	// 				dret:=_waitTime(T#0ms);
	// 				dret:=_resetMotionBuffer(axis:=Assi[W3].PosAx);
	// 				dret:=_resetMotionBuffer(axis:=Assi[W4].PosAx);
	// 				dret:=_resetMotionBuffer(axis:=Assi[Q1].PosAx);
	// 				dret:=_waitTime(T#0ms);
	// 			END_IF;
	// 		{
	// 		#else //FINE NO ELECTMRL
	// 		}	
	// 			IF W3_W4_W6_Q1.linearpathcommand.state=ACTIVE THEN
	// 				dret:=_stopPath(pathobject:=W3_W4_W6_Q1, 
	// 												stopmode:=STOP_AND_ABORT, 
	// 												velocityProfile:=USER_DEFAULT, 
	// 												mergeMode:=IMMEDIATELY, 
	// 												nextCommand:=IMMEDIATELY, 
	// 												commandId:=_getCommandId());
	// 				dret:=_waitTime(T#0ms);
	// 				dret:=_resetMotionBuffer(axis:=Assi[W3].PosAx);
	// 				dret:=_resetMotionBuffer(axis:=Assi[W4].PosAx);
	// 				dret:=_resetMotionBuffer(axis:=Assi[Q1].PosAx);
	// 				dret:=_resetMotionBuffer(Axis:=Assi[W6].PosAx);
	// 				dret:=_waitTime(T#0ms);
	// 			END_IF;
	// 		{
	// 		#endif //FINE ELECTMRL
	// 		}
	// 	{
	// 	#endif	//Fine NO ORBITALE
	// 	}
	// {
	// #endif	//Fine NO ETURN
	// }
	// {
	// #ifdef MATRICE_ELETTRICA
	// }
	// 	IF TestAsseY2()=TRUE THEN
	// 		IF Y1_U4_Y2.linearpathcommand.state=ACTIVE THEN
	// 			dret:=_stopPath(pathobject:=Y1_U4_Y2, 
	// 											stopmode:=STOP_AND_ABORT, 
	// 											velocityProfile:=USER_DEFAULT, 
	// 											mergeMode:=IMMEDIATELY, 
	// 											nextCommand:=IMMEDIATELY, 
	// 											commandId:=_getCommandId());
	// 			dret:=_waitTime(T#0ms);
	// 			dret:=_resetMotionBuffer(axis:=Assi[Y1].PosAx);
	// 			dret:=_resetMotionBuffer(axis:=Assi[U4].PosAx);
	// 			dret:=_resetMotionBuffer(axis:=Assi[Y2].PosAx);
	// 			dret:=_waitTime(T#0ms);
	// 		END_IF;
	// 		IF Y1_Y2.linearpathcommand.state=ACTIVE THEN
	// 			dret:=_stopPath(pathobject:=Y1_Y2, 
	// 											stopmode:=STOP_AND_ABORT, 
	// 											velocityProfile:=USER_DEFAULT, 
	// 											mergeMode:=IMMEDIATELY, 
	// 											nextCommand:=IMMEDIATELY, 
	// 											commandId:=_getCommandId());
	// 			dret:=_waitTime(T#0ms);
	// 			dret:=_resetMotionBuffer(axis:=Assi[Y1].PosAx);
	// 			dret:=_resetMotionBuffer(axis:=Assi[Y2].PosAx);
	// 			dret:=_waitTime(T#0ms);
	// 		END_IF;
	// 	END_IF;
	// {
	// #endif	//Fine MATRICE_ELETTRICA
	// }
	// {
	// #ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO
	// }
	// 	IF Q1_V1_V4.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=Q1_V1_V4, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[Q1].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[V1].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[V4].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// 	IF Q1_V1.linearpathcommand.state=ACTIVE THEN
	// 		dret:=_stopPath(pathobject:=Q1_V1, 
	// 										stopmode:=STOP_AND_ABORT, 
	// 										velocityProfile:=USER_DEFAULT, 
	// 										mergeMode:=IMMEDIATELY, 
	// 										nextCommand:=IMMEDIATELY, 
	// 										commandId:=_getCommandId());
	// 		dret:=_waitTime(T#0ms);
	// 		dret:=_resetMotionBuffer(axis:=Assi[Q1].PosAx);
	// 		dret:=_resetMotionBuffer(axis:=Assi[V1].PosAx);
	// 		dret:=_waitTime(T#0ms);
	// 	END_IF;
	// {
	// #endif	//Fine SOLLEVAMENTO_SLITTA_SPALLAMENTO
	// }
	{::ifdef INTERPOLA_Q1_V1_Z1}
		Q1_V1_Z1:= AsKinematicsRef(KinAxesQ1_V1_Z1_def);
		IF Q1_V1_Z1^.StatusWord.%X8  THEN
			Done:= FALSE; 
		// IF Q1_V1_Z1.linearpathcommand.state=ACTIVE THEN
		AxisKinematics[KinAxesQ1_V1_Z1_def].mc_GroupStop.Execute:= TRUE;
		AxisKinematics[KinAxesQ1_V1_Z1_def].mc_GroupStop.Mode := 0;
			// dret:=_stopPath(pathobject:=Q1_V1_Z1, 
			// 								stopmode:=STOP_AND_ABORT, 
			// 								velocityProfile:=USER_DEFAULT, 
			// 								mergeMode:=IMMEDIATELY, 
			// 								nextCommand:=IMMEDIATELY, 
			// 								commandId:=_getCommandId());
			// dret:=_waitTime(T#0ms);
			// dret:=_resetMotionBuffer(axis:=Assi[Q1].PosAx);
			// dret:=_resetMotionBuffer(axis:=Assi[V1].PosAx);
			// dret:=_resetMotionBuffer(axis:=Assi[Z1].PosAx);
			// dret:=_waitTime(T#0ms);
			ELSIF AxisKinematics[KinAxesQ1_V1_Z1_def].mc_GroupStop.Done THEN
				AxisKinematics[KinAxesQ1_V1_Z1_def].mc_GroupStop.Execute:= FALSE;
				Done:= TRUE; 

		END_IF;

	{::endif}	//Fine INTERPOLA_Q1_V1_Z1
	
	;
END_FUNCTION	

END_NAMESPACE