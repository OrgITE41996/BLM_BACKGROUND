USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING Siemens.Simatic.S71500.Clocks;
USING System.BitAccess;
USING System.Math;
USING Simatic.Ax.LAcycCom;
USING _STATE_CONST;
USING ENABLE_FC;
USING _FC_FB;
USING ENABLE_FC;
using EnableAxFunction;
using _FC_FC;
USING _FC_IO_FC;
USING ENABLEAX_FUNCTIONS;
using _TYPE;
USING _CONST_VAR_DEF;
USING FC_N;

NAMESPACE ENABLEPROGRAM

PROGRAM EnableAX

    VAR_EXTERNAL 
		Manipolatore2D_Attivo					:BOOL; 
		Appo_global                             :INT;
		Disp                           			:ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
		Axes                            		:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
        //System_Timer                : LDATE_AND_TIME; //EX RTC
		O_EmergencyFromSimotion		    		:BOOL;	//Only to be able to compile machines without safety PLCs
		i_sys_first_on	                		:BOOL;
		i_sys_RunOk                     		: INT; 
		i_sys_error_grave              			: INT;
		i_sys_ErrorLubr                			:BOOL;
		SystemStarted                  			:BOOL;
		cfg                             		:_TYPE.Config_Struct;
		I_Barrier                      			:BOOL;
		IO_Link_Input,
		IO_Link_Output				   			:WORD;
		i_sys_Motors_Verified	       			:BOOL; //Alla perdita delle ritentive verifica lo stato di impostazione dei motori presenti in macchina
		TEST_S                          		:INT;
		i_sys_ACCESSLEVEL                       : INT;	// LIVELLO DI ACCESSO 
														// è il minore fra sw_ACCESSLEVEL e hw_ACCESSLEVEL
														// 0 = PROGRAMMATORE COSTRUTTORE
														// 1 = COSTRUTTORE
														// 2 = MIS COSTRUTTORE
														// 3 = SAT COSTRUTTORE
														// 4 = MANUTENTORE CLIENTE
														// 5 = OPERATORE QUALIFICATO
														// 6 = OPERATORE CLIENTE
														// 7 = UTENTE QUALSIASI
		i_sys_CmdVGPCSettings          			:BOOL; // Disables the system startup waiting to allow
                                                        // the transfer of the command from VGO3D of the machine configuration.
                                                        // It's meant to prevent CPU stops for premature piece transfers,
                                                        // but it should allow some transfers to be executed in order to
                                                        // complete the system startup (CMD disabling motors not present in the machine).
		i_sys_STOPCOND                			: INT ; // 0=NO STOP, 1=STOP ACTIV
		PerformSafetyReset	            		:BOOL;
		O_Vofa_1_2_Enable               		: BOOL;
		IAddress                        		: ARRAY [0..MAXDIGITALINPUT] OF Digital_Input;
		OAddress                        		: ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
		O_PulserTwoHands                		: BOOL;
		PulserTwoHands				    		:BOOL;//TRUE enables the hand control with reset and start functions if provided simultaneously
		(*{
        #ifdef PROFINET
        }*)
        PresencePnCoupler               :BOOL; //TRUE if the first PnCoupler must be present.
        (*{
        #ifndef PLC_S7_1500
        }*)
        PresencePnCoupler2			    :BOOL;			//TRUE se deve essere presente il secondo PnCoupler
        (*{
        #endif	//Fine PLC_S7_1500
         }
        {
        #endif  //FINE PROFINET 
        }*)
		i_sys_DpCoupler                 		:BOOL;
		i_sys_PnCoupler                 		:BOOL;
		O_PnCoupler_Disable             		:BOOL;
		(*{
		#ifndef SELEZIONE_DOPPIO_PNPN
		}*)
		O_PnCoupler_2_Disable		    		:BOOL;	//Required for machine versions that do not include the variable in their ADDRESS LIST.
		(*{
		#endif	//Fine NO SELEZIONE_DOPPIO_PNPN
		}*)
		ChecksumPlcSafety               		:UDINT;
		pm                              		: ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
		Configuration_IO                		:SINT;
		InitializeIOExecuted            		:BOOL; 
		i_sys_EnableBusy                		:BOOL; //Se a TRUE indica che è in corso il programma
                                                        //ENABLE_AX e quindi non può essere eseguito
                                                        //il reset
        i_sys_Busy                      		:BOOL;//Viene messa TRUE da BlmMotion al momento
                                                        //del trasferimento dei PM e viene resettata
                                                        //da Simotion alla fine della copia ram/rom
		i_sys_OVRSTEP_DEFAULT               	: LREAL; //value of OVR_STEP calculated in INITVAR on the^slowest axis   
		i_sys_acc_master                    	: REAL;  //Master acceli_sys_InUseReadingSinamicseration
		i_sys_OVRSTEP                       	: LREAL; // Number of steps (*2ms) to be able to make the master accelerate or decelerate
		i_sys_sense_curve                   	:INT;		//1=Macchina DX  -1=Macchina SX
		//PosAx                      				:DB_ANY;
		i_sys_control_incongruity_encoder   	:BOOL; 
		i_sys_statusencoder                  	:ARRAY[(_FIRST_AXIS#FIRST_AXIs)..(_FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1)] OF REAL;
		i_sys_MotorState						:ARRAY[(_FIRST_AXIS#FIRST_AXIS)..(_FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1)] OF REAL; 
		Checksum_Plc_Safety_Hight 				:WORD;
		Checksum_Plc_Safety_Low 				:WORD;
		Configurazione_IO_to_PLC              	:SINT;
		Strobe_Config_IO                       	:BOOL;
		Axis_Address                            : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF DINT;
	
		Start_read								: BOOL;
		Dummy_LREAL_1                        	:LREAL;
		Dummy_LREAL_2                            :LREAL; 
		Dummy_LREAL_3                            :LREAL;
		i_sys_InUseWritingSynamics      		:BOOL;
		i_sys_ReadingSinamicsInUse      		:BOOL;
		i_sys_Param_RW          				:INT;   // 1=READ 2=WRITE
		PM_Modified                				:BOOL;
		i_sys_Copy_RAM_ROM           			:BOOL;
		i_sys_Param_AX_Number		 			:INT;		// da 0..MAXASSI-1   999= ALIMENTATORE
		i_sys_Param_Number						:LREAL; // Numero parametro
		i_sys_Param_Index           			:LREAL; // Index parametro
		i_sys_Param_Value						:LREAL; // Valore parametro
		i_sys_InUseReadingSinamics  			:BOOL;
		StartInRun                  			:BOOL;
		Started									:BOOL;
		CURRENT_STATE						    :INT; //canc after Test
		i_sys_enable_Alimen 				    :BOOL;
		i_sys_optimize_Z_preset          		:INT;
		i_sys_ResetPressedDriver                :BOOL;
		OPS_Active                              :BOOL;  //True gestione speciale pezzo seguente per consentire funzionamento
													//macchina con sistema OPS di Salvagnini
		OPS_StartDate               			: BOOL;  //True se l^operatore ha dato lo START per iniziare il pezzo seguente OPS
		i_sys_activedevice              : INT;  //Dispositivo selezionato per movimenti in JOG
		// da 0..31 asse
		// da 32 a 64 dispositivo IO
		// -1 = INDEFINITO

		//Velocità massime sicure per COMFORT3 nell^ordine di VGP per lettura da parte alta                                
		i_sys_OPMODE                    : INT;	// MODO OPERATIVO
		// 0 = JOG
		// 1 = REFERENCE
		// 2 = MDA
		// 3 = AUTO
		
	END_VAR

    VAR_EXTERNAL CONSTANT
		MAX_AXIS_LINEARISES                	:INT;
		FIRST_DEVICE                    	:INT;
		PROGRAM_JOG                      	:SINT;
		ADDRESS_POWER_SUPPLY            	:INT;
		ACCESS_SAT           	            :SINT;
		TRC_EXECUTE_MOTOR_SETTING           :INT; // Traces from 200 to 299 are to be used for device waiting, BlmMotion displays 
            									// the device number by adding it to trace 39 (TRC_DISPOSITIVI_BLMMOTION)
            									// the next available trace should be 300;
		TRC_WAIT_SYSTEM_STARTUP             :INT;
		MAXDIGITALINPUT                     :INT;
		MAXDIGITALOUTPUT                    :INT;
		INP_VOFA_1							:INT;
		INP_VOFA_2							:INT;
		K_CHECKSUM_PLC_SAFETY      			:UDINT;

		//Linear machines
		OUT_BOOSTER_JOB                     :INT;
		TRC_CHANGE_CONFIGURATION_IO			:INT;
		ACTIVE_DEACTIVATABLE_AXIS           :SINT;
		INACTIVE_DEACTIVATABLE_AXIS         :SINT;
		ERR_CHECKSUM_SAFETY             	:DINT;
		ELECTRIC_SLED_TYPE              	:SINT;
		ERR_INCONGRUENZA_NUMENCODER         :DINT;
		SETTING_OBJECT_OPERATION			:UDINT;
		U4 									:INT;  //Mobile slide
		driveObjectIdConstant               : UINT;
        hardwareIdConstant                  :WORD;
		TRC_ASSE_HOLD                       :INT;
		BRAKE_ENABLED                    	:SINT;
		BRAKE_DISABLED                  	:SINT;
		ERR_CONFIG_MECHANICS_SAFETY         :DINT;
		ERR_SAFETY                          :DINT;
		JOGMODE                             :INT;
		UNDEF_DEVICE                       	:INT;
		ERR_NUMSTATES                       :DINT;
		ERR_GRAVE_DOWNLOAD                  :DINT; //Errore grave nel download del programma
		LOCK_2D                         	:INT;  //40
		//Superato limite trasferimenti per errore_a_tempo
		
		X1 									:INT;    //Cart X
		X2     								:INT;  
		Z1 									:INT;  //Rotation Z
		B1      							:INT ;  //Rotation 4RIPC
		B2      							:INT ;  //Rotation 4RIPC
		A1      							:INT;  //Rotation 4RTL/"Rotazione 4RC
		A2      							:INT;  //Rotation 4RC
		U2 									:INT ;  //Core
		Y1 :    INT;  //Y-axis bending arm
		                (*{
                #ifdef MATRICE_ELETTRICA
                }*)
		Y2 : INT;	//Matrice
                (*{
                #endif	//Fine MATRICE_ELETTRICA
                }*)
		TRC_INCONGRUENCE_MANIPULATOR2D	    :INT;
          //ABSOLUTELY DO NOT MODIFY THE RETAIN IN THIS MODULE
		OldPosAxe                       :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF LREAL;

	END_VAR 

    VAR
		Homing									: Homing;
        PosAx									: REF_TO TO_PositioningAxis;
		MasterAxis								: REF_TO TO_PositioningAxis;
		FollAsse								: REF_TO TO_SynchronousAxis;
		iret									:INT;
		dret									:DINT;
		appo									:LREAL;
		appoReal							    :REAL;
		myPosAxis							    :REF_TO TO_PositioningAxis;
		Address	 							    :DINT;
		LocAsse	 							    :REF_TO TO_PositioningAxis;//:PosAxis;
		//FollAsse					 		    :FollowingObjectType;
		//Ritorno					 			    :Ret_write_par;
		acc,vel								    :REAL;
		tmax				 					:REAL;
		//ValoreQ								    :EnumYesNo;
		//ValoreY								    :EnumYesNo;
		NumeroAssi					 	        :INT; 
		Appo_prima_accensione	                :BOOL;
		Esito									:BOOL;
		RetDint							 	    :DINT;
		RiavvioDaFare					        :BOOL;
		GiriMotore						        :LREAL;
		StatoFreno						        :LREAL;
		NumeroAsse						        :INT;
		Stato									:INT;
		ErroreAsse						        :INT;
		Checksum_Plc_Safety		                :DWORD;
		LocalRetVal						        :BOOL;
		testword							    :DWORD;
		
		//CURRENT_STATE						    :INT:=GENERAL_STATE#STATE_INIT;
		CURRENT_AXIS						    :INT;
		//myCommandID							    :CommandIDType;
		AttesaSinamicsBool	                    :BOOL;
		NumSetParametri					        :INT;
		
		ScritturaDatiSinamics	                :_WRITE_PARAMETER_SINGL;
		AttesaAvvioSistema		                :FB_WaitingStartUpSystem;//Needed Luigi Check
		CopyRamRom						        :FB_CopyRamRom;
		CopyRomSinamic				       		 :FB_CopyRomSinamic;// +
		LetturaDatoAzionamento                  :_READ_PARAMETER_SINGLE;//FB_ReadActivationData; wait-add later (while....)
		LetturaSinamicsREAL		                :_READ_PARAMETER_SINGLE;
		ReadDatiSafety				            :FB_ReadSafetyData;
		CambiaSetParametri		                :FB_ChangeSetParameters;//Needed Luigi check
		EnableAssi						    	:FB_EnableAssi;
		DisableAssi						     	: FB_DisableAxes;
		InversioneSensoAsse		                :FB_InversioneSensoAsse;
		//LetturaCoppiaMaxMotori                  :FB_LetturaCoppiaMaxMotori;
		DisableAsseInCoppia		                :FB_Disable_Axis_In_Torque;
		ManagementBrake					        :FB_ManagementBrake;
		RestartActivationAsse	                :FB_RestartActivationAxis;
		Modulo_Asse						        :FB_Form_Axis;
		ResettingXxOrbita						:ResettingXxOrbita;
		Module_2Axes							:Module_2Axes;
		Modulo_0_360 :							Modulo_0_360;
/********************
 * PLC_OPEN_COMMAND *
 ********************/
		_resetAxisError							: MC_Reset;
		parameterType							: INT; //add new, needed to be test
		
    END_VAR

	VAR_TEMP
		i		   								:INT;
    END_VAR

     //GetSystemDateTime(value => System_Timer); //EX RCT https://console.simatic-ax.siemens.io/docs/libraries/simatic-1500-clocks/functions/getsystemdatetime
	 //WHILE CURRENT_STATE<>GENERAL_STATE#STATE_END DO
		
		//CURRENT_STATE:= TEST_S; 
		Dummy_LREAL_1:= appo;
		Dummy_LREAL_2 := i_sys_MotorState[CURRENT_AXIS];
		Dummy_LREAL_3 := i_sys_statusencoder[CURRENT_AXIS];
		

		CASE CURRENT_STATE OF
			GENERAL_STATE#STATE_INIT:
				Appo_prima_accensione:=i_sys_first_on;
				i_sys_RUNOK:=0;
				i_sys_error_grave:=0;
				i_sys_ErrorLubr:=FALSE;
				(*{#ifdef PROFINET}*)
					IF SystemStarted=TRUE THEN
						O_EmergencyFromSimotion:=FALSE;
					ELSE
						O_EmergencyFromSimotion:=TRUE;
					END_IF;
				(*{#endif}	//Fine PROFINET
				{#ifdef FESTO_ECPE}*)
					IF cfg.AC00006842_02=TRUE AND I_Barrier=TRUE THEN
						IF getbit(IO_Link_Input, USINT#0)=FALSE THEN
							//Rientro completo stelo reset comandi di alzo ed intermedio
							IO_Link_Output:=setbit(IO_Link_Output, USINT#1, FALSE);//Emulazione
							IO_Link_Output:=setbit(IO_Link_Output, USINT#4, FALSE);//Emulazione
							//Set comando arretramento totale stelo
							IO_Link_Output:=setbit(IO_Link_Output, USINT#0, TRUE);
						END_IF;
					END_IF;	
				(*{#endif}	//Fine FESTO_ECPE*)
					IF i_sys_Motors_Verified=FALSE THEN
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_WAIT_AXES_VERIFIED;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHECK_RUN_RESET;
					END_IF;

			_PROGRAM_ENABLEAX#STATE_ENABLEAX_WAIT_AXES_VERIFIED:
					//Verifica che gli axes disattivabili (2 encoder) abbiano la corrispondente 
					//variabile ritentiva (ma_asse_installato) valorizza a 1 o 2. Se è a 0 occorre
					//effettuare un comando di riconoiscimento dei motori presenti in macchina
					IF TestDeactivationAxesState(VerificaPm105:=FALSE, ErroreAsse:=NumeroAsse)=FALSE AND i_sys_CmdVGPCSettings=FALSE THEN
						IF i_sys_Accesslevel <= ACCESS_SAT OR CheckIOAddresses()=FALSE THEN
							//Messagi per BLM che può usare il 242
							set_trace(Code:=TRC_EXECUTE_MOTOR_SETTING, Axis:=NumeroAsse);
						END_IF;
						i_sys_stopcond:=1;
						reset_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);				
					ELSE
						reset_trace(Code:=TRC_EXECUTE_MOTOR_SETTING);
						i_sys_Motors_Verified:=TRUE;		
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHECK_RUN_RESET;
					END_IF;
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHECK_RUN_RESET:
					IF i_sys_first_on=TRUE THEN
						//Prima accensione si aspetta l^avvio anche degli azionamenti
						PerformSafetyReset:=FALSE;
						set_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);
						AttesaSinamicsBool:=TRUE;
					ELSE
						AttesaSinamicsBool:=FALSE;
					END_IF;
					IF AttesaAvvioSistema.OutputState<>GENERAL_STATE#STATE_END THEN
						AttesaAvvioSistema(AttesaSimotion:=TRUE, AttesaSinamics:=AttesaSinamicsBool);
					ELSE
						Esito:=AttesaAvvioSistema.ReturnValue;
						AttesaAvvioSistema();
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_SYSTEM_STARTED;
					END_IF;
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_SYSTEM_STARTED:
					SystemStarted:=TRUE;
					O_EmergencyFromSimotion:=FALSE;
					(*{#ifdef DH4010}*)
						//Il sistema di sicurezza Vofa si considera presente quando  i 2 ingressi hanno un collegamento virtuale/fisico
					O_Vofa_1_2_Enable:=IAddress[INP_VOFA_1].Bit>-1 AND IAddress[INP_VOFA_1].Port>-1 AND
										IAddress[INP_VOFA_2].Bit>-1 AND IAddress[INP_VOFA_2].Port>-1;
					(*{#endif}	//Fine DH4010			
					{#ifdef PROFINET}*)
					O_PulserTwoHands:=PulserTwoHands;
					IF i_sys_first_on=TRUE THEN	
						(*{#ifndef PLC_S7_1500}*)
							//Gestione disabilitazione dei due PnPnCoupler con hardware STEP7
						IF (PresencePnCoupler=TRUE AND PresencePnCoupler2=FALSE) OR
							(PresencePnCoupler=FALSE AND PresencePnCoupler2=TRUE) THEN
							//Solo uno dei due PnPnCoupler è attivo
							i_sys_DpCoupler:=FALSE;
							i_sys_PnCoupler:=TRUE;
								(*{#ifdef SELEZIONE_DOPPIO_PNPN}*)
								IF PresencePnCoupler=TRUE THEN
									//Si disattiva il secondo PnPnCoupler e si lascia attiva il primo
									O_PnCoupler_2_Disable:=TRUE;
									//dret:=_waitTime(T#100ms);
									O_PnCoupler_2_Disable:=FALSE;
								ELSE
									//Si disattiva il primo PnPnCoupler e si lascia attivo il secondo
									O_PnCoupler_Disable:=TRUE;
									//dret:=_waitTime(T#100ms);
									O_PnCoupler_Disable:=FALSE;
								END_IF;
									(*{#else}*)	//Fine SELEZIONE_DOPPIO_PNPN
									//Con solo un PnPnCoupler presente nella configurazione macchina è sufficiente che sia a TRUE
									//almeno uno dei due prametri Q0218 o Q222 per abilitare l^unico PnPnCoupler presente, quindi 
									//si disattiva il secondo (anche se non è realmente presente)
									O_PnCoupler_2_Disable:=TRUE;
									//dret:=_waitTime(T#100ms);
									O_PnCoupler_2_Disable:=FALSE;
									(*{#endif}*)	//Fine NO SELEZIONE_DOPPIO_PNPN
						ELSE
							//Nessun PnPnCoupler abilitato, oppure entrambi sono erroneamente abilitati, si disattivano !!!
							i_sys_PnCoupler:=FALSE;
							O_PnCoupler_Disable:=TRUE;
							O_PnCoupler_2_Disable:=TRUE;
							//dret:=_waitTime(T#100ms);
						END_IF;
						O_PnCoupler_Disable:=FALSE;
						O_PnCoupler_2_Disable:=FALSE;
						(*{#endif}	//Fine NO PLC_S7_1500
						{#ifdef PLC_S7_1500}*)
						IF (PresencePnCoupler) THEN
							i_sys_PNCoupler:=TRUE;
							i_sys_DPCoupler:=FALSE;
						END_IF;
						(*{#endif}		
						{#ifndef EMULAZIONE}*)
						//Verifica congruenza fra il checksum letto del PLC di sicurezza e quello 
						//previsto in simotion
						testword:=Checksum_Plc_Safety_Hight;
						Checksum_Plc_Safety.%W0:= Checksum_Plc_Safety_Low;
						Checksum_Plc_Safety.%W1:= Checksum_Plc_Safety_Hight;
						Checksum_Plc_Safety:=SHL(testword, UDINT#16);
						//Checksum_Plc_Safety:=Checksum_Plc_Safety OR Checksum_Plc_Safety_Low; //???
						IF Checksum_Plc_Safety=DWORD#0 THEN
							//Macchine che non prevedono il trasferimento del valore dal plc di sicurezza
							ChecksumPlcSafety:=K_CHECKSUM_PLC_SAFETY;
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CALCULATE_FACTOR_PRECOMMAND;
						ELSE
							IF TO_UDINT(Checksum_Plc_Safety)<>K_CHECKSUM_PLC_SAFETY THEN
							   CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENDLESS_LOOP_CHECKSUM;
							ELSE
								ChecksumPlcSafety:=TO_UDINT(Checksum_Plc_Safety);
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CALCULATE_FACTOR_PRECOMMAND;
							END_IF;
						END_IF;					
						(*{#else}*)
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CALCULATE_FACTOR_PRECOMMAND;
						(*{#endif};*)
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CALCULATE_FACTOR_PRECOMMAND;
					END_IF;
					(*{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CALCULATE_FACTOR_PRECOMMAND;
					(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENDLESS_LOOP_CHECKSUM:
					Init_trace();
					Init_error();
					iret:=Push_error(Code:=ERR_CHECKSUM_SAFETY);

			_PROGRAM_ENABLEAX#STATE_ENABLEAX_CALCULATE_FACTOR_PRECOMMAND:
					(*{#ifdef ORBITALE}*)
					CalculateFactorPrecommand();
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENABLE_BUSY;
					CambiaSetParametri();
					(*{#else}*)
					IF pm[X1].ma_num_encs>1 THEN
						IF OAddress[OUT_BOOSTER_JOB].stato=TRUE THEN
							NumSetParametri:=3;
						ELSE	
							NumSetParametri:=2;
						END_IF;
						IF CambiaSetParametri.OutputState<>GENERAL_STATE#STATE_END THEN

							CambiaSetParametri(Axis:=Axes[X1].PosAx,
												NumeroAsse:=X1,
												NumeroSetParametri:=NumSetParametri,
												ProssimoComando:=EnumNextCommandEnable#WHEN_COMMAND_DONE);						
						ELSE
							CambiaSetParametri();//retur value Curen_State = State_End
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENABLE_BUSY;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENABLE_BUSY;
					END_IF;
					(*{#endif};*)
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENABLE_BUSY:
					i_sys_EnableBusy:=TRUE;
					IF i_sys_Busy=TRUE THEN
						//Invio dati da BlmMotion con richiesta copia da Ram a Rom
						IF CopyRamRom.OutputState<>GENERAL_STATE#STATE_END THEN
							CopyRamRom();
						ELSE
							CopyRamRom();
							//SF [21/04/22]: gestione che in seguito ad un trasferimento da BLMMotion verifica se attivare o meno gli IO opzionali montati su PLC Safety
							(*{#ifdef PLC_S7_1500}*)
								IF (Configuration_IO<>GetPlcIOConfig()) THEN
									Configuration_IO:=SINT#1;
									Configuration_IO:=GetPlcIOConfig();
									Configurazione_IO_to_PLC:=Configuration_IO;
									//dret:=_waitTime(T#1000ms);
									Strobe_Config_IO:=FALSE;
									set_trace(Code:=TRC_CHANGE_CONFIGURATION_IO);
								END_IF;
							(*{#endif}*) //FINE PLC_S7_1500
							i_sys_Busy:=FALSE;
							InitializeIOExecuted:=FALSE;
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_SMART_LINE;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_SMART_LINE;
					END_IF;
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_SMART_LINE:
					ResetSmartLine();
					// INIZIALIZZAZIONE PARAMETRI ASSI COPIANDO DA RETAIN
					//Trovo l^asse meno performante
					FOR i:=0 TO _MAXIS#MAXAXIS-1 DO
						IF pm[i].ma_Asse_Abilitato=TRUE AND pm[i].ma_Asse_Installato<>INACTIVE_DEACTIVATABLE_AXIS THEN
							IF NoTestL1(Axis:=i)=TRUE AND NoTestZ2(Axis:=i)=TRUE THEN
								//Gli axes L1 e Z2 non deveno essere considerati nel calcolo del tempo piu^ lungo
								//in quanto non partecipano alle camme e poi per l^AL5 le loro accelerazioni sono
								//state ridotte per evitare oscillazioni meccaniche e scivolamenti del tubo sulle rotelle
								myPosAxis := AsPositioningAxisRef(Axes[i].PosAx);//myPosAxis:=Assi[i].PosAx;
								acc:=pm[i].MA_max_ax_accel;
								vel:=pm[i].max_ax_velo;
								IF acc>0 THEN 
									IF vel/acc>tmax THEN
										tmax:=vel/acc;
									END_IF;
								END_IF;
							END_IF;
						END_IF;	
					END_FOR;
					IF tmax>0 THEN
						//Il tempo maxesmo di accelerazione/decelerazione non può essere superiore a 1.5 secondo, altrimenti
						//gli axes non si fermano più al cambio di modo (l^override calerebbe troppo lentamente)
						tmax:=MIN(tmax,REAL#1.5);
						i_sys_ovrstep_default:=1*100/(1000*tmax);
						//Accelerazione per stop ciclo non deve essere inferiore a 0.5 secondi, 
						i_sys_acc_master:=MAX(1/tmax, 2);
					ELSE
						i_sys_ovrstep_default:=1;
						//Accelerazione per stop ciclo non deve essere inferiore a 0.5 secondi
						i_sys_acc_master:=2;
					END_IF;
					i_sys_ovrstep:=i_sys_ovrstep_default;
					CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_REVERSAL_DIRECTION;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_REVERSAL_DIRECTION:
					IF i_sys_first_on=TRUE THEN
						CURRENT_AXIS:=CURRENT_AXIS+1;
						IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
							(*{#ifndef DXSX_PROCESSO} 
								{#ifdef THREE_RUNNER_TWO_RUNNER}*)
									CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_REVERSE_SENSE_REVERSES_AXIS;
								(*{#else}*)
									IF i_sys_sense_curve <>0 THEN
										CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_REVERSE_SENSE_REVERSES_AXIS;
									END_IF;
								(*{#endif};
							{#else}	*)					
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_REVERSE_SENSE_REVERSES_AXIS;
							(*{#endif};*)
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_WAITING_STARTUP_SYSTEM;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_VECTORIAL_OBJECTS_ACCELERATION;
					END_IF;
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_REVERSE_SENSE_REVERSES_AXIS:
			;
					(*IF InversioneSensoAsse.OutputState<>GENERAL_STATE#STATE_END THEN
						InversioneSensoAsse(Asse:=CURRENT_AXIS, Senso:=i_sys_sense_curve);
					ELSE
						InversioneSensoAsse();
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_REVERSAL_DIRECTION;
					END_IF;	*)	
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_WAITING_STARTUP_SYSTEM:
			;
					IF AttesaAvvioSistema.OutputState<>GENERAL_STATE#STATE_END THEN
						AttesaAvvioSistema(AttesaSimotion:=TRUE, AttesaSinamics:=FALSE);
					ELSE
						AttesaAvvioSistema();
						IF i_sys_control_incongruity_encoder AND VerificationSimulationAxes()=FALSE THEN
							myPosAxis := AsPositioningAxisRef(Axes[X1].PosAx);
							Appo:=0;					
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_WAIT_SINAMIC_READY;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_MOBILE_SLIDE_TYPE;
						END_IF;
					END_IF;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_WAIT_SINAMIC_READY:
					IF Appo=0 THEN
						IF NOT LetturaSinamicsREAL.Done THEN
							LetturaSinamicsREAL	(Start := Start_read,
												driveObjectId      := driveObjectIdConstant,
												hardwareId          := hardwareIdConstant,
												parameterNumber     := uint#460,
												index              := uint#0);
							parameterType      := 0;//add new, needed to be test
						ELSIF LetturaSinamicsREAL.Done THEN
							appo:=LetturaSinamicsREAL.Value_REAL;
							LetturaSinamicsREAL.Start :=false;
						ELSIF LetturaSinamicsREAL.error THEN
							LetturaSinamicsREAL.Start :=false;
						END_IF;
					ELSE
						CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_CHANGE_DATASET;
					END_IF;

			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_CHANGE_DATASET:
					CURRENT_AXIS:=CURRENT_AXIS+1;
					IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
						myPosAxis := AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
						//myPosAxis:=Axes[CURRENT_AXIS].PosAx;
						IF myPosAxis^.VirtualAxis.Mode <> UDINT#0 AND pm[CURRENT_AXIS].ma_Asse_Abilitato=TRUE AND
							Axes[CURRENT_AXIS].IsSiemensAxis THEN	
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENGINE_STATUS; 
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_MOBILE_SLIDE_TYPE;
					END_IF;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENGINE_STATUS:
					//Lettura stato motore dal parametro 105
					myPosAxis := AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
					IF NOT LetturaSinamicsREAL.Done THEN
						LetturaSinamicsREAL	(Start := Start_read,
											driveObjectId      	:= driveObjectIdConstant, //,
											hardwareId          := hardwareIdConstant,
											parameterNumber     := TO_UINT (SETTING_OBJECT_OPERATION),
											index              	:= uint#0);
											//parameterType      := 0;//add new, needed to be test
					ELSIF LetturaSinamicsREAL.Done THEN
						i_sys_MotorState[CURRENT_AXIS]:=LetturaSinamicsREAL.Value_REAL;
						LetturaSinamicsREAL.Start :=false;
						IF pm[CURRENT_AXIS].ma_asse_installato=INACTIVE_DEACTIVATABLE_AXIS AND
							myPosAxis = AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx) AND 
						   myPosAxis^.Sensor[4].Existent =true THEN
							IF myPosAxis^.Sensor[2]._Interface.AddressIn.DB_NUMBER > UINT#0 THEN
								_resetAxisError.Execute := TRUE;
								//myCommandID:=_getCommandID();
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_SIMULATION_PARAMETERS_SET;
							ELSE
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_NON_SIMULATION_AXIS;
							END_IF;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_NON_SIMULATION_AXIS;
						END_IF;
					ELSIF LetturaSinamicsREAL.error THEN
						LetturaSinamicsREAL.Start :=false;
					END_IF;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_SIMULATION_PARAMETERS_SET:
					IF CambiaSetParametri.OutputState<>GENERAL_STATE#STATE_END THEN
						
						CambiaSetParametri (Axis:=Axes[i].PosAx,/// axis:=MyPosAxis ???
																NumeroAsse:=i,
																NumeroSetParametri:=2,
																ProssimoComando:=EnumNextCommandEnable#WHEN_COMMAND_DONE);					
					ELSE
						myPosAxis^.Simulation.Mode := UDINT#1;
						_resetAxisError.Restart:=TRUE;

						_resetAxisError.Execute:=TRUE;
						
						IF  (_resetAxisError.Done) THEN
							CambiaSetParametri();
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER;
						END_IF;
					END_IF;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_NON_SIMULATION_AXIS:
					(*{#ifndef ORBITALE}*)
						myPosAxis:=AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
						IF myPosAxis^.Sensor[4].Existent =true THEN
							IF pm[CURRENT_AXIS].ma_num_encs>1 THEN
								//Legge se fisicamente è veramente montato il secondo encoder
								//Effettua una lettura di un parametro SINAMICS per attendere che gli
								//azionamenti sono accesi
								IF NOT LetturaSinamicsREAL.Done THEN
									LetturaSinamicsREAL	(Start := Start_read,
														driveObjectId      	:= driveObjectIdConstant, //,
														hardwareId          := hardwareIdConstant,
														parameterNumber     := uint#146,
														index              	:= uint#0);
														//parameterType      := 0;//add new, needed to be test
								ELSIF LetturaSinamicsREAL.Done THEN
									i_sys_statusencoder[CURRENT_AXIS]:=LetturaSinamicsREAL.Value_REAL;
									LetturaSinamicsREAL.Start :=false;
									IF pm[CURRENT_AXIS].ma_num_encs=2 THEN
										IF myPosAxis = AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx) AND (myPosAxis^.ActualPosition<pm[CURRENT_AXIS].Ma_Pos_Limit_Minus_Attivo) AND
											 (myPosAxis^.ActualPosition>pm[CURRENT_AXIS].Ma_Pos_Limit_Plus_Attivo) THEN	;
											//Se l^asse è fuori dai suoi limiti software si imposta la quota attiva
											//a metà della sua corsa, questo per evitare saltuari errori assi durante
											//il successivo posizionamento
											(*dret:=_redefineposition(axis:=Assi[CURRENT_AXIS].PosAx,
																							redefinemode:=ABSOLUTE,
																							position:=((pm[CURRENT_AXIS].Ma_Pos_Limit_Plus_Attivo-pm[CURRENT_AXIS].Ma_Pos_Limit_Minus_Attivo)/2)+pm[CURRENT_AXIS].Ma_Pos_Limit_Minus_Attivo);*)										 
										END_IF;
										pm[CURRENT_AXIS].ma_stato_index:=SINT#0;
										IF i_sys_statusencoder[CURRENT_AXIS]=1 THEN
											(*{#ifdef CAMBIO_MECCANICO}*)
												//Presente secondo sistema di misura pertanto imposto il secondo dataset
												//sgancia il booster
											//Set_disp(Ndisp:=BOOSTER, Stato:=RIPOSO, TimerRisparmio:=TRUE);
											(*{#endif}*)	//Fine CAMBIO_MECCANICO
											CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_TRUE_ENCODER_PARAMETERS_SET;
										ELSE
											//incongruenza fra dato macchina e numero encoder presenti
											i_sys_error_grave:=1;
											CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER;
										END_IF;
									ELSE
										IF pm[i].ma_num_encs<>1 THEN								
											i_sys_error_grave:=1;
										END_IF;	
										CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER;
									END_IF;
								END_IF;
							ELSE
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER;
							END_IF;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER;
						END_IF;
					(*{#else}*)
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER;
					(*{#endif};*)
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_TRUE_ENCODER_PARAMETERS_SET:

					IF CambiaSetParametri.OutputState<>GENERAL_STATE#STATE_END THEN
						CambiaSetParametri(
											Axis:=Axes[CURRENT_AXIS].PosAx,
											NumeroAsse:=CURRENT_AXIS,
											NumeroSetParametri:=2,
											ProssimoComando:=EnumNextCommandEnable#WHEN_COMMAND_DONE);
												
					ELSE
						CambiaSetParametri();
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER;
					END_IF;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_ENCODER_NUMBER:
					PosAx := AsPositioningAxisRef(Axes[CURRENT_AXIS].posax);
					IF PosAx^.VirtualAxis.Mode = UDINT#1 AND pm[CURRENT_AXIS].ma_Asse_Abilitato=TRUE THEN
						IF pm[CURRENT_AXIS].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS AND Axes[CURRENT_AXIS].IsSiemensAxis THEN	
							//Se asse non è disabilitato legge il numero di serie dell^encoder
							//e lo salva nella struttura assi
							myPosAxis:=AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
							(*Address:=UDINT_TO_DINT(myPosAxis.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)		
							IF NOT (i_sys_ReadingSinamicsInUse=TRUE) THEN
								Appo:=0;
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_SINAMICS_ENCODER_NUMBER;
							END_IF;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_CHANGE_DATASET;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_CHANGE_DATASET;
					END_IF;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_SINAMICS_ENCODER_NUMBER:
					IF Appo=0 THEN
						IF NOT LetturaSinamicsREAL.Done THEN
							LetturaSinamicsREAL(Start := Start_read,
												driveObjectId := driveObjectIdConstant,
												parameterNumber:=uint#463,
												index:=uint#0,
												hardwareId          := hardwareIdConstant);					
						ELSIF LetturaSinamicsREAL.Done THEN
							appo:=LetturaSinamicsREAL.Value_REAL;
							LetturaSinamicsREAL.Start :=false;
						ELSIF LetturaSinamicsREAL.error THEN
							LetturaSinamicsREAL.Start :=false;
						END_IF;	
					ELSE
						Axes[CURRENT_AXIS].numserieencoder:=Appo;
						//dret:=_waitTime(T#50ms);
						PosAx:= AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
						IF pm[CURRENT_AXIS].ma_verifica_num_serie_encoder=TRUE THEN;
							Axes[CURRENT_AXIS].Homed:=Axes[CURRENT_AXIS].numserieencoder=pm[CURRENT_AXIS].ma_num_serie_encoder AND PosAx^.StatusWord.%x5;
						ELSE ;
							Axes[CURRENT_AXIS].Homed:=PosAx^.StatusWord.%X5;
						END_IF;
						//Se l^encoder è assoluto non richiede presa index dopo STOP/RUN
						IF GetEncoderAbsolute(i) THEN
							Axes[CURRENT_AXIS].Homing_al_restart:=FALSE;
							Axes[CURRENT_AXIS].tipo_encoder:=SINT#1; 
						ELSE
							//Incrementale chiede sempre HOMING
							Axes[CURRENT_AXIS].tipo_encoder:=SINT#2;
							Axes[CURRENT_AXIS].Homed:=FALSE;
						END_IF;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_CHANGE_DATASET;
					END_IF;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_MOBILE_SLIDE_TYPE:
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHECK_SERIOUS_ERROR;
					(*{#ifdef SMART}*)
						//La SMART ha la slitta mobile a molla, ma viene considerata come elettrica in quanto
						//si mette l^asse in simulazione
						cfg.TipoSlittaMobile:=ELECTRIC_SLED_TYPE;
					(*{#endif}	//Fine SMART
					{#ifdef SMART_A}*)
						cfg.TipoSlittaMobile:=ELECTRIC_SLED_TYPE;
					(*{#endif}	//Fine SMART_A	
					{#ifdef ELECTM}*)
						IF NOT (i_sys_InUseWritingSynamics=TRUE) THEN
							LocAsse:=AsPositioningAxisRef(Axes[U4].PosAx);
							(*Address:=UDINT_TO_DINT(LocAsse.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);	*)
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_MOBILE_SLIDE_TYPE;
						END_IF;
					(*{#endif};
					{#ifdef ELECTL_ALL}*)
						//La ELECTL ha solo la slitta mobile elettrica
						cfg.TipoSlittaMobile:=ELECTRIC_SLED_TYPE;
					(*{#endif}	//Fine ELECTL_ALL
					{#ifdef ELECTXL}*)
						// La ELECTXL ha solo la slitta mobile elettrica
						cfg.TipoSlittaMobile:=ELECTRIC_SLED_TYPE; 
					(*{#endif}	//Fine ELECTXL*)
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHECK_SERIOUS_ERROR:
					IF i_sys_error_grave=0 THEN
						i_sys_control_incongruity_encoder:=FALSE; 
						//LetturaCoppiaMaxMotori();
						i_sys_stopcond:=0;
						CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_AXIS_ENCODER_LOOP_OK;
					ELSE
						iret:=Push_error(Code:=ERR_INCONGRUENZA_NUMENCODER);	
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_VECTORIAL_OBJECTS_ACCELERATION;
					END_IF;
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_AXIS_ENCODER_LOOP_OK:
					CURRENT_AXIS:=CURRENT_AXIS+1;
					IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_WAIT_REAL_VELOCITY_ZERO ;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_VECTORIAL_OBJECTS_ACCELERATION;
					END_IF;
					
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_WAIT_REAL_VELOCITY_ZERO:
					IF TestVelRealZero(NomeAsse:=Axes[CURRENT_AXIS].PosAx, NumeroAsse:=CURRENT_AXIS, ErroreInseguimento:=TRUE)=FALSE THEN
						set_trace(Code:=TRC_ASSE_HOLD , axis:=CURRENT_AXIS);
					ELSE
						reset_trace(Code:=TRC_ASSE_HOLD );
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_AXIS_ENCODER_LOOP_OK;
					END_IF;
			(*_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_VECTORIAL_OBJECTS_ACCELERATION:
					//Allinea accelerazione e velocità degli oggetti vettoriali
					{#ifdef INTERPOLA_Y1_X1}
						Y1_X1.userdefault.pathdynamics.velocity:=MAX(pm[Y1].MAX_ax_velo,pm[X1].MAX_ax_velo);
						Y1_X1.userdefault.pathdynamics.positiveaccel:=MAX(pm[Y1].MA_max_ax_accel,pm[X1].MA_max_ax_accel);
						Y1_X1.userdefault.pathdynamics.negativeaccel:=MAX(pm[Y1].MA_max_ax_accel,pm[X1].MA_max_ax_accel);
					{#endif}	//Fine INTERPOLA_Y1_X1
					{#ifdef INTERPOLA_X3_X6}
						X3_X6.userdefault.pathdynamics.velocity:=MAX(pm[X3].MAX_ax_velo,pm[X6].MAX_ax_velo);
						X3_X6.userdefault.pathdynamics.positiveaccel:=MAX(pm[X3].MA_max_ax_accel,pm[X6].MA_max_ax_accel);
						X3_X6.userdefault.pathdynamics.negativeaccel:=MAX(pm[X3].MA_max_ax_accel,pm[X6].MA_max_ax_accel);
					{#endif}	//Fine INTERPOLA_X3_X6
					{#ifdef INTERPOLA_X2_X6}
						X2_X6.userdefault.pathdynamics.velocity:=MAX(pm[X2].MAX_ax_velo,pm[X6].MAX_ax_velo);
						X2_X6.userdefault.pathdynamics.positiveaccel:=MAX(pm[X2].MA_max_ax_accel,pm[X6].MA_max_ax_accel);
						X2_X6.userdefault.pathdynamics.negativeaccel:=MAX(pm[X2].MA_max_ax_accel,pm[X6].MA_max_ax_accel);
					{#endif}	//Fine INTERPOLA_X2_X6
					{#ifdef INTERPOLA_X1_Y1}
						X1_Y1.userdefault.pathdynamics.velocity:=MAX(pm[X1].MAX_ax_velo,pm[Y1].MAX_ax_velo);
						X1_Y1.userdefault.pathdynamics.positiveaccel:=MAX(pm[X1].MA_max_ax_accel,pm[Y1].MA_max_ax_accel);
						X1_Y1.userdefault.pathdynamics.negativeaccel:=MAX(pm[X1].MA_max_ax_accel,pm[Y1].MA_max_ax_accel);
					{#endif}	//Fine INTERPOLA_X1_Y1
					{#ifdef INTERPOLA_X2_Y2}
						X2_Y2.userdefault.pathdynamics.velocity:=MAX(pm[X2].MAX_ax_velo,pm[Y2].MAX_ax_velo);
						X2_Y2.userdefault.pathdynamics.positiveaccel:=MAX(pm[X2].MA_max_ax_accel,pm[Y2].MA_max_ax_accel);
						X2_Y2.userdefault.pathdynamics.negativeaccel:=MAX(pm[X2].MA_max_ax_accel,pm[Y2].MA_max_ax_accel);
					{#endif}	//Fine INTERPOLA_X2_Y2
					{#ifdef SLITTA_MOBILE_DX}
						Y1_U4.userdefault.pathdynamics.velocity:=MAX(pm[Y1].MAX_ax_velo,pm[U4].MAX_ax_velo);
						Y1_U4.userdefault.pathdynamics.positiveaccel:=MAX(pm[Y1].MA_max_ax_accel,pm[U4].MA_max_ax_accel);
						Y1_U4.userdefault.pathdynamics.negativeaccel:=MAX(pm[Y1].MA_max_ax_accel,pm[U4].MA_max_ax_accel);
					{#endif}	//Fine SLITTA_MOBILE_DX
					{#ifdef SLITTA_MOBILE_SX}
						Y1_U5.userdefault.pathdynamics.velocity:=MAX(pm[Y1].MAX_ax_velo,pm[U5].MAX_ax_velo);
						Y1_U5.userdefault.pathdynamics.positiveaccel:=MAX(pm[Y1].MA_max_ax_accel,pm[U5].MA_max_ax_accel);
						Y1_U5.userdefault.pathdynamics.negativeaccel:=MAX(pm[Y1].MA_max_ax_accel,pm[U5].MA_max_ax_accel);
					{#endif}	//Fine SLITTA_MOBILE_SX
					{#ifndef ORBITALE}
						X1_U2.userdefault.pathdynamics.velocity:=MAX(pm[X1].MAX_ax_velo,pm[U2].MAX_ax_velo);
						X1_U2.userdefault.pathdynamics.positiveaccel:=MAX(pm[X1].MA_max_ax_accel,pm[U2].MA_max_ax_accel);
						X1_U2.userdefault.pathdynamics.negativeaccel:=MAX(pm[X1].MA_max_ax_accel,pm[U2].MA_max_ax_accel);
					{#endif}	//Fine NO ORBITALE
					{#ifndef ETURN}
						{#ifndef ORBITALE}
							{#ifndef ELECTMRL}
								W3_W4_Q1.userdefault.pathdynamics.velocity:=MAX(pm[W3].MAX_ax_velo,pm[W4].MAX_ax_velo,pm[Q1].MAX_ax_velo);
								W3_W4_Q1.userdefault.pathdynamics.positiveaccel:=MAX(pm[W3].MA_max_ax_accel,pm[W4].MA_max_ax_accel,pm[Q1].MA_max_ax_accel);
								W3_W4_Q1.userdefault.pathdynamics.negativeaccel:=MAX(pm[W3].MA_max_ax_accel,pm[W4].MA_max_ax_accel,pm[Q1].MA_max_ax_accel);
							{#else} //FINE NO ELECTMRL
								W3_W4_W6_Q1.userdefault.pathdynamics.velocity:=MAX(pm[W3].MAX_ax_velo,pm[W4].MAX_ax_velo,pm[Q1].MAX_ax_velo, pm[W6].max_ax_velo);
								W3_W4_W6_Q1.userdefault.pathdynamics.positiveaccel:=MAX(pm[W3].MA_max_ax_accel,pm[W4].MA_max_ax_accel,pm[Q1].MA_max_ax_accel,pm[W6].MA_max_ax_accel);
								W3_W4_W6_Q1.userdefault.pathdynamics.negativeaccel:=MAX(pm[W3].MA_max_ax_accel,pm[W4].MA_max_ax_accel,pm[Q1].MA_max_ax_accel,pm[w6].MA_max_ax_accel);
							{#endif} //FINE ELECTMRL
						{#endif}	//Fine NO ORBITALE
					{#endif}	//Fine NO ETURN
					{#ifdef MATRICE_ELETTRICA}
						IF TestAsseY2()=TRUE THEN
							Y1_U4_Y2.userdefault.pathdynamics.velocity:=MAX(pm[Y1].MAX_ax_velo,pm[U4].MAX_ax_velo,pm[Y2].MAX_ax_velo);
							Y1_U4_Y2.userdefault.pathdynamics.positiveaccel:=MAX(pm[Y1].MA_max_ax_accel,pm[U4].MA_max_ax_accel,pm[Y2].MA_max_ax_accel);
							Y1_U4_Y2.userdefault.pathdynamics.negativeaccel:=MAX(pm[Y1].MA_max_ax_accel,pm[U4].MA_max_ax_accel,pm[Y2].MA_max_ax_accel);
							Y1_Y2.userdefault.pathdynamics.velocity:=MAX(pm[Y1].MAX_ax_velo, pm[Y2].MAX_ax_velo);
							Y1_Y2.userdefault.pathdynamics.positiveaccel:=MAX(pm[Y1].MA_max_ax_accel, pm[Y2].MA_max_ax_accel);
							Y1_Y2.userdefault.pathdynamics.negativeaccel:=MAX(pm[Y1].MA_max_ax_accel, pm[Y2].MA_max_ax_accel);
						END_IF;
					{#endif} //Fine MATRICE_ELETTRICA
					{#ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO}
						Q1_V1_V4.userdefault.pathdynamics.velocity:=MAX(pm[Q1].MAX_ax_velo, pm[V1].MAX_ax_velo, pm[V4].MAX_ax_velo);
						Q1_V1_V4.userdefault.pathdynamics.positiveaccel:=MAX(pm[Q1].MA_max_ax_accel, pm[V1].MA_max_ax_accel, pm[V4].MA_max_ax_accel);
						Q1_V1_V4.userdefault.pathdynamics.negativeaccel:=MAX(pm[Q1].MA_max_ax_accel, pm[V1].MA_max_ax_accel, pm[V4].MA_max_ax_accel);
						
						Q1_V1.userdefault.pathdynamics.velocity:=MAX(pm[Q1].MAX_ax_velo, pm[V1].MAX_ax_velo);
						Q1_V1.userdefault.pathdynamics.positiveaccel:=MAX(pm[Q1].MA_max_ax_accel, pm[V1].MA_max_ax_accel);
						Q1_V1.userdefault.pathdynamics.negativeaccel:=MAX(pm[Q1].MA_max_ax_accel, pm[V1].MA_max_ax_accel);
					{#endif}	//Fine SOLLEVAMENTO_SLITTA_SPALLAMENTO	
					{#ifdef INTERPOLA_Q1_V1_Z1}
						Q1_V1_Z1.userdefault.pathdynamics.velocity:=MAX(pm[Q1].MAX_ax_velo, pm[V1].MAX_ax_velo, pm[Z1].MAX_ax_velo);
						Q1_V1_Z1.userdefault.pathdynamics.negativeaccel:=MAX(pm[Q1].MA_max_ax_accel, pm[V1].MA_max_ax_accel, pm[Z1].MA_max_ax_accel);
						Q1_V1_Z1.userdefault.pathdynamics.positiveaccel:=MAX(pm[Q1].MA_max_ax_accel, pm[V1].MA_max_ax_accel, pm[Z1].MA_max_ax_accel);
					{#endif}	//Fine INTERPOLA_Q1_V1_Z1	
					CURRENT_STATE:=STATE_ENABLEAX_ABILITA_ASSI;*)
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENABLE_AXES:
					//Abilita tutti gli assi se non ci sono errori
					NumeroAssi:=0;
					PM_Modified:=FALSE;			   
					FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
						IF Axes[i].Request_Restart_Oggetto_Simotion=TRUE OR 
							 Axes[i].Request_Restart_Oggetto_Sinamics=TRUE THEN
							 PM_Modified:=TRUE;
						END_IF;
					END_FOR; 				
					IF PM_Modified=TRUE OR i_sys_Param_RW=2 THEN
						i_sys_stopcond:=1;
						PM_Modified:=TRUE;
						CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLE_RESTART_LOOP;
					ELSE
						CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS;
					END_IF;

			_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLE_RESTART_LOOP:
				CURRENT_AXIS:=CURRENT_AXIS+1;
				IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
					IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE THEN
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLE_RESTART_LOOP_EXECUTION;
					ELSE
						Axes[CURRENT_AXIS].Request_Restart_Oggetto_Simotion:=FALSE;
					END_IF;
				ELSE	
					CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLE_RESTART_LOOP_EXECUTION:
				IF DisableAssi.OutputState<>GENERAL_STATE#STATE_END THEN
					DisableAssi(Asse:=CURRENT_AXIS);	
				ELSE
					IF Axes[CURRENT_AXIS].Request_Restart_Oggetto_Simotion=TRUE THEN
						IF RestartActivationAsse.OutputState<>GENERAL_STATE#STATE_END THEN
							RestartActivationAsse(NumAsse:=CURRENT_AXIS, Attesa:=TRUE);
						ELSE
							DisableAssi();
							RestartActivationAsse();
							Axes[CURRENT_AXIS].Request_Restart_Oggetto_Simotion:=FALSE;
							LocAsse:=AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
							pm[CURRENT_AXIS].max_ax_velo:=TO_REAL(LocAsse^.DynamicLimits.MaxVelocity);
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLE_RESTART_LOOP;
						END_IF;
					ELSE
						DisableAssi();
						LocAsse:=AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
						pm[CURRENT_AXIS].max_ax_velo:=TO_REAL(LocAsse^.DynamicLimits.MaxVelocity);
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLE_RESTART_LOOP_EXECUTION;
					END_IF;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE:
				CURRENT_AXIS:=CURRENT_AXIS+1;
				IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
					IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE THEN
						LocAsse:=AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
						
						IF LocAsse^.sensor[1].Existent =true THEN
							//Address:=UDINT_TO_DINT(LocAsse.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);
							IF Axes[CURRENT_AXIS].Request_Restart_Oggetto_Sinamics THEN
								IF pm[CURRENT_AXIS].az_p1441>=0 THEN
									CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE_1441;
								ELSE
									CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE_1460;
								END_IF;
							END_IF;
						ELSE
							Axes[CURRENT_AXIS].Request_Restart_Oggetto_Sinamics:=FALSE;
						END_IF;
					ELSE
						Axes[CURRENT_AXIS].Request_Restart_Oggetto_Sinamics:=FALSE;
						i_sys_Copy_RAM_ROM:=TRUE;
					END_IF;
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHECK_PARAM_RW_2;
				END_IF;

			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE_1441:
				IF NOT ScritturaDatiSinamics.Done THEN
					ScritturaDatiSinamics(Start := Start_read,
											driveObjectId :=driveObjectIdConstant,
											parameterNumber:=uint#1441,
											index:=uint#0,
											hardwareId := hardwareIdConstant,
											value:=pm[CURRENT_AXIS].az_p1441);				
				ELSIF ScritturaDatiSinamics.Done THEN
					ScritturaDatiSinamics.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE_1460;
				ELSIF ScritturaDatiSinamics.error THEN
					ScritturaDatiSinamics.Start :=false;
				END_IF;	
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE_1460:
				IF pm[CURRENT_AXIS].az_p1460>=0 THEN 
					IF NOT ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics(Start := Start_read,
											driveObjectId :=driveObjectIdConstant,
											hardwareId := hardwareIdConstant,
											parameterNumber:=uint#1460,
											index:=uint#0,
											value:=pm[CURRENT_AXIS].az_p1460);				
					ELSIF ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics.Start :=false;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE_1460;
					ELSIF ScritturaDatiSinamics.error THEN
						ScritturaDatiSinamics.Start :=false;
					END_IF;					
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1462;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1462:
				IF pm[CURRENT_AXIS].az_p1462>=0 THEN 
					IF NOT ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics(Start := Start_read,
											driveObjectId :=driveObjectIdConstant,
											hardwareId := hardwareIdConstant,
											parameterNumber:=uint#1462,
											index:=uint#0,
											value:=pm[CURRENT_AXIS].az_p1462);				
					ELSIF ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics.Start :=false;
						CURRENT_STATE:=_PROGRAM_ENABLEAX# STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1520;
					ELSIF ScritturaDatiSinamics.error THEN
						ScritturaDatiSinamics.Start :=false;
					END_IF;						
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX# STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1520;
				END_IF;
				
			_PROGRAM_ENABLEAX# STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1520:
				IF pm[CURRENT_AXIS].az_p1520>=0 THEN 
					IF NOT ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics(Start := Start_read,
											driveObjectId :=driveObjectIdConstant,
											hardwareId := hardwareIdConstant,
											parameterNumber:=uint#1520,
											index:=uint#0,
											value:=pm[CURRENT_AXIS].az_p1520);				
					ELSIF ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics.Start :=false;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1521;
					ELSIF ScritturaDatiSinamics.error THEN
						ScritturaDatiSinamics.Start :=false;
					END_IF;					
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1521;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1521:
				IF pm[CURRENT_AXIS].az_p1521>=0 THEN 
					IF NOT ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics(Start := Start_read,
											driveObjectId :=driveObjectIdConstant,
											hardwareId := hardwareIdConstant,
											parameterNumber:=uint#1521,
											index:=uint#0,
											value:=pm[CURRENT_AXIS].az_p1521);				
					ELSIF ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics.Start :=false;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1715;
					ELSIF ScritturaDatiSinamics.error THEN
						ScritturaDatiSinamics.Start :=false;
					END_IF;				
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1715;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1715:
				IF pm[CURRENT_AXIS].az_p1715>=0 THEN 
					IF NOT ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics(Start := Start_read,
											driveObjectId :=driveObjectIdConstant,
											hardwareId := hardwareIdConstant,
											parameterNumber:=uint#1715,
											index:=uint#0,
											value:=pm[CURRENT_AXIS].az_p1715);				
					ELSIF ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics.Start :=false;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1717;
					ELSIF ScritturaDatiSinamics.error THEN
						ScritturaDatiSinamics.Start :=false;
					END_IF;				
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1717;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1717:
				IF pm[CURRENT_AXIS].az_p1717>=0 THEN 
					IF NOT ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics(Start := Start_read,
											driveObjectId :=driveObjectIdConstant,
											hardwareId := hardwareIdConstant,
											parameterNumber:=uint#1717,
											index:=uint#0,
											value:=pm[CURRENT_AXIS].az_p1717);				
					ELSIF ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics.Start :=false;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_2175;
					ELSIF ScritturaDatiSinamics.error THEN
						ScritturaDatiSinamics.Start :=false;
					END_IF;					
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_2175;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_2175:

				IF NOT ScritturaDatiSinamics.Done THEN
					ScritturaDatiSinamics(Start := Start_read,
										driveObjectId :=driveObjectIdConstant,
										hardwareId := hardwareIdConstant,
										parameterNumber:=uint#2175,
										index:=uint#0,
										value:=pm[CURRENT_AXIS].az_p2175);				
				ELSIF ScritturaDatiSinamics.Done THEN
					ScritturaDatiSinamics.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_2177;
				ELSIF ScritturaDatiSinamics.error THEN
					ScritturaDatiSinamics.Start :=false;
				END_IF;	 		
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_2177:
				IF NOT ScritturaDatiSinamics.Done THEN
					ScritturaDatiSinamics(Start := Start_read,
										driveObjectId :=driveObjectIdConstant,
										hardwareId := hardwareIdConstant,
										parameterNumber:=uint#2177,
										index:=uint#0,
										value:=pm[CURRENT_AXIS].az_p2177);				
				ELSIF ScritturaDatiSinamics.Done THEN
					ScritturaDatiSinamics.Start :=false;
					Axes[CURRENT_AXIS].Request_Restart_Oggetto_Sinamics:=FALSE;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_2177;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE;
				ELSIF ScritturaDatiSinamics.error THEN
					ScritturaDatiSinamics.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHECK_PARAM_RW_2 :
				IF i_sys_Param_RW=2 THEN
					(*{#ifdef ALIMENTATORE_ACTIVE_LINE}*)
						IF i_sys_Param_AX_Number=999 THEN
							Address:=ADDRESS_POWER_SUPPLY;  //INDIRIZZO ALIMENTATORE
						ELSE
							LocAsse:=AsPositioningAxisRef(Axes[i_sys_Param_AX_Number].PosAx);
							(*Address:=UDINT_TO_DINT(LocAsse.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
						END_IF;
					(*{#else}*)	//Fine ALIMENTATORE_ACTIVE_LINE
						LocAsse:=AsPositioningAxisRef(Axes[i_sys_Param_AX_Number].PosAx);
						(*Address:=UDINT_TO_DINT(LocAsse.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
					(*{#endif}*)	//Fine NO ALIMENTATORE_ACTIVE_LINE
					IF NOT ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics(Start := Start_read,
											driveObjectId :=driveObjectIdConstant,
											hardwareId := hardwareIdConstant,
											parameterNumber:=TO_UINT(i_sys_Param_Number),
											index:=TO_UINT(i_sys_Param_Index),
											value:=TO_REAL(i_sys_Param_Value));				
					ELSIF ScritturaDatiSinamics.Done THEN
						ScritturaDatiSinamics.Start :=false;
						i_sys_Copy_RAM_ROM:=TRUE;
						i_sys_Param_RW:=0;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SYSTEM_STARTUP_WAIT_2;
					ELSIF ScritturaDatiSinamics.error THEN
						ScritturaDatiSinamics.Start :=false;
					END_IF;	 		
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SYSTEM_STARTUP_WAIT_2;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_SYSTEM_STARTUP_WAIT_2:
				IF AttesaAvvioSistema.OutputState<>GENERAL_STATE#STATE_END THEN
					AttesaAvvioSistema(AttesaSimotion:=TRUE, AttesaSinamics:=FALSE);
				ELSE
					AttesaAvvioSistema();
					IF i_sys_Copy_RAM_ROM THEN
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_COPY_RAM_ROM_2;
					ELSE
						CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS;
					END_IF;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_COPY_RAM_ROM_2:
				IF CopyRamRom.OutputState<>GENERAL_STATE#STATE_END THEN
					CopyRamRom();
				ELSE
					Init_trace();	//Azzeramento di tutte le trace
					IF CopyRomSinamic.OutputState<>GENERAL_STATE#STATE_END THEN
						CopyRomSinamic(CosaSalvare:='ALL');
					ELSE
						CopyRamRom();
						CopyRomSinamic();
						i_sys_Copy_RAM_ROM:=FALSE;
						i_sys_stopcond:=0; 
						CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS;
					END_IF;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS:
				//Solo se è la prima accensione o se è stato modificato un parametro effettua anche la lettura
				IF i_sys_first_on=TRUE OR PM_Modified=TRUE THEN		
					CURRENT_AXIS:=CURRENT_AXIS+1;
					IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
						IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE THEN
							LocAsse:=AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
							IF LocAsse^.sensor[1].Existent =true  THEN
								(*Address:=UDINT_TO_DINT(LocAsse.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WAIT_SINAMICS_FREE_READ;
							ELSE
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_SIMULATION_VERIFICATION;
							END_IF;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_SIMULATION_VERIFICATION;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ALIGN_ACCELERATION_VELOCITY;
					END_IF;
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ALIGN_ACCELERATION_VELOCITY;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_WAIT_SINAMICS_FREE_READ:
				IF NOT (i_sys_InUseReadingSinamics=TRUE) THEN
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_SIMULATION_VERIFICATION;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_SIMULATION_VERIFICATION:
				IF VerificationSimulationAxes()=FALSE THEN
					IF pm[CURRENT_AXIS].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS AND pm[CURRENT_AXIS].ma_asse_abilitato=TRUE THEN
						PosAx:=AsPositioningAxisRef(Axes[CURRENT_AXIS].posax);
						IF PosAx^.sensor[1].Existent =true AND Axes[CURRENT_AXIS].IsSiemensAxis THEN
							//Per essere sicuro di leggere i valori normali e non eventualmente limitati
							IF DisableAsseInCoppia.OutputState<>GENERAL_STATE#STATE_END THEN
								DisableAsseInCoppia(NumeroAsse:=CURRENT_AXIS, Verifica:=FALSE);
							ELSE
								DisableAsseInCoppia();
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_35;
							END_IF;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_RESTART_ACTIVATION_NO_PROFINET;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_RESTART_ACTIVATION_NO_PROFINET;
					END_IF;
				ELSE
					pm[CURRENT_AXIS].az_r1538:=pm[CURRENT_AXIS].AZ_P1520;
					pm[CURRENT_AXIS].az_r1539:=pm[CURRENT_AXIS].AZ_P1521;		
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_RESTART_ACTIVATION_NO_PROFINET;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_35:
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#35
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					Axes[CURRENT_AXIS].Temperatura_Motore:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_1441;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_1441:
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1441
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p1441:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_1460;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
	
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_1460:
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1460
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p1460:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1462;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;

				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1462 :
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1462
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p1462:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1520;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1520:
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1520
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p1520:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1521;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1521:
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1521
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p1521:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					IF PM_Modified=TRUE THEN
						//Lettura dei valori reali di coppia massima solo dopo che sono
						//stati impostati i parametri 1520 e 1521 e non ad ogni avvio del CN	
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1538;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1715        ;
					END_IF;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1538:
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1538
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p1520:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1539;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1539:
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1539
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_r1539:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
						IF ABS(pm[CURRENT_AXIS].az_r1538)<0.1 OR ABS(pm[CURRENT_AXIS].az_r1539)<0.1 THEN
							//In caso di errata attivazione dei parametri che indicano i limiti di coppia attivi,
							//si recuperano da quelli impostati da BlmMotion.
							//Serve per evitare che un^errata momentanea configurazione hardware degli azionamenti
							//comprometta la necessaria valorizzazione dei parametri r1538 e r1539
							pm[CURRENT_AXIS].az_r1538:=pm[CURRENT_AXIS].AZ_P1520;
							pm[CURRENT_AXIS].az_r1539:=pm[CURRENT_AXIS].AZ_P1521;
						END_IF;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1715        ;	
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;

				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1715        :
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1715
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p1715:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1717;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;

				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1717:

				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1717
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p1717:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_2175;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_2175:
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#2175
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p2175:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_2177;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_2177:

				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#2177
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					pm[CURRENT_AXIS].az_p2177:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1215;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1215:
				
				IF NOT LetturaDatoAzionamento.Done THEN
					LetturaDatoAzionamento	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     := uint#1215
										);
					(*parameterType      := 0;//add new, needed to be test*)
				ELSIF LetturaDatoAzionamento.Done THEN
					StatoFreno:=LetturaDatoAzionamento.Value_REAL;
					LetturaDatoAzionamento.Start :=false;
						IF StatoFreno=BRAKE_DISABLED THEN
							//Il freno se è presente non può mai essere disabilitato
							IF ManagementBrake.OutputState<>GENERAL_STATE#STATE_END THEN
								ManagementBrake(Asse:=CURRENT_AXIS, Stato:=BRAKE_ENABLED);
							ELSE
								LetturaDatoAzionamento();
								ManagementBrake();
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_RESTART_ACTIVATION_NO_PROFINET;
							END_IF;
					ELSE
						LetturaDatoAzionamento();
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_RESTART_ACTIVATION_NO_PROFINET;
					END_IF;
				ELSIF LetturaDatoAzionamento.error THEN
					LetturaDatoAzionamento.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_RESTART_ACTIVATION_NO_PROFINET:
				(*{#ifndef PROFINET}
					//Con le nuove macchine non serve in quanto il dato del TO viene mantenuto allineato al dato azionamento
					//sempre che sia attivo il parametro DriveControlConfig.dataAdaption
					IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE THEN
						PosAx:=AsPositioningAxisRef(Axes[CURRENT_AXIS].posax);
						IF PosAx^.sensor[1].Existent =true THEN
							// Axes[CURRENT_AXIS].PosAx.ActiveConfigData.TypeOfAxis.SetPointDriverInfo.DriveData.maxTorque<>pm[CURRENT_AXIS].az_r1538 THEN
							IF pm[CURRENT_AXIS].az_p1520<>pm[CURRENT_AXIS].az_r1538 THEN
							//Axes[CURRENT_AXIS].PosAx.SetConfigData.TypeOfAxis.SetPointDriverInfo.DriveData.maxTorque:=pm[CURRENT_AXIS].az_r1538;
								pm[CURRENT_AXIS].az_p1520:=pm[CURRENT_AXIS].az_r1538;	
								IF RestartActivationAsse.OutputState<>GENERAL_STATE#STATE_END THEN
									RestartActivationAsse(NumAsse:=CURRENT_AXIS, Attesa:=TRUE);
								ELSE
									RestartActivationAsse();
									CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS;
								END_IF;
							ELSE
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS;
							END_IF;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS;
					END_IF;
				(*{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ALIGN_ACCELERATION_VELOCITY:
				PM_Modified:=FALSE;
				//Allinea accelerazione e velocità di DEFAULT
				FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
					myPosAxis:=AsPositioningAxisRef(Axes[i].PosAx);
					IF myPosAxis <> NULL THEN
						//Necessario usare una variabile di appoggio
						myPosAxis^.DynamicDefaults.Deceleration:=pm[i].MA_max_ax_accel;
						myPosAxis^.DynamicDefaults.Acceleration:=pm[i].MA_max_ax_accel;
						myPosAxis^.DynamicDefaults.Velocity:=pm[i].max_ax_velo;
						// imposta la coppia limite di calibrazione = coppia massima asse
						myPosAxis^.Clamping.PositionTolerance:=pm[i].az_r1538;
						FollAsse:=AsSynchronousAxisRef(Axes[i].posax);
						FollAsse^.DynamicDefaults.Acceleration:=pm[i].MA_max_ax_accel;
						FollAsse^.DynamicDefaults.Deceleration:=pm[i].MA_max_ax_accel;
						FollAsse^.DynamicDefaults.Velocity:=pm[i].max_ax_velo;
					END_IF;  
				END_FOR;			
				CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENGINE_SPEED_NO_PROFINET_READ:
				(*{#ifndef PROFINET}
					//Con le nuove macchine non serve in quanto il dato del TO viene mantenuto allineato al dato azionamento
					//sempre che sia attivo il parametro DriveControlConfig.dataAdaption
					IF VerificationSimulationAxes()=FALSE THEN	
						//Alla messa in RUN della CPU si armonizza il numero giri standard del motore
						//indicato nei TO prendendolo da quello reale del motore, serve per le letture
						//in drive click dei motori per cui e^ previsto per gestire diverse taglie di
						//motori nello stesso progetto
						RiavvioDaFare:=FALSE;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_READ;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_GENERIC_DRIVE_READ;
					END_IF;
				(*{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_GENERIC_DRIVE_READ;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_READ:
				(*{#ifndef PROFINET}
					CURRENT_AXIS:=CURRENT_AXIS+1;
					IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
						PosAx:=AsPositioningAxisRef(Axes[CURRENT_AXIS].posax);
						IF PosAx^.sensor[1].Existent =true AND pm[CURRENT_AXIS].ma_Asse_Abilitato=TRUE THEN
							IF StartInRun=TRUE THEN
								(*Address:=UDINT_TO_DINT(Axes[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_WAIT_SINAMICS_FREE_READ;
							(*END_IF;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENGINE_SPEED_READ_COPY_RAM_ROM_NO_PROFINET;
					END_IF;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_WAIT_SINAMICS_FREE_READ:
				(*{#ifndef PROFINET}*)
					(*IF NOT (i_sys_InUseReadingSinamics=TRUE) THEN*)
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_READ_NO_PROFINET_READ_2000;
					(*END_IF;*)
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_READ_NO_PROFINET_READ_2000:
				(*{#ifndef PROFINET}
					IF NOT LetturaSinamicsREAL.Done THEN
						LetturaSinamicsREAL(Start := Start_read,
											hardwareId    := hardwareIdConstant,
											driveObjectId := driveObjectIdConstant,
											parameterNumber:=uint#2000,
											index:=uint#0);					
					ELSIF LetturaSinamicsREAL.Done THEN
						GiriMotore:=LetturaSinamicsREAL.Value_REAL;
						LetturaSinamicsREAL.Start :=false;
							PosAx:=AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
							IF ABS(PosAx^.Actor.DriveParameter.ReferenceSpeed-GiriMotore)>1 THEN
								//In caso di diversita^ fra il numero di giri del motore fra azionamenti e TO si 
								//adatta quest^utimo, copia in ROM e si chiede di riavviare il Simotion
								PosAx^.Actor.DriveParameter.ReferenceSpeed:=GiriMotore;
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_RESTART_READ;
							ELSE
								IF PosAx^.Actor.DriveParameter.MaxSpeed>2*GiriMotore THEN
									//La MaxSpeed non puo^ essere superiore a 2 volte la NominalSpeed
									PosAx^.Actor.DriveParameter.MaxSpeed:=2*GiriMotore;*)
									CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_RESTART_READ;
								(*ELSE
									CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_READ;
								(*END_IF;
							END_IF;
					ELSIF LetturaSinamicsREAL.error THEN
						LetturaSinamicsREAL.Start :=false;
					END_IF;	

				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_RESTART_READ: ;
				(*{#ifndef PROFINET}
					RiavvioDaFare:=TRUE;
					IF RestartActivationAsse.OutputState<>GENERAL_STATE#STATE_END THEN
						RestartActivationAsse(NumAsse:=CURRENT_AXIS, Attesa:=TRUE);
					ELSE
						RestartActivationAsse();*)
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_READ;
					(*END_IF;
				{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENGINE_SPEED_READ_COPY_RAM_ROM_NO_PROFINET:
				(*{#ifndef PROFINET}
					IF RiavvioDaFare=TRUE THEN
						//Memorizzazione in ROM eventuale modifica numero di giri
						IF CopyRamRom.OutputState<>GENERAL_STATE#STATE_END THEN
							CopyRamRom();
						ELSE
							CopyRamRom();
							RiavvioDaFare:=FALSE;
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_GENERIC_DRIVE_READ;
						END_IF;
					ELSE*)
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_GENERIC_DRIVE_READ;
					(*END_IF;
				{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_GENERIC_DRIVE_READ:
				StartInRun:=FALSE;
				//Richiesta lettura generica di un dato azionamento				
				IF i_sys_Param_RW=1 THEN
					(*{#ifdef ALIMENTATORE_ACTIVE_LINE}*)
						IF i_sys_Param_AX_Number=999 THEN ;
							(*Address:=ADDRESS_ALIMENTATORE;  //INDIRIZZO ALIMENTATORE*)
						ELSE
							LocAsse:=AsPositioningAxisRef(Axes[i_sys_Param_AX_Number].PosAx);
							(*Address:=UDINT_TO_DINT(LocAsse.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
						END_IF;
					(*{#else}	//Fine ALIMENTATORE_ACTIVE_LINE*)
						LocAsse:=AsPositioningAxisRef(Axes[i_sys_Param_AX_Number].PosAx);
						(*Address:=UDINT_TO_DINT(LocAsse.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
					(*{#endif}	//Fine NO ALIMENTATORE_ACTIVE_LINE*)	
					IF NOT (i_sys_InUseReadingSinamics=TRUE) THEN
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_GENERIC_DRIVE_READ_PARAM;
					END_IF;
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_GENERIC_DRIVE_READ_PARAM:

				IF NOT LetturaSinamicsREAL.Done THEN
					LetturaSinamicsREAL	(Start := Start_read,
										driveObjectId      := driveObjectIdConstant,
										hardwareId          := hardwareIdConstant,
										parameterNumber     :=TO_UINT(i_sys_Param_Number),
										index              := TO_UINT(i_sys_Param_Index));
				ELSIF LetturaSinamicsREAL.Done THEN
					i_sys_Param_Value:=LetturaSinamicsREAL.Value_REAL;
					LetturaSinamicsREAL.Start :=false;
					i_sys_Param_RW:=0;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP;
				ELSIF LetturaSinamicsREAL.error THEN
					LetturaSinamicsREAL.Start :=false;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_LOOP:
				NumeroAssi:=0;
				FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
					IF pm[i].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS AND pm[i].ma_Asse_Abilitato=TRUE THEN
						//Non controllo gli errori sugli assi disattivati
						PosAx:=AsPositioningAxisRef(Axes[i].PosAx);
						IF Posax^.StatusWord.%X1 AND Posax^.ErrorDetail.Reaction<> 0 THEN 
							NumeroAssi:=NumeroAssi+1;
						END_IF;	
					END_IF;
				END_FOR;
				IF Appo_prima_accensione=TRUE AND NumeroAssi>0 THEN
					//Durante il primo avviamento in presenza di eventuali errori sugli axes, si effettua
					//il reset per eliminarli, altrimenti non si attiverebbe mai la potenza
					iret:=reset_error();
					//dret:=_waitTime(T#100ms);
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ORBITAL_XX_RESET;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_ORBITAL_XX_RESET:
				(*{#ifdef ORBITALE}*)
					IF Appo_prima_accensione=TRUE THEN
						IF ResettingXxOrbita.outputState<>GENERAL_STATE#STATE_END THEN
							ResettingXxOrbita();
						ELSE
							ResettingXxOrbita();
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLEABLE_AXIS_STATE_TEST;
						END_IF;	
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLEABLE_AXIS_STATE_TEST;
					END_IF;
				(*{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLEABLE_AXIS_STATE_TEST;
				(*{#endif};*)

		_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLEABLE_AXIS_STATE_TEST:
				IF TestDeactivationAxesState(VerificaPm105:=TRUE, ErroreAsse:=NumeroAsse)=FALSE AND i_sys_CmdVGPCSettings=FALSE THEN
					//Verifica congruenza stato motore azionamento con situazione trasferita da VGP
					IF i_sys_Accesslevel<=ACCESS_SAT OR CheckIOAddresses()=FALSE THEN
						//Messagi per BLM che può usare il 242
						set_trace(Code:=TRC_EXECUTE_MOTOR_SETTING, Axis:=NumeroAsse);
						reset_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);
					END_IF;
					i_sys_stopcond:=1;				
				ELSE
					reset_trace(Code:=TRC_EXECUTE_MOTOR_SETTING);
					IF NumeroAssi=0 THEN
						i_sys_enable_alimen:=TRUE;
						iret:=reset_error();
						//dret:=_waitTime(T#20ms);
						//Non controlla i limiti software per gli assi che non hanno fatto la presa index
						FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
							IF  CheckIfTOactive(Axis:=NumeroAsse)=TRUE THEN
								PosAx:= AsPositioningAxisRef(Axes[_FIRST_AXIS#FIRST_AXIS].PosAx);
								IF NOT PosAx^.StatusWord.%X5  THEN 
								//IF Axes[i].PosAx.PositioningState.Homed=NO THEN 
								PosAx^.PositionLimits_SW.Active := FALSE;
									//Assi[i].PosAx.swlimit.state:=INACTIVE;
								ELSE
								PosAx^.PositionLimits_SW.Active := TRUE;   
									//Assi[i].PosAx.swlimit.state:=ACTIVE;
								END_IF;
							END_IF;
						END_FOR;	
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_Z1;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_STARTUP_FALSE_FINAL;
					END_IF;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_Z1:
				(*{#ifdef CURVATURA}
					{#ifndef DH4010}*)
						IF i_sys_optimize_Z_preset =1 THEN
							IF Modulo_Asse.OutputState<>GENERAL_STATE#STATE_END THEN
								Modulo_Asse(Asse:=Z1);
							ELSE
								Modulo_Asse();
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_B1_B2;
							END_IF;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_B1_B2;
						END_IF;
					(*{#else}*)
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_B1_B2;
					(*{#endif};
				{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_B1_B2;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_B1_B2:
				(*{#ifdef RUNNER}
					{#ifdef B1_B2}*)
						IF Module_2Axes.outputState<>GENERAL_STATE#STATE_END THEN
							Module_2Axes(Asse1:=B1, Asse2:=B2, Attesa:=FALSE);
						ELSE
							Module_2Axes();
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_A1_A2;
						END_IF;
					(*{#else}*)
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_A1_A2;
					(*{#endif};
				{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_FOUR_RUNNER_PARAMETERS_SET;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_A1_A2:
				(*{#ifdef A1_A2}*)
					IF (pm[A1].ma_asse_abilitato=TRUE AND pm[A1].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS) AND 
						 (pm[A2].ma_asse_abilitato=TRUE AND pm[A2].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS) THEN
						IF Module_2Axes.outputState<>GENERAL_STATE#STATE_END THEN
							Module_2Axes(Asse1:=A1, Asse2:=A2, Attesa:=FALSE);
						ELSE
							Module_2Axes();
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_FOUR_RUNNER_PARAMETERS_SET;
						END_IF;						 
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_FOUR_RUNNER_PARAMETERS_SET;
					END_IF;
				(*{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_FOUR_RUNNER_PARAMETERS_SET;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_CHANGE_FOUR_RUNNER_PARAMETERS_SET:
				(*{#ifdef FOUR_RUNNER}*)
					IF pm[A1].ma_asse_abilitato=TRUE AND pm[A2].ma_asse_abilitato=TRUE THEN
						IF CambiaSetParametri.OutputState<>GENERAL_STATE#STATE_END THEN
							CambiaSetParametri(Axis:=Axes[A1].PosAx,
																 NumeroAsse:=A1,
																 NumeroSetParametri:=3,
																 ProssimoComando:=EnumNextCommandEnable#WHEN_COMMAND_DONE);						
						ELSE
							IF Module_2Axes.outputState<>GENERAL_STATE#STATE_END THEN
								Module_2Axes(Asse1:=A1, Asse2:=A2, Attesa:=FALSE);
							ELSE
								CambiaSetParametri();
								Module_2Axes();
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SYSTEM_STARTUP_3_WAIT;
							END_IF;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SYSTEM_STARTUP_3_WAIT;
					END_IF;
				(*{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SYSTEM_STARTUP_3_WAIT;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_SYSTEM_STARTUP_3_WAIT:
				IF AttesaAvvioSistema.OutputState<>GENERAL_STATE#STATE_END THEN
					AttesaAvvioSistema(AttesaSimotion:=TRUE, AttesaSinamics:=FALSE);
				ELSE
					AttesaAvvioSistema();
					CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP:
				CURRENT_AXIS:=CURRENT_AXIS+1;
				IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
					IF CheckIfTOactive(Axis:=CURRENT_AXIS)=TRUE THEN
						On_The_PM_line(Axis:=CURRENT_AXIS);
						PosAx:=AsPositioningAxisRef(Axes[CURRENT_AXIS].posax);
						IF PosAx^.VirtualAxis.Mode =UDINT#0 (*Axes[CURRENT_AXIS].posax.typeofaxis<>VIRTUAL_AXIS*) AND pm[CURRENT_AXIS].ma_Asse_Abilitato=TRUE THEN
							IF pm[CURRENT_AXIS].ma_asse_installato=ACTIVE_DEACTIVATABLE_AXIS THEN
								IF PosAx^.Sensor[1].Existent =true (*Assi[CURRENT_AXIS].posax.activeconfigdata.TypeOfAxis.NumberOfEncoders.numberofencoders>1*) THEN
									IF PosAx^.Sensor[2]._Interface.AddressIn.DB_NUMBER > UINT#0  (*Assi[CURRENT_AXIS].posax.activeconfigdata.TypeOfAxis.NumberOfEncoders.encoder_2.encodermode=SENSOR_ANALOG*) THEN	
										//Riabilita il primo encodere negli assi che potrebbero essere disabilitati ed hanno il secondo encoder ANALOG
										//myCommandID:=_getCommandID();
										(*{#ifdef DH4010}*)
											IF i_sys_first_on=TRUE THEN
												//L^abilitazione dell^asse avviene solo durante la prima accensione. Serve per evitare
												//di cambiare il dataset di X6 ad ogni pressione del tasto di reset.
												//Se funzionerà si potrà eseguire la medesima gestione per tutte le altre macchine
												CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP_CHANGE_PARAMETERS;
											END_IF;
										(*{#else}*)
											CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP_CHANGE_PARAMETERS;
										(*{#endif};*)
									END_IF;
								END_IF;
							ELSE
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLE_LOOP_L1;
							END_IF;
						END_IF;			
					END_IF;				
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_SAFETY_DATA_BEFORE_STARTUP;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP_CHANGE_PARAMETERS:
				IF CambiaSetParametri.OutputState<>GENERAL_STATE#STATE_END THEN
					CambiaSetParametri(Axis:=Axes[CURRENT_AXIS].PosAx,
														 NumeroAsse:=CURRENT_AXIS,
														 NumeroSetParametri:=1,
														 ProssimoComando:=EnumNextCommandEnable#WHEN_COMMAND_DONE);					
				ELSE
					myPosAxis^.Simulation.Mode := UDINT#0;
					_resetAxisError.Restart:=TRUE;

					_resetAxisError.Execute:=TRUE;
					
					IF  (_resetAxisError.Done) THEN
						CambiaSetParametri();
					(*dret:=_disableAxisSimulation(axis:=Assi[i].PosAx, 
																			 nextCommand:=IMMEDIATELY, 
																			 commandId:=myCommandID);*)	
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP;
					END_IF;
				END_IF;
			
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_DISABLE_LOOP_L1:
				IF SiTestL1(Asse:=CURRENT_AXIS)=TRUE THEN 
					IF pm[CURRENT_AXIS].ma_asse_installato=INACTIVE_DEACTIVATABLE_AXIS THEN
						IF DisableAssi.OutputState<>GENERAL_STATE#STATE_END THEN
							DisableAssi(Asse:=CURRENT_AXIS (* , Regolatore:=INACTIVE*));	
						ELSE
							DisableAssi();
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP;
					END_IF;
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_ENCODER_LOOP;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_READ_SAFETY_DATA_BEFORE_STARTUP:
				IF i_sys_first_on=TRUE THEN
					//Lettura dati encoder degli assi con safety estesa, si effettua solo una volta all^avvio in quanto
					//la lettura impiega molto tempo ed il confronto non può essere eseguito ad ogni reset.	
					IF ReadDatiSafety.OutputState<>GENERAL_STATE#STATE_END THEN
						ReadDatiSafety();
					ELSE
						ReadDatiSafety();
						CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_DRIVER_PRESSED;
					END_IF;
				ELSE
					CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_DRIVER_PRESSED;				
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_DRIVER_PRESSED:
				IF i_sys_ResetPressedDriver=TRUE THEN
					CURRENT_AXIS:=CURRENT_AXIS+1;
					IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
						IF CheckIfTOactive(Axis:=CURRENT_AXIS)=TRUE AND Axes[CURRENT_AXIS].IsSiemensAxis THEN
							PosAx:= AsPositioningAxisRef(Axes[CURRENT_AXIS].posax);
							IF PosAx^.VirtualAxis.Mode=UDINT#0 AND pm[CURRENT_AXIS].ma_Asse_Abilitato=TRUE THEN
								//Tacitazione anomalie sugli azionamenti
								(*Address:=UDINT_TO_DINT(Assi[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)

								IF NOT ScritturaDatiSinamics.Done THEN
									ScritturaDatiSinamics(Start := Start_read,
														driveObjectId :=driveObjectIdConstant,
														hardwareId := hardwareIdConstant,
														parameterNumber:=uint#3981,
														index:=uint#0,
														value:=1);				
								ELSIF ScritturaDatiSinamics.Done THEN
									ScritturaDatiSinamics.Start :=false;
								ELSIF ScritturaDatiSinamics.error THEN
									ScritturaDatiSinamics.Start :=false;
								END_IF;	 
							END_IF;
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_DRIVER_PRESSED_MECHANICAL_SAFETY_TEST;
					END_IF;
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_0_360_TESTHEADS;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_DRIVER_PRESSED_MECHANICAL_SAFETY_TEST:
				(*{#ifdef PROFINET}*)
					ErroreAsse:=MechanicalSafetyTest();
					//Ad ogni pressione del tasto reset si verifica che tutti gli assi con safety estesa 
					//abbiano i dati della meccanica congruenti fra azionamento ed oggetto tecnologico;
					IF ErroreAsse>-1 THEN
						iret:=Push_error(Code:=ERR_CONFIG_MECHANICS_SAFETY , Asse:=ErroreAsse);	
					ELSE
						IF PerformSafetyReset=TRUE THEN
							//Durante l^avvio del sistema è stato rilevata un^anomalia Safety, occorre spegnere
							//il CN per cercare di tacitarla	
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_DRIVER_PRESSED_ENDLESS_LOOP_MECHANICAL_SAFETY;
						ELSE
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_0_360_TESTHEADS;
						END_IF;
					END_IF;
				(*{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_0_360_TESTHEADS;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_RESET_DRIVER_PRESSED_ENDLESS_LOOP_MECHANICAL_SAFETY:
				(*{#ifdef PROFINET}*)
					PerformSafetyReset:=FALSE;
					iret:=Push_error(Code:=ERR_SAFETY, Asse:=ErroreAsse);	
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_0_360_TESTHEADS:
				(*{#ifdef EFLEX_DH4010}*)
					IF i_sys_first_on=TRUE THEN
						IF Modulo_0_360.outputState<>GENERAL_STATE#STATE_END THEN
							Modulo_0_360(Asse:=Y1);
						ELSE
							Modulo_0_360();
							(*{#ifdef DH4010}t*)
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_0_360_TESTHEADS_2;
							(*{#else}*)
								CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_STARTUP_FALSE;
							(*{#endif};*)
						END_IF;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_STARTUP_FALSE;
					END_IF;
				(*{#else}*)
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_STARTUP_FALSE;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_MODULE_0_360_TESTHEADS_2:
				(*{#ifdef DH4010}*)
					IF Modulo_0_360.outputState<>GENERAL_STATE#STATE_END THEN
						Modulo_0_360(Asse:=Y2);
					ELSE
						Modulo_0_360();
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_STARTUP_FALSE;
					END_IF;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_STARTUP_FALSE:
				i_sys_ResetPressedDriver:=FALSE;
				i_sys_first_on:=FALSE; 
				reset_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);	
				IF EnableAssi.OutputState<>GENERAL_STATE#STATE_END THEN
					EnableAssi();
				ELSE
					IF EnableAssi.ReturnValue=TRUE THEN
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_NUM_STATES_TEST;
					END_IF;
					EnableAssi();
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_NUM_STATES_TEST:
				IF TestNumStati(_Program:=PROGRAM_JOG)=FALSE THEN
					IF OPS_Active =TRUE AND OPS_StartDate=FALSE THEN
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MASTER_VELOCITY_CHECK;
					ELSE
						IF i_sys_activedevice<>UNDEF_DEVICE OR i_sys_opmode<>JOGMODE THEN			
							i_sys_activedevice:=UNDEF_DEVICE;
							iret:=Push_error(Code:=ERR_NUMSTATES);
						END_IF;
					END_IF;
				ELSE
					CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_MASTER_VELOCITY_CHECK;
				END_IF;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_MASTER_VELOCITY_CHECK:
				(*{#ifdef DH4010}*)
					set_trace(Code:=TRC_INCONGRUENCE_MANIPULATOR2D);
					IF disp[LOCK_2D].Enable<>Manipolatore2D_Attivo THEN
						i_sys_stopcond:=1;
					ELSE
						reset_trace(TRC_INCONGRUENCE_MANIPULATOR2D);
						IF masteraxis= AsPositioningAxisRef(Axes[X1].masteraxis) AND masteraxis^.DynamicLimits.MaxVelocity<>1 OR
							masteraxis= AsPositioningAxisRef(Axes[X2].masteraxis) AND masteraxis^.DynamicLimits.MaxVelocity<>1 THEN
						(*IF Axes[X1].masteraxis.activeconfigdata.typeofaxis.MaxVelocity.maximum<>1 OR
							 Axes[X2].masteraxis.activeconfigdata.typeofaxis.MaxVelocity.maximum<>1 THEN*)
							//Verifica che il Master abbia velocita^ 1
							dret:=Push_error(Code:=ERR_GRAVE_DOWNLOAD);
						END_IF;		
						i_sys_stopcond:=0;
						i_sys_RUNOK:=1;
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DEVICES_ENABLE_LOOP;
					END_IF;
				(*{#else}*)
				IF masteraxis= AsPositioningAxisRef(Axes[X1].masteraxis) AND masteraxis^.DynamicLimits.MaxVelocity<>1 THEN
					//IF Assi[X1].masteraxis.activeconfigdata.typeofaxis.MaxVelocity.maximum<>1 THEN
						//Verifica che il Master abbia velocita^ 1
						dret:=Push_error(Code:=ERR_GRAVE_DOWNLOAD);
					ELSE
						i_sys_stopcond:=0;
						i_sys_RUNOK:=1;
						(*{#ifdef MS0017}*)
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_HOMING_CHECK_U2;
						(*{#else}*)
							CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DEVICES_ENABLE_LOOP;
						(*{#endif};*)
					END_IF;				
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_HOMING_CHECK_U2:
				(*{#ifdef MS0017}*)
					PosAx:= AsPositioningAxisRef(Axes[U2].PosAx);
					IF ABS(PosAx^.actualPosition)<0.1 AND 
					 	 posax^.VirtualAxis.Mode=UDINT#1 THEN
						Homing(Asse:=U2, Quota:=OldPosAxe[U2], Modo:=EnumHomingMode#DIRECT_HOMING_ABSOLUTE (*Modo:=DIRECT_HOMING*));
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DEVICES_ENABLE_LOOP;
					ELSE
						CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_DEVICES_ENABLE_LOOP;
					END_IF;
				(*{#endif};*)
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_DEVICES_ENABLE_LOOP:
				FOR i:=FIRST_DEVICE TO (_MAXDEVICES#MAXDEVICES+FIRST_DEVICE-1) DO
					//Disabilitazione movimento dispositivi
					AbilitazioneDispositivi(Dispositivo:=i, Stato:=TRUE);
				END_FOR;
				CURRENT_STATE:=_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_STARTUP_FALSE_FINAL;
				
			_PROGRAM_ENABLEAX#STATE_ENABLEAX_SET_STARTUP_FALSE_FINAL:
				i_sys_first_on:=FALSE;
				i_sys_EnableBusy:=FALSE;
				//Inizializzazione della variabile a -1 per forzare la lettura della temperatura
				//dei motori e così all^avviamento di poter attivare il feed immediatamente e non
				//attendere 1 minuto
				Appo_global :=-1;
				(*{#ifdef LINEARIZZA_ASSE} %%%%
					// FOR i:=0 TO MAX_AXIS_LINEARISES-1 DO
					// 	PosAx:=AsPositioningAxisRef(Axes[i].PosAx);
					// 	Axes[i].PosAx.servosettings.additionalcommandvalueswitch:=YES;
					// END_FOR;
				(*{#endif}	//Fine LINEARIZZA_ASSE	*)
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
				
		 END_CASE;
		//dret:=_waitTime(T#0ms);

/*********************
 * PLC_OPEN_COMMANDS *
 *********************/
		_resetAxisError (Axis:=myPosAxis^);
		
		//END_WHILE;

END_PROGRAM 


END_NAMESPACE

