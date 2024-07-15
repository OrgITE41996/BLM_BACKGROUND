USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING System.BitAccess;
USING Simatic.Ax.LAcycCom;
USING ENABLEPROGRAM;
USING ENABLEAX_FUNCTIONS;
USING  _TYPE;
USING _STATE_CONST;
USING Retain_Namespase;
USING _FC_IO_FC;
USING _CONST_VAR_DEF;
USING FC_N;
USING System.Math;
USING _FC_FC;
USING _CONST_VAR_DEF;
USING ENABLEAX_FUNCTIONS;
USING _UTILITIES_FB;

NAMESPACE _FC_FB
/*******************************************************************************************************************
 * IF POSAXIS ARE VIRTUAL AXIS, GET THE AXIS POSITION, THEN ACTIVATE THE POSITION, SET SYNCHRONIZATION, ACTIVATE SYNCHRONIZATION, AND FINALLY ENABLE THE AXIS. *
 *******************************************************************************************************************/
FUNCTION_BLOCK FB_SettingsTO 

    VAR_EXTERNAL
        Axes                        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
        pm                      	: ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    END_VAR

    VAR_INPUT
        Axis	:INT;
        State		:BOOL;
    END_VAR

    VAR
            CURRENT_STATE           :INT:=GENERAL_STATE#STATE_INIT;
            
            dret		            :DINT;
           (*  StatePosAx	            :StructRetGetStateOfTo;
            StateSynchro            :StructRetGetStateOfTo;
            getCommandIDPosAx       :CommandIDType;
            getCommandIDSynchro     :CommandIDType;
            setCommandIDPosAx       :CommandIDType;
            setCommandIDSynchro     :CommandIDType;
            actModePosAx            :EnumReqActDeactGetStateMode;
            actModeSynchro          :EnumReqActDeactGetStateMode;*)
    END_VAR

    VAR_OUTPUT
            OutputState:INT;
    END_VAR
    ;
        (*{
        #ifdef PROFINET
        }
            CASE CURRENT_STATE OF 
                GENERAL_STATE#STATE_INIT:
                    IF Axes[Axis].posax.typeofaxis<>VIRTUAL_AXIS THEN
                        getCommandIDPosAx:=_getCommandID();
                        getCommandIDSynchro:=_getCommandID();
                        setCommandIDPosAx:=_getCommandID();
                        setCommandIDSynchro:=_getCommandID();
                        CURRENT_STATE:=SettingsTO#STATE_GET_POSAX_TO_STATE;
                    ELSE
                        CURRENT_STATE:=GENERAL_STATE#STATE_END;
                    END_IF;
                    
                SettingsTO#STATE_GET_POSAX_TO_STATE:
                    StatePosAx:=_getstateofto(to_instance:=Axes[Axis].PosAx,
                                                                     reqactdeactgetstatemode:=actModePosAx,
                                                                     commandid:=getCommandIDPosAx,
                                                                     nextcommand:=IMMEDIATELY);
                    IF (StatePosAx.functionResult=16#7001 OR StatePosAx.functionResult=16#7002) THEN
                        actModePosAx:=REQUEST_FALSE;
                    ELSE
                        IF StatePosAx.functionResult=16#0000 THEN
                            actModePosAx:=REQUEST_TRUE;
                            CURRENT_STATE:=SettingsTO#STATE_ACTIVATE_DEACTIVATE_POSAX_TO;
                        ELSE
                            actModePosAx:=REQUEST_TRUE;
                            getCommandIDPosAx:=_getCommandID();
                        END_IF;
                    END_IF;
                    
                SettingsTO#STATE_ACTIVATE_DEACTIVATE_POSAX_TO:
                    IF (State=TRUE) THEN
                        CURRENT_STATE:=SettingsTO#STATE_ACTIVATE_POSAX_TO;
                    ELSE
                        CURRENT_STATE:=SettingsTO#STATE_DEACTIVATE_POSAX_TO;
                    END_IF;
                    
                SettingsTO#STATE_ACTIVATE_POSAX_TO:
                    IF StatePosAx.commandIdState<>ACTIVE THEN
                        dret:=_activateTo(TO_Instance:=Axes[Axis].posAx,
                                                            reqActDeactGetStateMode:=actModePosAx,
                                                            commandid:=setCommandIDPosAx,
                                                            nextCommand:=IMMEDIATELY);
                        IF dret=16#7001 OR dret=16#7002 THEN
                            actModePosAx:=REQUEST_FALSE;
                        ELSE
                            IF dret=16#0000 THEN
                                actModePosAx:=REQUEST_TRUE;
                                setCommandIDPosAx:=_getCommandId();
                                CURRENT_STATE:=STATE_GET_SYNCHRO_TO_STATE;
                            ELSE
                                actModePosAx:=REQUEST_TRUE;
                                setCommandIDPosAx:=_getCommandID();
                            END_IF;
                        END_IF;
                    ELSE
                        CURRENT_STATE:=SettingsTO#STATE_GET_SYNCHRO_TO_STATE;
                    END_IF;
                    
                SettingsTO#STATE_DEACTIVATE_POSAX_TO:
                    IF StatePosAx.commandIdState=ACTIVE THEN
                        dret:=_deactivateTo(TO_Instance:=Axes[Axis].posAx,
                                                            reqActDeactGetStateMode:=actModePosAx,
                                                            commandid:=setCommandIDPosAx,
                                                            nextCommand:=IMMEDIATELY);
                        IF dret=16#7001 OR dret=16#7002 THEN
                            actModePosAx:=REQUEST_FALSE;
                        ELSE
                            IF dret=16#0000 THEN
                                actModePosAx:=REQUEST_TRUE;
                                setCommandIDPosAx:=_getCommandId();
                                CURRENT_STATE:=SettingsTO#STATE_GET_SYNCHRO_TO_STATE;
                            ELSE
                                actModePosAx:=REQUEST_TRUE;
                            END_IF;
                        END_IF;
                    ELSE
                        CURRENT_STATE:=SettingsTO#STATE_GET_SYNCHRO_TO_STATE;
                    END_IF;	
                    
                SettingsTO#STATE_GET_SYNCHRO_TO_STATE:
                    StateSynchro:=_getstateofto(to_instance:=Axes[Axis].sincroasse,
                                                                         reqactdeactgetstatemode:=actModeSynchro,
                                                                         commandid:=getCommandIDSynchro,
                                                                         nextcommand:=IMMEDIATELY);
                    IF (StateSynchro.functionResult=16#7001 OR StateSynchro.functionResult=16#7002) THEN
                        actModeSynchro:=REQUEST_FALSE;
                    ELSE
                        IF StateSynchro.functionResult=16#0000 THEN
                            actModeSynchro:=REQUEST_TRUE;
                            CURRENT_STATE:=SettingsTO#STATE_ACTIVATE_DEACTIVATE_SYNCHRO_TO;
                        ELSE
                            actModeSynchro:=REQUEST_TRUE;
                            getCommandIDSynchro:=_getCommandID();
                        END_IF;
                    END_IF;
                    
                SettingsTO#STATE_ACTIVATE_DEACTIVATE_SYNCHRO_TO:
                    IF (State=TRUE) THEN
                        CURRENT_STATE:=SettingsTO#STATE_ACTIVATE_SYNCHRO_TO;
                    ELSE
                        CURRENT_STATE:=SettingsTO#STATE_DEACTIVATE_SYNCHRO_TO;
                    END_IF;
                    
                SettingsTO#STATE_ACTIVATE_SYNCHRO_TO:
                    IF StatePosAx.commandIdState<>ACTIVE THEN
                        dret:=_activateTo(TO_Instance:=Axes[Axis].sincroasse,
                                                            reqActDeactGetStateMode:=actModeSynchro,
                                                            commandid:=setCommandIDSynchro,
                                                            nextCommand:=IMMEDIATELY);
                        IF dret=16#7001 OR dret=16#7002 THEN
                            actModeSynchro:=REQUEST_FALSE;
                        ELSE
                            IF dret=16#0000 THEN
                                actModeSynchro:=REQUEST_TRUE;
                                setCommandIDSynchro:=_getCommandId();
                                CURRENT_STATE:=GENERAL_STATE#STATE_END;
                                
                                pm[Axis].ma_Asse_Abilitato:=TRUE;
                                Axes[Axis].ma_Asse_Abilitato_shadow:=TRUE;
                            ELSE
                                actModeSynchro:=REQUEST_TRUE;
                                setCommandIDSynchro:=_getCommandID();
                            END_IF;
                        END_IF;
                    ELSE
                        pm[Axis].ma_Asse_Abilitato:=TRUE;
                        Axes[Axis].ma_Asse_Abilitato_shadow:=TRUE;
                        CURRENT_STATE:=GENERAL_STATE#STATE_END;
                    END_IF;
                
                SettingsTO#STATE_DEACTIVATE_SYNCHRO_TO:
                    IF StatePosAx.commandIdState=ACTIVE THEN
                        dret:=_deactivateTo(TO_Instance:=Axes[Axis].sincroasse,
                                                            reqActDeactGetStateMode:=actModeSynchro,
                                                            commandid:=setCommandIDSynchro,
                                                            nextCommand:=IMMEDIATELY);
                        IF dret=16#7001 OR dret=16#7002 THEN
                            actModeSynchro:=REQUEST_FALSE;
                        ELSE
                            IF dret=16#0000 THEN
                                actModeSynchro:=REQUEST_TRUE;
                                setCommandIDSynchro:=_getCommandId();
                                
                                pm[Axis].ma_Asse_Abilitato:=FALSE;
                                Axes[Axis].ma_Asse_Abilitato_shadow:=FALSE;
                                
                                CURRENT_STATE:=GENERAL_STATE#STATE_END;
                            ELSE
                                actModeSynchro:=REQUEST_TRUE;
                            END_IF;
                        END_IF;
                    ELSE
                        pm[Axis].ma_Asse_Abilitato:=FALSE;
                        Axes[Axis].ma_Asse_Abilitato_shadow:=FALSE;
                        CURRENT_STATE:=GENERAL_STATE#STATE_END;
                    END_IF;	
                
                    GENERAL_STATE#STATE_END:
                    CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
            END_CASE;
            OutputState:=CURRENT_STATE;
        (*{
        #endif	//Fine PROFINET
        }
        ;*)
END_FUNCTION_BLOCK


FUNCTION_BLOCK FB_ReadingSinamicsREAL

    VAR_EXTERNAL
        i_sys_ReadingSinamicsInUse      :BOOL;
    END_VAR

    VAR_EXTERNAL CONSTANT
        TRC_READ_DRIVE_PARAMETERS        :INT;  //Waiting for drive parameters reading
    END_VAR

    VAR_INPUT
      log_address :DINT;
      par_number  :UDINT;
      sub_index   :UDINT;
      Verifica    :BOOL;
    END_VAR
    VAR
      Descrizione : LAcycCom_classReadDriveSingleParams; //StructRetReadDriveParameterDescription;
      Dati        : LAcycCom_classReadDriveParams;//StructRetReadDriveParameter;
      Errore1     :DINT;
      Errore2     :DINT;
      dret        :DINT;
      i           :DINT;
          
    CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
        //thisCommand		:commandIDType;
    END_VAR
      VAR_OUTPUT
          OutputState	:INT;
          ReturnValue	:REAL;
      END_VAR
      ;
      CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			(*IF VerificaPresenzaAzionamento(IndirizzoLogico:=log_address)=TRUE THEN
				i_sys_ReadingSinamicsInUse:=TRUE;
				IF VerificationSimulationAxes()=FALSE THEN
					Errore1:=1;
					Errore2:=1;
					i:=0;
					thisCommand:=_getCommandID();
					CURRENT_STATE:=ReadingSinamicsREAL#STATE_READ_SINAMICS_REAL_READ_PARAMETER;
				ELSE
					ReturnValue:=0;
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			ELSE
				ReturnValue:=0;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;*)
			;
        ReadingSinamicsREAL#STATE_READ_SINAMICS_REAL_READ_PARAMETER:
			IF (Errore1<>0 OR Errore2<>0) THEN;
        (*Dati:=_readdriveparameter Dati.execute (ADDR_TAG:= log_address);
                                                (ioid:=INPUT,
                                                logaddress:=log_address,
                                                parameternumber:=par_number,
                                                numberOfElements:=1,
                                                subIndex:=sub_index,
                                                nextcommand:=IMMEDIATELY,
                                                commandid:=thisCommand);*)
				(*IF NOT (Dati.AddatasetItem() =16#7001 OR Dati.AddatasetItem()=16#7002) (Dati.functionResult=16#7001 OR Dati.functionResult=16#7002) THEN
					thisCommand:=_getCommandID();
					IF Verifica THEN
						Errore1:=Dati.functionResult;
						Errore2:=Dati.parameterResult;
						i:=i+1;
						IF i>1000 THEN
							set_trace(Code:=TRC_READ_DRIVE_PARAMETERS);
							i:=0;
						END_IF;
					ELSE
						Errore1:=0;
						Errore2:=0;
					END_IF;
				END_IF;*)
			ELSE
				reset_trace(TRC_READ_DRIVE_PARAMETERS);
				Errore1:=1;
				Errore2:=1;
				i:=0;
				//thisCommand:=_getCommandID();
				CURRENT_STATE:=ReadingSinamicsREAL#STATE_READ_SINAMICS_REAL_READ_DESCRIPTION;
			END_IF;
			
            ReadingSinamicsREAL#STATE_READ_SINAMICS_REAL_READ_DESCRIPTION:
			(*IF (Errore1<>0 OR Errore2<>0) THEN
				Descrizione:=_readdriveparameterdescription(logaddress:=log_address,
																										(*parameternumber:=par_number,
																										nextcommand:=when_command_done,
																										commandid:=_getCommandID());*)
				(*IF NOT (Descrizione.functionResult=7001 OR Descrizione.functionResult=7002) THEN
					IF Verifica=TRUE THEN
						Errore1:=Descrizione.functionResult;
						Errore2:=Descrizione.parameterResult;
						i:=i+1;
						IF i>1000 THEN
							set_trace(Code:=TRC_READ_DRIVE_PARAMETERS);
							i:=0;
						END_IF;
					ELSE
						Errore1:=0;
						Errore2:=0;
					END_IF; 
				END_IF;*)
			(*ELSE
				reset_trace(TRC_READ_DRIVE_PARAMETERS);
							
				CASE Descrizione.datatype OF
					
					8:  ///// Floating_point
					 ReturnValue:=BIGBYTEARRAY_TO_ANYTYPE(bytearray:=Dati.data);
					
					7:  ////// Unsigned32
					 ReturnValue:=UDINT_TO_REAL(BIGBYTEARRAY_TO_ANYTYPE(bytearray:=Dati.data));
			
					6:  ///// Unsigned16
					 ReturnValue:=UINT_TO_REAL(BIGBYTEARRAY_TO_ANYTYPE(bytearray:=Dati.data));
					
					5:  ///// Unsigned8
					 ReturnValue:=USINT_TO_REAL(BIGBYTEARRAY_TO_ANYTYPE(bytearray:=Dati.data));
			
					4:  ; ///// DINT
					
					3:  ;///// INT
					 ReturnValue:=INT_TO_REAL(BIGBYTEARRAY_TO_ANYTYPE(bytearray:=Dati.data));
				END_CASE;
				i_sys_ReadingSinamicsInUse:=FALSE;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;*);
			
            GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;

	END_CASE;
	OutputState:=CURRENT_STATE;
      ;

END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_CopyRamRom

    VAR_EXTERNAL CONSTANT
        TRC_COPY_RAM_ROM                    :INT; // Waiting for copy from RAM to ROM 
    END_VAR
    
    VAR
      //Esito   :EnumSaveState;
      dret    :DINT;
          
          CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
    END_VAR
      VAR_OUTPUT
          OutputState		:INT;
      END_VAR
      
      CASE CURRENT_STATE OF
        GENERAL_STATE#STATE_INIT:
              Init_trace();  //Azzeramento di tutte le trace
              //Esito:=SAVE_ABORTED;
              CURRENT_STATE:=FB_COPY_RAM_ROM#STATE_COPY_RAM_ROM_WAIT_SAVE_FINISHED;
              
        FB_COPY_RAM_ROM#STATE_COPY_RAM_ROM_WAIT_SAVE_FINISHED:
        ;
              (*IF Esito<>SAVE_FINISHED THEN
                  set_trace(Code:=TRC_COPY_RAM_ROM);
                  Esito:=_saveConfigData();
              ELSE
                  reset_trace(Code:=TRC_COPY_RAM_ROM);
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;*)
              
        GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
      
      END_CASE;
      OutputState:=CURRENT_STATE;

  END_FUNCTION_BLOCK 

  //Attende l^avvio del sistema SIMOTION+SINAMICS, restituisce TRUE se il sistema è avviato
FUNCTION_BLOCK FB_WaitingStartUpSystem

    VAR_EXTERNAL
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
        Start_read					    : BOOL;
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        i_sys_ReadingSinamicsInUse      :BOOL;
        i_sys_RunOk                     : INT;
        (*{
        #ifdef ALIMENTATORE_ACTIVE_LINE
        }*)
        error_Alim						:BOOL;	//True se l^alimentatore è ok
        (*{
        #endif	//Fine ALIMENTATORE_ACTIVE_LINE
        }*)
        SystemStarted                   :BOOL;
    END_VAR

    VAR_EXTERNAL CONSTANT
        TRC_WAIT_SYSTEM_STARTUP                 :INT;
        TRC_WAIT_CU32_1                         :INT;
        TRC_WAIT_CU32_2                         :INT;
        TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION    :INT;
        TRC_WAIT_FOR_DRIVE_COMMUNICATION        :INT;
        TRC_EXECUTE_MOTOR_SETTING               :INT;

        NON_DEACTIVATABLE_AXIS                  :SINT;
        ACTIVE_DEACTIVATABLE_AXIS               :SINT;
        INACTIVE_DEACTIVATABLE_AXIS             :SINT;
        SETTING_OBJECT_OPERATION		        :UDINT;
        ADDRESS_POWER_SUPPLY                    :INT;
        ADDRESS_CU2                             :INT;
        (*{
        #ifdef PLC_S7_1500
        }*)
        ADDRESS_CU1                             :INT;
        (*{
        #endif //FINE NO PLC_S7_1500
        }*)
        OPERATION_OBJECT_STATE              :UDINT;
        NO_HARDWARE                        	:SINT;
        AXIS_ACTIVE                         :REAL;
        driveObjectIdConstant               : UINT;
        hardwareIdConstant                  :WORD;
    END_VAR

    VAR_INPUT
      AttesaSimotion  :BOOL;
      AttesaSinamics  :BOOL;
    END_VAR

    VAR 
        i                           :INT;
        dret                        :DINT;
        //Stato                       :StructRetGetStateOfTo;
        //myRetANY                    :EnumSetAndGetSafeValue;
        ValoreUDINT                 :UDINT;
        //ValoreENUM1                 :EnumAxisEncoderIdentification;
        //ValoreENUM2                 :EnumActiveInactive;
        //ValoreENUM3                 :EnumAxisDriverMode;
        Counter                     :UDINT;
        StatoAsse                   :REAL;
        NomeMaster                  :REF_TO TO_PositioningAxis;
        TempoAttesa					:INT;
        StatoAlimentatore		    :REAL;
        TensioneFrequenza		    :REAL;    
        StatoCU32_1,
        StatoCU32_2					:REAL;
        RestartEseguito			    :BOOL;     
          
          
        CURRENT_AXIS	            :INT;
        //thisCommand		            :commandIDType;
          
        AttesaStatoTO				:FB_AttesaStatoTO;
        LetturaSinamicsREAL		    :_READ_PARAMETER_SINGLE;
        RestartActivationAsse	    :FB_RestartActivationAxis;
        PosAx                       :REF_TO TO_PositioningAxis;
    END_VAR

      VAR_OUTPUT
          CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
          OutputState		:INT;
          ReturnValue		:BOOL;
      END_VAR
      CASE CURRENT_STATE OF
        GENERAL_STATE#STATE_INIT:
              ReturnValue:=FALSE;
              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
              //Selezione del tempo di attesa prima di emettere il messaggio in funzione se occorre
              //attendere o meno anche gli azionamenti
              IF AttesaSinamics=TRUE THEN
                  TempoAttesa:=600;
              ELSE
                  TempoAttesa:=100;
              END_IF;
              //thisCommand:=_getCommandID();
              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_MASTER_1;
              
        (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_MASTER_1:
              //Recupero del master in funzione dello slave indicato
              Stato:=_getstateofto(to_instance:=Axes[X1].masteraxis,
                                                       reqactdeactgetstatemode:=REQUEST_TRUE,
                                                       commandid:=thisCommand,
                                                       nextcommand:=IMMEDIATELY);
              IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                  IF AttesaSimotion=TRUE THEN
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITE_MASTER_1;
                  ELSE
                      IF Stato.commandIDState<>ACTIVE THEN
                          ReturnValue:=FALSE;
                          CURRENT_STATE:=GENERAL_STATE#STATE_END;
                      ELSE
                          (*{#ifdef DH4010}*)
                              (*thisCommand:=_getCommandID();
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_MASTER_2;
                          (*{#else}*)
                              (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_EMULATION;
                          (*{#endif}*)
                      (*END_IF;
                  END_IF;
              END_IF;*)
              
        (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITE_MASTER_1:
              IF (AttesaStatoTO.outputState<>GENERAL_STATE#STATE_END) THEN
                  AttesaStatoTO(Oggetto:=Axes[X1].masteraxis, Axis:=_MAXIS#MAXAXIS);
              ELSE
                  AttesaStatoTO(Oggetto:=Axes[X1].masteraxis, Axis:=_MAXIS#MAXAXIS);
                  (*{#ifdef DH4010}*)
                      (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_MASTER_2;
                  (*{#else}*)
                      (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_EMULATION;
                  (*{#endif}*)
              (*END_IF;
              
        (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_MASTER_2:
              (*{#ifdef DH4010}*)
                  //Recupero del master in funzione dello slave indicato
                  (*Stato:=_getstateofto(to_instance:=Axes[X2].masteraxis,
                                                           reqactdeactgetstatemode:=REQUEST_TRUE,
                                                           commandid:=thisCommand,
                                                           nextcommand:=IMMEDIATELY);
                  IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                      IF AttesaSimotion=TRUE THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAITING_START_SYSTEM_WAITING_MASTER_2;
                      ELSE
                          IF Stato.commandIDState<>ACTIVE THEN
                              ReturnValue:=FALSE;
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_EMULATION;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#endif};*)
              
        (*_FB_WaitingStartUpSystem#STATE_WAITING_START_SYSTEM_WAITING_MASTER_2:
              (*{#ifdef DH4010}*)
                  (*IF (AttesaStatoTO.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=Axes[X2].masteraxis, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=Axes[X2].masteraxis, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_EMULATION;
                  END_IF;*)
             (*{#endif};*)
      
        _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_EMULATION:
              //Attesa avvio di tutti gli assi slave
              reset_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);
              (*{#ifndef EMULAZIONE}*)
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_1_STATE;
              (*{#else}*)
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_FEEDER_ACTIVE_LINE;
              (*{#endif};*)
              
        _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_1_STATE:
              (*{#ifndef EMULAZIONE}*)
                  IF ADDRESS_CU1<>NO_HARDWARE AND SystemStarted=FALSE THEN
                      IF NOT LetturaSinamicsREAL.Done THEN
                          LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId:=TO_UINT(ADDRESS_CU1),
                                            parameterNumber:=TO_UINT(OPERATION_OBJECT_STATE),
                                            index:=uint#0,
                                            hardwareId:=hardwareIdConstant);
                      ELSE
                        LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId:=TO_UINT(ADDRESS_CU1),
                                            parameterNumber:=TO_UINT(OPERATION_OBJECT_STATE),
                                            index:=uint#0,
                                            hardwareId:=hardwareIdConstant);
                        IF LetturaSinamicsREAL.Done THEN
                          StatoCU32_1:=LetturaSinamicsREAL.Value_REAL;
                          LetturaSinamicsREAL.Start :=false;
                          IF StatoCU32_1=1 THEN
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_1_FIRMWARE;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_2_STATE;
                          END_IF;
                        END_IF;
                      END_IF;
                  ELSE
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_2_STATE;
                  END_IF;
              (*{#endif};*)	
              
        _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_1_FIRMWARE:
              (*{#ifndef EMULAZIONE}*)
                  IF NOT LetturaSinamicsREAL.Done THEN
                      LetturaSinamicsREAL(Start := Start_read,
                                        driveObjectId:=TO_UINT(ADDRESS_CU1),
                                        parameterNumber:=uint#18,
                                        index:=uint#0,
                                        hardwareId:= hardwareIdConstant);																	 
                  ELSE
                    LetturaSinamicsREAL(Start := Start_read,
                                        driveObjectId:=TO_UINT(ADDRESS_CU1),
                                        parameterNumber:=uint#18,
                                        index:=uint#0,
                                        hardwareId:= hardwareIdConstant);
                    IF LetturaSinamicsREAL.Done THEN	
                      StatoCU32_1:=LetturaSinamicsREAL.Value_REAL;
                      LetturaSinamicsREAL.Start :=false;
                      IF StatoCU32_1=0 THEN
                          Init_Error();
                          set_Trace(Code:=TRC_WAIT_CU32_1);
                      ELSE
                          reset_trace(Code:=TRC_WAIT_CU32_1);
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_2_STATE ;
                      END_IF;
                    END_IF;
                  END_IF;
              (*{#endif};*)
              
        _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_2_STATE:
              (*{#ifndef EMULAZIONE}*)
                  IF ADDRESS_CU2<>NO_HARDWARE AND SystemStarted=FALSE THEN
                      IF NOT LetturaSinamicsREAL.Done THEN
                          LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId:=TO_UINT(ADDRESS_CU2),
                                            parameterNumber:=TO_UINT(OPERATION_OBJECT_STATE),
                                            index:=uint#0);
                      ELSE
                            LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId:=TO_UINT(ADDRESS_CU2),
                                            parameterNumber:=TO_UINT(OPERATION_OBJECT_STATE),
                                            index:=uint#0);
                        IF LetturaSinamicsREAL.Done THEN                  
                          StatoCU32_2:=LetturaSinamicsREAL.Value_REAL;
                          LetturaSinamicsREAL.Start :=false;
                          IF StatoCU32_2=1 THEN
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_2_FIRMWARE;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_FEEDER_ACTIVE_LINE;
                          END_IF;
                        END_IF;
                      END_IF;
                  ELSE
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_FEEDER_ACTIVE_LINE;
                  END_IF;
              (*{#endif};*)	
              
        _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_CU32_2_FIRMWARE:
              (*{#ifndef EMULAZIONE}*)
                  IF NOT LetturaSinamicsREAL.Done THEN
                      LetturaSinamicsREAL(Start := Start_read,
                                        driveObjectId:=TO_UINT(ADDRESS_CU2),
                                        parameterNumber:=UINT#18,
                                        index:=UINT#0);																	 
                  ELSE
                        LetturaSinamicsREAL(Start := Start_read,
                                        driveObjectId:=TO_UINT(ADDRESS_CU2),
                                        parameterNumber:=UINT#18,
                                        index:=UINT#0);	
                    IF LetturaSinamicsREAL.Done THEN	
                      StatoCU32_2:=LetturaSinamicsREAL.Value_REAL;
                      LetturaSinamicsREAL.Start :=false;
                      IF StatoCU32_2=0 THEN
                          Init_Error();
                          set_Trace(Code:=TRC_WAIT_CU32_2);
                      ELSE
                          reset_trace(Code:=TRC_WAIT_CU32_2);
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_FEEDER_ACTIVE_LINE;
                      END_IF;
                    END_IF;
                  END_IF;
              (*{#endif};*)
              
        _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_FEEDER_ACTIVE_LINE:
              (*{#ifdef ALIMENTATORE_ACTIVE_LINE}*)
                  IF NOT LetturaSinamicsREAL.Done THEN
                      LetturaSinamicsREAL(Start := Start_read,
                                        driveObjectId:=TO_UINT(ADDRESS_POWER_SUPPLY),
                                        parameterNumber:=UINT#3410,
                                        index:=UINT#0);
                  ELSE		
                        LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId:=TO_UINT(ADDRESS_POWER_SUPPLY),
                                            parameterNumber:=UINT#3410,
                                            index:=UINT#0);
                    IF LetturaSinamicsREAL.Done THEN
                      TensioneFrequenza:=LetturaSinamicsREAL.Value_REAL;
                      LetturaSinamicsREAL.Start :=false;
                      IF AttesaSimotion=TRUE AND SystemStarted=FALSE AND TensioneFrequenza=0 THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_POWER_SUPPLY;
                      ELSE
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_AXES;
                      END_IF;
                    END_IF;
                  END_IF;
              (*{#else}*)
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_AXES;
              (*{#endif};*)
              
        _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_POWER_SUPPLY:
              (*{#ifdef ALIMENTATORE_ACTIVE_LINE}*)
                  IF NOT LetturaSinamicsREAL.Done  THEN
                      LetturaSinamicsREAL(Start := Start_read,
                                        driveObjectId:=TO_UINT(ADDRESS_POWER_SUPPLY),
                                        parameterNumber:=uint#2,
                                        index:=uint#0);
                  ELSE
                        LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId:=TO_UINT(ADDRESS_POWER_SUPPLY),
                                            parameterNumber:=uint#2,
                                            index:=uint#0);
                      StatoAlimentatore:=LetturaSinamicsREAL.Value_REAL;
                      IF (error_Alim=TRUE AND i_sys_RUNOK=1) OR (StatoAlimentatore<>42 AND StatoAlimentatore<>0 AND StatoAlimentatore<>45) THEN
                          set_trace(Code:=TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION);
                          init_error();
                      ELSE
                          reset_trace(Code:=TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION);
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_AXES;
                      END_IF;
                  END_IF;
              (*{#endif};*)
              
        _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_AXES:
              CURRENT_AXIS:=CURRENT_AXIS+1;
              IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
                  RestartEseguito:=FALSE;
                  IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE THEN
                      //dret:=_resetTechnologicalErrors();
                      //thisCommand:=_getCommandID();
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_START_POSAXIS;
                  END_IF;
              ELSE
                  //thisCommand:=_getCommandID();
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_OBJECTS_VECTORS;
              END_IF;
              
        (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_START_POSAXIS:
        Stato:=_getstateofto(to_instance:=Axes[CURRENT_AXIS].PosAx,
                             reqactdeactgetstatemode:=REQUEST_TRUE,
                             commandid:=thisCommand,
                             nextcommand:=IMMEDIATELY);
              IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                  IF AttesaSimotion=TRUE THEN
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_STATE_TO_POSAXIS;
                  ELSE
                      IF Stato.commandIdState<>ACTIVE THEN
                          CURRENT_STATE:=GENERAL_STATE#STATE_END;
                      ELSE
                          thisCommand:=_getCommandID();
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_TO_SYNCHRO;
                      END_IF;
                  END_IF;
              END_IF;*)
              
       (* _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_STATE_TO_POSAXIS:
              IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                  AttesaStatoTO(Oggetto:=Axes[CURRENT_AXIS].PosAx, Axis:=CURRENT_AXIS);
              ELSE
                  AttesaStatoTO(Oggetto:=Axes[CURRENT_AXIS].PosAx, Axis:=CURRENT_AXIS);
                  thisCommand:=_getCommandID();
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_TO_SYNCHRO;
              END_IF;*)
              
        (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_TO_SYNCHRO://TO.StatusDrive.CommunicaTIONok  

        Stato:=_getstateofto(to_instance:=Axes[CURRENT_AXIS].SincroAsse,
                             reqactdeactgetstatemode:=REQUEST_TRUE,
                             commandid:=thisCommand,
                             nextcommand:=IMMEDIATELY);
              IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                  IF AttesaSimotion=TRUE THEN
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAITING_START_SYSTEM_WAITING_STATE_TO_SYNCHRO;
                  ELSE
                      IF Axes[CURRENT_AXIS].sincroasse.activemaster<> NIL THEN
                          IF Stato.commandIdState<>ACTIVE THEN
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              IF Axes[i].sincroasse.activemaster<> NIL THEN
                                  thisCommand:=_getCommandID();
                                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_TO_CAMMA;
                              ELSE
                                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_READING_LOGADDRESSIN;
                              END_IF;
                          END_IF;
                      ELSE
                          IF Axes[i].sincroasse.activemaster<> NIL THEN
                              thisCommand:=_getCommandID();
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_TO_CAMMA;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_READING_LOGADDRESSIN;
                          END_IF;
                      END_IF;
                  END_IF;
              END_IF;*)
          
        (*_FB_WaitingStartUpSystem#STATE_WAITING_START_SYSTEM_WAITING_STATE_TO_SYNCHRO:
              IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                  AttesaStatoTO(Oggetto:=Axes[CURRENT_AXIS].SincroAsse, Axis:=CURRENT_AXIS);
              ELSE
                  AttesaStatoTO(Oggetto:=Axes[CURRENT_AXIS].SincroAsse, Axis:=CURRENT_AXIS);
                  thisCommand:=_getCommandID();
                  IF Axes[i].sincroasse.activemaster<> NIL THEN
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_TO_CAMMA;
                  ELSE
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_READING_LOGADDRESSIN;
                  END_IF;
              END_IF;*)
              
        (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_TO_CAMMA:
        Stato:=_getstateofto(to_instance:=Axes[CURRENT_AXIS].camma,
                             reqactdeactgetstatemode:=REQUEST_TRUE,
                             commandid:=thisCommand,
                             nextcommand:=IMMEDIATELY);
              IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                  IF AttesaSimotion=TRUE THEN
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_STATE_TO_CAMMA;
                  ELSE
                      IF Stato.commandIdState<>ACTIVE THEN
                          CURRENT_STATE:=GENERAL_STATE#STATE_END;
                      ELSE
                          thisCommand:=_getCommandID();
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_MASTER_AXIS;
                      END_IF;
                  END_IF;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_STATE_TO_CAMMA:
              IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                  AttesaStatoTO(Oggetto:=Axes[CURRENT_AXIS].camma, Axis:=CURRENT_AXIS);
              ELSE
                  AttesaStatoTO(Oggetto:=Axes[CURRENT_AXIS].camma, Axis:=CURRENT_AXIS);
                  thisCommand:=_getCommandID();
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_MASTER_AXIS;
              END_IF;*)
              
         (* _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_MASTER_AXIS:
        Stato:=_getstateofto(to_instance:=Axes[CURRENT_AXIS].masterName,
                             reqactdeactgetstatemode:=REQUEST_TRUE,
                             commandid:=thisCommand,
                             nextcommand:=IMMEDIATELY);
              IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                  IF AttesaSimotion=TRUE THEN
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_STATE_TO_MASTER;
                  ELSE
                      IF Stato.commandIdState<>ACTIVE THEN
                          CURRENT_STATE:=GENERAL_STATE#STATE_END;
                      ELSE
                          thisCommand:=_getCommandID();
                          IF Axes[i].posax.typeofaxis<>VIRTUAL_AXIS THEN
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_LOGADDRESSIN;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_WAIT_ACTION;
                          END_IF;
                      END_IF;
                  END_IF;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_STATE_TO_MASTER:
              IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                  AttesaStatoTO(Oggetto:=Axes[CURRENT_AXIS].masterName, Axis:=CURRENT_AXIS);
              ELSE
                  AttesaStatoTO(Oggetto:=Axes[CURRENT_AXIS].masterName, Axis:=CURRENT_AXIS);
                  IF Axes[i].posax.typeofaxis<>VIRTUAL_AXIS THEN
                      thisCommand:=_getCommandID();
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_LOGADDRESSIN;
                  ELSE
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAITING_WAIT_ACTION;
                  END_IF;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_LOGADDRESSIN:
          //Lettura sicura del paramentro del TO LogAddressIn
              myRetANY:=_getsafevalue(variable:=Axes[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn,
                                                              accessmode:=NO_CHANGE,
                                                              getvalue:=ValoreUDINT);
              TestSafeValue:=myRetAny;
              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_LOGADDRESSIN_RETANY_OK;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_LOGADDRESSIN_RETANY_OK:
              IF myRetANY<>OK THEN
                  IF NOT AttesaSimotion THEN
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_WAIT_SYSTEM_READ_NUMBEROFENCODERS;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_WAIT_SYSTEM_READ_NUMBEROFENCODERS:
              //Lettura sicura del paramentro del TO NumberOfEncoders
              myRetANY:=_getsafevalue(variable:=Axes[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.NumberOfEncoders.numberofencoders,
                                                              accessmode:=NO_CHANGE,
                                                              getvalue:=ValoreUDINT);
              TestSafeValue:=myRetANY;
              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFENCODERS_RETANY_OK;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFENCODERS_RETANY_OK:
              IF myRetANY<>OK THEN
                  IF NOT AttesaSimotion THEN
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFDATASETS;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFDATASETS:
              //Lettura sicura del paramentro del TO NumberOfDatasets
              myRetANY:=_getsafevalue(variable:=Axes[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.NumberOfDataSets.numberOfDataSets,
                                                              accessmode:=NO_CHANGE,
                                                              getvalue:=ValoreUDINT);
              TestSafeValue:=myRetANY;
              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFDATASETS_RETANY_OK;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFDATASETS_RETANY_OK:
              IF myRetANY<>OK THEN
                  IF NOT AttesaSimotion THEN
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_ENCODERIDENTIFICATION;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_ENCODERIDENTIFICATION:
              //Lettura sicura del paramentro del TO NumberOfDatasets
              myRetANY:=_getsafevalue(variable:=Axes[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.numberofencoders.encoder_1.encoderidentification,
                                                              accessmode:=NO_CHANGE,
                                                              getvalue:=ValoreENUM1);
              TestSafeValue:=myRetANY;
              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_ENCODERIDENTIFICATION_RETANY_OK;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_ENCODERIDENTIFICATION_RETANY_OK:
              IF myRetANY<>OK THEN
                  IF NOT AttesaSimotion THEN
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_SETPOINTDRIVERINFO;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_SETPOINTDRIVERINFO:
              //Lettura sicura del paramentro del TO NumberOfDatasets
              myRetANY:=_getsafevalue(variable:=Axes[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.setpointdriverinfo.mode,
                                                              accessmode:=NO_CHANGE,
                                                              getvalue:=ValoreENUM3);
              TestSafeValue:=myRetANY;
              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_SETPOINTDRIVERINFO_RETANY_OK;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_SETPOINTDRIVERINFO_RETANY_OK:
              IF myRetANY<>OK THEN
                  IF NOT AttesaSimotion THEN
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_CYCLICINTERFACE;
              END_IF;*)
      
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_CYCLICINTERFACE:
              //Lettura sicura del paramentro del TO cyclicInterface                             
              myRetANY:=_getsafevalue(variable:=Axes[CURRENT_AXIS].PosAx.actormonitoring.cyclicinterface,
                                                              accessmode:=NO_CHANGE,
                                                              getvalue:=ValoreENUM2);
              TestSafeValue:=myRetANY;
              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_CYCLICINTERFACE_RETANY_OK;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_READ_CYCLICINTERFACE_RETANY_OK:
              IF myRetANY<>OK THEN
                  IF NOT AttesaSimotion THEN
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                  END_IF;
              ELSE
                  IF AttesaSimotion=TRUE THEN
                      set_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION, Axis:=CURRENT_AXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_POSAX_NOT_NULL;
                  ELSE
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_AXES;
                  END_IF;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_POSAX_NOT_NULL:
              IF Axes[CURRENT_AXIS].PosAx<> NIL THEN
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_END_RESTART_TO;
              END_IF;*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_END_RESTART_TO:
              IF Axes[i].PosAx.restartactivation<>activate_restart THEN
                  Counter:=UINT#0;
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_AXIS_VALID;
              END_IF;*)
              
          _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_AXIS_VALID:
              PosAx:=AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);
              IF PosAx^.sensor[1].Existent =true and

                PosAx=AsPositioningAxisRef(Axes[i].posax) AND
                PosAx^.VirtualAxis.Mode <> UDINT#0 AND
                pm[i].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS THEN
                  //Attesa che gli assi realmente presenti assumano lo stato VALID
                  init_error();
                  Counter:=MIN(Counter+UDINT#1,UDINT#4294967290);
                  IF Counter> to_udint(TempoAttesa) THEN
                      IF pm[CURRENT_AXIS].ma_asse_installato>NON_DEACTIVATABLE_AXIS THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_PARAMETER_2_105;
                      ELSE
                          //Tempo di avvio terminato si passa a visualizzare solo la trace del singolo asse
                          reset_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);
                          set_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION, Axis:=CURRENT_AXIS);
                          IF RestartEseguito=FALSE THEN
                              RestartEseguito:=TRUE;
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_RESTART_ACTIVE_AXIS;
                          END_IF;
                      END_IF;
                  ELSE
                      //Non è ancora scaduto il tempo di attesa, si visualizza il messaggio ATTESA AVVIO SISTEMA
                      //con l^indicazione dell^asse 
                      reset_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION);
                      set_trace(Code:=TRC_WAIT_SYSTEM_STARTUP, Axis:=CURRENT_AXIS);
                  END_IF;
              ELSE
                  reset_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION);
                  reset_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_CHECK_AXES;
              END_IF;
              
          _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_RESTART_ACTIVE_AXIS:
              IF (RestartActivationAsse.outputState<>GENERAL_STATE#STATE_END) THEN
                  RestartActivationAsse(NumAsse:=CURRENT_AXIS, Attesa:=FALSE);
              ELSE
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_AXIS_VALID;
              END_IF;
              
          _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_PARAMETER_2_105:
              //L^asse è potenzialmente disattivabile. Si verifica lo stato in Simotion
              //sia congruente con il parametro 105 dell^azionamento
              IF i_sys_ReadingSinamicsInUse =FALSE THEN
                  (*Address:=UDINT_TO_DINT(Axes[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
                  CURRENT_AXIS:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_READING_PARAM_105;
              END_IF;
              
          _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_READING_PARAM_105:
              IF NOT LetturaSinamicsREAL.Done THEN
                  LetturaSinamicsREAL(Start := Start_read,
                                    driveObjectId:=driveObjectIdConstant ,
                                    parameterNumber:=TO_UINT(SETTING_OBJECT_OPERATION),
                                    index:=uint#0,
                                    hardwareId:= hardwareIdConstant);
              ELSE
                LetturaSinamicsREAL(Start := Start_read,
                                    driveObjectId:=driveObjectIdConstant ,
                                    parameterNumber:=TO_UINT(SETTING_OBJECT_OPERATION),
                                    index:=uint#0,
                                    hardwareId:= hardwareIdConstant);
                  StatoAsse:=LetturaSinamicsREAL.Value_REAL;
                  IF (StatoAsse=AXIS_ACTIVE AND pm[CURRENT_AXIS].ma_asse_abilitato=FALSE) OR
                                                      (StatoAsse<>AXIS_ACTIVE AND pm[CURRENT_AXIS].ma_asse_abilitato=TRUE) THEN
                      //Incongruenza fra SIMOTION ed azionamenti si richiede un comando 242
                      reset_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);
                      reset_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION);
                      set_trace(Code:=TRC_EXECUTE_MOTOR_SETTING, Axis:=CURRENT_AXIS);
                  ELSE
                      //Nessuna incongruenza ma non si riesce a collegare l^encoder dell^azionamento
                      reset_trace(Code:=TRC_WAIT_SYSTEM_STARTUP);
                      set_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION, Axis:=CURRENT_AXIS);		
                  END_IF;
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_AXIS_VALID;
              END_IF;
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_OBJECTS_VECTORS:
              //Attesa avvio oggetti vettoriali
              (*{#ifndef ORBITALE}*)
                  (*Stato:=_getstateofto(to_instance:=X1_U2,
                                                           reqactdeactgetstatemode:=REQUEST_TRUE,
                                                           commandid:=thisCommand,
                                                           nextcommand:=IMMEDIATELY);
                  IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                      IF AttesaSimotion=TRUE THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_X1_U2;
                      ELSE
                          IF Stato.commandidstate<>ACTIVE THEN
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_X1;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_X1;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_X1_U2:
              (*{#ifndef ORBITALE}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=X1_U2, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=X1_U2, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_X1;
                  END_IF;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_X1:
              (*{#ifdef INTERPOLA_Y1_X1}*)
                  (*Stato:=_getstateofto(to_instance:=Y1_X1,
                                                           reqactdeactgetstatemode:=REQUEST_TRUE,
                                                           commandid:=thisCommand,
                                                           nextcommand:=IMMEDIATELY);
                  IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                      IF AttesaSimotion=TRUE THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_X1 ;
                      ELSE
                          IF Stato.commandidstate<>ACTIVE THEN
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_U4;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_U4;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_X1 :
              (*{#ifdef INTERPOLA_Y1_X1}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=Y1_X1, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=Y1_X1, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_WAIT_STATE_Y1_U4;
                  END_IF;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_WAIT_STATE_Y1_U4:
              (*{#ifdef SLITTA_MOBILE_DX}*)
                  (*Stato:=_getstateofto(to_instance:=Y1_U4,
                                                           reqactdeactgetstatemode:=REQUEST_TRUE,
                                                           commandid:=thisCommand,
                                                           nextcommand:=IMMEDIATELY);
                  IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                      IF AttesaSimotion=TRUE THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_U4;
                      ELSE
                          IF Stato.commandidstate<>ACTIVE THEN
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U5;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)	
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U5;
              (*{#endif};*) //Fine SLITTA_MOBILE_DX
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_U4:
              (*{#ifdef SLITTA_MOBILE_DX}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=Y1_U4, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=Y1_U4, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U5;
                  END_IF;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U5:
              (*{#ifdef SLITTA_MOBILE_SX}*)
                  (*Stato:=_getstateofto(to_instance:=Y1_U5,
                                                           reqactdeactgetstatemode:=REQUEST_TRUE,
                                                           commandid:=thisCommand,
                                                           nextcommand:=IMMEDIATELY);
                  IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                      IF AttesaSimotion=TRUE THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_U5;
                      ELSE
                          IF Stato.commandidstate<>ACTIVE THEN
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_W3_W4_Q1;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_W3_W4_Q1;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_W3_W4_Q1:
              (*{#ifndef ORBITALE}
                  {#ifndef ETURN}
                      {#ifndef ELECTMRL}*)
                          (*Stato:=_getstateofto(to_instance:=W3_W4_Q1,
                                                                   reqactdeactgetstatemode:=REQUEST_TRUE,
                                                                   commandid:=thisCommand,
                                                                   nextcommand:=IMMEDIATELY);
                          IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                              IF AttesaSimotion=TRUE THEN
                                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_W3_W4_Q1;
                              ELSE
                                  IF Stato.commandidstate<>ACTIVE THEN
                                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                                  ELSE
                                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U4_Y2;
                                  END_IF;
                              END_IF;
                          END_IF;
                      (*{#else}*) //fine no electmrl
                          (*Stato:=_getstateofto(to_instance:=W3_W4_W6_Q1,
                                                                   reqactdeactgetstatemode:=REQUEST_TRUE,
                                                                   commandid:=thisCommand,
                                                                   nextcommand:=IMMEDIATELY);
                          IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                              IF AttesaSimotion=TRUE THEN
                                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_W3_W4_W6_Q1;
                              ELSE
                                  IF Stato.commandidstate<>ACTIVE THEN
                                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                                  ELSE
                                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U4_Y2;
                                  END_IF;
                              END_IF;
                          END_IF;
                      (*{#endif} //fine electmrl
                  {#else}*) // fine no eturn
                      (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U4_Y2;
                  (*{#endif} // fine eturn
              {#else}*) //fine no orbitale
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U4_Y2;
              (*{#endif}*) //fine orbitale
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_W3_W4_Q1:
              (*{#ifndef ORBITALE}
                  {#ifndef ETURN}
                      {#ifndef ELECTMRL}*)
                          (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                              AttesaStatoTO(Oggetto:=W3_W4_Q1, Axis:=_MAXIS#MAXAXIS);
                          ELSE
                              AttesaStatoTO(Oggetto:=W3_W4_Q1, Axis:=_MAXIS#MAXAXIS);
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U4_Y2;
                          END_IF;
                      (*{#endif};
                  {#endif};
              {#endif};*)
          
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_W3_W4_W6_Q1:
              (*{#ifdef ELECTMRL}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=W3_W4_W6_Q1, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=W3_W4_W6_Q1, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U4_Y2;
                  END_IF;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U4_Y2:
              (*{#ifdef MATRICE_ELETTRICA}*)
                  (*IF TestAsseY2()=TRUE THEN
                      Stato:=_getstateofto(to_instance:=Y1_U4_Y2,
                                                               reqactdeactgetstatemode:=REQUEST_TRUE,
                                                               commandid:=thisCommand,
                                                               nextcommand:=IMMEDIATELY);
                      IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                          IF AttesaSimotion=TRUE THEN
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_U4_Y2;
                          ELSE
                              IF Stato.commandidstate<>ACTIVE THEN
                                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
                              ELSE
                                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_Y2;
                              END_IF;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_V4 ;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_U4_Y2:
              (*{#ifdef MATRICE_ELETTRICA}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=Y1_U4_Y2, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=Y1_U4_Y2, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_Y2;
                  END_IF;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_Y2:
              (*{#ifdef MATRICE_ELETTRICA}*)
                  (*Stato:=_getstateofto(to_instance:=Y1_Y2,
                                                           reqactdeactgetstatemode:=REQUEST_TRUE,
                                                           commandid:=thisCommand,
                                                           nextcommand:=IMMEDIATELY);
                  IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                      IF AttesaSimotion=TRUE THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_Y2 ;
                      ELSE
                          IF Stato.commandidstate<>ACTIVE THEN
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_Y2;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_V4 ;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Y1_Y2 :
              (*{#ifdef MATRICE_ELETTRICA}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=Y1_Y2, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=Y1_Y2, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Y1_Y2;
                  END_IF;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_V4 :
              (*{#ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO}*)
                  (*IF TestAsseY2()=TRUE THEN
                      Stato:=_getstateofto(to_instance:=Q1_V1_V4,
                                                               reqactdeactgetstatemode:=REQUEST_TRUE,
                                                               commandid:=thisCommand,
                                                               nextcommand:=IMMEDIATELY);
                      IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                          IF AttesaSimotion=TRUE THEN
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Q1_V1_V4;
                          ELSE
                              IF Stato.commandidstate<>ACTIVE THEN
                                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
                              ELSE
                                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1;
                              END_IF;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_Z1;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Q1_V1_V4:
              (*{#ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=Q1_V1_V4, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=Q1_V1_V4, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1;
                  END_IF;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1:
              (*{#ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO}*)
                  (*Stato:=_getstateofto(to_instance:=Q1_V1,
                                                           reqactdeactgetstatemode:=REQUEST_TRUE,
                                                           commandid:=thisCommand,
                                                           nextcommand:=IMMEDIATELY);
                  IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                      IF AttesaSimotion=TRUE THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Q1_V1;
                      ELSE
                          IF Stato.commandidstate<>ACTIVE THEN
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_Z1;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_Z1;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Q1_V1:
              (*{#ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=Q1_V1, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=Q1_V1, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_Z1;
                  END_IF;
              (*{#endif};*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_Z1:
              (*{#ifdef INTERPOLA_Q1_V1_Z1}*)
                  (*Stato:=_getstateofto(to_instance:=Q1_V1_Z1,
                                                           reqactdeactgetstatemode:=REQUEST_TRUE,
                                                           commandid:=thisCommand,
                                                           nextcommand:=IMMEDIATELY);
                  IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                      IF AttesaSimotion=TRUE THEN
                          CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Q1_V1_Z1;
                      ELSE
                          IF Stato.commandidstate<>ACTIVE THEN
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          ELSE
                              CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_SYNAMICS;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#else}*)
                  (*CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_SYNAMICS;
              (*{#endif}*)
              
          (*_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_Q1_V1_Z1:
              (*{#ifdef INTERPOLA_Q1_V1_Z1}*)
                  (*IF (AttesaStatoTo.outputState<>GENERAL_STATE#STATE_END) THEN
                      AttesaStatoTO(Oggetto:=Q1_V1_Z1, Axis:=_MAXIS#MAXAXIS);
                  ELSE
                      AttesaStatoTO(Oggetto:=Q1_V1_Z1, Axis:=_MAXIS#MAXAXIS);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_SYNAMICS;
                  END_IF;
              (*{#endif};*)
              
          _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_SYNAMICS:
              reset_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION);
              IF AttesaSinamics=TRUE THEN
                  CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                  CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_LOOP_AXIS_ACTIVE;
              ELSE
                  //Qui si arriva se è stato verificato che il sistema richiesto è avviato, altrimenti sarebbe bloccato
                  //in qualche while di attesa
                  ReturnValue:=TRUE;
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
          _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_LOOP_AXIS_ACTIVE:
              CURRENT_AXIS:=CURRENT_AXIS+1;
              IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
                  IF pm[CURRENT_AXIS].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS AND 
                       VerificationSimulationAxes()=FALSE AND 
                       pm[CURRENT_AXIS].ma_Asse_Abilitato=TRUE AND					
                       TestAxisReady(Axis:=CURRENT_AXIS)=FALSE AND 
                       Axes[CURRENT_AXIS].IsSiemensAxis THEN
                      set_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION, Axis:=CURRENT_AXIS);
                      (*Address:=UDINT_TO_DINT(Axes[CURRENT_AXIS].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_ACTION_ACTIVE;
                  END_IF;
              ELSE	
                  //Qui si arriva se è stato verificato che il sistema richiesto è avviato, altrimenti sarebbe bloccato
                  //in qualche while di attesa
                  ReturnValue:=TRUE;
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
          _FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_WAIT_ACTION_ACTIVE:
              IF NOT LetturaSinamicsREAL.Done THEN
                  LetturaSinamicsREAL(Start := Start_read, driveObjectId:=driveObjectIdConstant , parameterNumber:=uint#146, index:=uint#0, hardwareId:= hardwareIdConstant);
              ELSE
                  LetturaSinamicsREAL(Start := Start_read, driveObjectId:=driveObjectIdConstant , parameterNumber:=uint#146, index:=uint#0, hardwareId:= hardwareIdConstant);
                  IF LetturaSinamicsREAL.Done THEN
                      reset_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION);
                      CURRENT_STATE:=_FB_WaitingStartUpSystem#STATE_WAIT_START_SYSTEM_LOOP_AXIS_ACTIVE;
                  END_IF;
              END_IF;
              
          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
      END_CASE;
      OutputState:=CURRENT_STATE;
      //dret:=_waitTime(T#50ms);
  END_FUNCTION_BLOCK

  FUNCTION_BLOCK FB_AttesaStatoTO

    VAR_EXTERNAL CONSTANT
        TRC_WAIT_FOR_DRIVE_COMMUNICATION    :INT;
    END_VAR

    VAR_INPUT
      //Oggetto : AnyObject;
      Axis    :INT;
    END_VAR
    VAR
      //Stato   :StructRetGetStateOfTo;
      dret    :DINT;
          
          CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
          //thisCommand		:CommandIDType;
          //requestMode		:EnumReqActDeactGetStateMode;
    END_VAR
      VAR_OUTPUT
          OutputState	:INT;
      END_VAR
      CASE CURRENT_STATE OF
        GENERAL_STATE#STATE_INIT:
              (*IF Stato.commandIdState<>ACTIVE THEN*)
                  IF Axis>_MAXIS#MAXAXIS-1 THEN
                      set_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION);
                  ELSE
                      set_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION, Axis:=Axis);
                  END_IF;
                  //requestMode:=REQUEST_TRUE;
                  //thisCommand:=_getCommandID();
                  CURRENT_STATE:=_FB_AttendedStatusTO#STATE_WAITING_TO_GET_STATE;
              (*ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;*)
              
              (*_FB_AttendedStatusTO#STATE_WAITING_TO_GET_STATE:
              Stato:=_getStateOfTo(to_instance:=Oggetto,
                                                       reqactdeactgetstatemode:=requestMode,
                                                       commandid:=thisCommand,
                                                       nextcommand:=IMMEDIATELY);
              IF NOT (Stato.functionResult=7001 OR Stato.functionResult=7002) THEN
                  reset_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION);
                  CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
              ELSE
                  requestMode:=REQUEST_FALSE;
              END_IF;*)
              
          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
              
      END_CASE;
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK

  FUNCTION_BLOCK FB_RestartActivationAxis

    VAR_EXTERNAL
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax; 
    END_VAR

    VAR_INPUT
          NumAsse	:INT;
          Attesa	:BOOL;
    END_VAR

    VAR
          CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
          PosAxes       : REF_TO TO_PositioningAxis;

/********************
 * PLC_OPEN_COMMAND *
 ********************/
         _restartactivation         :MC_RESET;
        
    END_VAR

      VAR_OUTPUT
          OutputState		:INT;
      END_VAR
      CASE CURRENT_STATE OF
        GENERAL_STATE#STATE_INIT:
        PosAxes:=AsPositioningAxisRef(Axes[NumAsse].posax);
              IF posaxes^.StatusWord.%X25 THEN
                  //Si evita di effettuare il restart dell^asse non attivi, perchè se sono
                  //in simulazione verrebbro ripristinati allo stato normale
                  _restartactivation.Execute:=TRUE;
                  IF Attesa THEN
                      CURRENT_STATE:=_FB_RestartActivationAxis#STATE_RESTART_AXIS_WAITING;
                  ELSE
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                  END_IF;
              ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
              _FB_RestartActivationAxis#STATE_RESTART_AXIS_WAITING:
              IF NOT _restartactivation.Done THEN //MC_Reset MC_rest.Done
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
      END_CASE;
      OutputState:=CURRENT_STATE;

      /*********************
 * PLC_OPEN_COMMANDS *
 *********************/
		_restartactivation (Axis:=PosAxes^);
  END_FUNCTION_BLOCK

  FUNCTION_BLOCK FB_CopyRomSinamic

    VAR_EXTERNAL
        Start_read                          : BOOL;
        i_sys_ReadingSinamicsInUse          :BOOL;
    END_VAR

    VAR_EXTERNAL CONSTANT
        TRC_COPY_RAM_ROM                    :INT; // Waiting for copy from RAM to ROM 
        ADDRESS_CU2                         :INT;
        NO_HARDWARE                        	:SINT;
        driveObjectIdConstant               : UINT;
        hardwareIdConstant                  :WORD;
        OPERATION_OBJECT_STATE              :UDINT;
        PARAMETER_COPY_RAM_ROM				:UDINT;
                (*{
                #ifdef PLC_S7_1500
                }*)
                ADDRESS_CU1                 :INT;
                (*{
                #endif //FINE NO PLC_S7_1500
                }*)
    END_VAR

    VAR_INPUT
      CosaSalvare				:STRING;  
    END_VAR
    VAR
          RetDint						:DINT;
          //Ritorno						:ret_write_par;
          dret							:DINT;
          StatoCX32_1,
          StatoCX32_2				    :REAL:=0;
          
          CURRENT_STATE					:INT:=GENERAL_STATE#STATE_INIT;
          ScritturaDatiSinamics	        :_WRITE_PARAMETER_SINGL;
          LetturaSinamicsREAL		    :_READ_PARAMETER_SINGLE;
    END_VAR
      VAR_OUTPUT
          OutputState				:INT;
      END_VAR
      
       CASE CURRENT_STATE OF
        GENERAL_STATE#STATE_INIT:
              IF VerificationSimulationAxes()=FALSE THEN 
                  Init_trace();
                  set_trace(Code:=TRC_COPY_RAM_ROM);	
                  IF CosaSalvare='ALL' OR CosaSalvare='CONTROL_UNIT' THEN
                      CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_CU;
                  ELSE	
                      CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_READ_CX1_STATE;
                  END_IF;
              ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_CU:
                IF NOT ScritturaDatiSinamics.Done THEN
                    set_trace(Code:=TRC_COPY_RAM_ROM);
                    ScritturaDatiSinamics(Start := Start_read,
                                        driveObjectId :=driveObjectIdConstant,
                                        hardwareId := hardwareIdConstant,
                                        parameterNumber:=TO_UINT(PARAMETER_COPY_RAM_ROM),
                                        value:=1);				
                ELSIF ScritturaDatiSinamics.Done THEN
                    ScritturaDatiSinamics(Start := Start_read,
                                            driveObjectId :=driveObjectIdConstant,
                                            hardwareId := hardwareIdConstant,
                                            parameterNumber:=TO_UINT(PARAMETER_COPY_RAM_ROM),
                                            value:=1);
                CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_READ_CX1_STATE;
                ELSIF ScritturaDatiSinamics.error THEN
                    ScritturaDatiSinamics.Start :=false;
                END_IF;	
              
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_READ_CX1_STATE:
                IF ADDRESS_CU1<>NO_HARDWARE AND (CosaSalvare='ALL' OR CosaSalvare='CX1') THEN
                    //Lettura stato abilitato/disabilitato
                    set_trace(Code:=TRC_COPY_RAM_ROM);
                    IF NOT LetturaSinamicsREAL.Done THEN
                        LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId :=TO_UINT (ADDRESS_CU1),
                                            hardwareId          := hardwareIdConstant,
                                            parameterNumber:=TO_uint(OPERATION_OBJECT_STATE),
                                            index:=uint#0);					
                    ELSIF LetturaSinamicsREAL.Done THEN
                        StatoCX32_1:=LetturaSinamicsREAL.Value_REAL;
                        LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId := driveObjectIdConstant,
                                            hardwareId          := hardwareIdConstant,
                                            parameterNumber:=TO_uint(OPERATION_OBJECT_STATE),
                                            index:=uint#0); 
                        IF StatoCX32_1=1 THEN
                            CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WRITE_CX2_PARAM;
                        ELSE
                            CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_READ_CX2_STATE;
                        END_IF;
                    END_IF;
                ELSE
                    CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_READ_CX2_STATE;
                END_IF;  
              
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WRITE_CX1_PARAM:
                IF NOT ScritturaDatiSinamics.Done THEN
                    set_trace(Code:=TRC_COPY_RAM_ROM);
                    ScritturaDatiSinamics(Start := Start_read,
                                        driveObjectId :=TO_UINT(ADDRESS_CU1),
                                        hardwareId := hardwareIdConstant,
                                        parameterNumber:=TO_UINT(PARAMETER_COPY_RAM_ROM),
                                        value:=1);				
                ELSIF ScritturaDatiSinamics.Done THEN
                    ScritturaDatiSinamics(Start := Start_read,
                                            driveObjectId :=TO_UINT(ADDRESS_CU1),
                                            hardwareId := hardwareIdConstant,
                                            parameterNumber:=TO_UINT(PARAMETER_COPY_RAM_ROM),
                                            value:=1);
                CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_READ_CX2_STATE;
                ELSIF ScritturaDatiSinamics.error THEN
                    ScritturaDatiSinamics.Start :=false;
                END_IF;	

        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_READ_CX2_STATE:
              IF ADDRESS_CU2<>NO_HARDWARE AND (CosaSalvare='ALL' OR CosaSalvare='CX2') THEN
                  //Lettura stato abilitato/disabilitato
                  set_trace(Code:=TRC_COPY_RAM_ROM);
                    IF NOT LetturaSinamicsREAL.Done THEN
                        LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId := driveObjectIdConstant,
                                            hardwareId          := hardwareIdConstant,
                                            parameterNumber:=TO_uint(OPERATION_OBJECT_STATE),
                                            index:=TO_uint(OPERATION_OBJECT_STATE));					
                    ELSIF LetturaSinamicsREAL.Done THEN
                        StatoCX32_1:=LetturaSinamicsREAL.Value_REAL;
                        LetturaSinamicsREAL(Start := Start_read,
                                            driveObjectId := driveObjectIdConstant,
                                            hardwareId          := hardwareIdConstant,
                                            parameterNumber:=uint#463,
                                            index:=uint#0); 
                        IF StatoCX32_1=1 THEN
                            CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WRITE_CX2_PARAM;
                        ELSE
                            CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CU_AVAILABLE;
                        END_IF;  
                    ELSIF LetturaSinamicsREAL.error THEN
                        LetturaSinamicsREAL.Start :=false;
                    END_IF;
              ELSE
                  CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CU_AVAILABLE;
              END_IF;
              
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CU_AVAILABLE:
              //Attesa termine copia RAM=>ROM
              IF CosaSalvare='ALL' OR CosaSalvare='CONTROL_UNIT' THEN
                  IF i_sys_ReadingSinamicsInUse=FALSE THEN
                      CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CU_COPY_COMPLETED;
                  ELSE
                      set_trace(Code:=TRC_COPY_RAM_ROM);
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX1_AVAILABLE;
              END_IF;
              
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CU_COPY_COMPLETED:

                IF NOT LetturaSinamicsREAL.Done THEN
                    LetturaSinamicsREAL	(Start := Start_read,
                                        driveObjectId      := driveObjectIdConstant,
                                        hardwareId          := hardwareIdConstant,
                                        parameterNumber     :=TO_UINT (PARAMETER_COPY_RAM_ROM),
                                        index              := uint#0);
                ELSE 
                    IF LetturaSinamicsREAL.Done AND LetturaSinamicsREAL.Value_REAL=1 THEN
                    LetturaSinamicsREAL.Start :=false;
                    set_trace(Code:=TRC_COPY_RAM_ROM);
                    ELSE
                        CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX1_AVAILABLE;
                    END_IF;
                LetturaSinamicsREAL	(Start := Start_read,
                                    driveObjectId      := driveObjectIdConstant,
                                    hardwareId          := hardwareIdConstant,
                                    parameterNumber     :=TO_UINT (PARAMETER_COPY_RAM_ROM),
                                    index              := uint#0);
                END_IF;
          
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX1_AVAILABLE:
              IF CosaSalvare='ALL' OR CosaSalvare='CX1' THEN
                  IF i_sys_ReadingSinamicsInUse=FALSE THEN
                      CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX1_COPY_COMPLETED;
                  ELSE
                      set_trace(Code:=TRC_COPY_RAM_ROM);
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX2_AVAILABLE;
              END_IF;
              
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX1_COPY_COMPLETED:
                IF NOT LetturaSinamicsREAL.Done THEN
                    LetturaSinamicsREAL	(Start := Start_read,
                                        driveObjectId      := TO_UINT(ADDRESS_CU1),
                                        hardwareId          := hardwareIdConstant,
                                        parameterNumber     := TO_UINT (PARAMETER_COPY_RAM_ROM),
                                        index              := uint#0);
                ELSE
                    IF LetturaSinamicsREAL.Done AND LetturaSinamicsREAL.Value_REAL=1 THEN
                    LetturaSinamicsREAL.Start :=false;
                    set_trace(Code:=TRC_COPY_RAM_ROM);
                    ELSE
                    CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX2_AVAILABLE;
                    END_IF;
                LetturaSinamicsREAL	(Start := Start_read,
                                    driveObjectId      := TO_UINT(ADDRESS_CU1),
                                    hardwareId          := hardwareIdConstant,
                                    parameterNumber     := TO_UINT (PARAMETER_COPY_RAM_ROM),
                                    index              := uint#0);
                END_IF;
              
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX2_AVAILABLE:
              IF CosaSalvare='ALL' OR CosaSalvare='CX2' THEN
                  IF i_sys_ReadingSinamicsInUse=FALSE THEN
                      CURRENT_STATE:=_FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX2_COPY_COMPLETED;
                  ELSE
                      set_trace(Code:=TRC_COPY_RAM_ROM);
                  END_IF;
              ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
        _FB_CopyRomSinamic#STATE_COPY_ROM_SINAMICS_WAIT_CX2_COPY_COMPLETED:
                IF NOT LetturaSinamicsREAL.Done THEN
                    LetturaSinamicsREAL	(Start := Start_read,
                                        driveObjectId      := TO_UINT(ADDRESS_CU2),
                                        hardwareId          := hardwareIdConstant,
                                        parameterNumber     := TO_UINT (PARAMETER_COPY_RAM_ROM),
                                        index              := uint#0);
                ELSE
                    IF LetturaSinamicsREAL.Done AND LetturaSinamicsREAL.Value_REAL=1 THEN
                    LetturaSinamicsREAL.Start :=false;
                    set_trace(Code:=TRC_COPY_RAM_ROM);
                    ELSE
                    CURRENT_STATE:=GENERAL_STATE#STATE_END;
                    END_IF;
                LetturaSinamicsREAL	(Start := Start_read,
                                    driveObjectId      := TO_UINT(ADDRESS_CU2),
                                    hardwareId          := hardwareIdConstant,
                                    parameterNumber     := TO_UINT (PARAMETER_COPY_RAM_ROM),
                                    index              := uint#0);
                END_IF;
              
          GENERAL_STATE#STATE_END:
              reset_trace(Code:=TRC_COPY_RAM_ROM);
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
              
      END_CASE;
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK

  (*{
#ifdef ORBITALE
}*)
FUNCTION_BLOCK ResettingXxOrbita

    VAR_EXTERNAL  
      Axes                            		:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
      pm                              		: ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
      AxisInLimitationCouple		          :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;
      TubeIntestate                         :SINT;
      ThreadIntext          	            :BOOL;
    END_VAR
  
    VAR_EXTERNAL CONSTANT
      X1                                : INT;    //Cart X
      X6                                :INT;
      PIPE_NO_INTESTATE                  :SINT;
    END_VAR
    
    VAR
        PosAx : REF_TO TO_PositioningAxis;
         Asse    :INT;
		CURRENT_STATE		:INT:=GENERAL_STATE#STATE_INIT;
		Homing  :Homing ;	
    END_VAR

    VAR_OUTPUT
		OutputState					:INT;
	END_VAR

    (*{
    #ifdef DH4010
    }*)
      Asse:=X6;
    (*{
    #else  //Fine DH4010
    }*)
      Asse:=X1;
   (* {
    #endif  //Fine NO DH4010
    }*)
    PosAx := AsPositioningAxisRef(Axes[Asse].PosAx);
    IF PosAx^.actualPosition<pm[Asse].MA_POS_LIMIT_MINUS OR
        PosAx^.actualPosition>pm[Asse].MA_POS_LIMIT_PLUS THEN
      //Nelle orbitali, in caso l^asse X1 sia fuori dai limite software si esegue un index a 0
      TubeIntestate:=PIPE_NO_INTESTATE;
      ThreadIntext:=FALSE;
      Homing (Asse:=Asse, Quota:=0, Modo:=EnumHomingMode#SET_OFFSET_OF_ABSOLUTE_ENCODER_BY_POSITION);            
    END_IF;
  END_FUNCTION_BLOCK
(*{
#endif  //Fine ORBITALE
}*)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Esegue l^impostazione dei flag che eseguono l^inversione del conteggio e del riferimento degli assi
//Sono da armonizzare le seguenti funzionalità:
//1) inversione direzione programmabile per ciascun asse mediante il parametro M32100 (-1, 0, +1)
//2) senso di curvatura DX<=>SX non in processo
FUNCTION_BLOCK FB_InversioneSensoAsse
    VAR_INPUT 
      Asse  :INT;
      Senso	:INT;
    END_VAR
    VAR
      Dret  :DINT;
          //CURRENT_STATE					:INT:=STATE_INIT;
         // FB_DisableAxes						:FB_FB_DisableAxes;
          //RestartActivationAsse	:FB_RestartActivationAsse;
    END_VAR
      VAR_OUTPUT
          OutputState		:INT;
      END_VAR
      ;
     (* CASE CURRENT_STATE OF
          STATE_INIT:
            IF pm[Asse].ma_Asse_Abilitato=TRUE THEN
                  CURRENT_STATE:=STATE_DISABLEASSI_DISABLE;
              ELSE
                  CURRENT_STATE:=STATE_END;
              END_IF;
              
          STATE_DISABLEASSI_DISABLE:
              IF (FB_DisableAxes.outputState<>STATE_END) THEN
                  FB_DisableAxes(Asse:=Asse, Regolatore:=INACTIVE);  
              ELSE
                  FB_DisableAxes(Asse:=Asse, Regolatore:=INACTIVE);  
                  CASE pm[Asse].ma_ax_motion_dir OF
                      0:;
                          {#ifdef CURVATURA}
                              CASE Asse OF
                                  //Nessuna forzatura di direzione si gestisce l^eventuale inversione del
                                  //conteggio e del riferimento DX<=>SX solo per le macchine in cui
                                  //l^operazione è possibile
                                  Y1,Q1:
                                      {#ifdef ELECT_ALL} 
                                          ConteggioDXSX(Asse);
                                      {#endif}
                                      {#ifdef ELECTM}
                                          ConteggioDXSX(Asse);
                                      {#endif}
                                      {#ifdef FOUR_RUNNER}
                                          ConteggioDXSX(Asse);
                                      {#endif}
                              END_CASE;
                              CURRENT_STATE:=STATE_INVERSIONEASSE_RESTART;
                          {#else}
                              CURRENT_STATE:=STATE_END;
                          {#endif};
                          
                      NO_INVERSIONE:
                          //Flag di inversione a OFF      
                          {#ifdef CURVATURA}
                              CASE Asse OF
                                  Y1,Q1{#ifdef MATRICE_ELETTRICA},Y2{#endif}:
                                      IF Senso=CURVATURA_DX THEN
                                          Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_1.inverscountdirection.encoderFeedbackPolarity:=NO;
                                          Assi[Asse].PosAx.setConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint:=NO;
                                      ELSE
                                          Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_1.inverscountdirection.encoderFeedbackPolarity:=YES;
                                          Assi[Asse].PosAx.setConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint:=YES;
                                      END_IF;
                                  ELSE
                                      Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_1.inverscountdirection.encoderFeedbackPolarity:=NO;
                                      Assi[Asse].PosAx.setConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint:=NO;
                                      IF Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.numberofencoders=2 THEN
                                          IF Assi[Asse].PosAx.setconfigdata.TypeOfAxis.NumberOfEncoders.encoder_2.encodermode<>SENSOR_ANALOG THEN
                                              //In caso di secondo encoder vero occorre gestire l^inversione del conteggio che per 
                                              //l^EFLEX facciamo diventare il contrario di quello dell^encoder 1.
                                              //DA SISTEMARE CON UNA GESTIONE MIGLIORE SE SARA^ NECESSARIO
                                              Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_2.inverscountdirection.encoderFeedbackPolarity:=YES;
                                          END_IF;
                                      END_IF;
                              END_CASE; 
                              CURRENT_STATE:=STATE_INVERSIONEASSE_RESTART;
                          {#else}	//Fine CURVATURA
                              Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_1.inverscountdirection.encoderFeedbackPolarity:=NO;
                              Assi[Asse].PosAx.setConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint:=NO;
                              IF Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.numberofencoders=2 THEN
                                  IF Assi[Asse].PosAx.setconfigdata.TypeOfAxis.NumberOfEncoders.encoder_2.encodermode<>SENSOR_ANALOG THEN
                                      //In caso di secondo encoder vero occorre gestire l^inversione del conteggio che per 
                                      //l^EFLEX facciamo diventare il contrario di quello dell^encoder 1.
                                      //DA SISTEMARE CON UNA GESTIONE MIGLIORE SE SARA^ NECESSARIO
                                      Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_2.inverscountdirection.encoderFeedbackPolarity:=YES;
                                  END_IF;
                              END_IF;
                              CURRENT_STATE:=STATE_INVERSIONEASSE_RESTART;
                          {#endif}	//Fine NO CURVATURA
                          
                      SI_INVERSIONE:;
                          //Flag di inversione a ON
                          {#ifdef CURVATURA}
                              CASE Asse OF
                                  Y1,Q1{#ifdef MATRICE_ELETTRICA},Y2{#endif}:
                                      IF Senso=CURVATURA_DX THEN
                                          Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_1.inverscountdirection.encoderFeedbackPolarity:=YES;
                                          Assi[Asse].PosAx.setConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint:=YES;
                                      ELSE
                                          Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_1.inverscountdirection.encoderFeedbackPolarity:=NO;
                                          Assi[Asse].PosAx.setConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint:=NO;
                                      END_IF;
                                  ELSE
                                      Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_1.inverscountdirection.encoderFeedbackPolarity:=YES;
                                      Assi[Asse].PosAx.setConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint:=YES;
                                      IF Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.numberofencoders=2 THEN
                                          IF Assi[Asse].PosAx.setconfigdata.TypeOfAxis.NumberOfEncoders.encoder_2.encodermode<>SENSOR_ANALOG THEN
                                              //In caso di secondo encoder vero occorre gestire l^inversione del conteggio che per 
                                              //l^EFLEX facciamo diventare il contrario di quello dell^encoder 1.
                                              //DA SISTEMARE CON UNA GESTIONE MIGLIORE SE SARA^ NECESSARIO
                                              Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_2.inverscountdirection.encoderFeedbackPolarity:=NO;
                                          END_IF;
                                      END_IF;
                              END_CASE;
                              CURRENT_STATE:=STATE_INVERSIONEASSE_RESTART;
                          {#else}	//Fine CURVATURA
                              Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_1.inverscountdirection.encoderFeedbackPolarity:=YES;
                              Assi[Asse].PosAx.setConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint:=YES;
                              IF Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.numberofencoders=2 THEN
                                  IF Assi[Asse].PosAx.setconfigdata.TypeOfAxis.NumberOfEncoders.encoder_2.encodermode<>SENSOR_ANALOG THEN
                                      //In caso di secondo encoder vero occorre gestire l^inversione del conteggio che per 
                                      //l^EFLEX facciamo diventare il contrario di quello dell^encoder 1.
                                      //DA SISTEMARE CON UNA GESTIONE MIGLIORE SE SARA^ NECESSARIO
                                      Assi[Asse].PosAx.SetConfigData.TypeOfAxis.numberofencoders.encoder_2.inverscountdirection.encoderFeedbackPolarity:=NO;
                                  END_IF;
                              END_IF;
                              CURRENT_STATE:=STATE_INVERSIONEASSE_RESTART;
                          {#endif}	//Fine NO CURVATURA
                  END_CASE;
              END_IF;
                  
          STATE_INVERSIONEASSE_RESTART:
              IF (RestartActivationAsse.outputState<>STATE_END) THEN
                  RestartActivationAsse(NumAsse:=Asse, Attesa:=TRUE);
              ELSE
                  RestartActivationAsse(NumAsse:=Asse, Attesa:=TRUE);
                  CURRENT_STATE:=STATE_END;
              END_IF;
              
          STATE_END:
              CURRENT_STATE:=STATE_INIT;
      END_CASE;
      OutputState:=CURRENT_STATE;*)
  END_FUNCTION_BLOCK
  
  FUNCTION_BLOCK FB_ManagementBrake
    
    VAR_EXTERNAL
        Start_read								: BOOL;
        i_sys_ReadingSinamicsInUse              :BOOL;
    END_VAR

    VAR_EXTERNAL CONSTANT
            driveObjectIdConstant               : UINT;
            hardwareIdConstant                  :WORD;
    END_VAR

    VAR_INPUT
      Asse         :INT;
      Stato        :SINT;
    END_VAR
    
    VAR
        Address 			:DINT;
        dret    			:DINT;
        //Ritorno 			:Ret_write_par;
          
        CURRENT_STATE					:INT:=GENERAL_STATE#STATE_INIT;
        FB_DisableAxes				    :FB_DisableAxes;
        ScritturaDatiSinamics	        :_WRITE_PARAMETER_SINGL;
    END_VAR

    VAR_OUTPUT
        OutputState	:INT;
    END_VAR

      CASE CURRENT_STATE OF
        GENERAL_STATE#STATE_INIT:
              IF (FB_DisableAxes.outputState<>GENERAL_STATE#STATE_END) THEN
                  //Occorre disabilitare l^asse altrimenti il nuovo firmware non accetta il comando
                  FB_DisableAxes(Asse:=Asse (*Regolatore:=INACTIVE*));  
              ELSE
                  FB_DisableAxes(Asse:=Asse (*Regolatore:=INACTIVE*));  
                (*Address:=UDINT_TO_DINT(Assi[Asse].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
                  CURRENT_STATE:=_FB_ManagementBrake#STATE_MANAGEMENT_BRAKE_WAIT_READING_SYNAMICS;
              END_IF;
              
        _FB_ManagementBrake#STATE_MANAGEMENT_BRAKE_WAIT_READING_SYNAMICS:
              IF NOT i_sys_ReadingSinamicsInUse THEN
                  CURRENT_STATE:=_FB_ManagementBrake#STATE_MANAGEMENT_BRAKE_WRITE_SYNAMICS;
              END_IF;
              
        _FB_ManagementBrake#STATE_MANAGEMENT_BRAKE_WRITE_SYNAMICS:

                IF NOT ScritturaDatiSinamics.Done THEN
                    ScritturaDatiSinamics(Start := Start_read,
                                        driveObjectId :=driveObjectIdConstant,
                                        hardwareId := hardwareIdConstant,
                                        parameterNumber:=uint#1215,
                                        index:=uint#0,
                                        value:=Stato);				
                ELSIF ScritturaDatiSinamics.Done THEN
                    ScritturaDatiSinamics(Start := Start_read,
                                        driveObjectId :=driveObjectIdConstant,
                                        hardwareId := hardwareIdConstant,
                                        parameterNumber:=uint#1215,
                                        index:=uint#0,
                                        value:=Stato);	                  
                    ScritturaDatiSinamics.Start :=false;
                    CURRENT_STATE:=GENERAL_STATE#STATE_END;
                ELSIF ScritturaDatiSinamics.error THEN
                    ScritturaDatiSinamics.Start :=false;
                END_IF;	
      END_CASE;
    //Occorre disabilitare l^asse altrimenti il nuovo firmware non accetta il comando
    FB_DisableAxes(Asse:=Asse (*Regolatore:=INACTIVE*));  
    (*Address:=UDINT_TO_DINT(Assi[Asse].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn);*)
    WHILE i_sys_ReadingSinamicsInUse=TRUE DO ;
      (*dret:=_waitTime(T#0ms); *)
    END_WHILE;

    IF NOT ScritturaDatiSinamics.Done THEN
        ScritturaDatiSinamics(Start := Start_read,
                            driveObjectId :=driveObjectIdConstant,
                            hardwareId := hardwareIdConstant,
                            parameterNumber:=uint#1215,
                            index:=uint#0,
                            value:=Stato);				
    ELSIF ScritturaDatiSinamics.Done THEN                  
        ScritturaDatiSinamics.Start :=false;
    ELSIF ScritturaDatiSinamics.error THEN
        ScritturaDatiSinamics.Start :=false;
    END_IF;
  END_FUNCTION_BLOCK
(*{
#ifdef RUNNER
}*)
	FUNCTION_BLOCK Module_2Axes

        VAR_EXTERNAL
            ModeSpecialReturnCuttingOrbital 	:INT;
            Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax; 
            pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        END_VAR
    
        VAR_EXTERNAL CONSTANT
            INACTIVE_DEACTIVATABLE_AXIS             :SINT;
        END_VAR

		VAR_INPUT
			Asse1          :INT;
			Asse2          :INT;
			Attesa         :BOOL;
		END_VAR

		VAR
            ResetMotionResetAxis				:FB_ResetMotionResetAxis;
            Homing          :Homing;
            Axes1          : REF_TO TO_SynchronousAxis;
            Axes2          : REF_TO TO_SynchronousAxis;
            Axes1MasterAxis :REF_TO TO_PositioningAxis;
            Axes2MasterAxis :REF_TO TO_PositioningAxis;
			Quota1         :LREAL;
			Quota2         :LREAL;
			QuotaIntera1   :DINT;
			QuotaIntera2   :DINT;
			QuotaIntera    :DINT;
			dret           :DINT; 
			Counter        :INT;
			
			CURRENT_STATE		:INT:=GENERAL_STATE#STATE_INIT;
            ReadPosSlave : ReadPosSlave;
		END_VAR
		VAR_OUTPUT
			OutputState			:INT;
		END_VAR
		CASE CURRENT_STATE OF
			GENERAL_STATE#STATE_INIT:
				IF ModeSpecialReturnCuttingOrbital >0 THEN
                    Axes1:= AsSynchronousAxisRef(Axes[Asse1].PosAx);
                    Axes2:= AsSynchronousAxisRef(Axes[Asse2].PosAx);
                    IF ABS(Axes1^.ActualVelocity )<0.1 AND  ABS(Axes2^. ActualVelocity )<0.1 THEN
					//IF ABS(Axes[Asse1].PosAx.motionStateData.commandVelocity)<0.1 AND  ABS(Assi[Asse2].PosAx.motionStateData.commandVelocity)<0.1 THEN
						CURRENT_STATE:=GENERAL_STATE#STATE_END;
					END_IF;
				ELSE
					IF pm[Asse1].ma_Asse_Abilitato=TRUE AND pm[Asse1].ma_Asse_Installato<>INACTIVE_DEACTIVATABLE_AXIS AND
						 pm[Asse2].ma_Asse_Abilitato=TRUE AND pm[Asse2].ma_Asse_Installato<>INACTIVE_DEACTIVATABLE_AXIS THEN
						 //La funzione viene eseguita solo se i due assi sono abilitati e non disattivati
						 IF Attesa THEN
							//Prima di eseguire il modulo si attende che i due assi raggiungano la posizione prevista dalla camma
                            Axes1MasterAxis:= AsPositioningAxisRef(Axes[Asse1].masteraxis);
							ReadPosSlave(NomeMaster:=Axes[Asse1].masteraxis,
																	 Asse:=Asse1,
																	 PosMaster:=Axes1MasterAxis^.actualposition, 
																	 Modo:=_EnumCamPositionMode#ACTUAL,
                                                                     PosSlave=>Quota1);
                            Axes2MasterAxis:= AsPositioningAxisRef(Axes[Asse2].masteraxis);                                         
							ReadPosSlave(NomeMaster:=Axes[Asse2].mastername,
																	 Asse:=Asse2, 
																	 PosMaster:=Axes2MasterAxis^.actualposition, 
																	 Modo:=_EnumCamPositionMode#ACTUAL,
                                                                     PosSlave=>Quota2);
							Counter:=0;
							CURRENT_STATE:=_Module_2Axes#STATE_MODULE_2AXIS_LOOP_WAITING;
						 ELSE
						 	CURRENT_STATE:=_Module_2Axes#STATE_MODULE_2AXIS_HOMING;
						 END_IF;
					ELSE
						CURRENT_STATE:=GENERAL_STATE#STATE_END;
					END_IF;
				END_IF;
				
            _Module_2Axes#STATE_MODULE_2AXIS_LOOP_WAITING:
                    Axes1:= AsSynchronousAxisRef(Axes[Asse1].PosAx);
                    Axes2:= AsSynchronousAxisRef(Axes[Asse2].PosAx);
				IF ABS(Axes1^.actualposition-Quota1)>0.001 OR
				   ABS(Axes2^.actualposition-Quota2)>0.001 THEN
					IF counter>50 THEN
						CURRENT_STATE:=_Module_2Axes#STATE_MODULE_2AXIS_RESET_AXIS1;
					(*ELSE
						dret:=_waitTime(T#10ms);	//%%%%
						Counter:=Counter+1;*)
					END_IF;
				ELSE
					CURRENT_STATE:=_Module_2Axes#STATE_MODULE_2AXIS_HOMING;
				END_IF;
				
            _Module_2Axes#STATE_MODULE_2AXIS_RESET_AXIS1:
				IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
					ResetMotionResetAxis(Asse:=Asse1);
				ELSE
					ResetMotionResetAxis(Asse:=Asse1);
					CURRENT_STATE:=_Module_2Axes#STATE_MODULE_2AXIS_RESET_AXIS2;
				END_IF;
				
            _Module_2Axes#STATE_MODULE_2AXIS_RESET_AXIS2:
				IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
					ResetMotionResetAxis(Asse:=Asse2);
				ELSE
					ResetMotionResetAxis(Asse:=Asse2);
					CURRENT_STATE:=_Module_2Axes#STATE_MODULE_2AXIS_HOMING;
				END_IF;
				
            _Module_2Axes#STATE_MODULE_2AXIS_HOMING:
                Axes1:= AsSynchronousAxisRef(Axes[Asse1].PosAx);
                Axes2:= AsSynchronousAxisRef(Axes[Asse2].PosAx);
				//Legge la quota dell^asse direttamente dal sensore di posizione
                Quota1:=Axes1^.StatusSensor[1].position;
				//Quota1:=Axes[Asse1].PosAx.sensordata[1].position;
				QuotaIntera1:=TO_DINT(Quota1/360);
                Quota2:=Axes2^.StatusSensor[1].position;
				//Quota2:=Axes[Asse2].PosAx.sensordata[1].position;
				QuotaIntera2:=TO_DINT(Quota2/360);
				IF ABS(QuotaIntera1)>ABS(QuotaIntera2) THEN
					Quota1:=Quota1-(QuotaIntera1*360);
					Quota2:=quota2-(QuotaIntera1*360);
				ELSE
					Quota1:=Quota1-(QuotaIntera2*360);
					Quota2:=quota2-(QuotaIntera2*360);
				END_IF;
				Homing(Asse:=Asse1, Quota:=Quota1, Modo:=EnumHomingMode#DIRECT_HOMING_ABSOLUTE);   
				CURRENT_STATE:=_Module_2Axes#STATE_MODULE_2AXIS_WAITING_HOMING_AXIS1;
				
            _Module_2Axes#STATE_MODULE_2AXIS_WAITING_HOMING_AXIS1:
                Axes1:= AsSynchronousAxisRef(Axes[Asse1].PosAx);
				IF Axes1^.StatusWord.%X5 THEN
					Homing(Asse:=Asse2, Quota:=Quota2, Modo:=EnumHomingMode#DIRECT_HOMING_ABSOLUTE);  
					CURRENT_STATE:=_Module_2Axes#STATE_MODULE_2AXIS_WAITING_HOMING_AXIS2;
				END_IF;
				
            _Module_2Axes#STATE_MODULE_2AXIS_WAITING_HOMING_AXIS2:
                Axes2:= AsSynchronousAxisRef(Axes[Asse2].PosAx);
				IF Axes2^.StatusWord.%X5 THEN
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
				
			GENERAL_STATE#STATE_END:
				CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
		END_CASE;
		OutputState:=CURRENT_STATE;
	END_FUNCTION_BLOCK  
(*{
#endif    //Fine RUNNER
}*)

/************************************************************************************************************
 * SUBJECT TO VERIFICATION OF THE ACTIVATION OF THE POWER SUPPLY, ENABLE ALL MACHINE AXES (DRIVE AND POWER) *
 *                                WITH VERIFICATION OF THE ACTUAL ACTIVATION                                *
 ************************************************************************************************************/
FUNCTION_BLOCK FB_EnableAssi

    VAR_EXTERNAL
        MinuteSavingEnergy                      :INT;
        i_sys_Change_DX_SX                      :BOOL;
        i_sys_enable_Alimen 				    :BOOL;
        i_sys_AutoSetting					    :INT;
        UsoPLCOpen						        :BOOL;
        i_sys_Alimen_Ok 			            :BOOL;
        Axes                                    :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax; 
        pm                                      : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        I_VerPLCSafety                          : BYTE;
    END_VAR

    VAR_EXTERNAL CONSTANT
        TRC_MISSING_ADDRESSES_IO            :INT;  //Mancano indirizzi hardware degli I/O
        TRC_MACHINE_DXSX                    :INT;  //Configurazione errata macchina DX SX
        TRC_BASCULANTE_DXSX                   :INT;
        TRC_BASCULANTE_SXDX                   :INT;
        TRC_BASCULANTE_NON_IMPOSTATO          :INT;
        AUTOSETTING_SETTING_SIDE_BASCULANT   :SINT;
        ERR_VERSION_SAFETY                 :DINT;
        VERSIONE_SAFETY                     :BYTE;
        TRC_VIOLATION_BARRIER                 :INT;
        TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION :INT;
        INACTIVE_DEACTIVATABLE_AXIS         :SINT;
        X1                                  :    INT;    //Cart X
        X2                                  :INT;
        A1                                  :INT;  //Rotation 4RTL/"Rotazione 4RC
        B1                                  :INT;  //Rotation 4RIPC
        INP_PHOTOCELLS_NOT_VIOLATED         :INT;
        TRC_DRIVE                           :INT;  //Azionamento non attivato + numero dellasse
        TRC_POWER                           :INT;  //Potenza non attivata + numero dellasse
        OUT_DRIVES_ON_DPCOUPLER				:INT;
        INP_BARRIER_2                       :INT;
    END_VAR

    VAR_INPUT
      AssiEsclusi		:DWORD:=DWORD#16#FFFFFFFF;
    END_VAR

    VAR
      PosAx  : REF_TO TO_SynchronousAxis;
      masteraxis : REF_TO TO_PositioningAxis;
      i							:INT;  
      i2            :INT;
      dret          :DINT;
          
          CURRENT_STATE									:INT:=GENERAL_STATE#STATE_INIT;
          CURRENT_AXIS									:INT:=-1;
          EnableAxisBLM									:FB_EnableAxisBLM;
          EnableMasterAxisBLM						    :FB_EnableMasterAxisBLM;
          VerificaConfiguraMacchinaDXSX	                :FB_VerificaConfiguraMacchinaDxSx;
    END_VAR
      VAR_OUTPUT
          OutputState	:INT;
          ReturnValue	:BOOL;
      END_VAR
      CASE CURRENT_STATE OF
        GENERAL_STATE#STATE_INIT:
              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
              StartTimerRisparmioEnergetico(Tempo:=MinuteSavingEnergy);
              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_CAMBIO_DX_SX;
              
        _FB_EnableAxis#STATE_ENABLEASSI_WAIT_CAMBIO_DX_SX:
              IF (i_sys_Change_DX_SX=FALSE) THEN
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_VERIFICA_IO;
              END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_WAIT_VERIFICA_IO:
              IF CheckIOAddresses()=TRUE THEN
                  i_sys_enable_alimen:=TRUE;
                  reset_trace(TRC_MISSING_ADDRESSES_IO);
                  (*{#ifdef CURVATURA}*)
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_VERIFICA_CONFIG_DX_SX;
                  (*{#else}*)
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_VERIFICA_SIMULAZIONE;
                  (*{#endif}*)
              ELSE
                  i_sys_enable_alimen:=FALSE;
                  set_trace(Code:=TRC_MISSING_ADDRESSES_IO);
              END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_WAIT_VERIFICA_CONFIG_DX_SX:
              (*{#ifdef CURVATURA}*)
                  IF (VerificaConfiguraMacchinaDxSx.OutputState<>GENERAL_STATE#STATE_END) THEN
                      VerificaConfiguraMacchinaDxSx();
                  ELSE
                      IF VerificaConfiguraMacchinaDxSx.ReturnValue=TRUE THEN
                          i_sys_enable_alimen:=TRUE;
                          reset_trace(TRC_MACHINE_DXSX);
                          (*{#ifdef SMART}*)
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_VERIFICA_BASCULANTE;
                          (*{#else}*)
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_VERIFICA_SIMULAZIONE;
                          (*{#endif}*)
                      ELSE
                          i_sys_enable_alimen:=FALSE;
                          set_trace(Code:=TRC_MACHINE_DXSX);
                      END_IF;
                      VerificaConfiguraMacchinaDxSx();
                  END_IF;			
              (*{#endif} //FINE CURVATURA*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_VERIFICA_BASCULANTE:
              (*{#ifdef SMART}*)
                  i:=VerificaBasculanteDxSx();
                  IF i<>2 THEN
                      i_sys_enable_alimen:=FALSE;
                      //VGP destro, configurazione attiva sinistra
                      IF (i=0) THEN
                          IF (i_sys_autosetting<>AUTOSETTING_SETTING_SIDE_BASCULANT) THEN
                              set_trace(Code:=TRC_BASCULANTE_DXSX);
                          END_IF;
                          i2:=i;
                      END_IF;
                      //VGP sinistro, configurazione attiva destra
                      IF (i=1) THEN
                          IF (i_sys_autosetting<>AUTOSETTING_SETTING_SIDE_BASCULANT) THEN
                              set_trace(Code:=TRC_BASCULANTE_SXDX);
                          END_IF;
                          i2:=i;
                      END_IF;
                      //configurazione attiva mancante 
                      IF (i=-1) THEN 
                          IF (i_sys_autosetting<>AUTOSETTING_SETTING_SIDE_BASCULANT) THEN
                              set_trace(Code:=TRC_BASCULANTE_NON_IMPOSTATO);
                          END_IF;
                          i2:=i;
                      END_IF;	
                  ELSE
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_VERIFICA_SIMULAZIONE;
                  END_IF;
              (*{#endif}; //FINE SMART*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_VERIFICA_SIMULAZIONE:
              IF VerificationSimulationAxes()=FALSE THEN
                 (*  {#ifdef PROFINET}
                      {#ifndef EMULAZIONE}*)
                          CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_VER_SAFETY;
                      (*{#else}	//FINE NO EMULAZIONE*)
                          CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_BARRIERE_NON_VIOLATE;
                     (*  {#endif} //FINE EMULAZIONE*)
                  (*{#else} //FINE PROFINET*)
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_BARRIERE_NON_VIOLATE;
                  (*{#endif}; //FINE NO PROFINET*)
              ELSE
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_GET_NEXT_STEP;
              END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_CHECK_VER_SAFETY:
              (*{#ifdef PROFINET}*)
                  IF I_VerPLCSafety<>VERSIONE_SAFETY THEN
                      dret:=Push_Error(code:=ERR_VERSION_SAFETY , Arresto:=TRUE);
                  ELSE
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_BARRIERE_NON_VIOLATE;
                  END_IF;
              (*{#endif}; //FINE PROFINET*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_WAIT_BARRIERE_NON_VIOLATE:
              IF (ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED)=TRUE) THEN
                  reset_trace(Code:=TRC_VIOLATION_BARRIER);
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_GET_NEXT_STEP;
              ELSE
                  set_trace(Code:=TRC_VIOLATION_BARRIER);
              END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_GET_NEXT_STEP:
              (*{#ifdef ALIMENTATORE_ACTIVE_LINE}*)
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_ALIMENTATORE_ON;
              (*{#else}*)
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_DH4010;
              (*{#endif}*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_WAIT_ALIMENTATORE_ON:
              (*{#ifdef ALIMENTATORE_ACTIVE_LINE}*)
                  IF (VerificationSimulationAxes()=FALSE AND i_sys_alimen_ok=FALSE) THEN
                      set_trace(Code:=TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION);
                  ELSE
                      reset_trace(TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION);
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_DH4010;
                  END_IF;
              (*{#endif};*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_CHECK_DH4010:
              (*{#ifdef DH4010}*)
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_1_DH4010;
              (*{#else}*)
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER;
              (*{#endif};*)
                  
        _FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_1_DH4010:
              (*{#ifdef DH4010}*)
                masteraxis := AsPositioningAxisRef(Axes[X1].masteraxis);
                IF masteraxis^.StatusWord.%X0 = TRUE THEN
                      IF (USOPLCOpen) THEN
                          IF (EnableMasterAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                              EnableMasterAxisBLM(0(* WHEN_COMMAND_DONE*)); ///Con altri casi e guale
                          ELSE
                              EnableMasterAxisBLM(0 (* , WHEN_COMMAND_DONE*));
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_2_DH4010;
                          END_IF;
                      ELSE
                          IF (EnableAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                              EnableAxisBlm(Axis:=Axes[X1].masteraxis, NumeroAsse:=-1, (*ModoAttesa:=WHEN_COMMAND_DONE,*) NumMaster:=0);
                          ELSE
                              EnableAxisBlm(Axis:=Axes[X1].masteraxis, NumeroAsse:=-1, (*ModoAttesa:=WHEN_COMMAND_DONE,*) NumMaster:=0);
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_2_DH4010;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#endif};*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_2_DH4010:
              (*{#ifdef DH4010}*)
              masteraxis := AsPositioningAxisRef(Axes[X2].masteraxis);
              IF masteraxis^.StatusWord.%X0 = TRUE THEN
                      IF (USOPLCOpen) THEN
                          IF (EnableMasterAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                              EnableMasterAxisBLM(1 (*Axes[X2].masteraxis, WHEN_COMMAND_DONE*));
                          ELSE
                              EnableMasterAxisBLM(1 (*Axes[X2].masteraxis, WHEN_COMMAND_DONE*));
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_FOUR_RUNNER;
                          END_IF;
                      ELSE
                          IF (EnableAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                              EnableAxisBlm(Axis:=Axes[X2].masteraxis, NumeroAsse:=-1, (*ModoAttesa:=WHEN_COMMAND_DONE,*) NumMaster:=1);
                          ELSE
                              EnableAxisBlm(Axis:=Axes[X2].masteraxis, NumeroAsse:=-1, (*ModoAttesa:=WHEN_COMMAND_DONE,*) NumMaster:=1);
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_FOUR_RUNNER;
                          END_IF;
                      END_IF;
                  END_IF;
              (*{#endif};*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER:
                masteraxis := AsPositioningAxisRef(Axes[X1].masteraxis);
                IF masteraxis^.StatusWord.%X0 = TRUE THEN
                  IF (USOPLCOpen) THEN
                      IF (EnableMasterAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                          EnableMasterAxisBLM(0 (*Axes[X1].masteraxis*));
                      ELSE
                          EnableMasterAxisBLM(0 (* , Axes[X1].masteraxis*));
                          CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_FOUR_RUNNER;
                      END_IF;
                  ELSE
                      IF (EnableAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                          EnableAxisBlm(Axis:=Axes[X1].masteraxis, NumeroAsse:=-1(* , ModoAttesa:=WHEN_COMMAND_DONE*));
                      ELSE
                          EnableAxisBlm(Axis:=Axes[X1].masteraxis, NumeroAsse:=-1 (* , ModoAttesa:=WHEN_COMMAND_DONE*));
                          CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_FOUR_RUNNER;
                      END_IF;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_FOUR_RUNNER;
              END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_CHECK_FOUR_RUNNER:
              (*{#ifdef FOUR_RUNNER_PLAUNO}
                  {#ifndef PLAUNO_LIGHT}*)
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_TAGLIO_ESTERNO;
                  (*{#else}*)
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_RUNNER;
                  (*{#endif}
              {#else}*)
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_RUNNER;
              (*{#endif};*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_TAGLIO_ESTERNO:
              (*{#ifdef FOUR_RUNNER_PLAUNO}
                  {#ifndef PLAUNO_LIGHT}*)
                        masteraxis := AsPositioningAxisRef(Axes[A1].masteraxis);
                        IF masteraxis^.StatusWord.%X0 = TRUE THEN
                          IF (USOPLCOpen) THEN
                              IF (EnableMasterAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                                  EnableMasterAxisBLM(1 (* , Axes[A1].masteraxis, WHEN_COMMAND_DONE*));
                              ELSE
                                  EnableMasterAxisBLM(1 (* , Axes[A1].masteraxis, WHEN_COMMAND_DONE*));
                                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_RUNNER;
                              END_IF;
                          ELSE
                              IF (EnableAxisBlm.outputState<>GENERAL_STATE#STATE_END) THEN
                                  EnableAxisBlm(Axis:=Axes[A1].masteraxis, NumeroAsse:=-1(* , ModoAttesa:=WHEN_COMMAND_DONE*));
                              ELSE
                                  EnableAxisBlm(Axis:=Axes[A1].masteraxis, NumeroAsse:=-1 (* , ModoAttesa:=WHEN_COMMAND_DONE*));
                                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_CHECK_RUNNER;
                              END_IF;
                          END_IF;
                      END_IF; 
                  (*{#endif};
              {#endif};*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_CHECK_RUNNER:
              (*{#ifdef RUNNER}
                  {#ifdef B1_B2}*)
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_TAGLIO_INTERNO;
                  (*{#else}*)
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_LOOP;
                  (*{#endif}
              {#else}*)
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_LOOP;
              (*{#endif};*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_ENABLE_MASTER_TAGLIO_INTERNO:
              (*{#ifdef B1_B2}*)
              masteraxis := AsPositioningAxisRef(Axes[B1].masteraxis);
                  IF masteraxis^.StatusWord.%X0 = TRUE THEN
                      IF (USOPLCOpen) THEN
                          IF (EnableMasterAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                              EnableMasterAxisBLM(2 (* , Axes[B1].masteraxis, WHEN_COMMAND_DONE*));
                          ELSE
                              EnableMasterAxisBLM(2(* , Axes[B1].masteraxis, WHEN_COMMAND_DONE*));
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_LOOP;
                          END_IF;
                      ELSE
                          IF (EnableAxisBlm.outputState<>GENERAL_STATE#STATE_END) THEN
                              EnableAxisBlm(Axis:=Axes[B1].masteraxis, NumeroAsse:=-1 (* , ModoAttesa:=WHEN_COMMAND_DONE*));
                          ELSE
                              EnableAxisBlm(Axis:=Axes[B1].masteraxis, NumeroAsse:=-1 (* , ModoAttesa:=WHEN_COMMAND_DONE*));
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_LOOP;
                          END_IF;
                      END_IF;
                  END_IF; 
              (*{#endif};*)
              
        _FB_EnableAxis#STATE_ENABLEASSI_ENABLE_LOOP:
              CURRENT_AXIS:=CURRENT_AXIS+1;
              IF (CURRENT_AXIS<_MAXIS#MAXAXIS) THEN
                  IF CheckIfTOactive(Axis:=CURRENT_AXIS) THEN
                      IF getBit(AssiEsclusi, TO_USINT(i))=FALSE OR AssiEsclusi=DWORD#16#FFFFFFFF THEN
                        PosAx:=AsSynchronousAxisRef(axes[i].PosAx);//Valore di I
                        IF TestAssePronto(Asse:=i)=FALSE OR 
                           PosAx^.StatusWord.%X0 = FALSE OR
                           PosAx^.StatusWord.%X0 = FALSE OR
                           posax^.VirtualAxis.Mode = UDINT#1 THEN
                              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_SINGLE_AXIS;
                          END_IF;
                      END_IF;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_RETURNVALUE;
              END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_ENABLE_SINGLE_AXIS:
              IF (EnableAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                  EnableAxisBlm(Axis:=Axes[CURRENT_AXIS].PosAx, NumeroAsse:=CURRENT_AXIS (* , ModoAttesa:=IMMEDIATELY*));
              ELSE
                  IF (VerificationSimulationAxes()=TRUE) THEN
                      CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_SIMULATION_SINGLE_AXIS;
                  ELSE
                      IF pm[i].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS AND pm[i].ma_Asse_Abilitato=TRUE THEN
                          CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_DRIVESTATE_ACTIVE;
                      ELSE
                          CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_LOOP;
                      END_IF;
                  END_IF;
              END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_ENABLE_SIMULATION_SINGLE_AXIS:
              IF pm[CURRENT_AXIS].ma_asse_installato=INACTIVE_DEACTIVATABLE_AXIS AND pm[CURRENT_AXIS].ma_Asse_Abilitato=TRUE THEN
                 ; //dret:=_enableAxisSimulation(axis:=Axes[CURRENT_AXIS].PosAx,nextCommand:=WHEN_COMMAND_DONE, commandId:=_getCommandId());	//%%%%
              END_IF;  
              CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_LOOP;
              
        // _FB_EnableAxis#STATE_ENABLEASSI_WAIT_DRIVESTATE_ACTIVE: %%%%% in attesa
        //       IF Axes[CURRENT_AXIS].PosAx.actormonitoring.drivestate<>INACTIVE THEN
        //           reset_trace(TRC_DRIVE);
        //           IF pm[i].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS THEN
        //               CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_POWER_ACTIVE;
        //           ELSE
        //               CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_CONTROL_ACTIVE;
        //           END_IF;
        //       ELSE
        //           set_trace(Code:=TRC_DRIVE, Axis:=CURRENT_AXIS);
        //       END_IF;
              
        // _FB_EnableAxis#STATE_ENABLEASSI_WAIT_POWER_ACTIVE:
        //       IF Axes[CURRENT_AXIS].PosAx.actormonitoring.power<>INACTIVE THEN
                  
        //           CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_WAIT_CONTROL_ACTIVE;
        //       ELSE
        //           set_trace(Code:=TRC_POWER , Axis:=CURRENT_AXIS); 
        //       END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_WAIT_CONTROL_ACTIVE:
              PosAx:=AsSynchronousAxisRef(axes[i].PosAx);//Valore di I
              IF PosAx^.StatusWord.%X0 = FALSE THEN
                  reset_trace(Code:=TRC_POWER );
                  CURRENT_STATE:=_FB_EnableAxis#STATE_ENABLEASSI_ENABLE_LOOP;
              ELSE
                  set_trace(Code:=TRC_POWER , Axis:=CURRENT_AXIS); 
              END_IF;
              
        _FB_EnableAxis#STATE_ENABLEASSI_RETURNVALUE:
              IF VerificationSimulationAxes()=FALSE THEN
                  (*{#ifdef ORBITALE}*)
                      //Verifica che nel frattempo non sia stata violata almeno una delle zone di sicurezza
                      IF readdigitalinput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED)=TRUE AND readdigitalinput(NumInput:=INP_BARRIER_2)=TRUE THEN
                          ReturnValue:=TRUE;
                      ELSE
                          ReturnValue:=FALSE;
                      END_IF;
                  (*{#else}	//Fine ORBITALE*)    
                      //Verifica che nel frattempo non sia stata violata la zona di sicurezza
                      IF readdigitalinput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED)=TRUE THEN
                          ReturnValue:=TRUE;
                      ELSE
                          ReturnValue:=FALSE;
                      END_IF;
                  (*{#endif} //Fine NO ORBITALE *)   
              ELSE
                  ReturnValue:=TRUE;
              END_IF;
              WriteDigitalOutput(NumOutput:=OUT_DRIVES_ON_DPCOUPLER, Value:=ReturnValue);
              CURRENT_STATE:=GENERAL_STATE#STATE_END;
              
          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
      END_CASE;
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK 
  
/*****************************************************************************************
 * VERIFIES THE CORRECT MATCH BETWEEN THE MACHINE'S PROGRAMMING AND THE DX SX STATUS SET *
 *****************************************************************************************/

  FUNCTION_BLOCK FB_VerificaConfiguraMacchinaDxSx

    VAR_EXTERNAL
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax; 
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        i_sys_sense_curve               :INT;		//1=Macchina DX  -1=Macchina SX
    END_VAR

    VAR_EXTERNAL CONSTANT
        Y1 :    INT;  //Y-axis bending arm
        Q1 :    INT;  //Horizontal head
        DX                                              :INT;
    END_VAR

	VAR 
		RiferimentoY  :_EnumYesNo;
		EncoderY      :_EnumYesNo;
		RiferimentoQ  :_EnumYesNo;
		EncoderQ      :_EnumYesNo;
		dret          :DINT;
		Esito         :BOOL;
		
		CURRENT_STATE			:INT:=GENERAL_STATE#STATE_INIT;
		AttesaAvvioSistema:FB_WaitingStartUpSystem;
        AxesY1         :REF_TO TO_SynchronousAxis;
        AxesQ1         :REF_TO TO_SynchronousAxis;
	END_VAR 
	VAR_OUTPUT
		OutputState			:INT;
		ReturnValue			:BOOL;
	END_VAR
	(*{#ifdef CURVATURA}*)
		CASE CURRENT_STATE OF
			GENERAL_STATE#STATE_INIT:
				(*{#ifdef DXSX_PROCESSO}*)
					ReturnValue:=TRUE;
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				(*{#else}*)
					ReturnValue:=FALSE;
					IF (i_sys_sense_curve=0) THEN
						CURRENT_STATE:=GENERAL_STATE#STATE_END;
					ELSE
						CURRENT_STATE:=_FB_VerificaConfiguraMacchinaDxSx#STATE_VERIFICATION_CONFIG_DXSX_WAITING_END_RESTART_Y1;
					END_IF;
				(*{#endif} //FINE NO DXSX_PROCESSO*)
				
			_FB_VerificaConfiguraMacchinaDxSx#STATE_VERIFICATION_CONFIG_DXSX_WAITING_END_RESTART_Y1:
                AxesY1:= AsSynchronousAxisRef(Axes[Y1].PosAx);
                AxesQ1:= AsSynchronousAxisRef(Axes[Q1].PosAx);
				IF NOT (AxesY1^.StatusWord.%X2  OR AxesQ1^.StatusWord.%X2) THEN
					CURRENT_STATE:=_FB_VerificaConfiguraMacchinaDxSx#STATE_VERIFY_CONFIG_DXSX_WAIT_RESTART_SYSTEM;
				END_IF;
				
			_FB_VerificaConfiguraMacchinaDxSx#STATE_VERIFY_CONFIG_DXSX_WAIT_RESTART_SYSTEM:
				IF (AttesaAvvioSistema.outputState<>GENERAL_STATE#STATE_END) THEN
					AttesaAvvioSistema(AttesaSimotion:=TRUE, AttesaSinamics:=FALSE);
				ELSE
					AttesaAvvioSistema(AttesaSimotion:=TRUE, AttesaSinamics:=FALSE);

                    AxesY1:= AsSynchronousAxisRef(Axes[Y1].PosAx);
                    IF AxesY1^.Sensor[1].InverseDirection  THEN
                        EncoderY:=_EnumYesNo#YES;
                    ELSE
                        EncoderY:=_EnumYesNo#NO;
                    END_IF;

                    IF AxesY1^.Actor.InverseDirection  THEN
                        RiferimentoY:=_EnumYesNo#YES;
                    ELSE
                        RiferimentoY:=_EnumYesNo#NO;
                    END_IF; 

                    AxesQ1:= AsSynchronousAxisRef(Axes[Q1].PosAx);
                    IF AxesQ1^.Sensor[1].InverseDirection  THEN
                        EncoderQ:=_EnumYesNo#YES;
                    ELSE
                        EncoderQ:=_EnumYesNo#NO;
                    END_IF;

                    IF AxesQ1^.Actor.InverseDirection  THEN
                        RiferimentoQ:=_EnumYesNo#YES;
                    ELSE
                        RiferimentoQ:=_EnumYesNo#NO;
                    END_IF;
                      
					//EncoderY:=Axes[Y1].PosAx.ActiveConfigData.TypeOfAxis.NumberOfEncoders.Encoder_1.InversCountDirection.encoderFeedbackPolarity;
					//RiferimentoY:=Axes[Y1].PosAx.ActiveConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint;
					//EncoderQ:=Axes[Q1].PosAx.ActiveConfigData.TypeOfAxis.NumberOfEncoders.Encoder_1.InversCountDirection.encoderFeedbackPolarity;
					//RiferimentoQ:=Axes[Q1].PosAx.ActiveConfigData.TypeOfAxis.SetPointDriverInfo.InvertSetPoint.invSetPoint;
					CURRENT_STATE:=_FB_VerificaConfiguraMacchinaDxSx#STATE_VERIFY_CONFIG_DXSX_RETURN_VALUE;
				END_IF;
				
			_FB_VerificaConfiguraMacchinaDxSx#STATE_VERIFY_CONFIG_DXSX_RETURN_VALUE:
				IF i_sys_sense_curve=DX THEN
					//Macchina Destra
					IF pm[Y1].ma_ax_motion_dir>=0 THEN
						//Nessuna inversione Y 
						IF EncoderY=_EnumYesNo#NO THEN
							IF RiferimentoY=_EnumYesNo#NO THEN
								IF pm[Q1].ma_ax_motion_dir>=0 THEN
									//Nessuna inversione Q
									IF EncoderQ=_EnumYesNo#NO THEN
										IF RiferimentoQ=_EnumYesNo#NO THEN
											ReturnValue:=TRUE;
										END_IF;
									END_IF;
								ELSE
									//Inversione Q    
									IF EncoderQ=_EnumYesNo#YES THEN
										IF RiferimentoQ=_EnumYesNo#YES THEN
											ReturnValue:=TRUE;
										END_IF;
									END_IF;
								END_IF;
							END_IF;
						END_IF;
					ELSE
						//Inversione Y      
						IF EncoderY=_EnumYesNo#YES THEN
							IF RiferimentoY=_EnumYesNo#YES THEN
								IF pm[Q1].ma_ax_motion_dir>=0 THEN
									//Nessuna inversione Q
									IF EncoderQ=_EnumYesNo#NO THEN
										IF RiferimentoQ=_EnumYesNo#NO THEN
											ReturnValue:=TRUE;
										END_IF;
									END_IF;
								ELSE
									//Inversione Q    
									IF EncoderQ=_EnumYesNo#YES THEN
										IF RiferimentoQ=_EnumYesNo#YES THEN
											ReturnValue:=TRUE;
										END_IF;
									END_IF;
								END_IF;
							END_IF;
						END_IF;
					END_IF;
				ELSE
					//Macchina Sinistra
					IF pm[Y1].ma_ax_motion_dir>=0 THEN
						//Nessuna inversione Y 
						IF EncoderY=_EnumYesNo#YES THEN
							IF RiferimentoY=_EnumYesNo#YES THEN
								IF pm[Q1].ma_ax_motion_dir>=0 THEN
									//Nessuna inversione Q
									IF EncoderQ=_EnumYesNo#YES THEN
										IF RiferimentoQ=_EnumYesNo#YES THEN
											ReturnValue:=TRUE;
										END_IF;
									END_IF;
								ELSE
									//Inversione Q    
									IF EncoderQ=_EnumYesNo#NO THEN
										IF RiferimentoQ=_EnumYesNo#NO THEN
											ReturnValue:=TRUE;
										END_IF;
									END_IF;
								END_IF;
							END_IF;
						END_IF;
					ELSE
						//Inversione Y      
						IF EncoderY=_EnumYesNo#NO THEN
							IF RiferimentoY=_EnumYesNo#NO THEN
								IF pm[Q1].ma_ax_motion_dir>=0 THEN
									//Nessuna inversione Q
									IF EncoderQ=_EnumYesNo#YES THEN
										IF RiferimentoQ=_EnumYesNo#YES THEN
											ReturnValue:=TRUE;
										END_IF;
									END_IF;
								ELSE
									//Inversione Q    
									IF EncoderQ=_EnumYesNo#NO THEN
										IF RiferimentoQ=_EnumYesNo#NO THEN
											ReturnValue:=TRUE;
										END_IF;
									END_IF;
								END_IF;
							END_IF;
						END_IF;
					END_IF;
				END_IF;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
				
			GENERAL_STATE#STATE_END:
				CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
		END_CASE;
	(*{#else}	//FINE CURVATURA*)
		ReturnValue:=TRUE;
		CURRENT_STATE:=GENERAL_STATE#STATE_END;
	(*{#endif} //FINE NO CURVATURA*)
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK 

FUNCTION_BLOCK FB_EnableAxisBlm

    VAR_EXTERNAL
        MinuteSavingEnergy              :INT;
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        Start_read						: BOOL;
    END_VAR

    VAR_EXTERNAL CONSTANT
        TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION    :INT;
        driveObjectIdConstant                   : UINT;
        hardwareIdConstant                      :WORD;
        TRC_DEACTIVATION_SAFETY                 :INT;
        INACTIVE_DEACTIVATABLE_AXIS             :SINT;

    END_VAR

    VAR_INPUT
      Axis        :DB_ANY;
      NumeroAsse  :INT;
      ModoAttesa  :EnumNextCommandEnable;
          NumMaster		:INT:=0;
    END_VAR

    VAR
          simotion_plc    : WORD;
          posax                   : REF_TO TO_SynchronousAxis;
          dret					:DINT;
          Address				:DINT;
          AppoSafety  	:DWORD;    
          StatoSafety		:REAL;
      
          CURRENT_STATE				:INT:=GENERAL_STATE#STATE_INIT;
          EnableSingleAxisBLM	    :FB_EnableSingleAxisBLM;
          DisableAssi			    :FB_DisableAxes;
          LetturaSinamicsREAL       :_READ_PARAMETER_SINGLE;
    END_VAR
      VAR_OUTPUT
          OutputState		:INT;
      END_VAR

      CASE CURRENT_STATE OF
          GENERAL_STATE#STATE_INIT:
              StartTimerRisparmioEnergetico(Tempo:=MinuteSavingEnergy);
              IF NumeroAsse=-1 THEN
                  //Asse Master sempre abilitato
                    posax:= AsSynchronousAxisRef(Axis);
                  IF PosAx^.StatusWord.%X0 = FALSE THEN //enable //.STATUSWORD.%X0
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_ENABLE;
                  ELSE
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                  END_IF;
              ELSE
                posax:= AsSynchronousAxisRef(Axes[NumeroAsse].posax);
                  IF posax^.VirtualAxis.Mode=UDINT#1 THEN
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_ENABLE;
                  ELSE
                     (*  {#ifdef PROFINET}
                          {#ifndef EMULAZIONE}*)
                              CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_PROFINET;
                          (*{#else}*)
                              CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_CHECK_SAFETY_INTEGRATED;
                          (*{#endif}
                      {#else}*)
                          CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_CHECK_SAFETY_INTEGRATED;
                     (*  {#endif}*)
                  END_IF;
              END_IF;
              
              _FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_PROFINET:
              (*{#ifdef PROFINET}*)
                  //Attesa che l^asse non sia in stato safety
                  AppoSafety:=ReadStateSafety(Asse:=NumeroAsse);
                  IF AppoSafety<>DWORD#0 AND pm[NumeroAsse].ma_Asse_Abilitato=TRUE AND ModoAttesa<>EnumNextCommandEnable#IMMEDIATELY THEN
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_WAIT_EMERGENZA;
                  ELSE
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_CHECK_SAFETY_INTEGRATED;
                  END_IF;
              (*{#endif};*)
              
              _FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_WAIT_EMERGENZA:
             (*  {#ifdef PROFINET}*)
                  AppoSafety:=ReadStateSafety(Asse:=NumeroAsse);
                  IF (getBit(AppoSafety, USINT#5)=TRUE) THEN
                      //In caso di stato di emergenza impostato dal PLC safety
                      //agli azionamenti, occorre indicare all^operatore di premere
                      //e rilasciare il fungo per resettare il PLC safety da
                      //tale condizione
                      set_trace(Code:= TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION);
                  ELSE
                      Reset_Trace(Code:= TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION);
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_WAIT_SAFETY_OFF;
                  END_IF;
              (*{#endif};*)
              
              _FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_WAIT_SAFETY_OFF:
              (*{#ifdef PROFINET}*)
                  AppoSafety:=ReadStateSafety(Asse:=NumeroAsse);
                  IF (AppoSafety<>DWORD#0) THEN
                      set_trace(Code:=TRC_DEACTIVATION_SAFETY , Axis:=NumeroAsse);
                  ELSE
                      reset_trace(Code:=TRC_DEACTIVATION_SAFETY );
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_CHECK_SAFETY_INTEGRATED;
                  END_IF;
              (*{#endif};*)
              
              _FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_CHECK_SAFETY_INTEGRATED:
              (*{#ifdef SAFETY_INTEGRATED}*)
                IF VerificationSimulationAxes()=FALSE AND pm[NumeroAsse].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS  THEN
                      //Attende per tutti gli assi attivi che la condizione di safety 
                      //venga tolta prima di abilitare l^asse

                      
                     LetturaSinamicsReal(
                                                        driveObjectId := driveObjectIdConstant,                                  
                                                        hardwareId := hardwareIdConstant, 
                                                        parameterNumber:=UINT#9722, 
                                                        index:=UINT#0,
                                                        Value_REAL=>StatoSafety);
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_INTEGRATED_WAIT_SAFETY_OFF;
                  ELSE
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_TESTL1;
                  END_IF;
              (*{#else}*)
                  //CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_TESTL1;
              (*{#endif};*)
              
              _FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_INTEGRATED_WAIT_SAFETY_OFF:
              (*{#ifdef SAFETY_INTEGRATED}*)
                 LetturaSinamicsReal(driveObjectId := driveObjectIdConstant,                                  
                                        hardwareId := hardwareIdConstant, 
                                        parameterNumber:=UINT#9722, 
                                        index:=UINT#0,
                                        Value_REAL=>StatoSafety);
                  IF getBit(TO_DWORD(StatoSafety),USINT#0)=TRUE OR
                                                          getBit(TO_DWORD(StatoSafety),USINT#1)=TRUE OR
                                                          getBit(TO_DWORD(StatoSafety),USINT#2)=TRUE OR  
                                                          getBit(TO_DWORD(StatoSafety),USINT#3)=TRUE THEN
                      set_trace(Code:=TRC_DEACTIVATION_SAFETY , Axis:=NumeroAsse);
                      //Avviso PLC di sicurezza per eventuale ripetizione comando
                      //di reset della safety
                      simotion_plc:=setbit(simotion_plc, USINT#4, TRUE);
                  ELSE
                      simotion_plc:=setbit(simotion_plc, USINT#4, FALSE);
                      reset_trace(Code:=TRC_DEACTIVATION_SAFETY );	
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_TESTL1;
                  END_IF;
              (*{#endif};*)
              
              _FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_TESTL1:
        IF NoTestL1(Axis:=NumeroAsse)=TRUE THEN 
          //Asse non L1 si abilita anche se è disattivato (sarebbe da migliorare ma occorre un test di verifica)
          IF pm[NumeroAsse].ma_asse_abilitato=TRUE THEN
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_ENABLE;
          END_IF;
        ELSE
          IF pm[NumeroAsse].ma_asse_installato=INACTIVE_DEACTIVATABLE_AXIS  THEN
            //Asse L1 si disabilita se è disattivato
                      CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_DISABLE; 
          ELSE
            //Asse L1 si abilita solo se non è disattivato
            IF pm[NumeroAsse].ma_asse_abilitato=TRUE THEN
                          CURRENT_STATE:=_FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_ENABLE;
            END_IF;
          END_IF;
        END_IF;
              
        _FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_ENABLE:
              IF (EnableSingleAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
                  EnableSingleAxisBLM(NumeroAsse );
                  //EnableSingleAxisBLM(Axis, NumeroAsse (* , ModoAttesa*));
              ELSE
                  EnableSingleAxisBLM(NumeroAsse );
                  //EnableSingleAxisBLM(Axis, NumeroAsse (* , ModoAttesa*));
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
        _FB_EnableSingleAxisBLM#STATE_ENABLESINGLEAXIS_DISABLE:
              IF (DisableAssi.outputState<>GENERAL_STATE#STATE_END) THEN
                  DisableAssi(Asse:=NumeroAsse(* , Regolatore:=INACTIVE*));  
              ELSE
                  DisableAssi(Asse:=NumeroAsse(* , Regolatore:=INACTIVE*));  
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;		
              
          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
      END_CASE;
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK

  FUNCTION_BLOCK FB_SET_Condizione_Move_Asse

    VAR_EXTERNAL
        R_Plus		                            :R_TRIG;
        R_Minus                                 :R_TRIG;
    END_VAR

    VAR_INPUT
      Asse	:INT;
    END_VAR
      VAR
          dret	:DINT;
          
          SkippedCode			:BOOL:=FALSE;
          CURRENT_STATE		:INT:=GENERAL_STATE#STATE_INIT;
        //   {#ifdef RUNNER} %%%%
        //       Modulo_2Assi		:FB_Modulo_2Assi;
        //   {#endif}
        //   MovimentaPaletta:FB_MovimentaPaletta;
      END_VAR
      VAR_OUTPUT
          OutputState			:INT;
      END_VAR
      CASE CURRENT_STATE OF
          GENERAL_STATE#STATE_INIT:
              IF Asse>=_FIRST_AXIS#FIRST_AXIS AND Asse<=_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 THEN
                //   {#ifdef RUNNER} %%%%
                //       {#ifdef ASPO_RUNNER}
                //           IF Asse=X1 AND i_sys_senzatubo=FALSE AND (r_plus.Q=TRUE OR r_minus.Q=TRUE) THEN
                //               //Chiusura rulli trascinamento
                //               Set_Disp(Ndisp:=RULLI_TRASCINAMENTO, Stato:=LAVORO, TimerRisparmio:=TRUE);
                //           END_IF;
                //       {#endif}	//Fine ASPO_RUNNER
                //       {#ifdef FOUR_RUNNER_PLAUNO}
                //           CASE Asse OF
                //               X1,Q1,V1:
                //                   Set_Disp(Ndisp:=FORATURA_MORSA, Stato:=RIPOSO, TimerRisparmio:=FALSE);
                //           END_CASE;
                //           IF Asse=X1 THEN
                //               WriteDigitalOutput(NumOutput:=OUT_ASPO_VERTICALE_NO_ATTREZZAGGIO, Value:=TRUE);
                //               {#ifdef B1_B2}
                //                   IF (Modulo_2Assi.outputState<>STATE_END) THEN
                //                       Modulo_2Assi(Asse1:=B1, Asse2:=B2, Attesa:=FALSE);
                //                       SkippedCode:=TRUE;
                //                       GOTO EsciPrima;
                //                   ELSE
                //                       SkippedCode:=FALSE;
                //                       Modulo_2Assi(Asse1:=B1, Asse2:=B2, Attesa:=FALSE);
                //                   END_IF;
                //               {#endif} //FINE B1_B2
                //           ELSE
                //               WriteDigitalOutput(NumOutput:=OUT_ASPO_VERTICALE_NO_ATTREZZAGGIO, Value:=FALSE);
                //           END_IF;
                //           {#ifndef PLAUNO_LIGHT}
                //               IF TestBloccaggio4RCPresente()=TRUE THEN
                //                   set_disp(Ndisp:=BLOCCAGGIO_4RC, Stato:=RIPOSO, TimerRisparmio:=FALSE);
                //               END_IF;
                //           {#endif}	//FINE NO PLAUNO_LIGHT
                //       {#endif}  //Fine FOUR_RUNNER_PLAUNO 
                //       {#ifdef THREE_RUNNER}
                //           IF Asse=X1 THEN
                //               WriteDigitalOutput(NumOutput:=OUT_ASPO_VERTICALE_NO_ATTREZZAGGIO, Value:=TRUE);
                //               {#ifdef B1_B2}
                //                   IF (Modulo_2Assi.outputState<>STATE_END) THEN
                //                       Modulo_2Assi(Asse1:=B1, Asse2:=B2, Attesa:=FALSE);
                //                       SkippedCode:=TRUE;
                //                       GOTO EsciPrima;
                //                   ELSE
                //                       SkippedCode:=FALSE;
                //                       Modulo_2Assi(Asse1:=B1, Asse2:=B2, Attesa:=FALSE);
                //                   END_IF;
                //               {#endif} //FINE B1_B2
                //           ELSE
                //               WriteDigitalOutput(NumOutput:=OUT_ASPO_VERTICALE_NO_ATTREZZAGGIO, Value:=FALSE);
                //           END_IF;
                //       {#endif}  //Fine THREE_RUNNER
                //   {#endif}  //Fine RUNNER
                //   EsciPrima:;
                  IF (SkippedCode=FALSE) THEN
                      IF r_plus.Q=TRUE OR r_minus.Q=TRUE THEN
                          CURRENT_STATE:=_FB_Set_Condizione_Move_Asse#STATE_SET_CONDIZIONE_MOVE_ASSE_PRESSIONE_DIREZIONE;
                    //   ELSE
                    //       {#ifdef DH4010}
                    //           CURRENT_STATE:=STATE_SET_CONDIZIONE_MOVE_ASSE_DH4010_RILASCIO_DIREZIONE;
                    //       {#else}
                    //           CURRENT_STATE:=STATE_END;
                    //        {#endif}
                      END_IF;
                  END_IF;
              ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
        _FB_Set_Condizione_Move_Asse#STATE_SET_CONDIZIONE_MOVE_ASSE_PRESSIONE_DIREZIONE:
            //   CASE Asse OF
                //   {#ifdef ORBITALE}
                    //   {#ifdef EFLEX} %%%%
                    //       X1:
                    //           IF i_sys_SenzaTubo=FALSE THEN
                    //               Set_Disp(Ndisp:=RULLI_TRASCINAMENTO, Stato:=LAVORO, TimerRisparmio:=TRUE);
                    //           END_IF;
                    //   {#endif}  //Fine EFLEX
                    //   {#ifdef DH4010}
                    //       X1,X2,X3,X6:
                    //           IF disp[BLOCCAGGIO_3D].position<>RIPOSO AND disp[BLOCCAGGIO_3D].enable=TRUE THEN
                    //               Set_Disp_Position(Ndisp:=BLOCCAGGIO_3D, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF;  
                    //           IF disp[BLOCCAGGIO_2D].position<>RIPOSO AND disp[BLOCCAGGIO_2D].enable=TRUE THEN
                    //               Set_Disp_Position(Ndisp:=BLOCCAGGIO_2D, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF;  
                    //           IF disp[PINZA_SX].position<>RIPOSO AND disp[PINZA_SX].enable=TRUE THEN
                    //               Set_Disp_Position(Ndisp:=PINZA_SX, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF;  
                    //           IF disp[PINZA_DX].position<>RIPOSO AND disp[PINZA_DX].enable=TRUE THEN
                    //               Set_Disp_Position(Ndisp:=PINZA_DX, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF; 
                    //           IF disp[TAGLIO_SX].position<>RIPOSO AND disp[TAGLIO_SX].enable=TRUE THEN
                    //               Set_Disp_Position(Ndisp:=TAGLIO_SX, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF; 
                    //           IF disp[TAGLIO_DX].position<>RIPOSO AND disp[TAGLIO_DX].enable=TRUE THEN
                    //               Set_Disp_Position(Ndisp:=TAGLIO_DX, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF; 
                    //           IF disp[SMUSSATORE_PINZA].position<>RIPOSO AND disp[SMUSSATORE_PINZA].enable=TRUE THEN
                    //               Set_Disp_Position(Ndisp:=SMUSSATORE_PINZA, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF; 
                    //           IF disp[SMUSSATORE_UTENSILE].position<>RIPOSO AND disp[SMUSSATORE_UTENSILE].enable=TRUE THEN
                    //               Set_Disp_Position(Ndisp:=SMUSSATORE_UTENSILE, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF;
                    //           IF Asse=X1 THEN
                    //               GestioneBloccaggioScomparsa(Bloccaggio:=BLOCCAGGIO_SUPPORTO1_SX, Scomparsa:=SCOMPARSA_SUPPORTO1_SX);
                    //               GestioneBloccaggioScomparsa(Bloccaggio:=BLOCCAGGIO_SUPPORTO2_SX, Scomparsa:=SCOMPARSA_SUPPORTO2_SX);
                    //           END_IF;  
                    //           IF Asse=X2 OR Asse=X3 THEN
                    //               GestioneBloccaggioScomparsa(Bloccaggio:=BLOCCAGGIO_SUPPORTO1_DX, Scomparsa:=SCOMPARSA_SUPPORTO1_DX);
                    //               GestioneBloccaggioScomparsa(Bloccaggio:=BLOCCAGGIO_SUPPORTO2_DX, Scomparsa:=SCOMPARSA_SUPPORTO2_DX);
                    //           END_IF;  
                    //           IF Asse=X6 THEN
                    //               IF TeAspoVersione2()=TRUE THEN
                    //                   WriteDigitalOutput(NumOutput:=OUT_ASPO_VER2_RIPRISTINO_FUNZ_AUTOMATICO, Value:=TRUE);
                    //                   IF ReadDigitalInput(NumInput:=INP_ASPO_VER2_NON_PRONTO, Verifica:=TRUE)=TRUE THEN
                    //                       set_trace(Code:=TRC_ATTESA_ASPO_NON_OK_OFF);
                    //                   ELSE
                    //                       reset_trace(Code:=TRC_ATTESA_ASPO_NON_OK_OFF);
                    //                   END_IF;
                    //                   IF ReadDigitalInput(NumInput:=INP_ASPO_VER2_IN_AUTOMATICO, Verifica:=TRUE)=FALSE AND TestCaricoAspo(Programma:=PROGRAMMA_JOG) THEN
                    //                       set_trace(Code:=TRC_ATTESA_ASPO_IN_MODO_AUTOMATICO);
                    //                   ELSE
                    //                       reset_trace(Code:=TRC_ATTESA_ASPO_IN_MODO_AUTOMATICO);
                    //                   END_IF;
                    //               ELSE
                    //                   reset_trace(Code:=TRC_ATTESA_ASPO_NON_OK_OFF);
                    //                   reset_trace(Code:=TRC_ATTESA_ASPO_IN_MODO_AUTOMATICO);
                    //               END_IF;
                    //               IF i_sys_SenzaTubo=FALSE THEN
                    //                   IF disp[RULLI_TRASCINAMENTO].position<>LAVORO THEN
                    //                       Set_Disp(Ndisp:=RULLI_TRASCINAMENTO, Stato:=LAVORO, TimerRisparmio:=TRUE);
                    //                   END_IF;
                    //               END_IF;
                    //           END_IF;
                    //       Y1,Q1:
                    //           IF Disp[SCOMPARSA_SPALLAMENTO_SX].Position<>RIPOSO THEN
                    //               Set_Disp_Position(Ndisp:=SCOMPARSA_SPALLAMENTO_SX, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //               CURRENT_STATE:=STATE_SET_CONDIZIONE_MOVE_ASSE_WAIT_TIME_500;
                    //               //dret:=_waitTime(T#500ms);
                    //           END_IF;
                    //       Y2,Q2:
                    //           IF Disp[SCOMPARSA_SPALLAMENTO_DX].Position<>RIPOSO THEN
                    //               Set_Disp_Position(Ndisp:=SCOMPARSA_SPALLAMENTO_DX, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //               CURRENT_STATE:=STATE_SET_CONDIZIONE_MOVE_ASSE_WAIT_TIME_500;
                    //               //dret:=_waitTime(T#500ms);
                    //           END_IF;
                    //   {#endif}  //Fine DH4010
                    //   {#ifdef FOUR_RUNNER_PLAUNO} %%%%
                    //       X1:
                    //           //Apertura morsa IPC
                    //           Set_Disp(Ndisp:=PINZA_INTERNA, Stato:=RIPOSO, TimerRisparmio:=TRUE);
                    //           RiposoTagliOrbitali(Attesa:=FALSE);
                    //           //Apertura morsa carrello accompagnamento
                    //           Set_Disp_Position(Ndisp:=PINZA_ACCOMPAG_CH_AP, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           {#ifdef PLAUNO_PLAUNO_LIGHT}
                    //               Set_Disp_Position(Ndisp:=BLOCCAGGIO_TUBO, Value:=RIPOSO, TimerRisparmio:=TRUE);
                    //           {#endif}  //Fine PLAUNO
                    //   {#endif}  //Fine FOUR_RUNNER_PLAUNO
                    //   {#ifdef TWO_RUNNER}
                    //       X1:
                    //           Set_Disp(Ndisp:=PINZA_CARRO_X1, Stato:=RIPOSO, TimerRisparmio:=TRUE);
                    //   {#endif}  //Fine TWO_RUNNER
                    //   {#ifdef THREE_RUNNER}
                    //       X1:
                    //           Set_Disp(Ndisp:=IPC_MORSA_STRAPPO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
                    //           Set_Disp(Ndisp:=PINZA_INTERNA, Stato:=RIPOSO, TimerRisparmio:=TRUE);
                    //   {#endif}  //Fine THREE_RUNNER
                    //   {#ifdef FOUR_RUNNER}
                    //       Q1, V1:
                    //           IF iaddress[INP_SBLOCCO_DX_SX].Port>-1 AND iaddress[INP_SBLOCCO_DX_SX].Bit>-1 THEN
                    //               Set_Disp(Ndisp:=BLOCCO_CANNOTTO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
                    //           END_IF;
                    //   {#endif} //FINE FOUR_RUNNER
                //   {#else}  //Fine ORBITALE
                //       X1: 
                        //   {#ifdef SMART} %%%%
                        //       IF TestBasculanteCarico()=FALSE THEN
                        //           //Basculante non è in posizione di carico si solleva la paletta
                        //           IF (MovimentaPaletta.outputState<>STATE_END) THEN
                        //               MovimentaPaletta(Stato:=LAVORO, Attesa:=FALSE);
                        //           ELSE
                        //               MovimentaPaletta(Stato:=LAVORO, Attesa:=FALSE);
                        //               CURRENT_STATE:=STATE_END;
                        //           END_IF;
                        //       ELSE
                        //           reset_trace(TRC_PALETTA_ALTA);
                        //           CURRENT_STATE:=STATE_END;
                        //       END_IF; 
                        //   {#else}  //Fine SMART
                        //       IF (MovimentaPaletta.outputState<>STATE_END) THEN
                        //           MovimentaPaletta(Stato:=LAVORO, Attesa:=FALSE);
                        //       ELSE
                        //           MovimentaPaletta(Stato:=LAVORO, Attesa:=FALSE);
                        //           CURRENT_STATE:=STATE_END;
                        //       END_IF;
                        //   {#endif}  //Fine NO SMART
                //       Z1:
                //           IF (MovimentaPaletta.outputState<>STATE_END) THEN
                //               MovimentaPaletta(Stato:=LAVORO, Attesa:=FALSE);
                //           ELSE
                //               MovimentaPaletta(Stato:=LAVORO, Attesa:=FALSE);
                //               CURRENT_STATE:=STATE_END;
                //           END_IF;
                // //   {#endif}  //Fine NO ORBITALE
            //   ELSE 
                  
                CURRENT_STATE:=GENERAL_STATE#STATE_END;
            //   END_CASE;
              
        _FB_Set_Condizione_Move_Asse#STATE_SET_CONDIZIONE_MOVE_ASSE_WAIT_TIME_500:
              //%%%%%
              CURRENT_STATE:=GENERAL_STATE#STATE_END;
              
        _FB_Set_Condizione_Move_Asse#STATE_SET_CONDIZIONE_MOVE_ASSE_DH4010_RILASCIO_DIREZIONE:
        //       {#ifdef DH4010} %%%%
        //           IF Asse=X6 AND TeAspoVersione2()=TRUE THEN
        //               IF f_plus.Q=TRUE OR f_minus.Q=TRUE THEN
        //                   //Rilascio + o -
        //                   WriteDigitalOutput(NumOutput:=OUT_ASPO_VER2_RIPRISTINO_FUNZ_AUTOMATICO, Value:=FALSE);
        //                   reset_trace(Code:=TRC_ATTESA_ASPO_NON_OK_OFF);
        //               END_IF;
        //           END_IF;
        //       {#endif};	//Fine DH4010
              
        //   STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
              
      END_CASE;
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK

  FUNCTION_BLOCK FB_SET_Condizione_Move_Dispositivo

    VAR_EXTERNAL CONSTANT
        FIRST_DEVICE                          :INT;
    END_VAR

    VAR_INPUT
      Dispositivo :INT;
    END_VAR
    VAR
      dret        :DINT;
      Differenza  :LREAL;
          
          CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
        //   PosAxeNoWait	:FB_PosAxeNoWait; %%%%
    END_VAR
      VAR_OUTPUT
          OutputState		:INT;
      END_VAR
      CASE CURRENT_STATE OF
          GENERAL_STATE#STATE_INIT:
              CURRENT_STATE:=GENERAL_STATE#STATE_END;
              IF Dispositivo>=FIRST_DEVICE AND Dispositivo <=FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1 THEN
            //       {#ifndef EFLEX}
            //           IF i_sys_OPMODE=MDAMODE OR r_plus.Q=TRUE OR r_minus.Q=TRUE THEN
            //               //In caso di override a 0 si evita di chiamare la funzione che esegue lo stacco del filo
            //               //dalla gola della testina in attesa di fine movimento per evitare di superare i tempi 
            //               //del background
            //               CASE Dispositivo OF
            //                   {#ifdef DH4010}
            //                       SCOMP_PINZA_SX:    
            //                           set_disp_position(Ndisp:=PINZA_SX, Value:=RIPOSO, TimerRisparmio:=TRUE);
            //                       SCOMP_PINZA_DX:    
            //                           set_disp_position(Ndisp:=PINZA_DX, Value:=RIPOSO, TimerRisparmio:=TRUE);
            //                       SCOMP_MANIPOLATORE_3D:
            //                           IF disp[BLOCCAGGIO_3D].position<>RIPOSO THEN
            //                               set_disp(Ndisp:=BLOCCAGGIO_3D, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                           ELSE
            //                               IF (PosAxeNoWait.outputState<>STATE_END) THEN
            //                                   CURRENT_STATE:=STATE_INIT;
            //                                   PosAxeNoWait(Asse:=Z1, Quota:=65, Accelerazione:=50);
            //                               ELSE
            //                                   PosAxeNoWait(Asse:=Z1, Quota:=65, Accelerazione:=50);
            //                                   CURRENT_STATE:=STATE_END;
            //                               END_IF;
            //                           END_IF;
            //                       SCOMPARSA_SUPPORTO1_SX:
            //                           set_disp(Ndisp:=BLOCCAGGIO_SUPPORTO1_SX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                       SCOMPARSA_SUPPORTO2_SX:
            //                           set_disp(Ndisp:=BLOCCAGGIO_SUPPORTO2_SX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                       SCOMPARSA_SUPPORTO1_DX:
            //                           set_disp(Ndisp:=BLOCCAGGIO_SUPPORTO1_DX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                       SCOMPARSA_SUPPORTO2_DX:
            //                           set_disp(Ndisp:=BLOCCAGGIO_SUPPORTO2_DX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                       SCOMP1_SX, SCOMP2_SX:    
            //                           set_disp(Ndisp:=PALPATORE_1_SX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                           set_disp(Ndisp:=PALPATORE_2_SX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                           set_disp(Ndisp:=PALPATORE_3_SX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                           set_disp(Ndisp:=PALPATORE_4_SX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                       SCOMP1_DX, SCOMP2_DX:    
            //                           set_disp(Ndisp:=PALPATORE_1_DX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                           set_disp(Ndisp:=PALPATORE_2_DX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                           set_disp(Ndisp:=PALPATORE_3_DX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                           set_disp(Ndisp:=PALPATORE_4_DX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                   {#else}  //Fine DH4010
            //                       {#ifdef RUNNER}
            //                           {#ifdef IPC_DISPOSITIVO}
            //                               IPC_AVANZAMENTO:;
            //                                   IF r_plus.Q=TRUE THEN
            //                                       LubrTaglioIPC(Programma:=PROGRAMMA_JOG, Stato:=TRUE);
            //                                   ELSE
            //                                       IF r_minus.Q=TRUE THEN
            //                                           LubrTaglioIPC(Programma:=PROGRAMMA_JOG, Stato:=FALSE);
            //                                       END_IF;
            //                                   END_IF;
            //                                   set_disp(Ndisp:=IPC_MOTORE, Stato:=LAVORO, TimerRisparmio:=TRUE);
            //                           {#endif}	//Fine IPC_DISPOSITIVO
            //                           {#ifdef A1_A2}
            //                               LEVA_SCARICO:
            //                                   IF TestBloccaggio4RCPresente()=TRUE THEN
            //                                       set_disp(Ndisp:=BLOCCAGGIO_4RC, Stato:=RIPOSO, TimerRisparmio:=FALSE);
            //                                   END_IF;
            //                           {#endif}	//FINE A1_A2
            //                           {#ifdef TWO_RUNNER}
            //                               PINZA_CARRO_X1:;	//Solo per compilare il codice
            //                           {#endif}	//Fine TWO_RUNNER	//Fine TWO_RUNNER
            //                           {#ifdef THREE_RUNNER}
            //                               IPC_STRAPPO:
            //                                   IF r_minus.Q=TRUE THEN
            //                                       set_disp(Ndisp:=IPC_MORSA_STRAPPO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                                   END_IF;
            //                               IPC_STRAPPO_SCARICO:
            //                                   IF r_minus.Q=TRUE THEN
            //                                       set_disp(Ndisp:=IPC_MORSA_STRAPPO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                                       set_disp(Ndisp:=IPC_STRAPPO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
            //                                   END_IF;
            //                           {#endif}  //Fine THREE_RUNNER
            //                       {#else}  //Fine RUNNER
            //                           PALETTA:
            //                               IF Disp[PALETTA].enable THEN
            //                                   IF (PosAxeNoWait.outputState<>STATE_END) THEN
            //                                       PosAxeNoWait(Asse:=Z1, Quota:=0);
            //                                       CURRENT_STATE:=STATE_INIT;
            //                                   ELSE
            //                                       PosAxeNoWait(Asse:=Z1, Quota:=0);
            //                                       CURRENT_STATE:=STATE_END;
            //                                   END_IF;
            //                               END_IF;
            //                           {#ifndef ELECTMRL}
            //                               PINZA_CARICO:
            //                                   IF disp[PINZA_CARICO].enable=TRUE THEN
            //                                       PompatoreCARM7(Stato:=TRUE);
            //                                   END_IF;
            //                           {#endif} //FINE NO ELECTMRL
            //                       {#endif}  //Fine NO RUNNER
            //                   {#endif}  //Fine NO DH4010
            //               END_CASE;
            //           END_IF;
            //       {#endif}  //Fine NO EFLEX;
            //   ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
      
        GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
      END_CASE;
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK

/**********************************
 * MANAGES THE JOG OF A REAL AXIS *
 **********************************/
  FUNCTION_BLOCK FB_Jog_Asse

    VAR_EXTERNAL
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax; 
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        mcpi_rapid                      : BOOL;
        i_sys_varinc                    : INT;
        i_sys_monitoring                : INT;
        R_Plus		                    :R_TRIG;
        Clk_Minus	                    :BOOL;
        Clk_Plus                        :BOOL;
        R_Rapid                         :R_TRIG; //fronte salita RAPID
        F_Rapid                         :F_TRIG;
        R_Minus                         :R_TRIG;
        F_Plus                          :F_TRIG;
        F_Minus                         :F_TRIG;
        i_sys_PosFinale				    :ARRAY[(_FIRST_AXIS#FIRST_AXIS) .. (_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS -1)] OF LREAL;
        i_sys_ACCESSLEVEL               : INT;
        UsoPLCOpen						:BOOL;
        INC                             :ARRAY[0..5] OF LREAL;
    END_VAR

    VAR_EXTERNAL CONSTANT
        W3                                  :INT;
        ACCESS_MANUTENTORE   	            :SINT;
        TRC_MONITORING_DISABILITATO         :INT;
        TRC_JOG_NON_IN_MONITORING           :INT;  //Macchina non in monitoring o livello di accesso troppo basso
        TRC_SAFETY_JOG_DEVICES       :INT;
        MY_BIG_VALUE_REAL                   :REAL;
        EXTRALIMIT         	                :REAL;
    END_VAR

    VAR_INPUT
      Asse          :INT;  
    END_VAR  

    VAR
      PosAx         : REF_TO TO_SynchronousAxis;
      jogvel        :LREAL;
      corsa         :LREAL;
      jogacc        :LREAL;
      dret          :DINT;
      pMode         :_EnumPositioningMode;
      AppoDirezione :INT;
      i             :INT;
          
          CURRENT_STATE				        :INT:=GENERAL_STATE#STATE_INIT;
          CURRENT_AXIS				        :INT;
          ResetMotionResetAxis              :FB_ResetMotionResetAxis;
          PosAsse							:FB_PosAsse;
    END_VAR

      VAR_OUTPUT
          OutputState					:INT;
      END_VAR
      CASE CURRENT_STATE OF 
          GENERAL_STATE#STATE_INIT:
              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
              IF Asse>=_FIRST_AXIS#FIRST_AXIS AND Asse<=_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 THEN
                  //Per evitare stop della CPU verifica che Asse contenga un numero valido
                  AzzeramentoDistanceTogo(Asse:=Asse);
                  CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_LOOP_RESET_ASSI;
              ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
            _FB_Jog_Asse#STATE_JOG_ASSE_LOOP_RESET_ASSI:
              CURRENT_AXIS:=CURRENT_AXIS+1;
              IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
                PosAx:= AsSynchronousAxisRef(Axes[CURRENT_AXIS].PosAx);
                IF PosAx^.StatusPositioning.Distance<>0 AND 
                  //IF Axes[CURRENT_AXIS].PosAx.poscommand.distancetogo<>0 AND 
          CURRENT_AXIS<>Asse AND posax^.simulation.Mode=UDINT#0 THEN //Axes[CURRENT_AXIS].posax.simulation<>ACTIVE THEN
          ;
            //       {#ifdef SOSTEGNO_ELETTRICO} %%%%
            //           IF CURRENT_AXIS<>S1 THEN
            //   //Serve per azzerare tutti gli assi tranne quello selezionato ed il
            //   //sostegno che si deve poter muovere con X1 e Z1 (tubi non tondi)
            //   CURRENT_STATE:=STATE_JOG_ASSE_RESET_ASSE;
            //           END_IF;
            //       {#else}
            //           CURRENT_STATE:=STATE_JOG_ASSE_RESET_ASSE;
            //       {#endif}
                  END_IF;
              ELSE
                  IF TestVelHoldToRun()=TRUE THEN
                      jogvel:=pm[Asse].ma_Vel_Safety_COMFORT3;
                  ELSE
                      IF mcpi_rapid=FALSE THEN
                          jogvel:=pm[Asse].MA_JOG_VELO*pm[Asse].max_ax_velo/100;
                      ELSE
                          jogvel:=pm[Asse].MA_JOG_VELO_RAPID*pm[Asse].max_ax_velo/100;
                      END_IF;  
                  END_IF;    
                  jogacc:=pm[Asse].MA_MAX_AX_ACCEL*pm[Asse].MA_JOG_ACC/100;
                //   {#ifdef B1_B2} %%%%
                //       //Gli assi B1 e B2 ache con OVERRIDE 1% sono troppo veloci 
                //       IF Asse=B1 OR Asse=B2 THEN
                //           jogvel:=1;
                //       END_IF;
                //   {#endif}  //Fine B1_B2
                //   {#ifdef BLOCCAGGIO_DX}
                //       IF Asse=W3 THEN
                //           AppoDirezione:=SN;
                //       ELSE
                //           AppoDirezione:=SN_SP;
                //       END_IF;
                //   {#else}    //Fine BLOCCAGGIO_DX
                //       AppoDirezione:=SN_SP;
                //   {#endif}    //Fine NO BLOCCAGGIO_DX      
                  
                  Axes[Asse].MC_Axes.move_jog_param.Acceleration :=jogacc;
                  Axes[Asse].MC_Axes.move_jog_param.Deceleration :=jogacc;
                  Axes[Asse].MC_Axes.move_jog_param.Velocity :=jogvel;
                  CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_JOG_PURO_POSITIVO;
              END_IF;
              
        _FB_Jog_Asse#STATE_JOG_ASSE_RESET_ASSE:
              IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
                  ResetMotionResetAxis(Asse:=CURRENT_AXIS);
              ELSE
                  ResetMotionResetAxis(Asse:=CURRENT_AXIS);
                  CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_LOOP_RESET_ASSI;
              END_IF;
      
        _FB_Jog_Asse#STATE_JOG_ASSE_JOG_PURO_POSITIVO:
              IF ((r_plus.Q=TRUE) AND (Clk_minus=FALSE)) OR 
                   ((Clk_plus=TRUE) AND (Clk_minus=FALSE) AND (f_rapid.Q=TRUE)) OR 
                   ((Clk_minus=FALSE) AND (Clk_plus=TRUE) AND (r_rapid.q=TRUE)) THEN
                  IF OK_Condizione_Move_Asse(Asse:=Asse, Direzione:=AppoDirezione) THEN
                      IF i_sys_varinc=0 THEN
                          // è in JOG puro e non incrementale
                          //limite positivo attivo
                          PosAx:= AsSynchronousAxisRef(Axes[asse].PosAx);
                          corsa:=PosAx^.PositionLimits_SW.MaxPosition-EXTRALIMIT;
                          //corsa:=Axes[asse].PosAx.swlimit.plusposition-EXTRALIMIT;
                          pMode:=_EnumPositioningMode#ABSOLUTE;     
                      ELSE
                          IF i_sys_varinc>=1 AND i_sys_varinc<=5 THEN  
                              // valore fisso  
                              corsa := INC[i_sys_varinc];
                              pMode:=_EnumPositioningMode#RELATIVE;
                            //   Axes[Asse].mcJogAxisInfo.Increment:=corsa;
                            //   Axes[Asse].mcJogAxisInfo.ModeInc:=TRUE;
                          ELSE // Quota target
                              // Se la quota supera il fine corsa la limita  
                                PosAx:= AsSynchronousAxisRef(Axes[asse].PosAx);
                                IF pm[asse].MA_JOG_TARGET > PosAx^.PositionLimits_SW.MaxPosition  THEN
                                   pm[asse].MA_JOG_TARGET:=PosAx^.PositionLimits_SW.MaxPosition-EXTRALIMIT;
                                END_IF; 
                            //   IF pm[asse].MA_JOG_TARGET > Axes[asse].PosAx.swlimit.plusposition THEN
                            //       pm[asse].MA_JOG_TARGET:=Axes[asse].PosAx.swlimit.plusposition-EXTRALIMIT;
                            //   END_IF;  
                              // se l^asse è oltre la quota target
                              PosAx:= AsSynchronousAxisRef(Axes[asse].PosAx);
                            //   IF Posax^.actualposition-Axes[Asse].Posax.servosettings.additionalcommandvalue[0]+0.1>pm[asse].MA_JOG_TARGET THEN
                            //       corsa:=PosAx^.PositionLimits_SW.MaxPosition-EXTRALIMIT;
                            //   ELSE
                            //       // valore fisso  
                            //       corsa:=pm[asse].MA_JOG_TARGET;
                            //   END_IF;  
                              pMode:=_EnumPositioningMode#ABSOLUTE;
                          END_IF;
                      END_IF;   
                      IF i_sys_monitoring<>0 OR i_sys_Accesslevel<=ACCESS_MANUTENTORE OR SbloccaMonitoring(Asse:=Asse)=TRUE THEN
                          IF Sbloccamonitoring(Asse:=Asse)=TRUE OR i_sys_monitoring=0 THEN
                              set_trace(Code:=TRC_MONITORING_DISABILITATO);
                          END_IF;
                          //Muove solo se in monitoring o con livello password alto e se l^asse non è oltre il finecorsa positivo
                          PosAx:=AsSynchronousAxisRef(Axes[Asse].Posax);
                          IF Posax^.actualposition+0.01 <  PosAx^.PositionLimits_SW.MaxPosition-EXTRALIMIT THEN //Axes[Asse].PosAx.swlimit.plusposition-EXTRALIMIT THEN
                              TestAsseUtensile(Asse);
                              i_sys_PosFinale[Asse]:=corsa;
                              IF (pMode=_EnumPositioningMode#RELATIVE AND USOPLCOpen) THEN
                                  //Axes[Asse].mcJogAxisInfo.Enable:=TRUE;
                                  Axes[Asse].MC_Axes.move_jog_param.JogForward :=TRUE;
                                  //Axes[Asse].mcJogAxisInfo.Forward:=TRUE;
                              ELSE
                                  CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_POS_ASSE_POSITIVE;
                              END_IF;
                          ELSE
                              CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_JOG_PURO_NEGATIVO;
                          END_IF;  
                      ELSE
                          set_trace(Code:=TRC_JOG_NON_IN_MONITORING);
                          CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_JOG_PURO_NEGATIVO;
                      END_IF;
                  END_IF;
              ELSE	
                  CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_JOG_PURO_NEGATIVO;
              END_IF;
          
        _FB_Jog_Asse#STATE_JOG_ASSE_POS_ASSE_POSITIVE:
              IF (PosAsse.outputState<>GENERAL_STATE#STATE_END) THEN
                  PosAsse(AttesaResetAxes:=TRUE,
                                  axis:=Axes[Asse].PosAx, 
                                  //direction:=BY_VALUE, 
                                  //positioningMode:=pMode, 
                                  position:=corsa,
                                  //velocityType:=DIRECT, 
                                  velocity:=jogvel, 
                                  //positiveAccelType:=DIRECT,
                                  //negativeAccelType:=DIRECT,
                                  positiveAccel:=jogacc,
                                  negativeAccel:=jogacc
                                  //velocityProfile:=TRAPEZOIDAL, 
                                  //blendingMode:=INACTIVE, 
                                  //mergeMode:=IMMEDIATELY,
                                  //nextCommand:=IMMEDIATELY
                                  );
              ELSE
                  PosAsse(AttesaResetAxes:=TRUE,
                                  axis:=Axes[Asse].PosAx, 
                                  //direction:=BY_VALUE, 
                                  //positioningMode:=pMode, 
                                  position:=corsa,
                                  //velocityType:=DIRECT, 
                                  velocity:=jogvel, 
                                  //positiveAccelType:=DIRECT,
                                  //negativeAccelType:=DIRECT,
                                  positiveAccel:=jogacc,
                                  negativeAccel:=jogacc
                                  //velocityProfile:=TRAPEZOIDAL, 
                                  //blendingMode:=INACTIVE, 
                                  //mergeMode:=IMMEDIATELY,
                                  //nextCommand:=IMMEDIATELY
                                  );
                  CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_JOG_PURO_NEGATIVO;
              END_IF;
              
        _FB_Jog_Asse#STATE_JOG_ASSE_JOG_PURO_NEGATIVO:
            //   {::ifdef BLOCCAGGIO_DX} %%%%% non c'e in config di machina
            //       IF Asse=W3 THEN
            //           AppoDirezione:=SN;
            //       ELSE
            //           AppoDirezione:=SN_SP;
            //       END_IF;
            //   {::else}
            //       AppoDirezione:=SN_SP;
            //   {::endif}  
              IF ((r_minus.Q=TRUE) AND (Clk_plus=FALSE)) OR
                   ((Clk_plus=FALSE) AND (Clk_minus=TRUE) AND (r_rapid.Q=TRUE)) OR 
                   ((Clk_minus=TRUE) AND (Clk_plus=FALSE) AND (f_rapid.q=TRUE)) THEN
                  IF OK_Condizione_Move_Asse(Asse:=Asse, Direzione:=AppoDirezione) THEN
                      reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
                      IF i_sys_varinc=0 THEN
                          // è in JOG puro e non incrementale
                          //limite negativo attivo
                          PosAx:= AsSynchronousAxisRef(Axes[asse].PosAx);
                          corsa:=PosAx^.PositionLimits_SW.MinPosition+EXTRALIMIT;
                          //corsa:=Axes[asse].PosAx.swlimit.minusposition+EXTRALIMIT;
                          pMode:=_EnumPositioningMode#ABSOLUTE;     
                      ELSE
                          IF i_sys_varinc>=1 AND i_sys_varinc<=5 THEN  
                              // valore fisso  
                              corsa:=-INC[i_sys_varinc];


                              corsa:=INC_Enum#inc_0;
                              pMode:=_EnumPositioningMode#RELATIVE;     
                            //   Axes[Asse].mcJogAxisInfo.Increment:=corsa; //???????
                            //   Axes[Asse].mcJogAxisInfo.ModeInc:=TRUE;
                          ELSE  // Quota target
                              //Se la quota supera il fine corsa la limita  
                              PosAx:= AsSynchronousAxisRef(Axes[asse].PosAx);
                                IF pm[asse].MA_JOG_TARGET<PosAx^.PositionLimits_SW.MinPosition THEN
                                pm[asse].MA_JOG_TARGET:=PosAx^.PositionLimits_SW.MinPosition+EXTRALIMIT;
                              //IF pm[asse].MA_JOG_TARGET<Axes[asse].PosAx.swlimit.minusposition THEN
                                  //pm[asse].MA_JOG_TARGET:=Axes[asse].PosAx.swlimit.minusposition+EXTRALIMIT;
                              END_IF;  
                              // se l^asse è oltre la quota target
                              PosAx:= AsSynchronousAxisRef(Axes[asse].PosAx);
                            //   IF Posax^.actualposition-Axes[Asse].Posax.servosettings.additionalcommandvalue[0]-0.1<pm[asse].MA_JOG_TARGET THEN ???
                            //       corsa:=PosAx^.PositionLimits_SW.MinPosition +EXTRALIMIT;
                            //   ELSE
                            //       // valore fisso  
                            //       corsa:=pm[asse].MA_JOG_TARGET;
                            //   END_IF;  
                              pMode:=_EnumPositioningMode#ABSOLUTE;
                          END_IF;  
                      END_IF;
                  END_IF;
                  IF i_sys_monitoring<>0 OR i_sys_Accesslevel<=ACCESS_MANUTENTORE OR SbloccaMonitoring(Asse:=Asse)=TRUE THEN
                      IF Sbloccamonitoring(Asse:=Asse)=TRUE OR i_sys_monitoring=0 THEN
                          set_trace(Code:=TRC_MONITORING_DISABILITATO);
                      END_IF;// muove solo se in monitoring o con livello password alto e non è oltre il finecorsa negativo
                      PosAx:= AsSynchronousAxisRef(Axes[asse].PosAx);
                      IF Posax^.actualposition-0.01>PosAx^.PositionLimits_SW.MinPosition+EXTRALIMIT THEN
                          TestAsseUtensile(Asse);
                          i_sys_PosFinale[Asse]:=corsa;
                          IF (pMode=_EnumPositioningMode#RELATIVE AND USOPLCOpen) THEN
                              //Axes[Asse].mcJogAxisInfo.Enable:=TRUE;
                              Axes[Asse].MC_Axes.move_jog_param.JogBackward :=TRUE;
                              //Axes[Asse].mcJogAxisInfo.Backward:=TRUE;
                          ELSE
                              CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_POS_ASSE_NEGATIVE;
                          END_IF;
                      ELSE
                          CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_RILASCIO_PULSANTI;
                      END_IF; 
                  ELSE
                      set_trace(Code:=TRC_JOG_NON_IN_MONITORING);
                      CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_RILASCIO_PULSANTI;
                  END_IF;  
              ELSE	
                  CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_RILASCIO_PULSANTI;
              END_IF;
              
        _FB_Jog_Asse#STATE_JOG_ASSE_POS_ASSE_NEGATIVE:
              IF (PosAsse.outputState<>GENERAL_STATE#STATE_END) THEN
                  PosAsse(AttesaResetAxes:=TRUE,
                                  axis:=Axes[Asse].PosAx, 
                                  //direction:=BY_VALUE, 
                                  positioningMode:=pMode,
                                  position:=corsa, 
                                  //velocityType:=DIRECT, 
                                  velocity:=jogvel,
                                  //positiveAccelType:=DIRECT,
                                  //negativeAccelType:=DIRECT,
                                  positiveAccel:=jogacc,
                                  negativeAccel:=jogacc
                                  //velocityProfile:=TRAPEZOIDAL, 
                                  //blendingMode:=INACTIVE,
                                  //mergeMode:=IMMEDIATELY,
                                  //nextCommand:=IMMEDIATELY
                                  );
              ELSE
                  PosAsse(AttesaResetAxes:=TRUE,
                                  axis:=Axes[Asse].PosAx, 
                                  //direction:=BY_VALUE, 
                                  positioningMode:=pMode,
                                  position:=corsa, 
                                  //velocityType:=DIRECT, 
                                  velocity:=jogvel,
                                  //positiveAccelType:=DIRECT,
                                  //negativeAccelType:=DIRECT,
                                  positiveAccel:=jogacc,
                                  negativeAccel:=jogacc
                                  //velocityProfile:=TRAPEZOIDAL, 
                                  //blendingMode:=INACTIVE,
                                  //mergeMode:=IMMEDIATELY,
                                  //nextCommand:=IMMEDIATELY
                                  );
                  CURRENT_STATE:=_FB_Jog_Asse#STATE_JOG_ASSE_RILASCIO_PULSANTI;
              END_IF;
              
        _FB_Jog_Asse#STATE_JOG_ASSE_RILASCIO_PULSANTI:
              //RILASCIATO TASTO ^+^ oppure RILASCIATO TASTO ^-^
              IF (f_plus.q=TRUE) OR (f_minus.q=TRUE) OR
                   ((r_plus.q=TRUE) AND (Clk_minus=TRUE)) OR
                   ((r_minus.q=TRUE) AND (Clk_plus=TRUE)) THEN
                  i_sys_posfinale[asse]:=-MY_BIG_VALUE_REAL;
                  IF (Axes[Asse].MC_Axes.State_move_jog.Busy) THEN
                    Axes[Asse].MC_Axes.move_jog_param.JogBackward :=FALSE;
                    Axes[Asse].MC_Axes.move_jog_param.JogForward :=FALSE;
                    //   Axes[Asse].mcJogAxisInfo.Backward:=FALSE;
                    //   Axes[Asse].mcJogAxisInfo.Forward:=FALSE;
                      //Axes[Asse].mcJogAxisInfo.Enable:=FALSE;
                  ELSE
                      StopAxis(asse:=asse  , Acceleration:=jogacc (*Deceleration:=jogacc*));
                  END_IF;
                  reset_trace(TRC_JOG_NON_IN_MONITORING);
                  reset_trace(Code:=TRC_MONITORING_DISABILITATO);
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
  
          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
                  
      END_CASE; 
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK
 
  
  FUNCTION_BLOCK FB_MuoviDispositivo

    VAR_EXTERNAL CONSTANT
        FIRST_DEVICE                          :INT;
    END_VAR
    
    VAR_INPUT
      Dispositivo    :INT;
      Stato          :INT;
    END_VAR
    VAR
      Esito            :BOOL;
      AppoStato        :INT;
      dret         		 :DINT;
          
          
          CURRENT_STATE				:INT:=GENERAL_STATE#STATE_INIT;
        //   {#ifndef ORBITALE}
        //       PinzaCoccodrillo	:FB_PinzaCoccodrillo;
        //   {#endif};
    END_VAR
      VAR_OUTPUT
          OutputState					:INT;
      END_VAR
      CASE CURRENT_STATE OF
          GENERAL_STATE#STATE_INIT:
              IF Dispositivo>=FIRST_DEVICE AND Dispositivo<=FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1 THEN
                //   {#ifdef ORBITALE}
                //       IF Disp[Dispositivo].TipoOut=DISP_OUT_V_C THEN
                //           AppoStato:=Stato;
                //           {#ifdef DH4010}
                //               IF Dispositivo=PINZA_SX OR Dispositivo=PINZA_DX THEN
                //                   //Gestione chiusura pinza su 2 posizioni. Se si parte da INTERMEDIO si può attivare LAVORO,
                //                   //altrimenti si attiva INTERMEDIO
                //                   IF Stato=LAVORO OR Stato=INTERMEDIO_PINZA THEN
                //                       IF Disp[Dispositivo].stato=INTERMEDIO_PINZA THEN
                //                           AppoStato:=LAVORO;
                //                       ELSE
                //                           AppoStato:=INTERMEDIO_PINZA;
                //                       END_IF;
                //                   END_IF;
                //                   IF Stato=RIPOSO OR Stato=INTERMEDIO_PINZA THEN
                //                       IF Disp[Dispositivo].stato=INTERMEDIO_PINZA THEN
                //                           AppoStato:=RIPOSO;
                //                       ELSE
                //                           AppoStato:=INTERMEDIO_PINZA;
                //                       END_IF;
                //                   END_IF;
                //               END_IF;    
                //           {#endif}	//Fine DH4010
                //           Set_Disp_Position(Ndisp:=Dispositivo, Value:=AppoStato, TimerRisparmio:=TRUE);
                //           CURRENT_STATE:=GENERAL_STATE#STATE_END;
                //       ELSE
                //           set_disp(Ndisp:=Dispositivo, Stato:=Stato, TimerRisparmio:=TRUE);
                //           CURRENT_STATE:=GENERAL_STATE#STATE_END;
                //       END_IF;
                //   {#else}  //Fine ORBITALE
                //       IF Dispositivo=PINZA THEN
                //           IF cfg.UsoPinzaCoccoPneumatica=TRUE AND programmi[PROGRAMMA_JOG].PinzaCoccodrillo3Posizioni=TRUE THEN
                //               IF Stato=RIPOSO AND  Disp[PINZA].position=LAVORO THEN
                //                   AppoStato:=PREAPERTO;
                //               ELSE
                //                   AppoStato:=Stato;
                //               END_IF;  
                //               CURRENT_STATE:=STATE_MUOVI_DISPOSITIVO_PINZA_COCCODRILLO;
                //           ELSE
                //               WriteDigitalOutput(NumOutput:=OUT_PREAPERTURA_PINZA_COCCODRILLO, Value:=FALSE);
                //               WriteDigitalOutput(NumOutput:=OUT_RESET_PINZA_COCCODRILLO_PREAPERTA, Value:=TRUE);
                //               set_disp(Ndisp:=Dispositivo, Stato:=Stato, TimerRisparmio:=TRUE);
                //               CURRENT_STATE:=GENERAL_STATE#STATE_END;
                //           END_IF;
                //       ELSE
                //           set_disp(Ndisp:=Dispositivo, Stato:=Stato, TimerRisparmio:=TRUE);
                //           CURRENT_STATE:=GENERAL_STATE#STATE_END;
                //       END_IF;
                //   {#endif}  //Fine NO ORBITALE			
            //   ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
        //   STATE_MUOVI_DISPOSITIVO_PINZA_COCCODRILLO:
        //   ;
            //   {#ifndef ORBITALE} %%%%
            //       IF PinzaCoccodrillo.OutputState<>GENERAL_STATE#STATE_END THEN
            //           PinzaCoccodrillo(Stato:=AppoStato, 
            //                                            FlagStopPoint:=FALSE,
            //                                            Impostazione:=TRUE, 
            //                                            Attesa:=FALSE,
            //                                            Trace:=FALSE);				
            //       ELSE
            //           Esito:=PinzaCoccodrillo.ReturnValue;
            //           PinzaCoccodrillo();
            //           Disp[PINZA].position:=AppoStato;
            //           Disp[PINZA].stato:=AppoStato;
            //           CURRENT_STATE:=GENERAL_STATE#STATE_END;
            //       END_IF;
            //   {#endif};
              
          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
      
      END_CASE;
      OutputState:=CURRENT_STATE;  
  END_FUNCTION_BLOCK

  FUNCTION_BLOCK FB_PosizionaSostegno
    VAR_INPUT
      Posizione :LREAL;
      Attesa    :BOOL:=TRUE;
    END_VAR
    VAR
      dret      :DINT;
      PosS1     :LREAL;
      OffsetS1  :LREAL;
      Programma :DINT;
          
          CURRENT_STATE				:INT:=GENERAL_STATE#STATE_INIT;
        //   Wait_Disp_Position	:FB_Wait_Disp_Position; %%%%
        //   PosAxeNoWait				:FB_PosAxeNoWait;
        //   WaitAxe							:FB_WaitAxe;
          EnableAxisBLM				:FB_EnableAxisBLM;
    END_VAR
      VAR_OUTPUT
          OutputState					:INT;
      END_VAR
      ;
    //   CASE CURRENT_STATE OF %%%%
    //       GENERAL_STATE#STATE_INIT:
    //           {#ifndef ORBITALE}
    //               {#ifdef SOSTEGNO_ELETTRICO}
    //                   //Impostazione dell^indice del pezzo attivo, se non siamo in AUTO il pezzo
    //                   //attivo da dove prendere i dati è 1
    //                   IF i_sys_OPMODE=AUTOMODE THEN
    //                       Programma:=i_sys_current_prog;
    //                   ELSE
    //                       Programma:=PROGRAMMA_JOG;
    //                   END_IF;
    //                   IF Assi[S1].PosAx.positioningState.homed=YES AND pm[S1].ma_stato_index=2 THEN
    //                       IF Programmi[Programma].diametro_tubo_sezione=TONDO THEN
    //                           //Gestione tubo tondo, la posizione S1 è calcolata direttamente da VGP
    //                           OffsetS1:=BlmMxToolsOffPezzo[(S1)].PosChiuso-
    //                                               BlmMxToolsOffPezzo[(S1)].Offset;
    //                           IF ABS(Posizione-RIPOSO)<=pm[S1].ma_tolerance OR
    //                                ABS(Posizione-BlmMxToolsOffPezzo[(S1)].PosAperto)<=pm[S1].ma_tolerance THEN
    //                               //Posizione bassa sostegno non si applica l^offset
    //                               PosS1:=BlmMxToolsOffPezzo[(S1)].PosAperto;
    //                           ELSE
    //                               IF ABS(Posizione-LAVORO)<=pm[S1].ma_tolerance THEN
    //                                   PosS1:=BlmMxToolsOffPezzo[(S1)].offset;
    //                               ELSE
    //                                   PosS1:=Posizione-OffsetS1;
    //                               END_IF;
    //                           END_IF;
    //                       ELSE
    //                           //Tubi non tondi la posizione di S1 dipende dalla posizione di Z1
    //                           IF ABS(Posizione-RIPOSO)<=pm[S1].ma_tolerance OR
    //                                ABS(Posizione-BlmMxToolsOffPezzo[(S1)].PosAperto)<=pm[S1].ma_tolerance THEN
    //                               //Posizione bassa sostegno non si applica l^offset
    //                               PosS1:=BlmMxToolsOffPezzo[(S1)].PosAperto;
    //                           ELSE
    //                               //Richiesta posizione alta occorre calcolarla in funzione di Z1
    //                               PosS1:=CalcS1daZ1(PosZ1:=Assi[Z1].PosAx.positioningState.actualPosition+
    //                                                                                Programmi[Programma].OffsetZ1RicSaldatura,
    //                                                                   Programma:=Programma,
    //                                                                   Sezione:=Programmi[Programma].diametro_tubo_sezione);
    //                           END_IF;
    //                       END_IF;
    //                       IF Attesa=FALSE AND Assi[S1].PosAx.control<>ACTIVE  THEN
    //                           CURRENT_STATE:=STATE_POSIZIONA_SOSTEGNO_ENABLE_SOSTEGNO;
    //                       ELSE
    //                           CURRENT_STATE:=STATE_POSIZIONA_SOSTEGNO_POS_SOSTEGNO_ELETTRICO;
    //                       END_IF;
    //                   ELSE
    //                       CURRENT_STATE:=GENERAL_STATE#STATE_END;
    //                   END_IF;
    //               {#else}
    //                   Set_disp_Position(Ndisp:=SUPPORT_TIRANT, Value:=Posizione, TimerRisparmio:=TRUE);
    //                   IF Attesa=TRUE THEN
    //                       CURRENT_STATE:=STATE_POSIZIONA_SOSTEGNO_WAIT_SOSTEGNO_PNEUMATICO;
    //                   ELSE
    //                       CURRENT_STATE:=GENERAL_STATE#STATE_END;
    //                   END_IF;
    //               {#endif}
    //           {#else}
    //               CURRENT_STATE:=GENERAL_STATE#STATE_END;
    //           {#endif};
              
    //       STATE_POSIZIONA_SOSTEGNO_POS_SOSTEGNO_ELETTRICO:
    //           {#ifdef SOSTEGNO_ELETTRICO}
    //               IF (PosAxeNoWait.outputState<>GENERAL_STATE#STATE_END) THEN
    //                   PosAxeNoWait(Quota:=PosS1, Asse:=S1);
    //               ELSE
    //                   PosAxeNoWait(Quota:=PosS1, Asse:=S1);
    //                   IF Attesa THEN
    //                       CURRENT_STATE:=STATE_POSIZIONA_SOSTEGNO_WAIT_SOSTEGNO_ELETTRICO;
    //                   ELSE
    //                       CURRENT_STATE:=GENERAL_STATE#STATE_END;
    //                   END_IF;
    //               END_IF;
    //           {#endif};
              
    //       STATE_POSIZIONA_SOSTEGNO_WAIT_SOSTEGNO_ELETTRICO:
    //           {#ifdef SOSTEGNO_ELETTRICO}
    //               IF (WaitAxe.outputState<>GENERAL_STATE#STATE_END) THEN
    //                   WaitAxe(Quota:=PosS1, Asse:=S1);
    //               ELSE
    //                   WaitAxe(Quota:=PosS1, Asse:=S1);
    //                   CURRENT_STATE:=GENERAL_STATE#STATE_END;
    //               END_IF;
    //           {#endif};
              
    //       STATE_POSIZIONA_SOSTEGNO_ENABLE_SOSTEGNO:
    //           {#ifdef SOSTEGNO_ELETTRICO}
    //               IF (EnableAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
    //                   EnableAxisBLM(Asse:=assi[S1].PosAx, 
    //                                               NumeroAsse:=S1, 
    //                                               ModoAttesa:=IMMEDIATELY);
    //               ELSE
    //                   EnableAxisBLM(Asse:=assi[S1].PosAx, 
    //                                               NumeroAsse:=S1, 
    //                                               ModoAttesa:=IMMEDIATELY);
    //                   CURRENT_STATE:=GENERAL_STATE#STATE_END;
    //               END_IF;
    //           {#endif};
              
    //       STATE_POSIZIONA_SOSTEGNO_WAIT_SOSTEGNO_PNEUMATICO:
    //           {#ifndef SOSTEGNO_ELETTRICO}
    //               IF (Wait_Disp_Position.outputState<>GENERAL_STATE#STATE_END) THEN
    //                   Wait_Disp_Position(Device:=SUPPORT_TIRANT, Position:=LREAL_TO_INT(Posizione));
    //               ELSE
    //                   CURRENT_STATE:=GENERAL_STATE#STATE_END;
    //               END_IF;
    //           {#endif};
              
    //       GENERAL_STATE#STATE_END:
    //           CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
          
    //   END_CASE;
    //   OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK

/****************************************************************************************
 * DISABLES AXES AND PERFORMS A PUSH_ERROR IF THE MESSAGE CODE TO BE DISPLAYED IS GIVEN *
 ****************************************************************************************/

FUNCTION_BLOCK FB_SpegneAssi

    VAR_EXTERNAL
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    END_VAR

    VAR_EXTERNAL CONSTANT
        INACTIVE_DEACTIVATABLE_AXIS         :SINT;
    END_VAR

    VAR_INPUT
      Messaggio  	:DINT;
      Arresto    	:BOOL;
    END_VAR  
    VAR  
      PosAx         :REF_TO TO_SynchronousAxis;
      i          	:INT;
      dret      	:DINT;
          
          CURRENT_STATE			:INT:=GENERAL_STATE#STATE_INIT;
          CURRENT_AXIS			:INT;
          DisableAssi				:FB_DisableAxes;
    END_VAR
      VAR_OUTPUT
          OutputState				:INT;
      END_VAR
      CASE CURRENT_STATE OF
          GENERAL_STATE#STATE_INIT:	
              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
              IF Messaggio>0 THEN
                  IF Arresto=TRUE THEN
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                      dret:=Push_error(code:=Messaggio, Arresto:=TRUE);
                  ELSE
                      CURRENT_STATE:=GENERAL_STATE#STATE_END;
                      dret:=Push_error(code:=Messaggio, Arresto:=FALSE);
                  END_IF;
              END_IF;
              IF CURRENT_STATE<>GENERAL_STATE#STATE_END THEN
                  CURRENT_STATE:=_FB_SpegneAssi#STATE_SPEGNE_ASSI_LOOP_ASSI;
              END_IF;
              
            _FB_SpegneAssi#STATE_SPEGNE_ASSI_LOOP_ASSI:
              CURRENT_AXIS:=CURRENT_AXIS+1;
              IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
                  IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE AND pm[CURRENT_AXIS].ma_asse_installato<>INACTIVE_DEACTIVATABLE_AXIS THEN
                    PosAx:= AsSynchronousAxisRef(Axes[CURRENT_AXIS].PosAx);
                    IF PosAx^.StatusWord.%X7 THEN
                    //IF Axes[CURRENT_AXIS].PosAx.motionstatedata.motionstate=STANDSTILL THEN
                          CURRENT_STATE:=_FB_SpegneAssi#STATE_SPEGNE_ASSI_DISABLE_AXIS;  
                      END_IF;
                  END_IF;			
              ELSE
                  CURRENT_STATE:=GENERAL_STATE#STATE_END;
              END_IF;
              
            _FB_SpegneAssi#STATE_SPEGNE_ASSI_DISABLE_AXIS:
              IF DisableAssi.OutputState<>GENERAL_STATE#STATE_END THEN
                  DisableAssi(Asse:=CURRENT_AXIS(* , Regolatore:=ACTIVE, ProssimoCmd:=IMMEDIATELY*));  
              ELSE
                  DisableAssi(Asse:=CURRENT_AXIS(* , Regolatore:=ACTIVE, ProssimoCmd:=IMMEDIATELY*));  
                  CURRENT_STATE:=_FB_SpegneAssi#STATE_SPEGNE_ASSI_LOOP_ASSI;
              END_IF;
              
          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
      
      END_CASE;
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK
  
  FUNCTION SbloccaMonitoring:BOOL

    VAR_EXTERNAL CONSTANT
        ACCESS_SAT           	:SINT;
    END_VAR

    VAR_EXTERNAL
        PagTabCorrezione                        :BOOL;
        pm                                      : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        i_sys_ACCESSLEVEL                       : INT;
        i_sys_monitoring                        : INT;
        access                                  :Access_Level;
    END_VAR

    VAR_INPUT
      Asse  :INT;
    END_VAR
    SbloccaMonitoring:=FALSE;
    IF PagTabCorrezione=TRUE THEN
      SbloccaMonitoring:=TRUE;
    ELSE
      IF Asse>=_FIRST_AXIS#FIRST_AXIS AND Asse<=_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 THEN
        IF pm[Asse].ma_AsseInVGP=FALSE THEN
          //Asse non presente in VGP, non può quindi indicarne i limiti software
          SbloccaMonitoring:=TRUE;
        END_IF;
       
      END_IF;
      IF SbloccaMonitoring=FALSE THEN
        IF TestAsseBloccaggio(Asse)=TRUE OR TestAsseSpallamento(Asse)=TRUE THEN
          //Per gli assi bloccaggio e spallamento lo sbloccaggio del monitoring
          //può avvenire a partire da password SAT

        {::ifdef MCP_1900} 
          SbloccaMonitoring:=(i_sys_accesslevel<=ACCESS_SAT AND i_sys_monitoring=0) ;
        {::else};
            SbloccaMonitoring:=(i_sys_accesslevel<=ACCESS_SAT AND i_sys_monitoring=0) 

                            OR ((NOT MCPI_FEED_STOP AND NOT MCPI_SPINDLE_STOP) AND (i_sys_accesslevel<=access.jog_in_monitoring));
        {::endif}; 

        ELSE
          SbloccaMonitoring:=(i_sys_accesslevel<=access.jog_libero AND i_sys_monitoring=0)
                               {::ifndef MCP_1900}
                              OR ((NOT MCPI_FEED_STOP AND NOT MCPI_SPINDLE_STOP) AND (i_sys_accesslevel<=access.jog_in_monitoring));
                               {::endif};
        END_IF;
      END_IF;
    END_IF;
  END_FUNCTION  

  FUNCTION OK_Condizione_Move_Asse:BOOL

    VAR_EXTERNAL CONSTANT
            TRC_SAFETY_JOG_DEVICES               :INT;
            PROGRAMMA_JOG                      	        :SINT;
            TRC_MACCHINA_PEZZO_DXSX                     :INT;
            MAXTOOLSOFFPEZZO                            :INT;
            W3 : INT;  
            X1 :INT;   
            U4 :INT;
            Z1 :INT;
            Y1 :INT;
            V1 :INT; 
            L1 :INT;
            ERR_SLITTA_MOBILE_NON_A_ZERO                :DINT;
            FIRST_DEVICE                                :INT;
            PALETTA                                     :INT;
            ERR_BRACCIO_SOTTO_90                        :DINT;
            MAXDIGITALINPUT                             :INT;
            ACCESS_MANUTENTORE   	                    :SINT;
            INP_AL5_TRASLATORE_INDIETRO                 :INT;
            ERR_L1_TRASLATORE                           :DINT;
            ERR_TASSELLO_BLOCCAGGIO_DXSX                :DINT;
             

    END_VAR

    VAR_EXTERNAL
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
        Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
        i_sys_current_prog	            :INT;
        i_sys_OPMODE                    : INT;
        BlmMxToolsOffPezzo              :ARRAY[0..MAXTOOLSOFFPEZZO-1] OF RigaUtDispositivo;
        Clk_Minus	                    :BOOL;
        mcpi_minus                      : BOOL;
        mcpi_plus                       : BOOL;
        Clk_Plus                        :BOOL;
        IAddress                        : ARRAY [0..MAXDIGITALINPUT] OF Digital_Input;
        i_sys_ACCESSLEVEL               : INT;
    END_VAR

    VAR_INPUT
      Asse      :INT;
      Direzione :INT;
    END_VAR

    VAR
      PosAx     : REF_TO TO_SynchronousAxis;
      iret      :INT;
      W3_Aperto :LREAL;
      TipoAl    :SINT;
      Programma :INT;
      Esito     :BOOL;
    END_VAR

      IF Asse<_FIRST_AXIS#FIRST_AXIS OR Asse>_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 THEN
          OK_Condizione_Move_Asse:=FALSE;
      ELSE
            PosAx:= AsSynchronousAxisRef(Axes[Asse].posax);
          IF posax^.StatusWord.%X2 THEN
              OK_Condizione_Move_Asse:=FALSE;
          ELSE
              IF i_sys_OpMode=PI_COMMANDS_CODE#AUTOMODE THEN
                  Programma:=I_SYS_Current_Prog;
              ELSE
                  Programma:=PROGRAMMA_JOG;
              END_IF;
              OK_Condizione_Move_Asse:=TRUE;
            //   {
            //   #ifdef EFLEX_DH4010 %%%%
            //   }
            //       {
            //       #ifdef EFLEX
            //       }
            //           CASE Asse OF
            //               X1:
            //                   IF i_sys_SenzaTubo=FALSE AND disp[RULLI_TRASCINAMENTO].position<>LAVORO THEN
            //                       OK_Condizione_Move_Asse:=FALSE;
            //                   END_IF;
            //                   IF ABS(Assi[W1].PosAx.positioningState.actualPosition)>0.1 THEN
            //                       //Taglio non a riposo
            //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=W1);
            //                   END_IF;
            //               Q1: 
            //                   IF (disp[TESTINA_1].position<>LAVORO OR disp[TESTINA_2].position<>LAVORO) AND mcpi_spindle_stop=TRUE THEN
            //                       OK_Condizione_Move_Asse:=FALSE;
            //                       iret:=Push_error(Code:=ERR_TESTINE_NO_BASSE);
            //                   END_IF;
            //           END_CASE;
            //       {
            //       #endif  //Fine EFLEX
            //       }
            //       {
            //       #ifdef DH4010
            //       }
            //           CASE Asse OF
            //               X1,X2,X3,X6:
            //                   IF Asse=X6 AND i_sys_SenzaTubo=FALSE AND disp[RULLI_TRASCINAMENTO].position<>LAVORO THEN
            //                       OK_Condizione_Move_Asse:=FALSE;
            //                       Set_Disp(Ndisp:=RULLI_TRASCINAMENTO, Stato:=LAVORO, TimerRisparmio:=TRUE);
            //                   END_IF;
            //                   OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=BLOCCAGGIO_3D, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
            //                   OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=BLOCCAGGIO_2D, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
            //                   OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=PINZA_SX, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
            //                   OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=PINZA_DX, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
            //                   OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=TAGLIO_SX, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
            //                   OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=TAGLIO_DX, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
            //                   OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=SMUSSATORE_PINZA, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
            //                   OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=SMUSSATORE_UTENSILE, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
            //                   IF Asse=X1 THEN
            //                       IF TestCoerenzaDispositivo(Dispositivo:=SCOMPARSA_SUPPORTO1_SX)=TRUE THEN
            //                           OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=SCOMPARSA_SUPPORTO1_SX, Stato:=LAVORO, StatoOld:=OK_Condizione_Move_Asse);
            //                       ELSE
            //                           OK_Condizione_Move_Asse:=FALSE;
            //                           iret:=Push_error(Code:=ERR_CONFIGURAZIONE_SOSTEGNI_DH4010, Asse:=SCOMPARSA_SUPPORTO1_SX);
            //                       END_IF;
            //                       IF TestCoerenzaDispositivo(Dispositivo:=SCOMPARSA_SUPPORTO2_SX)=TRUE THEN
            //                           OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=SCOMPARSA_SUPPORTO2_SX, Stato:=LAVORO, StatoOld:=OK_Condizione_Move_Asse);
            //                       ELSE
            //                           OK_Condizione_Move_Asse:=FALSE;
            //                           iret:=Push_error(Code:=ERR_CONFIGURAZIONE_SOSTEGNI_DH4010, Asse:=SCOMPARSA_SUPPORTO2_SX);
            //                       END_IF;
            //                   END_IF;
            //                   IF Asse=X2 OR Asse=X3 THEN
            //                       IF TestCoerenzaDispositivo(Dispositivo:=SCOMPARSA_SUPPORTO1_DX)=TRUE THEN
            //                           OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=SCOMPARSA_SUPPORTO1_DX, Stato:=LAVORO, StatoOld:=OK_Condizione_Move_Asse);
            //                       ELSE
            //                           OK_Condizione_Move_Asse:=FALSE;
            //                           iret:=Push_error(Code:=ERR_CONFIGURAZIONE_SOSTEGNI_DH4010, Asse:=SCOMPARSA_SUPPORTO1_DX);
            //                       END_IF;
            //                       IF TestCoerenzaDispositivo(Dispositivo:=SCOMPARSA_SUPPORTO2_DX)=TRUE THEN
            //                           OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=SCOMPARSA_SUPPORTO2_DX, Stato:=LAVORO, StatoOld:=OK_Condizione_Move_Asse);
            //                       ELSE
            //                           OK_Condizione_Move_Asse:=FALSE;
            //                           iret:=Push_error(Code:=ERR_CONFIGURAZIONE_SOSTEGNI_DH4010, Asse:=SCOMPARSA_SUPPORTO2_DX);
            //                       END_IF;
            //                   END_IF;
            //                   IF Asse=X6 THEN
            //                       IF TeAspoVersione2()=TRUE AND (r_plus.Q=TRUE OR r_minus.Q=TRUE) THEN
            //                           //Verifica solo sul fronte di salita dei tasti più o meno
            //                           OK_Condizione_Move_Asse:=NOT ReadDigitalInput(NumInput:=INP_ASPO_VER2_NON_PRONTO, Verifica:=TRUE) AND
            //                                                                            ReadDigitalInput(NumInput:=INP_ASPO_VER2_IN_AUTOMATICO, Verifica:=TRUE); 
            //                       END_IF;
            //                   END_IF;
            //               Q1:
            //                   IF i_sys_monitoring=0 THEN
            //                       IF Assi[Y1].PosAx.positioningState.actualPosition>180 OR Assi[Y1].PosAx.positioningState.actualPosition<0 THEN
            //                           OK_Condizione_Move_Asse:=FALSE;
            //                           set_trace(Code:=TRC_JOG_NON_IN_MONITORING);
            //                       END_IF;
            //                   END_IF;
            //                   IF (disp[TESTINA_1_SX].position<>LAVORO OR disp[TESTINA_2_SX].position<>LAVORO) AND mcpi_spindle_stop=TRUE THEN
            //                       OK_Condizione_Move_Asse:=FALSE;
            //                       iret:=Push_error(Code:=ERR_TESTINE_NO_BASSE);
            //                   END_IF;
            //               Q2: 
            //                   IF i_sys_monitoring=0 THEN
            //                       IF Assi[Y2].PosAx.positioningState.actualPosition>180 OR Assi[Y2].PosAx.positioningState.actualPosition<0 THEN
            //                           OK_Condizione_Move_Asse:=FALSE;
            //                           set_trace(Code:=TRC_JOG_NON_IN_MONITORING);
            //                       END_IF;
            //                   END_IF;
            //                   IF (disp[TESTINA_1_DX].position<>LAVORO OR disp[TESTINA_2_DX].position<>LAVORO) AND mcpi_spindle_stop=TRUE THEN
            //                       OK_Condizione_Move_Asse:=FALSE;
            //                       iret:=Push_error(Code:=ERR_TESTINE_NO_BASSE);
            //                   END_IF;
            //               Y1,Y2:
            //                   IF i_sys_monitoring=0 THEN
            //                       OK_Condizione_Move_Asse:=FALSE;
            //                       set_trace(Code:=TRC_JOG_NON_IN_MONITORING);
            //                   END_IF;
            //           END_CASE;
            //       {
            //       #endif  //Fine DH4010
            //       }
            //   {
            //   #else //Fine EFLEX_DH4010
            //   }
                  IF Utensili_DX_SX()=FALSE THEN 
                      OK_Condizione_Move_Asse:=FALSE;
                      set_trace(Code:=TRC_MACCHINA_PEZZO_DXSX);
                  ELSE
                      IF ShadowMotorCheck()>=_FIRST_AXIS#FIRST_AXIS THEN
                          OK_Condizione_Move_Asse:=FALSE;
                      END_IF;
                  END_IF;
                  {::ifdef BLOCCAGGIO_DX}
                      //Impostazione quota di morsa destra aperta
                      W3_Aperto:=BlmMxToolsOffPezzo[(W3)].PosAperto-5;
                  {::endif}
                //   {
                //   #ifdef ORBITALE %%%%
                //   }
                //       {
                //       #ifdef RUNNER
                //       }
                //           CASE Asse OF
                //               {
                //               #ifdef FOUR_RUNNER_PLAUNO
                //               }
                //                   X1:
                //                       OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=PINZA_INTERNA, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                       {
                //                       #ifdef B1_B2
                //                       }
                //                           IF ABS(Assi[B1].PosAx.positioningState.actualPosition)>5 AND pm[B1].ma_asse_abilitato=TRUE THEN
                //                               OK_Condizione_Move_Asse:=FALSE;
                //                               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=FOUR_RIPC);
                //                           ELSE
                //                               IF ABS(Assi[B2].PosAx.positioningState.actualPosition)>5 AND pm[B2].ma_asse_abilitato=TRUE THEN
                //                                   OK_Condizione_Move_Asse:=FALSE;
                //                                   set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=FOUR_RIPC);
                //                               END_IF;
                //                           END_IF;
                //                       {
                //                       #endif	//Fine B1_B2
                //                       }
                //                       {
                //                       #ifdef PLAUNO_PLAUNO_LIGHT
                //                       }
                //                           OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=BLOCCAGGIO_TUBO, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                       {
                //                       #endif	//Fine PLAUNO_PLAUNO_LIGHT
                //                       }
                //                       {
                //                       #ifdef IPC_DISPOSITIVO
                //                       }
                //                           OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=IPC_AVANZAMENTO, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                           IF cfg._4RUNNER_Mo1613=TRUE THEN
                //                               OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=IPC_LIGHT, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                           END_IF;
                //                       {
                //                       #endif	//Fine IPC_DISPOSITIVO
                //                       }
                //                       OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=PINZA_ACCOMPAG_CH_AP, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                       OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=FORATURA_MORSA, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                   V1, Q1:
                //                       OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=FORATURA_MORSA, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                       {
                //                       #ifdef FOUR_RUNNER
                //                       }
                //                           IF iaddress[INP_SBLOCCO_DX_SX].Port>-1 AND iaddress[INP_SBLOCCO_DX_SX].Bit>-1 THEN
                //                               IF Disp[BLOCCO_CANNOTTO].Position<>RIPOSO AND RunnerDXSX()=TRUE THEN
                //                                   Ok_condizione_move_Asse:=FALSE;
                //                                   Set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCO_CANNOTTO);
                //                               END_IF;
                //                           END_IF;
                //                       {
                //                       #endif //FINE FOUR_RUNNER
                //                       }
                //               {
                //               #endif	//Fine FOUR_RUNNER_PLAUNO
                //               }
                //               {
                //               #ifdef THREE_RUNNER
                //               }
                //                   X1:
                //                       OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=IPC_MORSA_STRAPPO, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                       OK_Condizione_Move_Asse:=TestCondizione(Dispositivo:=PINZA_INTERNA, Stato:=RIPOSO, StatoOld:=OK_Condizione_Move_Asse);
                //                       {
                //                       #ifdef B1_B2
                //                       }
                //                           IF ABS(Assi[B1].PosAx.positioningState.actualPosition)>5 THEN
                //                               OK_Condizione_Move_Asse:=FALSE;
                //                               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=THREE_RIPC);
                //                           ELSE
                //                               IF ABS(Assi[B2].PosAx.positioningState.actualPosition)>5 THEN
                //                                   OK_Condizione_Move_Asse:=FALSE;
                //                                   set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=THREE_RIPC);
                //                               END_IF;
                //                           END_IF;
                //                       {
                //                       #endif	//Fine B1_B2
                //                       }
                //               {
                //               #endif	//Fine THREE_RUNNER
                //               }
                //           U1:
                //               IF Clk_plus=TRUE THEN
                //                   TestY1Safety();
                //               END_IF;
                //               {
                //               #ifndef PLAUNO_LIGHT
                //               }
                //                   IF (cfg._4RUNNER_4RForm_LevaScaricoScivolo) AND (disp[LEVA_SCARICO].Stato=LAVORO) THEN 
                //                       Ok_Condizione_move_Asse:=FALSE;
                //                       iret:=Push_Error(Code:=ERR_MOVIMENTAZIONE_LEVA_SCARICO, Arresto:=TRUE);
                //                   END_IF;
                //               {
                //               #endif //FINE NO PLAUNO_LIGHT
                //               }
                //           W2:
                //               IF VerificaU2AllineataW1()=FALSE THEN
                //                   //Lo spintore per muoversi verso la morsa deve essere allineato, in caso contrario
                //                   //azzero il numero del dispositivo per fermarlo
                //                   OK_Condizione_Move_Asse:=FALSE;
                //                   set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=U2);
                //               END_IF;
                //           U2:
                //               IF ABS(Assi[W2].PosAx.positioningState.actualPosition-pm[W2].Ma_Pos_Limit_Plus)>pm[W2].ma_tolerance THEN
                //                   //Spintore non indietro, non si può muovere U2
                //                   OK_Condizione_Move_Asse:=FALSE;
                //                   set_trace(Code:=trc_SICUREZZA_JOG_DISPOSITIVI, Asse:=W2);
                //               END_IF;
                //               IF Clk_plus=TRUE THEN
                //                   TestY1Safety();
                //               END_IF;
                //               {
                //               #ifdef A1_A2
                //               }
                //                   IF TestBloccaggio4RCPresente()=TRUE AND Disp[BLOCCAGGIO_4RC].Position<>RIPOSO THEN 
                //                       OK_Condizione_Move_Asse:=FALSE;
                //                       set_trace(Code:=trc_SICUREZZA_JOG_DISPOSITIVI, Asse:=BLOCCAGGIO_4RC);
                //                   END_IF;
                //               {
                //               #endif	//Fine A1_A2
                //               }
                //               {
                //               #ifndef PLAUNO_LIGHT
                //               }
                //                   IF (cfg._4RUNNER_4RForm_LevaScaricoScivolo) AND (disp[LEVA_SCARICO].Stato=RIPOSO) THEN
                //                       OK_Condizione_Move_Asse:=FALSE;
                //                       iret:=Push_Error(Code:=ERR_MOVIMENTAZIONE_LEVA_SCARICO, Arresto:=TRUE);
                //                   END_IF;
                //               {
                //               #endif //FINE NO PLAUNO_LIGHT
                //               }
                //           END_CASE;
                //       {
                //       #endif	//Fine RUNNER
                //       }
                //   {
                //   #else	//Fine ORBITALE
                //   }
                      CASE Asse OF
                          X1: 
                            //   {
                            //   #ifdef SMART %%%%
                            //   }
                            //       IF Disp[PALETTA].enable=TRUE AND Disp[PALETTA].stato<>LAVORO AND TestBasculanteCarico()=FALSE THEN
                            //           //Alzo paletta se basculante non è in posizione di carico
                            //           OK_Condizione_Move_Asse:=FALSE;
                            //           IF i_sys_OPMODE=MDAMODE THEN
                            //               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=PALETTA);
                            //           END_IF;
                            //       END_IF; 
                            //   {
                            //   #else  //Fine SMART
                            //   }
                            //       IF Disp[PALETTA].stato<>LAVORO AND Disp[PALETTA].enable THEN
                            //           OK_Condizione_Move_Asse:=FALSE;
                            //           IF i_sys_OPMODE=MDAMODE THEN
                            //               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=PALETTA);
                            //           END_IF;
                            //       END_IF; 
                            //   {
                            //   #endif  //Fine NO SMART
                            //   }
                            //   {
                            //   #ifdef SOSTEGNO_ELETTRICO
                            //   }
                            //       IF Assi[S1].PosAx.control<>ACTIVE THEN
                            //           OK_Condizione_Move_Asse:=FALSE;
                            //           EnableAxisBlm(Asse:=assi[S1].PosAx, NumeroAsse:=S1, ModoAttesa:=IMMEDIATELY);
                            //       ELSE
                            //           IF (Assi[S1].PosAx.positioningState.homed<>YES OR pm[S1].ma_stato_index<>2) AND
                            //                Assi[X1].PosAx.motionstatedata.commandvelocity<-1 THEN
                            //               OK_Condizione_Move_Asse:=FALSE;
                            //               iret:=Push_error(Code:=ERR_AXES_NO_INDEX);
                            //           END_IF; 
                            //       END_IF; 
                            //   {
                            //   #endif  //Fine SOSTEGNO_ELETTRICO
                            //   }
                            //   {
                            //   #ifdef MS0017
                            //   }
                            //       IF IndicaTipoAl()=TIPO_MS0017 AND mcpi_minus=TRUE THEN
                            //           //Non si può muovere in SN l^asse X1 se L1 è dentro gli ingombri della macchina
                            //           //tranne se l^operatore non esegue lo sblocca monitoring ed ha almeno la password manutentore
                            //           IF MCPI_FEED_STOP=TRUE OR MCPI_SPINDLE_STOP=TRUE OR i_sys_accesslevel>=ACCESS_PROGRAMMATORE THEN
                            //               IF Assi[L1].PosAx.PositioningState.ActualPosition>Programmi[Programma].DatiAL.MS0017_PosL1FuoriIng THEN
                            //                   IF TestRobotDaAl(Programma:=PROGRAMMA_JOG)=FALSE THEN
                            //                       OK_Condizione_Move_Asse:=FALSE;
                            //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=L1);
                            //                   END_IF;
                            //               END_IF;
                            //           END_IF;
                            //       END_IF;
                            //   {
                            //   #endif	//Fine MS0017
                            //   }
                        //   Y1:
                        //       //Non torna con la Y se è fuori la slitta mobile
                        //       {
                        //       #ifdef SMART_SMART_A
                        //       }
                        //           IF Assi[U4].posax.typeofaxis<>VIRTUAL_AXIS THEN
                        //               IF Assi[W4].PosAx.positioningState.actualPosition>0 THEN
                        //                   //Spallamento sul lato DX
                        //                   IF Clk_minus=TRUE AND Assi[U4].PosAx.positioningState.actualPosition>pm[U4].ma_standstill_pos_tol THEN
                        //                       OK_Condizione_Move_Asse:=FALSE;
                        //                       iret:=Push_error(Code:=ERR_SLITTA_MOBILE_NON_A_ZERO);
                        //                   END_IF;
                        //               ELSE
                        //                   //Spallamento sul lato SX
                        //                   IF Clk_plus=TRUE AND Assi[U4].PosAx.positioningState.actualPosition>pm[U4].ma_standstill_pos_tol THEN
                        //                       OK_Condizione_Move_Asse:=FALSE;
                        //                       iret:=Push_error(Code:=ERR_SLITTA_MOBILE_NON_A_ZERO);
                        //                   END_IF;
                        //               END_IF;
                        //           END_IF;
                        //       {
                        //       #else	//Fine SMART_SMART_A %%%%
                        //       }
                                //   {
                                //   #ifdef SLITTA_MOBILE_DX
                                //   }
                                        PosAx:=AsSynchronousAxisRef(Axes[U4].posax);
                                      IF posax^.VirtualAxis.Mode=UDINT#0 THEN
                                          IF Clk_minus=TRUE AND PosAx^.actualPosition>pm[U4].ma_standstill_pos_tol THEN
                                              OK_Condizione_Move_Asse:=FALSE;
                                              iret:=Push_error(Code:=ERR_SLITTA_MOBILE_NON_A_ZERO);
                                          END_IF;
                                      END_IF;
                            //       {
                            //       #endif	//Fine SLITTA_MOBILE_DX
                            //       }
                            //   {
                            //   #endif	//Fine NO SMART_SMART_A
                            //   }
                            //   {
                            //   #ifdef SLITTA_MOBILE_SX %%%%
                            //   }
                            //       //Non torna con la Y se è fuori la slitta mobile sinistra
                            //       IF Assi[U5].posax.typeofaxis<>VIRTUAL_AXIS THEN
                            //           IF Clk_minus=TRUE AND Assi[U5].PosAx.positioningState.actualPosition>pm[U5].ma_standstill_pos_tol THEN
                            //               OK_Condizione_Move_Asse:=FALSE;
                            //               iret:=Push_error(Code:=ERR_SLITTA_MOBILE_NON_A_ZERO);
                            //           END_IF;
                            //       END_IF;
                            //   {
                            //   #endif	//Fine SLITTA_MOBILE_SX
                            //   }
                            //   {
                            //   #ifdef MATRICE_ELETTRICA
                            //   }
                            //       IF TestAsseY2()=TRUE THEN
                            //           IF Clk_minus=TRUE OR Clk_plus=TRUE THEN
                            //               IF Assi[W3].PosAx.positioningState.actualPosition<W3_Aperto AND SbloccaMonitoring(Asse:=Y1)=FALSE THEN
                            //                   OK_Condizione_Move_Asse:=FALSE;
                            //                   set_trace(Code:=TRC_APRIRE_BLOCCAGGIO);
                            //               ELSE
                            //                   reset_trace(TRC_APRIRE_BLOCCAGGIO);
                            //               END_IF;
                            //           ELSE
                            //               reset_trace(TRC_APRIRE_BLOCCAGGIO);
                            //           END_IF;
                            //       END_IF;
                            //   {
                            //   #endif  //Fine MATRICE_ELETTRICA
                            //   }
                            //   {
                            //   #ifdef SMART
                            //   }
                            //       //Per la SMART occorre verificare la posizione della scomparsa rulli e del tassello
                            //       //bloccaggio DX/SX
                            //       IF cfg.tiposlittamobile=TIPO_SLITTA_ELETTRICA THEN
                            //           IF Assi[U4].PosAx.positioningState.actualPosition>U4_Scomparsa+0.2 THEN
                            //               IF Disp[BLOCCAGGIO_DXSX].enable AND cfg.bloccoscomparsabloccaggio=-1 THEN
                            //                   //Rulli in posizione diversa da scomparsa
                            //                   IF Clk_plus AND 
                            //                        Assi[Y1].PosAx.positioningState.actualPosition>POSY_SCOMPBLOCC_SX AND 
                            //                        Disp[BLOCCAGGIO_DXSX].Position<>LAVORO AND
                            //                        Assi[W4].PosAx.positioningState.actualPosition<0 THEN
                            //                       OK_Condizione_Move_Asse:=FALSE;
                            //                       iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                            //                   ELSE
                            //                       IF Clk_minus AND 
                            //                            Assi[Y1].PosAx.positioningState.actualPosition<POSY_SCOMPBLOCC_DX AND 
                            //                            Disp[BLOCCAGGIO_DXSX].Position<>RIPOSO AND
                            //                            Assi[W4].PosAx.positioningState.actualPosition>0 THEN
                            //                           OK_Condizione_Move_Asse:=FALSE;
                            //                           iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                            //                       END_IF;
                            //                   END_IF;
                            //               END_IF;
                            //           END_IF;
                            //       ELSE
                            //           IF Disp[DISAPPEARANCE_ROLLERS].enable=TRUE THEN
                            //               IF Disp[DISAPPEARANCE_ROLLERS].stato<>LAVORO THEN
                            //                   IF Disp[BLOCCAGGIO_DXSX].enable=TRUE AND cfg.bloccoscomparsabloccaggio=-1 THEN
                            //                       //Rulli in posizione diversa da scomparsa
                            //                       IF Clk_plus AND 
                            //                            Assi[Y1].PosAx.positioningState.actualPosition>POSY_SCOMPBLOCC_SX AND 
                            //                            Disp[BLOCCAGGIO_DXSX].Position<>LAVORO AND
                            //                            Assi[W4].PosAx.positioningState.actualPosition<0 THEN
                            //                           OK_Condizione_Move_Asse:=FALSE;
                            //                           iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                            //                       ELSE
                            //                           IF Clk_minus AND 
                            //                                Assi[Y1].PosAx.positioningState.actualPosition<POSY_SCOMPBLOCC_DX AND 
                            //                                Disp[BLOCCAGGIO_DXSX].Position<>RIPOSO AND
                            //                                Assi[W4].PosAx.positioningState.actualPosition>0 THEN
                            //                               OK_Condizione_Move_Asse:=FALSE;
                            //                               iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                            //                           END_IF;
                            //                       END_IF;
                            //                   END_IF;
                            //               END_IF;
                            //           ELSE
                            //               //Scomparsa rulli disabilitatata
                            //               IF Disp[BLOCCAGGIO_DXSX].enable=TRUE AND cfg.bloccoscomparsabloccaggio=-1 THEN
                            //                   IF Clk_plus AND 
                            //                        Assi[Y1].PosAx.positioningState.actualPosition>POSY_SCOMPBLOCC_SX AND 
                            //                        Disp[BLOCCAGGIO_DXSX].Position<>LAVORO AND
                            //                        Assi[W4].PosAx.positioningState.actualPosition<0 THEN
                            //                       OK_Condizione_Move_Asse:=FALSE;
                            //                       iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                            //                   ELSE
                            //                       IF Clk_minus AND 
                            //                            Assi[Y1].PosAx.positioningState.actualPosition<POSY_SCOMPBLOCC_DX AND 
                            //                            Disp[BLOCCAGGIO_DXSX].Position<>RIPOSO AND
                            //                            Assi[W4].PosAx.positioningState.actualPosition>0 THEN
                            //                           OK_Condizione_Move_Asse:=FALSE;
                            //                           iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                            //                       END_IF;
                            //                   END_IF;
                            //               END_IF;
                            //           END_IF;
                            //       END_IF;
                            //   {
                            //   #endif  //Fine SMART
                            //   }
                            //   {
                            //   #ifdef SMART_A
                            //   }
                            //       //Per la SMART occorre verificare la posizione della scomparsa rulli e del tassello
                            //       //bloccaggio DX/SX
                            //       IF Assi[U4].PosAx.positioningState.actualPosition>U4_Scomparsa+0.2 THEN
                            //           IF Disp[BLOCCAGGIO_DXSX].enable AND cfg.bloccoscomparsabloccaggio=-1 THEN
                            //               //Rulli in posizione diversa da scomparsa
                            //               IF Clk_plus AND 
                            //                    Assi[Y1].PosAx.positioningState.actualPosition>POSY_SCOMPBLOCC_SX AND 
                            //                    Disp[BLOCCAGGIO_DXSX].Position<>LAVORO AND
                            //                    Assi[W4].PosAx.positioningState.actualPosition<0 THEN
                            //                   OK_Condizione_Move_Asse:=FALSE;
                            //                   iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                            //               ELSE
                            //                   IF Clk_minus AND 
                            //                        Assi[Y1].PosAx.positioningState.actualPosition<POSY_SCOMPBLOCC_DX AND 
                            //                        Disp[BLOCCAGGIO_DXSX].Position<>RIPOSO AND
                            //                        Assi[W4].PosAx.positioningState.actualPosition>0 THEN
                            //                       OK_Condizione_Move_Asse:=FALSE;
                            //                       iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                            //                   END_IF;
                            //               END_IF;
                            //           END_IF;
                            //       END_IF;
                            //   {
                            //   #endif  //Fine SMART_A
                            //   }
                            //   {
                            //   #ifdef AL_BLM
                            //   }
                            //       {
                            //       #ifndef MS0017
                            //       }
                            //           //In caso di AL5 l^asse Y1 non si deve muovere se l^asse 
                            //           //L1 non si trova in sicurezza
                            //           TipoAl:=IndicaTipoAl();
                            //           IF TipoAl=TIPO_AL5 OR TipoAl=TIPO_AL5_BUNDLE OR TipoAl=TIPO_AL5_FUTABA OR
                            //                TipoAl=TIPO_AL5_ROLAND OR TipoAl=TIPO_AL5_ROLAND_BUNDLE OR
                            //                TipoAl=TIPO_AL5_LUNGO OR TipoAl=TIPO_AL5_LUNGO_BUNDLE THEN
                            //               IF TestAl5Traslatore()=TRUE THEN
                            //                   IF IAddress[INP_AL5_TRASLATORE_INDIETRO].stato<>TRUE THEN
                            //                       iret:=Push_error(Code:=ERR_L1_TRASLATORE);
                            //                   END_IF;
                            //               ELSE
                            //                   IF Assi[L1].PosAx.PositioningState.ActualPosition>Programmi[PROGRAMMA_JOG].DatiAL.L1PosPrePresaTubo+pm[L1].MA_standstill_pos_tol THEN
                            //                       iret:=Push_error(Code:=ERR_L1_Y1);
                            //                   END_IF;
                            //               END_IF;
                            //           END_IF;  
                            //       {
                            //       #endif  //Fine No MS0017
                            //       }
                            //   {
                            //   #endif  //Fine AL_BLM
                            //   }
                            //   ;
                          Z1:
                              IF Disp[PALETTA].stato<>DEVICES_STATUS#LAVORO AND Disp[PALETTA].enable THEN
                                  OK_Condizione_Move_Asse:=FALSE;
                                  IF i_sys_OPMODE=PI_COMMANDS_CODE#MDAMODE THEN
                                      set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Axis:=PALETTA);
                                  END_IF;
                              END_IF; 
                            //   {
                            //   #ifdef SOSTEGNO_ELETTRICO %%%%
                            //   }
                            //       IF Assi[S1].PosAx.control<>ACTIVE THEN
                            //           OK_Condizione_Move_Asse:=FALSE;
                            //           EnableAxisBlm(Asse:=assi[S1].PosAx, NumeroAsse:=S1, ModoAttesa:=IMMEDIATELY);
                            //       ELSE
                            //           IF (Assi[S1].PosAx.positioningState.homed<>YES OR pm[S1].ma_stato_index<>2) AND
                            //                Assi[X1].PosAx.motionstatedata.commandvelocity<-1 THEN
                            //               OK_Condizione_Move_Asse:=FALSE;
                            //               iret:=Push_error(Code:=ERR_AXES_NO_INDEX);
                            //           END_IF; 
                            //       END_IF; 
                            //   {
                            //   #endif  //Fine SOSTEGNO_ELETTRICO
                            //   }
                        //   Q1:
                            //   {
                            //   #ifdef AL_BLM
                            //   }
                            //       {
                            //       #ifndef MS0017
                            //       }
                            //           //In caso di AL5 l^asse Q1 per potersi muovere quando l^asse 
                            //           //L1 non si trova in sicurezza, il braccio di curvatura deve essere
                            //           //a 0 o 180
                            //           TipoAl:=IndicaTipoAl();
                            //           IF TipoAl=TIPO_AL5 OR TipoAl=TIPO_AL5_BUNDLE OR TipoAl=TIPO_AL5_FUTABA OR
                            //                TipoAl=TIPO_AL5_ROLAND OR TipoAl=TIPO_AL5_ROLAND_BUNDLE OR
                            //                TipoAl=TIPO_AL5_LUNGO OR TipoAl=TIPO_AL5_LUNGO_BUNDLE THEN
                            //               IF TestAl5Traslatore()=TRUE THEN
                            //                   IF IAddress[INP_AL5_TRASLATORE_INDIETRO].stato<>TRUE AND
                            //                        i_sys_Accesslevel>ACCESS_MANUTENTORE THEN
                            //                       iret:=Push_error(Code:=ERR_L1_TRASLATORE);
                            //                   END_IF;
                            //               ELSE
                            //                   IF Assi[L1].PosAx.PositioningState.ActualPosition>Programmi[PROGRAMMA_JOG].DatiAL.L1PosPrePresaTubo+pm[L1].MA_standstill_pos_tol THEN
                            //                       IF Assi[Y1].PosAx.PositioningState.ActualPosition>1 AND 
                            //                            Assi[Y1].PosAx.PositioningState.ActualPosition<179 THEN
                            //                           iret:=Push_error(Code:=ERR_L1_Y1_Q1);
                            //                       END_IF;                 
                            //                   END_IF;
                            //               END_IF;  
                            //           END_IF;  
                            //       {
                            //       #endif	//Fine No MS0017
                            //       }
                            //   {
                            //   #endif  //Fine AL_BLM
                            //   }
                            //   ;
                          V1:;
                            //   {
                            //   #ifndef MS0017
                            //   }
                            //       {
                            //       #ifdef AL_BLM
                            //       }
                            //           //In caso di AL5 l^asse V1 per potersi muovere quando l^asse 
                            //           //L1 non si trova in sicurezza, il braccio di curvatura deve essere
                            //           //a 0 o 180
                            //           TipoAl:=IndicaTipoAl();
                            //           IF TipoAl=TIPO_AL5 OR TipoAl=TIPO_AL5_BUNDLE OR TipoAl=TIPO_AL5_FUTABA OR
                            //                TipoAl=TIPO_AL5_ROLAND OR TipoAl=TIPO_AL5_ROLAND_BUNDLE OR
                            //                TipoAl=TIPO_AL5_LUNGO OR TipoAl=TIPO_AL5_LUNGO_BUNDLE THEN
                            //               IF TestAl5Traslatore()=TRUE THEN
                            //                   IF IAddress[INP_AL5_TRASLATORE_INDIETRO].stato<>TRUE AND
                            //                        i_sys_Accesslevel>ACCESS_MANUTENTORE THEN
                            //                       iret:=Push_error(Code:=ERR_L1_TRASLATORE);
                            //                   END_IF;
                            //               ELSE
                            //                   IF Assi[L1].PosAx.PositioningState.ActualPosition>Programmi[PROGRAMMA_JOG].DatiAL.L1PosPrePresaTubo+pm[L1].MA_standstill_pos_tol THEN
                            //                       IF Assi[Y1].PosAx.PositioningState.ActualPosition>1 AND 
                            //                            Assi[Y1].PosAx.PositioningState.ActualPosition<179 THEN
                            //                           iret:=Push_error(Code:=ERR_L1_Y1_R1);
                            //                       END_IF;                 
                            //                   END_IF;
                            //               END_IF;
                            //           END_IF;  
                            //       {
                            //       #endif  //Fine AL_BLM
                            //       }
                            //   {
                            //   #endif  //Fine No MS0017
                            //   }
                            //   {
                            //   #ifdef BLOCCAGGIO_DX
                            //   }
                            //       {
                            //       #ifdef MATRICE_ELETTRICA
                            //       }
                            //           W3:
                            //               IF TestAsseY2()=TRUE THEN
                            //                   IF Direzione=SN THEN
                            //                       //In movimento in chiusura (SN) si deve verificare che la matrice sia nella stessa posizione del braccio
                            //                       //sia chiusa
                            //                       IF ABS(Assi[Y2].PosAx.positioningState.actualPosition-Assi[Y1].PosAx.positioningState.actualPosition)>pm[Y2].MA_standstill_pos_tol AND 
                            //                            SbloccaMonitoring(Asse:=W3)=FALSE THEN
                            //                           IF Clk_minus=TRUE THEN
                            //                               set_trace(Code:=TRC_MATRICE_APERTA);
                            //                               OK_Condizione_Move_Asse:=FALSE;
                            //                           ELSE
                            //                               reset_trace(TRC_MATRICE_APERTA);
                            //                           END_IF;
                            //                       ELSE
                            //                           reset_trace(TRC_MATRICE_APERTA);
                            //                       END_IF; 
                            //                   ELSE
                            //                       reset_trace(TRC_MATRICE_APERTA);
                            //                   END_IF;
                            //               END_IF;
                            //       {
                            //       #endif  //MATRICE_ELETTRICA    
                            //       }
                            //       ;
                            //   {
                            //   #endif  //Fine BLOCCAGGIO_DX
                            //   }
                              {::ifdef SLITTA_MOBILE_DX}
                                //   U4:
                                //       {
                                //       #ifdef SMART_SMART_A %%%%
                                //       }
                                //           IF Assi[W4].PosAx.positioningState.actualPosition>0 THEN
                                //               IF Clk_plus=TRUE AND Assi[Y1].PosAx.positioningState.actualPosition<90 THEN
                                //                   OK_Condizione_Move_Asse:=FALSE;
                                //                   iret:=Push_error(Code:=ERR_BRACCIO_SOTTO_90);
                                //               END_IF;
                                //           ELSE
                                //               IF Clk_plus=TRUE AND Assi[Y1].PosAx.positioningState.actualPosition>90 THEN
                                //                   OK_Condizione_Move_Asse:=FALSE;
                                //                   iret:=Push_error(Code:=ERR_BRACCIO_SOTTO_90);
                                //               END_IF;
                                //           END_IF;
                                //       {
                                //       #else //Fine SMART_SMART_A
                                //       }
                                            PosAx:= AsSynchronousAxisRef(Axes[Y1].PosAx);
                                          IF Clk_plus=TRUE AND PosAx^.actualPosition<90 THEN
                                              OK_Condizione_Move_Asse:=FALSE;
                                              iret:=Push_error(Code:=ERR_BRACCIO_SOTTO_90);
                                          END_IF;
                            //           {
                            //           #endif  //Fine NO SMART_SMART_A
                            //           }
                             {::endif}
                            //   {
                            //   #ifdef SLITTA_MOBILE_SX
                            //   }
                            //       U5:
                            //           IF Clk_plus=TRUE AND Assi[Y1].PosAx.positioningState.actualPosition<90 THEN
                            //               OK_Condizione_Move_Asse:=FALSE;
                            //               iret:=Push_error(Code:=ERR_BRACCIO_SOTTO_90);
                            //           END_IF;
                            //   {
                            //   #endif  //Fine SLITTA_MOBILE_SX
                            //   }
                            //   {
                            //   #ifdef MATRICE_ELETTRICA
                            //   }
                            //       Y2:
                            //           IF TestAsseY2()=TRUE THEN
                            //               IF Clk_minus=TRUE OR Clk_plus=TRUE THEN
                            //                   IF Assi[W3].PosAx.positioningState.actualPosition<=W3_Aperto AND 
                            //                        SbloccaMonitoring(Asse:=Y2)=FALSE THEN
                            //                       OK_Condizione_Move_Asse:=FALSE;
                            //                       set_trace(Code:=TRC_APRIRE_BLOCCAGGIO);
                            //                   ELSE
                            //                       reset_trace(TRC_APRIRE_BLOCCAGGIO);
                            //                   END_IF;
                            //               ELSE
                            //                   reset_trace(TRC_APRIRE_BLOCCAGGIO);
                            //               END_IF;  
                            //           END_IF;
                            //   {
                            //   #endif  //Fine MATRICE_ELETTRICA
                            //   }    
                            //   {
                            //   #ifdef SOSTEGNO_ELETTRICO
                            //   }
                            //       S1:
                            //           IF Assi[X1].PosAx.positioningState.actualPosition<MAX(i_sys_MAX_X_SOSTEGNO_ALTO, cfg.QuotaAbbassoSostegnoTirante)+pm[X1].ma_standstill_pos_tol THEN
                            //               OK_Condizione_Move_Asse:=FALSE;
                            //               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=X1);
                            //           END_IF;
                            //   {
                            //   #endif  //SOSTEGNO_ELETTRICO
                            //   }
                              {::ifdef AL_BLM}
                                  L1:
                                      IF TestAL5Traslatore()=TRUE THEN
                                          //Con traslatore orizzonatale tubo L1 può essere mosso solo con traslatore indietro
                                          IF IAddress[INP_AL5_TRASLATORE_INDIETRO].stato<>TRUE AND i_sys_Accesslevel>ACCESS_MANUTENTORE THEN
                                              OK_Condizione_Move_Asse:=FALSE;
                                              iret:=Push_error(Code:=ERR_L1_TRASLATORE);
                                          END_IF;  
                                      END_IF;
                                    //   {
                                    //   #ifdef MS0017 %%%%
                                    //   }
                                    //       IF IndicaTipoAl()=TIPO_MS0017 AND mcpi_plus=TRUE THEN
                                    //           //Non si può muovere L1 in SP se X1 è inferiore alla quota di carico tranne se
                                    //           //l^operatore non esegue lo sblocca monitoring ed ha almeno la password manutentore
                                    //           IF MCPI_FEED_STOP=TRUE OR MCPI_SPINDLE_STOP=TRUE OR i_sys_accesslevel>=ACCESS_PROGRAMMATORE THEN
                                    //               IF Assi[X1].PosAx.PositioningState.ActualPosition<pm[X1].Ma_Pos_Limit_Plus_Attivo-EXTRALIMIT THEN
                                    //                   IF TestRobotDaAl(Programma:=PROGRAMMA_JOG)=FALSE THEN
                                    //                       OK_Condizione_Move_Asse:=FALSE;
                                    //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=X1);
                                    //                   END_IF;
                                    //               END_IF;
                                    //           END_IF;
                                    //       END_IF;
                                    //   {
                                    //   #endif  //Fine MS0017
                                    //   }
                              {::endif}
                            //   {        
                            //   #ifdef BASCULANTE_ELETTRICO %%%%
                            //   }
                            //       B2:
                            //           IF BasculanteAvvio=FALSE THEN
                            //               IF Assi[V1].PosAx.positioningState.actualPosition-cfg.i_sys_postestabassaxmin>pm[V1].ma_standstill_pos_tol THEN
                            //                   iret:=Push_error(Code:=ERR_TESTA_NO_BASSA);
                            //                   OK_Condizione_Move_Asse:=FALSE;
                            //               END_IF;
                            //           END_IF;
                            //   {
                            //   #endif  //Fine BASCULANTE_ELETTRICO
                            //   }
                            //   {
                            //   #ifdef TAVOLI_ELETTRICI
                            //   }
                            //       //%%%%
                            //       T1:
                            //           IF Assi[Y1].PosAx.positioningState.actualPosition >= 90 AND Assi[Y1].PosAx.positioningState.actualPosition < 179 THEN
                            //               iret:=Push_error(Code:=ERR_BRACCIO_NON_OPPOSTO);
                            //               Ok_Condizione_Move_Asse:=FALSE;
                            //           END_IF;
                            //       T2:
                            //           IF Assi[Y1].PosAx.positioningState.actualPosition <= 90 AND Assi[Y1].PosAx.positioningState.actualPosition > 1 THEN
                            //               iret:=Push_error(Code:=ERR_BRACCIO_NON_OPPOSTO);
                            //               Ok_Condizione_Move_Asse:=FALSE;
                            //           END_IF;
                            //   {
                            //   #endif //FINE TAVOLI_ELETTRICI
                            //   }
                      END_CASE;
                //   {
                //   #endif  //Fine NO ORBITALE
                //   }
            //   {
            //   #endif  //Fine NO EFLEX_DH4010
            //   }
              IF (mcpi_minus=FALSE AND mcpi_plus=FALSE) OR OK_Condizione_Move_Asse=TRUE THEN
                  reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
              END_IF;
          END_IF;
    END_IF;    
  END_FUNCTION

  //Verifica se la configurazione degli utensili inviata da VGP3D (per curvature DX/SX sullo stesso 
//utensile è compatibile con la macchina attiva, oppure con un eventuale accessorio che
//consenta la curvatura DX/SX in processo
// {
// #ifndef EFLEX_DH4010 %%%%
// }
  FUNCTION Utensili_DX_SX:BOOL
    VAR
      i         			:INT;
      LatoUtensileDx  :INT;
      LatoUtensileSx  :INT;
    END_VAR 
    ;
//     {
//     #ifdef CURVATURA
//     }
// 			LatoUtensileDx:=0;
// 			LatoUtensileSx:=0;
// 			Utensili_DX_SX:=FALSE;
// 			{
// 			#ifdef DXSX_PROCESSO
// 			}
// 				Utensili_DX_SX:=TRUE;
// 				GOTO Fine;
// 			{
// 			#endif	//Fine DXSX_PROCESSO
// 			}
// 			IF VerificaElectDXSX_PiegaInversa()=TRUE OR MO1094()=TRUE OR RunnerDXSX()=TRUE THEN
// 				Utensili_DX_SX:=TRUE;
// 			ELSE
// 				//E^ possibile effettuare la curvatura solo DX o solo SX
// 				FOR i:=1 TO MAXTOOLS DO
// 					IF BlmMxToolsDx[i].Qqt<>0 AND BlmMxToolsDx[i].Qtt<>0 THEN
// 						LatoUtensileDx:=DX;
// 					END_IF;     
// 					IF BlmMxToolsSx[i].Qqt<>0 AND BlmMxToolsSx[i].Qtt<>0 THEN
// 						LatoUtensileSx:=SX;
// 					END_IF;
// 				END_FOR;    
// 				IF LatoUtensileDx=DX AND LatoUtensileSx=SX THEN
// 					//Indicazione che è contemporanemante presente un utesile DXSX
// 					Utensili_DX_SX:=FALSE;
// 				ELSE
// 					//Presenti utensili o solo DX o solo SX, si verifica che siano
// 					//compatibili con il senso di curvatura della macchina
// 					IF (LatoUtensileDx=DX AND i_sys_senso_curvatura=DX) OR 
// 						 (LatoUtensileSx=SX AND i_sys_senso_curvatura=SX) THEN
// 						Utensili_DX_SX:=TRUE;
// 					ELSE
// 						Utensili_DX_SX:=FALSE;
// 					END_IF;
// 				END_IF;
// 			END_IF;
// 			Fine:;
//     {
//     #else	//Fine CURVATURA
//     }
// 			Utensili_DX_SX:=TRUE;
//     {
//     #endif	//Fine CURVATURA
//     }
  END_FUNCTION
// {
// #endif	//Fine NO EFLEX_DH4010
// }
  
/*******************************************************************************
 * RETURNS TRUE IF THE TRANSLATOR CARRYING THE TUBE INTO THE MACHINE IS ACTIVE *
 *******************************************************************************/
FUNCTION TestAL5Traslatore:BOOL

    VAR_EXTERNAL
        OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
        Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
    END_VAR

    VAR_EXTERNAL CONSTANT
        MAXDIGITALOUTPUT                    :INT;
        FIRST_DEVICE                        :INT;
        OUT_AL5_AVANTI_TRASLATORE           :INT;
        OUT_AL5_INDIETRO_TRASLATORE         :INT;
        AL5_TRASLATORE                      :INT;
    END_VAR

    {::ifdef AL_BLM}
      IF OAddress[OUT_AL5_AVANTI_TRASLATORE].port>-1 AND
         OAddress[OUT_AL5_AVANTI_TRASLATORE].bit>-1 AND
         OAddress[OUT_AL5_INDIETRO_TRASLATORE].port>-1 AND
         OAddress[OUT_AL5_INDIETRO_TRASLATORE].bit>-1 THEN
        TestAL5Traslatore:=TRUE;
        Disp[AL5_TRASLATORE].Enable:=TRUE;
      ELSE
        TestAL5Traslatore:=FALSE;
        Disp[AL5_TRASLATORE].Enable:=FALSE;
      END_IF;
    {::endif}
    ;
  END_FUNCTION
  
  FUNCTION OK_Condizione_Move_Dispositivo:BOOL

    VAR_EXTERNAL
        Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
        cfg                             :_TYPE.Config_Struct;
        i_sys_MAX_X_SOSTEGNO_ALTO		:REAL;  //Quota massima X con sostegno alto, dopo si deve abbassare
        i_sys_monitoring                : INT ;
        Clk_Minus	                    :BOOL;
        Clk_Plus                        :BOOL;
    END_VAR

    VAR_EXTERNAL CONSTANT
        FIRST_DEVICE                          :INT;
        TRC_MACCHINA_PEZZO_DXSX               :INT;
        X1 : INT;
        Z1 : INT;
        W4 : INT;
        Y1 : INT;
        TRC_SAFETY_JOG_DEVICES              :INT;
        PROGRAM_JOG                      	:SINT;
        CLAMP                               :INT;  //32
        PALETTA                             :INT;  //33
        SUPPORT_TIRANT                      :INT;
        DISAPPEARANCE_ROLLERS               :INT;
        ERR_APRIRE_SLITTA_SPALLAMENTO       :DINT;
        LOCKING_DXSX                        :INT;
        POSY_SCOMPBLOCC_DX                	:REAL;
        POSY_SCOMPBLOCC_SX                	:REAL;
        ERR_TASSELLO_BLOCCAGGIO_DXSX        :DINT;
    END_VAR

    VAR_INPUT
      Dispositivo	:INT;
      Stato				:LREAL;
      TastoPiu		:BOOL;
      TastoMeno		:BOOL;
    END_VAR
    
    VAR
      PosAx             : REF_TO TO_SynchronousAxis;
      iret				:INT;
      dret				:DINT;
      i				    :INT;
      LimiteU2		    :REAL;
    END_VAR

      IF Dispositivo<FIRST_DEVICE OR Dispositivo>FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1 THEN
          OK_Condizione_Move_Dispositivo:=FALSE;
      ELSE
          OK_Condizione_Move_Dispositivo:=TRUE;
        //   {
        //   #ifdef EFLEX_DH4010 %%%%
        //   }
        //       {
        //       #ifdef DH4010
        //       }
        //           CASE Dispositivo OF
        //               SCOMP_PINZA_SX:
        //                   IF disp[PINZA_SX].stato<>RIPOSO THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //               SCOMP_PINZA_DX:
        //                   IF disp[PINZA_DX].stato<>RIPOSO THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //               SCOMP_MANIPOLATORE_3D:
        //                   IF Assi[X1].PosAx.PositioningState.ActualPosition+0.05<cfg.MinX1Manipolatore_Alto THEN
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=X1);
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   ELSE
        //                       IF Assi[X2].PosAx.PositioningState.ActualPosition+0.05<cfg.MinX2Manipolatore_Alto THEN
        //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=X2);
        //                           OK_Condizione_Move_Dispositivo:=FALSE;
        //                       END_IF;
        //                   END_IF;
        //                   IF Disp[BLOCCAGGIO_3D].stato<>RIPOSO THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //                   IF ABS(Assi[Z1].PosAx.positioningState.actualPosition-65)>pm[Z1].ma_tolerance THEN 
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //                   IF Disp[BLOCCAGGIO_2D].Enable=TRUE AND TastoPiu=FALSE THEN
        //                       //Con il bloccaggio 2D abilitato non è possibile alzare il bloccaggio3D
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_2D);
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //               SCOMPARSA_SUPPORTO1_SX:
        //                   IF Disp[BLOCCAGGIO_SUPPORTO1_SX].Stato<>RIPOSO THEN
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_SUPPORTO1_SX);
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //                   IF Disp[SCOMPARSA_SUPPORTO1_SX].Limite2_Lavoro=Disp[SCOMPARSA_SUPPORTO1_SX].Limite2_Riposo THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
        //                   END_IF;
        //               SCOMPARSA_SUPPORTO2_SX:
        //                   IF Disp[BLOCCAGGIO_SUPPORTO2_SX].Stato<>RIPOSO THEN
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_SUPPORTO2_SX);
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //                   IF Disp[SCOMPARSA_SUPPORTO2_SX].Limite2_Lavoro=Disp[SCOMPARSA_SUPPORTO2_SX].Limite2_Riposo THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
        //                   END_IF;
        //               SCOMPARSA_SUPPORTO1_DX:
        //                   IF Disp[BLOCCAGGIO_SUPPORTO1_DX].Stato<>RIPOSO THEN
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_SUPPORTO1_DX);
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //                   IF Disp[SCOMPARSA_SUPPORTO1_DX].Limite2_Lavoro=Disp[SCOMPARSA_SUPPORTO1_DX].Limite2_Riposo THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
        //                   END_IF;
        //               SCOMPARSA_SUPPORTO2_DX:
        //                   IF Disp[BLOCCAGGIO_SUPPORTO2_DX].Stato<>RIPOSO THEN
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_SUPPORTO2_DX);
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                   END_IF;
        //                   IF Disp[SCOMPARSA_SUPPORTO2_DX].Limite2_Lavoro=Disp[SCOMPARSA_SUPPORTO2_DX].Limite2_Riposo THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
        //                   END_IF;
        //               BLOCCAGGIO_SUPPORTO1_SX:    
        //                   IF Disp[SCOMPARSA_SUPPORTO1_SX].Position<>Disp[SCOMPARSA_SUPPORTO1_SX].Limite_Riposo THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_SUPPORTO1_SX);
        //                   END_IF;
        //               BLOCCAGGIO_SUPPORTO2_SX:    
        //                   IF Disp[SCOMPARSA_SUPPORTO2_SX].Position<>Disp[SCOMPARSA_SUPPORTO2_SX].Limite_Riposo THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_SUPPORTO2_SX);
        //                   END_IF;
        //               BLOCCAGGIO_SUPPORTO1_DX:    
        //                   IF Disp[SCOMPARSA_SUPPORTO1_DX].Position<>Disp[SCOMPARSA_SUPPORTO1_DX].Limite_Riposo THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_SUPPORTO1_DX);
        //                   END_IF;
        //               BLOCCAGGIO_SUPPORTO2_DX:    
        //                   IF Disp[SCOMPARSA_SUPPORTO2_DX].Position<>Disp[SCOMPARSA_SUPPORTO2_DX].Limite_Riposo THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BLOCCAGGIO_SUPPORTO2_DX);
        //                   END_IF;
        //               SCOMP1_SX, SCOMP2_SX:
        //                   IF readdigitalinput(NumInput:=INP_PALPATOIO_1_AVANTI_TESTA_SX)=TRUE AND Disp[PALPATORE_1_SX].Enable=TRUE THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       set_trace(Code:=TRC_PALPATORE_1_SX, Asse:=PALPATORE_1_SX);
        //                   ELSE
        //                       IF readdigitalinput(NumInput:=INP_PALPATOIO_2_AVANTI_TESTA_SX)=TRUE AND Disp[PALPATORE_2_SX].Enable=TRUE THEN
        //                           OK_Condizione_Move_Dispositivo:=FALSE;
        //                           set_trace(Code:=TRC_PALPATORE_2_SX, Asse:=PALPATORE_2_SX);
        //                       ELSE
        //                           IF readdigitalinput(NumInput:=INP_PALPATOIO_3_AVANTI_TESTA_SX)=TRUE AND Disp[PALPATORE_3_SX].Enable=TRUE THEN
        //                               OK_Condizione_Move_Dispositivo:=FALSE;
        //                               set_trace(Code:=TRC_PALPATORE_3_SX, Asse:=PALPATORE_3_SX);
        //                           ELSE    
        //                               IF readdigitalinput(NumInput:=INP_PALPATOIO_4_AVANTI_TESTA_SX)=TRUE AND Disp[PALPATORE_4_SX].Enable=TRUE THEN
        //                                   OK_Condizione_Move_Dispositivo:=FALSE;
        //                                   set_trace(Code:=TRC_PALPATORE_4_SX, Asse:=PALPATORE_4_SX);
        //                               END_IF;
        //                           END_IF;
        //                       END_IF;
        //                   END_IF;
        //               SCOMP1_DX, SCOMP2_DX:
        //                   IF readdigitalinput(NumInput:=INP_PALPATOIO_1_AVANTI_TESTA_DX)=TRUE AND Disp[PALPATORE_1_DX].Enable=TRUE THEN
        //                       OK_Condizione_Move_Dispositivo:=FALSE;
        //                       set_trace(Code:=TRC_PALPATORE_1_DX, Asse:=PALPATORE_1_DX);
        //                   ELSE
        //                       IF readdigitalinput(NumInput:=INP_PALPATOIO_2_AVANTI_TESTA_DX)=TRUE AND Disp[PALPATORE_2_DX].Enable=TRUE THEN
        //                           OK_Condizione_Move_Dispositivo:=FALSE;
        //                           set_trace(Code:=TRC_PALPATORE_2_DX, Asse:=PALPATORE_2_DX);
        //                       ELSE
        //                           IF readdigitalinput(NumInput:=INP_PALPATOIO_3_AVANTI_TESTA_DX)=TRUE AND Disp[PALPATORE_3_DX].Enable=TRUE THEN
        //                               OK_Condizione_Move_Dispositivo:=FALSE;
        //                               set_trace(Code:=TRC_PALPATORE_3_DX, Asse:=PALPATORE_3_DX);
        //                           ELSE    
        //                               IF readdigitalinput(NumInput:=INP_PALPATOIO_4_AVANTI_TESTA_DX)=TRUE AND Disp[PALPATORE_4_DX].Enable=TRUE THEN
        //                                   OK_Condizione_Move_Dispositivo:=FALSE;
        //                                   set_trace(Code:=TRC_PALPATORE_4_DX, Asse:=PALPATORE_4_DX);
        //                               END_IF;
        //                           END_IF;
        //                       END_IF;
        //                   END_IF;
        //           END_CASE;
        //       {
        //       #endif  //Fine DH4010
        //       }
        //   {
        //   #else  //Fine EFLEX_DH4010
        //   }
              IF Utensili_DX_SX()=FALSE THEN
                  set_trace(Code:=TRC_MACCHINA_PEZZO_DXSX);
                  OK_Condizione_Move_Dispositivo:=FALSE;
              ELSE
                  IF  ShadowMotorCheck()>=_FIRST_AXIS#FIRST_AXIS THEN
                      OK_Condizione_Move_Dispositivo:=FALSE;
                  END_IF;
              END_IF;
            //   {
            //   #ifdef PLAUNO_LIGHT %%%%
            //   }
            //       {
            //       #ifdef TAVOLI_SERPENTINE
            //       }
            //           CASE Dispositivo OF
            //               TAVOLO_POST_ORIZZONTALE, TAVOLO_POST_ORIZZ_DX, TAVOLO_POST_ORIZZ_SX, TAVOLO_POST_ROTAZIONE:
            //                   //Consentiti movimenti di chiusura tavoli verso la macchina solo con Z1 a 0° o 180°
            //                   IF Stato=RIPOSO THEN
            //                       IF ABS(Assi[Z1].PosAx.positioningState.actualPosition-0)>pm[Z1].ma_standstill_pos_tol AND
            //                            ABS(Assi[Z1].PosAx.positioningState.actualPosition-180)>pm[Z1].ma_standstill_pos_tol THEN
            //                           OK_Condizione_Move_Dispositivo:=FALSE;
            //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Z1);
            //                       END_IF;
            //                   END_IF;
            //               TAVOLO_ANT_ORIZZONTALE:
            //                   IF Stato=RIPOSO THEN
            //                       //Consentiti movimenti di chiusura tavolo verso la macchina solo con Z1 a 0° o 180°
            //                       IF ABS(Assi[Z1].PosAx.positioningState.actualPosition-0)>pm[Z1].ma_standstill_pos_tol AND
            //                            ABS(Assi[Z1].PosAx.positioningState.actualPosition-180)>pm[Z1].ma_standstill_pos_tol THEN
            //                           OK_Condizione_Move_Dispositivo:=FALSE;
            //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Z1);
            //                       END_IF;
            //                   END_IF;
            //                   IF Stato=LAVORO THEN
            //                       //Richiesta apertura tavolo orizzontale
            //                       IF ReadDigitalInput(NumInput:=INP_TAVOLO_ANTERIORE_ALTO, Verifica:=TRUE)=FALSE THEN
            //                           //Tavolo non è alto
            //                           IF RunnerDXSX()=TRUE THEN
            //                               IF Assi[Y1].PosAx.positioningState.actualPosition>1 AND Assi[Z1].PosAx.positioningState.actualPosition<179 THEN  
            //                                   //Con DXSX in processo non si può muovere se l^asse Y1 non è a 0° o 180°
            //                                   OK_Condizione_Move_Dispositivo:=FALSE;
            //                                   set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Y1);
            //                               END_IF;
            //                           ELSE
            //                               IF Assi[Y1].PosAx.positioningState.actualPosition>1 THEN  
            //                                   //Non si può muovere se l^asse Y1 non è a 0° 
            //                                   OK_Condizione_Move_Dispositivo:=FALSE;
            //                                   set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Y1);
            //                               END_IF;
            //                           END_IF;
            //                       END_IF;  
            //                   END_IF;  
            //               TAVOLO_ANT_ROTAZIONE:
            //                   //Consentiti movimenti di chiusura tavoli verso la macchina solo con Z1 a 0° o 180°
            //                   IF Stato=RIPOSO THEN
            //                       IF ABS(Assi[Z1].PosAx.positioningState.actualPosition-0)>pm[Z1].ma_standstill_pos_tol AND
            //                           ABS(Assi[Z1].PosAx.positioningState.actualPosition-180)>pm[Z1].ma_standstill_pos_tol THEN
            //                           OK_Condizione_Move_Dispositivo:=FALSE;
            //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Z1);
            //                       END_IF;
            //                   END_IF;
            //                   IF ReadDigitalInput(NumInput:=INP_TAVOLO_ANTERIORE_FUORI_INGOMBRO, Verifica:=TRUE)=TRUE THEN
            //                       //Tavolo anteriore orizzontale aperto
            //                       IF RunnerDXSX()=TRUE THEN
            //                           IF Assi[Y1].PosAx.positioningState.actualPosition>1 AND Assi[Z1].PosAx.positioningState.actualPosition<179 THEN  
            //                               //Con DXSX in processo non si può muovere se l^asse Y1 non è a 0° o 180°
            //                               OK_Condizione_Move_Dispositivo:=FALSE;
            //                               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Y1);
            //                           END_IF;
            //                       ELSE
            //                           IF Assi[Y1].PosAx.positioningState.actualPosition>1 THEN  
            //                               //Non si può muovere se l^asse Y1 non è a 0° 
            //                               OK_Condizione_Move_Dispositivo:=FALSE;
            //                               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Y1);
            //                           END_IF;
            //                       END_IF;
            //                   END_IF;
            //           END_CASE;
            //       {
            //       #endif  //Fine TAVOLI_SERPENTINE
            //       }
            //   {
            //   #else	//Fine PLAUNO_LIGHT
            //   }
                  CASE Dispositivo OF
                    //   {
                    //   #ifdef RUNNER %%%%
                    //   }
                    //       LEVA_SCARICO:
                    //           IF cfg._4RUNNER_TipoTaglioEsterno=TAGLIO_4RTL THEN
                    //               LimiteU2:=cfg._4RUNNER_Pos_U2_LEVA_ALTA;
                    //           ELSE
                    //               LimiteU2:=cfg._4RUNNER_Pos_U2_LEVA_ALTA_4RC;
                    //           END_IF;
                    //           IF Assi[U2].PosAx.positioningState.actualPosition>LimiteU2 THEN
                    //               OK_Condizione_Move_Dispositivo:=FALSE;
                    //               set_trace(Code:=trc_SICUREZZA_JOG_DISPOSITIVI, Asse:=U2);
                    //           END_IF;
                    //           IF ((Assi[U1].PosAx.PositioningState.ActualPosition<=cfg._4RUNNER_U1PosTaglio) AND (Assi[U1].PosAx.PositioningState.ActualPosition>=pm[U1].ma_pos_limit_minus + 50)) AND (cfg._4RUNNER_4RForm_LevaScaricoScivolo) AND (Stato=LAVORO) THEN
                    //               OK_Condizione_Move_Dispositivo:=FALSE;
                    //               iret:=Push_Error(Code:=ERR_MOVIMENTAZIONE_LEVA_SCARICO, Arresto:=TRUE);                    
                    //           END_IF; 
                    //           IF (Assi[U2].PosAx.PositioningState.ActualPosition>0) AND (cfg._4RUNNER_4RForm_LevaScaricoScivolo) AND (Stato=RIPOSO) THEN
                    //               OK_Condizione_Move_Dispositivo:=FALSE;
                    //               iret:=Push_Error(Code:=ERR_MOVIMENTAZIONE_LEVA_SCARICO, Arresto:=TRUE);
                    //           END_IF;
                    //           {
                    //           #ifdef THREE_RUNNER
                    //           }
                    //               IPC_STRAPPO:
                    //                   IF r_minus.Q=TRUE THEN
                    //                       IF Disp[IPC_MORSA_STRAPPO].stato<>RIPOSO THEN
                    //                           OK_Condizione_Move_Dispositivo:=FALSE;
                    //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=IPC_MORSA_STRAPPO);
                    //                       END_IF;
                    //                   END_IF;
                    //               IPC_STRAPPO_SCARICO:
                    //                   IF r_minus.Q=TRUE THEN
                    //                       IF Disp[IPC_MORSA_STRAPPO].stato<>RIPOSO THEN
                    //                           OK_Condizione_Move_Dispositivo:=FALSE;
                    //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=IPC_MORSA_STRAPPO);
                    //                       END_IF;
                    //                       IF Disp[IPC_STRAPPO].stato<>RIPOSO THEN
                    //                           OK_Condizione_Move_Dispositivo:=FALSE;
                    //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=IPC_STRAPPO);
                    //                       END_IF;
                    //                   END_IF;
                    //           {
                    //           #endif	//Fine THREE_RUNNER
                    //           }
                    //           {
                    //           #ifdef A1_A2
                    //           }
                    //               IF TestBloccaggio4RCPresente()=TRUE AND Disp[BLOCCAGGIO_4RC].Position<>RIPOSO THEN 
                    //                   OK_Condizione_Move_Dispositivo:=FALSE;
                    //                   set_trace(Code:=trc_SICUREZZA_JOG_DISPOSITIVI, Asse:=BLOCCAGGIO_4RC);
                    //               END_IF;
                    //           {
                    //           #endif	//Fine A1_A2
                    //           }
                    //           {
                    //           #ifdef FOUR_RUNNER
                    //           }
                    //               IPC_AVANZAMENTO:
                    //                   IF Disp[IPC_MOTORE].stato<>LAVORO THEN
                    //                       OK_Condizione_Move_Dispositivo:=FALSE;
                    //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=IPC_MOTORE);
                    //                   END_IF;
                    //               IPC_LIGHT:
                    //                   IF cfg._4RUNNER_Mo1613=TRUE THEN
                    //                       IF Disp[IPC_MOTORE].stato<>LAVORO THEN
                    //                           OK_Condizione_Move_Dispositivo:=FALSE;
                    //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=IPC_MOTORE);
                    //                       END_IF;
                    //                   END_IF;
                    //           {
                    //           #endif  //Fine FOUR_RUNNER
                    //           }
                    //           {
                    //           #ifdef TAVOLI_SERPENTINE
                    //           }
                    //               TAVOLO_POST_ORIZZONTALE, TAVOLO_POST_ORIZZ_DX, TAVOLO_POST_ORIZZ_SX, TAVOLO_POST_ROTAZIONE:
                    //                   //Consentiti movimenti di chiusura tavoli verso la macchina solo con Z1 a 0° o 180°
                    //                   IF Stato=RIPOSO THEN
                    //                       IF ABS(Assi[Z1].PosAx.positioningState.actualPosition-0)>pm[Z1].ma_standstill_pos_tol AND
                    //                            ABS(Assi[Z1].PosAx.positioningState.actualPosition-180)>pm[Z1].ma_standstill_pos_tol THEN
                    //                           OK_Condizione_Move_Dispositivo:=FALSE;
                    //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Z1);
                    //                       END_IF;
                    //                   END_IF;
                    //               TAVOLO_ANT_ORIZZONTALE:
                    //                   IF Stato=RIPOSO THEN
                    //                       //Consentiti movimenti di chiusura tavolo verso la macchina solo con Z1 a 0° o 180°
                    //                       IF ABS(Assi[Z1].PosAx.positioningState.actualPosition-0)>pm[Z1].ma_standstill_pos_tol AND
                    //                            ABS(Assi[Z1].PosAx.positioningState.actualPosition-180)>pm[Z1].ma_standstill_pos_tol THEN
                    //                           OK_Condizione_Move_Dispositivo:=FALSE;
                    //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Z1);
                    //                       END_IF;
                    //                   END_IF;
                    //                   IF Stato=LAVORO THEN
                    //                       //Richiesta apertura tavolo orizzontale
                    //                       IF ReadDigitalInput(NumInput:=INP_TAVOLO_ANTERIORE_ALTO, Verifica:=TRUE)=FALSE THEN
                    //                           //Tavolo non è alto
                    //                           IF RunnerDXSX()=TRUE THEN
                    //                               IF Assi[Y1].PosAx.positioningState.actualPosition>1 AND Assi[Z1].PosAx.positioningState.actualPosition<179 THEN  
                    //                                   //Con DXSX in processo non si può muovere se l^asse Y1 non è a 0° o 180°
                    //                                   OK_Condizione_Move_Dispositivo:=FALSE;
                    //                                   set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Y1);
                    //                               END_IF;
                    //                           ELSE
                    //                               IF Assi[Y1].PosAx.positioningState.actualPosition>1 THEN  
                    //                                   //Non si può muovere se l^asse Y1 non è a 0° 
                    //                                   OK_Condizione_Move_Dispositivo:=FALSE;
                    //                                   set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Y1);
                    //                               END_IF;
                    //                           END_IF;
                    //                       END_IF;  
                    //                   END_IF;  
                    //               TAVOLO_ANT_ROTAZIONE:
                    //                   //Consentiti movimenti di chiusura tavoli verso la macchina solo con Z1 a 0° o 180°
                    //                   IF Stato=RIPOSO THEN
                    //                       IF ABS(Assi[Z1].PosAx.positioningState.actualPosition-0)>pm[Z1].ma_standstill_pos_tol AND
                    //                            ABS(Assi[Z1].PosAx.positioningState.actualPosition-180)>pm[Z1].ma_standstill_pos_tol THEN
                    //                           OK_Condizione_Move_Dispositivo:=FALSE;
                    //                           set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Z1);
                    //                       END_IF;
                    //                   END_IF;
                    //                   IF ReadDigitalInput(NumInput:=INP_TAVOLO_ANTERIORE_FUORI_INGOMBRO, Verifica:=TRUE)=TRUE THEN
                    //                       //Tavolo anteriore orizzontale aperto
                    //                       IF RunnerDXSX()=TRUE THEN
                    //                           IF Assi[Y1].PosAx.positioningState.actualPosition>1 AND Assi[Z1].PosAx.positioningState.actualPosition<179 THEN  
                    //                               //Con DXSX in processo non si può muovere se l^asse Y1 non è a 0° o 180°
                    //                               OK_Condizione_Move_Dispositivo:=FALSE;
                    //                               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Y1);
                    //                           END_IF;
                    //                       ELSE
                    //                           IF Assi[Y1].PosAx.positioningState.actualPosition>1 THEN  
                    //                               //Non si può muovere se l^asse Y1 non è a 0° 
                    //                               OK_Condizione_Move_Dispositivo:=FALSE;
                    //                               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=Y1);
                    //                           END_IF;
                    //                   END_IF;
                    //               END_IF;
                    //           {
                    //           #endif  //Fine TAVOLI_SERPENTINE
                    //           }
                    //       {
                    //       #else	//Fine RUNNER
                    //       }
                              //Inizio macchine NO RUNNER
                              PALETTA:
                                  PosAx:= AsSynchronousAxisRef(Axes[Z1].PosAx);
                                  IF ABS(PosAx^.actualPosition)>pm[Z1].ma_tolerance AND Stato=DEVICES_STATUS#RIPOSO THEN 
                                      OK_Condizione_Move_Dispositivo:=FALSE;
                                  END_IF;
                              SUPPORT_TIRANT:
                                  PosAx:= AsSynchronousAxisRef(Axes[X1].PosAx);
                                  IF PosAx^.actualPosition<=MAX(i_sys_MAX_X_SOSTEGNO_ALTO, cfg.QuotaAbbassoSostegnoTirante) THEN
                                      OK_Condizione_Move_Dispositivo:=FALSE;
                                      set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Axis:=X1);
                                  END_IF; 
                              CLAMP:
                                  IF ImpostaDiametroPinza(DiametroRichiesto:=programmi[PROGRAM_JOG].PinzaDiamMinMax)=FALSE THEN
                                      OK_Condizione_Move_Dispositivo:=FALSE;
                                      set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Axis:=CLAMP);
                                  END_IF;
                                //   {
                                //   #ifdef SMART_SMART_A
                                //   }
                                //       {
                                //       #ifdef SMART
                                //       }
                                //           {        
                                //           #ifndef BASCULANTE_ELETTRICO
                                //           }
                                //               BASCULANTE:
                                //                   IF i_sys_monitoring=1 AND ABS(Disp[BASCULANTE].limite2_lavoro-Disp[BASCULANTE].limite2_riposo)<0.1 THEN
                                //                       //Limiti calcolati da VGP sono uguali significa collisione e non si può movimentare il dispositivo
                                //                       OK_Condizione_Move_Dispositivo:=FALSE;
                                //                       set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=BASCULANTE);
                                //                   ELSE
                                //                       IF Assi[V1].PosAx.positioningState.actualPosition-cfg.i_sys_postestabassaxmin>pm[V1].ma_standstill_pos_tol THEN
                                //                           iret:=Push_error(Code:=ERR_TESTA_NO_BASSA);
                                //                           OK_Condizione_Move_Dispositivo:=FALSE;
                                //                       END_IF;
                                //                   END_IF;
                                //           {
                                //           #endif  //Fine NO BASCULANTE_ELETTRICO
                                //           }
                                //       {
                                //       #endif  //Fine SMART
                                //       }
                                //       LOCKING_DXSX:
                                //           IF i_sys_monitoring=1 AND ABS(Disp[LOCKING_DXSX].limite2_lavoro-Disp[LOCKING_DXSX].limite2_riposo)<0.1 THEN
                                //               //Limiti calcolati da VGP sono uguali significa collisione e non si può movimentare il dispositivo
                                //               OK_Condizione_Move_Dispositivo:=FALSE;
                                //               set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Asse:=LOCKING_DXSX);
                                //           ELSE
                                //               {        
                                //               #ifdef SMART
                                //               }
                                //                   IF cfg.tiposlittamobile=TIPO_SLITTA_ELETTRICA THEN
                                //                       IF Clk_plus AND (Assi[W4].PosAx.positioningState.actualPosition>0) THEN
                                //                           //Richiesto tassello a SX, non si può fare con braccio minore di 45°
                                //                           IF Assi[Y1].PosAx.positioningState.actualPosition<45 THEN
                                //                               iret:=Push_error(Code:=ERR_SCOMPARSA_SLITTA);
                                //                               OK_Condizione_Move_Dispositivo:=FALSE;
                                //                           END_IF;
                                //                       ELSE
                                //                           IF Clk_minus AND (Assi[W4].PosAx.positioningState.actualPosition<0) THEN
                                //                               //Richiesto tassello a DX, non si può fare con braccio di curvatura supeiore a 145°
                                //                               IF Assi[Y1].PosAx.positioningState.actualPosition>POSY_SCOMPBLOCC_SX THEN
                                //                                   iret:=Push_error(Code:=ERR_SCOMPARSA_SLITTA);
                                //                                   OK_Condizione_Move_Dispositivo:=FALSE;
                                //                               END_IF;
                                //                           END_IF;
                                //                       END_IF;
                                //                   ELSE
                                //                       IF Disp[DISAPPEARANCE_ROLLERS].enable=TRUE THEN
                                //                           IF Disp[DISAPPEARANCE_ROLLERS].stato<>LAVORO THEN
                                //                               //Rulli non in posizione di scomparsa, il movimento del tassello morsa DX/SX è
                                //                               //condizionato dalla posizione del braccio di curvatura e dal tasto premuto
                                //                               IF Clk_plus AND (Assi[W4].PosAx.positioningState.actualPosition>0) THEN
                                //                                   //Richiesto tassello a SX, non si può fare con braccio minore di 45°
                                //                                   IF Assi[Y1].PosAx.positioningState.actualPosition<45 THEN
                                //                                       iret:=Push_error(Code:=ERR_SCOMPARSA_SLITTA);
                                //                                       OK_Condizione_Move_Dispositivo:=FALSE;
                                //                                   END_IF;
                                //                               ELSE
                                //                                   IF Clk_minus AND (Assi[W4].PosAx.positioningState.actualPosition<0) THEN
                                //                                       //Richiesto tassello a DX, non si può fare con braccio di curvatura supeiore a 145°
                                //                                       IF Assi[Y1].PosAx.positioningState.actualPosition>POSY_SCOMPBLOCC_SX THEN
                                //                                           iret:=Push_error(Code:=ERR_SCOMPARSA_SLITTA);
                                //                                           OK_Condizione_Move_Dispositivo:=FALSE;
                                //                                       END_IF;
                                //                                   END_IF;
                                //                               END_IF;
                                //                           END_IF;  
                                //                       END_IF;  
                                //                   END_IF;
                                //               {        
                                //               #endif	//Fine SMART
                                //               }
                                //               {        
                                //               #ifdef SMART_A
                                //               }
                                //                   IF Assi[U4].PosAx.positioningState.actualPosition>U4_Scomparsa+0.2 THEN
                                //                       //Rulli non in posizione di scomparsa, il movimento del tassello morsa DX/SX è
                                //                       //condizionato dalla posizione del braccio di curvatura e dal tasto premuto
                                //                       IF Clk_plus AND (Assi[W4].PosAx.positioningState.actualPosition>0) THEN
                                //                           //Richiesto tassello a SX, non si può fare con braccio minore di 45°
                                //                           IF Assi[Y1].PosAx.positioningState.actualPosition<45 THEN
                                //                               iret:=Push_error(Code:=ERR_SCOMPARSA_SLITTA);
                                //                               OK_Condizione_Move_Dispositivo:=FALSE;
                                //                           END_IF;
                                //                       ELSE
                                //                           IF Clk_minus AND (Assi[W4].PosAx.positioningState.actualPosition<0) THEN
                                //                               //Richiesto tassello a DX, non si può fare con braccio di curvatura supeiore a 145°
                                //                               IF Assi[Y1].PosAx.positioningState.actualPosition>POSY_SCOMPBLOCC_SX THEN
                                //                                   iret:=Push_error(Code:=ERR_SCOMPARSA_SLITTA);
                                //                                   OK_Condizione_Move_Dispositivo:=FALSE;
                                //                               END_IF;
                                //                           END_IF;
                                //                       END_IF;
                                //                   END_IF;
                                //               {        
                                //               #endif  //Fine SMART_A
                                //               }
                                        //   END_IF;
                                      DISAPPEARANCE_ROLLERS:
                                          IF i_sys_monitoring=1 AND ABS(Disp[DISAPPEARANCE_ROLLERS].limite2_lavoro-Disp[DISAPPEARANCE_ROLLERS].limite2_riposo)<0.1 THEN
                                              //Limiti calcolati da VGP sono uguali significa collisione e non si può movimentare il dispositivo
                                              OK_Condizione_Move_Dispositivo:=FALSE;
                                              set_trace(Code:=TRC_SAFETY_JOG_DEVICES, Axis:=DISAPPEARANCE_ROLLERS);
                                          ELSE
                                              IF (Clk_minus OR Clk_plus) AND Disp[DISAPPEARANCE_ROLLERS].enable=TRUE THEN 
                                                    PosAx:= AsSynchronousAxisRef(Axes[W4].PosAx);
                                                  IF PosAx^.actualPosition > 0 THEN
                                                      //Rulli lato DX si possono portare in posizione di lavoro solo se l^asse è sul limite positivo
                                                      IF PosAx^.actualPosition < PosAx^.PositionLimits_SW.MinPosition-1 THEN
                                                          iret:=Push_error(Code:=ERR_APRIRE_SLITTA_SPALLAMENTO);
                                                          OK_Condizione_Move_Dispositivo:=FALSE;
                                                      END_IF;
                                                  ELSE
                                                      //Rulli lato SX si possono portare in posizione di lavoro solo se l^asse è sul limite negativo
                                                      IF PosAx^.actualPosition > PosAx^.PositionLimits_SW.MinPosition+1 THEN
                                                          iret:=Push_error(Code:=ERR_APRIRE_SLITTA_SPALLAMENTO);
                                                          OK_Condizione_Move_Dispositivo:=FALSE;
                                                      END_IF;
                                              END_IF;
                                              IF Disp[LOCKING_DXSX].enable=TRUE AND cfg.bloccoscomparsabloccaggio=-1 THEN
                                                PosAx:= AsSynchronousAxisRef(Axes[Y1].PosAx);
                                                  IF PosAx^.actualPosition<POSY_SCOMPBLOCC_DX THEN
                                                      IF Disp[LOCKING_DXSX].stato<>DEVICES_STATUS#RIPOSO THEN
                                                          //Richiesta rulli in posizione di lavoro non si può
                                                          //eseguire con posizione Y inferiore a 35° e tassello
                                                          //bloccaggio in posizione diversa da DX
                                                          iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                                                          OK_Condizione_Move_Dispositivo:=FALSE;
                                                      END_IF;
                                                  ELSE
                                                      IF PosAx^.actualPosition>POSY_SCOMPBLOCC_SX THEN
                                                          IF Disp[LOCKING_DXSX].stato<>DEVICES_STATUS#LAVORO THEN
                                                              //Richiesta rulli in posizione di lavoro non si può
                                                              //eseguire con posizione Y superiore a 145° e tassello
                                                              //bloccaggio in posizione diversa da SX
                                                              iret:=Push_error(Code:=ERR_TASSELLO_BLOCCAGGIO_DXSX);
                                                              OK_Condizione_Move_Dispositivo:=FALSE;
                                                          END_IF;
                                                      END_IF;
                                                  END_IF;
                                              END_IF;
                                          END_IF;
                                      END_IF;
                                //   {
                                //   #endif  //Fine SMART_SMART_A
                                //   }
                                //   {
                                //   #ifdef ELECTL_SAFETY %%%%
                                //   }
                                //       UNITA_FORATURA:
                                //           IF ReadDigitalInput(NumInput:=INP_POMPA_AVVIATA_FORATURA, Verifica:=1) = FALSE OR
                                //                ReadDigitalInput(NumInput:=INP_LIVELLO_OLIO_FORATURA, Verifica:=1) = FALSE THEN
                                //               iret:=Push_Error(Code:=ERR_POMPA_NON_PRONTA);
                                //               OK_Condizione_Move_Dispositivo:=FALSE;
                                //           END_IF;
                                //       FORATURA_IDRAULICA:
                                //           IF ReadDigitalInput(NumInput:=INP_POMPA_AVVIATA_FORATURA, Verifica:=1) = FALSE OR
                                //                ReadDigitalInput(NumInput:=INP_LIVELLO_OLIO_FORATURA, Verifica:=1) = FALSE THEN
                                //               iret:=Push_Error(Code:=ERR_POMPA_NON_PRONTA);
                                //               OK_Condizione_Move_Dispositivo:=FALSE;
                                //           END_IF;
                                //   {
                                //   #endif FINE ELECTL_SAFETY
                                //   }
                        //           {
                        //           #ifdef ELECTMRL
                        //           }
                        //               {
                        //               #ifdef TAVOLI_ELETTRICI
                        //               }
                        //                   TAVOLO_PNEUMATICO: 
                        //                       IF assi[Y1].PosAx.positioningState.actualPosition <= 90 THEN
                        //                           iret:=Push_error(Code:=ERR_BRACCIO_NON_OPPOSTO);
                        //                           Ok_condizione_move_dispositivo:=FALSE;
                        //                       END_IF;
                        //               {
                        //               #endif //FINE TAVOLI_ELETTRICI
                        //               }
                        //           {
                        //           #endif //FINE ELECTMRL
                        //           }
                        //   {
                        //   #endif  //Fine NO RUNNER
                        //   }
                  END_CASE;
        //       {
        //       #endif //FINE NO PLAUNO_LIGHT
        //       }
        //       ;
        //   {
        //   #endif  //Fine NO EFLEX_DH4010
        //   }
      END_IF;
  END_FUNCTION
  
  FUNCTION ImpostaDiametroPinza:BOOL

    VAR_EXTERNAL
        cfg                             :_TYPE.Config_Struct;
        i_sys_OPMODE                    : INT;
        R_Plus		                    :R_TRIG;
        Clk_Minus	                    :BOOL;
    END_VAR

    VAR_EXTERNAL CONSTANT
        JOGMODE                               :INT;
    END_VAR

    VAR_INPUT
      DiametroRichiesto :SINT;  //1=Dt min, 2=DT max
    END_VAR
    VAR
      Esito                         :BOOL;
      dret                          :DINT;
    END_VAR
    // {
    // #ifndef ORBITALE %%%%
    // }
    //   ImpostaDiametroPinza:=FALSE;
    //   CASE cfg.i_sys_pinza_due_diametri OF
    //           1://Prima versione con cilindro non Festo
    //               IF i_sys_OPMODE=JOGMODE THEN
    //                   //Siamo in JOG
    //                   IF (R_Plus.Q AND NOT Clk_minus) OR (R_Minus.Q AND NOT Clk_plus) THEN
    //                       IF DiametroRichiesto=1 THEN
    //                           //Richiesto diametro minimo
    //                           Set_Disp(Ndisp:=PINZA, Stato:=RIPOSO, TimerRisparmio:=TRUE);	//Apertura pinza
    //                           //dret:=_waitTime(T#100ms);
    //                           set_trace(Code:=TRC_ATTESA_OFF_PINZA_DT_MAX);
    //                           WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_PINZA_DT_MAX, Value:=FALSE);
    //                           IF ReadDigitalInput(NumInput:=INP_PINZA_DIAMETRO_MAX, Verifica:=TRUE)=FALSE THEN
    //                               ImpostaDiametroPinza:=TRUE;
    //                           END_IF;
    //                       ELSE
    //                           //Richiesto diametro massimo
    //                           Set_Disp(Ndisp:=PINZA, Stato:=RIPOSO, TimerRisparmio:=TRUE);	//Apertura pinza
    //                           dret:=_waitTime(T#100ms);
    //                           set_trace(Code:=TRC_ATTESA_ON_PINZA_DT_MAX);
    //                           WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_PINZA_DT_MAX, Value:=TRUE);
    //                           IF ReadDigitalInput(NumInput:=INP_PINZA_DIAMETRO_MAX, Verifica:=TRUE)=TRUE THEN
    //                               ImpostaDiametroPinza:=TRUE;
    //                           END_IF;
    //                       END_IF;
    //                   END_IF;
    //               ELSE
    //                   //Siamo in automatico
    //                   IF DiametroRichiesto=1 THEN
    //                       //Richiesto diametro minimo
    //                       Set_Disp(Ndisp:=PINZA, Stato:=RIPOSO, TimerRisparmio:=TRUE);	//Apertura pinza
    //                       dret:=_waitTime(T#100ms);
    //                       set_trace(Code:=TRC_ATTESA_OFF_PINZA_DT_MAX);
    //                       WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_PINZA_DT_MAX, Value:=FALSE);
    //                       WHILE ReadDigitalInput(NumInput:=INP_PINZA_DIAMETRO_MAX, Verifica:=TRUE)=TRUE DO
    //                           //Attesa disattivazione finecorsa selezione diametro massimo
    //                           dret:=_waitTime(T#0ms);
    //                       END_WHILE;
    //                       ImpostaDiametroPinza:=TRUE;
    //                   ELSE
    //                       //Richiesto diametro massimo
    //                       Set_Disp(Ndisp:=PINZA, Stato:=RIPOSO, TimerRisparmio:=TRUE);	//Apertura pinza
    //                       dret:=_waitTime(T#100ms);
    //                       set_trace(Code:=TRC_ATTESA_ON_PINZA_DT_MAX);
    //                       WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_PINZA_DT_MAX, Value:=TRUE);
    //                       WHILE ReadDigitalInput(NumInput:=INP_PINZA_DIAMETRO_MAX, Verifica:=TRUE)=FALSE DO
    //                           //Attesa attivazione finecorsa selezione diametro massimo
    //                           dret:=_waitTime(T#0ms);
    //                       END_WHILE;
    //                       ImpostaDiametroPinza:=TRUE;
    //                   END_IF;
    //               END_IF;
    //           2://Seconda versione con cilindro Festo
    //               IF i_sys_OPMODE=JOGMODE THEN
    //                   //Siamo in JOG
    //                   IF (R_Plus.Q AND NOT Clk_minus) OR (R_Minus.Q AND NOT Clk_plus) THEN
    //                       IF DiametroRichiesto=1 THEN
    //                           //Richiesto diametro minimo
    //                           Set_Disp(Ndisp:=PINZA, Stato:=RIPOSO, TimerRisparmio:=TRUE);	//Apertura pinza
    //                           WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_PINZA_DT_MAX, Value:=FALSE);
    //                           ImpostaDiametroPinza:=TRUE;
    //                       ELSE
    //                           //Richiesto diametro massimo
    //                           Set_Disp(Ndisp:=PINZA, Stato:=RIPOSO, TimerRisparmio:=TRUE);	//Apertura pinza
    //                           WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_PINZA_DT_MAX, Value:=TRUE);
    //                           ImpostaDiametroPinza:=TRUE;
    //                       END_IF;
    //                   END_IF;
    //               ELSE
    //                   //Siamo in automatico
    //                   IF DiametroRichiesto=1 THEN
    //                       //Richiesto diametro minimo
    //                       Set_Disp(Ndisp:=PINZA, Stato:=RIPOSO, TimerRisparmio:=TRUE);	//Apertura pinza
    //                       WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_PINZA_DT_MAX, Value:=FALSE);
    //                       ImpostaDiametroPinza:=TRUE;
    //                   ELSE
    //                       //Richiesto diametro massimo
    //                       Set_Disp(Ndisp:=PINZA, Stato:=RIPOSO, TimerRisparmio:=TRUE);	//Apertura pinza
    //                       set_trace(Code:=TRC_ATTESA_ON_PINZA_DT_MAX);
    //                       WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_PINZA_DT_MAX, Value:=TRUE);
    //                       ImpostaDiametroPinza:=TRUE;
    //                   END_IF;
    //               END_IF;
    //   ELSE
    //     ImpostaDiametroPinza:=TRUE;
    //   END_CASE;
    //   IF ImpostaDiametroPinza=TRUE THEN
    //     reset_trace(Code:=TRC_ATTESA_ON_PINZA_DT_MAX);
    //     reset_trace(Code:=TRC_ATTESA_OFF_PINZA_DT_MAX);
    //   END_IF;
    // {
    // #endif  //Fine NO ORBITALE
    // }
    ;
  END_FUNCTION
  
  {::ifdef AL_BLM}
    FUNCTION_BLOCK FB_AL5CatenaDX
    
        VAR_EXTERNAL CONSTANT
            OUT_AL5_CATENA_DX_AVANTI                      :INT;
            OUT_AL5_CATENA_DX_INDIETRO                    :INT; 
        END_VAR

      VAR_INPUT
        Stato  :INT;
      END_VAR
      VAR
        dret  :DINT;
              
              CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
              VerificaStop	:FB_VerificaStop;
      END_VAR  
          VAR_OUTPUT
              OutputState	:INT;
          END_VAR
          CASE CURRENT_STATE OF
            GENERAL_STATE#STATE_INIT:
                  CASE Stato OF
                    DEVICES_STATUS#ARRESTO:
                          WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_DX_AVANTI, Value:=FALSE);
                          WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_DX_INDIETRO, Value:=FALSE);
                          CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          
                    DEVICES_STATUS#RIPOSO:
                          IF (VerificaStop.OutputState<>GENERAL_STATE#STATE_END) THEN
                              VerificaStop();
                          ELSE
                              VerificaStop();
                              WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_DX_AVANTI, Value:=FALSE);
                              WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_DX_INDIETRO, Value:=TRUE);
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          END_IF;
  
                    DEVICES_STATUS#LAVORO:
                          IF (VerificaStop.OutputState<>GENERAL_STATE#STATE_END) THEN
                              VerificaStop();
                          ELSE
                              VerificaStop();
                              WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_DX_AVANTI, Value:=TRUE);
                              WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_DX_INDIETRO, Value:=FALSE);
                              CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          END_IF;
                  END_CASE;  
              
            GENERAL_STATE#STATE_END:
                  CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
                  
          END_CASE;
          OutputState:=CURRENT_STATE;
    END_FUNCTION_BLOCK
  
    FUNCTION_BLOCK FB_AL5CatenaSX

        VAR_EXTERNAL CONSTANT
            OUT_AL5_CATENA_SX_AVANTI                      :INT;
            OUT_AL5_CATENA_SX_INDIETRO                    :INT;
        END_VAR

        VAR_INPUT
          Stato  :INT;
        END_VAR
        VAR
          dret  :DINT;
                
                CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
                VerificaStop	:FB_VerificaStop;
        END_VAR  
            VAR_OUTPUT
                OutputState	:INT;
            END_VAR
            CASE CURRENT_STATE OF
                GENERAL_STATE#STATE_INIT:
                    CASE Stato OF
                        DEVICES_STATUS#ARRESTO:
                            WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_SX_AVANTI, Value:=FALSE);
                            WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_SX_INDIETRO, Value:=FALSE);
                            CURRENT_STATE:=GENERAL_STATE#STATE_END;
                            
                        DEVICES_STATUS#RIPOSO:
                            IF (VerificaStop.OutputState<>GENERAL_STATE#STATE_END) THEN
                                VerificaStop();
                            ELSE
                                VerificaStop();
                                WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_SX_AVANTI, Value:=FALSE);
                                WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_SX_INDIETRO, Value:=TRUE);
                                CURRENT_STATE:=GENERAL_STATE#STATE_END;
                            END_IF;
    
                        DEVICES_STATUS#LAVORO:
                            IF (VerificaStop.OutputState<>GENERAL_STATE#STATE_END) THEN
                                VerificaStop();
                            ELSE
                                VerificaStop();
                                WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_SX_AVANTI, Value:=TRUE);
                                WriteDigitalOutput(NumOutput:=OUT_AL5_CATENA_SX_INDIETRO, Value:=FALSE);
                                CURRENT_STATE:=GENERAL_STATE#STATE_END;
                            END_IF;
                    END_CASE;  
                
                GENERAL_STATE#STATE_END:
                    CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
                    
            END_CASE;
            OutputState:=CURRENT_STATE;
      END_FUNCTION_BLOCK
      {::endif}
 END_NAMESPACE