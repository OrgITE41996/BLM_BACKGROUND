USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING System.BitAccess;
USING System.Math;
USING ENABLEPROGRAM;
USING ENABLEAX_FUNCTIONS;
USING  _TYPE;
USING EnableAxFunction;
USING _FC_IO_FC;
USING _CONST_VAR_DEF;
USING _STATE_CONST;
USING _UTILITIES_FB;
USING _FC_FB;
USING _FB;
USING FC_N;


NAMESPACE _FC_FC

/********************************************************************************************************
 * CHECKS IF THE MOTOR CONFIGURATION TRANSFERRED FROM VGP IS CONGRUENT WITH THE ONE ACTIVE IN SIMOTION. *
 *                              IF NOT, IT INDICATES THE INCONGRUENT AXIS.                              *
 ********************************************************************************************************/

FUNCTION ShadowMotorCheck : INT

    VAR_EXTERNAL
		pm                      	        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
		Axes                                :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
        i_sys_ACCESSLEVEL                   : INT ;	// LIVELLO DI ACCESSO 
    END_VAR

	VAR_EXTERNAL CONSTANT
        PROGRAM_JOG                      	:SINT;
        NON_DEACTIVATABLE_AXIS              :SINT;
        ACTIVE_DEACTIVATABLE_AXIS           :SINT;
        INACTIVE_DEACTIVATABLE_AXIS         :SINT;
        MAINTENANCE_ACCESS   	            :SINT;
        TRC_EXECUTE_MOTOR_SETTING           :INT; // Traces from 200 to 299 are to be used for device waiting, BlmMotion displays 
        TRC_MOTOR_DXSX_4RUNNER              :INT; 
        TRC_MOTOR_DRILL                     :INT;
        TRC_MOTOR_4RTL                      :INT;
        TRC_MOTOR_4RC                       :INT;
        TRC_MOTOR_SLIDE_RIGHT               :INT; 
        TRC_MOTOR_SLIDE                     :INT; 
        TRC_MOTOR_SLIDE_LEFT                :INT; 
        TRC_MOTOR_AL5                       :INT;
        W4                                  :INT;  //Rollers
        V2                                  :INT;  //Flow-Drill
        A1                                  :INT;  //Rotation 4RTL/"Rotazione 4RC
        A2                                  :INT;  //Rotation 4RC
        B1                                  :INT;  //Rotation 4RIPC
        B2                                  :INT;  //Rotation 4RIPC
        U4                                  :INT;  //Mobile slide
        U5                                  :INT;  ////Extra mobile sled
        L1                                  :INT;  //AL5
    END_VAR

    VAR 
        i              :INT;
        TraceData      :BOOL;
        NumeroStati    :INT;
      END_VAR


      ShadowMotorCheck:=_FIRST_AXIS#FIRST_AXIS-1;
      TraceData:=FALSE;
      (*{
      #ifndef EFLEX
      }
            {
            #ifdef DH4010
            }*)
               NumeroStati:=Max (GetNumberOfStates(Tabella:=TABLE#TABLE_LX , Programma:=PROGRAM_JOG),
                                GetNumberOfStates(Tabella:=TABLE#TABLE_RX , Programma:=PROGRAM_JOG));
            (*{  
            #else	//Fine DH4010
            }*)
                NumeroStati:=GetNumberOfStates(Tabella:=-SINT#1, Programma:=PROGRAM_JOG);
            (*{
            #endif	//Fine NO DH4010
            }*)
            IF NumeroStati>0 THEN
                FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1 DO
                    (*{
                    #ifdef ASSE_Z2
                    }*)
                        IF pm[i].ma_AsseInVGP=FALSE THEN
                            //Non si verifica la congruenza per gli assi di cui VGP non trasmette l^abilitazione
                            Axes[i].ma_asse_abilitato_shadow:=pm[i].ma_asse_abilitato;
                            Axes[i].ma_asse_installato_shadow:=pm[i].ma_asse_installato;
                        END_IF;
                    (*{
                    #endif	//Fine ASSE_Z2
                    }*)
                    IF (pm[i].ma_asse_installato=INACTIVE_DEACTIVATABLE_AXIS AND
                            Axes[i].ma_asse_installato_shadow<>INACTIVE_DEACTIVATABLE_AXIS) OR
                         (pm[i].ma_asse_installato=NON_DEACTIVATABLE_AXIS AND
                            Axes[i].ma_asse_installato_shadow=INACTIVE_DEACTIVATABLE_AXIS) OR
                         (pm[i].ma_asse_installato=ACTIVE_DEACTIVATABLE_AXIS AND
                            Axes[i].ma_asse_installato_shadow=INACTIVE_DEACTIVATABLE_AXIS) OR
                         (pm[i].ma_asse_abilitato<>Axes[i].ma_asse_abilitato_shadow AND 
                            pm[i].ma_Confronta_Asse_Abilitato=TRUE) THEN
                        //Tutti gli assi debbono avere "ma_asse_installato" e "ma_asse_installato_shadow" congruenti, 
                        //cioè a 0 o 1, oppure entrambi a 2. Il confronto fra "ma_asse_abilitato" e 
                        //"ma_asse_abilitato_shadow" deve essere eseguito solo per gli assi in cui VGP non esegue la
                        //disabilitazione "ma_Confronta_Asse_Abilitato" impostato a TRUE, al momento solo A1 e A2
                        ShadowMotorCheck:=i;
                       (* IF _getStateOfTaskId(_task.motiontask_CAMBIOCONHARD)=TASK_STATE_STOPPED THEN
                            Init_Trace();
                        END_IF;*)
                        IF i_sys_Accesslevel<MAINTENANCE_ACCESS OR CheckIOAddresses()=FALSE THEN
                            //Trace 242 per operatori BLM, oppure quando ritentive sono KO
                            set_trace(Code:=TRC_EXECUTE_MOTOR_SETTING);
                            TraceData:=TRUE;
                        ELSE
                            (*{
                            #ifndef PLAUNO_LIGHT
                            }
                                {
                                #ifndef DH4010
                                }*)
                                    //Trace singola per operatori finali
                                     CASE i OF
                                        (*{
                                        #ifdef ORBITALE
                                        }
                                            {
                                            #ifdef FOUR_RUNNER_PLAUNO
                                            }
                                                {
                                                #ifdef SPALLAMENTO_DX
                                                }*)
                                                    W4:
                                                        set_trace(Code:=TRC_MOTOR_DXSX_4RUNNER);
                                                        TraceData:=TRUE;
                                                (*{
                                                #endif	//Fine SPALLAMENTO_DX
                                                }
                                                {
                                                #ifdef FOUR_RUNNER
                                                }*)
                                                    V2:
                                                        set_trace(Code:=TRC_MOTOR_DRILL);
                                                        TraceData:=TRUE;
                                                (*{
                                                #endif	//Fine FOUR_RUNNER
                                                }
                                            {
                                            #endif	//Fine FOUR_RUNNER_PLAUNO
                                            }
                                            {
                                            #ifdef RUNNER
                                            }*)
                                                A1:
                                                    set_trace(Code:=TRC_MOTOR_4RTL);
                                                    TraceData:=TRUE;
                                                    (*{
                                                    #ifdef A1_A2
                                                    }*)
                                                    A2:
                                                            set_trace(Code:=TRC_MOTOR_4RC);
                                                            TraceData:=TRUE;
                                                    (*{
                                                    #endif	//Fine A1_A2
                                                    }
                                                    {
                                                    #ifdef THREE_RUNNER
                                                    }
                                                        {
                                                        #ifdef B1_B2
                                                        }*)
                                                             B1,B2:
                                                                set_trace(Code:=TRC_MOTOR_4RC);
                                                                TraceData:=TRUE;
                                                       (* {
                                                        #endif	//Fine B1_B2
                                                        }
                                                    {
                                                    #endif	//Fine THREE_RUNNER
                                                    }
                                            {    
                                            #endif	//Fine RUNNER
                                            }
                                        {
                                        #else	//Fine ORBITALE
                                        }
                                            //Curvatubi lineari
                                            {
                                            #ifdef SLITTA_MOBILE_DX
                                            }
                                                {
                                                #ifdef DXSX_PROCESSO
                                                }*)
                                                    U4:
                                                        set_trace(Code:=TRC_MOTOR_SLIDE_RIGHT);
                                                        TraceData:=TRUE;
                                                (*{
                                                #else  //Fine DXSX_PROCESSO
                                                }*)
                                                   (* U4:
                                                        set_trace(Code:=TRC_MOTOR_SLIDE);
                                                        TraceData:=TRUE;*)
                                                (*{
                                                #endif  //Fine NO DXSX_PROCESSO
                                                }
                                            {
                                            #endif  //Fine SLITTA_MOBILE_DX
                                            }
                                            {
                                            #ifdef SLITTA_MOBILE_SX
                                            }*)
                                                U5:
                                                    set_trace(Code:=TRC_MOTOR_SLIDE_LEFT);
                                                    TraceData:=TRUE;
                                            (*{
                                            #endif  //Fine SLITTA_MOBILE_SX
                                            }
                                            {
                                            #ifdef AL_BLM
                                            }
                                                 L1:
                                                    set_trace(Code:=TRC_MOTOR_AL5);
                                                    TraceData:=TRUE;
                                            (*{
                                            #endif  //Fine AL_BLM
                                            }
                                        {
                                        #endif	//Fine NO ORBITALE
                                        }*)
                                    END_CASE;
                                (*{
                                #endif //FINE NO DH4010
                                }
                            {
                            #endif //FINE NO PLAUNO_LIGHT
                            }*)
                            EXIT;
                        END_IF;
                    END_IF;
                    ;
                END_FOR;
            END_IF;
            (*IF ShadowMotorCheck<_FIRST_AXIS#FIRST_AXIS OR _getStateOfTaskId(_task.motiontask_CAMBIOCONHARD)<>TASK_STATE_STOPPED THEN
                reset_trace(Code:=TRC_MOTOR_DXSX_4RUNNER);
                reset_trace(Code:=TRC_MOTOR_4RTL);
                reset_trace(Code:=TRC_MOTOR_4RC);
                reset_trace(Code:=TRC_MOTOR_SLIDE_RIGHT);
                reset_trace(Code:=TRC_MOTOR_SLIDE_LEFT);
                reset_trace(Code:=TRC_MOTOR_AL5);
            END_IF;
            IF (ShadowMotorCheck>=_FIRST_AXIS#FIRST_AXIS AND TraceData=FALSE) OR CheckIOAddresses()=FALSE THEN
                //In caso non ci sia un^apposita trace si fornisce quella di impostazione totale, oppure se le ritentive sono KO
                set_trace(Code:=TRC_EXECUTE_MOTOR_SETTING);
            END_IF;
        (*{
        #endif	//Fine NO EFLEX
        }*)
    
END_FUNCTION

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Verifica che almeno un indirizzo fisico degli I/O digitali sia programmato, altrimenti esce con errore
FUNCTION CheckIOAddresses:BOOL

    VAR_EXTERNAL
        IAddress                             : ARRAY [0..MAXDIGITALINPUT] OF Digital_Input;
        OAddress                             : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
    END_VAR
    VAR_EXTERNAL CONSTANT
        MAXDIGITALINPUT                     :INT;
        MAXDIGITALOUTPUT                    :INT;
    END_VAR

    VAR
      i   :DINT;
    END_VAR

    CheckIOAddresses:=FALSE;
    FOR i:=0 TO MAXDIGITALINPUT DO
      IF IAddress[i].bit>0 THEN 
        CheckIOAddresses:=TRUE;
        EXIT;
      END_IF;    
    END_FOR;
    FOR i:=0 TO MAXDIGITALOUTPUT DO
      IF OAddress[i].bit>0 THEN 
        CheckIOAddresses:=TRUE;
        EXIT;
      END_IF;    
    END_FOR;
  END_FUNCTION

/**********************************************************************************************
 * DEPENDING ON THE TUBE DIAMETER, SET THE PRECOMMAND FACTOR TO SECOND ENCODER OF THE X^AXIS1 *
 **********************************************************************************************/
FUNCTION CalculateFactorPrecommand

    VAR_EXTERNAL
        i_sys_OPMODE                    : INT;	// MODO OPERATIVO
                                            // 0 = JOG
                                            // 1 = REFERENCE
                                            // 2 = MDA
                                            // 3 = AUT
        i_sys_current_prog	            :INT; // Programma corrente
        Programmi                       :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
        RifDtPrecommand                 :REAL; //Reference diameter on which the reference precommand is calculated
        RefPrecommand                   :REAL; //Precommand matched to reference diameter
        SlopePrecommand                 :REAL; //Slope for calculating the precommand as a function of the diameter
    END_VAR

    VAR_EXTERNAL CONSTANT
        X1                     :INT;    //Cart X
        X6                     :INT;
    END_VAR

    VAR
      PosAxis         :REF_TO TO_SynchronousAxis;
      DiameterTube    :REAL;
      ProgrammaAttivo :DINT;
      Differenza      :REAL;
      Fattore         :REAL;
      Axis            :INT;
    END_VAR
   (* {
    #ifdef ORBITALE
    }
      {
      #ifdef DH4010
      }*)
        Axis:=X6;
      (*{
      #else  //Fine DH4010
      }*)
        Axis:=X1;
      (*{
      #endif  //Fine NO DH4010
      }*)
      IF i_sys_OpMode<>PI_COMMANDS_CODE#AUTOMODE THEN
        ProgrammaAttivo:=1;
      ELSE
        ProgrammaAttivo:=I_SYS_Current_Prog;
      END_IF;
      //Calcolo il diametro del tubo/filo come media fra i due per avvicinarmi all^eventuale tubo
      //non tondo
      DiameterTube:=(programmi[ProgrammaAttivo].Diameter_Tube+programmi[ProgrammaAttivo].Diameter_Tube_B)/2;
      IF DiameterTube>0 AND RifDtPrecommand>0 THEN
        //Calcolo la differenza con il diametro di riferimento
        Differenza:=DiameterTube-RifDtPrecommand;
        //Impostazione del fattore di precomando
        Fattore:=(Differenza*SlopePrecommand)+RefPrecommand;
        PosAxis:=AsSynchronousAxisRef(Axes[Axis].PosAx);
        PosAxis^.PositionControl.Kpc:=Fattore;
      END_IF;
    (*{
    #endif  //Fine ORBITALE
    }*)
    ;
  END_FUNCTION

/***************************************************************************************
 * FOR ONLY MACHINES THAT HAVE L1^AXIS PRESENT, IT EXITS WITH FALSE WHEN THE PARAMETER *
 *                                INPUT PARAMETER IS L1                                *
 ***************************************************************************************/

  FUNCTION NoTestL1:BOOL

    VAR_EXTERNAL CONSTANT
        L1 :    INT;  //AL5
    END_VAR

    VAR_INPUT
      Axis  :INT;
    END_VAR
    NoTestL1:=TRUE;
      (*{
      #ifdef AL_BLM
      }*)
          IF Axis=L1 THEN
              NoTestL1:=FALSE;
          END_IF;
      (*{
      #endif    //Fine AL_BLM
      }*)
  END_FUNCTION

/*******************************************************************************************
 * FOR ONLY MACHINES THAT HAVE THE^AXIS Z2 PRESENT, IT EXITS WITH FALSE WHEN THE PARAMETER *
 *                                  INPUT PARAMETER IS Z2                                  *
 *******************************************************************************************/

 FUNCTION NoTestZ2:BOOL
  
VAR_EXTERNAL CONSTANT
    Z2  :INT;//Welding research on AL5
END_VAR

 VAR_INPUT
    Axis  :INT;
  END_VAR
  NoTestZ2:=TRUE;
    (*{
    #ifdef ASSE_Z2
    }*)
    IF Axis=Z2 THEN
      NoTestZ2:=FALSE;
    END_IF;
  (*{
  #endif    //Fine ASSE_Z2
  }*)
END_FUNCTION

/******************************************************************************************
 * RETURNS WHETHER THE AXISNUM AXIS IS AT THIS TIME CONFIGURED TO USE AN ABSOLUTE ENCODER *
 ******************************************************************************************/

FUNCTION GetEncoderAbsolute: BOOL

  VAR_EXTERNAL
    Axes                            		:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    pm                                  : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters; 
  END_VAR
  
  VAR_EXTERNAL CONSTANT
    INACTIVE_DEACTIVATABLE_AXIS       :SINT;
  END_VAR

  VAR_INPUT
    AxisNum : INT;
  END_VAR
  VAR
    MyPosAx : REF_TO TO_SynchronousAxis;
    Numencoder : UDINT; 
  END_VAR
  MyPosAx :=  AsSynchronousAxisRef(Axes[AxisNum].PosAx);
  IF MyPosAx^.VirtualAxis.Mode = UDINT#1 THEN
    IF pm[AxisNum].ma_asse_installato=INACTIVE_DEACTIVATABLE_AXIS THEN
      GetEncoderAbsolute:=TRUE;
    ELSE
      NumEncoder:=GetEncoderNumber(AxisNum);    
    END_IF;
    //MyPosAx:=Axes[AxisNum].PosAx;
    CASE NumEncoder OF
      UDINT#1: 
      GetEncoderAbsolute:= (MyPosAx^.Sensor[1]._Type = 2) OR
                              (MyPosAx^.Sensor[1]._Type = 1);
      UDINT#2: GetEncoderAbsolute:=(MyPosAx^.Sensor[2]._Type = 2) OR
                              (MyPosAx^.Sensor[2]._Type = 1);
    END_CASE;
  END_IF;
END_FUNCTION 

/***********************************************************
 * RETURNS THE ACTIVE ENCODER NUMBER FOR THE AXISNUM AXIS^ *
 ***********************************************************/

FUNCTION GetEncoderNumber:UDINT

  VAR_EXTERNAL
    Axes                            		:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    pm                                  : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters; 
  END_VAR
  
  VAR_INPUT
    AxisNum : INT;
  END_VAR
  VAR
    MyPosAx : REF_TO TO_PositioningAxis;
  END_VAR


  MyPosAx := AsPositioningAxisRef(Axes[AxisNum].PosAx);
  
  IF MyPosAx^.VirtualAxis.Mode = UDINT#1 AND pm[AxisNum].ma_asse_abilitato=TRUE THEN ;
    //MyPosAx:=Axes[AxisNum].PosAx;
    // trova il numero di encoder attivo
    IF MyPosAx^.StatusSensor[1].Control  AND MyPosAx^.Sensor[1].Existent THEN//Sensor[1].existent
      GetEncoderNumber:=UDINT#1;
    END_IF;
    //IF MyPosAx.SetConfigData.TypeOfAxis.NumberOfDataSets.numberOfDataSets>1 THEN
      IF MyPosAx^.StatusSensor[2].Control  AND MyPosAx^.Sensor[2].existent THEN
        GetEncoderNumber:=UDINT#2;
      END_IF;
   // END_IF;
    //IF MyPosAx.SetConfigData.TypeOfAxis.NumberOfDataSets.numberOfDataSets>2 THEN
      IF MyPosAx^.StatusSensor[3].Control  AND MyPosAx^.Sensor[3].existent THEN
        GetEncoderNumber:=UDINT#3;
      END_IF;
    //END_IF;
  END_IF;
END_FUNCTION 

FUNCTION TestVelRealZero:BOOL

  VAR_EXTERNAL  
		Axes                            		:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    pm                              		: ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    AxisInLimitationCouple		          :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    INACTIVE_DEACTIVATABLE_AXIS         :SINT;
  END_VAR
  
  VAR_INPUT
    NomeAsse            : DB_ANY;
    NumeroAsse          :INT;
    ErroreInseguimento  :BOOL;
  END_VAR

  VAR
    PosAxis             : REF_TO TO_PositioningAxis;
  END_VAR

  IF CheckIfTOactive(Axis:=NumeroAsse)=TRUE THEN
    PosAxis:=AsPositioningAxisRef(Axes[NumeroAsse].posax);
    IF  PosAxis^.VirtualAxis.Mode = UDINT#1 OR 
       pm[NumeroAsse].ma_asse_installato=INACTIVE_DEACTIVATABLE_AXIS THEN
       TestVelRealZero:=TRUE;   
    ELSE
      TestVelRealZero:=FALSE;   
      IF ErroreInseguimento=TRUE AND AxisInLimitationCouple[NumeroAsse]=FALSE THEN
        //Si verifica l^errore di inseguimento solo se l^asse non ha una limitazione di coppia
        //attiva
        IF (ABS(PosAxis^.actualvelocity)<=MAX((pm[NumeroAsse].ma_pos_win_vel*pm[NumeroAsse].max_ax_velo/100),0.01)) AND
           (ABS(PosAxis^.StatusPositioning.FollowingError)<=pm[NumeroAsse].ma_standstill_pos_tol) THEN
           TestVelRealZero:=TRUE;
        END_IF;
      ELSE
        IF ABS(PosAxis^.actualvelocity)<=MAX((pm[NumeroAsse].ma_pos_win_vel*pm[NumeroAsse].max_ax_velo/100),0.01) THEN
          TestVelRealZero:=TRUE;  
        END_IF;
      END_IF;  
    END_IF;
  ELSE
    TestVelRealZero:=TRUE;    
  END_IF;
END_FUNCTION  

FUNCTION reset_error:INT
  // azzera il buffer degli errori 
  // Pressione tasto RESET
  VAR_EXTERNAL
    i_sys_Error_ON                  :BOOL;
    i_sys_error_number	            :DINT;
    MessageStop                     :BOOL;
  END_VAR
  VAR
    iret  :INT;  
    dret  :DINT;
    i      :INT;
  END_VAR
  i_sys_Error_ON:=TRUE; 
	MessageStop:=FALSE;
  WHILE i_sys_error_number>0 DO
    pop_error();
  END_WHILE;         
  // Cancella gli errori dal task tecnologico
  //dret:=_resetTechnologicalErrors();
  // codice di allarme 
  reset_error:=0;
END_FUNCTION

/***************************************************************************************
 * FOR ONLY MACHINES THAT HAVE L1^AXIS PRESENT, IT EXITS WITH FALSE WHEN THE PARAMETER *
 *                                INPUT PARAMETER IS L1                                *
 ***************************************************************************************/

FUNCTION SiTestL1:BOOL

  VAR_EXTERNAL CONSTANT
    L1 :    INT;  //AL5
  END_VAR
  VAR_INPUT
    Asse  :INT;
  END_VAR
  SiTestL1:=FALSE;
  (*{
  #ifdef AL_BLM
  }*)
    IF Asse=L1 THEN
      SiTestL1:=TRUE;
    END_IF;
 (* {
    #endif    //Fine AL_BLM
  }*)
END_FUNCTION

(*{
#ifdef EFLEX_DH4010
}*)
	FUNCTION_BLOCK Modulo_0_360

    VAR_EXTERNAL  
      Axes                            		:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    END_VAR

		VAR_INPUT
			Asse				:INT;
		END_VAR

    VAR_OUTPUT 
      OutputState:INT; 
    END_VAR

		VAR
      PosAx : REF_TO TO_PositioningAxis;
      Homing:Homing; 
			QuotaIntera	:LREAL;
			Quota				:LREAL;
			dret				:DINT;
		END_VAR

    PosAx := AsPositioningAxisRef(Axes[Asse].PosAx);
		IF PosAx^.actualPosition >360 OR PosAx^.actualPosition<0 THEN
			//Legge la quota dell^asse direttamente dal sensore di posizione
			Quota:=PosAx^.StatusSensor[1].Position;
			QuotaIntera:=TO_DINT(Quota/360);
			Quota:=Quota-(QuotaIntera*360);
			IF Quota<0 THEN
				Quota:=360+Quota;
			END_IF;
			Homing(Asse:=Asse, Quota:=Quota, Modo:=EnumHomingMode#DIRECT_HOMING_ABSOLUTE);    
			(*WHILE NOT PosAx^.StatusWord.%X5 DO ;//%X5-asse homed
        //dret:=_waitTime(T#0ms)
			END_WHILE;*)
		END_IF;
	END_FUNCTION_BLOCK
(*{
#endif	//Fine EFLEX_DH4010
}*)

(*{
#ifdef SMART
}*)
	FUNCTION VerificaBasculanteDxSx:INT

  VAR_EXTERNAL
    cfg                             :_TYPE.Config_Struct;
    LatoBasculante            :INT;	//DX=0 SX=1
  END_VAR

		VerificaBasculanteDxSx:=-1;
		IF LatoBasculante=cfg.i_sys_LatoBasculante THEN
			VerificaBasculanteDxSx:=2;
		END_IF;
		IF LatoBasculante=1 AND cfg.i_sys_LatoBasculante=0 THEN
			VerificaBasculanteDxSx:=1;
		END_IF;
		IF LatoBasculante=0 AND cfg.i_sys_LatoBasculante=1 THEN
			VerificaBasculanteDxSx:=0;
		END_IF;
	END_FUNCTION
(*{
#endif	//Fine SMART
}*)

FUNCTION VAR_INC

  VAR_EXTERNAL
    i_sys_OPMODE                    : INT;	  // MODO OPERATIVO
                                              // 0 = JOG
                                              // 1 = REFERENCE
                                              // 2 = MDA
                                              // 3 = AUTO
    i_sys_varinc : INT;	//Variabile di incremento di passo in JOG
                                              // 0 = Non attivo INC
                                              // 1 = 1 um
                                              // 2 = 10 um
                                              // 3 = 100 um
                                              // 4 = 1000 um = 1 mm
                                              // 5 = 10 mm
    mcpo_1 : BOOL;
    mcpo_10  : BOOL;
    mcpo_100  : BOOL;
    mcpo_1000 : BOOL;
    mcpo_10000 : BOOL;
    mcpo_varinc : BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    JOGMODE                               :INT;
  END_VAR

  VAR_INPUT
    iVarValue:INT;
  END_VAR
  //Imposta la variazione incrementale di un passo di JOG
  //i_sys_VARINC viene impostata al Value incrementale selezionato
  //= 0 - nessuno
  //= 1 - 1 um
  //= 2 - 10 um
  //= 3 - 100 um
  //= 4 - 1 mm
  //= 5 - 10 mm
  //= 6 - TARGET
  // verifica di essere in stato JOG altrimenti resetta ed esce
  IF i_sys_OPMODE<>JOGMODE OR iVarValue<=0 THEN
    i_sys_varinc:=0;
    mcpo_10000:=FALSE;
    mcpo_1000:=FALSE;
    mcpo_100:=FALSE;
    mcpo_10:=FALSE;
    mcpo_1:=FALSE;
    mcpo_varinc:=FALSE;
  ELSE
    i_sys_varinc:=iVarValue;
    CASE iVarValue OF
      1:  mcpo_10000:=FALSE;
          mcpo_1000:=FALSE;
          mcpo_100:=FALSE;
          mcpo_10:=FALSE;
          mcpo_1:=TRUE;
          mcpo_varinc:=FALSE;

      2:  mcpo_10000:=FALSE;
          mcpo_1000:=FALSE;
          mcpo_100:=FALSE;
          mcpo_10:=TRUE;
          mcpo_1:=FALSE;
          mcpo_varinc:=FALSE;

      3:  mcpo_10000:=FALSE;
          mcpo_1000:=FALSE;
          mcpo_100:=TRUE;
          mcpo_10:=FALSE;
          mcpo_1:=FALSE;
          mcpo_varinc:=FALSE;

      4:  mcpo_10000:=FALSE;
          mcpo_1000:=TRUE;
          mcpo_100:=FALSE;
          mcpo_10:=FALSE;
          mcpo_1:=FALSE;
          mcpo_varinc:=FALSE;

      5:  mcpo_10000:=TRUE;
          mcpo_1000:=FALSE;
          mcpo_100:=FALSE;
          mcpo_10:=FALSE;
          mcpo_1:=FALSE;
          mcpo_varinc:=FALSE;
                    
      6:  mcpo_10000:=FALSE;
          mcpo_1000:=FALSE;
          mcpo_100:=FALSE;
          mcpo_10:=FALSE;
          mcpo_1:=FALSE;
          mcpo_varinc:=TRUE;

    END_CASE;    
  END_IF;     
END_FUNCTION

/****************************************************************************************
 * INCREMENTA IL CONTATORE DEL NUMERO DI ATTIVAZIONI DELL^EVENTO PASSATO COME PARAMETRO *
 ****************************************************************************************/

FUNCTION ConteggiaEvento
  VAR_EXTERNAL
    i_sys_buttonEventCounter    :DWORD;	//da bit 00 a 03 contatore attivazione Start ciclo 
                                    //da bit 04 a 07 contatore attivazione Stop ciclo
                                    //da bit 08 a 11 contatore attivazione Reset
                                    //da bit 12 a 15 contatore attivazione Pulsante +
                                    //da bit 16 a 19 contatore attivazione Pulsante -
  END_VAR
	VAR_INPUT
		Event    :USINT;
	END_VAR
	VAR
		Bit				:ARRAY[0..3] OF BOOL;
		Valore    :BYTE;
		i					:USINT;
	END_VAR
	//Legge l^attuale valore del contatore dell^evento indicato

	FOR i:=Event TO Event+USINT#3 DO
		Bit[i-Event]:=getBit(i_sys_buttonEventCounter, i);
	END_FOR;
  Valore.%X0:=Bit[0];
  Valore.%X1:=Bit[1];
  Valore.%X2:=Bit[2];
  Valore.%X3:=Bit[3];
  Valore:= TO_BYTE(TO_SINT(Valore)+SINT#1);
	//Incrementa di 1 il valore letto
  //Valore:=Bit[0]+2*(Bit[1])+4*Bit[2]+8*Bit[3]+1;
	//Riconverte in bit il nuovo valore
	FOR i:=0 TO USINT#3 DO
			Bit[i]:=getBit(Valore, i);
	END_FOR;
	//Memorizza il nuovo valore nella corretta posizione
	FOR i:=0 TO USINT#3 DO
		//Impostazione del nuovo valore incrementato
		i_sys_buttonEventCounter:=setBit(i_sys_buttonEventCounter, Event+i, Bit[i]); 
	END_FOR;
END_FUNCTION

/***********************************************************************
 * TRUE IF THE^INPUT ENABLING SLAVE OPERATION OF THE MACHINE IS ACTIVE *
 ***********************************************************************/

FUNCTION GetCmdCentraleCoupler:BOOL

  VAR_EXTERNAL
    i_sys_MachineMaster         :BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    INP_LOAD_COMMAND_CENTRAL_DPCOUPLER     :INT;
  END_VAR
  GetCmdCentraleCoupler:=ReadDigitalInput(NumInput:=INP_LOAD_COMMAND_CENTRAL_DPCOUPLER,Verifica:=FALSE)=TRUE AND i_sys_MachineMaster=FALSE;
END_FUNCTION

FUNCTION ComandoNastroTrasportatore

  VAR_EXTERNAL CONSTANT
    OUT_NAST_TRANSPORTER                            :INT;
    OUT_NAST_TRANSPORTER_DPCOUPLER                  :INT;
  END_VAR

	VAR_INPUT
		Stato	:BOOL;
	END_VAR
	WriteDigitalOutput(NumOutput:=OUT_NAST_TRANSPORTER, Value:=Stato);
	WriteDigitalOutput(NumOutput:=OUT_NAST_TRANSPORTER_DPCOUPLER, Value:=Stato);
END_FUNCTION

/************************************************************************************************
 * INDICATES ON WHICH ROW OF THE STATUS TABLE THE STOP POINT PASSED AS A PARAMETER IS^ MANAGED, *
 *                  I.E. ON THE ROW PRECEDING WHERE IT IS ACTUALLY PROGRAMMED                   *
 ************************************************************************************************/

FUNCTION RigaStopPoint:INT

  VAR_EXTERNAL
    Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
  END_VAR

  VAR_EXTERNAL CONSTANT
    MY_BIG_VALUE_INT                    :INT;
    COL_STOP_POINT                      :INT;
    COL_NUM_SEQ                         :INT;
  END_VAR

  VAR_INPUT
		Programma		:INT;
		PuntoArresto:INT;
		Sequenza		:DINT:=-DINT#1;
		Tabella			:SINT:=-SINT#1;
		Riga				:INT:=0;
  END_VAR
  
  VAR 
		i						:INT;
		NumeroStati	:INT;
  END_VAR;

	RigaStopPoint:=-MY_BIG_VALUE_INT;
	//Ricerca della riga della tabella stati dove e^ presente lo stop point, che indica
	//il termine della ricerca attiva del terminale del tubo
	(*{
	#ifdef DH4010    
	}*)
		NumeroStati:=programmi[Programma].NumStati[Tabella]-1;
	(*{
	#else	//Fine DH4010
	}
		NumeroStati:=Programmi[Programma].NumStati-1;
	{
	#endif	//Fine NO DH4010
	}*)
	FOR i:=Riga TO NumeroStati DO
		IF ReadTabStati(Riga:=i, Colonna:=COL_STOP_POINT, Programma:=Programma, Tabella:=Tabella)=PuntoArresto THEN
			IF Sequenza<0 THEN
				//Nessuna richiesta di cercare in una sequenza specifica, quindi si esce
				//dalla ricerca non appena si trova lo stop point
				RigaStopPoint:=MAX(i-1, 0);
				EXIT;
			ELSE  
				//La ricerca deve essere circoscritta ad una determinata sequenza
				//si verifica che la riga trovata con lo stop point faccia parte della
				//sequenza indicata
				IF ReadTabStati(Riga:=i, Colonna:=COL_NUM_SEQ, Programma:=Programma, Tabella:=Tabella)=TO_INT(Sequenza) THEN
					RigaStopPoint:=MAX(i-1, 0);
					EXIT;
				END_IF;  
			END_IF;  
		END_IF;
	END_FOR;
END_FUNCTION

FUNCTION Read_OpMode:INT

/********************************************************************************************
 * READS FROM MACHINE BUTTON PANEL IF SOMEONE HAS PRESSED A BUTTON TO CHANGE OPERATING MODE *
 ********************************************************************************************/

 VAR_EXTERNAL
  R_AUTO                                  : R_TRIG; //fronte salita 
  R_MDA			                              :R_TRIG; //fronte salita
  R_JOG			                              :R_TRIG; //fronte salita 
  R_REF			                              :R_TRIG; //fronte salita 
END_VAR
  VAR_EXTERNAL CONSTANT
    JOGMODE                               :INT;
    REFMODE                               :INT;
    MDAMODE                               :INT;
    AUTOMODE                              :INT;
  END_VAR

  VAR 
    iret  :INT;
  END_VAR
  iret:=-1;
  IF R_AUTO.Q THEN
    iret:=AUTOMODE;    
  ELSE
    IF R_MDA.Q THEN
      iret:= MDAMODE;
    ELSE
      IF R_JOG.Q THEN
        iret:=JOGMODE;
      ELSE
        IF R_REF.Q THEN
          iret:=REFMODE;
        END_IF;
      END_IF;
    END_IF;
  END_IF;
  read_opmode:=iret;
END_FUNCTION

/******************************************************************************
 * TURNS ON THE HAND CONTROL LEDS ACCORDING TO THE DEVICE PASSED AS PARAMETER *
 *                SETS THE ACTIVE DEVICE FOR MOVEMENTS IN JOG                 *
 *                                0 TO 31 AXES                                *
 *                             32 TO 63 IO DEVICE                             *
 *                                 -1 = NONE                                  *
 ******************************************************************************/

FUNCTION set_ActiveDevice

  VAR_EXTERNAL
    i_sys_OPMODE                    : INT;	// MODO OPERATIVO
                                            // 0 = JOG
                                            // 1 = REFERENCE
                                            // 2 = MDA
                                            // 3 = AUTO
    i_sys_activedevice              : INT;  //Dispositivo selezionato per movimenti in JOG
                                            // da 0..31 asse
                                            // da 32 a 64 dispositivo IO
                                            // -1 = INDEFINITO
                                            mcpo_ax0 : BOOL;
                                            mcpo_ax1 : BOOL;
                                            mcpo_ax2 : BOOL;
                                            mcpo_ax3 : BOOL;
                                            mcpo_ax4 : BOOL;
                                            mcpo_ax5 : BOOL;
                                            mcpo_ax6 : BOOL;
                                            mcpo_ax7 : BOOL;
                                            mcpo_ax8 : BOOL;
                                            mcpo_ax9 : BOOL;
                                            mcpo_ax10 : BOOL;
                                            mcpo_ax11 : BOOL;

                                            mcpo3 : BYTE;
                                            mcpo4  : BYTE;
                                            mcpo5 : BYTE;

                                            mcpo_plus : BOOL;
                                            mcpo_minus : BOOL;
   Key_Disp                  :ARRAY[0.._MAX_KEY_DISP#MAX_KEY_DISP-1] OF INT;
   KEY_ASSI                  : ARRAY[0..MAX_TASTI_ASSI-1] OF INT;
  END_VAR

  VAR_EXTERNAL CONSTANT
    HEAD_1               :INT;  //32
    HEAD_2               :INT;  //33
    HEAD_1_SX            :INT;  //32
    HEAD_2_SX            :INT;  //33
    HEAD_1_DX            :INT;  //34
    HEAD_2_DX            :INT;  //35
    DISAPPEARANCE_SHOULDER_LEFT:INT;  //36
    DISAPPEARANCE_SHOULDER_RIGHT:INT;  //37
    FIRST_COMPLEX_OPERATION           :INT;
    AUTOMODE                          :INT;
    UNDEF_DEVICE                      :INT;
    MAX_KEYS_AXIS                       :USINT;
    MORSA_FOUR_RLFORM       :INT ;	//47
    MAX_TASTI_ASSI          :USINT;
    
  END_VAR

  VAR_INPUT
    iDeviceNumber			:INT;
    AncheInAuto				:BOOL:=FALSE;
  END_VAR
  VAR
    i									:INT;  
    iCurrentKey				:INT;
		AppoiDeviceNumber	:INT;
    (*{
    #ifdef RUNNER
    }*)
			MascheraBit			:BYTE:=BYTE#00000010;
    (*{
    #else	//Fine RUNNER
    }
			{
			#ifdef EFLEX
			}
				MascheraBit		:BYTE:=00000010;
			{
			#else	//Fine EFLEX
			}
				{
				#ifdef DH4010
				}
					MascheraBit	:BYTE:=00000100;
				{
				#else	//Fine DH4010
				}
					MascheraBit	:BYTE:=00000000;
				{
				#endif	//Fine NO DH4010
				}*)
			(*{
			#endif	//Fine NO EFLEX
			}
    {
    #endif	//Fine NO RUNNER
    }*)
  END_VAR 

	AppoiDeviceNumber:=iDeviceNumber;
	//La movimentazione manuale di questi dispositivi comporta dei cicli complessi in quanto coinvolgono altri assi e quindi non
	//possono essere eseguiti nel background ma nel motiontask CICLI. Quindi occorre modificare in operazione complessa il
	//movimento dei successivi dispositivi
	// {
	// #ifdef EFLEX
	// }
	// 	CASE AppoiDeviceNumber OF %%%%
	// 		HEAD_1, HEAD_2:
	// 			AppoiDeviceNumber:=iDeviceNumber+FIRST_COMPLEX_OPERATION;
	// 	END_CASE;
	// // {
	// #endif    //Fine EFLEX
	// }
	// {
	// // #ifdef DH4010 %%%%
	// // }
	// 	CASE AppoiDeviceNumber OF
	// 		HEAD_1_DX, HEAD_1_SX, HEAD_2_DX, HEAD_2_SX, DISAPPEARANCE_SHOULDER_RIGHT, DISAPPEARANCE_SHOULDER_LEFT:
	// 			AppoiDeviceNumber:=iDeviceNumber+FIRST_COMPLEX_OPERATION;
	// 	END_CASE;
	// // {
	// // #endif	//Fine DH4010
	// // }
  IF (i_sys_OPMODE=AUTOMODE OR AppoiDeviceNumber=UNDEF_DEVICE) AND AncheInAuto=FALSE THEN
    //Reset   
    i_sys_activedevice:=UNDEF_DEVICE;
    mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
		mcpo3:=BYTE#0; mcpo5:=BYTE#0;
		//Serve per mantenere lo stato del led del pulsante del ciclo continuo, se presente nella macchina 
		mcpo4:=mcpo4 AND MascheraBit; 
    mcpo_plus:=FALSE;
    mcpo_minus:=FALSE;
  ELSE
    i_sys_activedevice:=AppoiDeviceNumber;
    IF AppoiDeviceNumber<_MAXIS#MAXAXIS OR TestMorsaFourRlform(Dispositivo:=AppoiDeviceNumber)=TRUE THEN
      //E^ un asse
      iCurrentKey:=-1;
      FOR i:=0 TO MAX_KEYS_AXIS-1 DO
        IF KEY_ASSI[i]=AppoideviceNumber THEN
          // verifica se deve accendere un led sulla pulsantiera
          iCurrentKey:=i;
          EXIT;
        END_IF;
      END_FOR;
      // // {
      // // #ifdef PLAUNO_LIGHT
      // // }
      //   IF AppoiDeviceNumber=MORSA_FOUR_RLFORM THEN
      //     iCurrentKey:=MORSA_FOUR_RLFORM;                    
      //   END_IF;
      // // {
      // // #endif  //Fine PLAUNO_LIGHT
      // // }
      IF iCurrentKey>=0 THEN
        //Accensione LED sulla pulsantiera per gestione assi
        CASE iCurrentKey OF
					 {::ifndef MCP_1900}
						0:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=TRUE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						1:  mcpo3:=BYTE#0; mcpo4:=BYTE#1; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						2:  mcpo3:=BYTE#128; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						3:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=TRUE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						4:  mcpo3:=BYTE#64; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						5:  mcpo3:=BYTE#2; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						6:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#0;  
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=TRUE; mcpo_ax9:=FALSE;
						7:  mcpo3:=BYTE#4; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						8:  mcpo3:=BYTE#8; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						9:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=TRUE;
						10: mcpo3:=BYTE#16; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						11: mcpo3:=BYTE#32; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						12: mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#16;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						13: mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#8;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						14: mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#4;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;            
						15: mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#2;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;  
						16: mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#1;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;    
						17: mcpo3:=BYTE#0; mcpo4:=BYTE#128; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						18: //Basculante elettrico
								mcpo3:=BYTE#0; mcpo4:=BYTE#4; mcpo5:=BYTE#0;
								mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;;
						// // {#ifdef PLAUNO_LIGHT}  %%%%
						// 	MORSA_FOUR_RLFORM:    
						// 		mcpo3:=BYTE#4; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
						// 		mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
						// // {#endif}  //Fine PLAUNO_LIGHT
            
            //MAX_KEYS_AXIS:
							//Spegne i LED perchè l^asse non ha un pulsante su pulsantiera macchina
							mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
							mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;   
              {::else} //{::endif} 
						0:
							mcpo_ax0:=TRUE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;
						1:
							mcpo_ax0:=FALSE; mcpo_ax1:=TRUE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;	
						2:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=TRUE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;								
						3:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=TRUE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;		
						4:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=TRUE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;
						5:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=TRUE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;
						6:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=TRUE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;	
						7:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=TRUE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;								
						8:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=TRUE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;	
						9:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=TRUE; mcpo_ax10:=FALSE; mcpo_ax11:=FALSE;
						10:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=TRUE; mcpo_ax11:=FALSE;	
						11:
							mcpo_ax0:=FALSE; mcpo_ax1:=FALSE; mcpo_ax2:=FALSE;
							mcpo_ax3:=FALSE; mcpo_ax4:=FALSE; mcpo_ax5:=FALSE;
							mcpo_ax6:=FALSE; mcpo_ax7:=FALSE; mcpo_ax8:=FALSE;
							mcpo_ax9:=FALSE; mcpo_ax10:=FALSE; mcpo_ax11:=TRUE;
		
					{::endif};
        END_CASE;
      ELSE
        //Spegne i LED perchè l^asse non ha un pulsante su pulsantiera macchina
        mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
        mcpo_ax0:=FALSE; mcpo_ax3:=FALSE;  mcpo_ax6:=FALSE;  mcpo_ax9:=FALSE;
      END_IF;
    ELSE
      iCurrentKey:=-1;
      FOR i:=0 TO _MAX_KEY_DISP#MAX_KEY_DISP-1 DO
        IF key_Disp[i]=AppoideviceNumber THEN
          // verifica se deve accendere un led sulla pulsantiera
          iCurrentKey:=i;
        END_IF;
      END_FOR;  
      IF iCurrentKey>-1 THEN
        CASE iCurrentKey OF
          //In base al tasto premuto (iCurrentKey) imposta i led della tastiera
          0:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#128;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          1:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#64;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          2:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#32;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          3:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#16;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          4:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#8;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          5:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#4;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          6:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#2;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          7:  mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#1;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          8:  mcpo3:=BYTE#0; mcpo4:=BYTE#128; mcpo5:=BYTE#0;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          9:  mcpo3:=BYTE#0; mcpo4:=BYTE#64; mcpo5:=BYTE#0;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          10: mcpo3:=BYTE#0; mcpo4:=BYTE#32; mcpo5:=BYTE#0;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          11: mcpo3:=BYTE#0; mcpo4:=BYTE#16; mcpo5:=BYTE#0;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          12: mcpo3:=BYTE#0; mcpo4:=BYTE#8; mcpo5:=BYTE#0;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          13: mcpo3:=BYTE#0; mcpo4:=BYTE#4; mcpo5:=BYTE#0;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          14: mcpo3:=BYTE#0; mcpo4:=BYTE#2; mcpo5:=BYTE#0;
              mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          // // {
          // // #ifdef DH4010 %%%%
          // // }
          //   15: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00000100; mcpo5:=BYTE#2#00010000;///Check with originif it is writen right BYTE)
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   16: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00000100; mcpo5:=BYTE#2#00001000;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   17: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00000100; mcpo5:=BYTE#2#00000010;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   18: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#01000100; mcpo5:=BYTE#2#00000000;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   19: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00000100; mcpo5:=BYTE#2#00000001;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   20: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00100100; mcpo5:=BYTE#2#00000000;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   21: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00000100; mcpo5:=BYTE#2#00000100;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   22: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00000100; mcpo5:=BYTE#2#01000000;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   23: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#10000100; mcpo5:=BYTE#2#00000000;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   24: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00010100; mcpo5:=BYTE#2#00000000;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   25: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00001100; mcpo5:=BYTE#2#00000000;
					// 			mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
					// 	26: mcpo3:=BYTE#2#00000000; mcpo4:=BYTE#2#00000110; mcpo5:=BYTE#2#00000000;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
					// // {
          // // #else //Fine DH4010
          // // }
          //   15: mcpo3:=BYTE#0; mcpo4:=BYTE#68; mcpo5:=BYTE#0; %%%%
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   16: mcpo3:=BYTE#0; mcpo4:=BYTE#36; mcpo5:=BYTE#0;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   17: mcpo3:=BYTE#0; mcpo4:=BYTE#20; mcpo5:=BYTE#0;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          //   18: mcpo3:=BYTE#0; mcpo4:=BYTE#6; mcpo5:=BYTE#0;
          //       mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
          // // {
          // // #endif //Fine NO DH4010
          // // }
        END_CASE;    
      ELSE
        //Spegne i LED perchè il dispositivo non ha un pulsante su pulsantiera macchina
        mcpo3:=BYTE#0; mcpo4:=BYTE#0; mcpo5:=BYTE#0;
        mcpo_ax0:=FALSE; mcpo_ax3:=FALSE; mcpo_ax6:=FALSE; mcpo_ax9:=FALSE;
      END_IF;     
    END_IF;
  END_IF;
END_FUNCTION

FUNCTION SetLedCicloContinuo

  VAR_EXTERNAL
    mcpo_teach_in  : BOOL;
    mcpo_t14 : BOOL;
  END_VAR

  VAR_INPUT
    Stato    :BOOL;
  END_VAR
  {::ifdef DH4010}
    mcpo_teach_in:=Stato;
  {::else}    //Fine DH4010
    mcpo_t14:=Stato;
  {::endif}    //Fine NO DH4010
  
END_FUNCTION

/*****************************************************************************
 * RESULTS TRUE IF THE INDICATED DEVICE IS THE PNEUMATIC CLAMP OF THE RLFORM *
 *****************************************************************************/

FUNCTION TestMorsaFourRlform:BOOL 

  VAR_EXTERNAL CONSTANT
    MORSA_FOUR_RLFORM       :INT;	//47
  END_VAR

  VAR_INPUT
    Dispositivo  :INT;
  END_VAR
  // {
  // #ifdef PLAUNO_LIGHT
  // } 
    TestMorsaFourRlform:=Dispositivo=MORSA_FOUR_RLFORM;
  // {
  // #else  //Fine PLAUNO_LIGHT
  // } 
    TestMorsaFourRlform:=FALSE;
  // {
  // #endif  //Fine NO PLAUNO_LIGHT
  // } 
END_FUNCTION

/*******************************************************************
 * CONVERTS THE SIMOTION DEVICE/AXIS NUMBER INTO THE VGP NUMBERING *
 *******************************************************************/

FUNCTION Get_Colonna_Device_Stati:INT

  VAR_EXTERNAL
    COL_DEVICE_STATI : ARRAY[0.._MAXCOL_DEVICE#MAXCOL_DEVICE-1] OF INT;
  END_VAR

  VAR_EXTERNAL CONSTANT
    UNDEF_DEVICE                       	:INT;
  END_VAR
  VAR_INPUT
    device_simotion:INT;
  END_VAR
  VAR 
    i    :INT;
  END_VAR;
  Get_Colonna_device_stati:=UNDEF_DEVICE;
  FOR i:=0 TO _MAXCOL_DEVICE#MAXCOL_DEVICE-1 DO
    IF COL_DEVICE_STATI[i]=device_simotion THEN
      Get_Colonna_device_stati:=i;
      EXIT;
    END_IF;    
  END_FOR;
END_FUNCTION  

/*******************************************************************
 * CHECK WHETHER A MANUAL LOAD IS FORESEEN IN THE ACTIVE PROGRAMME *
 *******************************************************************/

FUNCTION TestCaricoAspo:BOOL

  VAR_EXTERNAL
    Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
  END_VAR

  VAR_EXTERNAL CONSTANT
    CARICO_ASPO                        	:SINT;  //Carico della DH4010 con aspo
  END_VAR

  VAR_INPUT
    Programma  :DINT;
  END_VAR
  TestCaricoAspo:=programmi[Programma].TipoCarico=CARICO_ASPO;
END_FUNCTION

FUNCTION Read_NCStop:BOOL

  VAR_EXTERNAL
    i_sys_DpCoupler                 :BOOL;
    i_sys_PnCoupler                 :BOOL;
    MessageStop                     :BOOL;
    kvc_stop_cycle			            :BOOL;
    mcpi_ncstop                     :BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
  INP_EMERGENCY                                   :INT;
  INP_STOP_CYCLE                                  :INT;
  INP_STOP_CYCLE_DPCOUPLER                        :INT;
  ERR_STOP_CYCLE_BUTTONS                          :DINT;
  END_VAR

  VAR
    dret  :DINT;
  END_VAR

  IF (i_sys_DpCoupler=TRUE OR i_sys_PnCoupler=TRUE) AND GetCmdCentraleCoupler()=TRUE THEN
    //Stop da remoto
    IF readdigitalinput(NumInput:=INP_STOP_CYCLE_DPCOUPLER)=FALSE OR readdigitalinput(NumInput:=INP_EMERGENCY)=FALSE THEN 
      Read_NCStop:=TRUE;
      IF readdigitalinput(NumInput:=INP_STOP_CYCLE_DPCOUPLER)=FALSE AND MessageStop=FALSE THEN
        MessageStop:=TRUE;
        dret:=Push_error(code:=ERR_STOP_CYCLE_BUTTONS, Attesa:=FALSE, Arresto:=FALSE, SoloMessaggio:=TRUE);
      END_IF;
    ELSE
      IF MessageStop=TRUE THEN
        MessageStop:=FALSE;
        pop_error();
      END_IF;
    END_IF;
  ELSE
    Read_NCStop:=NOT(mcpi_NCStop) OR kvc_stop_cycle;
    //Stop Da Pulsantiera Esterna o a seguito Emergenza
    IF readdigitalinput(NumInput:=INP_STOP_CYCLE)=FALSE OR readdigitalinput(NumInput:=INP_EMERGENCY)=FALSE THEN 
      Read_NCStop:=TRUE;
      IF readdigitalinput(NumInput:=INP_STOP_CYCLE)=FALSE AND MessageStop=FALSE THEN 
        MessageStop:=TRUE;
        dret:=Push_error(code:=ERR_STOP_CYCLE_BUTTONS, Attesa:=FALSE, Arresto:=FALSE, SoloMessaggio:=TRUE);
      END_IF;
    ELSE
      IF MessageStop=TRUE THEN
        MessageStop:=FALSE;
        pop_error();
      END_IF;
    END_IF;
  END_IF;
END_FUNCTION

FUNCTION Read_NCStart: BOOL

  VAR_EXTERNAL
    R_Start                                 :R_TRIG; //fronte salita START
    R_StartPuls     :R_TRIG;  //Fronte salita START Pulsantiera
    SystemStarted                   :BOOL;
    access                            :Access_Level;
    i_sys_ACCESSLEVEL              : INT ;	// LIVELLO DI ACCESSO 
                                                // è il minore fra sw_ACCESSLEVEL e hw_ACCESSLEVEL
                                                // 0 = PROGRAMMATORE COSTRUTTORE
                                                // 1 = COSTRUTTORE
                                                // 2 = MIS COSTRUTTORE
                                                // 3 = SAT COSTRUTTORE
                                                // 4 = MANUTENTORE CLIENTE
                                                // 5 = OPERATORE QUALIFICATO
                                                // 6 = OPERATORE CLIENTE
                                                // 7 = UTENTE QUALSIASI
  END_VAR

  VAR_EXTERNAL CONSTANT
    INP_PHOTOCELLS_NOT_VIOLATED                     :INT;
    TRC_VIOLATION_BARRIER                           :INT;  //Attenzione help trace in comune con DH
  END_VAR
  //Legge lo start da pulpito o da pulsantiera esterna 
	IF ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED, Verifica:=TRUE)=FALSE AND SystemStarted=TRUE THEN
		set_trace(Code:=TRC_VIOLATION_BARRIER );
	END_IF;
  IF ((R_Start.Q=TRUE) OR (R_StartPuls.Q=TRUE)) AND (i_sys_ACCESSLEVEL<=access.start_programma) THEN
    Read_NCStart:=TRUE;
		IF ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED, Verifica:=TRUE)=TRUE THEN
			reset_trace(Code:=TRC_VIOLATION_BARRIER );
		END_IF;
  ELSE
    Read_NCStart:=FALSE;
  END_IF;
END_FUNCTION  

FUNCTION TestVelHoldToRun:BOOL

  VAR_EXTERNAL CONSTANT
    INP_HOLD_TO_RUN                                 :INT;
    INP_ENABLE_HOLD_TO_RUN                          :INT;
  END_VAR
  IF readdigitalinput(NumInput:=INP_ENABLE_HOLD_TO_RUN , Verifica:=FALSE)=TRUE OR
     readdigitalinput(NumInput:=INP_HOLD_TO_RUN, Verifica:=FALSE)=TRUE THEN
    TestVelHoldToRun:=TRUE;
  ELSE
    TestVelHoldToRun:=FALSE;
  END_IF;
END_FUNCTION

FUNCTION Init_LOG_Assi

  VAR_EXTERNAL
    Log_assi								:Trace_Asse;
  END_VAR
  
  VAR
    Asse,
        Riga:    INT;
  END_VAR

  FOR Riga:=0 TO _MAX_LOG#MAX_LOG_SIZE-1 DO
    FOR Asse:=_FIRST_AXIS#FIRST_AXIS TO _MAXIS#MAXAXIS+_FIRST_AXIS#FIRST_AXIS-1 DO
      log_assi.DatiAssi[Asse].DatiAsse[Riga].FollError:=0;
      log_assi.DatiAssi[Asse].DatiAsse[Riga].Position:=0;
      log_assi.DatiAssi[Asse].DatiAsse[Riga].Velocity:=0;
      log_assi.DatiAssi[Asse].DatiAsse[Riga].Torque:=0;
      log_assi.DatiAssi[Asse].DatiAsse[Riga].LimitTorque:=USINT#0;
      log_assi.DatiAssi[Asse].DatiAsse[Riga].DistanceToGo:=0;
      log_assi.DatiAssi[Asse].DatiAsse[Riga].Temperature:=USINT#0;
    END_FOR;
        log_assi.DatiGenerali_1[Riga].PosMaster:=0;
        log_assi.DatiGenerali_1[Riga].VelMaster:=0;
        log_assi.DatiGenerali_1[Riga].Riga:=0;
        log_assi.DatiGenerali_1[Riga].Error:=0;
        // {
        // #ifdef DH4010 %%%%
        // }
        //     log_assi.DatiGenerali_2[Riga].PosMaster:=0;
        //     log_assi.DatiGenerali_2[Riga].VelMaster:=0;
        //     log_assi.DatiGenerali_2[Riga].Riga:=0;
        //     log_assi.DatiGenerali_2[Riga].Error:=0;
        // // {
        // #endif    //Fine NO DH4010
        // }
  END_FOR;
  log_assi.pointer_log:=0;
END_FUNCTION

FUNCTION Read_Ovr:INT

  VAR_EXTERNAL 
        // GRAY TO DEC CONVERTER      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25, 26,27, 28, 29, 30, 31
        GRAY_OVR:ARRAY[0..31] OF INT;
    
  END_VAR

  VAR_INPUT
    a,b,c,d,e  :BOOL;
  END_VAR
  VAR 
    i          :INT;
  END_VAR;  
  // Restituisce il valore numerico del FEED 
  // partendo da 5 bit in codifica GRAY (OVERRIDE)
  i:=0;
  IF a THEN i:=i+1;  END_IF; 
  IF b THEN i:=i+2;  END_IF; 
  IF c THEN i:=i+4;  END_IF; 
  IF d THEN i:=i+8;  END_IF; 
  IF e THEN i:=i+16; END_IF; 
  Read_Ovr:=GRAY_OVR[i];
END_FUNCTION

FUNCTION Read_Ovr2:INT

  VAR_EXTERNAL
            // GRAY TO DEC CONVERTER2     0, 1, 2, 3, 4, 5, 6, 7,  8,  9, 10, 11,12,13, 14,15
            GRAY_OVR2:ARRAY[0..15] OF INT;
  END_VAR

  VAR_INPUT
    a,b,c,d  :BOOL;
  END_VAR
  VAR 
    i        :INT;
  END_VAR;  
  // Restituisce il valore numerico del FEED del mandrino 
  // partendo da 4 bit in codifica GRAY (OVERRIDE)
  i:=0;
  IF a THEN i:=i+1; END_IF; 
  IF b THEN i:=i+2; END_IF; 
  IF c THEN i:=i+4; END_IF; 
  IF d THEN i:=i+8; END_IF;
  Read_OVR2:=GRAY_OVR2[i]; 
END_FUNCTION

/**************************************************
 * LEGGE DA PULSANTIERA MACCHINA IL DEVICE ATTIVO *
 **************************************************/

FUNCTION_BLOCK FB_Read_ActiveDevice

  VAR_EXTERNAL
    i_sys_OPMODE                    : INT;
    mcpi_ax0  : BOOL;
    mcpi_ax1  : BOOL;
    mcpi_ax2  : BOOL;
    mcpi_ax3  : BOOL;
    mcpi_ax4  : BOOL;
    mcpi_ax5  : BOOL;
    mcpi_ax6  : BOOL;
    mcpi_ax7  : BOOL;
    mcpi_ax8  : BOOL;
    mcpi_ax9  : BOOL;
    mcpi_ax10  : BOOL;
    mcpi_ax11  : BOOL;

    mcpi_t0  : BOOL;
    mcpi_t1  : BOOL;
    mcpi_t2  : BOOL;
    mcpi_t3  : BOOL;
    mcpi_t4  : BOOL;
    mcpi_t5  : BOOL;
    mcpi_t6  : BOOL;
    mcpi_t7  : BOOL;
    mcpi_t8  : BOOL;
    mcpi_t9  : BOOL;
    mcpi_t10  : BOOL;
    mcpi_t11  : BOOL;
    mcpi_t12  : BOOL;
    mcpi_t13  : BOOL;
    mcpi_t14  : BOOL;

    Key_Disp                                :ARRAY[0.._MAX_KEY_DISP#MAX_KEY_DISP-1] OF INT;
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    i_sys_activedevice              : INT;
    i_sys_swOVR                     : LREAL;
    i_sys_stopcond                  : INT;
    old_i_sys_activedevice          :INT; 
  END_VAR

  VAR_EXTERNAL CONSTANT
    JOGMODE                               :INT;
    REFMODE                               :INT;
    MDAMODE                               :INT;
    MAX_KEYS_AXIS                         :USINT;
    FIRST_COMPLEX_OPERATION               :INT;
    MANUAL_BENDING                        :INT;   //100
    OVERRIDE_STANDARD                     :INT;

  END_VAR

  VAR
    PosAx : REF_TO TO_SynchronousAxis;
    iret  :INT;
    dwret :DWORD;
    dret  :DINT;
    i     :USINT;
    Tasto :INT;
    Asse  :INT;
		
		CURRENT_STATE				:INT:=GENERAL_STATE#STATE_INIT;
		CURRENT_AXIS				:INT;
		ResetMotionResetAxis:FB_ResetMotionResetAxis;
		EliminaCodaComandi	:FB_EliminaCodaComandi;

  END_VAR  
	VAR_OUTPUT
		OutputState	:INT;
		ReturnValue	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			iret:=-1;
			CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
			IF i_sys_OPMODE=JOGMODE OR i_sys_OPMODE=REFMODE OR i_sys_OPMODE=MDAMODE THEN
				IF mcpi_ax0=TRUE THEN 
					Tasto:=0;
				ELSE
					IF mcpi_ax1=TRUE THEN 
						Tasto:=1;
					ELSE
						IF mcpi_ax2=TRUE THEN 
							Tasto:=2;
						ELSE
							IF mcpi_ax3=TRUE THEN 
								Tasto:=3;
							ELSE
								IF mcpi_ax4=TRUE THEN 
									Tasto:=4;
								ELSE
									IF mcpi_ax5=TRUE THEN 
										Tasto:=5;
									ELSE
										IF mcpi_ax6=TRUE THEN 
											Tasto:=6;
										ELSE
											IF mcpi_ax7=TRUE THEN 
												Tasto:=7;
											ELSE
												IF mcpi_ax8=TRUE THEN 
													Tasto:=8;
												ELSE
													IF mcpi_ax9=TRUE THEN 
														Tasto:=9;
													ELSE
														IF mcpi_ax10=TRUE THEN 
															Tasto:=10;
														ELSE
															IF mcpi_ax11=TRUE THEN
																Tasto:=11;
															ELSE
																Tasto:=-1;
															END_IF;
														END_IF;
													END_IF;
												END_IF;
											END_IF;
										END_IF;
									END_IF;
								END_IF;
							END_IF;
						END_IF;
					END_IF;
				END_IF;
				IF Tasto>-1 AND Tasto<MAX_KEYS_AXIS THEN ;
					// {#ifdef PLAUNO_LIGHT} %%%%
					// 	IF (Tasto=7) THEN
					// 		iret:=MORSA_FOUR_RLFORM;
					// 	ELSE 
					// 		iret:=KEY_ASSI[Tasto];
					// 	END_IF;
					// {#else}  //Fine PLAUNO_LIGHT
					// 	iret:=KEY_ASSI[Tasto];
					// {#endif}  //Fine NO PLAUNO_LIGHT
				END_IF;
				IF mcpi_t0   THEN iret:=key_Disp[0]; END_IF;
				IF mcpi_t1   THEN iret:=key_Disp[1]; END_IF;
				IF mcpi_t2   THEN iret:=key_Disp[2]; END_IF;
				IF mcpi_t3   THEN iret:=key_Disp[3]; END_IF;
				IF mcpi_t4   THEN iret:=key_Disp[4]; END_IF;
				IF mcpi_t5   THEN iret:=key_Disp[5]; END_IF;
				IF mcpi_t6   THEN iret:=key_Disp[6]; END_IF;
				IF mcpi_t7   THEN iret:=key_Disp[7]; END_IF;
				IF mcpi_t8   THEN iret:=key_Disp[8]; END_IF;
				IF mcpi_t9   THEN iret:=key_Disp[9]; END_IF;
				IF mcpi_t10  THEN iret:=key_Disp[10]; END_IF;
				IF mcpi_t11  THEN iret:=key_Disp[11]; END_IF;
				IF mcpi_t12  THEN iret:=key_Disp[12]; END_IF;
				IF mcpi_t13  THEN iret:=key_Disp[13]; END_IF;
				IF mcpi_t14  THEN iret:=key_Disp[14]; END_IF;
				// {#ifdef ETURN}
				// 	IF mcpo_t13=TRUE THEN %%%%
				// 		//Simulazione tasti per utensili SX
				// 		IF mcpi_t9  THEN iret:=key_Disp[15]; END_IF;
				// 		IF mcpi_t10 THEN iret:=key_Disp[16]; END_IF;
				// 		IF mcpi_t11 THEN iret:=key_Disp[17]; END_IF;
				// 		IF mcpi_t14 THEN iret:=key_Disp[18]; END_IF;
				// 	END_IF;
				// {#endif}  //Fine ETURN
				// {#ifdef DH4010} %%%%
				// 	IF mcpo_t13=TRUE THEN
				// 		//Simulazione tasti per testa SX
				// 		IF mcpi_t1  THEN iret:=key_Disp[22]; END_IF;  //Taglio sinistro
				// 		IF mcpi_t3  THEN iret:=key_Disp[15]; END_IF;  //Testina 5
				// 		IF mcpi_t4  THEN iret:=key_Disp[16]; END_IF;  //Testina 6
				// 		IF mcpi_t5  THEN iret:=key_Disp[21]; END_IF;  //Scomparsa spallamento sinistra
				// 		IF mcpi_t6  THEN iret:=key_Disp[17]; END_IF;  //Cambio testina 5 curva a sinistra
				// 		IF mcpi_t7  THEN iret:=key_Disp[19]; END_IF;  //Cambio testina 6 curva a sinistra
				// 		IF mcpi_t8  THEN iret:=key_Disp[23]; END_IF;  //Scomparsa pinza sinistra
				// 		IF mcpi_t9  THEN iret:=key_Disp[18]; END_IF;  //Cambio testina 5 curva a destra
				// 		IF mcpi_t10 THEN iret:=key_Disp[20]; END_IF;  //Cambio testina 6 curva a destra
				// 		IF mcpi_t11 THEN iret:=key_Disp[24]; END_IF;  //Pinza 3D SX
				// 		IF mcpi_t12 THEN iret:=key_Disp[25]; END_IF;  //Scomparsa correttore SX
				// 		IF mcpi_t14 THEN iret:=key_Disp[26]; END_IF;  //Scomparsa pinza 2D SX
				// 	END_IF;
				// {#endif}  //Fine DH4010
				CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_OPERAZIONE_COMPLESSA;
			END_IF;	
			
    _FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_OPERAZIONE_COMPLESSA:
			IF iret>=FIRST_COMPLEX_OPERATION THEN
				CURRENT_AXIS:=CURRENT_AXIS+1;
				IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
          PosAx:= AsSynchronousAxisRef(Axes[Asse].PosAx);

          IF ABS(PosAx^.StatusPositioning.Distance)>0.01 AND posax^.simulation.Mode=UINT#0 THEN
					//IF ABS(Axes[Asse].PosAx.poscommand.distancetogo)>0.01 AND Axes[Asse].posax.simulation<>ACTIVE THEN
						CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_RESET_AXIS;
					END_IF;
				ELSE
					IF old_i_sys_activedevice<>iret THEN
            //iret rappresenta il codice dell^operazione richiesta, se è cambiata rispetto
            //a quella precedente ed è richiesta una operazione complessa (motiontask CICLI)
            //si resetta tutto per evitare partenze indesiderate di un eventuale ciclo manuale
            //non completato
						old_i_sys_activedevice:=iret;
						(*dwret:=_resetTaskId(_task.motiontask_CICLI);*)
						CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_ELIMINA_CODA_COMANDI;
					ELSE
						CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_STOPCOND;
					END_IF;
				END_IF;
			ELSE
				CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_STOPCOND;
			END_IF;
			
    _FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_RESET_AXIS:
			IF (ResetMotionResetAxis.OutputState<>GENERAL_STATE#STATE_END) THEN
				//Per sicurezza si resettano gli assi che hanno ancora un percorso residuo
        ResetMotionResetAxis(Asse:=Asse, Attesa:=FALSE);
			ELSE
				ResetMotionResetAxis(Asse:=Asse, Attesa:=FALSE);
				CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_OPERAZIONE_COMPLESSA;
			END_IF;
		
    _FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_ELIMINA_CODA_COMANDI:
			IF (EliminaCodaComandi.OutputState<>GENERAL_STATE#STATE_END) THEN
				EliminaCodaComandi();
			ELSE
				EliminaCodaComandi();
				// dwret:=_restartTaskId(_task.motiontask_CICLI); %%
				CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_STOPCOND;
			END_IF;
			
    _FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_STOPCOND:
			// {#ifdef CURVATURA}
				IF iret<>MANUAL_BENDING AND iret<>-1 AND i_sys_activedevice=MANUAL_BENDING THEN
					//Ripristino dell^override ed azzeramento dello stop ciclo se si abbandona l^andata curvatura
					//manuale
					i_sys_swOVR:=OVERRIDE_STANDARD;
					i_sys_stopcond:=0;
				END_IF;
			// {#else}  //Fine CURVATURA
				i_sys_swOVR:=OVERRIDE_STANDARD;
				i_sys_stopcond:=0;
			// {#endif}  //Fine NO CURVATURA
			CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_OPERAZIONE_COMPLESSA_2;
			
    _FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_OPERAZIONE_COMPLESSA_2:
			IF i_sys_activedevice>=FIRST_COMPLEX_OPERATION AND old_i_sys_activedevice<>i_sys_activedevice THEN
				old_i_sys_activedevice:=i_sys_activedevice;
				// dwret:=_resetTaskId(_task.motiontask_CICLI); %%
				CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_ELIMINA_CODA_COMANDI_2;
			ELSE
				CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_RETURN;
			END_IF;
			
    _FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_ELIMINA_CODA_COMANDI_2:
			IF (EliminaCodaComandi.OutputState<>GENERAL_STATE#STATE_END) THEN
				EliminaCodaComandi();
			ELSE
				EliminaCodaComandi();
				// dwret:=_restartTaskId(_task.motiontask_CICLI); %%
				CURRENT_STATE:=_FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_RETURN;
			END_IF;
			
    _FB_Read_ActiveDevice#STATE_READ_ACTIVE_DEVICE_RETURN:
			//La movimentazione manuale di questi dispositivi comporta dei cicli complessi in quanto coinvolgono altri assi e quindi non
			//possono essere eseguiti nel background ma nel motiontask CICLI. Quindi si decrementa del valore FIRST_COMPLEX_OPERATION
			//il codice dell^asse/dispositivo inviato a VGP3D per poter farlo tornare un comando semplice
			// {#ifdef DH4010} %%%%
			// 	CASE iret OF
			// 		C_TESTINA_1_DX, C_TESTINA_1_SX, C_TESTINA_2_DX, C_TESTINA_2_SX, C_SCOMPARSA_SPALLAMENTO_DX, C_SCOMPARSA_SPALLAMENTO_SX:
			// 			iret:=iret-FIRST_COMPLEX_OPERATION;
			// 	END_CASE;
			// {#endif}  //Fine DH4010
			// {#ifdef EFLEX}
			// 	CASE iret OF
			// 		C_TESTINA_1, C_TESTINA_2:
			// 			iret:=iret-FIRST_COMPLEX_OPERATION;
			// 	END_CASE;
			// {#endif}  //Fine EFLEX
			ReturnValue:=iret;
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
			ReturnValue:=-1;
		
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_Change_OpMode

  VAR_EXTERNAL
    i_sys_svuotare_cassa	  :BOOL;
    i_sys_Machine_Process		:WORD;
    Sostegno2StopBackground	:SINT;
    Axes                    :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    AsseCheTieneTubo        :INT;
    FlagStopPointLavoro			:BOOL;  //True se la macchina in automatico sta eseguendo il pezzo ma il Master è fermo
    i_sys_AttesaOperatore		:BOOL;
    StatusPrimoStart,
    StatusManualeCarico,
    StatusRobotCarico       : BOOL;
    i_sys_RunOk             : INT;
    i_sys_TempoWarning        :REAL;
    i_sys_flg_attesa_slitta_1	:BOOL;
    i_sys_flg_attesa_slitta_2	:BOOL;
    i_sys_stopcond            : INT;
    i_sys_swOVR               : LREAL;
    i_sys_PROGSTATUS         : INT;
    i_sys_CHANSTATUS         : INT;
    i_sys_hwOVRLimit_for_scaling            : REAL;
    i_sys_hwOVRLimit                        : REAL;
    i_sys_CR_SENZA_SICUREZZE                :BOOL;
    StopPezzoSeguente           :BOOL;
    RichJobSospeso              :BOOL;
    OPS_Active                    :BOOL;
    i_sys_FlagCoccodrilloStopPoint:BOOL;      //True se in automatico viene attivato lo STOP POINT 
    i_sys_PosFinale							:ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS - 1] OF LREAL;
    AsseInLimitazioneCoppia		  :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1] OF BOOL;
    AllargaStandStill           :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1] OF BOOL;
    i_sys_AttivazioneNastro		  :BOOL;
    mcpo_JOG                    : BOOL;
    mcpo_AUTO                   : BOOL;
    mcpo_REF                    : BOOL;
    mcpo_MDA                    : BOOL;
    i_sys_OPMODE                : INT;
    mcpo_ncstop                 : BOOL;
    mcpo_ncstart                :BOOL;
    i_sys_current_msg           :DINT;
    i_sys_InUsoLetturaSinamics  :BOOL;
    i_sys_mem_weld              :ARRAY[0..1] OF LREAL; 
    i_sys_monitoring            : INT;
    i_sys_ViolazioneProgrammata             :BOOL;
    AL5_AperturaPinze_JOG					:BOOL;	//TRUE => Apertura pinze AL5 all^attivazione del JOG
    i_sys_ModoPezzoSeguente       : INT;
    ProTube                       :Produzione;
    i_sys_PezzoSeguenteDisattivato	        :BOOL;
    Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    CammeInterpolate                :BOOL;
    i_sys_chan_request              : INT;
    i_SYS_OPMODE_request            : INT;
    i_sys_singleblockactive         : BOOL;
    singleblock			                :Flipflop;
    pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    AssiSpenti                      :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;  //True se asse è stato spento
    i_sys_Carm7CaricoContinuo               :BOOL;
    TaskCicliAttivo                         :BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    OUT_AL5_MARCATURA_POWER_OFF                   :INT;
    OUT_AL5_MARCATURA_START_CICLO                 :INT;
    OUT_AL5_MARCATURA_SEL_PROG_BIT_0              :INT;
    OUT_AL5_MARCATURA_SEL_PROG_BIT_1              :INT;
    //Non usare da 200 a 213 in quanto indirizzi comuni a tutte le macchine !!!!
    OUT_AL5_MARCATURA_SEL_PROG_BIT_2              :INT;
    OUT_AL5_MARCATURA_SEL_PROG_BIT_3              :INT;
    OUT_AL5_MARCATURA_SEL_PROG_BIT_4              :INT;
    OUT_AL5_MARCATURA_SEL_PROG_BIT_5              :INT;
    OUT_AL5_REGOLAZIONE_PRESS_PNEUMATICA          :INT;
    OUT_AL5_ENABLE_MOTORE_ROTAZIONE_TB            :INT;
    OUT_AL5_SELEZ_VEL_MAX_ROT_TB                  :INT;
    OUT_AL5_SELEZ_VEL_MIN_ROT_TB                  :INT;
    OUT_AL5_START_ROLAND                          :INT;
    OUT_AL5_STROBE_SELEZ_ROLAND                   :INT;
    OUT_AL5_SELEZ_M1_ROLAND                       :INT;
    OUT_AL5_SELEZ_M2_ROLAND                       :INT;
    OUT_AL5_SELEZ_M4_ROLAND                       :INT;
    OUT_AL5_SELEZ_M8_ROLAND                       :INT;
    OUT_AL5_SELEZ_M16_ROLAND                      :INT;
    OUT_AL5_ESCLUSIONE_MAGNETIZZAZIONE            :INT;
    OUT_ATTESA_ENABLE_SELEZ_ESTERNA_DPCOUPLER     :INT;
    OUT_ATTESA_ENABLE_SELEZ_ESTERNA               :INT;
    JOGMODE                                       :INT;
    REFMODE                                       :INT;
    MDAMODE                                       :INT;
    AUTOMODE                                      :INT;
    UNDEF_DEVICE                       	          :INT;
    OUT_MACCHINA_ATTESA_START_DPCOUPLER           :INT;
    OUT_TUBO_OK																		:INT;
    MY_BIG_VALUE_REAL                             :REAL;
    FIRST_DEVICE                                  :INT;
    OUT_MACCHINA_IN_CICLO_AUTOMATICO              :INT;
    PROGSTATUS_RESET 													    :INT;
    CHANNELRESET                                  :INT;									//PROGRAMMI TUTTI RESETTATI
    OVERRIDE_STANDARD                          	  :INT;
    BOOSTER                                       :INT;  //35
    RIPOSO                                        :SINT;
    VAR_INC_INIT                                  :INT;
    MSG_NOMESSAGE							                    :INT;
    PROGRAM_JOG                      	            :SINT;
    JOB_STATE_PRODUZIONE                          :SINT;
    JOB_STATE_SOSPESO                             :SINT;
    MODO_PEZZO_SEGUENTE                           :SINT;
    AT_STATE_UNDEFINED                	          :SINT;
    AL5_PINZA_DX                                  :INT;  //47
    AL5_PINZA_SX                                  :INT;
    MODO_PEZZO_SEGUENTE_SINGOLO                   :SINT;
    MODO_PEZZO_STANDARD                           :SINT;
  END_VAR

  VAR_INPUT
    Mode  :INT;
  END_VAR

  VAR
    iret  :INT;
    dwret :DWORD;
    Esito :BOOL;
    dret  :DINT;
    PosAxes : REF_TO TO_SynchronousAxis;
		
		CURRENT_STATE				:INT:=GENERAL_STATE#STATE_INIT;
		CURRENT_AXIS				:INT;
		EliminaCodaComandi	:FB_EliminaCodaComandi;
		DisableAssi					:FB_DisableAxes;
		ResetMotionResetAxis:FB_ResetMotionResetAxis;
		ResetAL							:FB_ResetAL;
		ArrestaMotori				:FB_ArrestaMotori;
    ResettingXxOrbita : ResettingXxOrbita;
    
  END_VAR

  VAR_TEMP
    i     :INT;
  END_VAR

	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			i_sys_svuotare_cassa:=FALSE;
			i_sys_Machine_Process:=WORD#0;
			Sostegno2StopBackground:=SINT#-1;
			SistemaUtensili();
			FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 DO
				Axes[i].statocamming:=FALSE;
			END_FOR;
			AsseCheTieneTubo:=UNDEF_DEVICE;
			FlagStopPointLavoro:=FALSE;
			WriteDigitalOutput(NumOutput:=OUT_MACCHINA_ATTESA_START_DPCOUPLER, Value:=FALSE);
			WriteDigitalOutput(NumOutput:=OUT_TUBO_OK, Value:=FALSE, Verifica:=FALSE);
			i_sys_AttesaOperatore:=FALSE; 
			StatusPrimoStart:=FALSE;
			StatusRobotCarico:=FALSE;
			StatusManualeCarico:=FALSE;
			// {#ifdef AL_BLM}
				WriteDigitalOutput(NumOutput:=OUT_AL5_MARCATURA_POWER_OFF, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_MARCATURA_START_CICLO, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_MARCATURA_SEL_PROG_BIT_0, Value:=FALSE);
				// WriteDigitalOutput(NumOutput:=OUT_AL5_MARCATURA_SEL_PROG_BIT_1, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_MARCATURA_SEL_PROG_BIT_2, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_MARCATURA_SEL_PROG_BIT_3, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_MARCATURA_SEL_PROG_BIT_4, Value:=FALSE);
			// {#endif}	//Fine AL_BLM
			// {#ifdef MS0017} %%%%
			// 	IF IndicaTipoAl()=TIPO_MS0017 THEN
			// 		ProgrammiX[0].num_pezzi_fatti:=0;
			// 		ProgrammiX[0].num_pezzi_totali:=0;
			// 		ProgrammiX[0].num_pezzi_fatti_job:=0;
			// 		ProgrammiX[0].num_pezzi_totali_job:=0;
			// 	ELSE
			// 		ProgrammiX[0].num_pezzi_fatti_AL5:=0;
			// 		ProgrammiX[0].num_pezzi_totali_AL5:=0;
			// 	END_IF;
			// {#endif}  //Fine MS0017
			// {#ifdef MODULO_ROLAND}
				WriteDigitalOutput(NumOutput:=OUT_AL5_START_ROLAND, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_ESCLUSIONE_MAGNETIZZAZIONE, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_SELEZ_M1_ROLAND, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_SELEZ_M2_ROLAND, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_SELEZ_M4_ROLAND, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_SELEZ_M8_ROLAND, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_SELEZ_M16_ROLAND, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_STROBE_SELEZ_ROLAND, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_ENABLE_MOTORE_ROTAZIONE_TB, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_SELEZ_VEL_MAX_ROT_TB, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_SELEZ_VEL_MIN_ROT_TB, Value:=FALSE);
				WriteDigitalOutput(NumOutput:=OUT_AL5_REGOLAZIONE_PRESS_PNEUMATICA, Value:=FALSE);
			// {#endif} //Fine MODULO_ROLAND %%%%
			// {#ifdef CARICO_SCARICO_PARALLELI}
			// 	dwret:=_resetTaskId(_task.motiontask_LoaderParallelo);
			// {#endif}	//Fine CARICO_SCARICO_PARALLELI
			// {#ifdef ORBITALE}
        ResettingXxOrbita();
				CalculateFactorPrecommand();
			// 	{#ifndef EFLEX}
			// 		{#ifndef DH4010}
			// 			dwret:=_resetTaskId(_task.motiontask_LevaParallela);
			// 			WriteDigitalOutput(NumOutput:=OUT_MARCATRICE_PRINT_TARGET_DETECTOR, Value:=FALSE);
			// 		{#endif}  //Fine NO DH4010
			// 	{#endif}  //Fine NO EFLEX
			// 	{#ifdef FOTOCELLULA_MISURA_X1}
			// 		DisableTastatore(NomeTO:=_to.FotocellulaMisuraX1);
			// 	{#endif}  //Fine FOTOCELLULA_MISURA_X1
			// 	WriteDigitalOutput(NumOutput:=OUT_ASPO_VERTICALE_NO_ATTREZZAGGIO, Value:=FALSE);
			// 	{#ifdef RUNNER}
			// 		BloccaAspo:=FALSE;
			// 		ResetCountLubrLav();
			// 		QuotaAvvioTaglio:=0;
			// 		{#ifdef FOUR_RUNNER_PLAUNO}
			// 			DisableTastatore(NomeTO:=_to.FotocellulaMisuraCartella);
			// 			ForaturaInCorso:=FALSE;
			// 			i_sys_EsegueMarcatura:=-1;
			// 			i_sys_OverrideMarcatura:=FALSE;
			// 		{#endif}  //Fine FOUR_RUNNER_PLAUNO
			// 	{#endif}  //Fine RUNNER
			// 	{#ifdef EFLEX_DH4010}
			// 		Set_Disp(Ndisp:=SMUSSATORE_MOTORE, Stato:=RIPOSO, TimerRisparmio:=TRUE);
			// 		set_Activedevice(iDeviceNumber:=UNDEF_DEVICE);
			// 	{#endif}  //Fine EFLEX_DH4010
			// {#else} //Fine ORBITALE
			// 	{#ifdef MISURA_RITORNO_ELASTICO_DUE_SENSORI}
			// 		DL_PN1:=_setbit(DL_PN1, 2, FALSE);    //Scongela il risultato della precedente misura
			// 	{#endif} //FINE MISURA_RITORNO_ELASTICO_DUE_SENSORI
			// 	DisableTastatore(NomeTO:=_to.Weld_Marker);
			// 	WriteDigitalOutput(NumOutput:=OUT_TAGLIO_COLTELLO_ORIZZONTALE, Value:=FALSE);
			// 	WriteDigitalOutput(NumOutput:=OUT_TAGLIO_COLTELLO_VERTICALE, Value:=FALSE);
			// 	{#ifdef PROFINET}
			// 		O_BarrieraDaSimotion:=FALSE;
			// 		WriteDigitalOutput(NumOutput:=OUT_LUBR_CONTROSLITTA_UT1, Value:=FALSE);
			// 		WriteDigitalOutput(NumOutput:=OUT_LUBR_CONTROSLITTA_UT2, Value:=FALSE);
			// 		WriteDigitalOutput(NumOutput:=OUT_LUBR_CONTROSLITTA_UT3, Value:=FALSE);
			// 		WriteDigitalOutput(NumOutput:=OUT_LUBR_CONTROSLITTA_UT4, Value:=FALSE);
			// 	{#endif}  //Fine PROFINET
			// {#endif} //Fine NO ORBITALE
			WriteDigitalOutput(NumOutput:=OUT_ATTESA_ENABLE_SELEZ_ESTERNA_DPCOUPLER, Value:=FALSE);
			WriteDigitalOutput(NumOutput:=OUT_ATTESA_ENABLE_SELEZ_ESTERNA, Value:=FALSE);
			ResetAllOutStopPoint();
			CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_ELIMINA_CODA_COMANDI;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_ELIMINA_CODA_COMANDI:
			IF (EliminaCodaComandi.outputState<>GENERAL_STATE#STATE_END) THEN
				EliminaCodaComandi();
			ELSE
				EliminaCodaComandi();
				ResetStatoRichiesto();
				IF (i_sys_runok=1) THEN
					CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_RESET_AL;
				ELSE
					CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_ARRESTA_MOTORI;
				END_IF;
			END_IF;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_RESET_AL:
			IF (ResetAL.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetAL();
			ELSE
				ResetAL();
				CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_ARRESTA_MOTORI;
			END_IF;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_ARRESTA_MOTORI:
			IF (ArrestaMotori.outputState<>GENERAL_STATE#STATE_END) THEN
				ArrestaMotori(Attesa:=FALSE);
			ELSE
				ArrestaMotori(Attesa:=FALSE);
				CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_POST_ARRESTO;
			END_IF;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_POST_ARRESTO:
			i_sys_TempoWarning:=0;
			i_sys_flg_attesa_slitta_1:=FALSE;
			i_sys_flg_attesa_slitta_2:=FALSE;
			i_sys_stopcond:=0;
			i_sys_swovr:=100;
			i_sys_progstatus:=PROGSTATUS_RESET;     
			i_sys_chanstatus:=CHANNELRESET; 
			i_sys_HWOvrLimit_for_Scaling:=OVERRIDE_STANDARD;
			i_sys_HWOvrLimit:=OVERRIDE_STANDARD;
			i_sys_CR_SENZA_SICUREZZE:=FALSE;
			RichJobSospeso:=FALSE;
			StopPezzoSeguente:=FALSE;
			ResetOutCaricoScaricoRobot();
			i_sys_FlagCoccodrilloStopPoint:=FALSE;
			OutScartoLavorazione(Stato:=FALSE);
			FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
				Axes[i].DistanceToGo:=0;
				Axes[i].Coppia_Massima_Misurata_in_ciclo:=0;      
				Axes[i].Contatore_Coppia_Misurata:=0;
				Axes[i].Somma_Coppia_Misurata:=0;
				i_sys_PosFinale[i]:=-MY_BIG_VALUE_REAL;
				AllargaStandStill[i]:=FALSE;
				AsseInLimitazioneCoppia[i]:=FALSE;
			END_FOR;		
			FOR i:=FIRST_DEVICE TO (_MAXDEVICES#MAXDEVICES+FIRST_DEVICE-1) DO
				//Abilitazione movimento dispositivi
				AbilitazioneDispositivi(Dispositivo:=i, Stato:=TRUE);
			END_FOR;
			(*dwret:=_restartTaskId(_task.MotionTask_TempoStandStill);*)
			WriteDigitalOutput(NumOutput:=OUT_MACCHINA_IN_CICLO_AUTOMATICO, Value:=FALSE);
			MacchinaInWarning(Stato:=FALSE);
			Res_StatoOut(); 
			IF OPS_Active=FALSE THEN
				i_sys_AttivazioneNastro:=FALSE;
				ComandoNastroTrasportatore(Stato:=FALSE);
			END_IF;
			// {#ifdef ORBITALE} %%%%
			// 	i_sys_CicloContinuo:=0;
			// 	SetLedCicloContinuo(Stato:=FALSE);
			// {#else} // FINE ORBITALE
				ImpostaOutWeld(Stato:=FALSE);
				ImpostaOutMisuraAnima(Stato:=FALSE);
				// {#ifdef CAMBIO_MECCANICO}
					IF Mode<>MDAMODE THEN
						//Sgancia il booster tranne quando è richiesto l^MDA
						Set_disp(Ndisp:=BOOSTER, Stato:=RIPOSO, TimerRisparmio:=TRUE);
					END_IF;
			// 	{#endif}  //Fine CAMBIO_MECCANICO
			// {#endif}  //Fine NO ORBITALE
			//Cancella eventuale messaggio verde
			i_sys_current_msg:=MSG_NOMESSAGE;
			i_sys_InUsoLetturaSinamics:=FALSE;
			// {#ifndef PROFINET}
				IF Mode<>AUTOMODE THEN
					//Attiva il programma del radar per modalità manuale
					ComandaRadar(Programma:=PROGRAM_JOG, ModoOperativo:=Mode);
				END_IF;
			// {#endif}  //Fine NO PROFINET
			PompatoreCARM7(Stato:=FALSE);
			FOR i:=0 TO 1 BY 1 DO
				i_sys_mem_weld[i]:=-MY_BIG_VALUE_REAL;
			END_FOR;
			IF i_sys_monitoring=0 THEN ;
				// {#ifdef DH4010} %%%%
				// 	//Per la DH4010 non si debbono toccare i limiti di X1, X2 e X3 in quanto impostati
				// 	//dal backgroud in accordo con i limiti calcolati e trasferiti da VGP
				// 	SetAxSwLimit1(PrimoAsseNoGestione:=X1, SecondoAsseNoGestione:=X2, TerzoAsseNoGestione:=X3);
				// {#else}  //Fine DH4010
				// 	SetAxSwLimit1();   
				// {#endif}  //Fine NO DH4010
			END_IF;  
			// {#ifdef CURVATURA} %%%%
			// 	{#ifndef EFLEX_DH4010}
			// 		CalcolaUltimoUtPila();
			// 	{#endif}  //Fine NO EFLEX_DH4010
			// {#endif}  //Fine CURVATURA
			CASE MODE OF
				JOGMODE:
					CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_JOGMODE;
				
				REFMODE:
					CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_REFMODE;
					
				MDAMODE:
					CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_MDAMODE;
				
				AUTOMODE:
					CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_AUTOMODE;
			END_CASE;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_JOGMODE:
			i_sys_OPMODE:=JOGMODE;
			i_sys_progstatus:=PROGSTATUS_RESET; 
			mcpo_JOG:=TRUE;
			mcpo_AUTO:=FALSE;
			mcpo_REF:=FALSE;
			mcpo_MDA:=FALSE;
			var_inc(VAR_INC_INIT);
			i_sys_ViolazioneProgrammata:=FALSE;
			IF i_sys_ModoPezzoSeguente=MODO_PEZZO_SEGUENTE_SINGOLO AND Protube.JobState=JOB_STATE_PRODUZIONE THEN
				SetJobState(Stato:=JOB_STATE_SOSPESO, ForzaturaStato:=TRUE);
			END_IF;
			IF (OPS_Active=FALSE AND i_sys_PezzoSeguenteDisattivato=TRUE) OR
				 (OPS_Active=FALSE AND i_sys_ModoPezzoSeguente=MODO_PEZZO_SEGUENTE)THEN
				//Reset di un eventuale richiesta di pezzo seguente che potrebbe essere rimasta attiva a causa di una disabilitazione del pezzo seguente in VGP
				//senza che si sia conclusa la produzione prevista. Non si esegue il reset della variabile se OPS attivo.
				FOR i:=0 TO TO_INT(_MPROGRAMS#MAXPROGRAMS-UINT#1) DO
					programmi[i].programmavalido:=FALSE;
				END_FOR;
				CammeInterpolate:=FALSE;
				i_sys_modopezzoseguente:=MODO_PEZZO_STANDARD;
				IF ProTube.JobState=JOB_STATE_PRODUZIONE THEN
					SetJobState(Stato:=JOB_STATE_SOSPESO, ForzaturaStato:=TRUE);
				END_IF;
			END_IF;
			i_sys_PezzoSeguenteDisattivato:=FALSE;
      // {#ifdef THREE_RUNNER} %%%%
      //   Sostegno3RIPC_ComandoManuale:=FALSE;
      //   Sostegno4RFORM_ComandoManuale:=FALSE;
      // {#endif} //FINE THREE_RUNNER
      // dwret:=_resetTaskId(_task.motiontask_AUTO);
      // {#ifdef DH4010    }
      //   dwret:=_resetTaskId(_task.MotionTask_AutoSx);
      //   dwret:=_resetTaskId(_task.MotionTask_AutoDx);
      // {#endif}  //Fine DH4010
      // dwret:=_resetTaskId(_task.motiontask_MDA);
      // dwret:=_resetTaskId(_task.motiontask_CICLI);
      // dwret:=_resetTaskId(_task.motiontask_Weld_Marker);
      // {#ifdef FOUR_RUNNER  }
      //   //SF [30/05/2018]: ora il motiontask viene interrotto al cambio di modo
      //   dwret:=_resetTaskId(_task.MotionTask_CaricoBoccola);
      // {#endif} //FINE FOUR_RUNNER
			// {#ifdef AL_BLM}
				IF AL5_AperturaPinze_JOG=TRUE THEN
					ImpostaAL(Stato:=AT_STATE_UNDEFINED, Movimento:=FALSE);
					WriteDigitalOutput(NumOutput:=OUT_AL5_REGOLAZIONE_PRESS_PNEUMATICA, Value:=FALSE);
					Set_Disp(Ndisp:=AL5_PINZA_DX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
					Set_Disp(Ndisp:=AL5_PINZA_SX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
					// {#ifdef AL5_LUNGO} %%%%
					// 	Set_Disp(Ndisp:=AL5_PINZA_3, Stato:=RIPOSO, TimerRisparmio:=TRUE);
					// 	Set_Disp(Ndisp:=AL5_PINZA_4, Stato:=RIPOSO, TimerRisparmio:=TRUE);
					// {#endif}	//Fine AL5_LUNGO
				END_IF;
			// {#endif}	//Fine AL_BLM
      i_sys_chan_request:=0;
      i_sys_OPMODE_request:=0;
      mcpo_NCSTOP:=FALSE;
      mcpo_NCSTART:=FALSE;
      singleblock(s1:=BOOL#0, r1:=BOOL#1, q1=>i_sys_singleblockactive);
      AzzeramentoDistanceTogo();
			CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
			CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_LOOP_ASSI_JOGMODE;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_LOOP_ASSI_JOGMODE:
			CURRENT_AXIS:=CURRENT_AXIS+1;
			IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
        PosAxes:= AsSynchronousAxisRef(Axes[i].PosAx);
        IF TestToAttivo(Asse:=i)=TRUE AND pm[i].ma_asse_abilitato=TRUE AND PosAxes^.VirtualAxis.Mode=UINT#0 AND 
				pm[i].ma_AsseSpegnibile=TRUE AND PosAxes^.StatusWord.%X0 THEN
					//Disabilitazione degli assi spegnibili
					AssiSpenti[i]:=TRUE;
					CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_RESET_AXIS_JOGMODE;
				END_IF;
			ELSE
				i_sys_Carm7CaricoContinuo:=FALSE;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_RESET_AXIS_JOGMODE:
			IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetMotionResetAxis(Asse:=CURRENT_AXIS);
			ELSE
				ResetMotionResetAxis(Asse:=CURRENT_AXIS);
				CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_DISABLE_AXIS_JOGMODE;
			END_IF;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_DISABLE_AXIS_JOGMODE:
			IF (DisableAssi.outputState<>GENERAL_STATE#STATE_END) THEN
				DisableAssi(Asse:=CURRENT_AXIS(* , Regolatore:=INACTIVE*));
			ELSE
				DisableAssi(Asse:=CURRENT_AXIS (* , Regolatore:=INACTIVE*));
				CURRENT_STATE:=_FB_Change_OpMode#STATE_CHANGE_OPMODE_LOOP_ASSI_JOGMODE;
			END_IF;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_REFMODE:
      i_sys_OPMODE:=REFMODE;
      i_sys_progstatus:=PROGSTATUS_RESET; 
      mcpo_JOG:=TRUE;
      mcpo_AUTO:=FALSE;
      mcpo_REF:=TRUE;
      mcpo_MDA:=FALSE;
      var_inc(VAR_INC_INIT);
      i_sys_ViolazioneProgrammata:=FALSE;
      // {#ifdef THREE_RUNNER} %%%%
      //   Sostegno3RIPC_ComandoManuale:=FALSE;
      //   Sostegno4RFORM_ComandoManuale:=FALSE;
      // {#endif} //FINE THREE_RUNNER
      (*dwret:=_resetTaskId(_task.motiontask_AUTO);*)
      // {#ifdef DH4010} %%%%
      //   dwret:=_resetTaskId(_task.MotionTask_AutoSx);
      //   dwret:=_resetTaskId(_task.MotionTask_AutoDx);
      // {#endif}  //Fine DH4010
      (*dwret:=_resetTaskId(_task.motiontask_MDA);
      dwret:=_resetTaskId(_task.motiontask_CICLI);
      dwret:=_resetTaskId(_task.motiontask_Weld_Marker);*)
      TaskCicliAttivo:=FALSE;
      i_sys_chan_request:=0;
      i_sys_OPMODE_request:=0;      
      mcpo_NCSTOP:=FALSE;
      mcpo_NCSTART:=FALSE;
      singleblock(s1:=BOOL#0, r1:=BOOL#1, q1=>i_sys_singleblockactive);
      AzzeramentoDistanceTogo();
      i_sys_Carm7CaricoContinuo:=FALSE;
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_MDAMODE:
      i_sys_OPMODE:=MDAMODE;
      mcpo_JOG:=TRUE;
      mcpo_AUTO:=FALSE;
      mcpo_REF:=FALSE;
      mcpo_MDA:=TRUE;
      var_inc(VAR_INC_INIT);
      i_sys_ViolazioneProgrammata:=FALSE;
      // {#ifdef THREE_RUNNER} %%%%
			// 	Sostegno3RIPC_ComandoManuale:=FALSE;
			// 	Sostegno4RFORM_ComandoManuale:=FALSE;
      // {#endif} //THREE_RUNNER
      // dwret:=_resetTaskId(_task.motiontask_AUTO);
      // {#ifdef DH4010}
      //   dwret:=_resetTaskId(_task.MotionTask_AutoSx);
      //   dwret:=_resetTaskId(_task.MotionTask_AutoDx);
      // {#endif}  //Fine DH4010
      (*dwret:=_resetTaskId(_task.motiontask_CICLI);
      dwret:=_resetTaskId(_task.motiontask_Weld_Marker);*)
      TaskCicliAttivo:=FALSE;
      (*dwret:=_restartTaskId(_task.motiontask_MDA);*)
      i_sys_chan_request:=0;
      i_SYS_OPMODE_request:=0;      
      mcpo_NCSTOP:=FALSE;
      mcpo_NCSTART:=FALSE;
      singleblock(s1:=BOOL#0, r1:=BOOL#1, q1=>i_sys_singleblockactive);
      AzzeramentoDistanceTogo();
      i_sys_Carm7CaricoContinuo:=FALSE;
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
		
    _FB_Change_OpMode#STATE_CHANGE_OPMODE_AUTOMODE:
      //Serve per evitare errori dovuti all^accesso alla tabella stati al momento
      //del cambio di modo
      Programmi[0].TabStatiInizio:=Programmi[PROGRAM_JOG].TabStatiInizio;
      Programmi[0].TabStatiFine:=Programmi[PROGRAM_JOG].TabStatiFine;
      programmi[0].numstati:=programmi[PROGRAM_JOG].numstati;
      i_sys_OPMODE:=AUTOMODE;
      i_sys_progstatus:=PROGSTATUS_RESET;
      mcpo_JOG:=FALSE;
      mcpo_AUTO:=TRUE;
      mcpo_REF:=FALSE;
      mcpo_MDA:=FALSE;
      // {#ifdef DH4010} %%%%
      //   //Per la DH4010 non si debbono toccare i limiti di X1, X2 e X3 in quanto impostati
      //   //dal backgroud in accordo con i limiti calcolati e trasferiti da VGP
      //   SetAxSwLimit1(PrimoAsseNoGestione:=X1, SecondoAsseNoGestione:=X2, TerzoAsseNoGestione:=X3);
      // {#else} //No DH4010
      //   SetAxSwLimit1();
      // {#endif}  //Fine DH4010
      // set_Activedevice(iDeviceNumber:=UNDEF_DEVICE);
      // var_inc(VAR_INC_INIT);
      // i_sys_ViolazioneProgrammata:=FALSE;
      // {#ifdef THREE_RUNNER}
      //   Sostegno3RIPC_ComandoManuale:=FALSE;
      //   Sostegno4RFORM_ComandoManuale:=FALSE;
      // {#endif} //FINE THREE_RUNNER
      // {#ifdef DH4010    }
      //   dwret:=_resetTaskId(_task.MotionTask_AutoSx);
      //   dwret:=_resetTaskId(_task.MotionTask_AutoDx);
      // {#endif}  //Fine DH4010
      (*dwret:=_resetTaskId(_task.motiontask_MDA);
      dwret:=_resetTaskId(_task.motiontask_CICLI);
      dwret:=_resetTaskId(_task.motiontask_Weld_Marker);*)
      TaskCicliAttivo:=FALSE;
      (*dwret:=_restartTaskId(_task.motiontask_AUTO);*)
      i_sys_chan_request:=0;
      i_sys_OPMODE_request:=0;      
      mcpo_NCSTOP:=FALSE;
      mcpo_NCSTART:=FALSE;
      singleblock(s1:=BOOL#0, r1:=BOOL#1, q1=>i_sys_singleblockactive);
      AzzeramentoDistanceTogo();
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
		
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK 

/*****************************************************************************
 * CHECKS WHETHER THE PIPE IS PRESENT ON THE^AL AND IS READY FOR LOADING, IF *
 *          NEGATIVE IT SETS THE STATUS AND POSITION TO UNDEFINED.           *
 *           IT ALSO RESETS OUTPUTS THAT CONTROL HAZARDOUS DEVICES           *
 *****************************************************************************/

FUNCTION_BLOCK FB_ResetAL

  VAR_EXTERNAL
    i_sys_OPMODE                          : INT;
    i_sys_current_prog	                  :INT;
    i_sys_AL_Robot                        :BOOL;
    OAddress                              : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
    Disp                                  :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
  END_VAR

  VAR_EXTERNAL CONSTANT
    AUTOMODE                              :INT;
    PROGRAM_JOG                      	    :SINT;
    TIPO_AL42                          	  :SINT;
    TIPO_AL42PLUS                      	  :SINT;
    MAXDIGITALOUTPUT                      :INT;
    OUT_ALZO_SOSTEGNO_TUBO_AL42           :INT;
    OUT_ABBASSO_SOSTEGNO_TUBO_AL42        :INT;
    CLAMP_CARGE                           :INT;
    FIRST_DEVICE                          :INT;
    AL42_STATO_CARICO                  	  :SINT;
    AT_STATE_UNDEFINED                	  :SINT;
    TIPO_AL5                          	  :SINT;
    TIPO_AL5_ROLAND                    	  :SINT;		//AL5 con sistema di misura ROLAND
    TIPO_AL5_LUNGO                    	  :SINT;
    AL5_CARICO_MACCHINA                	  :SINT;	//AL con tubo pronto in posizione di 
                                                                //carico tubo in macchina con pinze 
                                                                //chiuse, se presente traslatore avanti 
    TIPO_AL5_BUNDLE                    	:SINT;
    TIPO_AL5_ROLAND_BUNDLE            	:SINT;		//AL5+BUNDLE con sistema di misura ROLAND
    TIPO_AL5_LUNGO_BUNDLE              	:SINT;	//AL5 speciale lungo 6mt+BUNDLE con sistema di misura ROLAND
    TIPO_MS0017                        	:SINT;	//Caricatore per ditta Sogefi
    TIPO_AL5_FUTABA                    	:SINT;	//Caricatore per ditta Futaba con ricerca saldatura Roland   
    OUT_PRESSIONE_PNEUMATICA_AL         :INT;
  END_VAR

  VAR
    dwret     :DWORD;
    TipoAl    :SINT;  
    dret      :DINT;
    Address   :DINT;
    StatoAsse :LREAL;
    Programma :INT;
		
		CURRENT_STATE:INT:=GENERAL_STATE#STATE_INIT;
		AL5_Sollevatore	:FB_AL5_Sollevatore;
		AL5Catene				:FB_AL5Catene;
  END_VAR
	VAR_OUTPUT
		OutputState:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF i_sys_OpMode=AUTOMODE THEN
				Programma:=I_SYS_Current_Prog;
			ELSE
				Programma:=PROGRAM_JOG;
			END_IF;
			// {#ifdef AL_BLM}
				i_sys_AL_Robot:=FALSE;
    		TipoAl:=IndicaTipoAl();
				CASE TipoAL OF
					TIPO_AL42, TIPO_AL42PLUS:
						//Abilitazione forzatura uscite
						oaddress[OUT_ALZO_SOSTEGNO_TUBO_AL42].enable_forzatura:=FALSE;
						oaddress[OUT_ABBASSO_SOSTEGNO_TUBO_AL42].enable_forzatura:=FALSE;
						(*dwret:=_resetTaskId(_task.motiontask_AL);*) 
						Disp[CLAMP_CARGE].in_movimento:=FALSE;
						IF TestALReady()=TRUE THEN
							ImpostaAL(Stato:=AL42_STATO_CARICO, Movimento:=FALSE);          
						ELSE
							//Stato CARICO AL42 non compatibile con la situazione reale dei 
							//dispositivi quindi si mette ad indefinito lo stato del sistema
							Disp[CLAMP_CARGE].stato:=AT_STATE_UNDEFINED;
							Disp[CLAMP_CARGE].position:=AT_STATE_UNDEFINED;
						END_IF;
						CURRENT_STATE:=_FB_ResetAL#STATE_RESET_AL_RESET_USCITE_CARICO;
						
					TIPO_AL5, TIPO_AL5_ROLAND, TIPO_AL5_LUNGO:
						oaddress[OUT_ALZO_SOSTEGNO_TUBO_AL42].enable_forzatura:=FALSE;
						oaddress[OUT_ABBASSO_SOSTEGNO_TUBO_AL42].enable_forzatura:=FALSE;
						(*dwret:=_resetTaskId(_task.motiontask_AL);*) 
						Disp[CLAMP_CARGE].in_movimento:=FALSE;
						IF TestALReady()=TRUE THEN
							ImpostaAL(Stato:=AL5_CARICO_MACCHINA, Movimento:=FALSE);
						ELSE
							//Stato CARICO AL42 non compatibile con la situazione reale dei 
							//dispositivi quindi si mette ad indefinito lo stato del sistema
							Disp[CLAMP_CARGE].stato:=AT_STATE_UNDEFINED;
							Disp[CLAMP_CARGE].position:=AT_STATE_UNDEFINED;
						END_IF;
						CURRENT_STATE:=_FB_ResetAL#STATE_RESET_AL_RESET_USCITE_CARICO;
						
					TIPO_AL5_BUNDLE, TIPO_AL5_ROLAND_BUNDLE, TIPO_AL5_LUNGO_BUNDLE, TIPO_MS0017, TIPO_AL5_FUTABA:
						oaddress[OUT_ALZO_SOSTEGNO_TUBO_AL42].enable_forzatura:=FALSE;
						oaddress[OUT_ABBASSO_SOSTEGNO_TUBO_AL42].enable_forzatura:=FALSE;
						(*dwret:=_resetTaskId(_task.motiontask_AL);*)
						Disp[CLAMP_CARGE].in_movimento:=FALSE;
						IF TestALReady()=TRUE THEN
							ImpostaAL(Stato:=AL5_CARICO_MACCHINA, Movimento:=FALSE);
						ELSE
							//Stato CARICO AL42 non compatibile con la situazione reale dei 
							//dispositivi quindi si mette ad indefinito lo stato del sistema
							Disp[CLAMP_CARGE].stato:=AT_STATE_UNDEFINED;
							Disp[CLAMP_CARGE].position:=AT_STATE_UNDEFINED;
						END_IF;	
						CURRENT_STATE:=_FB_ResetAL#STATE_RESET_AL_AL5_SOLLEVATORE;
						
				ELSE
					//Per sicurezza abbasso la leva di carico, anche se teoricamente nell^AL42
					//non dovrebbe essere presente
					oaddress[OUT_PRESSIONE_PNEUMATICA_AL].stato:=TRUE;
					//Abilitazione forzatura uscite
					oaddress[OUT_ALZO_SOSTEGNO_TUBO_AL42].enable_forzatura:=TRUE;
					oaddress[OUT_ABBASSO_SOSTEGNO_TUBO_AL42].enable_forzatura:=TRUE;
					//Forzatura uscite per abbasso leva
					oaddress[OUT_ALZO_SOSTEGNO_TUBO_AL42].stato_forzatura:=FALSE;
					oaddress[OUT_ABBASSO_SOSTEGNO_TUBO_AL42].stato_forzatura:=TRUE;
					//Memorizzazione condizione di uscite per leva bassa per successivo reset
					//stato forzatura
					oaddress[OUT_ALZO_SOSTEGNO_TUBO_AL42].statopreforzatura:=SINT#0;
					oaddress[OUT_ABBASSO_SOSTEGNO_TUBO_AL42].statopreforzatura:=SINT#1;
					CURRENT_STATE:=_FB_ResetAL#STATE_RESET_AL_RESET_USCITE_CARICO;
				END_CASE;
			// {#endif} //FINE AL_BLM
		
    _FB_ResetAL#STATE_RESET_AL_AL5_SOLLEVATORE:
			// {#ifdef AL_BLM}
				AL5FrenoSollevatore(Stato:=FALSE);
				IF (AL5_Sollevatore.outputState<>GENERAL_STATE#STATE_END) THEN
					AL5_Sollevatore(Stato:=DEVICES_STATUS#ARRESTO, Attesa:=FALSE);
				ELSE
					AL5_Sollevatore(Stato:=DEVICES_STATUS#ARRESTO, Attesa:=FALSE);
					CURRENT_STATE:=_FB_ResetAL#STATE_RESET_AL_AL5_CATENE;
				END_IF;
			// {#endif} //FINE AL_BLM
			
    _FB_ResetAL#STATE_RESET_AL_AL5_CATENE:
			// {#ifdef AL_BLM}
				IF (AL5Catene.outputState<>GENERAL_STATE#STATE_END) THEN
					AL5Catene(Stato:=DEVICES_STATUS#ARRESTO);
				ELSE
					AL5Catene(Stato:=DEVICES_STATUS#ARRESTO);
					AL5LucePrenotazioneCarico(Stato:=FALSE);
					CURRENT_STATE:=_FB_ResetAL#STATE_RESET_AL_RESET_USCITE_CARICO;
				END_IF;
			// {#endif} //FINE AL_BLM

    _FB_ResetAL#STATE_RESET_AL_RESET_USCITE_CARICO:
			//Reset uscite dialogo AL con robot
			ALRobotProntoScarico(Stato:=FALSE, Programma:=Programma);  
			ALRobotTuboLibero(Stato:=FALSE, Programma:=Programma);
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_ArrestaMotori
  VAR_INPUT
    Attesa  :BOOL;
  END_VAR  
	VAR
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
		// {#ifdef RUNNER} %%%%
		// 	TaglioRTL			:FB_TaglioRTL;
		// {#endif}
	END_VAR
	VAR_OUTPUT
		OutputState		:INT;
	END_VAR
	// CASE CURRENT_STATE OF %%%%
	// 	GENERAL_STATE#STATE_INIT:
  //   ;
	// 		// {#ifdef IPC_DISPOSITIVO} %%%%
	// 		// 	Set_Disp(Ndisp:=IPC_MOTORE, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 		// {#endif}  //Fine IPC_DISPOSITIVO
	// 		// {#ifdef EFLEX_DH4010}
	// 		// 	Set_Disp(Ndisp:=SMUSSATORE_MOTORE, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 		// {#endif}  //Fine EFLEX_DH4010
	// 		// {#ifdef RUNNER}
	// 		// 	Set_disp_position(Ndisp:=RULLATORE, Value:=RIPOSO, TimerRisparmio:=TRUE);
	// 		// 	WriteDigitalOutput(NumOutput:=OUT_MOTORE_FORATURA, Value:=FALSE);
	// 		// 	CURRENT_STATE:=STATE_ARRESTA_MOTORI_TAGLIORTL;
	// 		// {#else}
	// 		// 	CURRENT_STATE:=GENERAL_STATE#STATE_END;
	// 		// {#endif};  //Fine RUNNER
			
	// 	STATE_ARRESTA_MOTORI_TAGLIORTL:
	// 		{#ifdef RUNNER}
	// 			IF (TaglioRTL.outputState<>GENERAL_STATE#STATE_END) THEN
	// 				TaglioRTL(Velocita:=0, Attesa:=Attesa);
	// 			ELSE
	// 				TaglioRTL(Velocita:=0, Attesa:=Attesa);
	// 				CURRENT_STATE:=GENERAL_STATE#STATE_END;
	// 			END_IF;
	// 		{#endif};
			
	// 	GENERAL_STATE#STATE_END:
	// 		CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	
	// END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

/**********************************************************************************************************
 *     ON 28/04/2014 ADDED CHAIN SYNCHRONISATION MANAGEMENT. WHEN ONE OF THE TWO LIMIT SWITCHES TO ON     *
 * WITH THE OTHER STILL AT OFF, ONLY THE CHAIN WITH THE SENSOR AT ON IS STOPPED. WHEN THE SENSOR THAT WAS *
 *    TO OFF ALSO SWITCHES TO ON, 0.1 IS ALLOWED TO ELAPSE WITH BOTH CHAINS STOPPED, AND THEN RESTARTS    *
 *                                              BOTH CHAINS                                               *
 **********************************************************************************************************/

// {#ifdef AL_BLM}
  FUNCTION_BLOCK FB_AL5Catene

    VAR_EXTERNAL
      TimerSistema                    :ARRAY[0..MAX_TIMER_SISTEMA-1] OF Timer;
    END_VAR

    VAR_EXTERNAL CONSTANT
      INP_AL5_SINCR_CATENA_SX                         :INT;
      T_SINCR_CATENE_AL5	                            :INT;
      MAX_TIMER_SISTEMA 	                            :USINT;
      INP_AL5_SINCR_CATENA_DX                         :INT;
    END_VAR

		VAR_INPUT
			Stato  :INT;
		END_VAR
		VAR
			dret  :DINT;
			
			CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
			AL5CatenaDX		:FB_AL5CatenaDX;
			AL5CatenaSX		:FB_AL5CatenaSX;
			// {#ifdef MS0017}
			// 	AL5Catena3		:FB_AL5Catena3;
			// {#endif}
		END_VAR  
		VAR_OUTPUT
			OutputState	:INT;
		END_VAR
		CASE CURRENT_STATE OF
				GENERAL_STATE#STATE_INIT:
					// {#ifdef MS0017} %%%%
					// 	CURRENT_STATE:=STATE_AL5_CATENE_MS0017_CATENA_DX;
					// {#else}
						IF Stato=DEVICES_STATUS#LAVORO THEN
							CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_LAVORO;
						ELSE
							CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_CATENA_DX_ALTRI_STATI;
						END_IF;
				// 	{#endif};
					
				// STATE_AL5_CATENE_MS0017_CATENA_DX:
				// 	{#ifdef MS0017}
				// 		IF (AL5CatenaDX.outputState<>GENERAL_STATE#STATE_END) THEN
				// 			AL5CatenaDX(Stato:=Stato);
				// 		ELSE
				// 			AL5CatenaDX(Stato:=Stato);
				// 			CURRENT_STATE:=STATE_AL5_CATENE_MS0017_CATENA_SX;
				// 		END_IF;
				// 	{#endif};
					
				// STATE_AL5_CATENE_MS0017_CATENA_SX:
				// 	{#ifdef MS0017}
				// 		IF (AL5CatenaSX.outputState<>GENERAL_STATE#STATE_END) THEN
				// 			AL5CatenaSX(Stato:=Stato);
				// 		ELSE
				// 			AL5CatenaSX(Stato:=Stato);
				// 			CURRENT_STATE:=STATE_AL5_CATENE_MS0017_CATENA_3;
				// 		END_IF;
				// 	{#endif};
					
				// STATE_AL5_CATENE_MS0017_CATENA_3:
				// 	{#ifdef MS0017}
				// 		IF (AL5Catena3.outputState<>GENERAL_STATE#STATE_END) THEN
				// 			AL5Catena3(Stato:=Stato);
				// 		ELSE
				// 			AL5Catena3(Stato:=Stato);
				// 			CURRENT_STATE:=GENERAL_STATE#STATE_END;
				// 		END_IF;
				// 	{#endif};
					
				_FB_AL5CATENE#STATE_AL5_CATENE_LAVORO:
					//Si sincronizza solo durante rotazione che porta il tubo verso le leve
					IF TestSincrCatene() THEN
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_CATENA_DX_SINCRONIZZATE;
					ELSE
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_NO_SINCRONIZZATE;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_CATENA_DX_SINCRONIZZATE:
					//Catene sincronizzate si comanda la rotazione
					IF (AL5CatenaDX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaDX(Stato:=DEVICES_STATUS#LAVORO);
					ELSE	
						AL5CatenaDX(Stato:=DEVICES_STATUS#LAVORO);
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_CATENA_SX_SINCRONIZZATE;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_CATENA_SX_SINCRONIZZATE:
					//Catene sincronizzate si comanda la rotazione
					IF (AL5CatenaSX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaSX(Stato:=DEVICES_STATUS#LAVORO);
					ELSE	
						AL5CatenaSX(Stato:=DEVICES_STATUS#LAVORO);
						IF ReadDigitalInput(NumInput:=INP_AL5_SINCR_CATENA_SX, Verifica:=FALSE)=TRUE THEN
							//Se i due sensori sono a ON, si fa partire il timer che blocca la verifica della
							//sincronizzazione delle catene, così da evitare che durante la disattivazione dei sensori
							//si ricada in una nuova sincronizzazione. Attenzione che il tempo del timer deve
							//garantire la disattivazione dei finecorsa, ma essere più breve del tempo di rotazione 
							//in modo che al successivo fronte di salita la sincronizzazione sia nuovamente abilitatata		
							TimerSistema[T_SINCR_CATENE_AL5].Value:=1500;
            	TimerSistema[T_SINCR_CATENE_AL5].Status:=SINT#1;
						END_IF;
						CURRENT_STATE:=GENERAL_STATE#STATE_END;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_NO_SINCRONIZZATE:
					IF ReadDigitalInput(NumInput:=INP_AL5_SINCR_CATENA_SX, Verifica:=FALSE)=FALSE THEN
            //Occorre fermare la catena DX e ruotare la SX fino a quando si attiva il sensore SX
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_DX_ARRESTO;
					ELSE
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_SX_ARRESTO;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_DX_ARRESTO:
					IF (AL5CatenaDX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaDX(Stato:=DEVICES_STATUS#ARRESTO);
					ELSE
						AL5CatenaDX(Stato:=DEVICES_STATUS#ARRESTO);
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_SX_LAVORO_ATTESA_SYNC;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_SX_LAVORO_ATTESA_SYNC:
					IF (AL5CatenaSX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaSX(Stato:=DEVICES_STATUS#LAVORO);
					ELSE
						AL5CatenaSX(Stato:=DEVICES_STATUS#LAVORO);
						IF ReadDigitalInput(NumInput:=INP_AL5_SINCR_CATENA_SX, Verifica:=FALSE)=TRUE THEN
							CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_SX_ARRESTO_SYNC;
						END_IF;
					END_IF;
				
        _FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_SX_ARRESTO_SYNC:
					IF (AL5CatenaSX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaSX(Stato:=DEVICES_STATUS#ARRESTO);
					ELSE
						AL5CatenaSX(Stato:=DEVICES_STATUS#ARRESTO);
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_MANTIENI_CATENE_FERME;
					END_IF;
				
        _FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_SX_ARRESTO:
					IF (AL5CatenaSX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaSX(Stato:=DEVICES_STATUS#ARRESTO);
					ELSE
						AL5CatenaSX(Stato:=DEVICES_STATUS#ARRESTO);
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_DX_LAVORO_ATTESA_SYNC;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_DX_LAVORO_ATTESA_SYNC:
					IF (AL5CatenaDX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaDX(Stato:=DEVICES_STATUS#LAVORO);
					ELSE
						AL5CatenaDX(Stato:=DEVICES_STATUS#LAVORO);
						IF ReadDigitalInput(NumInput:=INP_AL5_SINCR_CATENA_DX, Verifica:=FALSE)=TRUE THEN
							CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_DX_ARRESTO_SYNC;
						END_IF;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_NO_SYNC_CATENA_DX_ARRESTO_SYNC:
					IF (AL5CatenaDX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaDX(Stato:=DEVICES_STATUS#ARRESTO);
					ELSE
						AL5CatenaDX(Stato:=DEVICES_STATUS#ARRESTO);
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_MANTIENI_CATENE_FERME;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_MANTIENI_CATENE_FERME:
          //Mantenimento catene ferme, in pratica migliora la precisione della sincronizzazione
          //dret:=_waitTime(T#100ms);
					IF (AL5CatenaDX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaDX(Stato:=DEVICES_STATUS#LAVORO);
					ELSE
						AL5CatenaDX(Stato:=DEVICES_STATUS#LAVORO);
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_CATENA_SX_LAVORO_POST_SYNC;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_CATENA_SX_LAVORO_POST_SYNC:
					IF (AL5CatenaSX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaSX(Stato:=DEVICES_STATUS#LAVORO);
					ELSE
						AL5CatenaSX(Stato:=DEVICES_STATUS#LAVORO);
						//Avvio del timer per bloccare la verifica di sincronizzazione durante la disattivazione
						//dei finecorsa
						TimerSistema[T_SINCR_CATENE_AL5].Value:=1000;
						TimerSistema[T_SINCR_CATENE_AL5].Status:=SINT#1;
						CURRENT_STATE:=GENERAL_STATE#STATE_END;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_CATENA_DX_ALTRI_STATI:
					IF (AL5CatenaDX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaDX(Stato:=Stato);
					ELSE
						AL5CatenaDX(Stato:=Stato);
						CURRENT_STATE:=_FB_AL5CATENE#STATE_AL5_CATENE_CATENA_SX_ALTRI_STATI;
					END_IF;
					
        _FB_AL5CATENE#STATE_AL5_CATENE_CATENA_SX_ALTRI_STATI:
					IF (AL5CatenaSX.OutputState<>GENERAL_STATE#STATE_END) THEN
						AL5CatenaSX(Stato:=Stato);
					ELSE
						AL5CatenaSX(Stato:=Stato);
						CURRENT_STATE:=GENERAL_STATE#STATE_END;
					END_IF;
					
				GENERAL_STATE#STATE_END:
					CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
				
		END_CASE;
		OutputState:=CURRENT_STATE;
	END_FUNCTION_BLOCK
// {#endif}  //Fine AL_BLM

/********************************************************
 * MANAGES THE JOG OF A REAL AXIS WITH ABSOLUTE ENCODER *
 ********************************************************/

FUNCTION_BLOCK FB_Homing_absolute

  VAR_EXTERNAL
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    cfg                             :_TYPE.Config_Struct;
    i_sys_AutoSetting					      :INT;
    i_sys_OPMODE                    : INT;
    i_sys_activedevice              : INT;
    Start_read								      : BOOL;
    mcHome									        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF MC_Home;
    UsoPLCOpen						          :BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    INACTIVE_DEACTIVATABLE_AXIS           :SINT;
    X1 :INT;
    REFMODE                               :INT;
    UNDEF_DEVICE                       	  :INT;
    SALVA_DATI_SIMOTION                   :SINT;
    driveObjectIdConstant                 : UINT;
        hardwareIdConstant                :WORD;
  END_VAR

  VAR_INPUT
    Asse      :INT;  
    Position    :LREAL;
  END_VAR  
  
  VAR
    PosAx     : REF_TO TO_SynchronousAxis;
    iret      :INT;
    dwret     :DWORD;
    dret      :DINT;
		
		CURRENT_STATE				:INT:=GENERAL_STATE#STATE_INIT;
		LetturaSinamicsREAL	: _READ_PARAMETER_SINGLE;
		ResetAxisBLM				:FB_ResetAxisBLM;
    Homing              : Homing;
  END_VAR 

	VAR_OUTPUT
		OutputState	:INT;
		ReturnValue	:DINT;
	END_VAR

	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF pm[Asse].ma_stato_index=0 THEN
        PosAx:= AsSynchronousAxisRef(Axes[Asse].PosAx);
				IF pm[Asse].ma_asse_installato=INACTIVE_DEACTIVATABLE_AXIS AND PosAx^.Simulation.Mode=UDINT#1 THEN
					//Per gli assi in simulazione e disattivati da VGP3D, quindi con attivo il secondo encoder
					//ANALOG per fare in modo di non perdere la quota di index, occorre eliminare la simulazione
					//prima di fare l^index e poi ripristinarla
					IF (ResetAxisBLM.OutputState<>GENERAL_STATE#STATE_END) THEN
						ResetAxisBLM(Asse, TRUE);
					ELSE
						ResetAxisBLM(Asse, TRUE);
						CURRENT_STATE:=_FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_HOME_SIMULATED_AXIS;
					END_IF;
				ELSE
					CURRENT_STATE:=_FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_HOME_ACTIVATED_AXIS;
				END_IF;
			ELSE
				ReturnValue:=0;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
    _FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_HOME_SIMULATED_AXIS:
			Homing(Asse:=Asse, Quota:=Position, Modo:=EnumHomingMode#SET_OFFSET_OF_ABSOLUTE_ENCODER_BY_POSITION);
			// dret:=_enableAxisSimulation(axis:=Axes[Asse].PosAx, %%%%%
			// 														nextCommand:=IMMEDIATELY, 
			// 														commandId:=_getCommandId());
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
    _FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_HOME_ACTIVATED_AXIS:
			Homing(Asse:=Asse, Quota:=Position, Modo:=EnumHomingMode#SET_OFFSET_OF_ABSOLUTE_ENCODER_BY_POSITION);
			IF (UsoPLCOpen) THEN
				CURRENT_STATE:=_FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_WAIT_PLCOPEN_RESET;
			ELSE
				CURRENT_STATE:=_FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_READ_NUMERO_SERIE_ENCODER;
			END_IF;
			
    _FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_WAIT_PLCOPEN_RESET:
			IF (mcHome[Asse].done=TRUE) THEN
				CURRENT_STATE:=_FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_READ_NUMERO_SERIE_ENCODER;
			END_IF;
			
    _FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_READ_NUMERO_SERIE_ENCODER:
      posax:= AsSynchronousAxisRef(Axes[Asse].posax);
			IF posax^.VirtualAxis.Mode = UDINT#0 AND i_sys_OPMODE<>PI_COMMANDS_CODE#AUTOMODE AND Axes[Asse].IsSiemensAxis THEN

        IF NOT LetturaSinamicsREAL.Done THEN
          LetturaSinamicsREAL	(Start := Start_read,
                              driveObjectId      := driveObjectIdConstant,
                              hardwareId          := hardwareIdConstant,
                              parameterNumber     :=uint#463,
                              index              := uint#0);
        
				// IF (LetturaSinamicsREAL.outputState<>GENERAL_STATE#STATE_END) THEN
				// 	LetturaSinamicsREAL(log_address:=TO_DINT(Axes[Asse].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn),
				// 											par_number:=463,
				// 											sub_index:=0,
				// 											Verifica:=TRUE);
				ELSE
					Axes[Asse].NumSerieEncoder:=LetturaSinamicsREAL.Value_REAL;
                                LetturaSinamicsREAL	(Start := Start_read,
                                driveObjectId      := driveObjectIdConstant,
                                hardwareId          := hardwareIdConstant,
                                parameterNumber     :=uint#463,
                                index              := uint#0);
					// LetturaSinamicsREAL(log_address:=UDINT_TO_DINT(Assi[Asse].PosAx.setconfigdata.TypeOfAxis.SetPointDriverInfo.logAddressIn),
					// 										par_number:=463,
					// 										sub_index:=0,
					// 										Verifica:=TRUE);
          LetturaSinamicsREAL.Start :=false;
					CURRENT_STATE:=_FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_SET_VALUES;
				END_IF;
			ELSE
				CURRENT_STATE:=_FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_SET_VALUES;
			END_IF;
			
    _FB_Homing_Absolute#STATE_HOMING_ABSOLUTE_SET_VALUES:
			pm[Asse].MA_NUM_SERIE_ENCODER:=Axes[Asse].NumSerieEncoder;
      posax:= AsSynchronousAxisRef(Axes[Asse].posax);
			Axes[Asse].Homed:=PosAx^.StatusWord.%X5;
      PosAx^.PositionLimits_SW.Active:= TRUE;
			//Axes[Asse].PosAx.swlimit.state:=ACTIVE; ///???
			IF Asse=X1 THEN
				ImpostazioneOffsetX1(Offset:=cfg.OffsetMultiCannottoX1);
			END_IF;
			pm[Asse].ma_stato_index:=SINT#2;
			i_sys_AutoSetting:=SALVA_DATI_SIMOTION;
			(*dwret:=_startTaskId(_task.MotionTask_CambioConHard);*)
			IF i_sys_OPMODE=REFMODE THEN
				i_sys_activedevice:=UNDEF_DEVICE;
			END_IF;
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;

	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION SistemaUtensili
  VAR 
    Tasto1    :INT;
    Tasto2    :INT;
    Tasto3    :INT;
    Tasto4    :INT;
    i         :INT;
    Esito     :BOOL;
    Testa     :INT;
    EsitoDX,
    EsitoSX   :BOOL;
  END_VAR
  // {
  // #ifdef EFLEX_DH4010 %%%%
  // }
  //   BlmMxToolsTestine_1_5:=BlmMxToolsTestine_1_5_shadow;
  //   BlmMxToolsTestine_2_6:=BlmMxToolsTestine_2_6_shadow;
  // {
  // #else    //Fine EFLEX_DH4010
  // }
  //   BlmMxToolsOffPezzo:=BlmMxToolsOffPezzo_shadow;
  //   BlmMxCoppiaW3:=BlmMxCoppiaW3_shadow;
  //   BlmMxCoppiaW4:=BlmMxCoppiaW4_shadow;
  //   BlmMxCoppiaW5:=BlmMxCoppiaW5_shadow;
  //   BlmMxCoppiaW6:=BlmMxCoppiaW6_shadow;
  //   i_sys_Max_Ut_Macchina:=MaxTools_DX_Possibili+MaxTools_SX_Possibili;
	// 	BlmMxToolsDx:=BlmMxToolsDx_shadow;
	// 	BlmMxToolsSx:=BlmMxToolsSx_shadow;
  // {
  // #endif    //Fine NO EFLEX_DH4010
  // }
  // {
  // #ifdef SMART_SMART_A %%%%
  // }
  //   //Per poter eseguire il cambio raggio da una situazione di utensili tutti da una lato
  //   //e partenza della testa dal lato opposto, occore che anche la tabella relativa al lato
  //   //dove non ci sono utensili debba essere compilata con però la colonna QQt a 0
  //   IF MaxTools_DX_Installati=0 THEN
  //     //Tutti gli utensili installati sono sul lato SX ,si copia la tabella SX sulla DX
  //     //azzerando la colonna del raggio medio
  //     BLMMxToolsDX:=BLMMxToolsSX;
  //     FOR i:=1 TO MAXTOOLS*2 DO
  //       BLMMxToolsDX[i].qqt:=0;
  //     END_FOR;
  //   ELSE
  //     IF MaxTools_SX_Installati=0 THEN
  //       //Tutti gli utensili installati sono sul lato DX, si copia la tabella DX sulla SX
  //       //azzerando la colonna del raggio medio
  //       BLMMxToolsSX:=BLMMxToolsDX;
  //       FOR i:=1 TO MAXTOOLS*2 DO
  //         BLMMxToolsSX[i].qqt:=0;
  //       END_FOR;
  //     END_IF;
  //   END_IF;
  //   IF MaxTools_DX_Possibili=i_sys_Max_Ut_Macchina/2 THEN
  //     //Smart standard, metà utensili DX e metà utensili SX
  //     IF BlmMxToolsDx[1].Qqt>0.1 THEN
  //       Tasto1:=C_R_POSIZIONE_1;  
  //     ELSE
  //       Tasto1:=C_R_POSIZIONE_1+MaxTools_DX_Possibili;
  //     END_IF;
  //     IF BlmMxToolsDx[2].Qqt>0.1 THEN
  //       Tasto2:=C_R_POSIZIONE_2; 
  //     ELSE
  //       Tasto2:=C_R_POSIZIONE_2+MaxTools_DX_Possibili; 
  //     END_IF;
  //     IF BlmMxToolsDx[3].Qqt>0.1 THEN
  //       Tasto3:=C_R_POSIZIONE_3; 
  //     ELSE
  //       Tasto3:=C_R_POSIZIONE_3+MaxTools_DX_Possibili;  
  //     END_IF;
  //     IF BlmMxToolsDx[4].Qqt>0.1 THEN
  //       Tasto4:=C_R_POSIZIONE_4; 
  //     ELSE
  //       Tasto4:=C_R_POSIZIONE_4+MaxTools_DX_Possibili;  
  //     END_IF;
  //   ELSE
  //     //Smart con solo utensili DX o SX
  //     Tasto1:=C_R_POSIZIONE_1;  
  //     Tasto2:=C_R_POSIZIONE_2; 
  //     Tasto3:=C_R_POSIZIONE_3; 
  //     Tasto4:=C_R_POSIZIONE_4; 
  //   END_IF;
  //   InizializzaKeyDisp(Tasto9:=Tasto1,
  //                      Tasto10:=Tasto2,
  //                      Tasto11:=Tasto3,
  //                      Tasto14:=Tasto4,
  //                      Tasto15:=-1,
  //                      Tasto16:=-1,
  //                      Tasto17:=-1,
  //                      Tasto18:=-1);
  // {
  // #endif	//Fine SMART_SMART_A 
  // }
    // {
    // #ifdef ELECTMRL %%%%
    // }
		// 	IF BlmMxToolsDx[1].Qqt>0.1 THEN
		// 		Tasto1:=C_R_POSIZIONE_1;  
		// 	ELSE
		// 		Tasto1:=C_R_POSIZIONE_1+MaxTools_DX_Possibili;
		// 	END_IF;
		// 	IF BlmMxToolsDx[2].Qqt>0.1 THEN
		// 		Tasto2:=C_R_POSIZIONE_2; 
		// 	ELSE
		// 		Tasto2:=C_R_POSIZIONE_2+MaxTools_DX_Possibili; 
		// 	END_IF;
		// 	IF BlmMxToolsDx[3].Qqt>0.1 THEN
		// 		Tasto3:=C_R_POSIZIONE_3; 
		// 	ELSE
		// 		Tasto3:=C_R_POSIZIONE_3+MaxTools_DX_Possibili;  
		// 	END_IF;
		// 	IF BlmMxToolsDx[4].Qqt>0.1 THEN
		// 		Tasto4:=C_R_POSIZIONE_4; 
		// 	ELSE
		// 		Tasto4:=C_R_POSIZIONE_4+MaxTools_DX_Possibili;  
		// 	END_IF;
		// 	InizializzaKeyDisp(Tasto9:=Tasto1,
		// 										 Tasto10:=Tasto2,
		// 										 Tasto11:=Tasto3,
		// 										 Tasto14:=Tasto4,
		// 										 Tasto15:=-1,
		// 										 Tasto16:=-1,
		// 										 Tasto17:=-1,
		// 										 Tasto18:=-1);
    // {
    // #endif //FINE ELECTMRL
    // }
  // {
  // #ifdef SMART_A %%%%
  // }
  //   //Imposta la zona dell^asse W4 all^interno della quale il sostegno tirante non
  //   //si può muovere
  //   ImpostaLimitiW4SostegnoTirante();
  // {
  // #endif	//Fine SMART_A 
  // }
  //  {
  // #ifdef ETURN
  // }
  //   EsitoDX:=FALSE;
  //   EsitoSX:=FALSE;
  //   FOR i:=1 TO MAXTOOLS*2 DO
  //     IF BlmMxToolsDx[i].qqt>0.1 THEN
  //       Testa:=DX;
  //       EsitoDX:=TRUE;
  //       //Calcolo coordinate teoriche per P3 DX
  //       Esito:=ConvRealiTeoriciEturn(PosRealeQ:=BlmMxToolsDx[i].P3qq,
  //                                    PosRealeT:=BlmMxToolsDx[i].p3qt,
  //                                    LatoTesta:=Testa,
  //                                    PosTeoricaQ:=BlmMxToolsDx[i].p3qqt,
  //                                    PosTeoricaT:=BlmMxToolsDx[i].p3qtt);
  //       //Impostazione punto P4 per passaggio da Dx a Sx
  //       BlmMxToolsDx[i].p4qt:=BlmMxToolsDx[i].p3qt;
  //       BlmMxToolsDx[i].p4qq:=80;
  //       Esito:=ConvRealiTeoriciEturn(PosRealeQ:=BlmMxToolsDx[i].P4qq,
  //                                    PosRealeT:=BlmMxToolsDx[i].p4qt,
  //                                    LatoTesta:=Testa,
  //                                    PosTeoricaQ:=BlmMxToolsDx[i].p4qqt,
  //                                    PosTeoricaT:=BlmMxToolsDx[i].p4qtt);
  //       //Impostazione punto P5 (0, 90) per passaggio intermedio da Dx a Sx
  //       BlmMxToolsDx[i].p5qt:=0;
  //       BlmMxToolsDx[i].p5qq:=90;
  //       Esito:=ConvRealiTeoriciEturn(PosRealeQ:=BlmMxToolsDx[i].P5qq,
  //                                    PosRealeT:=BlmMxToolsDx[i].p5qt,
  //                                    LatoTesta:=Testa,
  //                                    PosTeoricaQ:=BlmMxToolsDx[i].p5qqt,
  //                                    PosTeoricaT:=BlmMxToolsDx[i].p5qtt);
  //     END_IF;
  //     IF BlmMxToolsSx[i].qqt>0.1 THEN
  //       Testa:=SX;
  //       EsitoSX:=TRUE;
  //       //Calcolo coordinate teoriche per P3 SX
  //       Esito:=ConvRealiTeoriciEturn(PosRealeQ:=BlmMxToolsSx[i].P3qq,
  //                                    PosRealeT:=BlmMxToolsSx[i].p3qt,
  //                                    LatoTesta:=Testa,
  //                                    PosTeoricaQ:=BlmMxToolsSx[i].p3qqt,
  //                                    PosTeoricaT:=BlmMxToolsSx[i].p3qtt);
  //       //Impostazione punto P4 per passaggio da Dx a Sx
  //       BlmMxToolsSx[i].p4qt:=BlmMxToolsSx[i].p3qt;
  //       BlmMxToolsSx[i].p4qq:=100;
  //       Esito:=ConvRealiTeoriciEturn(PosRealeQ:=BlmMxToolsSx[i].P4qq,
  //                                    PosRealeT:=BlmMxToolsSx[i].p4qt,
  //                                    LatoTesta:=Testa,
  //                                    PosTeoricaQ:=BlmMxToolsSx[i].p4qqt,
  //                                    PosTeoricaT:=BlmMxToolsSx[i].p4qtt);    
  //       //Impostazione punto P5 (0, 90) per passaggio intermedio da Sx a Dx
  //       BlmMxToolsSx[i].p5qt:=0;
  //       BlmMxToolsSx[i].p5qq:=90;
  //       Esito:=ConvRealiTeoriciEturn(PosRealeQ:=BlmMxToolsSx[i].P5qq,
  //                                    PosRealeT:=BlmMxToolsSx[i].p5qt,
  //                                    LatoTesta:=Testa,
  //                                    PosTeoricaQ:=BlmMxToolsSx[i].p5qqt,
  //                                    PosTeoricaT:=BlmMxToolsSx[i].p5qtt);    
  //     END_IF;
  //   END_FOR;
	// 	CalcolaUltimoUtPila();
  //   IF EsitoDX=TRUE AND EsitoSX=FALSE THEN
  //     //Significa che non è dichiarato nessun utensile sul lato SX, ma per poter fare il
  //     //rientro fuori ingombri mi servono le informazioni anche del lato SX, così imposto
  //     //un utensile fittizzio SX uguale a quello DX
  //     BlmMxToolsSX[1].qqt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].qqt;
  //     BlmMxToolsSX[1].qtt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].qtt;
  //     BlmMxToolsSX[1].qq:=180-BlmMxToolsDX[i_sys_ultimo_ut_DX].qq;
  //     BlmMxToolsSX[1].qt:=-BlmMxToolsDX[i_sys_ultimo_ut_DX].qt;
  //     BlmMxToolsSX[1].p2qqt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].p2qqt;
  //     BlmMxToolsSX[1].p2qtt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].p2qtt;
  //     BlmMxToolsSX[1].p2qq:=180-BlmMxToolsDX[i_sys_ultimo_ut_DX].p2qq;
  //     BlmMxToolsSX[1].p2qt:=-BlmMxToolsDX[i_sys_ultimo_ut_DX].p2qt;
  //     BlmMxToolsSX[1].p3qq:=180-BlmMxToolsDX[i_sys_ultimo_ut_DX].p3qq;
  //     BlmMxToolsSX[1].p3qt:=-BlmMxToolsDX[i_sys_ultimo_ut_DX].p3qt;
  //     BlmMxToolsSX[1].p3qqt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].p3qqt;
  //     BlmMxToolsSX[1].p3qtt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].p3qtt;
  //     BlmMxToolsSX[1].p4qq:=180-BlmMxToolsDX[i_sys_ultimo_ut_DX].p4qq;
  //     BlmMxToolsSX[1].p4qt:=-BlmMxToolsDX[i_sys_ultimo_ut_DX].p4qt;
  //     BlmMxToolsSX[1].p4qqt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].p4qqt;
  //     BlmMxToolsSX[1].p4qtt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].p4qtt;
  //     BlmMxToolsSX[1].p5qq:=180-BlmMxToolsDX[i_sys_ultimo_ut_DX].p5qq;
  //     BlmMxToolsSX[1].p5qt:=-BlmMxToolsDX[i_sys_ultimo_ut_DX].p5qt;
  //     BlmMxToolsSX[1].p5qqt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].p5qqt;
  //     BlmMxToolsSX[1].p5qtt:=BlmMxToolsDX[i_sys_ultimo_ut_DX].p5qtt;
	// 		BlmMxToolsSx_shadow:=BlmMxToolsSx;
	// 		//Ricalcolo ultimo utensile pila sinistra doopo aver sistemato la BlmMxToolsSX
	// 		CalcolaUltimoUtPila();
  //   END_IF;
  //   IF EsitoSX=TRUE AND EsitoDX=FALSE THEN
  //     //Significa che non è dichiarato nessun utensile sul lato DX, ma per poter fare il
  //     //rientro fuori ingombri mi servono le informazioni anche del lato DX, così imposto
  //     //un utensile fittizzio DX uguale a quello SX
  //     BlmMxToolsDX[1].qqt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].qqt;
  //     BlmMxToolsDX[1].qtt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].qtt;
  //     BlmMxToolsDX[1].qq:=180-BlmMxToolsSX[i_sys_ultimo_ut_SX].qq;
  //     BlmMxToolsDX[1].qt:=-BlmMxToolsSX[i_sys_ultimo_ut_SX].qt;
  //     BlmMxToolsDX[1].p2qqt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].p2qqt;
  //     BlmMxToolsDX[1].p2qtt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].p2qtt;
  //     BlmMxToolsDX[1].p2qq:=180-BlmMxToolsSX[i_sys_ultimo_ut_SX].p2qq;
  //     BlmMxToolsDX[1].p2qt:=-BlmMxToolsSX[i_sys_ultimo_ut_SX].p2qt;
  //     BlmMxToolsDX[1].p3qq:=180-BlmMxToolsSX[i_sys_ultimo_ut_SX].p3qq;
  //     BlmMxToolsDX[1].p3qt:=-BlmMxToolsSX[i_sys_ultimo_ut_SX].p3qt;
  //     BlmMxToolsDX[1].p3qqt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].p3qqt;
  //     BlmMxToolsDX[1].p3qtt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].p3qtt;
  //     BlmMxToolsDX[1].p4qq:=180-BlmMxToolsSX[i_sys_ultimo_ut_SX].p4qq;
  //     BlmMxToolsDX[1].p4qt:=-BlmMxToolsSX[i_sys_ultimo_ut_SX].p4qt;
  //     BlmMxToolsDX[1].p4qqt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].p4qqt;
  //     BlmMxToolsDX[1].p4qtt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].p4qtt;
  //     BlmMxToolsDX[1].p5qq:=180-BlmMxToolsSX[i_sys_ultimo_ut_SX].p5qq;
  //     BlmMxToolsDX[1].p5qt:=-BlmMxToolsSX[i_sys_ultimo_ut_SX].p5qt;
  //     BlmMxToolsDX[1].p5qqt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].p5qqt;
  //     BlmMxToolsDX[1].p5qtt:=BlmMxToolsSX[i_sys_ultimo_ut_SX].p5qtt;
	// 		BlmMxToolsDx_shadow:=BlmMxToolsDx;
	// 		CalcolaUltimoUtPila();
	// 		//Ricalcolo ultimo utensile pila destra doopo aver sistemato la BlmMxToolsDX
  //   END_IF;
	// 	//Copia anche nella shadow i dati calcolati perchè servono nella gestione delle piattine													 
	// 	BlmMxToolsDx_shadow:=BlmMxToolsDx;
	// 	BlmMxToolsSx_shadow:=BlmMxToolsSx;
  // {
  // #endif    //Fine ETURN
  // }
  ;
END_FUNCTION

/*********************************************
 * RESETS THE REQUIRED STATUS OF ALL DEVICES *
 *********************************************/

FUNCTION ResetStatoRichiesto

  VAR_EXTERNAL
    Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
  END_VAR

  VAR_EXTERNAL CONSTANT
    FIRST_DEVICE                    :INT;
  END_VAR

  VAR
    i  :INT;
  END_VAR
  FOR i:=FIRST_DEVICE TO FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1 DO
    disp[i].StatoRichiesto:=-1;
  END_FOR;
END_FUNCTION

FUNCTION ResetOutCaricoScaricoRobot

  VAR_EXTERNAL CONSTANT
    OUT_CARICO_MACCHINA_PRONTA                    :INT;
    OUT_CARICO_MACCHINA_PRONTA_DPCOUPLER          :INT;
    OUT_FCY                                       :INT;
    OUT_CARICO_MACCHINA_TUBO_PRESO                :INT;
    OUT_CARICO_MACCHINA_TUBO_PRESO_DPCOUPLER      :INT;
    OUT_SCARICO_MACCHINA_PRONTA                   :INT;
    OUT_SCARICO_MACCHINA_PRONTA_DPCOUPLER         :INT;
    OUT_SCARICO_MACCHINA_TUBO_LIBERO			        :INT;
    OUT_SCARICO_MACCHINA_TUBO_LIBERO_DPCOUPLER      :INT;
    OUT_CODIFICA_PROGRAMMA_OK_DPCOUPLER             :INT;
  END_VAR
  // {
  // #ifdef CARICO_AUTOMATICO
  // }
    WriteDigitalOutput(NumOutput:=OUT_CARICO_MACCHINA_PRONTA, Value:=FALSE, Verifica:=FALSE);
    WriteDigitalOutput(NumOutput:=OUT_CARICO_MACCHINA_PRONTA_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
    WriteDigitalOutput(NumOutput:=OUT_FCY, Value:=FALSE, Verifica:=FALSE);
    WriteDigitalOutput(NumOutput:=OUT_CARICO_MACCHINA_TUBO_PRESO, Value:=FALSE, Verifica:=FALSE);
    WriteDigitalOutput(NumOutput:=OUT_CARICO_MACCHINA_TUBO_PRESO_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
    // {
    // #ifndef DH4010 %%%%
    // }
    //   WriteDigitalOutput(NumOutput:=OUT_CARICO_MACCHINA_X1_FI, Value:=FALSE, Verifica:=FALSE);
    //   WriteDigitalOutput(NumOutput:=OUT_CARICO_MACCHINA_X1_FI_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
    //   WriteDigitalOutput(NumOutput:=OUT_CARICO_TUBO_PRESENTE_AL, Value:=FALSE, Verifica:=FALSE);
    //   WriteDigitalOutput(NumOutput:=OUT_CARICO_TUBO_PRESENTE_AL_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
    //   WriteDigitalOutput(NumOutput:=OUT_CARICO_TUBO_LIBERO_AL, Value:=FALSE, Verifica:=FALSE);
    //   WriteDigitalOutput(NumOutput:=OUT_CARICO_TUBO_LIBERO_AL_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
    //   WriteDigitalOutput(NumOutput:=OUT_CARICO_TUBO_KO_AL_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
		// {
    // #endif  //Fine DH4010
    // }
  // {
  // #endif  //Fine CARICO_AUTOMATICO
  // }
  WriteDigitalOutput(NumOutput:=OUT_SCARICO_MACCHINA_PRONTA, Value:=FALSE, Verifica:=FALSE);
  WriteDigitalOutput(NumOutput:=OUT_SCARICO_MACCHINA_PRONTA_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
  WriteDigitalOutput(NumOutput:=OUT_SCARICO_MACCHINA_TUBO_LIBERO, Value:=FALSE, Verifica:=FALSE);
  WriteDigitalOutput(NumOutput:=OUT_SCARICO_MACCHINA_TUBO_LIBERO_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
  WriteDigitalOutput(NumOutput:=OUT_CODIFICA_PROGRAMMA_OK_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
	ResetCodificaPezzo();
END_FUNCTION

FUNCTION OutScartoLavorazione
  VAR_INPUT
    Stato  :BOOL;
  END_VAR
  // {
  // #ifndef ORBITALE %%%%
  // }
  //   IF cfg.TipoTaglioColtello=TAGLIO_COLTELLO_PRIMA_VER THEN
  //     //Prima versione taglio a coltello, la temporizzazione avviene mediante un timer elettromeccanico
  //     WriteDigitalOutput(NumOutput:=OUT_ESPULSIONE_SCARTO_LAVORAZIONE, Value:=Stato);
  //   ELSE
  //     //Seconda versione taglio a coltello, la temporizzazione è software
  //     IF Stato=TRUE THEN
  //       IF TimerSistema[T_SCARTO_LAVORAZ].Stato=0 THEN
  //         TimerSistema[T_SCARTO_LAVORAZ].Value:=(TempoSoffioSfridiTaglioColtello*1000)+VALORE_FINE_TIMER;
  //         TimerSistema[T_SCARTO_LAVORAZ].Stato:=1;
  //       END_IF;
  //     END_IF;
  //   END_IF;    
  // {
  // #endif  //Fine NO ORBITALE
  // }
  ;
END_FUNCTION

FUNCTION MacchinaInWarning

  VAR_EXTERNAL
    i_sys_DpCoupler                 :BOOL;
    i_sys_PnCoupler                 :BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    MAXDIGITALOUTPUT                    :INT;
    OUT_MACCHINA_IN_WARNING_DPCOUPLER               :INT;
  END_VAR

  VAR_INPUT
    Stato  :BOOL;
  END_VAR

  IF i_sys_DpCoupler=TRUE OR i_sys_PnCoupler=TRUE THEN
		WriteDigitalOutput(NumOutput:=OUT_MACCHINA_IN_WARNING_DPCOUPLER, Value:=Stato);
	ELSE
		WriteDigitalOutput(NumOutput:=OUT_MACCHINA_IN_WARNING_DPCOUPLER, Value:=FALSE);
  END_IF;
END_FUNCTION

/******************************************************************
 * RESET STRUCTURE STATUS_OUT WHERE I STORE DIGITAL OUTPUT STATUS *
 ******************************************************************/
FUNCTION Res_StatoOut

  VAR_EXTERNAL
    Stato_Out                                 :ARRAY[0..MAXDIGITALOUTPUT] OF Struttura_Stato_Out;
  END_VAR

  VAR_EXTERNAL CONSTANT
    MAXDIGITALOUTPUT                    :INT;
  END_VAR
  VAR
    i:DINT;   
  END_VAR
  FOR i:=0 TO MAXDIGITALOUTPUT DO
    Stato_Out[i].Stato:=BOOL#0;     
  END_FOR;   
END_FUNCTION

FUNCTION ImpostaOutWeld
  VAR_INPUT
    Stato  :BOOL;
  END_VAR
  // {
  // #ifndef ORBITALE
  // }
  //   WriteDigitalOutput(NumOutput:=OUT_WELD, Value:=Stato);
  // {
  // #endif  //Fine NO ORBITALE
  // }
  ;
END_FUNCTION

FUNCTION ImpostaOutMisuraAnima
  VAR_INPUT
    Stato  :BOOL;
  END_VAR
  // {
  // #ifndef ORBITALE
  // }
  //   WriteDigitalOutput(NumOutput:=OUT_MISURA_ANIMA_KO, Value:=Stato);
  // {
  // #endif  //Fine NO ORBITALE
  // }
  ;
END_FUNCTION

FUNCTION Set_Disp_Position   


  VAR_EXTERNAL 
      Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
  END_VAR

    VAR_EXTERNAL CONSTANT
      FIRST_DEVICE                    :INT;
      DISP_IN_V_C                     :SINT;
      DISP_IN_V                       :SINT;
      RIPOSO                          :SINT;
  END_VAR

  VAR_INPUT
    Ndisp						:INT; 
    Value						:LREAL;
    TimerRisparmio	:BOOL;
  END_VAR
  VAR
    Valore          :REAL;
  END_VAR

  IF Disp[NDisp].enable THEN
    Disp[NDisp].statorichiesto:=TO_REAL(Value);
    CASE Disp[Ndisp].tipoin OF
      DISP_IN_V:
        Set_Disp(Ndisp:=Ndisp, Stato:=TO_INT(Value), TimerRisparmio:=TimerRisparmio);
      DISP_IN_V_C:
        Valore:=TO_REAL(Value);
        // IF _getStateOfTaskId(_task.motiontask_AUTO)<>TASK_STATE_STOPPED THEN
        //   SetDispComplesso(NDisp:=NDisp, Value:=Valore, Attesa:=TRUE, TimerRisparmio:=TimerRisparmio);
        // ELSE
          //Se non è attivo AUTOMODE non si esegue attesa, altrimenti si 
          //superano i tempi del background
          SetDispComplesso(NDisp:=NDisp, Value:=Valore, Attesa:=FALSE, TimerRisparmio:=TimerRisparmio);
        // END_IF;
    ELSE
      //Dispositivo reale si accettano due stati possibili LAVORO o RIPOSO
      IF Value>(DISP[Ndisp].Limite_Lavoro-DISP[Ndisp].Limite_Riposo)/2 THEN
        Set_Disp(Ndisp:=Ndisp, Stato:=DEVICES_STATUS#LAVORO, TimerRisparmio:=TimerRisparmio);
      ELSE   
        Set_Disp(Ndisp:=Ndisp, Stato:=RIPOSO, TimerRisparmio:=TimerRisparmio);
      END_IF;
    END_CASE;
  END_IF;
END_FUNCTION

FUNCTION ComandaRadar

    VAR_EXTERNAL CONSTANT
      AUTOMODE                              :INT;
      MIN_ZONA_RADAR_VGP                    :SINT;
    END_VAR

    VAR_EXTERNAL
      Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    END_VAR


  VAR_INPUT
		Programma			:DINT;
		ModoOperativo	:INT;
  END_VAR
  // {
  // #ifndef PROFINET
  // }
    IF ModoOperativo=AUTOMODE THEN
      IF Programmi[Programma].UsoRadar_Pezzo>MIN_ZONA_RADAR_VGP THEN
        AttivaProgrammaRadar(Codice:=Programmi[Programma].UsoRadar_Pezzo);
      END_IF;
    ELSE
      IF Programmi[Programma].usoradar_manuale>MIN_ZONA_RADAR_VGP THEN
        AttivaProgrammaRadar(Codice:=Programmi[Programma].usoradar_manuale);
      END_IF;
    END_IF;
  // {
  // #endif  //Fine NO PROFINET
  // }
  ;
END_FUNCTION  

/**************************************************
 * SET THE OUTPUT THAT ACTIVATES THE CARM7 PUMPER *
 **************************************************/

FUNCTION PompatoreCARM7
  VAR_INPUT
    Stato  :BOOL;
  END_VAR
  ;
  // {
  // #ifdef SMART %%%%
  // }
  //   IF disp[CLAMP_CARGE].enable=TRUE THEN
  //     WriteDigitalOutput(NumOutput:=OUT_CARICO_RIC_CAR, Value:=Stato);
  //   END_IF;  
  // {
  // #endif    //Fine SMART
  // }
END_FUNCTION

/*********************************************************************************
 * THE FUNCTION TAKES CARE OF SETTING THE JOB STATUS FOR PROTUBE AND WEBAPP DATA *
 *          THE STATUS SUSPENDED CAN^ ONLY BE SET FOLLOWING A TRANSFER           *
 *                             OF A NEW JOBID BY VGP                             *
 *********************************************************************************/

FUNCTION SetJobState

  VAR_EXTERNAL
    ProTube                         :Produzione;
    i_sys_ModoPezzoSeguente         : INT;
    WebApp                          :WebApplication;
  END_VAR

  VAR_EXTERNAL CONSTANT
    MODO_PEZZO_SEGUENTE                             :SINT;
    MODO_PEZZO_STANDARD                     :SINT;
  END_VAR

	VAR_INPUT
		Stato						:INT;
		ForzaturaStato	:BOOL:=FALSE;
		DatiProTube			:BOOL:=TRUE;
		DatiWebApp			:BOOL:=TRUE;
	END_VAR

	IF Protube.jobid=J_ID_RESET#JOB_ID_RESET AND Protube.JobGUID='' AND i_sys_modopezzoseguente>MODO_PEZZO_STANDARD THEN
		//Protube.jobstate:=JOB_STATE_NULLO;
		;
	ELSE
		IF DatiProTube=TRUE THEN
			IF i_sys_modopezzoseguente>=MODO_PEZZO_SEGUENTE OR ForzaturaStato=TRUE THEN
				Protube.jobstate:=Stato;
			END_IF;
		END_IF;	
	END_IF;
	IF DatiWebApp=TRUE THEN
		IF WebApp.wd_job_code<>'-1' THEN
			WebApp.wd_job_state:=Stato;
		ELSE
			//WebApp.wd_job_state:=JOB_STATE_NULLO;
			;
		END_IF;
	END_IF;	
END_FUNCTION	

FUNCTION ImpostaAL

  VAR_EXTERNAL
    Disp                                 :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
    Programmi                            :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    i_sys_current_prog	                 :INT;
    AL_LunghezzaTubo                     :LREAL;
  END_VAR
  
  VAR_EXTERNAL CONSTANT
    FIRST_DEVICE                    :INT;
    AL42_STATO_CARICO               :SINT;
    AL5_CARICO_MACCHINA             :SINT;
    CLAMP_CARGE                     :INT;
    AL42PLUS_STATO_CARICO           :SINT; 
  END_VAR

  VAR_INPUT
    Stato    :INT;
    Movimento  :BOOL;
  END_VAR
  // {
  // #ifdef AL_BLM
  // }
    Disp[CLAMP_CARGE].in_movimento:=Movimento;
    Disp[CLAMP_CARGE].stato:=Stato;
    Disp[CLAMP_CARGE].position:=Stato;
    IF Stato=AL42_STATO_CARICO OR
       Stato=AL42PLUS_STATO_CARICO OR
       Stato=AL5_CARICO_MACCHINA THEN
      //Al momento dell^impostazione dello stato carico si memorizza la lunghezza
      //del tubo preso sull^AL
      AL_LunghezzaTubo:=Programmi[i_sys_current_prog].LunghezzaTuboEsecuzione;
    END_IF;
  // {
  // #endif  //Fine AL_BLM
  // }
  ;    
END_FUNCTION

/**************************************************************************************************
 * RESETTING DISTANCE FROM THE END OF POSITIONING WITH POSSIBLE MAINTENANCE OF THE INDICATED AXIS *
 **************************************************************************************************/

FUNCTION AzzeramentoDistanceTogo

  VAR_EXTERNAL
    i_sys_PosFinale							:ARRAY[(_FIRST_AXIS#FIRST_AXIS) .. (_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS -1)] OF LREAL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    MY_BIG_VALUE_INT                :INT;
    MY_BIG_VALUE_REAL               :REAL;
  END_VAR

  VAR_INPUT
    Asse  :INT:=-MY_BIG_VALUE_INT;
  END_VAR
  VAR
    i     :INT;
  END_VAR 
  FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
    IF Asse>=_FIRST_AXIS#FIRST_AXIS AND Asse<=_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 THEN
      IF i<>Asse THEN
        i_sys_PosFinale[i]:=-MY_BIG_VALUE_REAL;
      END_IF;
    ELSE
      i_sys_PosFinale[i]:=-MY_BIG_VALUE_REAL;
    END_IF;
  END_FOR;
END_FUNCTION

FUNCTION_BLOCK FB_AL5_Sollevatore

  VAR_EXTERNAL CONSTANT
    AL5_SOLLEVATORE_FASCIO                        :INT;  //42
    OUT_AL5_ALZO_FASCIO                           :INT;
    OUT_AL5_ABBASSO_FASCIO                        :INT;
    TRC_AL5_SOLLEVATORE_FASCIO_ALTO               :INT;
    TRC_AL5_SOLLEVATORE_FASCIO_BASSO              :INT;
  END_VAR

  VAR_INPUT
    Stato    :INT;
    Attesa  :BOOL;
  END_VAR

  VAR
    dret    :DINT;
		
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
		Wait_Disp			:FB_Wait_disp;
		VerificaStop	:FB_VerificaStop;
  END_VAR  
	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
  ;  
  {
  #ifdef AL_BLM
  }
		CASE CURRENT_STATE OF
			GENERAL_STATE#STATE_INIT:
				IF STATO=DEVICES_STATUS#ARRESTO THEN
					AL5FrenoSollevatore(Stato:=FALSE);
					WriteDigitalOutput(NumOutput:=OUT_AL5_ALZO_FASCIO, Value:=FALSE);
					WriteDigitalOutput(NumOutput:=OUT_AL5_ABBASSO_FASCIO, Value:=FALSE);
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				ELSE
					IF (VerificaStop.OutputState<>GENERAL_STATE#STATE_END) THEN
						VerificaStop();
					ELSE
						VerificaStop();
						AL5FrenoSollevatore(Stato:=TRUE);
						Set_Disp(Ndisp:=AL5_SOLLEVATORE_FASCIO, Stato:=Stato, TimerRisparmio:=TRUE);
						IF Attesa=TRUE THEN
							CURRENT_STATE:=_FB_AL5_Sollevatore#STATE_SOLLEVATORE_WAIT_SOLLEVATORE;
						ELSE 
							CURRENT_STATE:=_FB_AL5_Sollevatore#STATE_SOLLEVATORE_VERIFICA_STOP;
						END_IF;
					END_IF;
				END_IF;		
				
        _FB_AL5_Sollevatore#STATE_SOLLEVATORE_WAIT_SOLLEVATORE:
				IF Stato=DEVICES_STATUS#LAVORO THEN
          set_trace(Code:=TRC_AL5_SOLLEVATORE_FASCIO_ALTO);
        ELSE
          set_trace(Code:=TRC_AL5_SOLLEVATORE_FASCIO_BASSO);
        END_IF;
				IF (wait_disp.outputstate<>GENERAL_STATE#STATE_END) THEN
					wait_disp(Device:=AL5_SOLLEVATORE_FASCIO, State:=Stato, Trace:=FALSE);
				ELSE
					reset_trace(Code:=TRC_AL5_SOLLEVATORE_FASCIO_ALTO);
        	reset_trace(Code:=TRC_AL5_SOLLEVATORE_FASCIO_BASSO);
					VerificaStop();
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
				
        _FB_AL5_Sollevatore#STATE_SOLLEVATORE_VERIFICA_STOP:
				IF (VerificaStop.OutputState<>GENERAL_STATE#STATE_END) THEN
					VerificaStop();
				ELSE
					VerificaStop();
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
      
			GENERAL_STATE#STATE_END:
				CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
		END_CASE;
		OutputState:=CURRENT_STATE;
  // {
  // #endif  //Fine AL_BLM
  // }
END_FUNCTION_BLOCK

/*********************************
 * RETURNS THE TYPE OF ACTIVE AL *
 *********************************/

FUNCTION IndicaTipoAl:SINT

  VAR_EXTERNAL
    cfg                             :_TYPE.Config_Struct;
  END_VAR

  VAR_EXTERNAL CONSTANT
    TIPO_CARM                          	            :SINT;
  END_VAR
  // {
  // #ifdef AL_BLM
  // }
    IF cfg.i_sys_caricatoreblm_dx>=TIPO_CARM THEN
      IndicaTipoAl:=cfg.i_sys_caricatoreblm_dx;  
    ELSE
      IF cfg.i_sys_caricatoreblm_sx>=TIPO_CARM THEN
        IndicaTipoAl:=cfg.i_sys_caricatoreblm_sx;  
      ELSE  
        IndicaTipoAl:=SINT#0;
      END_IF;  
    END_IF;
  // {
  //   #else    //Fine AL_BLM
  // }
    IndicaTipoAl:=SINT#0;
  // {
  //   #endif    //Fine NO AL_BLM
  // }
END_FUNCTION

/*********************************************************************************
 * CHECK IF THE TUBE IS PRESENT ON THE^AL AND READY FOR LOADING INTO THE MACHINE *
 *********************************************************************************/

FUNCTION TestALReady:BOOL

  VAR_EXTERNAL CONSTANT
    PROGRAM_JOG                      	:SINT;
  END_VAR

  VAR_EXTERNAL
    i_sys_OPMODE                    : INT;
  END_VAR
  
  VAR 
    ProgrammaAttivo                       :DINT;
    TipoAl                                :SINT;
    i_sys_current_prog	                  :INT;
  END_VAR

  TestALReady:=FALSE;
  TipoAl:=IndicaTipoAl();  
  IF i_sys_OpMode=PI_COMMANDS_CODE#AUTOMODE THEN
    ProgrammaAttivo:=I_SYS_Current_Prog;
  ELSE
    ProgrammaAttivo:=PROGRAM_JOG ;
  END_IF;
  // {
  // #ifndef MS0017 %%%%
  // }
  //   CASE TipoAl OF
  //     TIPO_AL42:  
  //       IF TestTuboAL42()=2 AND
  //          Disp[AL42SOSTEGNO].stato=RIPOSO AND 
  //          (Disp[AL42SPINTORE].stato=DEVICES_STATUS#LAVORO OR i_sys_SenzaTubo=TRUE) THEN
  //         TestALReady:=TRUE;
  //       ELSE
  //         IF Disp[PINZA_CARICO].stato=AL42_STATO_CARICO THEN
  //           //Reset stato carico se la posizione dei dispositivi non 
  //           //è compatibile
  //           Disp[PINZA_CARICO].stato:=AL_STATO_INDEFINITO;
  //         END_IF;
  //       END_IF;
  //     TIPO_AL42PLUS:
  //       //Condizioni per essere nello stato CARICO:
  //       //- Pinze chiuse
  //       //- Tubo presente in pinza
  //       //- Traslatore avanti
  //       //- L1 alla quota di carico
  //       IF readdigitalinput(NumInput:=INP_PRESENZA_PEZZO_SU_PINZA_AL42_PLUS)=FALSE AND
  //          i_sys_SenzaTubo=FALSE THEN  
  //         TestALReady:=FALSE;
  //       ELSE
  //         IF readdigitalinput(NumInput:=INP_PINZE_CHIUSE_AL42_PLUS)=TRUE AND
  //            readdigitalinput(NumInput:=INP_PINZE_APERTE_AL42_PLUS)=FALSE AND
  //            readdigitalinput(NumInput:=INP_TRASLAZIONE_AVANTI_AL42_PLUS)=TRUE AND 
  //            readdigitalinput(NumInput:=INP_TRASLAZIONE_INDIETRO_AL42_PLUS)=FALSE AND
  //            ABS(Axes[L1].PosAx.positioningState.actualPosition-Programmi[ProgrammaAttivo].DatiAL.L1PosCarico)<pm[L1].ma_standstill_pos_tol THEN
  //           TestALReady:=TRUE;
  //         END_IF;
  //       END_IF; 
  //     TIPO_AL5, TIPO_AL5_BUNDLE, TIPO_AL5_ROLAND, TIPO_AL5_ROLAND_BUNDLE, 
  //     TIPO_AL5_LUNGO, TIPO_AL5_LUNGO_BUNDLE, TIPO_AL5_FUTABA:
  //       //Condizioni per essere nello stato CARICO:
  //       //- Pinze non aperte
  //       //- L1 alla quota di carico
  //       TestALReady:=FALSE;
  //       IF ABS(Axes[L1].PosAx.positioningState.actualPosition-Programmi[ProgrammaAttivo].DatiAL.L1PosCarico)<pm[L1].ma_standstill_pos_tol THEN
  //         IF i_sys_SenzaTubo=TRUE THEN
  //           TestALReady:=TRUE;
  //         ELSE  
  //           IF readdigitalinput(NumInput:=INP_AL5_PINZA_APERTA_DX)=FALSE AND 
  //              readdigitalinput(NumInput:=INP_AL5_PINZA_APERTA_SX)=FALSE THEN
  //             TestALReady:=TRUE;
  //           END_IF;  
  //         END_IF;  
  //       END_IF;  
  //   END_CASE;
  // {
  // #endif  //Fine No MS0017
  // }
END_FUNCTION
// {
// #endif  //Fine AL_BLM
// }

FUNCTION AL5FrenoSollevatore

  VAR_EXTERNAL CONSTANT
    OUT_AL5_FRENO_FASCIO                          :INT;
  END_VAR
  
  VAR_INPUT
    Stato  :BOOL;
  END_VAR
  ;  
  // {
  // #ifdef AL_BLM
  // }
    WriteDigitalOutput(NumOutput:=OUT_AL5_FRENO_FASCIO, Value:=Stato);
  // {
  // #endif  //Fine AL_BLM
  // }
END_FUNCTION

/**********************************
 * LOAD RESERVATION LIGHT SETTING *
 **********************************/
FUNCTION AL5LucePrenotazioneCarico

  VAR_EXTERNAL CONSTANT
    OUT_AL5_RICHIESTA_CARICO_FASCIO               :INT;
  END_VAR
  
  VAR_INPUT
    Stato  :BOOL;
  END_VAR  
  // {
  // #ifdef AL_BLM
  // }
    WriteDigitalOutput(NumOutput:=OUT_AL5_RICHIESTA_CARICO_FASCIO, Value:=Stato);
  // {
  // #endif  //Fine AL_BLM
  // }
  ;
END_FUNCTION

/********************************************************************************
 * SETTING THE ^OUTPUT ENABLING THE LOADING ROBOT TO TAKE THE HOSE FROM THE ^AL *
 ********************************************************************************/
// {
// #ifdef AL_BLM
// }
  FUNCTION ALRobotProntoScarico

    VAR_EXTERNAL
      i_sys_DpCoupler                 :BOOL;
      i_sys_PnCoupler                 :BOOL;
      OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
    END_VAR

    VAR_EXTERNAL CONSTANT
      MAXDIGITALOUTPUT                              :INT;
      OUT_CARICO_TUBO_PRESENTE_AL                   :INT;
      OUT_CARICO_TUBO_PRESENTE_AL_DPCOUPLER         :INT;
    END_VAR

    VAR_INPUT
      Stato      :BOOL;
      Programma  :INT;
    END_VAR
    VAR
      Uscita    :INT;
    END_VAR
    IF TestRobotDaAl(Programma:=Programma)=TRUE THEN
      IF (i_sys_DpCoupler=TRUE OR i_sys_PnCoupler=TRUE) AND
         oaddress[OUT_CARICO_TUBO_PRESENTE_AL].port=-1 AND 
         oaddress[OUT_CARICO_TUBO_PRESENTE_AL].bit=-1 THEN
        Uscita:=OUT_CARICO_TUBO_PRESENTE_AL_DPCOUPLER;
      ELSE
        Uscita:=OUT_CARICO_TUBO_PRESENTE_AL;
      END_IF;            
      WriteDigitalOutput(NumOutput:=Uscita, Value:=Stato, Verifica:=TRUE);
    END_IF;
  END_FUNCTION

/************************************************************************************
 * OUTLET SETTING THAT ENABLES THE LOADING ROBOT TO EVACUATE THE TUBE FROM THE^AL42 *
 ************************************************************************************/
  FUNCTION ALRobotTuboLibero

    VAR_EXTERNAL
      i_sys_DpCoupler                 :BOOL;
      i_sys_PnCoupler                 :BOOL;
      OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
    END_VAR

    VAR_EXTERNAL CONSTANT
      MAXDIGITALOUTPUT                    :INT;

      OUT_CARICO_TUBO_LIBERO_AL                     :INT;
      OUT_CARICO_TUBO_LIBERO_AL_DPCOUPLER           :INT;
    END_VAR

    VAR_INPUT
      Stato      :BOOL;
      Programma :INT;
    END_VAR
    VAR
      Uscita    :INT;
    END_VAR
    IF TestRobotDaAl(Programma:=Programma)=TRUE THEN
      IF (i_sys_DpCoupler=TRUE OR i_sys_PnCoupler=TRUE) AND
         oaddress[OUT_CARICO_TUBO_LIBERO_AL].port=-1 AND 
         oaddress[OUT_CARICO_TUBO_LIBERO_AL].bit=-1 THEN
        Uscita:=OUT_CARICO_TUBO_LIBERO_AL_DPCOUPLER;
      ELSE
        Uscita:=OUT_CARICO_TUBO_LIBERO_AL;
      END_IF;            
      WriteDigitalOutput(NumOutput:=Uscita, Value:=Stato, Verifica:=TRUE);
    END_IF;
  END_FUNCTION

/****************************************************************
 * TRUE IF THE TWO SYNCHRONISATION SENSORS HAVE THE SAME STATUS *
 ****************************************************************/

FUNCTION TestSincrCatene:BOOL

  VAR_EXTERNAL
    TimerSystem                     :ARRAY[0..MAX_TIMER_SYSTEM-1] OF Timer;
  END_VAR

  VAR_EXTERNAL CONSTANT
    INP_AL5_SINCR_CATENA_SX                         :INT;
    INP_AL5_SINCR_CATENA_DX                         :INT;
    T_SINCR_CATENE_AL5	                            :INT;
    MAX_TIMER_SYSTEM 	                              :USINT;
  END_VAR

  VAR
    dret  :DINT;
  END_VAR  
  ;  
  // {
  // #ifdef AL_BLM
  // }
    IF TimerSystem[T_SINCR_CATENE_AL5].Value=0 THEN
      //Serve per inibite la verifica del sincronismo fin tanto che il timer non si è azzerato.
      IF (ReadDigitalInput(NumInput:=INP_AL5_SINCR_CATENA_SX, Verifica:=FALSE)=TRUE AND
          ReadDigitalInput(NumInput:=INP_AL5_SINCR_CATENA_DX, Verifica:=FALSE)=TRUE) OR
         (ReadDigitalInput(NumInput:=INP_AL5_SINCR_CATENA_SX, Verifica:=FALSE)=FALSE AND
          ReadDigitalInput(NumInput:=INP_AL5_SINCR_CATENA_DX, Verifica:=FALSE)=FALSE) THEN
          TestSincrCatene:=TRUE;
      ELSE
        TestSincrCatene:=FALSE;
      END_IF;
    ELSE
      TestSincrCatene:=TRUE;
    END_IF;
  // {
  // #endif
  // }
END_FUNCTION

/********************************************************************************************************
 * CHECK WHETHER A ROBOT LOAD THAT PICKS UP THE PART FROM THE^AL IS FORESEEN IN THE INDICATED PROGRAMME *
 ********************************************************************************************************/

FUNCTION TestRobotDaAL:BOOL

  VAR_EXTERNAL
    Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
  END_VAR

  VAR_EXTERNAL CONSTANT
    CARICO_AL_BLM_ROBOT                	            :SINT;
  END_VAR

  VAR_INPUT
    Programma  :DINT;
  END_VAR
  TestRobotDaAL:=programmi[Programma].TipoCarico=CARICO_AL_BLM_ROBOT;
END_FUNCTION

FUNCTION_BLOCK FB_Wait_disp   

  VAR_EXTERNAL 
    Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
    StatusTrace                     :BOOL;
    i_sys_OPMODE                    : INT;
  END_VAR

  VAR_EXTERNAL CONSTANT
    FIRST_DEVICE                    :INT;
    AUTOMODE                        :INT;
    TRC_DISPOSITIVI                 :INT;
  END_VAR

  VAR_INPUT
    Device			:INT;
    State				:INT;
    Trace				:BOOL:=TRUE;
  END_VAR  
  VAR
    dret				:DINT;
    Attivazione	:BOOL;
		
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
  END_VAR
	VAR_OUTPUT
		OutputState		:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF Disp[Device].Enable=TRUE THEN
				IF i_sys_OPMODE=AUTOMODE THEN
					//Serve per evitare di segnalare macchna ferma quando si attende un dispositivo complesso
					//la cui gestione dei tempi non è prevista da VGP
					Attivazione:=TRUE;
					StatusTrace:=TRUE;
				ELSE
					Attivazione:=FALSE;
				END_IF;  
				CURRENT_STATE:=_FB_WaitDisp#STATE_ATTESA_DISPOSITIVO;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
    _FB_WaitDisp#STATE_ATTESA_DISPOSITIVO:
			IF Disp[Device].Stato<>State OR Disp[Device].consenso_minus=FALSE OR Disp[Device].consenso_plus=FALSE THEN
				IF Trace=TRUE THEN
					Set_Trace(Code:=TRC_DISPOSITIVI+Device, Axis:=Device);
				END_IF;
			ELSE
				IF Attivazione=TRUE THEN
					StatusTrace:=FALSE;
				END_IF;  
				IF Trace=TRUE THEN
					reset_trace(TRC_DISPOSITIVI+Device);
				END_IF;
				IF Disp[Device].SpegnimentoUscita=TRUE THEN
					IF Disp[Device].Stato=State THEN
						IF State=Disp[Device].limite_lavoro THEN
							WriteDigitalOutput(NumOutput:=Disp[Device].out_lavoro, Value:=FALSE);
						ELSE
							IF State=Disp[Device].limite_riposo THEN
								WriteDigitalOutput(NumOutput:=Disp[Device].out_riposo, Value:=FALSE);
							END_IF;
						END_IF;
					END_IF;
				END_IF;  
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
				
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;	
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_VerificaStop

  VAR_EXTERNAL CONSTANT
    i_sys_stopcond                          : INT;
  END_VAR

  VAR
		CURRENT_STATE:INT:=GENERAL_STATE#STATE_INIT;
  END_VAR
	VAR_OUTPUT
		OutputState	:INT:=GENERAL_STATE#STATE_INIT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF NOT (i_sys_StopCond=1) THEN
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

/*************************************************************
 * SETS THE DIGITAL OUTPUTS FOR SELECTING THE PROTECTED AREA *
 *************************************************************/
FUNCTION AttivaProgrammaRadar

  VAR_EXTERNAL CONSTANT
    OUT_RADAR_0                                 :INT; //Gestione RADAR 
    OUT_RADAR_1                                 :INT; //Gestione RADAR 
    OUT_RADAR_2                                 :INT; //Gestione RADAR 
    OUT_RADAR_3                                 :INT; //Gestione RADAR
  END_VAR

  VAR_INPUT
    Codice:  INT;
  END_VAR 
  // {
  // #ifdef PROFINET
  // }
    // {
    // #ifndef RUNNER %%%%
    // }
    //   {
    //   #ifndef EFLEX_DH4010
    //   }
    //     CASE Codice OF
    //       0:O_Radar_Zona_1:=FALSE;
    //         O_Radar_Zona_2:=FALSE;
    //         O_Radar_Zona_3:=FALSE;
    //       1:O_Radar_Zona_1:=TRUE;
    //         O_Radar_Zona_2:=FALSE;
    //         O_Radar_Zona_3:=FALSE;
    //       2:O_Radar_Zona_1:=FALSE;
    //         O_Radar_Zona_2:=TRUE;
    //         O_Radar_Zona_3:=FALSE;
    //       3:O_Radar_Zona_1:=FALSE;
    //         O_Radar_Zona_2:=FALSE;
    //         O_Radar_Zona_3:=TRUE;
    //     END_CASE;
    //   {
    //   #endif  //FINE NO EFLEX_DH4010
    //   }
    // {
    // #endif  //Fine NO RUNNER
    // }
  // {
  // #else  //Fine PROFINET
  // }
    CASE Codice OF
      1:WriteDigitalOutput(NumOutput:=OUT_RADAR_0, Value:=BOOL#0, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_1, Value:=BOOL#1, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_2, Value:=BOOL#0, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_3, Value:=BOOL#0, Verifica:=TRUE);
          
      2:WriteDigitalOutput(NumOutput:=OUT_RADAR_0, Value:=BOOL#1, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_1, Value:=BOOL#0, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_2, Value:=BOOL#0, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_3, Value:=BOOL#0, Verifica:=TRUE);
          
      3:WriteDigitalOutput(NumOutput:=OUT_RADAR_0, Value:=BOOL#0, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_1, Value:=BOOL#0, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_2, Value:=BOOL#0, Verifica:=TRUE);
        WriteDigitalOutput(NumOutput:=OUT_RADAR_3, Value:=BOOL#1, Verifica:=TRUE);
    END_CASE;
  // {
  // #endif    //Fine NO PROFINET
  // }
  ;
END_FUNCTION  

FUNCTION ResetCodificaPezzo
  // {
  // #ifdef CARICO_AUTOMATICO %%%%
  // }
	// 	WriteDigitalOutput(NumOutput:=OUT_SELEZ_PEZZO_BIT0, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_SELEZ_PEZZO_BIT1, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_SELEZ_PEZZO_BIT2, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_SELEZ_PEZZO_BIT3, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_SELEZ_PEZZO_BIT4, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_SELEZ_PEZZO_BIT5, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_SELEZ_PEZZO_BIT6, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_PEZZO_ESEGUITO_BIT0_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_PEZZO_ESEGUITO_BIT1_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_PEZZO_ESEGUITO_BIT2_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_PEZZO_ESEGUITO_BIT3_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_PEZZO_ESEGUITO_BIT4_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_PEZZO_ESEGUITO_BIT5_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
	// 	WriteDigitalOutput(NumOutput:=OUT_PEZZO_ESEGUITO_BIT6_DPCOUPLER, Value:=FALSE, Verifica:=FALSE);
	// 	{
	// 	#ifdef PROFINET
	// 	}    
	// 		pncoupler_o51:=0;
	// 	{
	// 	#endif  //Fine PROFINET
	// 	}    
	// 	dbcoupler_o51:=0;
	// {
  // #endif  //Fine CARICO_AUTOMATICO
  // }
  ;
END_FUNCTION

/*******************************************************************************
 * SETTING ON X1 AXIS WITH ABSOLUTE ENCODER, OF THE OFFSET CALCULATED BY VGP3D *
 *******************************************************************************/

FUNCTION ImpostazioneOffsetX1
  VAR_INPUT
    Offset:LREAL;
  END_VAR
  VAR
    dret  :DINT;
    Delta  :LREAL;
  END_VAR
  // {
  // #ifndef ORBITALE
  // }
  //   IF pm[X1].ma_num_encs=1 THEN 
  //     Delta:=Assi[X1].PosAx.positioningstate.actualposition-Assi[X1].PosAx.sensordata[1].position;
  //     IF ABS(Offset-Delta)>0.1 THEN
  //       dret:=_redefineposition(axis:=Assi[X1].PosAx,
  //                               redefinespecification:=COMMAND_VALUE,
  //                               redefinemode:=RELATIVE,
  //                               position:=Offset-Delta,
  //                               nextcommand:=WHEN_COMMAND_DONE);
  //     END_IF;
  //   END_IF;
  // {
  // #endif  //Fine NO ORBITALE
  // }
  ;
END_FUNCTION

//Verifica se l^asse passato come parametro è un bloccaggio
FUNCTION TestAsseBloccaggio:BOOL

  VAR_EXTERNAL CONSTANT
    W3 :    INT;
  END_VAR

  VAR_INPUT
    Asse  :INT;
  END_VAR;
  // {
  // #ifdef BLOCCAGGIO_SX %%%%
  // }
  //   TestAsseBloccaggio:=(Asse=W3) OR (Asse=W5);
  // {
  // #else    //Fine BLOCCAGGIO_SX
  // }
    // {
    // #ifdef BLOCCAGGIO_DX
    // }
      TestAsseBloccaggio:=(Asse=W3);
    // {
    // #endif    //Fine BLOCCAGGIO_DX
    // }
  // {
  // #endif    //Fine NO BLOCCAGGIO_SX
  // }
END_FUNCTION

/*************************************************************************
 * SETS THE ^ ACTIVE TOOL TO INDEFINITE IF ONE OF THE HEAD AXES IS MOVED *
 *************************************************************************/
FUNCTION TestAsseUtensile
  VAR_INPUT
    Asse  :INT;
  END_VAR
  // {
  // #ifdef CURVATURA
  // }
  //   {
  //   #ifdef EFLEX_DH4010
  //   }
  //     {
  //     #ifdef EFLEX
  //     }
  //       IF Asse=Q1 THEN
  //         i_sys_utensileAttivo:=0;
  //       END_IF;
  //     {
  //     #endif    //Fine EFLEX
  //     }
  //     {
  //     #ifdef DH4010
  //     }
  //       CASE Asse OF
  //         Q1:i_sys_utensileAttivo[TABELLA_SX]:=0;
  //         Q2:i_sys_utensileAttivo[TABELLA_DX]:=0;
  //       END_CASE
  //     {
  //     #endif    //Fine DH4010
  //     }
  //   {
  //   #else    //Fine EFLEX_DH4010
  //   }
  //     IF Asse=V1 OR Asse=Q1 THEN
  //       i_sys_utensileAttivo:=0;
  //     END_IF;
  //   {
  //   #endif  //Fine NO EFLEX_DH4010
  //   }  
  // {
  // #endif  //Fine CURVATURA
  // }  
  ;
END_FUNCTION

/***********************************************
 * READS THE ACTIVE SAFETY STATUS ON THE^DRIVE *
 ***********************************************/

// {
// ::ifdef PROFINET
// }     
  FUNCTION ReadStateSafety:DWORD

    VAR_EXTERNAL
      StatoSafetety_X1:WORD;
      StatoSafetety_Y1:WORD;
      StatoSafetety_Z1:WORD;
      StatoSafetety_Q1:WORD;
      StatoSafetety_R1:WORD;
      StatoSafetety_W3:WORD;
      StatoSafetety_W4:WORD;
      StatoSafetety_U4:WORD;
      StatoSafetety_L1:WORD;
      StatoSafetety_Z2:WORD;
    END_VAR

    VAR_EXTERNAL CONSTANT
      Z2  :INT;
      X1:INT;
    
      Y1:INT;
   
      Z1:INT;
    
      Q1:INT;
  
      V1:INT;
      W3 : INT;
      W4 :INT;
      U4 :INT;
      L1 :INT;
 
    END_VAR

    VAR_INPUT
      Asse  :INT;
    END_VAR
    ;
    // {
    // #ifndef EMULAZIONE
    // }
      // {
      // #ifdef ORBITALE
      // }
      //   CASE Asse OF
      //     {
      //     #ifdef RUNNER
      //     }
      //       X1:
      //         ReadStateSafety:=StatoSafetety_X1;
      //       W2:
      //         ReadStateSafety:=StatoSafetety_W2;
      //       U1:
      //         ReadStateSafety:=StatoSafetety_U1;
      //       U2:
      //         ReadStateSafety:=StatoSafetety_U2;
      //       {
      //       #ifndef PLAUNO_LIGHT
      //       }
      //         W1:
      //           ReadStateSafety:=StatoSafetety_W1;
      //         A1:
      //           ReadStateSafety:=StatoSafetety_A1;
      //         {
      //         #ifndef THREE_RUNNER
      //         }
      //           A2:
      //             ReadStateSafety:=StatoSafetety_A2;
      //         {
      //         #endif //FINE NO THREE_RUNNER
      //         }
      //       {
      //       #endif //FINE NO PLAUNO_LIGHT
      //       }
      //       {
      //       #ifdef FOUR_RUNNER_PLAUNO
      //       }
      //         Y1:
      //           ReadStateSafety:=StatoSafetety_Y1;
      //         Z1:
      //           ReadStateSafety:=StatoSafetety_Z1;
      //         Q1:
      //           ReadStateSafety:=StatoSafetety_Q1;
      //         V1:
      //           ReadStateSafety:=StatoSafetety_V1;
      //         W3:
      //           ReadStateSafety:=StatoSafetety_W3;
      //         {
      //         #ifdef SPALLAMENTO_DX
      //         }
      //           W4:
      //             ReadStateSafety:=StatoSafetety_W4;
      //         {
      //         #endif    //Fine SPALLAMENTO_DX
      //         }
      //         {
      //         #ifdef FOUR_RUNNER
      //         }
      //           V2:
      //             ReadStateSafety:=StatoSafetety_V2;
      //         {
      //         #endif    //Fine FOUR_RUNNER
      //         }

      //       {
      //       #endif    //Fine FOUR_RUNNER_PLAUNO
      //       }
      //       {
      //       #ifdef B1_B2
      //       }
      //         B1:
      //           ReadStateSafety:=StatoSafetety_B1;
      //         B2:
      //           ReadStateSafety:=StatoSafetety_B2;
      //       {
      //       #endif    //Fine B1_B2
      //       }
      //     {
      //     #endif    //Fine RUNNER
      //     }
      //     {
      //     #ifdef EFLEX
      //     }
      //                   X1:
      //                       ReadStateSafety:=StatoSafetety_X1;
      //                   Y1:
      //                       ReadStateSafety:=StatoSafetety_Y1;
      //                   Z1:
      //                       ReadStateSafety:=StatoSafetety_Z1;
      //                   Q1:
      //                       ReadStateSafety:=StatoSafetety_Q1;
      //       W1:
      //         ReadStateSafety:=StatoSafetety_W1;
      //     {
      //     #endif    //EFLEX
      //     }
      //     {
      //     #ifdef DH4010
      //     }
      //                   X1:
      //                       ReadStateSafety:=StatoSafetety_X1;
      //                   X2:
      //                       ReadStateSafety:=StatoSafetety_X2;
      //                   Y1:
      //                       ReadStateSafety:=StatoSafetety_Y1;
      //                   Y2:
      //                       ReadStateSafety:=StatoSafetety_Y2;
      //                   Q1:
      //                       ReadStateSafety:=StatoSafetety_Q1;
      //                   Q2:
      //                       ReadStateSafety:=StatoSafetety_Q2;
      //                   Z1:
      //                       ReadStateSafety:=StatoSafetety_Z1;
      //       X3:
      //         ReadStateSafety:=StatoSafetety_X3;
      //       X6:
      //         ReadStateSafety:=StatoSafetety_X6;
      //     {
      //     #endif    //DH4010
      //     }
      //   ELSE
      //     ReadStateSafety:=1;
      //   END_CASE;
      // {
      // #else    //Fine macchine oribitali
      // }
        CASE Asse OF
          X1:
            ReadStateSafety:=StatoSafetety_X1;
          Y1:
            ReadStateSafety:=StatoSafetety_Y1;
          Z1:
            ReadStateSafety:=StatoSafetety_Z1;
          Q1:
            ReadStateSafety:=StatoSafetety_Q1;
          V1:
            ReadStateSafety:=StatoSafetety_R1;
          // {
          // #ifdef BLOCCAGGIO_DX
          // }
                        W3:
                            ReadStateSafety:=StatoSafetety_W3;
          // {
          // #endif    //Fine BLOCCAGGIO_DX
          // }
          // {
          // #ifdef SPALLAMENTO_DX
          // }
                        W4:
                            ReadStateSafety:=StatoSafetety_W4;
          // {
          // #endif    //Fine SPALLAMENTO_DX
          // }
          // {
          // #ifdef SLITTA_MOBILE_DX
          // }
                        U4:
                            ReadStateSafety:=StatoSafetety_U4;
          // {
          // #endif    //Fine SLITTA_MOBILE_DX
          // }
          // {
          // #ifdef NO_ASSE_U2 %%%%
          // }
          //               U2:
          //                   //Non esiste l^azionamento U2 si prende l^informazione da un asse sicuramente presente
          //                   ReadStateSafety:=StatoSafetety_Y1;
          // {
          // #else    //Fine NO_ASSE_U2
          // }
          //               U2:
          //                   ReadStateSafety:=StatoSafetety_U2;
          // {
          // #endif    //Fine NO NO_ASSE_U2
          // }
          // {
          // #ifdef BLOCCAGGIO_SX %%%%
          // }
          //   W5:
          //     ReadStateSafety:=StatoSafetety_W5;
          // {
          // #endif    //Fine BLOCCAGGIO_SX
          // }
          // {
          // #ifdef SPALLAMENTO_SX
          // }
          //   W6:
          //     ReadStateSafety:=StatoSafetety_W6;
          // {
          // #endif    //Fine SPALLAMENTO_SX
          // }
          // {
          // #ifdef SLITTA_MOBILE_SX
          // }
          //   U5:
          //     ReadStateSafety:=StatoSafetety_U5;
          // {
          // #endif    //Fine SLITTA_MOBILE_SX
          // }
					// {                
					// #ifdef MATRICE_ELETTRICA
					// }
          //   Y2:
          //     ReadStateSafety:=StatoSafetety_Y2;
          // {
          // #endif    //Fine MATRICE_ELETTRICA
          // }
					// {
					// #ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO
					// }
          //   V4:
          //     ReadStateSafety:=StatoSafetety_V4;
					// {
					// #endif    //Fine SOLLEVAMENTO_SLITTA_SPALLAMENTO
					// }
          // {
          // #ifdef AL_BLM
          // }
            L1:
              ReadStateSafety:=StatoSafetety_L1;
          // {
          // #endif    //Fine AL_BLM
          // }
					// {
					// #ifdef ASSE_Z2
					// }
            Z2:
              ReadStateSafety:=StatoSafetety_Z2;
					// {
					// #endif //Fine ASSE_Z2
					// }
					// {
					// #ifdef PINZA_ELETTRICA
					// }
					// 	W1:
					// 		ReadStateSafety:=StatoSafetety_W1;
					// {
					// #endif //Fine PINZA_ELETTRICA
					// }
					// {
					// #ifdef SOSTEGNO_ELETTRICO
					// }
					// 	S1:
					// 		ReadStateSafety:=StatoSafetety_S1;
					// {
					// #endif //Fine SOSTEGNO_ELETTRICO
					// }
					// {
					// #ifdef BASCULANTE_ELETTRICO
					// }
					// 	B2:
					// 		ReadStateSafety:=StatoSafetety_B2;
					// {
					// #endif //Fine BASCULANTE_ELETTRICO
					// }
        ELSE
          ReadStateSafety:=DWORD#1;
        END_CASE;
      // {
      // #endif    //Fine NO ORBITALE
      // }
    // {
    // ::endif    //Fine NO EMULAZIONE
    // };
  END_FUNCTION
// {
// ::endif    //Fine PROFINET
// };  

/***************************************************************
 * CHECKS WHETHER THE^AXIS PASSED AS A PARAMETER IS A SHOULDER *
 ***************************************************************/
FUNCTION TestAsseSpallamento:BOOL

  VAR_EXTERNAL CONSTANT
    W4:INT;
  END_VAR

  VAR_INPUT
    Asse  :INT;
  END_VAR;
    TestAsseSpallamento:=FALSE;
  // {
  // #ifdef SPALLAMENTO_SX
  // }
  //   TestAsseSpallamento:=(Asse=W4) OR (Asse=W6);
  // {
  // #else    //Fine SPALLAMENTO_SX
  // }
  {::ifdef SPALLAMENTO_DX}
      TestAsseSpallamento:=(Asse=W4);
  {::endif}    //Fine SPALLAMENTO_DX

END_FUNCTION

END_NAMESPACE