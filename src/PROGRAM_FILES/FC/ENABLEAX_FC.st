USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING System.BitAccess;
USING System.Math;
USING ENABLEPROGRAM;
USING ENABLEAX_FUNCTIONS;
USING  _TYPE;
USING _FC_FC; 
USING _FC_IO_FC;
USING _CONST_VAR_DEF;

NAMESPACE EnableAxFunction
////_ENABLEAX

/******************************************************************
 * THE FUNCTION SETS CONFIGURATION DATA FOR THE POSITIONING AXIS. *
 ******************************************************************/
 
 FUNCTION On_The_PM_line 

	VAR_EXTERNAL 
		////_VARGLOBAL
		Axes                    		: ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
		pm                      		: ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
		i_sys_TrackingErrorX1    		:REAL;
		i_sys_TrackingErrorX2      		:REAL;

		///TO Defenition
		PosAx                      :DB_ANY;
	END_VAR

	VAR_EXTERNAL CONSTANT
		// ASSI MACCHINA
		X1                      :INT;
		X2 						:INT;    //Carro X
		///_CONST
		//MY_BIG_VALUE_REAL     	:REAL;
	END_VAR

	VAR_INPUT 
		Axis 		            :INT;
	END_VAR

	VAR
		dret		    :DINT;
		Esito		    :BOOL;
		myPosAxis	    :REF_TO TO_PositioningAxis;

		//PosAxRef 		:REF_TO TO_PositioningAxis;
		//myPosAxisRef	:REF_TO TO_PositioningAxis;
		FirstCycle   	:BOOL;
	END_VAR	

	(*IF NOT FirstCycle THEN
        FirstCycle:=TRUE;
		PosAxRef        	:= AsPositioningAxisRef(PosAx);
        myPosAxisRef       	:= AsPositioningAxisRef(myPosAxis);
	END_IF;*)
	
	
		IF CheckIfTOactive(Axis:=Axis)=TRUE THEN
			myPosAxis := AsPositioningAxisRef(Axes[Axis].PosAx);
		//IF Axes[Axis].PosAx.typeofaxis<> VIRTUAL_AXIS AND Axes[Axis].PosAx <> 0  AND pm[Axis].ma_asse_abilitato=TRUE THEN
		IF myPosAxis^.VirtualAxis.Mode <> UDINT#0 AND myPosAxis <> NULL AND pm[Axis].ma_asse_abilitato=TRUE THEN //= "TO_PositioningAxis" AND <> VIRTUAL_AXIS AND myPosAxis <> 0  AND pm[Axis].ma_asse_abilitato=TRUE THEN
			//myPosAxis:=Axes[Axis].PosAx;// canc
			myPosAxis^.PositionLimits_SW.active:=TRUE;  
			pm[Axis].max_ax_velo:= TO_REAL (myPosAxis^.DynamicLimits.MaxVelocity);
			//Si effettua il test sul valore della variabile prima di impostarla in quanto il comando è
			//lento e la ripetizione initile porta oltre 6 secondi (per tutti gli assi)
			(*IF myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_1.dynamicfollowing.minpositiontolerance<>myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_1.dynamicfollowing.maxpositiontolerance THEN
				myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_1.dynamicfollowing.minpositiontolerance:=myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_1.dynamicfollowing.maxpositiontolerance;
			END_IF;
			IF MyPosAxis.SetConfigData.TypeOfAxis.NumberOfDataSets.numberofdatasets>1 THEN
				IF myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_2.dynamicfollowing.minpositiontolerance<>myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_2.dynamicfollowing.maxpositiontolerance THEN
					myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_2.dynamicfollowing.minpositiontolerance:=myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_2.dynamicfollowing.maxpositiontolerance;
				END_IF;
			END_IF;
			IF MyPosAxis.SetConfigData.TypeOfAxis.NumberOfDataSets.numberOfDataSets>2 THEN
				IF myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_3.dynamicfollowing.minpositiontolerance<>myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_3.dynamicfollowing.maxpositiontolerance THEN
					myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_3.dynamicfollowing.minpositiontolerance:=myPosAxis.SetConfigData.typeofaxis.numberofdatasets.dataset_3.dynamicfollowing.maxpositiontolerance;
				END_IF;
			END_IF;*)
			IF myposaxis^.PositioningMonitoring.MinDwellTime <> 0 THEN
				myposaxis^.PositioningMonitoring.MinDwellTime:=0;
			END_IF;
			IF myPosAxis^.PositioningMonitoring.ToleranceTime <>pm[Axis].MA_Tolerance THEN
				myPosAxis^.PositioningMonitoring.ToleranceTime:=pm[Axis].MA_Tolerance;//36010
			END_IF;
			IF myPosAxis^.PositioningMonitoring.Window <>pm[Axis].MA_POS_WIN_TOL_TIME THEN
				myPosAxis^.PositioningMonitoring.Window:=pm[Axis].MA_POS_WIN_TOL_TIME;//36020
			END_IF;
			IF myPosAxis^.StandstillSignal.VelocityThreshold<>pm[Axis].MA_standstill_pos_tol THEN
				myPosAxis^.StandstillSignal.VelocityThreshold:=pm[Axis].MA_standstill_pos_tol;//36030
			END_IF;
			IF myPosAxis^.StandstillSignal.MinDwellTime <>pm[Axis].MA_standstill_DELAY_TIME THEN
				myPosAxis^.StandstillSignal.MinDwellTime:=pm[Axis].MA_standstill_DELAY_TIME;//36040
			END_IF;
			(*{
			#ifdef EFLEX
			}
				IF Axis=X1 THEN
					i_sys_TrackingErrorX1:=LREAL_TO_REAL(Axes[X1].PosAx.SetConfigData.TypeOfAxis.numberofdatasets.dataset_2.dynamicfollowing.maxpositiontolerance);
				END_IF;
			{
			#endif	//Fine EFLEX
			}
			{
			#ifdef DH4010  
			}	
				CASE Axis OF
					X1:
						i_sys_TrackingErrorX1:=LREAL_TO_REAL(Axes[X1].PosAx.SetConfigData.TypeOfAxis.numberofdatasets.dataset_1.dynamicfollowing.maxpositiontolerance);
					X2:
						i_sys_TrackingErrorX2:=LREAL_TO_REAL(Axes[X2].PosAx.SetConfigData.TypeOfAxis.numberofdatasets.dataset_1.dynamicfollowing.maxpositiontolerance);
				END_CASE;
			(*{
			#endif	//Fine DH4010  
			}*)	
		END_IF;
	END_IF;
END_FUNCTION

/**************************************************************************************************
 *   CHECKS WHETHER IT IS NECESSARY TO PERFORM A SETTING COMMAND FOR THE MOTORS PRESENT IN THE    *
 *              MACHINE BASED ON THE STATE OF THE VARIABLE PM[].MA_ASSE_INSTALLATO,               *
 * AS AN EVENTUAL VALUE OF 0 IS NOT CORRECT WITH DEACTIVATABLE AXES, THE RETENTION HAS BEEN LOST, *
 *                 AND THEREFORE IT IS NECESSARY TO PERFORM THE SETTING COMMAND.                  *
 *     IT ALSO VERIFIES AS AN OPTION THE STATE OF THE MOTOR ON THE DRIVE AND THAT SET BY VGP.     *
 *                   RETURNS TRUE IF THE OUTCOME IS OK. IN CASE OF KO OUTCOME,                    *
 *                   IT INDICATES THE FIRST AXIS THAT HAS NOT PASSED THE CHECK.                   *
 **************************************************************************************************/

FUNCTION TestDeactivationAxesState : BOOL

	VAR_EXTERNAL
		pm                      	:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
		Axes                        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
		i_sys_MotorState			:ARRAY[(_FIRST_AXIS#FIRST_AXIS)..(_FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1)] OF REAL; 
	END_VAR

	VAR_EXTERNAL CONSTANT
		NON_DEACTIVATABLE_AXIS              :SINT;
		ACTIVE_DEACTIVATABLE_AXIS           :SINT;
		INACTIVE_DEACTIVATABLE_AXIS         :SINT;
		AXIS_ACTIVE                         :REAL;
	END_VAR

	VAR_INPUT
		VerificaPm105				:BOOL;
	END_VAR
	VAR_IN_OUT
		ErroreAsse					:INT;
	END_VAR
	VAR
		i							:INT;
		PosAx						: REF_TO TO_PositioningAxis;
	END_VAR


	TestDeactivationAxesState:=TRUE;
  	ErroreAsse:=ShadowMotorCheck();
  IF ErroreAsse>=_FIRST_AXIS#FIRST_AXIS THEN
    TestDeactivationAxesState:=FALSE;

  ELSE
    (*{
    #ifndef EMULAZIONE
    }*)
      //Il precedente controllo ha dato esito positivo, occorre quindi effettuare questo
     FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1 DO
		PosAx := AsPositioningAxisRef(Axes[_FIRST_AXIS#FIRST_AXIS].PosAx);
       	IF PosAx^.VirtualAxis.Mode = UDINT#0 AND pm[i].ma_Asse_Abilitato=TRUE THEN;
           IF PosAx^.Sensor[1].Existent =true THEN //Statussensor[4].State = 2 THEN //StatusSensor[1..4].Adjusted
            IF PosAx^.Sensor[2]._Interface.AddressIn.DB_NUMBER > UINT#0 THEN
              IF pm[i].ma_asse_installato= NON_DEACTIVATABLE_AXIS THEN
                TestDeactivationAxesState:=FALSE;
                ErroreAsse:=i;
                EXIT;
              END_IF;
              IF VerificaPm105=TRUE THEN
                //Se richiesta verifica la compatibilità fra lo stato del
                //motore previsto negli azionamenti e quanto trasferito da VGP
                CASE pm[i].ma_asse_installato OF
					NON_DEACTIVATABLE_AXIS:
                    IF i_sys_MotorState[i]<>AXIS_ACTIVE THEN
						TestDeactivationAxesState:=FALSE;
                      ErroreAsse:=i;
                      EXIT;
                    END_IF;
					ACTIVE_DEACTIVATABLE_AXIS:
                    IF i_sys_MotorState[i]<>AXIS_ACTIVE THEN
						TestDeactivationAxesState:=FALSE;
                      ErroreAsse:=i;
                      EXIT;
                    END_IF;
					INACTIVE_DEACTIVATABLE_AXIS:
                    IF i_sys_MotorState[i]=AXIS_ACTIVE THEN
						TestDeactivationAxesState:=FALSE;
                      ErroreAsse:=i;
                      EXIT;
                    END_IF;
                END_CASE;
              END_IF;
            END_IF;
          END_IF;
        END_IF;
      END_FOR;
   (*  {
    #endif
    }*)

    ;
  END_IF;
END_FUNCTION

/*********************************
 * RESET POWER SUPPLY SMART_LINE *
 *********************************/

FUNCTION ResetSmartLine

	VAR_EXTERNAL CONSTANT
		OUT_RESET_POWER_SUPPLY				:INT;
	END_VAR

	VAR
		dret	:DINT;
	END_VAR
//   {
//   #ifndef ALIMENTATORE_ACTIVE_LINE
//   }
    {
    #ifdef PROFINET
    }
      WriteDigitalOutput(NumOutput:=OUT_RESET_POWER_SUPPLY, Value:=TRUE);
      //dret:=_waitTime(T#20ms);
      WriteDigitalOutput(NumOutput:=OUT_RESET_POWER_SUPPLY, Value:=FALSE);
    {
    #else	//Fine PROFINET
    }
      (*reset_smart_line:=TRUE;
      dret:=_waitTime(T#20ms);
      reset_smart_line:=FALSE;
    (*{
    #endif	//Fine NO PROFINET
    }*)
  {
  #endif	//Fine NO ALIMENTATORE_ACTIVE_LINE
  }
  ;
END_FUNCTION

/***********************************************************************************
 * VERIFY THAT ALL AXES THAT INVOLVE THE EXTENDED FUNCTIONALITY OF SAFETY HAVE THE *
 *  NECESSARY MECHANICAL DATA (SCREW PITCH, LOAD REVOLUTIONS, MOTOR REVOLUTIONS)   *
 *    CONSISTENT WITH THOSE OF THE ASSOCIATED TECHNOLOGICAL OBJECT'S MECHANICS.    *
 ***********************************************************************************/

FUNCTION MechanicalSafetyTest:INT

	VAR_EXTERNAL
		pm                      	:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
		Axes                        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
		ExtendedSafetyState 	    :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;
		AxisType          	        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF INT;            
		LoadRpmAz      	            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
		ScrewPitchAZ       	        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
		EngineRpmAz      	        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
	END_VAR

	VAR
		ScrewPitchTo					:REAL;
		LoadRevolutionsTo				:REAL;
		MotorRevolutionsTo				:REAL;
		TotalRatioTo					:REAL;
		TotalRatioAz					:REAL;
		Address	 						:DINT;
		Axis							:INT;
		PosAx							:REF_TO TO_PositioningAxis;
	END_VAR

	MechanicalSafetyTest:=-1;
	FOR Axis :=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1 BY 1 DO
		PosAx := AsPositioningAxisRef(Axes[Axis].posax);
		IF CheckIfTOactive(Axis:=Axis)=TRUE AND Axes[Axis].IsSiemensAxis THEN
			IF pm[Axis].ma_asse_abilitato=TRUE AND PosAx^.VirtualAxis.Mode <> UDINT#0 THEN
				(*Address:=TO_DINT(PosAx^.Actor._Interface.AddressIn.RID);*)
				IF ExtendedSafetyState[Axis]=TRUE THEN
					//Check if alarm 30711 or 1711 is present, because in such a case it is necessary to execute the activation of safety data
					//Verification of the consistency of the screw pitch parameters, only for linear axes.
					IF AxisType[Axis]=0 THEN
						ScrewPitchTo:=TO_REAL(PosAx^.Mechanics.LeadScrew);
					ELSE
						//Rotating axes do not have a screw pitch.
						ScrewPitchTo:=1;
					END_IF;
					//Verifica della congruenza dei rapporti di riduzione fra TO e parametru safety azionamento
					(*LoadRevolutionsTo:=UDINT_TO_REAL(Axes[Axis].PosAx.setConfigData.TypeOfAxis.NumberOfDataSets.DataSet_1.gear.denFactor);
					MotorRevolutionsTo:=UDINT_TO_REAL(Axes[Axis].PosAx.setConfigData.TypeOfAxis.NumberOfDataSets.DataSet_1.gear.numFactor);*)
					TotalRatioTo:=(MotorRevolutionsTo/LoadRevolutionsTo)*ScrewPitchTo;
					TotalRatioAz:=(EngineRpmAz[Axis]/LoadRpmAz[Axis])*ScrewPitchAZ[Axis];
					IF ABS(TotalRatioTo-TotalRatioAz) > LREAL#0.1 THEN
						MechanicalSafetyTest:=Axis;
						EXIT;
					END_IF;
				END_IF;
			END_IF;
		END_IF;
	END_FOR;
END_FUNCTION

FUNCTION IsOptionalConfig:BOOL

	VAR_INPUT
		port		:INT;
		io_type	:BOOL; //false = input, true = output
	END_VAR

	IsOptionalConfig:=FALSE;
	IF NOT (io_type) THEN
		IF (port=3) OR (port=4) THEN
			IsOptionalConfig:=TRUE;
		END_IF;
	ELSE
		IF (port=3) OR (port=4) OR (port=5) THEN
			IsOptionalConfig:=TRUE;
		END_IF;
	END_IF;
END_FUNCTION

FUNCTION GetPLCIOConfig:SINT

	VAR_EXTERNAL
		IAddress                        	: ARRAY [0..MAXDIGITALINPUT] OF Digital_Input;
		OAddress                        	: ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
	END_VAR

	VAR_EXTERNAL CONSTANT
		MAXDIGITALOUTPUT                    :INT;
		MAXDIGITALINPUT                     :INT;
	END_VAR
	VAR
		i									:INT;
	END_VAR

	GetPLCIOConfig:=SINT#0;
	FOR i:=0 TO MAXDIGITALINPUT -1 DO
		IF (IsOptionalConfig(iaddress[i].Port, BOOL#0)) THEN
			GetPLCIOConfig:=SINT#1;
			EXIT;
		END_IF;
	END_FOR;
	IF (GetPLCIOConfig <> 1) THEN
		FOR i:=0 TO MAXDIGITALOUTPUT -1 DO
			IF (IsOptionalConfig(oaddress[i].Port, BOOL#1)) THEN
				GetPLCIOConfig:=SINT#1;
				EXIT;
			END_IF;
		END_FOR;
	END_IF;
END_FUNCTION

END_NAMESPACE