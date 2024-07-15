USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING System.BitAccess;
USING ENABLEPROGRAM;
USING ENABLEAX_FUNCTIONS;
USING _STATE_CONST;
USING  _TYPE;
USING _FC_FC; 
USING _FC_IO_FC;
USING EnableAxFunction;
USING _FC_FB;
USING _CONST_VAR_DEF;
USING FC_N;

NAMESPACE ENABLE_FC

/************************************************************************************
 * FOR EACH AXIS THAT USES EXTENDED SAFETY, IT READS THE ENCODER CONFIGURATION DATA *
 *                AND STORES THEM IN APPROPRIATE GLOBAL STRUCTURES.                 *
 ************************************************************************************/
    
FUNCTION_BLOCK FB_ReadSafetyData

    VAR_EXTERNAL
		pm                      	    :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
        ExtendedSafetyState 	        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;
        AxisType          	            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF INT;
        LoadRpmAz      	                :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
        ScrewPitchAZ       	            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
        EngineRpmAz      	            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
        PerformSafetyReset	            :BOOL;
		Start_read						: BOOL;

	END_VAR

	VAR_EXTERNAL CONSTANT
		driveObjectIdConstant               : UINT;
        hardwareIdConstant                  :WORD;
    END_VAR

	VAR
		Axis						:INT;
		StateSafety				    :BYTE;
		NumberOfErrors		        :INT;
		ErrorCode			        :INT;
		i							:INT;
		dret						:DINT;
		
		CURRENT_STATE				:INT:= GENERAL_STATE#STATE_INIT;
		CURRENT_AXIS				:INT;
		LetturaSinamicsREAL	        :_READ_PARAMETER_SINGLE; 
        posax                       :REF_TO TO_SynchronousAxis;
	END_VAR

	VAR_OUTPUT
		OutputState				:INT;
	END_VAR

	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
			CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_LOOP_AXES;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_LOOP_AXES:
			CURRENT_AXIS:=CURRENT_AXIS+1;
            PosAx := AsSynchronousAxisRef(Axes[CURRENT_AXIS].posax);
			IF CURRENT_AXIS<_CONST_VAR_DEF._MAXIS#MAXAXIS THEN
				IF CheckIfTOactive(Axis:=CURRENT_AXIS)=TRUE AND Axes[CURRENT_AXIS].IsSiemensAxis THEN
					IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE AND PosAx^.VirtualAxis.Mode <> UDINT#0 THEN
						//Address:=TO_DINT(PosAx^.Actor._Interface.AddressIn.RID);	
						CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_MAXIMUM_TEMPERATURE;
					END_IF;
				END_IF;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_MAXIMUM_TEMPERATURE:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#604,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				Axes[CURRENT_AXIS].TemperaturaMaxPossibile:=TO_USINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#604,
									index:=UINT#0,
									hardwareId:= word#264);
				LetturaSinamicsREAL.Start :=false;
				(*{#ifdef PROFINET}*)
					CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9398_0;
				(*{#else}*)
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				(*{#endif}*);
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9398_0:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9398,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				Axes[CURRENT_AXIS].DatiSafety.r9398[0]:=TO_UDINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9398,
									index:=UINT#0,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9398_1;
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9398_1:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9398,
									index:=UINT#1,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				Axes[CURRENT_AXIS].DatiSafety.r9398[1]:=TO_UDINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9398,
									index:=UINT#1,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9398_0;
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9728_0:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9728,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				Axes[CURRENT_AXIS].DatiSafety.r9728[0]:=TO_UDINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9728,
									index:=UINT#0,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9728_1;
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9728_1:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9728,
									index:=UINT#1,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				Axes[CURRENT_AXIS].DatiSafety.r9728[1]:=TO_UDINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9728,
									index:=UINT#1,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9728_2;
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9728_2:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9728,
									index:=UINT#2,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				Axes[CURRENT_AXIS].DatiSafety.r9728[2]:=TO_UDINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9728,
									index:=UINT#2,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9798_0;
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9798_0:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9798,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				Axes[CURRENT_AXIS].DatiSafety.r9798:=TO_UDINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9798,
									index:=UINT#0,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9898_0;
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;	
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9898_0:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9898,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				Axes[CURRENT_AXIS].DatiSafety.r9898:=TO_UDINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9898,
									index:=UINT#0,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9601_0;
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;		
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_R9601_0:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9601,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				//StateSafety:=REAL_VALUE_TO_BYTE(LetturaSinamicsREAL.ReturnValue);
				ExtendedSafetyState[Axis]:=GetBit(StateSafety,USINT#2);
				//Axes[CURRENT_AXIS].DatiSafety.r9601:=TO_UDINT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9601,
									index:=UINT#0,
									hardwareId:= word#264);
				IF ExtendedSafetyState[Axis]=TRUE THEN
					CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_NUMBER_ERRORS;
				ELSE
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_NUMBER_ERRORS:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9744,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				NumberOfErrors:=TO_INT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9744,
									index:=UINT#0,
									hardwareId:= word#264);
					IF NumberOfErrors>0 THEN
						CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_LOOP_ERROR_CODES;
						i:=-1;
					ELSE
						CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_AXIS_TYPE;
					END_IF;
			ELSIF LetturaSinamicsREAL.error THEN
				LetturaSinamicsREAL.Start :=false;
			END_IF;	
	
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_LOOP_ERROR_CODES:
			i:=i+1;
			IF i<64 THEN
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_ERROR;
			ELSE
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_AXIS_TYPE;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_ERROR:

			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9747,
									index:=TO_UINT(i),
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				ErrorCode:=TO_INT(LetturaSinamicsREAL.Value_REAL);
				IF ErrorCode=30711 OR ErrorCode=1711 THEN
					PerformSafetyReset:=TRUE;
					CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_AXIS_TYPE;
				ELSE
					CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_LOOP_ERROR_CODES;
				END_IF;
			END_IF;
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_AXIS_TYPE:

			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9502,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				AxisType[CURRENT_AXIS]:=TO_INT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9502,
									index:=UINT#0,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_LOAD_REVOLUTIONS_AZ;
			END_IF;

            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_LOAD_REVOLUTIONS_AZ:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9521,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				LoadRpmAz[CURRENT_AXIS]:=TO_INT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9521,
									index:=UINT#0,
									hardwareId:= word#264);
				IF AxisType[CURRENT_AXIS]=0 THEN
					CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_LEAD_AZ;
				ELSE
					ScrewPitchAZ[CURRENT_AXIS]:=1;
					CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_MOTOR_REVOLUTIONS_AZ;
				END_IF;
			END_IF;		
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_LEAD_AZ:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9520,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				ScrewPitchAZ [CURRENT_AXIS]:=TO_INT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9520,
									index:=UINT#0,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_MOTOR_REVOLUTIONS_AZ;
			END_IF;		
			
            _FB_ReadSafetyData#STATE_READ_SAFETY_DATA_READ_MOTOR_REVOLUTIONS_AZ:
			IF NOT LetturaSinamicsREAL.Done THEN
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9522,
									index:=UINT#0,
									hardwareId:= word#264);
			ELSIF LetturaSinamicsREAL.Done THEN
				EngineRpmAz[CURRENT_AXIS]:=TO_INT(LetturaSinamicsREAL.Value_REAL);
				LetturaSinamicsREAL(Start := Start_read,
									driveObjectId:=driveObjectIdConstant,
									parameterNumber:=UINT#9522,
									index:=UINT#0,
									hardwareId:= word#264);
				CURRENT_STATE:=_FB_ReadSafetyData#STATE_READ_SAFETY_DATA_LOOP_AXES;
			END_IF;		
			
            GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_ReadActivationData

	VAR_EXTERNAL
		i_sys_ReadingSinamicsInUse          :BOOL;
		Start_read							: BOOL;
	END_VAR
	
	VAR_INPUT
		Parameter							:UINT;
		SubIndex							:UINT:=UINT#0;
		_hardwareId							:WORD;
		_driveObjectIdConstant				:UINT;
	END_VAR	
	VAR
		dret								:DINT;
		
		CURRENT_STATE						:INT:=GENERAL_STATE#STATE_INIT;
		LetturaSinamicsREAL					:_READ_PARAMETER_SINGLE;
	END_VAR	
	VAR_OUTPUT
		ReturnValue:REAL;
		OutputState:REAL;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			ReturnValue:=0;
			IF i_sys_ReadingSinamicsInUse=FALSE THEN
				IF NOT LetturaSinamicsREAL.Done THEN
					LetturaSinamicsREAL(Start := Start_read,
										driveObjectId:=_driveObjectIdConstant,
										parameterNumber:=Parameter,
										index:=SubIndex,
										hardwareId:=_hardwareId);
				ELSE
					ReturnValue:=LetturaSinamicsREAL.Value_REAL;
					LetturaSinamicsREAL(Start := Start_read,
										driveObjectId:=_driveObjectIdConstant,
										parameterNumber:=Parameter,
										index:=SubIndex,
										hardwareId:=_hardwareId);
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			END_IF;
			
			GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	
	END_CASE;
	OutputState:=CURRENT_STATE;

END_FUNCTION_BLOCK

END_NAMESPACE