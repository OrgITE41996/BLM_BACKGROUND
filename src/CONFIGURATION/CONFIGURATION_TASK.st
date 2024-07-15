USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING System.BitAccess;
USING Siemens.Simatic.S71500.Clocks;
USING ENABLEPROGRAM;
USING BACKGROUND_NAMESPACE;
USING ENABLEAX_FUNCTIONS;
USING EnableAxFunction;
USING  _TYPE;
USING Retain_Namespase;
USING _FC_FB;
USING _FC_IO_FC;
USING _CONST_VAR_DEF;
USING ENABLE_FC;
USING BACKGROUND_NAMESPACE_FB;
USING _UTILITIES_FB;
USING _FB;
USING _PLC_OPEN;
USING _LoadConfigurationTest;



CONFIGURATION _Configuration_Task

    TASK MainCyclicProgram : ProgramCycle;
    // TASK CyclicProgram_2 : ProgramCycle;
    TASK CyclicProgram_3 : ProgramCycle;
    TASK CyclicProgramPLC_Open : ProgramCycle;
    TASK MyStartupTask : Startup;
    TASK MaximumCycleTimeExceeded : MaximumCycleTimeExceeded;

    PROGRAM MyStartupProgram WITH MyStartupTask : LoadConfigurationTest;
    PROGRAM CyclicProgramMain WITH MainCyclicProgram : EnableAX; 
    // PROGRAM Cyclic_2 WITH CyclicProgram_2 : SaveRetain;
    PROGRAM Cyclic_3 WITH CyclicProgram_3 : BACKGROUND;
    PROGRAM TimeErrors WITH MaximumCycleTimeExceeded : HandleTimeErrors;
    PROGRAM PLCOpenProgram WITH CyclicProgramPLC_Open : ExecuteAll;



END_CONFIGURATION

PROGRAM HandleTimeErrors

    VAR_EXTERNAL
        counter:LReal;
    END_VAR
    ;counter:=Counter+1;
END_PROGRAM

Program ExecuteAll
    VAR_EXTERNAL
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
        AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
    END_VAR

    VAR_EXTERNAL CONSTANT
    
        FIRST_AXIS_MASTER				        :INT; 
          
    END_VAR 

    VAR_TEMP
        i	:INT;
    END_VAR


FOR i:=0 TO _MAXIS#MAXAXIS-1 DO 
    Axes[i].MC_Axes.Axes_ctrl(Axis:=Axes[i].PosAx);
    //Axes[i].MC_Axes(Axis:=Axes[i].PosAx);   
END_FOR;
// MC_HomeAllAxis();
// MC_ResetAllAxis();
// MC_PosAllAxis();
//Loc_MC_StopAllAxis();
// MC_JogAllAxis();
// MC_MoveAllAxis();
END_PROGRAM

NAMESPACE _PLC_OPEN

    FUNCTION MC_EnableAllAxis

        VAR_EXTERNAL
            Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
            AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
        END_VAR

        VAR_EXTERNAL CONSTANT
    
            FIRST_AXIS_MASTER				        :INT; 
          
          END_VAR 
        VAR_TEMP
            i	:INT;
        END_VAR
        FOR i:=0 TO _MAXIS#MAXAXIS-1 DO
            Axes[i].MC_Axes.Axes_ctrl(Axis:=Axes[i].PosAx);
        END_FOR;
        FOR i:=0 TO _MAXAXIS_MASTER#MAXAXIS_MASTER-1 DO
            Axes[i].MC_Axes.Axes_ctrl(Axis:=AssiMaster[i].PosAx);
        END_FOR;
    END_FUNCTION

    // FUNCTION_BLOCK MC_StopAllAxis
    //     VAR

    //         LocStopSingleAxis :StopSingleAxis;
    //     END_VAR

    //     VAR_TEMP
    //         i	:INT;
    //     END_VAR
    //     FOR i:=0 TO _MAXIS#MAXAXIS-1 DO
    //         LocStopSingleAxis(i);
    //     END_FOR;
    // END_FUNCTION_BLOCK

    // FUNCTION_BLOCK StopSingleAxis

    //     VAR_INPUT
    //         AxisToManage	:INT;
    //         Stop_master : BOOL;
    //         //PosAx            : DB_ANY;
    //     END_VAR

    //     VAR_EXTERNAL
    //         Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    //         AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
    //     END_VAR
    
    //     VAR_EXTERNAL CONSTANT
    
    //         FIRST_AXIS_MASTER				        :INT; 
          
    //       END_VAR 

    //       VAR_TEMP
    //         Axis : REF_TO TO_PositioningAxis;
    //       END_VAR
    //       ;
    //     //   Axis:= AsPositioningAxisRef(PosAx);
    //     IF Stop_master THEN
    //         Axis:= AsPositioningAxisRef(AssiMaster[0].PosAx);
    //         AssiMaster[0].mcStopAxis(Axis:=Axis^);
    //     ELSE
    //         Axis:= AsPositioningAxisRef(Axes[0].PosAx);
    //         Axes[0].mcStopAxis(Axis:=Axis^);
    //     END_IF;  
    // END_FUNCTION_BLOCK
END_NAMESPACE