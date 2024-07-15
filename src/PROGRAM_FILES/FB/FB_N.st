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

NAMESPACE FC_N 

FUNCTION_BLOCK _READ_PARAMETER_SINGLE

    Var_external
        Dummy                                         : int; 
    End_var

    VAR_INPUT
        Start                                               :BOOL;
        driveObjectId                                       :UINT;
        hardwareId                                          : word;
        parameterNumber                                     : uint;
        index                                               : uint;
        parameterType                                       :INT; ///0=REAL; 1=DWORD;
    END_VAR
    VAR
        Resourcemanager                                    : OOPLAcycCom_ResourceManager;
        ReadParameterSingle                                : LAcycCom_classReadDriveSingleParams;      
        FirstCycle                                         : Bool := TRUE;
    END_VAR
    VAR_OUTPUT
        Value_REAL                                           : REAL;
        Done                                                 :BOOL;
        Busy                                                 :BOOL;
        error                                                : BOOL;
        diagnostic                                           : LAcycCom_ooptypeDrivediagnostics;
    END_VAR
          
        IF Start  THEN
              ReadParameterSingle.execute();
              Resourcemanager.execute();
              Done :=FALSE;
        ELSE
            error := false;
   //         done := false;
            Busy := false;
        END_IF;

        IF FirstCycle THEN
            ReadParameterSingle.Config(requestBuffer   := Resourcemanager);
            FirstCycle:= FALSE;
        END_IF;

        CASE ReadParameterSingle.Status() OF
            LAcycComstateDef#BUSY :
            busy := TRUE;
            ;
            LAcycComstateDef#IDLE :
                ReadParameterSingle.Start (driveObjectId      := driveObjectId,
                                             hardwareId          := hardwareId,
                                             parameterNumber     := parameterNumber,
                                             index              :=  index);

             LAcycComstateDef#DONE :
             CASE parameterType OF
                0:
                Value_REAL:= ReadParameterSingle.GetValueREAL();
                1:
                Value_REAL:= TO_REAL (ReadParameterSingle.GetValueDWORD());
             END_CASE;

             Done := True;
             Busy := false;

            LAcycComstateDef#ERROR :
                    error := true;
                    busy := false;
                    diagnostic := ReadParameterSingle.errordiagnostics();
                    
        END_CASE;

        (*IF ReadParameterSingle.Status() = LAcycComstateDef#DONE THEN
            ReadParameter.execute();
            CASE ReadParameter.Status() OF
                LAcycComstateDef#BUSY :
                    ;
            
                    LAcycComstateDef#IDLE :
                    datasetitemread.parameterNumber := uint#322;
                    elements := ReadParameter.AddatasetItem(datasetItem := datasetitemread,
                                                            element_no  := -1);
        
                    datasetitemread.parameterNumber := uint#323;
                    elements := ReadParameter.AddatasetItem(datasetItem := datasetitemread,
                                                             element_no := -1);
        
                    ReadParameter.Start(driveObjectId  := uint#3,
                                        hardwareId     := word#266);
        
                LAcycComstateDef#DONE :
                    datasetitemread := ReadParameter.ReaddatasetItem(element_no := 0);
                    RVALUEp322 := datasetitemread.Rvalue;
                    datasetitemread := ReadParameter.ReaddatasetItem(element_no := 1);
                    RVALUEp323 := datasetitemread.Rvalue;
        
                LAcycComstateDef#ERROR :
                    diagnostic := ReadParameter.errordiagnostics();
            END_CASE;
        END_IF;*)
END_FUNCTION_BLOCK

FUNCTION_BLOCK _READ_PARAMETER

    VAR_INPUT
        parameterNumber                                     : ARRAY [0..MaxParameterNumber] of uint;
        Start                                               :BOOL;
        parameterType                                       :INT; ///0=REAL; 1=DWORD;
    END_VAR

    VAR_EXTERNAL CONSTANT
        MaxParameterNumber                                 : uint;
    END_VAR
    
    VAR
        ReadParameter                                      : LAcycCom_classReadDriveParams;
        Resourcemanager                                    : OOPLAcycCom_ResourceManager;
        datasetitemread                                    : LAcycCom_typeDriveDataset;
        elements                                           : int;
        FirstCycle                                         : Bool := TRUE;
    END_VAR

    VAR_OUTPUT
        diagnostic                                              : LAcycCom_ooptypeDrivediagnostics;
        Value_1_REAL                                            : REAL;
        Value_1_DWORD                                           : DWORD;
        Value_2_REAL                                            : REAL;
        Value_2_DWORD                                           : DWORD;
        Done                                                    :BOOL;
        Busy                                                    :BOOL;
        error                                                   : BOOL;
    END_VAR

    IF Start  THEN
        ReadParameter.execute();
        Resourcemanager.execute();
        Done :=FALSE;
    ELSE
      error := false;
      Busy := false;
  END_IF;

  IF FirstCycle THEN
      ReadParameter.Config(requestBuffer   := Resourcemanager);
      FirstCycle:= FALSE;
  END_IF;

    CASE ReadParameter.Status() OF
        LAcycComstateDef#BUSY :
        busy := TRUE;
            
    
            LAcycComstateDef#IDLE :
            datasetitemread.parameterNumber := uint#322;
            elements := ReadParameter.AddatasetItem(datasetItem := datasetitemread,
                                                    element_no  := -1);

            datasetitemread.parameterNumber := uint#323;
            elements := ReadParameter.AddatasetItem(datasetItem := datasetitemread,
                                                     element_no := -1);

            ReadParameter.Start(driveObjectId  := uint#3,
                                hardwareId     := word#266);

        LAcycComstateDef#DONE :
        IF parameterType = 0 THEN
            datasetitemread := ReadParameter.ReaddatasetItem(element_no := 0);
            Value_1_REAL := datasetitemread.Rvalue;
            datasetitemread := ReadParameter.ReaddatasetItem(element_no := 1);
            Value_2_REAL := datasetitemread.Rvalue;
        ELSE
            datasetitemread := ReadParameter.ReaddatasetItem(element_no := 0);
            Value_1_DWORD := datasetitemread.DWvalue;
            datasetitemread := ReadParameter.ReaddatasetItem(element_no := 1);
            Value_2_DWORD := datasetitemread.DWvalue;
        END_IF;

        Done := True;
        Busy := false;

        LAcycComstateDef#ERROR :
            error := true;
            busy := false;
            diagnostic := ReadParameter.errordiagnostics();
    END_CASE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK _WRITE_PARAMETER_SINGL

    VAR_EXTERNAL
        Dummy_LREAL_4                                        :LREAL;
    END_VAR

    VAR_INPUT
        parameterNumber                                     : uint;
        Start                                               :BOOL;
        parameterType                                       :INT; ///0=REAL; 1=DWORD;
        driveObjectId                                       :UINT;
        hardwareId                                          : word;
        index                                               : uint;
        value                                               : REAL;

    END_VAR

    VAR
        WriteParameterSingle                                : LAcycCom_classWriteDriveSingleParams;
        Resourcemanager                                    : OOPLAcycCom_ResourceManager;
        elements                                           : int;
        FirstCycle                                         : Bool := TRUE;
    END_VAR

    VAR_OUTPUT
        Value_REAL                                           : REAL;
        Value_DWORD                                          : DWORD;
        Done                                                 :BOOL;
        Busy                                                 :BOOL;
        error                                                : BOOL;
        diagnostic                                           : LAcycCom_ooptypeDrivediagnostics;
    END_VAR

    IF Start  THEN
        Resourcemanager.execute();
        WriteParameterSingle.execute();
        Done :=FALSE;
    ELSE
      error := false;
      Busy := false;
  END_IF;

  IF FirstCycle THEN
    
    WriteParameterSingle.Config(requestBuffer   := Resourcemanager);
    FirstCycle:= FALSE;
  END_IF;

    CASE WriteParameterSingle.Status() OF
        LAcycComstateDef#BUSY :
            busy := TRUE;
            Dummy_LREAL_4 :=1;  
             
            LAcycComstateDef#IDLE :
            Dummy_LREAL_4 := 2;

            WriteParameterSingle.Start (driveObjectId        := driveObjectId,
                                         hardwareId          := hardwareId,
                                         parameterNumber     := parameterNumber,
                                         Value                 := value,
                                         index              :=  index);                                 
            
        LAcycComstateDef#DONE :
            Dummy_LREAL_4 := 3;
            Done := True;
            Busy := false;

       LAcycComstateDef#ERROR :
            Dummy_LREAL_4 := 4;
            error := true;
            busy := false;
            diagnostic := WriteParameterSingle.errordiagnostics();
    END_CASE;
END_FUNCTION_BLOCK

END_NAMESPACE