USING _TYPE;
/////////////////////////////////////	PLCOpen

//strutture contenenti i valori di chiamata dei blocchi PLCOpen 
//è necessario avere strutture di appoggio poichè non si può modificare i valori di execute, enable etc fuori dalla chiamata dell^FB.


TYPE


    MoveAbsoluteParamIn:STRUCT
        Position:LREAL;
        Velocity:LREAL:=-1;
        Acceleration:LREAL:=-1;
        Deceleration:LREAL:=-1;
    END_STRUCT;

    MoveAbsoluteParamOut:STRUCT
        Done : BOOL;
        Busy  : BOOL;
        Error : BOOL;
        ErrorID : WORD;
    END_STRUCT;



    MoveRelativeParamIn:STRUCT
        Distance:LREAL;
        Velocity:LREAL:=-1;
        Acceleration:LREAL:=-1;
        Deceleration:LREAL:=-1;
    END_STRUCT;

    MoveRelativeParamOut:STRUCT
        Done : BOOL;
        Busy  : BOOL;
        Error : BOOL;
        ErrorID : WORD;
    END_STRUCT;
    
    ResetAxisParamIn:STRUCT
        Restart:BOOL;
    END_STRUCT;

    ResetAxisParamOut:STRUCT
        Done : BOOL;
        Busy : BOOL;
        Error : BOOL;
        ErrorID : WORD;
    END_STRUCT;

    HomeAxisParamIn:STRUCT
        Position:LREAL;
        Mode:EnumHomingMode;
    END_STRUCT;

    HomeAxisParamOut:STRUCT
        Done : BOOL;
        Busy : BOOL;
        Error : BOOL;
        ErrorID : WORD;
    END_STRUCT;
    
    EnableAxisParam:STRUCT
        Status: BOOL;
        Busy  : BOOL;
        Error : BOOL;
        ErrorID : WORD;
    END_STRUCT;

    StopAxisParamIn:STRUCT
	    Deceleration:LREAL:=-1;
    END_STRUCT;

    StopAxisParamOut:STRUCT
        Acceleration:LREAL:=-1;
        Done : BOOL;
        Busy : BOOL;
        Error : BOOL;
        ErrorID : WORD;
    END_STRUCT;

   MoveVelocityParam:STRUCT
	Execute:BOOL;
	Velocity:LREAL:=-1;
	Acceleration:LREAL:=-1;
	Deceleration:LREAL:=-1;
	Direction:EnumDirection:=EnumDirection#USER_DEFAULT;
    END_STRUCT;

    JogAxisParamIn:STRUCT
        JogForward:BOOL;
        JogBackward:BOOL;
        //ModeInc:BOOL;
        //Increment:LREAL;
        Velocity:LREAL;
        Acceleration:LREAL;
        Deceleration:LREAL;
    END_STRUCT;

    JogAxisParamOut:STRUCT
        InVelocity : BOOL;
        Busy : BOOL;
        Error : BOOL;
        ErrorID : WORD;
    END_STRUCT;

    CamInParam:STRUCT
        // CamTable:_MC_CAM_REF;
        Execute:BOOL;
        MasterOffset:LREAL:=0.0;
        SlaveOffset:LREAL:=0.0;
        MasterScaling:LREAL:=1.0;
        SlaveScaling:LREAL:=1.0;
        MasterAbsolute:BOOL:=TRUE;
        SlaveAbsolute:BOOL:=TRUE;
        CyclicMode:BOOL:=FALSE;
        Mode:EnumSyncProfileReference;
    END_STRUCT;

    CamOutparamIn:STRUCT
        SyncProfileReference:DINT;
    END_STRUCT;

    CamOutparamOut:STRUCT
        Done : BOOL;
        Busy : BOOL;
        Error : BOOL;
        ErrorID : WORD;
    END_STRUCT;

    CamSegmentLimit : STRUCT 
        Segment_Start : LREAL;
        Segment_End    : LREAL;
    END_STRUCT;   
END_Type


////////////////////////////////////	FINE PLCOpen
