USING System.EdgeDetection;

NAMESPACE _UTILITIES_FB

FUNCTION_BLOCK R_TRIG

VAR_INPUT
    CLK : BOOL;
END_VAR

VAR_OUTPUT
    Q : BOOL;
END_VAR

VAR
    LocRising : RisingEdge;
END_VAR

LocRising (Signal:= CLK,
            Detected=>Q);

END_FUNCTION_BLOCK

FUNCTION_BLOCK F_TRIG

    VAR_INPUT
        CLK : BOOL;
    END_VAR
    
    VAR_OUTPUT
        Q : BOOL;
    END_VAR
    
    VAR
        LocFalling : FallingEdge;
    END_VAR
    
    LocFalling (Signal:= CLK,
                Detected=>Q);
    
    END_FUNCTION_BLOCK

END_NAMESPACE

