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

NAMESPACE _AUTO_FC
FUNCTION ResetRc

    VAR_EXTERNAL
         Rc                          :INT;//Riga verso cui ci si sta muovendo
        // {
        // #ifdef DH4010  
        // }
        //   Rc                        :ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;//Riga verso cui ci si sta muovendo
    END_VAR
	// {
	// #ifdef DH4010  
	// }
		// Rc[TABLE#TABLE_LX]:=0;
		// Rc[TABLE#TABLE_RX]:=0;
	// {
	// #else  //Fine DH4010  
	// }
		 Rc:=0;
	// {
	// #endif  //Fine NO DH4010  
	// }
END_FUNCTION
END_NAMESPACE