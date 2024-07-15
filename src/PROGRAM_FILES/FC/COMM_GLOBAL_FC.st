
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

NAMESPACE _COMM_GLOBAL_FC
FUNCTION InvalidaProgrammi

    VAR_EXTERNAL
        Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    END_VAR
	
    VAR
		i	:INT;
	END_VAR

	FOR i:=0 TO TO_INT(_CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS-UINT#1) DO
		// {
		// #ifdef DH4010
		// }
			Programmi[i].NumStati[TABLE#TABLE_LX]:=-1;
			Programmi[i].NumStati[TABLE#TABLE_RX]:=-1;
		// {
		// #else	//Fine DH4010
		// }
		// 	Programmi[i].Numstati:=-1;
		// {
		// #endif	//Fine NO DH4010
		// }
	END_FOR;
END_FUNCTION


END_NAMESPACE