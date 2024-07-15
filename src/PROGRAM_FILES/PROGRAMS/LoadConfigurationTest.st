USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING System.BitAccess;
USING Siemens.Simatic.S71500.Clocks;
USING ENABLEPROGRAM;
USING ENABLEAX_FUNCTIONS;
USING EnableAxFunction;
USING  _TYPE;
USING Retain_Namespase;
USING _FC_FB;
USING _FC_IO_FC;
USING _CONST_VAR_DEF;
USING ENABLE_FC;

NAMESPACE _LoadConfigurationTest

PROGRAM LoadConfigurationTest

    VAR_EXTERNAL
        Axes                            		    :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
        pm                                            : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters; 
      
        AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
      END_VAR

      VAR_EXTERNAL CONSTANT
  
          FIRST_AXIS_MASTER				        :INT; 
        
        END_VAR 

////////////[AxesMaster]///////////////
      AssiMaster[0].PosAx:=UINT#200; 

///////////[AxesSynchro]//////////////
      Axes[0].PosAx:=UINT#100;
      Axes[1].PosAx:=UINT#101;
      Axes[2].PosAx:=UINT#102;
      Axes[3].PosAx:=UINT#103;
      Axes[4].PosAx:=UINT#104;
      Axes[5].PosAx:=UINT#105;
      Axes[6].PosAx:=UINT#106;
      Axes[7].PosAx:=UINT#107;
      Axes[8].PosAx:=UINT#108;
      Axes[9].PosAx:=UINT#109;    
      Axes[10].PosAx:=UINT#109; //!!!!!! (ifdef _maxAxsea)
      Axes[11].PosAx:=UINT#109;
      Axes[12].PosAx:=UINT#109;
      Axes[13].PosAx:=UINT#109;
      Axes[14].PosAx:=UINT#109;
      Axes[15].PosAx:=UINT#109;
      Axes[16].PosAx:=UINT#109;

      Axes[0].masteraxis:=UINT#200; 
      Axes[1].masteraxis:=UINT#200; 
      Axes[2].masteraxis:=UINT#200; 
      Axes[3].masteraxis:=UINT#200; 
      Axes[4].masteraxis:=UINT#200; 
      Axes[5].masteraxis:=UINT#200; 
      Axes[6].masteraxis:=UINT#200; 
      Axes[7].masteraxis:=UINT#200; 
      Axes[8].masteraxis:=UINT#200;
      Axes[9].masteraxis:=UINT#200; 
      Axes[10].masteraxis:=UINT#200; 
      Axes[11].masteraxis:=UINT#200; 
      Axes[12].masteraxis:=UINT#200; 
      Axes[13].masteraxis:=UINT#200; 
      Axes[14].masteraxis:=UINT#200; 
      Axes[15].masteraxis:=UINT#200; 
      Axes[16].masteraxis:=UINT#200; 
      
END_PROGRAM
END_NAMESPACE