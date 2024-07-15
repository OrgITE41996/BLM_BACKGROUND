USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING System.Math;
USING System.BitAccess;
USING ENABLEPROGRAM;
USING ENABLEAX_FUNCTIONS;
USING  _TYPE;
USING EnableAxFunction;
USING _CONST_VAR_DEF;
USING _FC_FC;
USING _STATE_CONST;


NAMESPACE _FC_IO_FC

/********************************************************
 * RETURNS THE NUMBER OF STATES OF THE INDICATED TABLE. *
 ********************************************************/

FUNCTION GetNumberOfStates:INT
    
    VAR_EXTERNAL

        Programmi                       :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    END_VAR

    VAR_EXTERNAL CONSTANT
    
    END_VAR

    VAR_INPUT
      Tabella    :SINT;
      Programma  :INT;
    END_VAR
    ;
    (*{
    #ifdef DH4010
    }*)
    // GetNumberOfStates:=Programmi[Programma].numstati[Tabella]-1;%%%%
    (*{
    #else  //Fine DH4010
    }*)
    // GetNumberOfStates:=Programmi[Programma].numstati-1; 
  //  {
  //   #endif  //Fine NO DH4010
  //   }
  END_FUNCTION 

  FUNCTION WriteDigitalOutput

    VAR_EXTERNAL
      SystemStarted                   :BOOL;
      OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
      i_sys_DpCoupler                 :BOOL;
      i_sys_PnCoupler                 :BOOL;
      i_sys_StatoIOAL5                :BOOL;
      i_sys_StatoIOAL5Opzionale       :BOOL;
      i_sys_current_prog	            :INT;
      i_sys_StatusIOoptional          :BOOL;
      i_sys_StatusIOAL42              :BOOL;
      i_sys_StatoFestoOpzionale       :BOOL;
      OutputSenzaIndirizzo            :DINT;
    END_VAR

    VAR_EXTERNAL CONSTANT
      MAXDIGITALOUTPUT                :INT;
      INP_OUT_TIPO_SAFETY					    :SINT;
      ERR_IO_NOT_DEFINED              :DINT;
    END_VAR

    VAR_INPUT
      NumOutput  :DINT;
      Value      :BOOL;
      Verifica   :BOOL:=FALSE;
    END_VAR  

    VAR
      Porta      :DINT;
      Bit        :USINT;
      AppoBit    :DINT;
      AppoValue  :BOOL;
      iret       :INT;
    END_VAR 

    IF NumOutput>=0 AND SystemStarted=TRUE THEN 
      AppoValue:=Value;
      IF OAddress[NumOutput].Port=INP_OUT_TIPO_SAFETY AND OAddress[NumOutput].Bit=INP_OUT_TIPO_SAFETY THEN
        //Uscite del tipo safety non forzabili
        OAddress[NumOutput].Enable_Forzatura:=FALSE;
      ELSE
        IF OAddress[NumOutput].Enable_Forzatura=TRUE THEN
          IF OAddress[NumOutput].StatoPreForzatura=0 THEN
            IF OAddress[NumOutput].Stato=TRUE THEN
              //Stato originale a ON
              OAddress[NumOutput].StatoPreForzatura:=SINT#1;
            ELSE
              //Stato originale a OFF
              OAddress[NumOutput].StatoPreForzatura:=SINT#-1;
            END_IF;    
          END_IF;  
          AppoValue:=OAddress[NumOutput].Stato_Forzatura;
        ELSE
          //Uscita non forzata si ripristina lo stato reale
          IF OAddress[NumOutput].StatoPreForzatura=1 THEN
            AppoValue:=TRUE;
            OAddress[NumOutput].Stato:=TRUE;
            OAddress[NumOutput].StatoPreForzatura:=SINT#0;
          ELSE
            IF OAddress[NumOutput].StatoPreForzatura=-1 THEN
              AppoValue:=FALSE;
              OAddress[NumOutput].Stato:=FALSE;
              OAddress[NumOutput].StatoPreForzatura:=SINT#0;
            END_IF;
          END_IF;
        END_IF;
      END_IF;    
      porta:=OAddress[NumOutput].Port;
      AppoBit:=OAddress[NumOutput].Bit;
      OAddress[NumOutput].stato:=AppoValue;
      IF OAddress[NumOutput].inversione=TRUE THEN
        ///Inversione dello stato da impostare sull^uscita fisica se prevista la proprietà
        IF AppoValue=TRUE THEN
          AppoValue:=FALSE;
        ELSE
          AppoValue:=TRUE;
        END_IF;
      END_IF;
      IF Porta>-1 AND AppoBit>-1 THEN 
        bit:=TO_USINT(AppoBit);
        ScritturaUsciteDpCoupler(UsoDpCoupler:=i_sys_DpCoupler, 
                                 Porta:=porta,
                                 Bit:=bit, 
                                 Stato:=AppoValue, 
                                 Programma:=i_sys_current_prog);
        ScritturaUscitePnCoupler(UsoPnCoupler:=i_sys_PnCoupler,
                                 Porta:=porta, 
                                 Bit:=bit, 
                                 Stato:=AppoValue,
                                 Programma:=i_sys_current_prog);
        ScritturaUsciteSiemensProfinet(Porta:=porta,
                                       Bit:=bit,
                                       Stato:=AppoValue);
        // ScritturaUsciteSiemensProfibus(Porta:=porta,
        //                                Bit:=bit,
        //                                Stato:=AppoValue,
        //                                StatoIOopzionale:=i_sys_StatusIOoptional,
        //                                StatoIOAL42:=i_sys_StatusIOAL42,
        //                                StatoIOAL5:=i_sys_StatoIOAL5,
        // ScritturaUsciteFestoProfinet(Porta:=porta, 
        //                              Bit:=bit,
        //                              Stato:=AppoValue);
      ELSE
        IF Verifica=TRUE THEN
          OutputSenzaIndirizzo:=NumOutput;
          iret:=Push_error(Code:=ERR_IO_NOT_DEFINED, Arresto:=FALSE);
        ELSE
          OutputSenzaIndirizzo:=-1;
        END_IF;
      END_IF;
    END_IF;
  END_FUNCTION  

  FUNCTION set_trace

    VAR_EXTERNAL CONSTANT
            INP_EMERGENCY                       :INT;
           // TRACE DECLARATION (YELLOW MESSAGES) 
           TRC_COPY_RAM_ROM                    :INT; // Waiting for copy from RAM to ROM 
           TRC_CONFIGURATION_CHANGE            :INT; // Configuration change performed, turn off and on the CN 
           TRC_START_CMD                       :INT; 
           TRC_CMD_VGP_AXIS_DEACTIVATION       :INT; 
           TRC_CMD_SETTING_V_HZ                :INT; 
           TRC_RELEASE_EMERGENCY               :INT; 
           TRC_EXECUTE_MOTOR_SETTING           :INT; // Traces from 200 to 299 are to be used for device waiting, BlmMotion displays 
           // the device number by adding it to trace 39 (TRC_DISPOSITIVI_BLMMOTION)
           // the next available trace should be 300;
           TRC_MOTOR_SLIDE_RIGHT               :INT; 
           TRC_MOTOR_SLIDE                     :INT; 
           TRC_MOTOR_SLIDE_LEFT                :INT; 
           TRC_MOTOR_AL5                       :INT; 
           TRC_MOTOR_DXSX_4RUNNER              :INT; 
           TRC_MOTOR_4RTL                      :INT;
           TRC_MOTOR_4RC                       :INT;
           TRC_ACTIVATION_4RFORM               :INT; 
           TRC_WAIT_SYSTEM_STARTUP             :INT; 
           TRC_CMD_DXSX                        :INT; // + Enable DXSX (W4 On), - Disable DXSX (W4 Off) 
           TRC_FLOW_DRILL                      :INT; // + Enable FLOW_DRILL (V2 On), - Disable FLOW_DRILL (V2 Off) 
           TRC_CMD_4RTL                        :INT; // + Enable 4RTL (A1 On, A2 Off), - Disable DXSX (A1 Off, A2 Off) 
           TRC_CMD_4RC                         :INT; // + Enable 4RTC (A1 On, A2 On), - Disable DXSX (A1 Off, A2 Off) 
           TRC_CMD_4RIPC                       :INT;
           TRC_CMD_3RIPC                       :INT; 
           TRC_CMD_U4                          :INT; // + Enable U4, - Disable U4 
           TRC_CMD_U5                          :INT; // + Enable U5, - Disable U5 
           TRC_CMD_L1                          :INT; // + Enable L1, - Disable L1 
           TRC_SET_MACHINE_NAME                :INT; 
           TRC_SET_4RFORM                      :INT; 
           TRC_OPERATION_OK                    :INT; 
           TRC_SETTING_Q5004_DX                :INT; 
           TRC_SETTING_Q5004_SX                :INT; 
           TRC_FUNGO_LOADER                    :INT; 
           TRC_FUNGO_ROBOT                     :INT; 
           TRC_FUNGO_CONTROL_PANEL             :INT; 
           TRC_SET_TILTING_SIDE                :INT; 
           TRC_TILTING_SIDE_NOT_SET            :INT; 
           TRC_FUNGO_CONSOLE                   :INT; 
           TRC_WAIT_FOR_DRIVE_COMMUNICATION    :INT; // Waiting for communication with the drive
           TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION :INT; // Waiting for power supply activation
           TRC_WAIT_FOR_SYSTEM_STARTUP_KEYBOARD :INT; 
           TRC_WAIT_FOR_SYSTEM_STARTUP_PLC_SAFETY :INT; 
           TRC_WAIT_CU32_1                     :INT; 
           TRC_WAIT_CU32_2                     :INT; 
           TRC_WAIT_AFTER_UNCONTROLLED_STOP    :INT;
    END_VAR

    VAR_INPUT
      Code :DINT;      //Codice trace
      Axis :INT:=-1;  // Parametro della TRACE
    END_VAR
   // IF readdigitalinput(NumInput:=INP_EMERGENCY)=FALSE THEN
      CASE Code OF
        //Gestione visualizzazione delle trace che debbono essere visualizzate anche
        //con emergenza premuta
           TRC_COPY_RAM_ROM,
           TRC_CONFIGURATION_CHANGE,            
           TRC_START_CMD,                       
           TRC_CMD_VGP_AXIS_DEACTIVATION,       
           TRC_CMD_SETTING_V_HZ,                
           TRC_RELEASE_EMERGENCY,              
           TRC_EXECUTE_MOTOR_SETTING,
           TRC_MOTOR_SLIDE,           
           TRC_MOTOR_SLIDE_RIGHT,                                    
           TRC_MOTOR_SLIDE_LEFT,                 
           TRC_MOTOR_AL5,                        
           TRC_MOTOR_DXSX_4RUNNER,               
           TRC_MOTOR_4RTL,                      
           TRC_MOTOR_4RC,                       
           TRC_ACTIVATION_4RFORM,               
           TRC_WAIT_SYSTEM_STARTUP,             
           TRC_CMD_DXSX,                      
           TRC_FLOW_DRILL,                      
           TRC_CMD_4RTL, TRC_CMD_4RC, TRC_CMD_4RIPC, TRC_CMD_3RIPC,                       
           TRC_CMD_U4, TRC_CMD_U5,                          
           TRC_CMD_L1,                          
           TRC_SET_MACHINE_NAME,                 
           TRC_SET_4RFORM ,                      
           TRC_OPERATION_OK,                     
           TRC_SETTING_Q5004_DX ,                
           TRC_SETTING_Q5004_SX,                 
           TRC_FUNGO_LOADER ,                    
           TRC_FUNGO_ROBOT,                      
           TRC_FUNGO_CONTROL_PANEL,              
           TRC_SET_TILTING_SIDE,                 
           TRC_TILTING_SIDE_NOT_SET,             
           TRC_FUNGO_CONSOLE,                    
           TRC_WAIT_FOR_DRIVE_COMMUNICATION,    
           TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION , 
           TRC_WAIT_FOR_SYSTEM_STARTUP_KEYBOARD,  
           TRC_WAIT_FOR_SYSTEM_STARTUP_PLC_SAFETY,  
           TRC_WAIT_CU32_1, TRC_WAIT_CU32_2,                      
           TRC_WAIT_AFTER_UNCONTROLLED_STOP: 
           ; //InserimentoTrace(Code:=TO_INT(Code), Axis:=Axis);
      END_CASE;
    //ELSE
      //set_trace1:;
      //InserimentoTrace(Code:=TO_INT(Code), Axis:=Axis);
    //END_IF;
    //End:; 
  END_FUNCTION  

/*****************************************************************************************
 * SETS THE STATUS OF THE OUTPUTS EXPORTED BY THE DPDPCOUPLER MODULE ON PROFIBUS NETWORK *
 *****************************************************************************************/

FUNCTION ScritturaUsciteDpCoupler

  VAR_EXTERNAL
    V_dbcoupler_o  : dbcoupler_o;
    i_sys_DL_PN1	 :BOOL;	//True se è abilitato il modulo Keyence per la lettura dell^angolo del ritorno elastico
  END_VAR

  VAR_INPUT
    UsoDpCoupler	:BOOL;
    Porta         :DINT;
    Bit           :USINT;
    Stato         :BOOL;
    Programma     :INT;
  END_VAR

  VAR
    value : BYTE;
 
   END_VAR

  IF UsoDpCoupler=TRUE THEN
    ImpostaDatiNumericiCoupler(Programma:=Programma);
    CASE porta OF
      900:
      value:=V_dbcoupler_o.address_db_o[0];
      V_dbcoupler_o.address_db_o[0]:=setbit(value, bit, Stato);
      901:
      value:=V_dbcoupler_o.address_db_o[1];
      V_dbcoupler_o.address_db_o[1]:=setbit(value, bit, Stato);
      902:
      value:=V_dbcoupler_o.address_db_o[2];
      V_dbcoupler_o.address_db_o[2]:=setbit(value, bit, Stato);
      903:
      value:=V_dbcoupler_o.address_db_o[3];
      V_dbcoupler_o.address_db_o[3]:=setbit(value, bit, Stato);
      920:
      value:=V_dbcoupler_o.address_db_o[20];
      V_dbcoupler_o.address_db_o[20]:=setbit(value, bit, Stato);
      923:
      value:=V_dbcoupler_o.address_db_o[23];
      V_dbcoupler_o.address_db_o[23]:=setbit(value, bit, Stato);
      942:
      value:=V_dbcoupler_o.address_db_o[42];
      V_dbcoupler_o.address_db_o[42]:=setbit(value, bit, Stato);
      944:
      value:=V_dbcoupler_o.address_db_o[44];
      V_dbcoupler_o.address_db_o[44]:=setbit(value, bit, Stato);
      945:
      value:=V_dbcoupler_o.address_db_o[45];
      V_dbcoupler_o.address_db_o[45]:=setbit(value, bit, Stato);
      946:
      value:=V_dbcoupler_o.address_db_o[46];
      V_dbcoupler_o.address_db_o[46]:=setbit(value, bit, Stato);
      947:
      value:=V_dbcoupler_o.address_db_o[47];
      V_dbcoupler_o.address_db_o[1]:=setbit(value, bit, Stato);
      948:
      value:=V_dbcoupler_o.address_db_o[48];
      V_dbcoupler_o.address_db_o[48]:=setbit(value, bit, Stato);
      949:
      value:=V_dbcoupler_o.address_db_o[49];
      V_dbcoupler_o.address_db_o[49]:=setbit(value, bit, Stato);
      950:
      value:=V_dbcoupler_o.address_db_o[50];
      V_dbcoupler_o.address_db_o[50]:=setbit(value, bit, Stato);
      956:
      value:=V_dbcoupler_o.address_db_o[56];
      V_dbcoupler_o.address_db_o[56]:=setbit(value, bit, Stato);
      957:
      value:=V_dbcoupler_o.address_db_o[57];
      V_dbcoupler_o.address_db_o[57]:=setbit(value, bit, Stato);
      958:
      value:=V_dbcoupler_o.address_db_o[58];
      V_dbcoupler_o.address_db_o[58]:=setbit(value, bit, Stato);
      959:
      value:=V_dbcoupler_o.address_db_o[59];
      V_dbcoupler_o.address_db_o[59]:=setbit(value, bit, Stato);
      960:
      value:=V_dbcoupler_o.address_db_o[60];
      V_dbcoupler_o.address_db_o[60]:=setbit(value, bit, Stato);
      961:
      value:=V_dbcoupler_o.address_db_o[61];
      V_dbcoupler_o.address_db_o[61]:=setbit(value, bit, Stato);
      962:
      value:=V_dbcoupler_o.address_db_o[62];
      V_dbcoupler_o.address_db_o[62]:=setbit(value, bit, Stato);
      963:
      value:=V_dbcoupler_o.address_db_o[63];
      V_dbcoupler_o.address_db_o[63]:=setbit(value, bit, Stato);
    END_CASE;
		IF i_sys_DL_PN1=FALSE THEN
			CASE porta OF
				952:
        value:=V_dbcoupler_o.address_db_o[52];
        V_dbcoupler_o.address_db_o[52]:=setbit(value, bit, Stato);
				953:
        value:=V_dbcoupler_o.address_db_o[53];
        V_dbcoupler_o.address_db_o[53]:=setbit(value, bit, Stato);
				954:
        value:=V_dbcoupler_o.address_db_o[54];
        V_dbcoupler_o.address_db_o[54]:=setbit(value, bit, Stato);
				955:
        value:=V_dbcoupler_o.address_db_o[55];
        V_dbcoupler_o.address_db_o[55]:=setbit(value, bit, Stato);
			END_CASE;
		END_IF;

  END_IF;
END_FUNCTION 

FUNCTION ImpostaDatiNumericiCoupler

  VAR_EXTERNAL
    V_pncoupler_o  : pncoupler_o;
    V_dbcoupler_o  : dbcoupler_o;
    
    WebApp                                  :WebApplication;
    i_sys_hwOVR                             :LREAL;
    i_sys_OPMODE                            : INT;
    ProTube                                 :Produzione;
    ProgrammiX                              :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS-UINT#1] OF ProgrammaX;
    OPS_Attivo                              :BOOL;  //True gestione speciale pezzo seguente per consentire funzionamento
    i_sys_error                             :DINT;
  END_VAR

  VAR_EXTERNAL CONSTANT
    PROGRAMMA_JOG                      	            :SINT;
  END_VAR

  VAR_INPUT
    Programma       :INT;
  END_VAR
  VAR
    RisultatoByte		:ARRAY[0..15] OF BYTE;
    Appo            :DINT;
    IndiceProgramma	:INT;
  END_VAR    
  //Trasmissione variabile MachineState
  TrasmetteNByte(Variabile:=Protube.MachineState.MachineState,
                 NumeroByte:=USINT#2,
                 RisultatoByte:=RisultatoByte);
  {::ifdef PROFINET}    
  V_pncoupler_o.address_pn_o[21]:=RisultatoByte[0];
  V_pncoupler_o.address_pn_o[22]:=RisultatoByte[1];
  {::endif}  //Fine PROFINET
       
  V_dbcoupler_o.address_db_o[21]:=RisultatoByte[0];
  V_dbcoupler_o.address_db_o[22]:=RisultatoByte[1];
  
  IF Programma>=0 AND Programma <= TO_INT(_MPROGRAMS#MAXPROGRAMS) - 1 THEN

    //////////////////////////////////////////////////////////////////
    //Numero pezzi da fare 

   // TrasmetteNByte(Variabile:=TO_DINT(MIN(DINT#4294967295, (ProgrammiX[Programma].num_pezzi_totali))),


   TrasmetteNByte(Variabile:=MIN(DINT#2147483647, (ProgrammiX[Programma].num_pezzi_totali)),
   NumeroByte:=USINT#4,
                   RisultatoByte:=RisultatoByte);
    {::ifdef PROFINET}    
    V_pncoupler_o.address_pn_o[4]:=RisultatoByte[0];
    V_pncoupler_o.address_pn_o[5]:=RisultatoByte[1];
    V_pncoupler_o.address_pn_o[44]:=RisultatoByte[2];
    V_pncoupler_o.address_pn_o[45]:=RisultatoByte[3];
    {::endif}  //Fine PROFINET
       
    V_dbcoupler_o.address_db_o[4]:=RisultatoByte[0];
    V_dbcoupler_o.address_db_o[5]:=RisultatoByte[1];
    V_dbcoupler_o.address_db_o[44]:=RisultatoByte[2];
    V_dbcoupler_o.address_db_o[45]:=RisultatoByte[3];
    
    //////////////////////////////////////////////////////////////////
    //Numero pezzi fatti 2 byte

    TrasmetteNByte(Variabile:=MIN(DINT#2147483647, (ProgrammiX[Programma].num_pezzi_fatti)),
                   NumeroByte:=USINT#4,
                   RisultatoByte:=RisultatoByte);
    {::ifdef PROFINET}    
    V_pncoupler_o.address_pn_o[6]:=RisultatoByte[0];
    V_pncoupler_o.address_pn_o[7]:=RisultatoByte[1];
    V_pncoupler_o.address_pn_o[46]:=RisultatoByte[2];
    V_pncoupler_o.address_pn_o[47]:=RisultatoByte[3];
    {::endif}  //Fine NO PROFINET
       
    V_dbcoupler_o.address_db_o[8]:=RisultatoByte[0];
    V_dbcoupler_o.address_db_o[9]:=RisultatoByte[1];
    V_dbcoupler_o.address_db_o[46]:=RisultatoByte[2];
    V_dbcoupler_o.address_db_o[47]:=RisultatoByte[3];
    
    //////////////////////////////////////////////////////////////////
    //Riga attiva della tabella stati 2 byte

    IF (i_sys_OPMODE=PI_COMMANDS_CODE#AUTOMODE AND TestMasterLavoro()=TRUE) OR OPS_Attivo=FALSE THEN
      //Con OPS attivo si valorizza la riga attiva del programma solo quando si esegue il pezzo
      //altrimenti i dati del programma pezzo potrebbero non essere consistenti
      // {
      // #ifdef DH4010  %%%%
      // }
      //   Appo:=Rc[TABELLA_SX];
      // {
      // #else  //Fine DH4010  
      // }
      //   Appo:=Rc;
      // {
      // #endif  //Fine NO DH4010  
      // }
      TrasmetteNByte(Variabile:=Appo,
                     NumeroByte:=USINT#2,
                     RisultatoByte:=RisultatoByte);
      {::ifdef PROFINET}    
      V_pncoupler_o.address_pn_o[8]:=RisultatoByte[0];
      V_pncoupler_o.address_pn_o[9]:=RisultatoByte[1];
      {::endif}  //Fine PROFINET
      
      V_dbcoupler_o.address_db_o[8]:=RisultatoByte[0];
      V_dbcoupler_o.address_db_o[9]:=RisultatoByte[1];
    ELSE
      V_dbcoupler_o.address_db_o[8]:=BYTE#0;
      V_dbcoupler_o.address_db_o[9]:=BYTE#0;
    END_IF;

    //////////////////////////////////////////////////////////////////
    //Numero righe della tabella stati 2 byte

    IF (i_sys_OPMODE=PI_COMMANDS_CODE#AUTOMODE AND TestMasterLavoro()=TRUE) OR OPS_Attivo=FALSE THEN
      //Con OPS attivo si valorizza il numero di righe della tabella stati del programma solo 
      //quando si esegue il pezzo altrimenti i dati del programma pezzo potrebbero non essere 
      //consistenti
      // {
      // #ifdef DH4010  
      // }
      //   Appo:=GetNumStati(Tabella:=TABELLA_SX, Programma:=Programma)+1;
      // {
      // #else  //Fine DH4010  
      // }
      //   Appo:=LREAL_TO_DINT(Programmi[Programma].NumStati)+1;
      // {
      // #endif  //Fine NO DH4010  
      // }
      TrasmetteNByte(Variabile:=Appo,
                     NumeroByte:=USINT#2,
                     RisultatoByte:=RisultatoByte);
      {::ifdef PROFINET}    
      V_pncoupler_o.address_pn_o[10]:=RisultatoByte[0];
      V_pncoupler_o.address_pn_o[11]:=RisultatoByte[1];
      {::endif}  //Fine PROFINET
      
      V_dbcoupler_o.address_db_o[10]:=RisultatoByte[0];
      V_dbcoupler_o.address_db_o[11]:=RisultatoByte[1];
    ELSE
      V_dbcoupler_o.address_db_o[10]:=BYTE#0;
      V_dbcoupler_o.address_db_o[11]:=BYTE#0;
    END_IF;

    //////////////////////////////////////////////////////////////////
    //Numero sequenza attiva 2 byte

    IF (i_sys_OPMODE=PI_COMMANDS_CODE#AUTOMODE AND TestMasterLavoro()=TRUE) OR OPS_Attivo=FALSE THEN
      //Con OPS attivo si valorizza il numero di sequenze del programma solo quando si esegue il pezzo
      //altrimenti i dati del programma pezzo potrebbero non essere consistenti
      // {
      // #ifdef DH4010  %%%%
      // }
      //   Appo:=REAL_TO_DINT(ReadTabStati(Riga:=Rc[TABELLA_SX], 
      //                                   Colonna:=COL_NUM_SEQ, 
      //                                   Programma:=i_sys_current_prog,
      //                                   Tabella:=TABELLA_SX));
      // {
      // #else  //Fine DH4010  
      // }
      //   Appo:=REAL_TO_DINT(ReadTabStati(Riga:=Rc, 
      //                                   Colonna:=COL_NUM_SEQ, 
      //                                   Programma:=i_sys_current_prog,
      //                                   Tabella:=-1));
      // {
      // #endif  //Fine NO DH4010  
      // }

      TrasmetteNByte(Variabile:=Appo,
                     NumeroByte:=USINT#2,
                     RisultatoByte:=RisultatoByte);
      {::ifdef PROFINET}    
      V_pncoupler_o.address_pn_o[12]:=RisultatoByte[0];
      V_pncoupler_o.address_pn_o[13]:=RisultatoByte[1];
      {::endif}  //Fine PROFINET
          
      V_dbcoupler_o.address_db_o[12]:=RisultatoByte[0];
      V_dbcoupler_o.address_db_o[13]:=RisultatoByte[1];
    ELSE
      V_dbcoupler_o.address_db_o[12]:=BYTE#0;
      V_dbcoupler_o.address_db_o[13]:=BYTE#0;
    END_IF;
    
    //////////////////////////////////////////////////////////////////
    //Numero totale sequenze 2 byte
    IF (i_sys_OPMODE=PI_COMMANDS_CODE#AUTOMODE AND TestMasterLavoro()=TRUE) OR OPS_Attivo=FALSE THEN
      //Con OPS attivo si valorizza la sequenza attiva del programma solo quando si esegue il pezzo
      //altrimenti i dati del programma pezzo potrebbero non essere consistenti
      // {
      // #ifdef DH4010  
      // }
      //   Appo:=Programmi[Programma].MaxSequenze[TABELLA_SX];
      // {
      // #else  //Fine DH4010  
      // }
      //   Appo:=Programmi[Programma].MaxSequenze;
      // {
      // #endif  //Fine NO DH4010
      // }
      TrasmetteNByte(Variabile:=Appo,
                     NumeroByte:=USINT#2,
                     RisultatoByte:=RisultatoByte);
      {::ifdef PROFINET}    
      V_pncoupler_o.address_pn_o[14]:=RisultatoByte[0];
      V_pncoupler_o.address_pn_o[15]:=RisultatoByte[1];
      {::endif}  //Fine PROFINET
          
      V_dbcoupler_o.address_db_o[14]:=RisultatoByte[0];
      V_dbcoupler_o.address_db_o[15]:=RisultatoByte[1];
    ELSE
      V_dbcoupler_o.address_db_o[14]:=BYTE#0;
      V_dbcoupler_o.address_db_o[15]:=BYTE#0;
    END_IF;

    //////////////////////////////////////////////////////////////////
    //Codice di errore 4 byte
    Appo:=i_sys_error;
    TrasmetteNByte(Variabile:=Appo,
                   NumeroByte:=USINT#4,
                   RisultatoByte:=RisultatoByte);
    {::ifdef PROFINET}    
    V_pncoupler_o.address_pn_o[16]:=RisultatoByte[0];
    V_pncoupler_o.address_pn_o[17]:=RisultatoByte[1];
    V_pncoupler_o.address_pn_o[18]:=RisultatoByte[2];
    V_pncoupler_o.address_pn_o[19]:=RisultatoByte[3];
    {::endif}  //Fine PROFINET
        
    V_dbcoupler_o.address_db_o[16]:=RisultatoByte[0];
    V_dbcoupler_o.address_db_o[17]:=RisultatoByte[1];
    V_dbcoupler_o.address_db_o[18]:=RisultatoByte[2];
    V_dbcoupler_o.address_db_o[19]:=RisultatoByte[3];
    
    //////////////////////////////////////////////////////////////////
    //Codice del pezzo in conversione ascii
    //Si parte dall^elemento 1 in quanto l^elemento 0 è vuoto
    IF Programma=0 THEN
      IndiceProgramma:=PROGRAMMA_JOG;
    ELSE
      IndiceProgramma:=Programma;
    END_IF;
    {::ifdef PROFINET}    
    V_pncoupler_o.address_pn_o[24]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[1]);
    V_pncoupler_o.address_pn_o[25]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[2]);
    V_pncoupler_o.address_pn_o[26]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[3]);
    V_pncoupler_o.address_pn_o[27]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[4]);
    V_pncoupler_o.address_pn_o[28]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[5]);
    V_pncoupler_o.address_pn_o[29]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[6]);
    V_pncoupler_o.address_pn_o[30]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[7]);
    V_pncoupler_o.address_pn_o[31]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[8]);
    V_pncoupler_o.address_pn_o[32]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[9]);
    V_pncoupler_o.address_pn_o[33]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[10]);
    V_pncoupler_o.address_pn_o[34]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[11]);
    V_pncoupler_o.address_pn_o[35]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[12]);
    V_pncoupler_o.address_pn_o[36]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[13]);
    V_pncoupler_o.address_pn_o[37]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[14]);
    V_pncoupler_o.address_pn_o[38]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[15]);
    V_pncoupler_o.address_pn_o[39]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[16]);
    {::endif}  //Fine PROFINET
        
    V_dbcoupler_o.address_db_o[24]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[1]);
    V_dbcoupler_o.address_db_o[25]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[2]);
    V_dbcoupler_o.address_db_o[26]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[3]);
    V_dbcoupler_o.address_db_o[27]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[4]);
    V_dbcoupler_o.address_db_o[28]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[5]);
    V_dbcoupler_o.address_db_o[29]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[6]);
    V_dbcoupler_o.address_db_o[30]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[7]);
    V_dbcoupler_o.address_db_o[31]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[8]);
    V_dbcoupler_o.address_db_o[32]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[9]);
    V_dbcoupler_o.address_db_o[33]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[10]);
    V_dbcoupler_o.address_db_o[34]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[11]);
    V_dbcoupler_o.address_db_o[35]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[12]);
    V_dbcoupler_o.address_db_o[36]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[13]);
    V_dbcoupler_o.address_db_o[37]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[14]);
    V_dbcoupler_o.address_db_o[38]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[15]);
    V_dbcoupler_o.address_db_o[39]:=TO_BYTE(webapp.pezzo[IndiceProgramma].wd_part_name[16]);
    
    //////////////////////////////////////////////////////////////////
    //Codice trace 2 byte
    TrasmetteNByte(Variabile:=TrovaTrace(), NumeroByte:=USINT#2, RisultatoByte:=RisultatoByte);
    
    {::ifdef PROFINET}    
    V_pncoupler_o.address_pn_o[40]:=RisultatoByte[0];
    V_pncoupler_o.address_pn_o[41]:=RisultatoByte[1];
    {::endif}  //Fine PROFINET
       
    V_dbcoupler_o.address_db_o[40]:=RisultatoByte[0];
    V_dbcoupler_o.address_db_o[41]:=RisultatoByte[1];
    
    //////////////////////////////////////////////////////////////////
		//02/09/2019 A seguito richiesta di FUTABA è stato deciso di esportare il valore dell^override hardware
		//					 anche se la macchina non è stata attivata come master dell^impianto
		
    // {::ifdef PROFINET}  ????L
    // V_pncoupler_o.address_pn_o[43]:=TO_BYTE(i_sys_hwovr);
    // //pncoupler_o43:=LREAL_VALUE_TO_BYTE(i_sys_hwovr);
		// // {::endif}  //Fine PROFINET	    
		// V_dbcoupler_o.address_db_o[43]:=(i_sys_hwovr);
    // //dbcoupler_o43:=LREAL_VALUE_TO_BYTE(i_sys_hwovr);
  END_IF;

END_FUNCTION 

/**************************************
 * CONVERTS AN INTEGER UP TO 16 BYTES *
 **************************************/
FUNCTION TrasmetteNByte

  VAR_INPUT
    Variabile			:DINT;
    NumeroByte		:USINT;
  END_VAR  
  VAR_IN_OUT
    RisultatoByte	:ARRAY[0..15] OF BYTE;
  END_VAR  
  VAR
    NumeroWord		:DWORD;
    ShiftRight  	:WORD;
    i,k						:USINT;
  END_VAR 
  NumeroWord:=TO_DWORD(Variabile);
  k:=USINT#0;
  FOR i:=0 TO NumeroByte-USINT#1 DO
    //ShiftRight:=DWORD_TO_WORD(SHR(in:=NumeroWord, n:=k));
    RisultatoByte[i]:=TO_BYTE(ShiftRight);
    k:=k+USINT#8;
  END_FOR;
END_FUNCTION 

/****************************************************************
 * RETURNS THE NUMBER OF THE FIRST TRACE IN THE LISTTRACE TABLE *
 ****************************************************************/
FUNCTION TrovaTrace:INT

  VAR_EXTERNAL
    i_sys_trace_number              :DINT;
    ElencoTrace                     :ARRAY[0..MAXTRACE-1] OF Trace;
  END_VAR

  VAR_EXTERNAL CONSTANT
    MAXTRACE                              :INT;
  END_VAR

  VAR
    i  :INT;
  END_VAR
  TrovaTrace:=0;
  IF i_sys_trace_number>0 THEN
    FOR i:=0 TO MAXTRACE-1 DO
      IF ElencoTrace[i].Stato>0 THEN
        TrovaTrace:=ElencoTrace[i].codice;
        EXIT;
      END_IF;
    END_FOR;
  END_IF;
END_FUNCTION  

/*****************************************************************************************
 * SETS THE STATUS OF THE OUTPUTS EXPORTED BY THE PNPNCOUPLER MODULE ON PROFINET NETWORK *
 *****************************************************************************************/
FUNCTION ScritturaUscitePnCoupler

  VAR_EXTERNAL
    V_pncoupler_o               : pncoupler_o;

    i_sys_DL_PN1						    :BOOL;
  END_VAR

  VAR_INPUT
    UsoPnCoupler	:BOOL;
    Porta         :DINT;
    bit           :USINT;
    Stato         :BOOL;
    Programma     :INT;
  END_VAR

  VAR
   value : BYTE;

  END_VAR



  {::ifdef PROFINET}

    IF UsoPnCoupler=TRUE THEN
      ImpostaDatiNumericiCoupler(Programma:=Programma);
      CASE porta OF
        900:
        value:=V_pncoupler_o.address_pn_o[0];
        V_pncoupler_o.address_pn_o[0]:=setbit(value, bit, Stato);
        901:
        value:=V_pncoupler_o.address_pn_o[1];
        V_pncoupler_o.address_pn_o[1]:=setbit(value, bit, Stato);
        902:
        value:=V_pncoupler_o.address_pn_o[2];
        V_pncoupler_o.address_pn_o[2]:=setbit(value, bit, Stato);
        903:
        value:=V_pncoupler_o.address_pn_o[3];
        V_pncoupler_o.address_pn_o[3]:=setbit(value, bit, Stato);
        920:
        value:=V_pncoupler_o.address_pn_o[20];
        V_pncoupler_o.address_pn_o[20]:=setbit(value, bit, Stato);
        923:
        value:=V_pncoupler_o.address_pn_o[23];
        V_pncoupler_o.address_pn_o[23]:=setbit(value, bit, Stato);
        942:
        value:=V_pncoupler_o.address_pn_o[42];
        V_pncoupler_o.address_pn_o[42]:=setbit(value, bit, Stato);
        944:
        value:=V_pncoupler_o.address_pn_o[44];
        V_pncoupler_o.address_pn_o[44]:=setbit(value, bit, Stato);
        945:
        value:=V_pncoupler_o.address_pn_o[45];
        V_pncoupler_o.address_pn_o[45]:=setbit(value, bit, Stato);
        946:
        value:=V_pncoupler_o.address_pn_o[46];
        V_pncoupler_o.address_pn_o[46]:=setbit(value, bit, Stato);
        947:
        value:=V_pncoupler_o.address_pn_o[47];
        V_pncoupler_o.address_pn_o[47]:=setbit(value, bit, Stato);
        948:
        value:=V_pncoupler_o.address_pn_o[48];
        V_pncoupler_o.address_pn_o[48]:=setbit(value, bit, Stato);
        949:
        value:=V_pncoupler_o.address_pn_o[49];
        V_pncoupler_o.address_pn_o[49]:=setbit(value, bit, Stato);
        950:
        value:=V_pncoupler_o.address_pn_o[49];
        V_pncoupler_o.address_pn_o[49]:=setbit(value, bit, Stato);
        956:
        value:=V_pncoupler_o.address_pn_o[56];
        V_pncoupler_o.address_pn_o[56]:=setbit(value, bit, Stato);
        957:
        value:=V_pncoupler_o.address_pn_o[57];
        V_pncoupler_o.address_pn_o[57]:=setbit(value, bit, Stato);
        958:
        value:=V_pncoupler_o.address_pn_o[58];
        V_pncoupler_o.address_pn_o[58]:=setbit(value, bit, Stato);
        959:
        value:=V_pncoupler_o.address_pn_o[59];
        V_pncoupler_o.address_pn_o[59]:=setbit(value, bit, Stato);
        960:
        value:=V_pncoupler_o.address_pn_o[60];
        V_pncoupler_o.address_pn_o[60]:=setbit(value, bit, Stato);
        961:
        value:=V_pncoupler_o.address_pn_o[61];
        V_pncoupler_o.address_pn_o[61]:=setbit(value, bit, Stato);
        962:
        value:=V_pncoupler_o.address_pn_o[62];
        V_pncoupler_o.address_pn_o[62]:=setbit(value, bit, Stato);
        963:
        value:=V_pncoupler_o.address_pn_o[63];
        V_pncoupler_o.address_pn_o[63]:=setbit(value, bit, Stato);
      END_CASE;
			IF i_sys_DL_PN1=FALSE THEN
				CASE porta OF
					952:
          value:=V_pncoupler_o.address_pn_o[52];
          V_pncoupler_o.address_pn_o[52]:=setbit(value, bit, Stato);
					953:
          value:=V_pncoupler_o.address_pn_o[53];
          V_pncoupler_o.address_pn_o[53]:=setbit(value, bit, Stato);
					954:
          value:=V_pncoupler_o.address_pn_o[54];
          V_pncoupler_o.address_pn_o[54]:=setbit(value, bit, Stato);
					955:
          value:=V_pncoupler_o.address_pn_o[55];
          V_pncoupler_o.address_pn_o[55]:=setbit(value, bit, Stato);
				END_CASE;
			END_IF;
    END_IF;
  {::endif}  //Fine PROFINET
 
  ;
END_FUNCTION  

// {::ifdef PLC_S7_1500} %%%%
	FUNCTION ScritturaUsciteSiemensProfinet

    VAR_EXTERNAL
      V_ph_o                                  : ph_o;
      V_ph_o_old                              : ph_o_old;
      V_PHOAL5_Opz                            : PHOAL5_Opz;
      V_PHOAL5_Opz_old                        : PHOAL5_Opz_old;
      i_sys_StatoIOAL5                        :BOOL;
      i_sys_StatoIOAL5_old			              :BOOL;
      i_sys_StatoIOAL5Opzionale               :BOOL;
      i_sys_StatoIOAL5Opzionale_old           :BOOL;
    END_VAR

		VAR_INPUT
			Porta								:DINT;
			Bit									:USINT;
			Stato								:BOOL;
		END_VAR

    VAR
      value : BYTE;
   
     END_VAR

		CASE porta OF
			//Macchine seconda serie con profinet
			00:
      value:=V_ph_o.address_ph_o[0];
      V_ph_o.address_ph_o[0]:=setbit(value, bit, Stato);  //32.0÷7
			01:
      value:=V_ph_o.address_ph_o[1];
      V_ph_o.address_ph_o[1]:=setbit(value, bit, Stato);  //33.0÷7
			02:
      value:=V_ph_o.address_ph_o[2];
      V_ph_o.address_ph_o[2]:=setbit(value, bit, Stato);  //34.0÷7
			//Uscite opzionali
			03:
      value:=V_ph_o.address_ph_o[3];
      V_ph_o.address_ph_o[3]:=setbit(value, bit, Stato);  //37.0÷7
			04:
      value:=V_ph_o.address_ph_o[4];
      V_ph_o.address_ph_o[4]:=setbit(value, bit, Stato);  //38.0÷7
			05:
      value:=V_ph_o.address_ph_o[5];
      V_ph_o.address_ph_o[5]:=setbit(value, bit, Stato);  //39.0÷7
		END_CASE;
		{::ifdef AL_BLM}
			IF i_sys_StatoIOAL5_old=TRUE THEN
					//IM 151-3 PN ST (STEP 7) uscite standard AL5
				CASE porta OF
					10:
          value:=V_ph_o_old.address_ph_o_old[0];
          V_ph_o_old.address_ph_o_old[0]:=setbit(value, bit, Stato);  //42.0÷3
					11:
          value:=V_ph_o_old.address_ph_o_old[1];
          V_ph_o_old.address_ph_o_old[1]:=setbit(value, bit, Stato);  //43.0÷3
					12:
          value:=V_ph_o_old.address_ph_o_old[2];
          V_ph_o_old.address_ph_o_old[2]:=setbit(value, bit, Stato);  //44.0÷3
					13:
          value:=V_ph_o_old.address_ph_o_old[3];
          V_ph_o_old.address_ph_o_old[3]:=setbit(value, bit, Stato);  //45.0÷3
					14:
          value:=V_ph_o_old.address_ph_o_old[4];
          V_ph_o_old.address_ph_o_old[4]:=setbit(value, bit, Stato);  //46.0÷3
					15:
          value:=V_ph_o_old.address_ph_o_old[5];
          V_ph_o_old.address_ph_o_old[5]:=setbit(value, bit, Stato);  //47.0÷3
					16:
          value:=V_ph_o_old.address_ph_o_old[6];
          V_ph_o_old.address_ph_o_old[6]:=setbit(value, bit, Stato);  //48.0÷3
				END_CASE;
			END_IF;
			IF i_sys_StatoIOAL5Opzionale_old=TRUE THEN
					//IM 151-3 PN ST (STEP 7) uscite opzionali AL5
				CASE porta OF
					17:
          value:=V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[0];
          V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[0]:=setbit(value, bit, Stato);  //49.0÷3
					18:
          value:=V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[1];
          V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[1]:=setbit(value, bit, Stato);  //50.0÷3
					19:
          value:=V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[2];
          V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[2]:=setbit(value, bit, Stato);  //51.0÷3
					20:
          value:=V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[3];
          V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[3]:=setbit(value, bit, Stato);  //52.0÷3
					21:
          value:=V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[4];
          V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[4]:=setbit(value, bit, Stato);  //53.0÷3
					22:
          value:=V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[5];
          V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[5]:=setbit(value, bit, Stato);  //54.0÷3
					23:
          value:=V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[6];
          V_PHOAL5_Opz_old.address_PHOAL5_Opz_old[6]:=setbit(value, bit, Stato);  //55.0÷3
				END_CASE;
			END_IF;
		
			IF i_sys_StatoIOAL5=TRUE THEN
				//IM 155-6 PN HF (TIA PORTAL) uscite standard AL5
				CASE porta OF
					06:
          value:=V_ph_o.address_ph_o[6];
          V_ph_o.address_ph_o[6]:=setbit(value, bit, Stato);  //42.0÷7
					07:
          value:=V_ph_o.address_ph_o[7];
          V_ph_o.address_ph_o[7]:=setbit(value, bit, Stato);  //43.0÷7
					08:
          value:=V_ph_o.address_ph_o[8];
          V_ph_o.address_ph_o[8]:=setbit(value, bit, Stato);  //44.0÷7
					09:
          value:=V_ph_o.address_ph_o[9];
          V_ph_o.address_ph_o[9]:=setbit(value, bit, Stato);  //45.0÷7
				END_CASE;
			END_IF;
			IF i_sys_StatoIOAL5Opzionale=TRUE THEN
				//IM 155-6 PN HF (TIA PORTAL) uscite opzionali AL5
				CASE porta OF
					15:
          V_PHOAL5_Opz.address_PHOAL5_Opz[0]:=setbit(value, bit, Stato);  //49.0÷7
					16:
          V_PHOAL5_Opz.address_PHOAL5_Opz[1]:=setbit(value, bit, Stato);  //50.0÷7
					17:
          V_PHOAL5_Opz.address_PHOAL5_Opz[2]:=setbit(value, bit, Stato);  //51.0÷7
					18:
          V_PHOAL5_Opz.address_PHOAL5_Opz[3]:=setbit(value, bit, Stato);  //52.0÷7
				END_CASE;
			END_IF;
		{::endif}  //Fine AL_BLM
		
		// {
		// #ifdef TAVOLI_ELETTRICI %%%%
		// }
		// 	CASE porta OF
		// 		24:O_Tavoli_0:=_setbit(O_Tavoli_0, bit, Stato);
		// 		25:O_Tavoli_1:=_setbit(O_Tavoli_1, bit, Stato);
		// 		26:O_Tavoli_2:=_setbit(O_Tavoli_2, bit, Stato);
		// 	END_CASE;  //Fine CASE della porta    
		// {
		// #endif //FINE TAVOLI ELETTRICI
		// }
		{::ifndef EMULAZIONE}
			CASE porta OF
				INDIRIZZO_X142_SIMOTION:
					CASE bit OF
						0:X142_0:=Stato;
						1:X142_1:=Stato;
						2:X142_2:=Stato;
						3:X142_3:=Stato;
						4:X142_4:=Stato;
						5:X142_5:=Stato;
						6:X142_6:=Stato;
						7:X142_7:=Stato;
					END_CASE;
				INDIRIZZO_X132_SINAMICS:
					CASE bit OF
						4:X132_d12:=Stato;
						5:X132_d13:=Stato;
						6:X132_d14:=Stato;
						7:X132_d15:=Stato;
					END_CASE;
			END_CASE;
		{::endif}  //Fine NO EMULAZIONE
		
	END_FUNCTION


FUNCTION_BLOCK FB_ChangeSetParameters
  //Imposta il set di parametri indicato, nell^asse indicato
  
  VAR_EXTERNAL
    Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    i_sys_current_prog	            :INT;
    SystemStarted                   :BOOL;
    pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    TubeIntestate                   :SINT;
    (*{
    #ifdef ORBITALE
    }*)
    ThreadIntext          	        :BOOL; 
    WireSmussed					            :BOOL;
    ChamferPresent                  :BOOL; 
    PosTestWireInHeader 	          :LREAL;
    (*{
    #else  //Fine ORBITALE
    }*)
    i_sys_first_on	                :BOOL;
    i_sys_OPMODE                    : INT;	// MODO OPERATIVO
                                              // 0 = JOG
                                              // 1 = REFERENCE
                                              // 2 = MDA
                                              // 3 = AUTO
  END_VAR
  
  VAR_EXTERNAL CONSTANT
    ERR_INDICATION_DATE_SET             :DINT;
    TRC_WAITING_ACTIVATION_DATASET      :INT;  //Attesa attivazione del dataset desiderato
    PIPE_NO_INTESTATE                   :SINT;
    MY_BIG_VALUE_REAL                   :REAL;
    AUTOMODE                            :INT;
    PROGRAM_JOG                      	  :SINT;
  END_VAR

  VAR_INPUT
    Axis                : DB_ANY;
    NumeroAsse          :INT;
    NumeroSetParametri  :INT;
    ProssimoComando     :EnumNextCommandEnable;
  END_VAR
  
  VAR_TEMP
    i                   :INT;
  END_VAR

  VAR
    PosAx               :REF_TO TO_PositioningAxis;
    dret                :DINT;
    Counter             :INT;
    //Ritorno             :StructRetEncoderValue;
    AsseDaRiattivare    :BOOL;
    QuotaEncoder1       :LREAL;
    QuotaEncoder2       :LREAL;
    Programma           :INT;
    Homing              : Homing;
		QuotaIndex					:LREAL;
		
		CURRENT_STATE				:INT:=GENERAL_STATE#STATE_INIT;
		ResetAxisBLM				:FB_ResetAxisBLM;
		ResetMotionResetAxis:FB_ResetMotionResetAxis;
		DisableAssi					:FB_DisableAxes; 
		//thisCommand					:CommandIDType;
		EliminaCodaComandi	:FB_EliminaCodaComandi;
		EnableSingleAxisBLM	:FB_EnableSingleAxisBLM;
    SetEncoder            : MC_SetSensor;
  END_VAR
	VAR_OUTPUT
		OutputState					:INT;
	END_VAR
  ;
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			Counter:=0;
      PosAx:= AsPositioningAxisRef(Axis);
			IF pm[NumeroAsse].ma_asse_abilitato=TRUE AND SystemStarted=TRUE AND
          GetEncoderNumber (AxisNum:=NumeroAsse)<>TO_UDINT(NumeroSetParametri) AND //Axis.datasetmonitoring.actualdataset<>NumeroSetParametri AND
          PosAx^.Sensor[1].Existent THEN //PosAx^.activeconfigdata.TypeOfAxis.NumberOfDataSets.numberOfDataSets>1 THEN
          IF NumeroSetParametri>4 OR 
                    NumeroSetParametri<1 THEN
            CURRENT_STATE:=GENERAL_STATE#STATE_END;
            dret:=Push_error(code:=ERR_INDICATION_DATE_SET, Asse:=NumeroAsse, Arresto:=FALSE);
          ELSE
              IF VerificationSimulationAxes()=TRUE AND pm[NumeroAsse].ma_num_encs>1 THEN
						(*{#ifdef ORBITALE}*)
                  //CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_SIMUL;%%%%
                (*{#else}*)
                  //Serve per mettere in simulazione gli assi con il secondo esterno vero (booster)
                  SetEncoder.Execute:= TRUE;
                  SetEncoder.Sensor:= NumeroSetParametri;
                  SetEncoder.Mode := 2;

                      IF SetEncoder.Done OR SetEncoder.Error THEN 
                        SetEncoder.Execute:=FALSE;
                        CURRENT_STATE:=GENERAL_STATE#STATE_END;
                      END_IF; 
                    (*{#endif}*)
              ELSE
                  set_trace(Code:=TRC_WAITING_ACTIVATION_DATASET);
                    IF VerificationSimulationAxes()=TRUE THEN
                      CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_SET_DATASET;
                    ELSE
                      (*{#ifdef ORBITALE}*)
                      CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_AXIS;
                      (*{#else}*)
                          SetEncoder.Execute:= TRUE;
                          SetEncoder.Sensor:= NumeroSetParametri;
                          SetEncoder.Mode := 2;

                          IF SetEncoder.Done OR SetEncoder.Error THEN 
                            SetEncoder.Execute:=FALSE;    
                            CURRENT_STATE:=GENERAL_STATE#STATE_END;
                          END_IF; 
                        CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_WAITING_SET_DATASET;
                      (*{#endif}*)
                    END_IF;
                END_IF;
              END_IF;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;

      //Inizio per commente
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_SIMUL:
			(*{#ifdef ORBITALE}*)
          TubeIntestate :=PIPE_NO_INTESTATE;
          ThreadIntext:=FALSE;
          WireSmussed:=FALSE;
          PosTestWireInHeader:=-REAL#99999.0;
          ChamferPresent:=FALSE;
          FOR i:=0 TO TO_INT(_MPROGRAMS#MAXPROGRAMS-UINT#1) DO
            ResetNumberJob(_Program:=i);
          END_FOR;  
          CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_RESET_AXIS;
			(*{#endif};*)
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_RESET_AXIS:
            IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
              ResetMotionResetAxis(Asse:=NumeroAsse);
            ELSE
              ResetMotionResetAxis(Asse:=NumeroAsse);
              CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_DISABLE_AXES;
            END_IF;
            
            _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_DISABLE_AXES:
            IF (DisableAssi.outputState<>GENERAL_STATE#STATE_END) THEN
              DisableAssi(Asse:=NumeroAsse);
            ELSE
              DisableAssi(Asse:=NumeroAsse);
              QuotaEncoder1:=0;
              QuotaEncoder2:=0;
              //thisCommand:=_getCommandID();
              CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_SET_DATASET;
            END_IF;
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_SET_DATASET:
            SetEncoder.Execute:= TRUE;
            SetEncoder.Sensor:= NumeroSetParametri;
            SetEncoder.Mode := 2;

            IF SetEncoder.Done OR SetEncoder.Error THEN 
              SetEncoder.Execute:=FALSE;
              CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_HOMING;
            END_IF; 
			(*dret:=_setAxisDataSetActive(axis:=Asse, 
																	dataSetNumber:=NumeroSetParametri, 
																	nextCommand:=IMMEDIATELY, 
																	commandId:=thisCommand);*)
			IF NOT (dret=7001 OR dret=7002) THEN
				CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_HOMING;
			END_IF;
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_HOMING:
          Homing (Asse:=NumeroAsse, Quota:=0, Modo:=EnumHomingMode#SET_OFFSET_OF_ABSOLUTE_ENCODER_BY_POSITION); 
          SetEncoder.Execute:= TRUE;
          SetEncoder.Sensor:= NumeroSetParametri;
          SetEncoder.Mode := 2;

          IF SetEncoder.Done OR SetEncoder.Error THEN 
            SetEncoder.Execute:=FALSE;
            CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_RESET_ASSE_2;
          END_IF;

			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_RESET_ASSE_2:
          IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
            ResetMotionResetAxis(Asse:=NumeroAsse);
          ELSE
            ResetMotionResetAxis(Asse:=NumeroAsse);
            CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_ENABLE_AXIS;
          END_IF;
        
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_ENABLE_AXIS:
          IF (EnableSingleAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
            EnableSingleAxisBLM((*Asse:=Axis,*)NumeroAsse:=NumeroAsse (*ModoAttesa:=WHEN_COMMAND_DONE*));
           ELSE
            EnableSingleAxisBLM((*Asse:=Axis,*) NumeroAsse:=NumeroAsse (*ModoAttesa:=WHEN_COMMAND_DONE*));
           
             CURRENT_STATE:=GENERAL_STATE#STATE_END;
          END_IF;
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_SET_DATASET:
          SetEncoder.Execute:= TRUE;
          SetEncoder.Sensor:= NumeroSetParametri;
          SetEncoder.Mode := 2;
          IF prossimoComando=EnumNextCommandEnable#WHEN_COMMAND_DONE THEN
            IF SetEncoder.Done OR SetEncoder.Error THEN 
              SetEncoder.Execute:=FALSE;
              CURRENT_STATE:=GENERAL_STATE#STATE_END;
            END_IF;
          ELSE 
            CURRENT_STATE:=GENERAL_STATE#STATE_END;
          END_IF; 
			(*dret:=_setAxisDataSetActive(axis:=Asse, 
																	dataSetNumber:=NumeroSetParametri, 
																	nextCommand:=IMMEDIATELY, 
																	commandId:=thisCommand);
			IF prossimoComando=WHEN_COMMAND_DONE THEN
				IF NOT (dret=7001 OR dret=7002) THEN
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;*)
			
     _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_AXIS:
			(*{#ifdef ORBITALE}*)
				IF TestEncoderEsterno(Asse:=NumeroAsse)=TRUE THEN
					IF i_sys_first_on=FALSE THEN
						//Serve per evitare di perdere la situazione della testa del filo all^avvio della macchina
						TubeIntestate:=PIPE_NO_INTESTATE;
						ThreadIntext:=FALSE;
						PosTestWireInHeader:=-REAL#99999.0;
						ChamferPresent:=FALSE;
						(*{#ifdef DH4010}*)
							IF i_sys_opmode=AUTOMODE THEN
								Programma:=i_sys_current_prog;
							ELSE
								Programma:=PROGRAM_JOG ;
							END_IF;
							IF Programmi[Programma].UsoSoloTestaSx=TRUE THEN
								WireSmussed:=FALSE;
							END_IF;
						(*{#endif}*)  //Fine DH4010
						FOR i:=0 TO TO_INT(_MPROGRAMS#MAXPROGRAMS-UINT#1) DO
							ResetNumberJob(_Program:=i);
						END_FOR;  
						CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_RESET_AXIS;
					END_IF;
				ELSE
          SetEncoder.Execute:= TRUE;
          SetEncoder.Sensor:= NumeroSetParametri;
          SetEncoder.Mode := 2;

          IF SetEncoder.Done OR SetEncoder.Error THEN 
            SetEncoder.Execute:=FALSE;
            CURRENT_STATE:=GENERAL_STATE#STATE_END;
          END_IF; 
        CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_WAITING_SET_DATASET;
        END_IF;
					(*dret:=_setAxisDataSetActive(axis:=Asse, 
																			dataSetNumber:=NumeroSetParametri, 
																			nextCommand:=IMMEDIATELY, 
																			commandId:=_getCommandId());
					CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_WAITING_SET_DATASET;
				END_IF;*)
			(*{#endif};*)
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_RESET_AXIS:
			IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetMotionResetAxis(Asse:=NumeroAsse);
			ELSE
				ResetMotionResetAxis(Asse:=NumeroAsse);
				AsseDaRiattivare:=FALSE;
				(*{#ifndef EMULAZIONE}*)
				  (*IF Axis.driveData.pulsesEnabledState=ACTIVE THEN
						CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_DISABLE_AXIS;
					ELSE

						CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_SET_DATASET;
					END_IF;
				(*{#else}*)
					(*thisCommand:=_getCommandID();*)
					CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_SET_DATASET;
				(*{#endif}*)
			END_IF;
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_DISABLE_AXIS:
			IF (DisableAssi.outputState<>GENERAL_STATE#STATE_END) THEN
				DisableAssi(Asse:=NumeroAsse (*Modo:=DRIVE, ProssimoCmd:=WHEN_COMMAND_DONE, regolatore:=INACTIVE*));
			ELSE
				DisableAssi(Asse:=NumeroAsse (*Modo:=DRIVE, ProssimoCmd:=WHEN_COMMAND_DONE, regolatore:=INACTIVE*));
				AsseDaRiattivare:=TRUE;
				//thisCommand:=_getCommandID();
				CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_SET_DATASET;
			END_IF;
		
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_SET_DATASET:
			QuotaEncoder1:=0;
			QuotaEncoder2:=0;
			(*{#ifdef DH4010}*)
				//La DH4010 non deve perdere l^index di X6 alla partenza, così si prende come valida
				//la quota dell^encoder esterno (il numero 3)
        PosAx:=AsPositioningAxisRef(Axes[NumeroAsse].PosAx);
        QuotaEncoder1:=PosAx^.StatusSensor[3].Position;
        QuotaEncoder2:=PosAx^.StatusSensor[3].Position;

				(*QuotaEncoder1:=Assi[NumeroAsse].PosAx.sensordata[3].position;
				QuotaEncoder2:=Assi[NumeroAsse].PosAx.sensordata[3].position;*)
			(*{#endif}*)  //Fine DH4010*)

        SetEncoder.Execute:= TRUE;
        SetEncoder.Sensor:= NumeroSetParametri;
        SetEncoder.Mode := 2;

        IF SetEncoder.Done OR SetEncoder.Error THEN 
          SetEncoder.Execute:=FALSE;
          IF GetEncoderNumber(AxisNum:=NumeroAsse)=UDINT#1 THEN
            QuotaIndex:=QuotaEncoder2;
          ELSE
            QuotaIndex:=QuotaEncoder1;
          END_IF;
          CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_HOMING;
        END_IF;
			(*dret:=_setAxisDataSetActive(axis:=Asse, 
																	dataSetNumber:=NumeroSetParametri, 
																	nextCommand:=IMMEDIATELY, 
																	commandId:=thisCommand);
			IF NOT (dret=7001 OR dret=7002) THEN*)
				(*IF Asse.datasetmonitoring.actualdataset=1 THEN
					QuotaIndex:=QuotaEncoder2;
				ELSE
					QuotaIndex:=QuotaEncoder1;
				END_IF;*)
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_HOMING:
			Homing(Asse:=NumeroAsse, 
						 Quota:=QuotaIndex, 
						 Modo:=EnumHomingMode#SET_OFFSET_OF_ABSOLUTE_ENCODER_BY_POSITION);
			IF (AsseDaRiattivare=TRUE AND pm[NumeroAsse].ma_asse_abilitato=TRUE) THEN
				CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_RESET_AXIS_2;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_RESET_AXIS_2:
			IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetMotionResetAxis(Asse:=NumeroAsse);
			ELSE
				ResetMotionResetAxis(Asse:=NumeroAsse);
				CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_ENABLE_AXIS;
			END_IF;
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_ENABLE_AXIS:
			IF (EnableSingleAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
				EnableSingleAxisBLM((*Asse:=Axis,*) NumeroAsse:=NumeroAsse (*ModoAttesa:=WHEN_COMMAND_DONE*));
			ELSE
				EnableSingleAxisBLM((*Asse:=Axis,*) NumeroAsse:=NumeroAsse (*ModoAttesa:=WHEN_COMMAND_DONE*));
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;

		//FIne8 per commente
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_WAITING_SET_DATASET:
			IF (prossimoComando=EnumNextCommandEnable#WHEN_COMMAND_DONE) AND GetEncoderNumber(AxisNum:=NumeroAsse)<>TO_UDINT(NumeroSetParametri) THEN
				(*dret:=_setAxisDataSetActive(axis:=Asse, 
																		dataSetNumber:=NumeroSetParametri, 
																		nextCommand:=IMMEDIATELY, 
																		commandId:=_getCommandId());*)
          SetEncoder.Execute:= TRUE;
          SetEncoder.Sensor:= NumeroSetParametri;
          SetEncoder.Mode := 2;

          IF SetEncoder.Done OR SetEncoder.Error THEN 
            SetEncoder.Execute:=FALSE;
                IF counter>500 THEN
                  //Se dopo circa 5 secondi non è ancora stato impostato il corretto dataset
                  //meglio resettare tutto
                  CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_EMERGENCY_RESET;
                  ResetAxisBLM(NumeroAsse, TRUE);
                  EliminaCodaComandi();
                  Counter:=0;
                ELSE
                  Counter:=Counter+1;
                END_IF;
          END_IF;
			ELSE
				  CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_EMERGENCY_RESET:
			IF (ResetAxisBLM.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetAxisBLM(NumeroAsse, TRUE);
			ELSE
				ResetAxisBLM(NumeroAsse, TRUE);
				CURRENT_STATE:=_FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ELIMINATE_QUEUE_COMMANDS;
			END_IF;
			
      _FB_CHANGE_SET_PARAMETERS#STATE_CHANGE_SET_PARAMETERS_ELIMINATE_QUEUE_COMMANDS:
			IF (EliminaCodaComandi.outputState<>GENERAL_STATE#STATE_END) THEN
				EliminaCodaComandi();
			ELSE
				EliminaCodaComandi();
				Counter:=0;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
		
		GENERAL_STATE#STATE_END:
  		reset_trace(Code:=TRC_WAITING_ACTIVATION_DATASET);
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;

	END_CASE;
	OutputState:=CURRENT_STATE;
  SetEncoder(Axis:=PosAx^);
END_FUNCTION_BLOCK
 
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Azzera il numero di tutte le lavorazioni presenti sul pezzo attivo
(*{
#ifdef ORBITALE
}*)
  FUNCTION ResetNumberJob

    VAR_EXTERNAL
      Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    END_VAR

    VAR_INPUT
      _Program  :INT;
    END_VAR
    VAR
      i          :USINT;
    END_VAR
    ;
    FOR i:=0 TO _MAX_TYPES_WORKS#MAX_TYPES_WORKS-Usint#1 DO
      programmi[_Program].DatiLavorazione[i].NumLavExecuted:=0;
    END_FOR;
  END_FUNCTION
(*{
#endif  //Fine ORBITALE
}*)

(*FUNCTION Set_Disp:VOID     
  VAR_INPUT
    Ndisp            :INT; 
    Stato            :INT;
    TimerRisparmio  :BOOL;
  END_VAR
  VAR
    Valore          :REAL;
    dret            :DINT;
  END_VAR
  Disp[Ndisp].StatoRichiesto:=Stato;
  IF Disp[NDisp].Enable=FALSE THEN ;
    oaddress[Ndisp].stato:=FALSE;
    GOTO End;
  END_IF;
  IF TimerRisparmio=TRUE THEN
    StartTimerRisparmioEnergetico(Tempo:=MinutiRisparmioEnergetico);
  END_IF;
  CASE Disp[Ndisp].TipoOut OF
    DISP_OUT_DO:
      IF Disp[Ndisp].spegnimentouscita=TRUE THEN
        IF Disp[Ndisp].Stato=Disp[Ndisp].StatoRichiesto THEN
          GOTO End;
        END_IF;
      END_IF;
      IF Stato=LAVORO THEN
        WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Riposo, Value:=FALSE);
        WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Riposo_2, Value:=FALSE);
        IF DISP[Ndisp].Ritardo_Uscite>0 THEN 
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro, Value:=TRUE);
          dret:=_waitTime(timeValue:=REAL_TO_TIME(DISP[Ndisp].Ritardo_Uscite*1000));
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro_2, Value:=TRUE);
        ELSE
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro_2, Value:=TRUE);
          dret:=_waitTime(timeValue:=REAL_TO_TIME(ABS(DISP[Ndisp].Ritardo_Uscite*1000)));
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro, Value:=TRUE);
        END_IF;
      ELSE
        IF Stato=RIPOSO THEN
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro, Value:=FALSE);
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro_2, Value:=FALSE);
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Riposo, Value:=TRUE);
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Riposo_2, Value:=TRUE);
        END_IF;  
      END_IF;
    DISP_OUT_AO:;
      //Uscita analogica
  END_CASE;
  //Muove solo se non si trova già sullo stato finale    
  IF Stato<>DISP[Ndisp].Stato THEN
    //Doppia uscita digitale
    CASE DISP[Ndisp].TipoIn OF
      DISP_IN_TT:  
        //Tempo lavoro, Tempo riposo
        IF Stato=LAVORO THEN
          IF Disp[NDisp].Stato<>VERSO_LAVORO OR NOT DISP[NDisp].In_Movimento THEN
            DISP[NDisp].Timer:=((DISP[Ndisp].limite_Lavoro-DISP[Ndisp].Position)/DISP[Ndisp].Vel_Lavoro*1000);
          END_IF; 
          Disp[NDisp].Stato:=VERSO_LAVORO; 
          DISP[NDisp].In_Movimento:=TRUE;           
        ELSE
          IF Stato=RIPOSO THEN
            IF Disp[NDisp].Stato<>VERSO_RIPOSO OR NOT DISP[NDisp].In_Movimento THEN
              DISP[NDisp].Timer:=(DISP[Ndisp].Position-DISP[Ndisp].Limite_Riposo)/DISP[Ndisp].Vel_Riposo*1000;
            END_IF; 
            Disp[NDisp].Stato:=VERSO_RIPOSO;  
            DISP[NDisp].In_Movimento:=TRUE; 
          END_IF;
        END_IF;
      DISP_IN_TM:  // 3  - TEMPO LAVORO - MICRO RIPOSO   xxxxxxx0011 
        IF Stato=LAVORO THEN
          IF Disp[NDisp].Stato<>VERSO_LAVORO OR NOT DISP[NDisp].In_Movimento THEN
            DISP[NDisp].Timer:=((DISP[Ndisp].Limite_Lavoro-DISP[Ndisp].Position)/DISP[Ndisp].Vel_Lavoro*1000);
          END_IF; 
          Disp[NDisp].Stato:=VERSO_LAVORO; 
          DISP[NDisp].In_Movimento:=TRUE;           
        ELSE
          IF Stato=RIPOSO THEN
            IF Disp[NDisp].Stato<>VERSO_RIPOSO OR NOT DISP[NDisp].In_Movimento THEN
              DISP[NDisp].Timer:=((DISP[Ndisp].Position-DISP[Ndisp].Limite_Riposo)/DISP[Ndisp].Vel_Lavoro*1000);
            END_IF; 
            DISP[NDisp].In_Movimento:=TRUE;  
            Disp[NDisp].Stato:=VERSO_RIPOSO; 
          END_IF;
        END_IF;
      DISP_IN_MT:  // 5  - MICRO LAVORO - TEMPO RIPOSO   xxxxxxx0101
        IF Stato=LAVORO THEN
          IF Disp[NDisp].Stato<>VERSO_LAVORO OR NOT DISP[NDisp].In_Movimento THEN
            DISP[NDisp].Timer:=((DISP[Ndisp].Limite_Lavoro-DISP[Ndisp].Position)/DISP[Ndisp].Vel_Lavoro*1000);
          END_IF; 
          DISP[NDisp].In_Movimento:=TRUE;  
          Disp[NDisp].Stato:=VERSO_LAVORO;   
         ELSE
          IF Stato=RIPOSO THEN
            IF Disp[NDisp].Stato<>VERSO_RIPOSO OR NOT DISP[NDisp].In_Movimento THEN
              DISP[NDisp].Timer:=(DISP[Ndisp].Position-DISP[Ndisp].Limite_Riposo)/DISP[Ndisp].Vel_Lavoro*1000;
            END_IF; 
              DISP[NDisp].In_Movimento:=TRUE; 
              Disp[NDisp].Stato:=VERSO_RIPOSO;  
            END_IF;
        END_IF;
      DISP_IN_MM:  // 7  - MICRO LAVORO - MICRO RIPOSO   xxxxxxx0111
        DISP[NDisp].In_Movimento:=TRUE; 
        IF Stato=LAVORO THEN
          Disp[NDisp].Stato:=VERSO_LAVORO;
          DISP[NDisp].Timer:=((DISP[Ndisp].Limite_Lavoro-DISP[Ndisp].Position)/DISP[Ndisp].Vel_Lavoro*1000);
        ELSE
          IF Stato=RIPOSO THEN
            Disp[NDisp].Stato:=VERSO_RIPOSO;
            DISP[NDisp].Timer:=(DISP[Ndisp].Position-DISP[Ndisp].Limite_Riposo)/DISP[Ndisp].Vel_Lavoro*1000;
          END_IF;
        END_IF;
       DISP_IN_AI:;  // 9  - INGRESSO ANALOGICO      xxxxxxx1001 
       DISP_IN_V:;  // 16  - INGRESSO VIRTUALE 
       DISP[NDisp].In_Movimento:=TRUE;  
    END_CASE;
    //Incremento del timer del valore del filtro
    DISP[NDisp].Timer:=DISP[NDisp].Timer+(Disp[Ndisp].timerfiltro*1000);
  ELSE
    DISP[NDisp].In_Movimento:=FALSE;  
  END_IF;
  {
  #ifdef ELECTL_ALL
  }
    Set_Disp_ElectL(Ndisp,Stato);
  {
  #endif  //Fine ELECTL_ALL
  }
  {
  #ifdef ELECTXL
  }
    Set_Disp_ElectXL(Ndisp,Stato);
  {
  #endif  //Fine ELECTXL
  }
  End:;
END_FUNCTION*)

FUNCTION TestAxisReady : BOOL/// Check with Luigi

  VAR_EXTERNAL
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
  END_VAR

  VAR_INPUT
    AXIS  :INT;
  END_VAR

  VAR
    PosAxes : REF_TO TO_PositioningAxis;
  END_VAR;


  IF CheckIfTOactive(axis:=axis)=TRUE THEN ;
  PosAxes:=AsPositioningAxisRef(Axes[AXIS].PosAx);
    IF PosAxes^.StatusWord.%X0 =TRUE THEN 
      TestAxisReady:=TRUE;
    ELSE
      TestAxisReady:=FALSE;
    END_IF;
  END_IF;



END_FUNCTION

FUNCTION_BLOCK FB_DisableAxes 

  VAR_EXTERNAL
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    pm                      	      :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    UsoPLCOpen						          :BOOL;
  END_VAR

  VAR_INPUT
    Asse        :INT;
    Bit         :UINT:=UINT#0;

  END_VAR

  VAR
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
  END_VAR

	VAR_OUTPUT
		OutputState	:INT;
	END_VAR

	CASE CURRENT_STATE OF  /// Pensare con MC_Power
		GENERAL_STATE#STATE_INIT:
			IF pm[Asse].ma_Asse_Abilitato=TRUE THEN
          IF UsoPLCOpen THEN
            //PLCOpen
          //Posaxis:=AsPositioningAxisRef(Axes[Asse].PosAx);
          Axes[Asse].MC_Axes.enable_axes :=FALSE;
          CURRENT_STATE:=GENERAL_STATE#STATE_END;
          END_IF;
			END_IF;

			
    GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;


END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_EnableSingleAxisBLM

  VAR_EXTERNAL
		UsoPLCOpen						:BOOL;
		Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
	END_VAR

	VAR_INPUT
		NumeroAsse      :INT;
	END_VAR

	VAR
		dret					    :LREAL;
		CURRENT_STATE	    :INT:=GENERAL_STATE#STATE_INIT;
	END_VAR
	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF UsoPLCOpen THEN
				//PLCOpen
				IF (NumeroAsse>-1) THEN
          Axes[NumeroAsse].MC_Axes.enable_axes := TRUE;
				END_IF;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;

			END_IF;
			
    GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;



END_FUNCTION_BLOCK

///////////////////////////////////////
//True se l^asse indicato ha l^encoder esterno per il conteggio
//dello spostamento del tubo da coil
FUNCTION TestEncoderEsterno:BOOL

  VAR_EXTERNAL CONSTANT
    X6      :INT;
    X1 :    INT;
  END_VAR

  VAR_INPUT
    Asse  :INT;
  END_VAR

  TestEncoderEsterno:=FALSE;
  (*{
  #ifdef ORBITALE
  }
    {
    #ifdef DH4010
    }*)
      IF Asse=X6 THEN
        TestEncoderEsterno:=TRUE;
      END_IF;
   (*{
    #else  //Fine FDH4010
    }*)
      IF Asse=X1 THEN
        TestEncoderEsterno:=TRUE;
      END_IF;
    (*{
    #endif  //Fine NO FDH4010
    }
  {
  #endif  //Fine ORBITALE
  }*)
END_FUNCTION  

/***************************************************************************************************
 * THE FUNCTION SETS THE MINIMUM OR MAXIMUM LIMITS OF THE MACHINE CONSIDERING THE FACT THAT IF THE *
 *              CURRENT POSITION EXCEEDS THE LIMIT, THIS SHOULD BE TAKEN AS THE LIMIT              *
 ***************************************************************************************************/

FUNCTION ImpostaLimiteSW
VAR_EXTERNAL
  Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
END_VAR

  VAR_INPUT
    Asse          :INT;
    ValorePlus    :LREAL:=REAL#99999.0;//MY_BIG_VALUE_REAL;
    ValoreMinus    :LREAL:=REAL#99999.0;//MY_BIG_VALUE_REAL;
  END_VAR
  VAR
    Tolleranza    :LREAL;
    PosizioneAsse :LREAL;
    Esegue        :BOOL;
  END_VAR
  ;
  (*IF Asse>PRIMO_ASSE-1 THEN
    IF TestToAttivo(Asse:=Asse)=TRUE THEN
      Tolleranza:=0;
      {
      #ifdef ORBITALE
      }
        {
        #ifdef DH4010
        }
          //Asse con encoder esterno è X6
          Esegue:=Asse=X6;
        {
        #else  //Fine DH4010
        }
          //Asse con encoder esterno è X1
          Esegue:=Asse=X1;
        {
        #endif  //Fine NO DH4010
        }
        IF Esegue=TRUE THEN
          IF Assi[Asse].PosAx.sensorData[1].position>0 THEN 
            PosizioneAsse:=MAX(Assi[Asse].PosAx.sensorData[1].position, 
                               Assi[Asse].PosAx.sensorData[2].position);
          ELSE
            PosizioneAsse:=MIN(Assi[Asse].PosAx.sensorData[1].position, 
                               Assi[Asse].PosAx.sensorData[2].position);
          END_IF;
        ELSE
          PosizioneAsse:=Assi[Asse].PosAx.positioningState.actualPosition;
        END_IF;
      {
      #else  //Fine ORBITALE
      }
        PosizioneAsse:=Assi[Asse].PosAx.positioningState.actualPosition;
        IF Asse=X1 THEN
          IF i_sys_OpMode=AUTOMODE THEN
            IF ABS(Assi[Asse].PosAx.motionstatedata.commandvelocity)>1 THEN
              //Si allarga la tolleranza per fare in modo che durante il ciclo automatico, l^asse X se si sta muovendo
              //da una posizione in cui l^asse è dentro il limite software, per evitare errori il limite viene impostato
              //con una posizione di 10*EXTRALIMIT oltre la posizione in cui si trova l^asse
              Tolleranza:=10*EXTRALIMIT;
            END_IF;
          END_IF;
        END_IF;
      {
      #endif  //Fine NO ORBITALE
      }
      IF ABS(ValorePlus-MY_BIG_VALUE_REAL)>0.01 THEN
        //Richiesta impostazione limite positivo
        IF PosizioneAsse+Tolleranza<ValorePlus THEN
          //Valore richiesto è OK si imposta se è differente dall^attuale
          IF ABS(Assi[Asse].PosAx.swlimit.plusposition-ValorePlus+Tolleranza)>0.01 THEN
            Assi[Asse].PosAx.swlimit.plusposition:=ValorePlus+Tolleranza;
          END_IF;
        ELSE
          //Asse oltre il limite richiesto imposto la posizione attuale dell^asse
          //con un extra di sicurezza, verificando comunque di non abbassare il 
          //limite massimo già attivo
          Assi[Asse].PosAx.swlimit.plusposition:=MAX(Assi[Asse].PosAx.swlimit.plusposition, PosizioneAsse+EXTRALIMIT);
        END_IF;
      END_IF;
      IF ABS(ValoreMinus-MY_BIG_VALUE_REAL)>0.01 THEN
        //Richiesta impostazione limite negativo
        IF PosizioneAsse>ValoreMinus-Tolleranza THEN
          //Valore richiesto è OK si imposta se è differente dall^attuale
          IF ABS(Assi[Asse].PosAx.swlimit.minusposition-ValoreMinus-Tolleranza)>0.01 THEN
            Assi[Asse].PosAx.swlimit.minusposition:=ValoreMinus-Tolleranza;
          END_IF;
        ELSE
          //Asse oltre il limite richiesto imposto la posizione attuale dell^asse
          //con un extra di sicurezza, verificando comunque di non innalzare il 
          //limite minimo già attivo
          Assi[Asse].PosAx.swlimit.minusposition:=MIN(Assi[Asse].PosAx.swlimit.minusposition,
                                                      PosizioneAsse-EXTRALIMIT);
        END_IF;
      END_IF;
    END_IF;
  END_IF;*)
END_FUNCTION

FUNCTION_BLOCK FB_EliminaCodaComandi

  VAR_EXTERNAL
    Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    Default_Second_Barrier         :BOOL;
    O_Switch_Second_Barrier        :BOOL;
    Strobes_For_Remote             :BOOL;
    FlagEnableCamming			          :BOOL;  //True se è possibile mettere in camming gli assi
    CammeInterpolate                :BOOL; // TRUE se le camme sono già state interpolate
    StatusTrace                     :BOOL;
    StatusWaitingMaterial           :BOOL;
    Status_Waiting_Lubricant        :BOOL;
    SpintaB2Eseguita                :BOOL;
    i_sys_Z2_Roland				          :LREAL;
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
    MemoPosAxisPreViolation         :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
    TimeReadingMotors               :REAL;
    PosX1TubeOnPalette              :LREAL;  //Posizione X1 quando si e^ attivato il tubo su paletta
    EnableTubeOnPalette          :SINT; //0 disabilita nell^IPO la misura X1 per la misura della posizione del tubo
                                                //1 abilita nell^IPO attesa attivazione del sensore tubo su paletta
                                                //2 indica l^avvenuta misura della posizione X1 dove il tubo e^ stato rilevato
                                                //3 indica l^avvenuto arresto dell^asse X1 dopo la misura del terminale del tubo
    LoadRobotParallel                   :BOOL;
    Disp                                :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
    O_ReleaseTube                       :BOOL;
    i_sys_CmdLowerMultiSupport          :SINT;
                (*{
            #ifdef AL_BLM
            }*)
            AL5LoadExecuted				    :BOOL;
            AL42LoadExecuted			        :BOOL;
            AL42PlusLoadExecuted	            :BOOL;
          (*{
          #endif  //Fine AL_BLM
          }*)
    i_sys_AL_Robot            :BOOL;  //True indica che il robot ha completato il prelevio del tubo dall^AL
    OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
    BlockSupport                        :BOOL;
    i_sys_Change_DX_SX                  :BOOL;
    W3ControlTorque ,        
    W4ControlTorque ,    
    W5ControlTorque,        
    W6ControlTorque                     :BOOL;  //True asse in modulazione di coppia durante curvatura 
    i_sys_first_on	                    :BOOL;
    
           (*{
            #ifdef PROFINET
            }*)
              //I nuovi hardware funzionano meglio con il nuovo sistema di reset (tutti gli assi contemporaneamente)
              EnablingOptimizedAxisReset  :BOOL;
           (* {
            #else //Fine PROFINET
            }*)
    pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    AxesResetInProgress                 :DWORD;
    AllargaStandStill                   :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;
    UsoPLCOpen						              :BOOL;
    i_sys_TrackingErrorX1               :REAL;
    i_sys_TrackingErrorX2               :REAL;
    i_sys_CalculationBoundariesInCourseX1	    :BOOL;
    i_sys_CalculationBoundariesInCourseX2	    :BOOL;

                (*{
            #ifdef LINEARIZZA_ASSE
            }*)
            TabLinearizzaAsse		:ARRAY [0.._CONST_VAR_DEF._MAX_AXIS_LINEARISES#MAX_AXIS_LINEARISES-1] OF Linearizza;
            (* {
             #endif  //Fine LINEARIZZA_ASSE
             }*)
    mcReset									:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1-1] OF MC_Reset;

  END_VAR

  VAR_EXTERNAL CONSTANT
                (*{
            #ifdef PLC_S7_1500
            }*)
            TEMPO_ARRESTO_BARRIERA  :REAL;	//Tempo di arresto assi durante violazione barriera di sicurezza
                                                //Se è 0=>Si ferma con il tempo dell^asse più lento
                       (*{
            #endif PLC_S7_1500
            }*)
    PALETTA                           :INT;  //33
    FIRST_DEVICE                      :INT;
    PROGRAM_JOG                      	:SINT;
    MY_BIG_VALUE_REAL                 :REAL;
    AL5_PINZA_DX                      :INT;  //47
    AL5_PINZA_SX                      :INT;  //33
    MAXDIGITALOUTPUT                  :INT;
    OUT_CUTTING_KNIFE_HORIZONTAL        :INT;
		OUT_CUTTING_KNIFE_VERTICAL          :INT;
		OUT_ANTICIPATION_ENTRY_ROBOT_DPCOUPLER         :INT;
    MAX_AXIS_LINEARISES               :INT;
    CLAMP_CARGE                       :INT;  //43
    AT_STATE_UNDEFINED                :SINT;
    U4                                :    INT;  //Mobile slide
    U5                                :INT;   //Extra mobile sled
    X1                                :    INT;    //Cart X
    X2                                :INT;
            (*{
                #ifdef MATRICE_ELETTRICA
                }*)
                // Y2 : INT;	//Matrice
                (*{
                #endif	//Fine MATRICE_ELETTRICA
                }*)
    V4 : INT; //Sollevamento slitta
    INP_PRESENCE_PIECE_1_AL42                       :INT;
    INP_PRESENCE_PIECE_2_AL42                       :INT;
  END_VAR

  VAR_TEMP
    i	      				:INT;
  END_VAR

	VAR
    simotion_plc    : WORD;
    PosAx           : REF_TO TO_PositioningAxis;
		dret    				:DINT;
		dwret		  			:DWORD;
		AssiInizioReset	:DWORD;
		Counter					:USINT;
		
		Attesa								:BOOL;
		//thisCommand						:commandIDType;
		CURRENT_STATE					:INT:=GENERAL_STATE#STATE_INIT;
		CURRENT_AXIS					:INT;
		StopAssiMaster				:FB_StopAssiMaster;
		ResetAllCamme					:FB_ResetAllCamme;
		ResetMotionResetAxis	:FB_ResetMotionResetAxis;
		ResetAssiMaster				:FB_ResetAssiMaster;
		DisableAsseInCoppia		:FB_Disable_Axis_In_Torque;
    StopKinDone           :BOOL;
	END_VAR
	VAR_OUTPUT
		OutputState	:INT;
	END_VAR
  ;
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			(*{#ifdef SELEZIONE_BARRIERA_1_2}*)
				// IF Programmi[PROGRAM_JOG].AbilSelDueBarriere=TRUE AND Default_Second_Barrier=TRUE THEN %%%%
				// 	O_Switch_Second_Barrier:=TRUE;
				// ELSE
				// 	O_Switch_Second_Barrier:=FALSE;
				// END_IF;
			(*{#endif}	//Fine SELEZIONE_BARRIERA_1_2*)
			Strobes_For_Remote:=FALSE;
			FlagEnableCamming:=FALSE;
			CammeInterpolate:=FALSE;
			StatusTrace:=FALSE;
			StatusWaitingMaterial:=FALSE;
			Status_Waiting_Lubricant:=FALSE;
			AssiInizioReset:=DWORD#0;
			SpintaB2Eseguita:=FALSE;
			i_sys_Z2_Roland:=-MY_BIG_VALUE_REAL;
			FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
				Axes[i].statocamming:=FALSE;
			(*{#ifdef CURVATURA}*)
					MemoPosAxisPreViolation[i]:=-MY_BIG_VALUE_REAL;
				(*{#endif}	//Fine CURVATURA*)
			END_FOR;
			TimeReadingMotors:=60;
			PosX1TubeOnPalette:=-MY_BIG_VALUE_REAL;	
			EnableTubeOnPalette:=SINT#0;
			LoadRobotParallel:=FALSE;
			
			(*{#ifdef MS0017}*)
				
			(*{#endif}	//Fine MS0017
			
			{#ifdef SAFETY_INTEGRATED}
				{#ifndef EMULAZIONE}
					//Reset bit richiesta invio comando di reset della safety
          simotion_plc.%X4:= FALSE;
					//simotion_plc:=_setbit(simotion_plc, 4, FALSE);
				(*{#endif}	//Fine NO EMULAZIONE
			{#endif}	//Fine SAFETY_INTEGRATED*)
					
			{::ifdef PROFINET}
				// {#ifndef ORBITALE}
					O_ReleaseTube:=FALSE;
				// {#endif}	//Fine NO ORBITALE
			{::endif}	//Fine PROFINET*)
			
			IF i_sys_cmdlowermultisupport=4 THEN
				//Ripristino del funzionamento del multisostegno se c^è stato una apertura durante
				//il carico di un nuovo tubo
				i_sys_cmdlowermultisupport:=SINT#0;
			END_IF;
			
			{::ifdef AL_BLM}
				AL42PlusLoadExecuted:=FALSE;
				AL42LoadExecuted:=FALSE;
				IF Disp[AL5_PINZA_DX].stato<>DEVICES_STATUS#LAVORO OR Disp[AL5_PINZA_SX].stato<>DEVICES_STATUS#LAVORO THEN
					//Reset flag solo se le pinze non sono chiuse, perchè potrebbe significare
					//che il tubo è presente sull^AL5, e se si resetasse il flag si perderebbe
					//la possibilità di sganciare il caricatore dalla tabella stati una volta
					//completato il carico tubo in macchina
					AL5LoadExecuted:=FALSE;
				END_IF;
				i_sys_AL_Robot:=FALSE;
				//Reset degli ingressi forzati per mantenere lo stato AL42 a 0 (carico)
				DisAbilitaForzaturaInput(Indirizzo:=INP_PRESENCE_PIECE_1_AL42);
				DisAbilitaForzaturaInput(Indirizzo:=INP_PRESENCE_PIECE_2_AL42);
			{::endif}	//Fine AL_BLM*)
			
			{::ifndef ORBITALE} 
				// {::ifdef PROFINET}
					//Per sicurezza si spengono le valvole che comandano la fuoriuscita delle lame
					//si considera meno pericoloso un arretramento
					OAddress[OUT_CUTTING_KNIFE_HORIZONTAL].stato:=FALSE;
					OAddress[OUT_CUTTING_KNIFE_VERTICAL].stato:=FALSE;
				// {#endif}	//Fine PROFINET
			{::endif}	//Fine NO ORBITALE
			OAddress[OUT_ANTICIPATION_ENTRY_ROBOT_DPCOUPLER].stato:=FALSE;
			BlockSupport:=FALSE;
			IF (i_sys_Change_DX_SX)=FALSE THEN
				CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_SET_LINEARIZE_AXES;
			ELSE
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
      _FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_SET_LINEARIZE_AXES:
			(*{#ifdef LINEARIZZA_ASSE}*)
				// FOR i:=0 TO MAX_AXIS_LINEARISES-1 DO %%%%
        //     PosAx:=AsPositioningAxisRef(Axes[0].PosAx);
        //   IF PosAx^.Sensor[4].System <>0 THEN 
				// 	//IF Axes[i].PosAx.servosettings.additionalcommandvalue[0]<>0 THEN
        //   TabLinearizzaAsse[i].OffsetOld:=LREAL_TO_REAL(Axes[i].PosAx.servosettings.additionalcommandvalue[0]);
				// 	//TabLinearizzaAsse[i].OffsetOld:=LREAL_TO_REAL(Axes[i].PosAx.servosettings.additionalcommandvalue[0]);
				// 	END_IF;
				// END_FOR;
			(*{#endif}	//Fine LINEARIZZA_ASSE*)
			W3ControlTorque:=FALSE;
			W4ControlTorque:=FALSE;						
			W5ControlTorque:=FALSE;
			W6ControlTorque:=FALSE;
			CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_SET_STOP_AXIS_MASTER;
			
      _FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_SET_STOP_AXIS_MASTER:
			IF (StopAssiMaster.outputState<>GENERAL_STATE#STATE_END) THEN
				StopAssiMaster(AccelerazioneMaster:=1/TEMPO_ARRESTO_BARRIERA (*ProssimoComando:=IMMEDIATELY*));
			ELSE
				StopAssiMaster(AccelerazioneMaster:=1/TEMPO_ARRESTO_BARRIERA (*  ProssimoComando:=IMMEDIATELY*));
				CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_RESET_AXES_MASTER;
			END_IF;
			
      _FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_RESET_AXES_MASTER:	
			IF (ResetAssiMaster.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetAssiMaster();
			ELSE
				ResetAssiMaster();
				CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_RESET_VECTOR_OBJECTS;
			END_IF;
			
      _FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_RESET_VECTOR_OBJECTS:
			ResetOggettiVettoriali(done=>StopKinDone) ;
			CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
      IF StopKinDone THEN

			CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_STOP_AXIS;
			END_IF;
      _FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_STOP_AXIS:
			CURRENT_AXIS:=CURRENT_AXIS+1;
			IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
				IF TestAsseDaResettare(CURRENT_AXIS) AND pm[CURRENT_AXIS].ma_asse_abilitato THEN
          PosAx:= AsSynchronousAxisRef();
          IF posax^.StatusWord.%X6 THEN
					//IF Axes[CURRENT_AXIS].posax.motionstatedata.motioncommand<>MOTION_DONE THEN
						CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_STOP_SINGLE_AXIS;
					END_IF;
				END_IF;
			ELSE
				CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
				CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_AXIS_RESET;
			END_IF;
			
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_STOP_SINGLE_AXIS:
			StopAxis(Asse:=CURRENT_AXIS);
			CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_STOP_AXIS;
			
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_AXIS_RESET:
			CURRENT_AXIS:=CURRENT_AXIS+1;
			IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
				IF TestAsseDaResettare(CURRENT_AXIS) AND CheckIfTOactive(CURRENT_AXIS) THEN
					//Annulla riscontro fisso solo sugli assi su cui è attivo
          PosAx:= AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx);  
          IF PosAx^.StatusWord.%X26 THEN
					//IF Axes[CURRENT_AXIS].PosAx.movingtoendstopcommand.state=ACTIVE THEN ;
						//thisCommand:=_getCommandID();
						CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_CANCEL_RESET;
					ELSE
            IF PosAx^.StatusTorqueData.CommandAdditiveTorqueActive=1 THEN
						//IF Axes[CURRENT_AXIS].PosAx.torquelimitingcommand.state=ACTIVE THEN
							CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_DISABLE_TORQUE;
						ELSE
							IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE AND NOT PosAx^.StatusWord.%X25 AND NOT PosAx^.StatusWord.%X2 THEN; //Axes[CURRENT_AXIS].posax.simulation<>ACTIVE AND Axes[CURRENT_AXIS].posax.reset<>ACTIVE THEN
								CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_EXECUTE_RESET_AXIS;
							END_IF;
						END_IF;
					END_IF;
				END_IF;
			ELSE
				IF EnablingOptimizedAxisReset=TRUE (*_getStateOfTaskId(_task.StartupTask)=TASK_STATE_STOPPED*) THEN
					CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
					Counter:=USINT#0;
					CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_AXES_IN_RESET;
				ELSE
					CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_WIDEN_TIME_STANDSTILL;
				END_IF;
			END_IF;
			
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_CANCEL_RESET:
			// dret:=_disableMovingToEndStop(axis:=Axes[CURRENT_AXIS].PosAx, %%%%%
			// 															nextCommand:=WHEN_COMMAND_DONE,
			// 															commandId:=thisCommand);
			IF NOT (dret=7001 OR dret=7002) THEN
        PosAx:= AsPositioningAxisRef(Axes[CURRENT_AXIS].PosAx); 
        IF PosAx^.StatusTorqueData.CommandAdditiveTorqueActive=1 THEN 
				//IF Axes[CURRENT_AXIS].PosAx.torquelimitingcommand.state=ACTIVE THEN
					CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_DISABLE_TORQUE;
				ELSE
					IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE AND NOT PosAx^.StatusWord.%X25 AND NOT PosAx^.StatusWord.%X2 THEN //IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE AND Axes[CURRENT_AXIS].posax.simulation<>ACTIVE AND Axes[CURRENT_AXIS].posax.reset<>ACTIVE THEN
						CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_EXECUTE_RESET_AXIS;
					ELSE
						CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_AXIS_RESET;
					END_IF;
				END_IF;
			END_IF;
			
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_DISABLE_TORQUE:
			IF (DisableAsseInCoppia.outputState<>GENERAL_STATE#STATE_END) THEN
				DisableAsseInCoppia(NumeroAsse:=CURRENT_AXIS, Verifica:=FALSE);
			ELSE
				DisableAsseInCoppia(NumeroAsse:=CURRENT_AXIS, Verifica:=FALSE);
				IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE AND NOT PosAx^.StatusWord.%X25 AND NOT PosAx^.StatusWord.%X2 THEN
					CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_EXECUTE_RESET_AXIS;
				ELSE
					CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_AXIS_RESET;
				END_IF;
			END_IF;
			
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_EXECUTE_RESET_AXIS:
			//Si evita di effettuare il reset degli assi non attivi, perchè se sono
			//in simulazione verrebbro ripristinati allo stato normale
			IF EnablingOptimizedAxisReset=TRUE THEN
				//Con il modo reset ottimizzato si attiva il comando senza però attendere la conclusione
				AssiInizioReset:=setBit(AssiInizioReset,TO_USINT(CURRENT_AXIS),TRUE);
				AxesResetInProgress:=setBit(AxesResetInProgress, TO_USINT(CURRENT_AXIS),TRUE);
				Attesa:=FALSE;
			ELSE
				Attesa:=TRUE;
			END_IF;
			IF (ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END) THEN
				ResetMotionResetAxis(Asse:=CURRENT_AXIS, Attesa:=Attesa);
			ELSE
				ResetMotionResetAxis(Asse:=CURRENT_AXIS, Attesa:=Attesa);
				CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_AXIS_RESET;
			END_IF;
			
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_AXES_IN_RESET:
			IF AssiInizioReset>DWORD#0 THEN
				//Con la modalità di reset ottimizzato si attende che tutti gli assi abbiano iniziato il reset
				IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
					//Con la modalità di reset ottimizzato si attende che tutti gli assi abbiano iniziato il reset
					FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 DO
						IF pm[i].ma_asse_abilitato=TRUE THEN
							IF (UsoPLCOpen) THEN
								IF mcReset[i].busy=TRUE THEN
									AssiInizioReset:=setBit(AssiInizioReset, TO_USINT(i),FALSE);
								END_IF;
							ELSE
                PosAx:=AsSynchronousAxisRef(Axes[i].posax);
								IF PosAx^.StatusWord.%X2 THEN
									AssiInizioReset:=setBit(AssiInizioReset, TO_USINT(i),FALSE);
								END_IF;						
							END_IF;
						END_IF;
					END_FOR;
				(* 	{#ifndef PROFINET}
						//Solo le macchine con hardware D435 v1 hanno manifestato stop cpu per superamento
						//tempi del background. Quindi, per il momento, la gestione viene fatta solo per 
						//quel tipo di hardware. Sarà da estendere a tutti i tipi se emergeranno segnalazioni.
						IF AssiInizioReset>0 THEN			
							//Se dopo i tentativi impostati gli assi non si sono portati nello stato di reset
							//si esce dal loop per evitare lo stop della cpu per superamento dei tempi del background.
							//E^ una situazione anomala, probabilmente dovuta ad un continuo flusso di richieste di
							//reset assi che il sistema non risce a gestire. Si assume quindi che è possibile uscire
							//in quanto almeno un reset effettuato in precedenza sia comunque andato a buon fine.
							Counter:=Counter+1;
							//dret:=_waitTime(T#10ms);
							IF Counter>2 THEN
								AssiInizioReset:=0;
							END_IF;
						END_IF;	
					{#endif}	//Fine NO PROFINET*)
				ELSE
					CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_WIDEN_TIME_STANDSTILL;
				END_IF;
			ELSE
				CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_WIDEN_TIME_STANDSTILL;
			END_IF;
			
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_WIDEN_TIME_STANDSTILL:
			// (*{#ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO}*) %%%%
			// 	//Ripristino tolleranza asse fermo originale V4
			// 	AllargaStandStill[V4]:=FALSE;
			// (*{#endif};	//Fine SOLLEVAMENTO_SLITTA_SPALLAMENTO*)
			
			(*{#ifdef SLITTA_MOBILE_DX}*)
				//Ripristino tolleranza asse fermo originale U4
				AllargaStandStill[U4]:=FALSE;
			(*{#endif}	//Fine SLITTA_MOBILE_DX	*)
			
			// (*{#ifdef SLITTA_MOBILE_SX}*) %%%%
			// 	//Ripristino tolleranza asse fermo originale U5
			// 	AllargaStandStill[U5]:=FALSE;
			// (*{#endif}	//SLITTA_MOBILE_SX*)
			
			(*{#ifdef MATRICE_ELETTRICA}*)
				// IF TestAxesY2()=TRUE THEN
					//Ripristino tolleranza asse fermo originale Y2
				// 	AllargaStandStill[Y2]:=FALSE;
				// END_IF;
			(*{#endif}	//MATRICE_ELETTRICA*)
			//Ripristino tolleranza asse fermo originale X1
			AllargaStandStill[X1]:=FALSE;
			(*dwret:=_restartTaskId(_task.MotionTask_TempoStandStill);*)
			CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_RESET_ALL_CAMS;
		
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_RESET_ALL_CAMS:
			IF (ResetAllCamme.outputState<>GENERAL_STATE#STATE_END) THEN
				;// (ProssimoComando:=IMMEDIATELY);
			ELSE
				CURRENT_STATE:=_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_FINAL_SETTINGS;
			END_IF;
			
		_FB_DeleteQueueCommands#STATE_ELIMINATE_QUEUE_COMMANDS_FINAL_SETTINGS:
			(*{#ifdef LINEARIZZA_ASSE} %%%%
				// FOR i:=0 TO MAX_AXIS_LINEARISES-1 DO
				// 	Axes[i].PosAx.servosettings.additionalcommandvalueswitch:=YES;
				// END_FOR;
			(*{#endif}	//Fine LINEARIZZA_ASSE*)
			
			(*{#ifdef AL_BLM}*)
				(*dwret:=_resetTaskId(_task.motiontask_AL);*)
				Disp[CLAMP_CARGE].statorichiesto:=AT_STATE_UNDEFINED;
				// (*{#ifdef MS0017}*) %%%%
				// 	dwret:=_resetTaskId(_task.motiontask_MS0017);
				// (*{#endif}	//Fine MS0017*)
			(*{#endif}	//Fine AL_BLM*)
			(*{#ifdef EFLEX  }*)
				IF i_sys_first_on=FALSE THEN
          PosAx:= AsSynchronousAxisRef(Axes[X1].PosAx);
          PosAx^.FollowingError.MaxValue:= i_sys_TrackingErrorX1;
					//Axes[X1].PosAx.SetConfigData.TypeOfAxis.numberofdatasets.dataset_2.dynamicfollowing.maxpositiontolerance:=i_sys_TrackingErrorX1;
				END_IF;
			(*{#endif}	//Fine EFLEX
			{#ifdef DH4010  }*)
				// IF i_sys_first_on=FALSE THEN
				// 	Axes[X1].PosAx.SetConfigData.TypeOfAxis.numberofdatasets.dataset_1.dynamicfollowing.maxpositiontolerance:=i_sys_TrackingErrorX1;
				// 	Axes[X2].PosAx.SetConfigData.TypeOfAxis.numberofdatasets.dataset_1.dynamicfollowing.maxpositiontolerance:=i_sys_TrackingErrorX2;
				// END_IF;
				// i_sys_CalculationBoundariesInCourseX1:=FALSE;
				// i_sys_CalculationBoundariesInCourseX2:=FALSE;
			(*{#endif}	//Fine DH4010*)
			FOR i:=FIRST_DEVICE TO FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1 DO
				Disp[i].in_movimento:=FALSE;
			END_FOR;
			IF i_sys_first_on=FALSE THEN
				Init_Trace();
			END_IF;
			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
    GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
			
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_StopAssiMaster

  VAR_EXTERNAL
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
  END_VAR

  VAR_EXTERNAL CONSTANT
    X1 :    INT;    //Cart X
    X2      :INT; 
  END_VAR

	VAR_INPUT
		AccelerazioneMaster	:LREAL;
	END_VAR

	VAR
		CURRENT_STATE	:INT:=GENERAL_STATE#STATE_INIT;
		StopMaster		:FB_StopMaster;
	END_VAR

	VAR_OUTPUT
		OutputState		:INT;
	END_VAR

	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF (StopMaster.OutputState<>GENERAL_STATE#STATE_END) THEN
				StopMaster(//NomeMaster:=Axes[X1].masteraxis, 
									 AccelerazioneMaster:=AccelerazioneMaster);
        CURRENT_STATE:=GENERAL_STATE#STATE_END;
			ELSE
				StopMaster(//NomeMaster:=Axes[X1].masteraxis, 
									 AccelerazioneMaster:=AccelerazioneMaster);
				{::ifdef DH4010}
					CURRENT_STATE:=_FB_StopMasterAxes#STATE_STOP_ASSI_MASTER_STOP_MASTER_2;
				{::else}
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				{::endif}
			END_IF;
			
      // _FB_StopMasterAxes#STATE_STOP_ASSI_MASTER_STOP_MASTER_2:
			(*{#ifdef DH4010} %%%%
				// IF (StopMaster.OutputState<>GENERAL_STATE#STATE_END) THEN
				// 	StopMaster(NomeMaster:=Axes[X2].masteraxis,
				// 						 AccelerazioneMaster:=AccelerazioneMaster);
				// ELSE
				// 	StopMaster(NomeMaster:=Axes[X2].masteraxis,
				// 						 AccelerazioneMaster:=AccelerazioneMaster);
				// 	CURRENT_STATE:=GENERAL_STATE#STATE_END;
				// END_IF;
			(*{#endif};*)
			
      GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
			
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

/****************************************************************************************
 *               SETS THE POSITION OF Z TO BE WITHIN THE VALUE -180 +180                *
 *                       THE PRESET IS DONE ONLY ON THE VARIABLE                        *
 *                           POSITIONINGSTATE.ACTUALPOSITION.                           *
 * IT IS NOT AN ACTUAL INDEX BECAUSE THE^OFFSET OF THE ABSOLUTE ENCODER IS NOT CHANGED. *
 ****************************************************************************************/

FUNCTION_BLOCK FB_Form_Axis

  VAR_EXTERNAL
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
  END_VAR

  VAR_EXTERNAL CONSTANT
    Z1                          :    INT;  //Rotation Z
  END_VAR

  VAR_INPUT
    Asse            :DINT;
  END_VAR
 
  VAR
    PosAx           :REF_TO TO_PositioningAxis;
    Homing          :Homing;
    Quota           :LREAL;
    QuotaIntera			:DINT;
    dret            :DINT; 
		
		CURRENT_STATE		:INT:=GENERAL_STATE#STATE_INIT;
  END_VAR
	VAR_OUTPUT
		OutputState			:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			(*{#ifdef ORBITALE}
				{#ifndef THREE_RUNNER_TWO_RUNNER}*)
					IF Asse=Z1 THEN
						//Per le orbitali mai il modulo sull^asse Z1
						CURRENT_STATE:=GENERAL_STATE#STATE_END;
					END_IF;				
				(*{#endif}
			{#endif};*)
			IF CURRENT_STATE<>GENERAL_STATE#STATE_END THEN
				//Se la quota è maggiore di 270 o minore di -270
        PosAx:= AsPositioningAxisRef(Axes[Asse].PosAx);
				IF PosAx^.actualPosition>270 OR
          PosAx^.actualPosition<-270 THEN			
					//Legge la quota dell^asse direttamente dal sensore di posizione
          Quota:=PosAx^.StatusSensor[1].Position;
					//Quota:=Axes[Asse].PosAx.sensordata[1].position;
					QuotaIntera:=TO_DINT(Quota/360);
					Quota:=Quota-(QuotaIntera*360);
					IF Quota>=180 THEN
						Quota:=Quota-360;
					END_IF;			
					Homing (Asse:=TO_INT(Asse), Quota:=Quota, Modo:=EnumHomingMode#DIRECT_HOMING_ABSOLUTE);
					CURRENT_STATE:=_FB_Form_Axis#STATE_MODULE_WAIT_AXIS_HOMED;
				ELSE
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
				END_IF;
			END_IF;
			
    _FB_Form_Axis#STATE_MODULE_WAIT_AXIS_HOMED:

    PosAx:= AsPositioningAxisRef(Axes[Asse].PosAx);
    IF  PosAx^.StatusWord.%X5 Then
			//IF PosAx.positioningstate.homed<>NO THEN
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
	
		General_State#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK  

FUNCTION Init_Disp

  VAR_EXTERNAL
    Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
  END_VAR

  VAR_EXTERNAL CONSTANT
    INTERMEDIATE                    :SINT;
    FIRST_DEVICE                    :INT;
    DISP_IN_V_C :SINT;	//17 - Dispositivo virtuale composto da più dispositivi reali
    DISP_IN_TT  :SINT;		//01 - TEMPO LAVORO - TEMPO RIPOSO   xxxxxxx0001
  END_VAR

  VAR_INPUT
    Ndisp  :INT;
  END_VAR  
  Disp[NDisp].Stato:=INTERMEDIATE;
  Disp[NDisp].position:=(DISP[Ndisp].Limite_Lavoro-DISP[Ndisp].Limite_Riposo)/2;
  DISP[NDisp].In_Movimento:=FALSE;
  AbilitazioneDispositivi(Dispositivo:=NDisp, Stato:=TRUE);
 (* {
  #ifdef EMULAZIONE_INPUT
  }*)
		IF Disp[NDisp].TipoIn<>DISP_IN_V_C THEN
			Disp[NDisp].TipoIn:=DISP_IN_TT;
    END_IF;
	(*{
  #endif  //Fine EMULAZIONE_INPUT
  }*)
END_FUNCTION 

FUNCTION AbilitazioneDispositivi

  VAR_EXTERNAL
    Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
  END_VAR

  VAR_EXTERNAL CONSTANT
    FIRST_DEVICE                    :INT;
  END_VAR

  VAR_INPUT
    Dispositivo    :INT;
    Stato      :BOOL;
  END_VAR  
  Disp[Dispositivo].consenso_minus:=Stato;
  Disp[Dispositivo].consenso_plus:=Stato;
END_FUNCTION  

FUNCTION StartTimerRisparmioEnergetico

  VAR_EXTERNAL
    MinuteSavingEnergy              :INT;
    StatusStandBy                   :BOOL;
    TimerSistema                :ARRAY[0..MAX_TIMER_SISTEMA-1] OF Timer;
  END_VAR

  VAR_EXTERNAL CONSTANT
    T_ENERGY_SAVING    	            :INT;
    MAX_TIMER_SISTEMA 	            :USINT;
  END_VAR

  VAR_INPUT
    Tempo  :INT;
  END_VAR;
  IF MinuteSavingEnergy>0 THEN
    StatusStandBy:=FALSE;
    TimerSistema[T_ENERGY_SAVING].Value:=Tempo*60000.0;
    TimerSistema[T_ENERGY_SAVING].Status:=SINT#1;
  END_IF;
END_FUNCTION

FUNCTION ReadDigitalInput:BOOL

  VAR_EXTERNAL
    SystemStarted                   :BOOL;
    IAddress                        : ARRAY [0..MAXDIGITALINPUT] OF Digital_Input;
    O_EmergencyFromSimotion		      :BOOL;	//Only to be able to compile machines without safety PLCs
    InputWithoutAddress             :DINT;
    i_sys_DpCoupler                 :BOOL;
    i_sys_PnCoupler                 :BOOL;
    i_sys_StatusIOoptional          :BOOL;
    i_sys_StatusIOAL42               :BOOL;
    i_sys_StatusIOAL5                :BOOL;
    i_sys_StatusIOAL5Optional       :BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    MAXDIGITALINPUT                    :INT;
    INP_OUT_TIPO_SAFETY					       :SINT;
    INP_EMERGENCY                      :INT;
    ERR_IO_NOT_DEFINED                 :DINT ; //Manca associazione virtuale/fisico
  END_VAR

  VAR_INPUT
    NumInput  :DINT;
    Verifica  :BOOL:=FALSE;
  END_VAR  

  VAR
    Porta     :DINT;
    Bit       :USINT;
    iret      :INT;
    OldStato  :BOOL;
  END_VAR 

  IF NumInput>=0 AND SystemStarted=TRUE THEN
    IF IAddress[NumInput].Enable_Forzatura=TRUE THEN
      (*{
      #ifdef EMULAZIONE
			}*)  
				IF IAddress[NumInput].Stato_Forzatura=FALSE THEN
					ReadDigitalInput:=FALSE;  
				ELSE
					ReadDigitalInput:=TRUE;
				END_IF;
      (*{
      #else  //Fine EMULAZIONE
      }*)  
				IF IAddress[NumInput].Port=INP_OUT_TIPO_SAFETY OR IAddress[NumInput].Bit=INP_OUT_TIPO_SAFETY THEN
					//Gli ingressi il cui stato è impostato dal plc di sicurezza non si possono forzare, tranne
					//l^ingresso di emergenza che può essere forzato solo basso (per esigenze di teleassistenza).
					IF NumInput=INP_EMERGENCY AND IAddress[NumInput].Enable_Forzatura=TRUE THEN
						IAddress[NumInput].Stato_Forzatura:=FALSE;
						ReadDigitalInput:=FALSE;
						O_EmergencyFromSimotion:=TRUE;
					ELSE
          	IAddress[NumInput].Enable_Forzatura:=FALSE;
					END_IF;
        ELSE
					//Ingressi standard si possono forzare
          IF IAddress[NumInput].STATO_FORZATURA=FALSE THEN
            ReadDigitalInput:=FALSE;  
          ELSE
            ReadDigitalInput:=TRUE;  
          END_IF;
        END_IF;
      (*{
      #endif  //Fine NO EMULAZIONE
      }*) 
    ELSE
      Porta:=IAddress[NumInput].Port;
      Bit:=TO_USINT(IAddress[NumInput].Bit);
      IF IAddress[NumInput].Port=INP_OUT_TIPO_SAFETY AND IAddress[NumInput].Bit=INP_OUT_TIPO_SAFETY THEN
        //Lettura dello stato degli ingressi che vengono valorizzati da informazioni proveniente dal plc di sicurezza
        ReadDigitalInput:=LetturaIngressiSafety(NumeroInput:=NumInput);
      ELSE  
        //Lettura ingressi collegati ai moduli standard e non impostati dal plc di sicurezza
        IF Verifica=TRUE AND (Porta<0 OR Bit<0) THEN
          //Verifica, se necessario, la definizione del collegamento virtuale fisico
          InputWithoutAddress:=NumInput;
          iret:=Push_error(Code:=ERR_IO_NOT_DEFINED, Arresto:=FALSE);
          ReadDigitalInput:=FALSE;  
        ELSE
          OldStato:=FALSE;
          ReadDigitalInput:=LetturaIngressiDpCoupler(OldStato:=OldStato,
                                                     UsoDpCoupler:=i_sys_DpCoupler, 
                                                     Porta:=porta, 
                                                     Bit:=bit);
          OldStato:=ReadDigitalInput;
          ReadDigitalInput:=LetturaIngressiPnCoupler(OldStato:=OldStato,
                                                     UsoPnCoupler:=i_sys_PnCoupler, 
                                                     Porta:=porta, 
                                                     Bit:=bit);
          OldStato:=ReadDigitalInput;
          ReadDigitalInput:=LetturaIngressiSiemensProfinet(OldStato:=OldStato,
                                                           Porta:=porta, 
                                                           Bit:=bit);
          OldStato:=ReadDigitalInput;
          ReadDigitalInput:=LetturaIngressiSiemensProfibus(OldStato:=OldStato,
                                                           Porta:=porta, 
                                                           Bit:=bit, 
                                                           StatoIOopzionale:=i_sys_StatusIOoptional,
                                                           StatoIOAL42:=i_sys_StatusIOAL42,
                                                           StatoIOAL5:=i_sys_StatusIOAL5,
                                                           StatoIOAL5Opzionale:=i_sys_StatusIOAL5Optional);
          OldStato:=ReadDigitalInput;
          ReadDigitalInput:=LetturaIngressiFestoProfinet(OldStato:=OldStato,
                                                         Porta:=porta, 
                                                         Bit:=bit);
        END_IF;
      END_IF; 
      IF IAddress[NumInput].inversione=TRUE THEN
        ///Inversione dello stato dell^ingresso fisico se prevista la proprietà
        IF ReadDigitalInput=TRUE THEN
          ReadDigitalInput:=FALSE;
        ELSE
          ReadDigitalInput:=TRUE;
        END_IF;
      END_IF;
    END_IF;  //Fine else degli ingressi di tipo standard port>-1 bit>-1
  ELSE
    ReadDigitalInput:=FALSE;  
  END_IF; 
END_FUNCTION 

/**************************************************
 * READING STATUS OF VALUED INPUTS VIA SAFETY PLC *
 **************************************************/

FUNCTION LetturaIngressiSafety:BOOL

  VAR_EXTERNAL
    I_Emergenza,
    I_Barriera,
    I_RiarmoAzionamenti,
    I_Barriera_2,
    I_Pedale,
    I_Hold_To_Run,
    I_Radar_Zona_1,
    I_Radar_Zona_2,
    I_Radar_Zona_3,
    //Stato degli ingressi dei moduli di sicurezza
        //Stato degli ingressi dei moduli di sicurezza
    I_Modulo_1_Input_1,I_Modulo_1_Input_2,I_Modulo_1_Input_3,I_Modulo_1_Input_4,
    I_Modulo_2_Input_1,I_Modulo_2_Input_2,I_Modulo_2_Input_3,I_Modulo_2_Input_4,
    I_Modulo_3_Input_1,I_Modulo_3_Input_2,I_Modulo_3_Input_3,I_Modulo_3_Input_4  : BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    INP_PHOTOCELLS_NOT_VIOLATED             :INT;
    INP_MEASURE_TERMINAL_TUBE               :INT;
    INP_SEARCH_WELD                         :INT;
    INP_INTEGRATED_SOUL                     :INT;
    INP_EMERGENCY                           :INT;
    INP_RESET_DRIVES                        :INT;
    INP_BARRIER_2                           :INT;
    INP_PEDALE_SAFETY                       :INT;
    INP_HOLD_TO_RUN                         :INT;
    INP_EMERGENCY_PLC_SAFETY						    :INT;
    INP_PHOTOCELLS_NOT_VIOLATED_PLC_SAFETY	:INT;
    INP_HOLD_TO_RUN_PLC_SAFETY						  :INT;
    INP_PEDALE_SAFETY_PLC_SAFETY					  :INT;
    INP_RESET_DRIVE_PLC_SAFETY				      :INT;
    INP_BARRIER_2_PLC_SAFETY						    :INT;
    INP_OPTICAL_REARMING_PLC_SAFETY					:INT;
    INP_BARRIERA_3_PLC_SAFETY						    :INT;
    INP_EMERGENCY_EXTERIOR_PLC_SAFETY				:INT;
    INP_EXTERNAL_BARRIER_PLC_SAFETY					:INT;
    INP_RESET_EXTERNAL_DRIVES_PLC_SAFETY	  :INT;
    INP_RADAR_ZONA_BIT_0_PLC_SAFETY					:INT;
    INP_RADAR_ZONA_BIT_1_PLC_SAFETY					:INT;
    INP_RADAR_ZONA_1                        :INT;
    INP_RADAR_ZONA_2                        :INT;
    INP_RADAR_ZONA_3                        :INT;
    INP_RADAR_ZONA_BIT_0                    :INT;
    INP_RADAR_ZONA_BIT_1                    :INT;


  END_VAR

  VAR_INPUT
    MeasuringInput         : DB_ANY;
    NumeroInput  :DINT;  
  END_VAR 
  
  VAR
    Weld_Marker : REF_TO TO_MeasuringInput;
  END_VAR
  
  CASE NumeroInput OF
    (*{
    #ifdef SAFETY_INTEGRATED
    }
      INP_SAFETY_UOMO_MORTO:
        LetturaIngressiSafety:=i_sys_UomoMorto;
    {
    #endif  //Fine SAFETY_INTEGRATED
    }*)
    (*{
    #ifdef ORBITALE
    }
      {
      #ifdef FOTOCELLULA_MISURA_X1 %%%%
      }*)
        // INP_MEASURE_TERMINAL_TUBE:
        //   IF _to.FotocellulaMisuraX1.actualinputstate=POSITIVE THEN
        //     LetturaIngressiSafety:=TRUE;
        //   ELSE
        //     LetturaIngressiSafety:=FALSE;
        //   END_IF;
      (*{
      #endif  //Fine FOTOCELLULA_MISURA_X1
      }
    {
    #else  //Fine ORBITALE
    }*)
      INP_SEARCH_WELD, INP_INTEGRATED_SOUL:
        //Nelle macchine prima serie si usava lo stesso ingresso fisico
        //per leggere due diversi dispositivi
        Weld_Marker:= AsMeasuringInputRef(MeasuringInput);
        IF Weld_Marker^.InputState = TRUE THEN
          LetturaIngressiSafety:=TRUE;
        ELSE
          LetturaIngressiSafety:=FALSE;
        END_IF;
    (*{
    #endif  //Fine NO ORBITALE
    }
    {
    #ifdef PROFINET
    }*)
      INP_EMERGENCY:
        LetturaIngressiSafety:=I_Emergenza;
      INP_PHOTOCELLS_NOT_VIOLATED:
        LetturaIngressiSafety:=I_Barriera;
        INP_RESET_DRIVES:
        LetturaIngressiSafety:=I_RiarmoAzionamenti;
      INP_BARRIER_2:
        LetturaIngressiSafety:=I_Barriera_2;
			(*{
      #ifndef ORBITALE
      }*)
        INP_PEDALE_SAFETY:
          LetturaIngressiSafety:=I_Pedale;
        INP_HOLD_TO_RUN:
          LetturaIngressiSafety:=I_Hold_To_Run;
      (*{
      #endif  //Fine NO ORBITALE
      }     
			{
			#ifdef DIAGNOSTICA_PLC_SAFETY
			}*)
				//Ingressi di diagnosdtica per visualizzazione in BlmMotion
				INP_EMERGENCY_PLC_SAFETY:
					LetturaIngressiSafety:=I_Modulo_1_Input_1;
        INP_PHOTOCELLS_NOT_VIOLATED_PLC_SAFETY:
					LetturaIngressiSafety:=I_Modulo_1_Input_2;
				(*{
				#ifndef ORBITALE
				}*)
					INP_HOLD_TO_RUN_PLC_SAFETY:			
						LetturaIngressiSafety:=I_Modulo_1_Input_3;
					INP_PEDALE_SAFETY_PLC_SAFETY:
						LetturaIngressiSafety:=I_Modulo_2_Input_1;
				(*{
				#endif	//Fine NO ORBITALE
				}*)
				INP_RESET_DRIVE_PLC_SAFETY:
					LetturaIngressiSafety:=I_Modulo_1_Input_4;
				(*{
				#ifdef PLC_S7_1500
				}*)
					//Nuovo hardware associato a Tia portal
					INP_BARRIER_2_PLC_SAFETY:
						LetturaIngressiSafety:=I_Modulo_2_Input_2;
          INP_OPTICAL_REARMING_PLC_SAFETY	:
						LetturaIngressiSafety:=I_Modulo_2_Input_3;
					INP_BARRIERA_3_PLC_SAFETY:
						LetturaIngressiSafety:=I_Modulo_2_Input_4;
          INP_EMERGENCY_EXTERIOR_PLC_SAFETY:
						LetturaIngressiSafety:=I_Modulo_3_Input_1;
          INP_EXTERNAL_BARRIER_PLC_SAFETY:
						LetturaIngressiSafety:=I_Modulo_3_Input_2;
          INP_RESET_EXTERNAL_DRIVES_PLC_SAFETY:
						LetturaIngressiSafety:=I_Modulo_3_Input_3;
				(*{			
				#else //FINE PLC_S7_1500
				}*)
					//Vecchio hardware associato a Step7
					(*{
					#ifdef ORBITALE
					}
            INP_BARRIER_2_PLC_SAFETY:
							(*{
							#ifdef DH4010
							}
								LetturaIngressiSafety:=I_Modulo_2_Input_2;
							(*{
							#else	//Fine DH4010
							}
								LetturaIngressiSafety:=I_Modulo_1_Input_3;
							(*{
							#endif	//Fine NO DH4010
							}
					{
					#else	//Fine ORBITALE
					}
            INP_BARRIER_2_PLC_SAFETY:
							LetturaIngressiSafety:=I_Modulo_2_Input_2;*)
						INP_RADAR_ZONA_BIT_0_PLC_SAFETY:
							LetturaIngressiSafety:=I_Modulo_2_Input_3;
						INP_RADAR_ZONA_BIT_1_PLC_SAFETY:
							LetturaIngressiSafety:=I_Modulo_2_Input_4;
						//Con la selezione a 3 bit di 3 zone radar l^impostazione
						//degli ingressi che vengono realmente cablati al PLC safety è:
						//0 0 1 = Zona 1
						//0 1 0 = Zona 2
						//1 0 0 = Zona 3
						INP_RADAR_ZONA_1:
							LetturaIngressiSafety:=I_Radar_Zona_1;
						INP_RADAR_ZONA_2:
							LetturaIngressiSafety:=I_Radar_Zona_2;
						INP_RADAR_ZONA_3:
							LetturaIngressiSafety:=I_Radar_Zona_3;
						//Con la selezione a 2 bit di 3 zone l^impostazione
						//degli ingressi che vengono realmente cablati al PLC safety è:
						//1 0 = Zona 1
						//0 1 = Zona 2
						//0 0 = Zona 3
						INP_RADAR_ZONA_BIT_0:
							IF I_Radar_Zona_1=TRUE THEN 
								LetturaIngressiSafety:=FALSE;
							END_IF;
							IF I_Radar_Zona_2=TRUE THEN 
								LetturaIngressiSafety:=TRUE;
							END_IF;
							IF I_Radar_Zona_3=TRUE THEN 
								LetturaIngressiSafety:=FALSE;
							END_IF;
						INP_RADAR_ZONA_BIT_1:
							IF I_Radar_Zona_1=TRUE THEN 
								LetturaIngressiSafety:=TRUE;
							END_IF;
							IF I_Radar_Zona_2=TRUE THEN 
								LetturaIngressiSafety:=FALSE;
							END_IF;
							IF I_Radar_Zona_3=TRUE THEN
								LetturaIngressiSafety:=FALSE;
							END_IF;
					(*{
					#endif	//Fine NO ORBITALE
					}
				{
				#endif //FINE NO PLC_S7_1500
				}
			{
			#endif	//Fine DIAGNOSTICA_PLC_SAFETY
			}
    {
    #endif  //Fine PROFINET
    } *)    
  END_CASE;
END_FUNCTION 

FUNCTION LetturaIngressiDpCoupler:BOOL

  VAR_EXTERNAL
    V_dbcoupler_i  : dbcoupler_i;
  END_VAR

  VAR_INPUT
    OldStato      :BOOL;
    UsoDpCoupler  :BOOL;
    Porta					:DINT;
    Bit						:USINT;
  END_VAR



    LetturaIngressiDpCoupler:=OldStato;
    IF UsoDpCoupler=TRUE THEN
      CASE porta OF
      //Ingressi da DpCoupler
      900:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[0], bit);//dbcoupler_i00.%X0; //_getbit(dbcoupler_i00, bit);
      901:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[1], bit);
      902:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[2], bit);
      903:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[3], bit);
      904:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[4], bit);
      905:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[5], bit);
      906:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[6], bit);
      907:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[7], bit);
      908:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[8], bit);
      909:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[9], bit);
      910:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[10], bit);
      911:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[11], bit);
      912:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[12], bit);
      913:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[13], bit);
      914:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[14], bit);
      915:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[15], bit);
      916:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[16], bit);
      917:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[17], bit);
      918:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[18], bit);
      919:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[19], bit);
      920:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[20], bit);
      921:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[21], bit);
      922:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[22], bit);
      923:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[23], bit);
      924:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[24], bit);
      925:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[25], bit);
      926:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[26], bit);
      927:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[27], bit);
      928:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[28], bit);
      929:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[29], bit);
      930:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[30], bit);
      931:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[31], bit);
      932:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[32], bit);
      933:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[33], bit);
      934:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[34], bit);
      935:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[35], bit);
      936:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[36], bit);
      937:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[37], bit);
      938:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[38], bit);
      939:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[39], bit);
      940:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[40], bit);
      941:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[41], bit);
      942:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[42], bit);
      943:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[43], bit);
      944:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[44], bit);
      945:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[45], bit);
      946:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[46], bit);
      947:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[47], bit);
      948:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[48], bit);
      949:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[49], bit);
      950:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[50], bit);
      951:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[51], bit);
      952:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[52], bit);
      953:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[53], bit);
      954:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[54], bit);
      955:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[55], bit);
      956:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[56], bit);
      957:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[57], bit);
      958:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[58], bit);
      959:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[59], bit);
      960:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[60], bit);
      961:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[61], bit);
      962:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[62], bit);
      963:LetturaIngressiDpCoupler:=getbit(V_dbcoupler_i.address_db_i[63], bit);
    END_CASE;
  END_IF;
END_FUNCTION  

/***********************************************************************
 * READING STATUS OF VALUED INPUTS VIA PNPNCOUPLER ON PROFINET NETWORK *
 ***********************************************************************/

 FUNCTION LetturaIngressiPnCoupler:BOOL

  VAR_EXTERNAL
    V_pncoupler_i  : pncoupler_i;

  END_VAR

  VAR_INPUT
    OldStato      :BOOL;
    UsoPnCoupler	:BOOL;
    Porta					:DINT;
    Bit						:USINT;
  END_VAR
  LetturaIngressiPnCoupler:=OldStato;
  {::ifdef PROFINET}
    IF UsoPnCoupler=TRUE THEN
      CASE porta OF
        //Ingressi da PnCoupler
        900:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[0], bit);
        901:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[1], bit);
        902:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[2], bit);
        903:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[3], bit);
        904:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[4], bit);
        905:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[5], bit);
        906:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[6], bit);
        907:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[7], bit);
        908:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[8], bit);
        909:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[9], bit);
        910:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[10], bit);
        911:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[11], bit);
        912:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[12], bit);
        913:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[13], bit);
        914:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[14], bit);
        915:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[15], bit);
        916:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[16], bit);
        917:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[17], bit);
        918:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[18], bit);
        919:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[19], bit);
        920:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[20], bit);
        921:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[21], bit);
        922:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[22], bit);
        923:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[23], bit);
        924:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[23], bit);
        925:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[25], bit);
        926:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[26], bit);
        927:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[27], bit);
        928:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[28], bit);
        929:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[29], bit);
        930:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[30], bit);
        931:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[31], bit);
        932:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[32], bit);
        933:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[33], bit);
        934:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[34], bit);
        935:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[35], bit);
        936:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[36], bit);
        937:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[37], bit);
        938:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[38], bit);
        939:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[39], bit);
        940:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[40], bit);
        941:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[41], bit);
        942:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[42], bit);
        943:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[43], bit);
        944:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[44], bit);
        945:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[45], bit);
        946:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[46], bit);
        947:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[47], bit);
        948:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[48], bit);
        949:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[49], bit);
        950:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[50], bit);
        951:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[51], bit);
        952:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[52], bit);
        953:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[53], bit);
        954:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[54], bit);
        955:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[55], bit);
        956:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[56], bit);
        957:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[57], bit);
        958:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[58], bit);
        959:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[59], bit);
        960:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[60], bit);
        961:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[61], bit);
        962:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[62], bit);
        963:LetturaIngressiPnCoupler:=getbit(V_pncoupler_i.address_pn_i[63], bit);
      END_CASE;
    END_IF;
  {::endif}  //Fine PROFINET
 
END_FUNCTION  

FUNCTION LetturaIngressiSiemensProfinet:BOOL

  VAR_EXTERNAL
    i_sys_StatoIOAL5_old			              :BOOL;
    i_sys_StatoIOAL5Opzionale_old           :BOOL;
    i_sys_StatoIOAL5                        :BOOL;
    i_sys_StatoIOAL5Opzionale               :BOOL;
    V_ph_i  : ph_i;
    V_ph_i_old  : ph_i_old;
    V_PHIAL5_Opz  : PHIAL5_Opz;
    V_PHIAL5_Opz_old  : PHIAL5_Opz_old;

                X122_d0 : BOOL;
                X122_d1 : BOOL;
                X122_d2 : BOOL;
                X122_d3 : BOOL;
                X122_d16 : BOOL;
                X122_d17 : BOOL;

                X132_d4 : BOOL;
                X132_d5 : BOOL;
                X132_d6 : BOOL;
                X132_d7 : BOOL;
                X132_d20 : BOOL;
                X132_d21 : BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    INDIRIZZO_X122_SINAMICS             :INT;
    INDIRIZZO_X132_SINAMICS             :INT;
  END_VAR

  VAR_INPUT
    OldStato:BOOL;
    Porta		:DINT;
    Bit			:USINT;
  END_VAR;

  LetturaIngressiSiemensProfinet:=OldStato;
  CASE Porta OF
    //Macchine seconda serie con profinet
    00:LetturaIngressiSiemensProfinet:=getbit(V_ph_i.address_ph_i[0], bit);  //32.0÷7
    01:LetturaIngressiSiemensProfinet:=getbit(V_ph_i.address_ph_i[1], bit);  //33.0÷7
    02:LetturaIngressiSiemensProfinet:=getbit(V_ph_i.address_ph_i[2], bit);  //34.0÷7
    //Ingressi opzionali
    03:LetturaIngressiSiemensProfinet:=getbit(V_ph_i.address_ph_i[3], bit);  //37.0÷7
    04:LetturaIngressiSiemensProfinet:=getbit(V_ph_i.address_ph_i[4], bit);  //38.0÷7
  END_CASE;
  {::ifdef AL_BLM}
    IF i_sys_StatoIOAL5_old=TRUE THEN
      CASE Porta OF
        //IM 151-3 PN ST (STEP 7)ingressi standard AL5
        08:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[0], bit);  //42.0÷3
        09:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[1], bit);  //43.0÷3
        10:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[2], bit);  //44.0÷3
        11:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[3], bit);  //45.0÷3
        12:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[4], bit);  //46.0÷3
        13:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[5], bit);  //47.0÷3
        14:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[6], bit);  //48.0÷3
      END_CASE;
    END_IF;
    IF i_sys_StatoIOAL5Opzionale_old=TRUE THEN
      CASE Porta OF
        //IM 151-3 PN ST (STEP 7)ingressi opzionali 
        15:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz_old.address_PHIAL5_Opz_old[0], bit);  //49.0÷3
        16:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz_old.address_PHIAL5_Opz_old[1], bit);  //50.0÷3
        17:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz_old.address_PHIAL5_Opz_old[2], bit);  //51.0÷3
        18:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz_old.address_PHIAL5_Opz_old[3], bit);  //52.0÷3
        19:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz_old.address_PHIAL5_Opz_old[4], bit);  //53.0÷3
        20:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz_old.address_PHIAL5_Opz_old[5], bit);  //54.0÷3
        21:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz_old.address_PHIAL5_Opz_old[6], bit);  //55.0÷3
      END_CASE;
    END_IF;	

    IF i_sys_StatoIOAL5=TRUE THEN
      //IM 155-6 PN HF (TIA PORTAL) ingressi standard AL5 
      CASE Porta OF
        05:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[0], bit);  //42.0÷7
        06:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[1], bit);  //43.0÷7
        07:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[2], bit);  //44.0÷7
        08:LetturaIngressiSiemensProfinet:=getbit(V_ph_i_old.address_ph_i_old[3], bit);  //45.0÷7
      END_CASE;
    END_IF;
    IF i_sys_StatoIOAL5Opzionale THEN
      //IM 155-6 PN HF (TIA PORTAL) ingressi opzionali 
      CASE Porta OF
        12:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz.address_PHIAL5_Opz[0], bit);  //49.0÷7
        13:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz.address_PHIAL5_Opz[1], bit);  //50.0÷7
        14:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz.address_PHIAL5_Opz[2], bit);  //51.0÷7
        15:LetturaIngressiSiemensProfinet:=getbit(V_PHIAL5_Opz.address_PHIAL5_Opz[3], bit);  //52.0÷7
      END_CASE;
    END_IF;
  {::endif}  //Fine AL_BLM
  
  // CASE Porta OF
    // {::ifdef TAVOLI_ELETTRICI} %%%%
   
    //   23:LetturaIngressiSiemensProfinet:=getbit(I_Tavoli_0, bit);
    //   24:LetturaIngressiSiemensProfinet:=getbit(I_Tavoli_1, bit);
    //   25:LetturaIngressiSiemensProfinet:=getbit(I_Tavoli_2, bit);
    // {::endif} //FINE TAVOLI ELETTRICI
    
      {::ifndef EMULAZIONE}
    
      INDIRIZZO_X122_SINAMICS:
        CASE bit OF
          0:LetturaIngressiSiemensProfinet:=X122_d0;
          1:LetturaIngressiSiemensProfinet:=X122_d1;
          2:LetturaIngressiSiemensProfinet:=X122_d2;
          3:LetturaIngressiSiemensProfinet:=X122_d3;
          8:LetturaIngressiSiemensProfinet:=X122_d16;
          9:LetturaIngressiSiemensProfinet:=X122_d17;
        END_CASE;

      INDIRIZZO_X132_SINAMICS:
        CASE bit OF
          0:LetturaIngressiSiemensProfinet:=X132_d4;
          1:LetturaIngressiSiemensProfinet:=X132_d5;
          2:LetturaIngressiSiemensProfinet:=X132_d6;
          3:LetturaIngressiSiemensProfinet:=X132_d7;
          8:LetturaIngressiSiemensProfinet:=X132_d20;
          9:LetturaIngressiSiemensProfinet:=X132_d21;
        END_CASE;
      {::endif}  //Fine NO EMULAZIONE
  
  // END_CASE;

END_FUNCTION
// {
// #else //FINE PLC_S7_1500
// }

/**************************************************************************
 * READING STATUS OF INPUTS VALUED BY SIEMENS MODULES ON PROFIBUS NETWORK *
 **************************************************************************/

FUNCTION LetturaIngressiSiemensProfibus:BOOL

  VAR_EXTERNAL
    X122_d0 : BOOL;
    X122_d1 : BOOL;
    X122_d2 : BOOL;
    X122_d3 : BOOL;


    X132_d4 : BOOL;
    X132_d5 : BOOL;
    X132_d6 : BOOL;
    X132_d7 : BOOL;

    V_ph_i                   : ph_i;
    V_PHIAL5_Opz             : PHIAL5_Opz;

    // PHIAL5_Opz_1  :BYTE;
    // PHIAL5_Opz_2 :BYTE;
    // PHIAL5_Opz_3 :BYTE;
    // PHIAL5_Opz_4 :BYTE;
    // PHIAL5_Opz_5  :BYTE;
    // PHIAL5_Opz_6 :BYTE;
    // PHIAL5_Opz_7  :BYTE;

    
    // phi00  :BYTE;
    // phi01  :BYTE;
    // phi02  :BYTE;
    // phi03  :BYTE;
    // phi04  :BYTE;
  END_VAR

  VAR_EXTERNAL CONSTANT
    INDIRIZZO_X122_SINAMICS             :INT;
    INDIRIZZO_X132_SINAMICS             :INT;
  END_VAR 

  VAR_INPUT
    OldStato            :BOOL;
    Porta								:DINT;
    Bit									:USINT;
    StatoIOopzionale    :BOOL;
    StatoIOAL42					:BOOL;
    StatoIOAL5          :BOOL;
    StatoIOAL5Opzionale	:BOOL;
  END_VAR  

  LetturaIngressiSiemensProfibus:=OldStato;
  //{::ifndef PROFINET}%%%%
    // IF StatoIOopzionale=TRUE THEN %%%%
      //Modulo IO opzionale
    //   CASE porta OF
    //     00:LetturaIngressiSiemensProfibus:=getbit(PHI00_opzionale,bit);
    //     01:LetturaIngressiSiemensProfibus:=getbit(PHI01_opzionale,bit);
    //     02:LetturaIngressiSiemensProfibus:=getbit(PHI02_opzionale,bit);
    //     03:LetturaIngressiSiemensProfibus:=getbit(PHI03_opzionale,bit);
    //     04:LetturaIngressiSiemensProfibus:=getbit(PHI04_opzionale,bit);
    //     08:LetturaIngressiSiemensProfibus:=getbit(PHI08_opzionale,bit);
    //     09:LetturaIngressiSiemensProfibus:=getbit(PHI09_opzionale,bit);
    //     10:LetturaIngressiSiemensProfibus:=getbit(PHI10_opzionale,bit);
    //   END_CASE;
    // ELSE
    //   //Modulo IO standard 
    //   CASE porta OF
    //     00:LetturaIngressiSiemensProfibus:=getbit(PHI00,bit);
    //     01:LetturaIngressiSiemensProfibus:=getbit(PHI01,bit);
    //     02:LetturaIngressiSiemensProfibus:=getbit(PHI02,bit);
    //     03:LetturaIngressiSiemensProfibus:=getbit(PHI03,bit);
    //     04:LetturaIngressiSiemensProfibus:=getbit(PHI04,bit);
    //   END_CASE;
    // END_IF;
    {::ifndef EMULAZIONE}
      CASE porta OF
        INDIRIZZO_X122_SINAMICS:
          CASE bit OF
            0:LetturaIngressiSiemensProfibus:=X122_d0;
            1:LetturaIngressiSiemensProfibus:=X122_d1;
            2:LetturaIngressiSiemensProfibus:=X122_d2;
            3:LetturaIngressiSiemensProfibus:=X122_d3;
          END_CASE;
        INDIRIZZO_X132_SINAMICS:
          CASE bit OF
            0:LetturaIngressiSiemensProfibus:=X132_d4;
            1:LetturaIngressiSiemensProfibus:=X132_d5;
            2:LetturaIngressiSiemensProfibus:=X132_d6;
            3:LetturaIngressiSiemensProfibus:=X132_d7;
          END_CASE;
      END_CASE;
    {::endif}  //Fine NO EMULAZIONE
    {::ifdef AL_BLM}
    //   {
    //   #ifdef ETURN %%%%
    //   }
        // IF StatoIOAL42=TRUE THEN
        //   CASE porta OF
        //     11:LetturaIngressiSiemensProfibus:=getbit(PHI20,bit);
        //     12:LetturaIngressiSiemensProfibus:=getbit(PHI21,bit);
        //     13:LetturaIngressiSiemensProfibus:=getbit(PHI22,bit);
        //     14:LetturaIngressiSiemensProfibus:=getbit(PHI23,bit);
        //     15:LetturaIngressiSiemensProfibus:=getbit(PHI24,bit);
        //   END_CASE;
        // END_IF;
      // {
      // #endif  //Fine ETURN
      // }
      // IF StatoIOAL5=TRUE THEN
      //   CASE porta OF
      //     11:LetturaIngressiSiemensProfibus:=getbit(PHI20_opzionale, bit);
      //     12:LetturaIngressiSiemensProfibus:=getbit(PHI21_opzionale, bit);
      //     13:LetturaIngressiSiemensProfibus:=getbit(PHI22_opzionale, bit);
      //     14:LetturaIngressiSiemensProfibus:=getbit(PHI23_opzionale, bit);
      //     15:LetturaIngressiSiemensProfibus:=getbit(PHI24_opzionale, bit);
      //     16:LetturaIngressiSiemensProfibus:=getbit(PHI25_opzionale, bit);
      //     17:LetturaIngressiSiemensProfibus:=getbit(PHI26_opzionale, bit);
      //   END_CASE;
      // END_IF;
      // {::ifdef MODULO_ROLAND} %%%%
      //   IF StatoIOAL5Opzionale=TRUE THEN
      //     CASE porta OF
      //       18:LetturaIngressiSiemensProfibus:=getbit(PHIAL5_Opz_1, bit);
      //       19:LetturaIngressiSiemensProfibus:=getbit(PHIAL5_Opz_2, bit);
      //       20:LetturaIngressiSiemensProfibus:=getbit(PHIAL5_Opz_3, bit);
      //       21:LetturaIngressiSiemensProfibus:=getbit(PHIAL5_Opz_4, bit);
      //       22:LetturaIngressiSiemensProfibus:=getbit(PHIAL5_Opz_5, bit);
      //       23:LetturaIngressiSiemensProfibus:=getbit(PHIAL5_Opz_6, bit);
      //       24:LetturaIngressiSiemensProfibus:=getbit(PHIAL5_Opz_7, bit);
      //     END_CASE;
      //   END_IF;
      // {::endif}  //Fine MODULO_ROLAND
      
  {::endif}  //Fine AL_BLM

  // {
  // #endif  //Fine NO PROFINET
  // }
END_FUNCTION


/**************************************************************************
 * READING STATUS OF INPUTS VALUED BY SIEMENS MODULES ON PROFIBUS NETWORK *
 **************************************************************************/

FUNCTION LetturaIngressiFestoProfinet:BOOL

  VAR_EXTERNAL CONSTANT
    FESTO_INP_LINEA_1_MODULO_1          :INT;  //bit 0÷15  16 input
    FESTO_INP_LINEA_1_MODULO_3          :INT;  //bit 0÷07  08 input
    FESTO_INP_LINEA_2_MODULO_2          :INT;  //bit 0÷15  16 input
    FESTO_INP_LINEA_2_MODULO_3          :INT;  //bit 0÷07  08 input
    FESTO_INP_LINEA_3_MODULO_2          :INT;  //bit 0÷15  16 input
    FESTO_INP_LINEA_3_MODULO_3          :INT;  //bit 0÷07  08 input
    FESTO_INP_LINEA_4_MODULO_2          :INT;  //bit 0÷15  16 input
    FESTO_INP_LINEA_4_MODULO_3          :INT;  //bit 0÷07  08 input
END_VAR

  VAR_INPUT
    OldStato	:BOOL;
    Porta			:DINT;
    Bit				:USINT;
  END_VAR      
  LetturaIngressiFestoProfinet:=OldStato;
  {::ifdef IO_FESTO_PROFINET}  
    CASE porta OF
      FESTO_INP_LINEA_1_MODULO_1:
        LetturaIngressiFestoProfinet:=getbit(festo_inp_linea_1a, bit);  //16 ingressi
      FESTO_INP_LINEA_1_MODULO_3:
        LetturaIngressiFestoProfinet:=getbit(festo_inp_linea_1b, bit);  //08 ingressi
      FESTO_INP_LINEA_2_MODULO_2:
        LetturaIngressiFestoProfinet:=getbit(festo_inp_linea_2a, bit);  //16 ingressi
      FESTO_INP_LINEA_2_MODULO_3:
        LetturaIngressiFestoProfinet:=getbit(festo_inp_linea_2b, bit);  //08 ingressi
      FESTO_INP_LINEA_3_MODULO_2:  
        LetturaIngressiFestoProfinet:=getbit(festo_inp_linea_3a, bit);  //16 ingressi
      FESTO_INP_LINEA_3_MODULO_3:
        LetturaIngressiFestoProfinet:=getbit(festo_inp_linea_3b, bit);  //08 ingressi
      FESTO_INP_LINEA_4_MODULO_2:
        LetturaIngressiFestoProfinet:=getbit(festo_inp_linea_4a, bit);  //16 ingressi
      FESTO_INP_LINEA_4_MODULO_3:
        LetturaIngressiFestoProfinet:=getbit(festo_inp_linea_4b, bit);  //08 ingressi
    END_CASE;
  {::endif} //Fine IO_FESTO_PROFINET
  
  ;

END_FUNCTION

FUNCTION_BLOCK FB_EnableMasterAxisBLM

  VAR_EXTERNAL
    AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
  END_VAR

  VAR_EXTERNAL CONSTANT

    FIRST_AXIS_MASTER				        :INT; 
  
  END_VAR

	VAR_INPUT
		NumMaster	:INT;
		//Asse			:PosAxis;
	END_VAR

	VAR
		dret					:LREAL;
		CURRENT_STATE	:INT;
		//thisCommand		:CommandIDType;
	END_VAR

	VAR_OUTPUT
		OutputState:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			(*IF UsoPLCOpen THEN
				//PLCOpen*)
				AssiMaster[NumMaster].MC_Axes.enable_axes :=TRUE;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			(*ELSE
				thisCommand:=_getCommandID();
				CURRENT_STATE:=STATE_ENABLEMASTER_ENABLE;
			END_IF;
		
		STATE_ENABLEMASTER_ENABLE:
			dret:=_enableAxis(axis:=Asse, 
							movingMode:=POSITION_CONTROLLED,
							enableMode:=ALL,
							servoControlMode:=ACTIVE,
							servoCommandToActualMode:=INACTIVE,
							nextCommand:=IMMEDIATELY,
							commandId:=thisCommand,
							forcecontrolMode:=INACTIVE);
			IF (ModoAttesa=WHEN_COMMAND_DONE) THEN
				IF (dret<>7001 AND dret<>7002) THEN
					CURRENT_STATE:=STATE_END;
				END_IF;
			ELSE*)
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			//END_IF;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

/**********************************************************************
 * RESET OF OUTPUTS THAT NEED TO BE RESET WITH CYCLE STOP AND STORAGE *
 *                     OF THE STATE BEFORE RESET                      *
 **********************************************************************/
FUNCTION OnStopCicloDispositivi

  VAR_EXTERNAL
    OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
  END_VAR

  VAR_EXTERNAL CONSTANT
    MAXDIGITALOUTPUT                    :INT;
    MAXDIGITALINPUT                     :INT;
  END_VAR

  VAR
    i    :DINT;
  END_VAR;
  FOR i:=0 TO MAXDIGITALOUTPUT DO
    IF oaddress[i].AbilGestStopCiclo=TRUE THEN
			IF oaddress[i].stato=TRUE THEN
				//Si memorizza lo stato dell^uscita solamente se è attiva
				oaddress[i].statoprestopciclo:=oaddress[i].stato;
			END_IF;
      WriteDigitalOutput(NumOutput:=i, Value:=FALSE);
    END_IF;
  END_FOR;
END_FUNCTION 

/*****************************************************************************
 * READS FROM THE STATE TABLE THE VALUE INDICATED BY THE ROW AND COLUMN. THE *
 *                 PROGRAMME NUMBER FROM 0 TO MAXPROGRAMS-1                  *
 *****************************************************************************/

FUNCTION ReadTabStati:REAL

  VAR_EXTERNAL
    Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
    // {
    // #ifdef DH4010
    // }
    TabellaStati	:ARRAY[TABLE#TABLE_LX ..TABLE#TABLE_RX ] OF TabStato;
  END_VAR

  VAR_EXTERNAL CONSTANT
    ERR_NUMSTATES                       :DINT;
  END_VAR

  VAR_INPUT
    Riga,
    Colonna,
    Programma	:INT;
    Tabella   :SINT;
  END_VAR

  VAR
    Puntatore	:DINT;
    iret      :INT;
  END_VAR

  ;
//  {
//   #ifdef DH4010
//   }
    // IF GetNumberOfStates(Tabella:=Tabella, Programma:=Programma)>0 THEN
    //   Puntatore:=Riga+Programmi[Programma].TabStatiInizio[Tabella];
    //   ReadTabStati:=TabellaStati[Tabella][Puntatore][Colonna];
    // ELSE
    //   iret:=Push_error(Code:=ERR_NUMSTATES, Arresto:=FALSE, Attesa:=FALSE);
    // END_IF;
  // {
  // #else  //Fine DH4010
  // }
    // IF GetNumberOfStates(Tabella:=Tabella, Programma:=Programma)>0 THEN
    //   Puntatore:=Riga+Programmi[Programma].TabStatiInizio;
    //   ReadTabStati:=TabellaStati[Puntatore][Colonna];
    // ELSE
    //   iret:=Push_error(Code:=ERR_NUMSTATES, Arresto:=FALSE, Attesa:=FALSE);
    // END_IF;
  // {
  // #endif  //Fine NO DH4010
  // }
END_FUNCTION    

FUNCTION SetAxSwLimit1

  VAR_EXTERNAL
    pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
  END_VAR

  VAR_EXTERNAL CONSTANT
    EXTRALIMIT         	:REAL ; // extralimite per limiti fisici
    Q1 : INT;
    V1 : INT;
  END_VAR

  VAR_INPUT
		PrimoAsseNoGestione		:INT:=-1;
		SecondoAsseNoGestione	:INT:=-1;
		TerzoAsseNoGestione		:INT:=-1;
		QuartoAsseNoGestione	:INT:=-1;
		QuintoAsseNoGestione	:INT:=-1;
		SestoAsseNoGestione		:INT:=-1;
  END_VAR
  VAR
    i											:INT;
		Eseguire							:BOOL:=TRUE;
  END_VAR
  FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 DO
    pm[i].MA_LIMITE_ATTIVO:=1;
    IF i<>PrimoAsseNoGestione AND i<>SecondoAsseNoGestione AND 
       i<>TerzoAsseNoGestione AND i<>QuartoAsseNoGestione AND
       i<>QuintoAsseNoGestione AND i<>SestoAsseNoGestione THEN
			(*{
			#ifdef ELECTXL
			}*)
				IF i=Q1 OR i=V1 THEN
					//Per ELECT-XL occorre gestire i limiti degli assi Q1 e V1 quando è abilitato da VGP
					//l^accessorio
					Eseguire:=Set_Limiti_Q1V1()=FALSE ;
				END_IF;
			(*{
			#endif //Fine ELECTXL
			}*)
			IF Eseguire=TRUE THEN 
				//Se non sono in monitoring attivo i limiti software 1 agli assi, considerando la posizione
				//attuale del^asse
				ImpostaLimiteSW(Asse:=i, ValoreMinus:=pm[i].Ma_Pos_Limit_Minus_Attivo-EXTRALIMIT);
				ImpostaLimiteSW(Asse:=i, ValorePlus:=pm[i].Ma_Pos_Limit_Plus_Attivo+EXTRALIMIT);
			END_IF;
    END_IF;
  END_FOR;
END_FUNCTION 

/**********************************************************************************************************************
 *           WITH ACCESSORY MO00003632 (INCREASE AXIS TRAVEL Q1 FOR MEDIUM RADIUS UP TO 610MM, THE TRAVELS            *
 *                                   OF Q1 AND V1 ACCORDING TO THE FOLLOWING RULES:                                   *
 *      - MAXIMUM LIMIT INCREASE OF Q1, INDICATED BY VGP VIA THE VARIABLE CFG.LIMQ1CAUSATODAV1, IF THE POSITION       *
 *              OF V1 IS HIGHER THAN THE POSITION INDICATED BY VGP IN THE VARIABLE CFG.LIMV1CAUSATODAQ1               *
 *      - RECORDING OF THE STANDARD POSITIVE LIMIT OF Q1, INDICATED IN PARAMETER M36110, WHEN THE POSITION OF V1      *
 *                   IS LOWER THAN THE VALUE INDICATED BY VGP IN THE VARIABLE CFG.LIMV1CAUSATODAQ1                    *
 * - REDUCTION OF THE NEGATIVE LIMIT OF V1 TO THE VALUE INDICATED BY VGP IN THE VARIABLE CFG.LIMV1CAUSATODAQ1, IF THE *
 *           POSITION OF Q1 IS HIGHER THAN THE STANDARD POSITIVE LIMIT INDICATED IN THE PARAMETER IN M36110           *
 *   - REDETERMINATION OF THE STANDARD NEGATIVE LIMIT OF V1, INDICATED IN PARAMETER M36110, WHEN THE POSITION OF Q1   *
 *                         IS LOWER THAN THE POSITIVE LIMIT OF Q1 GIVEN IN PARAMETER M36110.                          *
 **********************************************************************************************************************/

 (*{
#ifdef ELECTXL
}*)
	FUNCTION Set_Limiti_Q1V1:BOOL 

    VAR_EXTERNAL
    cfg                             :_TYPE.Config_Struct;
    pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
  END_VAR

    VAR_EXTERNAL CONSTANT
      EXTRALIMIT         	                :REAL; // extralimite per limiti fisici
      MY_BIG_VALUE_REAL                   :REAL;
      V1 : INT;
      Q1 : INT;
    END_VAR

    VAR
      PosAx : REF_TO TO_SynchronousAxis;
    END_VAR

		IF cfg.LimQ1CausatoDaV1<MY_BIG_VALUE_REAL-1 AND cfg.LimQ1CausatoDaV1<>-1 AND 
			 cfg.LimV1CausatoDaQ1<MY_BIG_VALUE_REAL-1 AND cfg.LimV1CausatoDaQ1<>-1 THEN
			//Gestione limite V1 in funzione della posizione di Q1
      PosAx := AsSynchronousAxisRef(Axes[Q1].PosAx);
			IF PosAx^.ActualPosition>pm[Q1].Ma_Pos_Limit_Plus THEN
				ImpostaLimiteSw(Asse:=V1, ValoreMinus:=cfg.LimV1CausatoDaQ1-EXTRALIMIT);
			ELSE 
				ImpostaLimiteSw(Asse:=V1, ValoreMinus:=pm[V1].Ma_Pos_Limit_Minus-EXTRALIMIT);
			END_IF;
			//Gestione limite Q1 in funzione della posizione di V1
      PosAx := AsSynchronousAxisRef(Axes[V1].PosAx);
			IF PosAx^.ActualPosition>cfg.LimV1CausatoDaQ1 THEN
				ImpostaLimiteSw(Asse:=Q1, ValorePlus:=cfg.LimQ1CausatoDaV1+EXTRALIMIT);
			ELSE
				ImpostaLimiteSw(Asse:=Q1, ValorePlus:=pm[Q1].Ma_Pos_Limit_Plus+EXTRALIMIT);
			END_IF;
			Set_Limiti_Q1V1:=TRUE;
		ELSE
			Set_Limiti_Q1V1:=FALSE;
		END_IF;
	END_FUNCTION
(*{
#endif //FINE ELECTXL
}*) 

/****************************************************
 * TRUE IF THE INDICATED AXLE IS READY FOR MOVEMENT *
 ****************************************************/

FUNCTION TestAssePronto:BOOL

  VAR_EXTERNAL
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
  END_VAR

  VAR_INPUT
    Asse  :INT;
  END_VAR
  ;
  // IF  CheckIfTOactive(Axis:=Asse)=TRUE THEN %%%%%
  //   IF Axes[Asse].PosAx.actormonitoring.drivestate=ACTIVE AND 
  //      Axes[Asse].PosAx.actormonitoring.power=ACTIVE THEN 
  //     TestAssePronto:=TRUE;
  //   ELSE
  //     TestAssePronto:=FALSE;
  //   END_IF;
  // END_IF;
END_FUNCTION

/*******************************************************************
 * REACTIVATION OF OUTPUTS WHICH MUST BE RESET WITH CYCLE STOP AND *
 *     SUBSEQUENTLY RESET WITH THE VALUE MEMORISED WITH START      *
 *******************************************************************/

FUNCTION OffStopCicloDispositivi

  VAR_EXTERNAL
    i_sys_stopcond      : INT; // 0=NO STOP, 1=STOP ATTIVO
  END_VAR

  VAR_EXTERNAL CONSTANT
    MAXDIGITALOUTPUT                    :INT;
    OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
  END_VAR
  VAR
    i    :DINT;
  END_VAR;
  FOR i:=0 TO MAXDIGITALOUTPUT DO
    IF oaddress[i].AbilGestStopCiclo=TRUE AND 
       i_sys_stopcond=0 AND 
       oaddress[i].StatoPreStopCiclo=TRUE THEN
      WriteDigitalOutput(NumOutput:=i, Value:=oaddress[i].statoprestopciclo);
      oaddress[i].statoprestopciclo:=FALSE;
    END_IF;
  END_FOR;
END_FUNCTION 

FUNCTION ResetAllOutStopPoint

  VAR_EXTERNAL CONSTANT
    PRIMO_STOP_POINT_SET_OUT              	:INT;
    MAX_INP_OUT_STOP_POINT                  :INT;
    PRIMO_STOP_POINT_RESET_OUT            	:INT; //252
    ULTIMO_STOP_POINT_SET_OUT             	:INT;//251
    ULTIMO_STOP_POINT_RESET_OUT           	:INT;//275
  END_VAR

  VAR
    i :INT;
  END_VAR
  FOR i:=PRIMO_STOP_POINT_RESET_OUT TO ULTIMO_STOP_POINT_RESET_OUT DO
    OutStopPoint(Codice:=i, Stato:=FALSE);
  END_FOR;
END_FUNCTION

/*************************************
 * SETTING COMMAND OUTPUTS FROM STOP *
 *************************************/
FUNCTION OutStopPoint

  VAR_EXTERNAL CONSTANT
    PRIMO_STOP_POINT_SET_OUT              	:INT;
    MAX_INP_OUT_STOP_POINT                  :INT;
    PRIMO_STOP_POINT_RESET_OUT            	:INT; //252
    ULTIMO_STOP_POINT_SET_OUT             	:INT;//251
    ULTIMO_STOP_POINT_RESET_OUT           	:INT;//275
    ADDRES_OUT_STOPP: ARRAY[PRIMO_STOP_POINT_SET_OUT..ULTIMO_STOP_POINT_SET_OUT] OF INT;
  END_VAR

  VAR_INPUT
    Codice  :LREAL;
    Stato   :BOOL;  
  END_VAR
  VAR
    i       :INT;
    dret    :DINT;
  END_VAR
  IF Stato=TRUE THEN
    FOR i:=PRIMO_STOP_POINT_SET_OUT TO ULTIMO_STOP_POINT_SET_OUT DO
      IF Codice=i THEN
        WriteDigitalOutput(NumOutput:=ADDRES_OUT_STOPP[TO_INT(Codice)], Value:=Stato);
      END_IF;
    END_FOR;
  ELSE
    FOR i:=PRIMO_STOP_POINT_RESET_OUT TO ULTIMO_STOP_POINT_RESET_OUT DO
      IF Codice=i THEN
        WriteDigitalOutput(NumOutput:=ADDRES_OUT_STOPP[TO_INT(Codice-MAX_INP_OUT_STOP_POINT)], Value:=Stato);
      END_IF;
    END_FOR;
  END_IF;
END_FUNCTION

FUNCTION Set_Disp  
  
  VAR_EXTERNAL
    Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
    OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
    MinutiRisparmioEnergetico       :INT;
  END_VAR

  VAR_EXTERNAL CONSTANT
    FIRST_DEVICE                          :INT;
    MAXDIGITALOUTPUT                      :INT;
    DISP_IN_TT                            :SINT;		//01 - TEMPO LAVORO - TEMPO RIPOSO   xxxxxxx0001
    DISP_IN_TM                            :SINT;		//03 - TEMPO LAVORO - MICRO RIPOSO   xxxxxxx0011
    DISP_IN_MT                            :SINT;		//05 - MICRO LAVORO - TEMPO RIPOSO   xxxxxxx0101
    DISP_IN_MM                            :SINT;		//07 - MICRO LAVORO - MICRO RIPOSO   xxxxxxx0111
    DISP_IN_AI	                          :SINT;		//09 - INGRESSO ANALOGICO      xxxxxxx1001 
    DISP_IN_V                             :SINT;	//16 - INGRESSO VIRTUALE       xxxxxx10000 COMANDATO DALLA LOGICA INTERNA (es: un altro task)
    
    
    DISP_OUT_DO                           :SINT;		//1 - USCITA DIGITALE         xxxxxxxxx01
    DISP_OUT_AO                           :SINT; 	//3 - USCITA ANALOGICA        xxxxxxxxx11
    
  END_VAR

  VAR_INPUT
    Ndisp            :INT; 
    Stato            :INT;
    TimerRisparmio  :BOOL;
  END_VAR
  VAR
    Valore          :REAL;
    dret            :DINT;
  END_VAR
  Disp[Ndisp].StatoRichiesto:=Stato;
  IF Disp[NDisp].Enable=FALSE THEN ;
    oaddress[Ndisp].stato:=FALSE;
    // GOTO End;
  END_IF;
  IF TimerRisparmio=TRUE THEN
    StartTimerRisparmioEnergetico(Tempo:=MinutiRisparmioEnergetico);
  END_IF;
  CASE Disp[Ndisp].TipoOut OF
    DISP_OUT_DO:
      IF Disp[Ndisp].spegnimentouscita=TRUE THEN
        IF Disp[Ndisp].Stato=Disp[Ndisp].StatoRichiesto THEN ;
          // GOTO End;
        END_IF;
      END_IF;
      IF Stato=DEVICES_STATUS#LAVORO THEN
        WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Riposo, Value:=FALSE);
        WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Riposo_2, Value:=FALSE);
        IF DISP[Ndisp].Ritardo_Uscite>0 THEN 
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro, Value:=TRUE);
          //dret:=_waitTime(timeValue:=REAL_TO_TIME(DISP[Ndisp].Ritardo_Uscite*1000));
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro_2, Value:=TRUE);
        ELSE
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro_2, Value:=TRUE);
          //dret:=_waitTime(timeValue:=REAL_TO_TIME(ABS(DISP[Ndisp].Ritardo_Uscite*1000)));
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro, Value:=TRUE);
        END_IF;
      ELSE
        IF Stato=DEVICES_STATUS#RIPOSO THEN
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro, Value:=FALSE);
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Lavoro_2, Value:=FALSE);
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Riposo, Value:=TRUE);
          WriteDigitalOutput(NumOutput:=DISP[Ndisp].Out_Riposo_2, Value:=TRUE);
        END_IF;  
      END_IF;
    DISP_OUT_AO:;
      //Uscita analogica
  END_CASE;
  //Muove solo se non si trova già sullo stato finale    
  IF Stato<>DISP[Ndisp].Stato THEN
    //Doppia uscita digitale
    CASE DISP[Ndisp].TipoIn OF
      DISP_IN_TT:  
        //Tempo lavoro, Tempo riposo
        IF Stato=DEVICES_STATUS#LAVORO THEN
          IF Disp[NDisp].Stato<>DEVICES_STATUS#VERSO_LAVORO OR NOT DISP[NDisp].In_Movimento THEN
            DISP[NDisp].Timer:=((DISP[Ndisp].limite_Lavoro-DISP[Ndisp].Position)/DISP[Ndisp].Vel_Lavoro*1000);
          END_IF; 
          Disp[NDisp].Stato:=DEVICES_STATUS#VERSO_LAVORO; 
          DISP[NDisp].In_Movimento:=TRUE;           
        ELSE
          IF Stato=DEVICES_STATUS#RIPOSO THEN
            IF Disp[NDisp].Stato<>DEVICES_STATUS#VERSO_RIPOSO OR NOT DISP[NDisp].In_Movimento THEN
              DISP[NDisp].Timer:=(DISP[Ndisp].Position-DISP[Ndisp].Limite_Riposo)/DISP[Ndisp].Vel_Riposo*1000;
            END_IF; 
            Disp[NDisp].Stato:=DEVICES_STATUS#VERSO_RIPOSO;  
            DISP[NDisp].In_Movimento:=TRUE; 
          END_IF;
        END_IF;
      DISP_IN_TM:  // 3  - TEMPO LAVORO - MICRO RIPOSO   xxxxxxx0011 
        IF Stato=DEVICES_STATUS#LAVORO THEN
          IF Disp[NDisp].Stato<>DEVICES_STATUS#VERSO_LAVORO OR NOT DISP[NDisp].In_Movimento THEN
            DISP[NDisp].Timer:=((DISP[Ndisp].Limite_Lavoro-DISP[Ndisp].Position)/DISP[Ndisp].Vel_Lavoro*1000);
          END_IF; 
          Disp[NDisp].Stato:=DEVICES_STATUS#VERSO_LAVORO; 
          DISP[NDisp].In_Movimento:=TRUE;           
        ELSE
          IF Stato=DEVICES_STATUS#RIPOSO THEN
            IF Disp[NDisp].Stato<>DEVICES_STATUS#VERSO_RIPOSO OR NOT DISP[NDisp].In_Movimento THEN
              DISP[NDisp].Timer:=((DISP[Ndisp].Position-DISP[Ndisp].Limite_Riposo)/DISP[Ndisp].Vel_Lavoro*1000);
            END_IF; 
            DISP[NDisp].In_Movimento:=TRUE;  
            Disp[NDisp].Stato:=DEVICES_STATUS#VERSO_RIPOSO; 
          END_IF;
        END_IF;
      DISP_IN_MT:  // 5  - MICRO LAVORO - TEMPO RIPOSO   xxxxxxx0101
        IF Stato=DEVICES_STATUS#LAVORO THEN
          IF Disp[NDisp].Stato<>DEVICES_STATUS#VERSO_LAVORO OR NOT DISP[NDisp].In_Movimento THEN
            DISP[NDisp].Timer:=((DISP[Ndisp].Limite_Lavoro-DISP[Ndisp].Position)/DISP[Ndisp].Vel_Lavoro*1000);
          END_IF; 
          DISP[NDisp].In_Movimento:=TRUE;  
          Disp[NDisp].Stato:=DEVICES_STATUS#VERSO_LAVORO;   
         ELSE
          IF Stato=DEVICES_STATUS#RIPOSO THEN
            IF Disp[NDisp].Stato<>DEVICES_STATUS#VERSO_RIPOSO OR NOT DISP[NDisp].In_Movimento THEN
              DISP[NDisp].Timer:=(DISP[Ndisp].Position-DISP[Ndisp].Limite_Riposo)/DISP[Ndisp].Vel_Lavoro*1000;
            END_IF; 
              DISP[NDisp].In_Movimento:=TRUE; 
              Disp[NDisp].Stato:=DEVICES_STATUS#VERSO_RIPOSO;  
            END_IF;
        END_IF;
      DISP_IN_MM:  // 7  - MICRO LAVORO - MICRO RIPOSO   xxxxxxx0111
        DISP[NDisp].In_Movimento:=TRUE; 
        IF Stato=DEVICES_STATUS#LAVORO THEN
          Disp[NDisp].Stato:=DEVICES_STATUS#VERSO_LAVORO;
          DISP[NDisp].Timer:=((DISP[Ndisp].Limite_Lavoro-DISP[Ndisp].Position)/DISP[Ndisp].Vel_Lavoro*1000);
        ELSE
          IF Stato=DEVICES_STATUS#RIPOSO THEN
            Disp[NDisp].Stato:=DEVICES_STATUS#VERSO_RIPOSO;
            DISP[NDisp].Timer:=(DISP[Ndisp].Position-DISP[Ndisp].Limite_Riposo)/DISP[Ndisp].Vel_Lavoro*1000;
          END_IF;
        END_IF;
       DISP_IN_AI:;  // 9  - INGRESSO ANALOGICO      xxxxxxx1001 
       DISP_IN_V:;  // 16  - INGRESSO VIRTUALE 
       DISP[NDisp].In_Movimento:=TRUE;  
    END_CASE;
    //Incremento del timer del valore del filtro
    DISP[NDisp].Timer:=DISP[NDisp].Timer+(Disp[Ndisp].timerfiltro*1000);
  ELSE
    DISP[NDisp].In_Movimento:=FALSE;  
  END_IF;
  // {
  // #ifdef ELECTL_ALL %%%%
  // }
    // Set_Disp_ElectL(Ndisp,Stato);
  // {
  // #endif  //Fine ELECTL_ALL
  // }
  // {
  // #ifdef ELECTXL %%%%
  // }
  //   Set_Disp_ElectXL(Ndisp,Stato);
  // {
  // #endif  //Fine ELECTXL
  // }
  //End:;
END_FUNCTION

FUNCTION SetDispComplesso 

  VAR_EXTERNAL
    MinutiRisparmioEnergetico               :INT;
    cfg                                     :_TYPE.Config_Struct;
    Disp                                    :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
  END_VAR

  VAR_EXTERNAL CONSTANT
    CILSOTTOTESTA                                 :INT;
    SUPPORT_TIRANT                                :INT;
    FIRST_DEVICE                                  :INT;
    OUT_CHIUSURA_CILINDRO_SOTTOTESTA              :INT;
    OUT_APERTURA_CILINDRO_SOTTOTESTA              :INT;
    OUT_APERTURA_INTERMEDIA_CILINDRO_SOTTOTESTA   :INT;
  END_VAR

  VAR_INPUT
    Ndisp           :INT; 
    Value           :REAL;
    Attesa          :BOOL;
    TimerRisparmio  :BOOL;
    
  END_VAR

  VAR
    dret            :DINT;
    ProgrammaAttivo :DINT;
    Stato           :INT;
		AppoValue				:REAL;
  END_VAR

  IF TimerRisparmio=TRUE THEN
    StartTimerRisparmioEnergetico(Tempo:=MinutiRisparmioEnergetico);
  END_IF;
  // {
  // #ifdef ORBITALE %%%%
  // }
  //   {
  //   #ifdef DH4010
  //   }
	// 		CASE Ndisp OF
	// 			BLOCCAGGIO_2D:
	// 				IF Disp[NDisp].Enable=FALSE THEN 
	// 					Disp[Ndisp].in_movimento:=FALSE;
	// 					oaddress[OUT_CHIUSURA_RESISTENZA_1].stato:=FALSE;
	// 					oaddress[OUT_CHIUSURA_RESISTENZA_2].stato:=FALSE;
	// 					oaddress[OUT_CHIUSURA_RESISTENZA_3].stato:=FALSE;
	// 					oaddress[OUT_CHIUSURA_RESISTENZA_4].stato:=FALSE;
	// 					oaddress[OUT_APERTURA_RESISTENZA_1].stato:=FALSE;
	// 					oaddress[OUT_APERTURA_RESISTENZA_2].stato:=FALSE;
	// 					oaddress[OUT_APERTURA_RESISTENZA_3].stato:=FALSE;
	// 					oaddress[OUT_APERTURA_RESISTENZA_4].stato:=FALSE;
	// 				ELSE
	// 					IF Disp[Ndisp].StatoRichiesto=Disp[Ndisp].stato THEN
	// 						Disp[Ndisp].in_movimento:=FALSE;
	// 					ELSE
	// 						Disp[Ndisp].in_movimento:=TRUE;
	// 					END_IF;
	// 					CASE Value OF
	// 						RIPOSO:
	// 							//Apertura tutte insieme
	// 							Set_Disp(Ndisp:=BLOCCAGGIO_RESISTENZA_1, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 							Set_Disp(Ndisp:=BLOCCAGGIO_RESISTENZA_2, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 							Set_Disp(Ndisp:=BLOCCAGGIO_RESISTENZA_3, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 							Set_Disp(Ndisp:=BLOCCAGGIO_RESISTENZA_4, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 						LAVORO:
	// 							//Chiusura delle sole posizioni abilitate
	// 							IF Centratore_cfg.ResistenzeProg[0]=TRUE THEN
	// 								Set_Disp(Ndisp:=BLOCCAGGIO_RESISTENZA_1, Stato:=LAVORO, TimerRisparmio:=TRUE);
	// 							END_IF;
	// 							IF Centratore_cfg.ResistenzeProg[1]=TRUE THEN
	// 								Set_Disp(Ndisp:=BLOCCAGGIO_RESISTENZA_2, Stato:=LAVORO, TimerRisparmio:=TRUE);
	// 							END_IF;
	// 							IF Centratore_cfg.ResistenzeProg[2]=TRUE THEN
	// 								Set_Disp(Ndisp:=BLOCCAGGIO_RESISTENZA_3, Stato:=LAVORO, TimerRisparmio:=TRUE);
	// 							END_IF;
	// 							IF Centratore_cfg.ResistenzeProg[3]=TRUE THEN
	// 								Set_Disp(Ndisp:=BLOCCAGGIO_RESISTENZA_4, Stato:=LAVORO, TimerRisparmio:=TRUE);
	// 							END_IF;
	// 					END_CASE;
	// 				END_IF;
	// 			PINZA_SX:
	// 				IF Disp[NDisp].Enable=FALSE THEN ;
	// 					oaddress[OUT_PINZA_PRECHIUSA1_TESTA_SX].stato:=FALSE;
	// 					oaddress[OUT_PINZA_APERTA_TESTA_SX].stato:=FALSE;
	// 				ELSE
	// 					IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
	// 						IF ((Disp[Ndisp].stato=LAVORO AND mcpi_minus=TRUE) OR
	// 								(Disp[Ndisp].stato=RIPOSO AND mcpi_plus=TRUE) OR
	// 								(Disp[Ndisp].stato<>LAVORO AND Disp[Ndisp].stato<>RIPOSO) OR
	// 								(i_sys_activedevice=C_TAGLIO_SX) OR (i_sys_activedevice=CMD_CURVA_MANUALE) OR (i_sys_activedevice=CMD_SNERVAMENTO) OR (i_sys_activedevice=CMD_FINE_SNERVAMENTO)) OR
	// 								i_sys_OPMODE<>JOGMODE THEN
	// 							//Se è attivo il JOG la gestione dei tasti + o - deve essere condizionata allo stato
	// 							//di partenza del dispositivo
	// 							Disp[Ndisp].in_movimento:=TRUE;
	// 							//In caso di posizione indefinita si esegue comunque l^apertura
	// 							IF Disp[Ndisp].Position=INDEFINITO AND i_sys_opmode=JOGMODE AND i_sys_activedevice=Ndisp THEN
	// 								AppoValue:=RIPOSO;
	// 							ELSE
	// 								AppoValue:=Value;
	// 							END_IF;
	// 							CASE AppoValue OF
	// 								RIPOSO:
	// 									//Apertura pinza
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_PRECHIUSA1_TESTA_SX, Value:=FALSE);
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_APERTA_TESTA_SX, Value:=TRUE);
	// 									Set_Disp(Ndisp:=PINZA_SX_PRECHIUSURA, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 								INTERMEDIO_PINZA:
	// 									//Prechiusura pinza
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_PRECHIUSA1_TESTA_SX, Value:=TRUE);
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_APERTA_TESTA_SX, Value:=FALSE);
	// 									Set_Disp(Ndisp:=PINZA_SX_PRECHIUSURA, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 								LAVORO:
	// 									//Chiusura pinza
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_PRECHIUSA1_TESTA_SX, Value:=TRUE);
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_APERTA_TESTA_SX, Value:=FALSE);
	// 									Set_Disp(Ndisp:=PINZA_SX_PRECHIUSURA, Stato:=LAVORO, TimerRisparmio:=TRUE);
	// 							END_CASE;
	// 						END_IF;
	// 					END_IF;
	// 				END_IF;
	// 			PINZA_DX:
	// 				IF Disp[NDisp].Enable=FALSE THEN ;
	// 					oaddress[OUT_PINZA_PRECHIUSA1_TESTA_DX].stato:=FALSE;
	// 					oaddress[OUT_PINZA_APERTA_TESTA_DX].stato:=FALSE;
	// 				ELSE
	// 					IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
	// 						IF ((Disp[Ndisp].stato=LAVORO AND mcpi_minus=TRUE) OR
	// 								(Disp[Ndisp].stato=RIPOSO AND mcpi_plus=TRUE) OR
	// 								(Disp[Ndisp].stato<>LAVORO AND Disp[Ndisp].stato<>RIPOSO) OR
	// 								(i_sys_activedevice=C_TAGLIO_DX) OR (i_sys_activedevice=CMD_CURVA_MANUALE) OR (i_sys_activedevice=CMD_SNERVAMENTO) OR (i_sys_activedevice=CMD_FINE_SNERVAMENTO)) OR
	// 							 i_sys_OPMODE<>JOGMODE THEN
	// 							//Se è attivo il JOG la gestione dei tasti + o - deve essere condizionata allo stato
	// 							//di partenza del dispositivo
	// 							Disp[Ndisp].in_movimento:=TRUE;
	// 							//In caso di posizione indefinita si esegue comunque l^apertura
	// 							IF Disp[Ndisp].Position=INDEFINITO AND i_sys_opmode=JOGMODE AND i_sys_activedevice=Ndisp THEN
	// 								AppoValue:=RIPOSO;
	// 							ELSE
	// 								AppoValue:=Value;
	// 							END_IF;
	// 							CASE AppoValue OF
	// 								RIPOSO:
	// 									//Apertura pinza
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_PRECHIUSA1_TESTA_DX, Value:=FALSE);
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_APERTA_TESTA_DX, Value:=TRUE);
	// 									Set_Disp(Ndisp:=PINZA_DX_PRECHIUSURA, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 								INTERMEDIO_PINZA:
	// 									//Prechiusura pinza
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_PRECHIUSA1_TESTA_DX, Value:=TRUE);
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_APERTA_TESTA_DX, Value:=FALSE);
	// 									Set_Disp(Ndisp:=PINZA_DX_PRECHIUSURA, Stato:=RIPOSO, TimerRisparmio:=TRUE);
	// 								LAVORO:
	// 									//Chiusura pinza
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_PRECHIUSA1_TESTA_DX, Value:=TRUE);
	// 									WriteDigitalOutput(NumOutput:=OUT_PINZA_APERTA_TESTA_DX, Value:=FALSE);
	// 									Set_Disp(Ndisp:=PINZA_DX_PRECHIUSURA, Stato:=LAVORO, TimerRisparmio:=TRUE);
	// 							END_CASE;
	// 						END_IF;
	// 					END_IF;
	// 				END_IF;
	// 			ELSE
  //       ;
  //     END_CASE;
  //   {
  //   #endif  //Fine DH4010
  //   }
  //   {
  //   #ifdef EFLEX
  //   }
  //     CASE Ndisp OF
  //       TESTINA_1, TESTINA_2:
  //         IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
  //           Disp[Ndisp].in_movimento:=TRUE;
  //           CASE Value OF
  //             RIPOSO:
  //               //Sollevamento testina
  //               WriteDigitalOutput(NumOutput:=Disp[Ndisp].Out_Riposo, Value:=TRUE);
  //               WriteDigitalOutput(NumOutput:=Disp[Ndisp].Out_Lavoro, Value:=FALSE);
  //             INTERMEDIO:
  //               IF Disp[Ndisp].position=LAVORO THEN
  //                 //Si parte da testina bassa, si solleva la testina
  //                 WriteDigitalOutput(NumOutput:=Disp[Ndisp].Out_Riposo, Value:=TRUE);
  //                 WriteDigitalOutput(NumOutput:=Disp[Ndisp].Out_Lavoro, Value:=FALSE);
  //               ELSE
  //                 IF Disp[Ndisp].position=RIPOSO THEN
  //                   //Si parte da testina alta, si abbassa la testina
  //                   WriteDigitalOutput(NumOutput:=Disp[Ndisp].Out_Lavoro, Value:=TRUE);
  //                   WriteDigitalOutput(NumOutput:=Disp[Ndisp].Out_Riposo, Value:=FALSE);
  //                 END_IF;
  //               END_IF;
  //             LAVORO:
  //               //Abbassamento testina
  //               WriteDigitalOutput(NumOutput:=Disp[Ndisp].Out_Lavoro, Value:=TRUE);
  //               WriteDigitalOutput(NumOutput:=Disp[Ndisp].Out_Riposo, Value:=FALSE);
  //           END_CASE;
  //         END_IF;
  //     ELSE
  //       ;
  //     END_CASE;
  //   {
  //   #endif  //Fine EFLEX
  //   }
  //   {
  //   #ifdef RUNNER
  //   }
  //     CASE Ndisp OF
  //       RULLATORE:
  //         IF Disp[Ndisp].StatoRichiesto<>Disp[Ndisp].Stato OR Disp[Ndisp].StatoRichiesto<>Disp[Ndisp].Position THEN
  //           Disp[Ndisp].in_movimento:=TRUE;
  //           CASE Value OF
  //             LAVORO:
  //               //Rotazione veloce senso orario
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_INVERSIONE_ROTAZIONE, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_VEL_MIN, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_VEL_MAX, Value:=TRUE);
  //             INTERMEDIO:
  //               //Rotazione lenta senso orario
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_INVERSIONE_ROTAZIONE, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_VEL_MAX, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_VEL_MIN, Value:=TRUE);
  //           ELSE
  //             IF Value=-INTERMEDIO THEN
  //               //Rotazione lenta senso antiorario
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_VEL_MIN, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_VEL_MAX, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_INVERSIONE_ROTAZIONE, Value:=TRUE);
  //             ELSE
  //               //Arresto rullatore
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_VEL_MIN, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_VEL_MAX, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_4RROT_MOTORE_INVERSIONE_ROTAZIONE, Value:=FALSE);
  //             END_IF;
  //           END_CASE;
  //         END_IF;  
  //       {
  //       #ifdef ASPO_RUNNER
  //       }
	// 				ASPO:
	// 					IF IAddress[INP_INVERTER_ASPO_ON].Port=-1 THEN
	// 						//Gestione aspo senza inverter
	// 						IF i_sys_OpMode=AUTOMODE THEN
	// 							ProgrammaAttivo:=I_SYS_Current_Prog;
	// 						ELSE
	// 							ProgrammaAttivo:=PROGRAMMA_JOG;
	// 						END_IF;
	// 						IF cfg.ASPO_NuovoModo=TRUE THEN
	// 							IF programmi[ProgrammaAttivo].ModoAspo=ASPO_IN_FOLLE OR i_sys_SenzaTubo=TRUE THEN
	// 								//ASPO_IN_FOLLE
	// 								WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 								WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 								WriteDigitalOutput(NumOutput:=OUT_4RASPO_PRESSIONE_FRIZIONE, Value:=FALSE);
	// 							ELSE
	// 								//ASPO_MOTORIZZATO o ASPO_IN_FOLLE_AVANZAMENTO
	// 								Disp[Ndisp].in_movimento:=TRUE;
	// 								CASE Value OF
	// 									LAVORO:
	// 										//Svolgimento aspo
	// 										IF programmi[ProgrammaAttivo].ModoAspo=ASPO_MOTORIZZATO THEN
	// 											//Svolgimento motorizzato si attiva la frizione
	// 											WriteDigitalOutput(NumOutput:=OUT_4RASPO_PRESSIONE_FRIZIONE, Value:=TRUE);
	// 										ELSE
	// 											//Svolgimento in folle si disattiva la frizione
	// 											WriteDigitalOutput(NumOutput:=OUT_4RASPO_PRESSIONE_FRIZIONE, Value:=FALSE);
	// 										END_IF;
	// 										IF readdigitalinput(NumInput:=Disp[Ndisp].in_lavoro, Verifica:=TRUE)=FALSE THEN
	// 											//Svolgimento tubo solo se braccio aspo non è aperto
	// 											WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 											IF programmi[ProgrammaAttivo].ModoAspo<>ASPO_IN_FOLLE_AVANZAMENTO OR
	// 												 OAddress[OUT_ASPO_VERTICALE_NO_ATTREZZAGGIO].port=-1 THEN
	// 												//Con aspo verticale non si deve attivare la valvola di svolgimento dell^aspo
	// 												//indicazione telefonica per cliente Bemasch di Alberto Pittioni 08/11/2017
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=TRUE);
	// 											END_IF;
	// 										ELSE
	// 											//Con braccio aperto si ferma lo svolgimento dell^aspo e si aggancia comunque
	// 											//la frizione per eseguire la migliore frenatura possibile
	// 											WriteDigitalOutput(NumOutput:=OUT_4RASPO_PRESSIONE_FRIZIONE, Value:=TRUE);
	// 											WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 										END_IF;  
	// 									RIPOSO:
	// 										//Riavvolgimento aspo sempre motorizzato, sempre con frizione inserita
	// 										WriteDigitalOutput(NumOutput:=OUT_4RASPO_PRESSIONE_FRIZIONE, Value:=TRUE);
	// 										IF readdigitalinput(NumInput:=Disp[Ndisp].in_riposo, Verifica:=TRUE)=FALSE THEN
	// 											//Avvolgimento tubo solo se braccio aspo non è chiuso 
	// 											WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 											WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=TRUE);
	// 										ELSE
	// 											//Con braccio chiuso si ferma l^avvolgimento dell^aspo
	// 											WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 										END_IF;
	// 									INTERMEDIO:
	// 										//Arresto aspo
	// 										IF cfg.ASPO_MantieneSvolgimento=FALSE OR 
	// 											 i_sys_OPMODE<>AUTOMODE OR
	// 											 TestMasterLavoro()=FALSE OR
	// 											 OAddress[OUT_4RASPO_ASPO_SVOLGI_TUBO].stato=FALSE OR
	// 											 (cfg.ASPO_MantieneSvolgimento=TRUE AND
	// 												readdigitalinput(NumInput:=INP_ASPO_BRACCIO_APERTO, Verifica:=TRUE)=TRUE) THEN
	// 											//In automatico mantiene lo svolgimento dell^aspo fino all^attivazione
	// 											//del sensore di braccio aperto
	// 											WriteDigitalOutput(NumOutput:=OUT_4RASPO_PRESSIONE_FRIZIONE, Value:=TRUE);
	// 											WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 											WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 										END_IF;
	// 								END_CASE;  //Case stati dell^Aspo    
	// 							END_IF;
	// 						ELSE
	// 							//Vecchio modo
	// 							IF Disp[Ndisp].statorichiesto=Disp[Ndisp].stato THEN
	// 								IF Value=INTERMEDIO THEN
	// 									//Arresto aspo
	// 									WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 									WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 								END_IF;
	// 							ELSE
	// 								//Aspo in movimento
	// 								Disp[Ndisp].in_movimento:=TRUE;
	// 								IF programmi[ProgrammaAttivo].ModoAspo=ASPO_IN_FOLLE THEN
	// 									//Richiesta rotazione in folle dell^aspo, si disabilita la frizione che trasmette il movimento
	// 									WriteDigitalOutput(NumOutput:=OUT_4RASPO_PRESSIONE_FRIZIONE, Value:=FALSE);
	// 								ELSE
	// 									//Richiesta rotazione standard dell^aspo, si abilita la frizione che trasmette il movimento
	// 									WriteDigitalOutput(NumOutput:=OUT_4RASPO_PRESSIONE_FRIZIONE, Value:=TRUE);
	// 								END_IF;
	// 								CASE Value OF
	// 									LAVORO:
	// 										IF readdigitalinput(NumInput:=Disp[Ndisp].in_lavoro, Verifica:=TRUE)=FALSE THEN
	// 											//Svolgimento tubo solo se braccio aspo non è aperto
	// 											IF i_sys_SenzaTubo=TRUE THEN
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 											ELSE  
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=TRUE);
	// 											END_IF;
	// 										END_IF;  
	// 									RIPOSO:
	// 										IF readdigitalinput(NumInput:=Disp[Ndisp].in_riposo, Verifica:=TRUE)=FALSE THEN
	// 											//Avvolgimento tubo solo se braccio aspo non è chiuso
	// 											IF i_sys_SenzaTubo=TRUE THEN
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 											ELSE  
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 												WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=TRUE);
	// 											END_IF;
	// 										END_IF;
	// 									INTERMEDIO:
	// 										//Arresto aspo
	// 										WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_lavoro, Value:=FALSE);
	// 										WriteDigitalOutput(NumOutput:=Disp[Ndisp].out_riposo, Value:=FALSE);
	// 								END_CASE;  //Case stati dell^Aspo    
	// 							END_IF;
	// 						END_IF;
	// 					END_IF;
	// 				{
	// 				#endif	//Fine ASPO_RUNNER
	// 				}
  //       {
  //       #ifdef THREE_RUNNER
  //       }
  //         IPC_STRAPPO_SCARICO:
  //           IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
  //             Disp[Ndisp].in_movimento:=TRUE;
  //           END_IF;
  //           //Per evitare che rimbalzi del sensore che rileva la posizione arretrata dello
  //           //strappo_scarico, si esegue comunque l^impostazione delle uscite
  //           CASE Value OF
  //             LAVORO:
  //               WriteDigitalOutput(NumOutput:=OUT_STRAPPO_SCARICO_AVANTI, Value:=TRUE);
  //               WriteDigitalOutput(NumOutput:=OUT_STRAPPO_SCARICO_INTERMEDIO, Value:=TRUE);
  //             RIPOSO:
  //               WriteDigitalOutput(NumOutput:=OUT_STRAPPO_SCARICO_INTERMEDIO, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_STRAPPO_SCARICO_AVANTI, Value:=FALSE);
  //             INTERMEDIO:
  //               WriteDigitalOutput(NumOutput:=OUT_STRAPPO_SCARICO_AVANTI, Value:=FALSE);
  //               WriteDigitalOutput(NumOutput:=OUT_STRAPPO_SCARICO_INTERMEDIO, Value:=TRUE);
  //           END_CASE;
  //       {
  //       #endif  //Fine THREE_RUNNER
  //       }
  //       {
  //       #ifdef FOUR_RUNNER_PLAUNO
  //       }
  //         {
  //         #ifdef PLAUNO_PLAUNO_LIGHT
  //         }
  //           BLOCCAGGIO_TUBO:
  //             IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
  //               Disp[Ndisp].in_movimento:=TRUE;
  //               Stato:=LREAL_TO_INT(Disp[Ndisp].statorichiesto);
  //               IF Stato=INTERMEDIO THEN
  //                 IF Assi[Q1].PosAx.positioningState.actualPosition>0.1 THEN
  //                   Set_Disp(Ndisp:=BLOCCAGGIO_TUBO_DX, Stato:=LAVORO, TimerRisparmio:=TRUE);
  //                   Set_Disp(Ndisp:=BLOCCAGGIO_TUBO_SX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
  //                 ELSE
  //                   Set_Disp(Ndisp:=BLOCCAGGIO_TUBO_DX, Stato:=RIPOSO, TimerRisparmio:=TRUE);
  //                   Set_Disp(Ndisp:=BLOCCAGGIO_TUBO_SX, Stato:=LAVORO, TimerRisparmio:=TRUE);
  //                 END_IF;
  //               ELSE
  //                 Set_Disp(Ndisp:=BLOCCAGGIO_TUBO_DX, Stato:=Stato, TimerRisparmio:=TRUE);
  //                 Set_Disp(Ndisp:=BLOCCAGGIO_TUBO_SX, Stato:=Stato, TimerRisparmio:=TRUE);
  //               END_IF;
  //             END_IF;
  //         {
	// 				#endif  //Fine PLAUNO_PLAUNO_LIGHT
  //         }
  //         {
  //         #ifdef TAVOLI_SERPENTINE
  //         }
  //           TAVOLO_POST_ORIZZONTALE:
  //             IF cfg._4RUNNER_TavoliSerpentine=TRUE THEN
  //               IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
  //                 Disp[Ndisp].in_movimento:=TRUE;
  //                 Stato:=LREAL_TO_INT(Disp[Ndisp].statorichiesto);
  //                 //Tavoli posteriori destro e sinistro
  //                 Set_Disp(Ndisp:=TAVOLO_POST_ORIZZ_DX, Stato:=Stato, TimerRisparmio:=TRUE);
  //                 Set_Disp(Ndisp:=TAVOLO_POST_ORIZZ_SX, Stato:=Stato, TimerRisparmio:=TRUE);
  //                 IF (Attesa=TRUE AND i_sys_OPMODE<>AUTOMODE) OR (Attesa=TRUE AND i_sys_OPMODE=AUTOMODE AND TestMasterLavoro()=FALSE) THEN
  //                   WHILE Disp[TAVOLO_POST_ORIZZ_DX].Stato<>Stato OR Disp[TAVOLO_POST_ORIZZ_SX].Stato<>Stato DO
  //                     //Serve per attendere i due tavoli utilizzando la trace del dispositivo fittizzio ma solo se non siamo durante 
	// 										//l^esecuzione della tabella stati. In tal caso l^attesa del dispositivo si attende nella prossima riga con flyby a 0
  //                     dret:=_waitTime(T#0ms);
  //                     wait_disp(Device:=TAVOLO_POST_ORIZZONTALE, State:=Stato, Trace:=TRUE);
  //                   END_WHILE;
  //                 END_IF;
  //               END_IF;
  //             END_IF;
  //         {
	// 				#endif  //Fine TAVOLI_SERPENTINE
  //         }
  //         PINZA_ACCOMPAGNAMENTO:
  //           IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
  //             Disp[Ndisp].in_movimento:=TRUE;
  //             CASE Value OF
  //               RIPOSO:
  //                 //Apertura pinza
  //                 Set_Disp(Ndisp:=PINZA_ACCOMPAG_CH_AP, Stato:=RIPOSO, TimerRisparmio:=TRUE);
  //                 IF Attesa=TRUE THEN
  //                   wait_disp(Device:=PINZA_ACCOMPAG_CH_AP, 
	// 														State:=RIPOSO, 
	// 														Trace:=TRUE);
  //                 END_IF;
  //                 //Arretramento cilindro accompagnamento
  //                 IF Attesa=TRUE THEN
  //                   Set_Disp(Ndisp:=PINZA_ACCOMPAG_AV_IN, Stato:=RIPOSO, TimerRisparmio:=TRUE);
  //                 ELSE
  //                   IF Disp[PINZA_ACCOMPAG_CH_AP].stato=Disp[PINZA_ACCOMPAG_CH_AP].StatoRichiesto THEN
  //                     //In caso di movimento senza attesa, si comanda l^arretramento
  //                     //del carrello di accompagnamento, quando le pinze si sono aperte
  //                     Set_Disp(Ndisp:=PINZA_ACCOMPAG_AV_IN, Stato:=RIPOSO, TimerRisparmio:=TRUE);
  //                   END_IF;
  //                 END_IF;
  //                 IF Attesa=TRUE THEN
  //                   set_trace(Code:=TRC_CARRELLO_ACCOMPAGNAMENTO_INDIETRO);
  //                   wait_disp(Device:=PINZA_ACCOMPAG_AV_IN, 
	// 														State:=RIPOSO, 
	// 														Trace:=FALSE);
	// 									reset_trace(TRC_CARRELLO_ACCOMPAGNAMENTO_INDIETRO);
  //                 END_IF;
  //               LAVORO:
  //                 //Chiusura pinza
  //                 Set_Disp(Ndisp:=PINZA_ACCOMPAG_CH_AP, Stato:=LAVORO, TimerRisparmio:=TRUE);
  //                 IF Attesa=TRUE THEN
  //                   //Attende se richiesto oppure se la pinza impiega meno di 1 secondo a chiudersi
  //                   wait_disp(Device:=PINZA_ACCOMPAG_CH_AP, 
	// 														State:=LAVORO, 
	// 														Trace:=FALSE);
  //                   //Avanzamento spintore
  //                   Set_Disp(Ndisp:=PINZA_ACCOMPAG_AV_IN, Stato:=LAVORO, TimerRisparmio:=TRUE);
  //                 ELSE
  //                   IF Disp[PINZA_ACCOMPAG_CH_AP].stato=LAVORO THEN
  //                     //Avanzamento spintore dopo la chiusura pinza, senza attendere per non bloccare
  //                     //il background
  //                     Set_Disp(Ndisp:=PINZA_ACCOMPAG_AV_IN, Stato:=LAVORO, TimerRisparmio:=TRUE);
  //                   END_IF;
  //                 END_IF;
  //               INTERMEDIO:
  //                 //Apertura pinza
  //                 Set_Disp(Ndisp:=PINZA_ACCOMPAG_CH_AP, Stato:=RIPOSO, TimerRisparmio:=TRUE);
  //                 IF Attesa=TRUE THEN
  //                   wait_disp(Device:=PINZA_ACCOMPAG_CH_AP, 
	// 														State:=RIPOSO, 
	// 														Trace:=FALSE);
  //                 END_IF;
  //             END_CASE;
  //           END_IF;
  //       {
  //       #endif  //Fine FOUR_RUNNER_PLAUNO
  //       }
  //     END_CASE;  //Case dei dispositivi  
  //   {
  //   #endif  //Fine RUNNER
  //   }
  // {
  // #else  //Fine ORBITALE
  // }
    //Inizio macchine lineari
    CASE Ndisp OF
      CILSOTTOTESTA:
        IF cfg.SottoTesta3Pos=1 OR cfg.SottoTesta3Pos=2 THEN
          IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
            Disp[Ndisp].in_movimento:=TRUE;
          END_IF;
          CASE TO_INT(Value) OF
            DEVICES_STATUS#RIPOSO:
              IF cfg.SottoTesta3Pos=1 THEN
                //Apertura intermedia cilindro sottotesta
                WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_CILINDRO_SOTTOTESTA, Value:=FALSE);
                WriteDigitalOutput(NumOutput:=OUT_APERTURA_CILINDRO_SOTTOTESTA, Value:=FALSE);
                WriteDigitalOutput(NumOutput:=OUT_APERTURA_INTERMEDIA_CILINDRO_SOTTOTESTA, Value:=TRUE);
              ELSE
                //Apertura totale cilindro sottotesta
                WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_CILINDRO_SOTTOTESTA, Value:=FALSE);
                WriteDigitalOutput(NumOutput:=OUT_APERTURA_CILINDRO_SOTTOTESTA, Value:=TRUE);
                WriteDigitalOutput(NumOutput:=OUT_APERTURA_INTERMEDIA_CILINDRO_SOTTOTESTA, Value:=TRUE);
              END_IF;
              DEVICES_STATUS#LAVORO:
              //Chiusura cilindro sottotesta
              WriteDigitalOutput(NumOutput:=OUT_APERTURA_CILINDRO_SOTTOTESTA, Value:=FALSE);
              WriteDigitalOutput(NumOutput:=OUT_APERTURA_INTERMEDIA_CILINDRO_SOTTOTESTA, Value:=FALSE);
              WriteDigitalOutput(NumOutput:=OUT_CHIUSURA_CILINDRO_SOTTOTESTA, Value:=TRUE);
          END_CASE;
        END_IF;
       SUPPORT_TIRANT:
        // {
        // #ifdef TIRANTE_ANIMA_FC
        // }
        //   IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
        //     Disp[Ndisp].in_movimento:=TRUE;
        //   END_IF;
        //   CASE Value OF
        //     RIPOSO:
        //       Set_Disp(Ndisp:=SOSTEGNO_TIRANTE1, Stato:=RIPOSO, TimerRisparmio:=TRUE);
        //       Set_Disp(Ndisp:=SOSTEGNO_TIRANTE2, Stato:=RIPOSO, TimerRisparmio:=TRUE);
        //     INTERMEDIO:
        //       Set_Disp(Ndisp:=SOSTEGNO_TIRANTE1, Stato:=LAVORO, TimerRisparmio:=TRUE);
        //       Set_Disp(Ndisp:=SOSTEGNO_TIRANTE2, Stato:=RIPOSO, TimerRisparmio:=TRUE);
        //     LAVORO:
        //       Set_Disp(Ndisp:=SOSTEGNO_TIRANTE1, Stato:=LAVORO, TimerRisparmio:=TRUE);
        //       Set_Disp(Ndisp:=SOSTEGNO_TIRANTE2, Stato:=LAVORO, TimerRisparmio:=TRUE);
        //   END_CASE;
        // {
        // #endif  //Fine TIRANTE_ANIMA_FC
        // }
        ;
      // {
      // #ifdef SPINTORE_ULTIMA_CURVA
      // }
      //   SPINTA_ULTIMA_CURVA:
      //     IF Disp[Ndisp].statorichiesto<>Disp[Ndisp].stato THEN
      //       Disp[Ndisp].in_movimento:=TRUE;
      //       CASE Value OF
      //         RIPOSO:
      //           //Spintore indietro e bloccato
      //           Set_Disp(Ndisp:=BLOCCO_SBLOCCO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
      //           wait_disp(Device:=BLOCCO_SBLOCCO, State:=RIPOSO, Trace:=FALSE);
      //           //Arretramento spintore
      //           Set_Disp(Ndisp:=AVANTI_INDIETRO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
      //           wait_disp(Device:=AVANTI_INDIETRO, State:=RIPOSO, Trace:=FALSE);
      //           //Blocco dispositivi di spinta
      //           Set_Disp(Ndisp:=BLOCCO_SBLOCCO, Stato:=LAVORO, TimerRisparmio:=TRUE);
      //           wait_disp(Device:=BLOCCO_SBLOCCO, State:=LAVORO, Trace:=FALSE);
      //         LAVORO:
      //           //Spintore avanti e bloccato
      //           Set_Disp(Ndisp:=BLOCCO_SBLOCCO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
      //           wait_disp(Device:=BLOCCO_SBLOCCO, State:=RIPOSO, Trace:=FALSE);
      //           //Avanzamento spintore
      //           Set_Disp(Ndisp:=AVANTI_INDIETRO, Stato:=LAVORO, TimerRisparmio:=TRUE);
      //           wait_disp(Device:=AVANTI_INDIETRO, State:=LAVORO, Trace:=FALSE);
      //           //Blocco dispositivi di spinta
      //           Set_Disp(Ndisp:=BLOCCO_SBLOCCO, Stato:=LAVORO, TimerRisparmio:=TRUE);
      //           wait_disp(Device:=BLOCCO_SBLOCCO, State:=LAVORO, Trace:=FALSE);
      //         INTERMEDIO:
      //           //Comando di avanzamento spintore senza attesa
      //           Set_Disp(Ndisp:=BLOCCO_SBLOCCO, Stato:=RIPOSO, TimerRisparmio:=TRUE);
      //           wait_disp(Device:=BLOCCO_SBLOCCO, State:=RIPOSO, Trace:=FALSE);
      //           //Avanzamento spintore
      //           Set_Disp(Ndisp:=AVANTI_INDIETRO, Stato:=LAVORO, TimerRisparmio:=TRUE);
      //       END_CASE;
      //     END_IF;
      // // {
      // // #endif  //Fine SPINTORE_ULTIMA_CURVA
      // // }
    END_CASE;
  // {
  // #endif  //Fine macchine lineari
  // }
  ;
END_FUNCTION

FUNCTION_BLOCK FB_PosAsse

  VAR_EXTERNAL
    AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    UsoPLCOpen						          :BOOL;
  END_VAR

  VAR_EXTERNAL CONSTANT
    FIRST_AXIS_MASTER				      :INT;
  END_VAR

  VAR_INPUT
    Axis                : DB_ANY;
    AttesaResetAxes		  :BOOL;
    //axis              :PosAxis;
    //direction         :EnumDirection;
    positioningMode     :_EnumPositioningMode;
    position          :LREAL;
    //velocityType      :EnumVelocity;
    velocity          :LREAL;
    //positiveAccelType	:EnumAcceleration;
    positiveAccel			:LREAL;
    //negativeAccelType :EnumAcceleration;
    negativeAccel     :LREAL;
    //velocityProfile   :EnumProfile;
    //blendingMode      :EnumBlendingMode;
    //mergeMode         :EnumMergeMode;
    //nextCommand       :EnumNextCommand;
  END_VAR
  VAR
    dret							:DINT;
		axisInt						:INT;
		//thisCommand				:commandIDType;
		
		CURRENT_STATE			:INT:=GENERAL_STATE#STATE_INIT;
		AttesaFineResetAssi:FB_AttesaFineResetAssi;
  END_VAR
	VAR_OUTPUT
		OutputState:INT;
	END_VAR
	CASE CURRENT_STATE OF
		GENERAL_STATE#STATE_INIT:
			IF AttesaResetAxes=TRUE THEN
				IF (AttesaFineResetAssi.outputState<>GENERAL_STATE#STATE_END) THEN
					AttesaFineResetAssi();
				ELSE
					AttesaFineResetAssi();
					IF (UsoPLCOpen) THEN
						CURRENT_STATE:=_FB_PosAsse#STATE_POSASSE_PLCOPEN;
					ELSE
						//thisCommand:=_getCommandID();
						CURRENT_STATe:=_FB_PosAsse#STATE_POSASSE_NOPLCOPEN;
					END_IF;
				END_IF;
			ELSE
				IF (UsoPLCOpen) THEN
					CURRENT_STATE:=_FB_PosAsse#STATE_POSASSE_PLCOPEN;
				ELSE
					//thisCommand:=_getCommandID();
					CURRENT_STATe:=_FB_PosAsse#STATE_POSASSE_NOPLCOPEN;
				END_IF;
			END_IF;
			
      _FB_PosAsse#STATE_POSASSE_PLCOPEN:
			axisInt:=GetAxisFromTO(axis);
      IF positioningMode = _EnumPositioningMode#ABSOLUTE THEN

        IF (axisInt=-1) THEN
          axisInt:=GetMasterAxisFromTO(axis);
          AssiMaster[axisInt].MC_Axes.move_abs_param.Position :=position;
          AssiMaster[axisInt].MC_Axes.move_abs_param.Velocity:=velocity;
          AssiMaster[axisInt].MC_Axes.move_abs_param.Acceleration:=positiveAccel;
          AssiMaster[axisInt].MC_Axes.move_abs_param.Deceleration:=negativeAccel;
          AssiMaster[axisInt].MC_Axes.move_absolute := TRUE ;
        ELSE
          Axes[axisInt].MC_Axes.move_abs_param.Position :=position;
          Axes[axisInt].MC_Axes.move_abs_param.Velocity:=velocity;
          Axes[axisInt].MC_Axes.move_abs_param.Acceleration:=positiveAccel;
          Axes[axisInt].MC_Axes.move_abs_param.Deceleration:=negativeAccel;
          Axes[axisInt].MC_Axes.move_absolute := TRUE ;
        END_IF;

    ELSE
        IF (axisInt=-1) THEN
          axisInt:=GetMasterAxisFromTO(axis);
          AssiMaster[axisInt].MC_Axes.move_rel_param.Distance :=position;
          AssiMaster[axisInt].MC_Axes.move_rel_param.Velocity:=velocity;
          AssiMaster[axisInt].MC_Axes.move_rel_param.Acceleration:=positiveAccel;
          AssiMaster[axisInt].MC_Axes.move_rel_param.Deceleration:=negativeAccel;
          AssiMaster[axisInt].MC_Axes.move_relative:=TRUE;
        ELSE
          Axes[axisInt].MC_Axes.move_rel_param.Distance :=position;
          Axes[axisInt].MC_Axes.move_rel_param.Velocity:=velocity;
          Axes[axisInt].MC_Axes.move_rel_param.Acceleration:=positiveAccel;
          Axes[axisInt].MC_Axes.move_rel_param.Deceleration :=negativeAccel;
          Axes[axisInt].MC_Axes.move_relative:=TRUE;
        END_IF;
    END_IF;

			CURRENT_STATE:=GENERAL_STATE#STATE_END;
			
		_FB_PosAsse#STATE_POSASSE_NOPLCOPEN:

			(*dret:=_pos(axis:=axis, 
								 direction:=direction, 
								 positioningMode:=positioningMode, 
								 position:=position, 
								 velocityType:=velocityType, 
								 velocity:=velocity, 
								 positiveAccelType:=positiveAccelType,
								 positiveAccel:=positiveAccel,
								 negativeAccelType:=DIRECT,
								 negativeAccel:=negativeAccel,
								 velocityProfile:=velocityProfile, 
								 blendingMode:=blendingMode, 
								 mergeMode:=mergeMode, 
								 nextCommand:=IMMEDIATELY, 
								 commandId:=thisCommand);*)
			// IF (nextCommand=IMMEDIATELY) THEN
			// 	CURRENT_STATE:=GENERAL_STATE#STATE_END;
			// ELSE
			// 	IF NOT (dret=7001 OR dret=7002) THEN
					CURRENT_STATE:=GENERAL_STATE#STATE_END;
			// 	END_IF;
			// END_IF;
			
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK

/****************************************************************
 * WAITS UNTIL THERE IS NO LONGER ANY RESET COMMAND ON ALL AXES *
 ****************************************************************/

FUNCTION_BLOCK FB_AttesaFineResetAssi

  VAR_EXTERNAL
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
    pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
  END_VAR

  VAR_EXTERNAL CONSTANT
    TRC_WAIT_FOR_DRIVE_COMMUNICATION    :INT;
  END_VAR
  
  VAR
    posax : REF_TO TO_SynchronousAxis;
    i      :INT;
    dret  :DINT;
		
		CURRENT_STATE:INT:=GENERAL_STATE#STATE_INIT;
		CURRENT_AXIS:INT;
  END_VAR  

	VAR_OUTPUT
		OutputState:INT;
	END_VAR

	CASE CURRENT_STATE OF 
		GENERAL_STATE#STATE_INIT:
			CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
			CURRENT_STATE:=_FB_AttesaFineResetAssi#STATE_ATTESA_FINE_RESET_ASSI_CICLO_ASSI;
			
      _FB_AttesaFineResetAssi#STATE_ATTESA_FINE_RESET_ASSI_CICLO_ASSI:
			CURRENT_AXIS:=CURRENT_AXIS+1;
			IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
				IF pm[CURRENT_AXIS].ma_asse_abilitato=TRUE THEN
					CURRENT_STATE:=_FB_AttesaFineResetAssi#STATE_WAIT_ATTESA_AZIONAMENTO;
				END_IF;
			ELSE
				CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
				CURRENT_STATE:=GENERAL_STATE#STATE_END;
			END_IF;
			
      _FB_AttesaFineResetAssi#STATE_WAIT_ATTESA_AZIONAMENTO:
			posax := AsSynchronousAxisRef(Axes[CURRENT_AXIS].posax);
      IF posax^.StatusWord.%X2 THEN
				set_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION, Axis:=CURRENT_AXIS);
			ELSE
				reset_trace(Code:=TRC_WAIT_FOR_DRIVE_COMMUNICATION);
				CURRENT_STATE:=_FB_AttesaFineResetAssi#STATE_ATTESA_FINE_RESET_ASSI_CICLO_ASSI;
			END_IF;
	
		GENERAL_STATE#STATE_END:
			CURRENT_STATE:=GENERAL_STATE#STATE_INIT;
	END_CASE;
	OutputState:=CURRENT_STATE;
END_FUNCTION_BLOCK 

FUNCTION GetAxisFromTO:INT

  VAR_EXTERNAL
    Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;
  END_VAR

	VAR_INPUT
		Axis	: DB_ANY;
	END_VAR

	VAR
		i			:INT;
   posax : REF_TO TO_SynchronousAxis;
   LocAxis : REF_TO TO_SynchronousAxis;
	END_VAR

	GetAxisFromTO:=-1;
  LocAxis := AsSynchronousAxisRef(Axis);

	FOR i:=0 TO _MAXIS#MAXAXIS -1 DO
  posax:= AsSynchronousAxisRef(Axes[i].PosAx);
    IF (PosAx=LocAxis) THEN
		//IF (Axes[i].PosAx=Axis) THEN
			GetAxisFromTO:=i;
		END_IF;
	END_FOR;
END_FUNCTION 

END_NAMESPACE