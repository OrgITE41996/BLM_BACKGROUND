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
USING BACKGROUND_NAMESPACE_FB;
 
NAMESPACE BACKGROUND_NAMESPACE

PROGRAM BACKGROUND

    VAR_EXTERNAL
        Checksum_Plc_Safety_Hight          :WORD;
        Checksum_Plc_Safety_Low            :WORD;
        Configurazione_IO_to_PLC           :SINT;
         (*{
            #ifdef PROFINET
            }*)
        Configuration_IO                :SINT; //preset of IO configurations (0 = standard, 1 = optional io);
        PresencePnCoupler               :BOOL;
        CountPN_PN_Deactivated			:INT;
        O_PnCoupler_Disable             :BOOL;
    END_VAR
    
    VAR
      i                               :INT;
      dret                            :DINT; 
      NumeroAssi                      :INT;
      dwret                           :DWORD;
      dev                             :INT;
      Esito                           :BOOL;
      Primo_giro                      :BOOL:=TRUE;
      vel                             :LREAL;
      myRetDWORD                      :DWORD;
      Appo_swovr                      :REAL;
      iret                            :INT;
      TuttiFermi                      :BOOL;
      TuttiSpenti                     :BOOL;
      TmpInizioFermo                  :DATE_AND_TIME;
      AppoStatoGiallo                 :BOOL;
      AzionamentiON                   :BOOL;
      Delta                           :REAL;
      AppoStart                       :BOOL;
      AbilitazioneStartDaPulsantiera  :BOOL:=TRUE;
      Sensore1, Sensore2			  :LREAL;
      RisultatoByte					  :ARRAY[0..15] OF BYTE;
      TempoInvioDati				  :REAL;
          (*{#ifdef PLC_S7_1500}*)
      PLC_Active					  :BOOL:=FALSE;
      PN_PN_Deactivated				  :BOOL:=FALSE;
          (*{#endif}*)	//Fine PLC_S7_1500
      MCP483	:FB_MCP483;
        //   CmdManualiConEmergenza	:FB_CmdManualiConEmergenza;
        //   Change_OpMode	:FB_Change_OpMode;
        //   IndexAsseMaster	:FB_IndexAsseMaster;
        //   ModulazioneCoppiaW3W4W5W6	:FB_ModulazioneCoppiaW3W4W5W6;
        //   SicurezzeTaglioColtello	:FB_SicurezzeTaglioColtello;
        //   //SorveglianzaRotazioneLamaDiTaglio	:FB_SorveglianzaRotazioneLamaDiTaglio;
        //   GestioneSostegno	:FB_GestioneSostegno;
        //   VerificaSicurezzeMacchina	:FB_VerificaSicurezzeMacchina;
        //   ArrestaMotori	:FB_ArrestaMotori;
        //   DisableAssi	:FB_DisableAssi;
        //   PompaTaglioColtello	:FB_PompaTaglioColtello;
        //   RegolazioneSostegno	:FB_RegolazioneSostegno;
        //   SicurezzaRadar	:FB_SicurezzaRadar;
    END_VAR;
      {::ifdef PLC_S7_1500}
          IF Checksum_plc_safety_hight>WORD#0 OR Checksum_plc_safety_low>WORD#0 THEN
              //Attesa comunicazione con plc di sicurezza
              PLC_Active:=TRUE;
              Configurazione_IO_to_PLC:=Configuration_IO;
              IF PresencePnCoupler=FALSE THEN
                  //Se non è presente il PnPnCoupler, il segnale di disabilitazione verso
                  //il plc di sicurezza viene mantenuto ON per 100 cicli di background
                  IF CountPN_PN_Deactivated<100 THEN
                      CountPN_PN_Deactivated:=CountPN_PN_Deactivated+1;
                      O_PNCoupler_Disable:=TRUE;
                  ELSE	
                      //Al centesimo ciclo di background si mette a OFF il segnale di 
                      //disabilitazione del PnPnCoupler
                      O_PNCoupler_Disable:=FALSE;
                  END_IF;
              END_IF;
          ELSE	
              //Mentra si attende la comunicazione con il plc di sicurezza si resetta il contatore 
              //per il mantenimento dell^uscita di disabilitazione del PnPnCoupler
              CountPN_PN_Deactivated:=0;
          END_IF;
    {::endif} //FINE PLC_S7_1500
   
      
    //   WHILE MCP483.outputState<>GENERAL_STATE#STATE_END DO
    //     MCP483();
    //   END_WHILE;
      MCP483();
    END_PROGRAM
END_NAMESPACE