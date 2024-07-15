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
USING _STATE_CONST;

CONFIGURATION _Configuration_Global
    VAR_GLOBAL
        // {
        // #ifdef DH4010
        // }
          TabellaStati	:ARRAY[TABLE#TABLE_LX ..TABLE#TABLE_RX ] OF TabStato;
        // {
        // #else  //Fine DH4010
        // }
        //   TabellaStati  :ARRAY[0..MAXSTATI-1] OF Riga_stato;
        // {
        // #endif  //Fine NO DH4010
        // }
        InputWithoutAddress             :DINT;
        CURRENT_STATE				    :INT:=_STATE_CONST.GENERAL_STATE#STATE_INIT; //canc after Test
        //WriteOutcome 	            :StructRetUnitDataSetCommand;
        //ReadingOutcome	        :StructRetUnitDataSetCommand;
    ///ENABLEAX
        ExtendedSafetyState 	        :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;
        AxisType          	            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF INT;
        LoadRpmAz      	                :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
        ScrewPitchAZ       	            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
        EngineRpmAz      	            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;
        PerformSafetyReset	            :BOOL;
        O_EmergencyFromSimotion		    :BOOL;	//Only to be able to compile machines without safety PLCs

    ///_VARGLOBAL
        AxisKinematics                  :ARRAY[_FIRST_KIN_AXES#FIRST_KIN_AXES.._MAX_KIN_AXES#MAX_KIN_AXES-1] OF Kinematics;
        TimerSistema                    :ARRAY[0..MAX_TIMER_SISTEMA-1] OF Timer;
        StatusStandBy                   :BOOL;
        MessageStop                     :BOOL;
        AssiMaster                      :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MasterAxis;
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
        i_sys_ReadingSinamicsInUse      :BOOL;
        i_sys_InUseWritingSynamics      :BOOL;
        i_sys_TrackingErrorX1           :REAL;
        i_sys_TrackingErrorX2           :REAL;
        ProTube                         :Produzione;
        System_Timer                    : LDATE_AND_TIME; // Timer di sistema
        i_sys_MotorState				:ARRAY[(_FIRST_AXIS#FIRST_AXIS)..(_FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1)] OF REAL; 
        SystemStarted                   :BOOL;
        Started						    :BOOL;
        ElencoTrace                     :ARRAY[0..MAXTRACE-1] OF Trace;
        i_sys_Trace_ON                  :BOOL;  //True per far visualizzare la nuova trace inserita, sarà BlmMottion a metterlo a FALSE
        i_sys_Error_ON                  :BOOL;  //Indica a VGP che c^è un cambiamento nella situazione dei messaggi di errore
        i_sys_current_trace             :DINT;			// TRACE attiva
        i_sys_current_trace_axis        :INT;  // Parametro della trace attiva
        i_sys_trace_number              :DINT;
        i_sys_first_on	                :BOOL:=TRUE;
        i_sys_RunOk                     : INT;  // 0 = NON IN RUN
                                                // 1 = INIZIALIZZAZIONE E RUN ATTIVATI
                                                // 2 = INIZIALIZZAZIONE COMPLETATA
                                        // Deve essere forzato dalla parte alta
        i_sys_error_grave              : INT;
        i_sys_ErrorLubr                :BOOL:=FALSE;
        I_Barrier                      :BOOL;
        i_sys_Motors_Verified	       :BOOL:=FALSE; //Alla perdita delle ritentive verifica lo stato di impostazione dei motori presenti in macchina
        i_sys_error                    :DINT;	// codice errore
        i_sys_error_number	           :DINT;	//Numero di errori presenti nel sistema
        Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
        AlarmsQueue                    : ARRAY[0..MAXALARMS_LIFO-1] OF Alarms;
        IO_Link_Input,
        IO_Link_Output				   :WORD;
        i_sys_ACCESSLEVEL              : INT := 7;	// LIVELLO DI ACCESSO 
                                                    // è il minore fra sw_ACCESSLEVEL e hw_ACCESSLEVEL
                                                    // 0 = PROGRAMMATORE COSTRUTTORE
                                                    // 1 = COSTRUTTORE
                                                    // 2 = MIS COSTRUTTORE
                                                    // 3 = SAT COSTRUTTORE
                                                    // 4 = MANUTENTORE CLIENTE
                                                    // 5 = OPERATORE QUALIFICATO
                                                    // 6 = OPERATORE CLIENTE
                                                    // 7 = UTENTE QUALSIASI
        i_sys_CmdVGPCSettings          :BOOL:=FALSE; // Disables the system startup waiting to allow
                                                    // the transfer of the command from VGO3D of the machine configuration.
                                                    // It's meant to prevent CPU stops for premature piece transfers,
                                                    // but it should allow some transfers to be executed in order to
                                                    // complete the system startup (CMD disabling motors not present in the machine).
          
            ////TO Defenition
        PosAx                           :DB_ANY;
        TEST_S                          :INT;//TEST
        O_PulserTwoHands                : BOOL;
        i_sys_DpCoupler                 :BOOL:=FALSE;
        i_sys_PnCoupler                 :BOOL:=FALSE;
        O_PnCoupler_Disable             :BOOL;
        (*{
        #ifndef SELEZIONE_DOPPIO_PNPN
        }*)
        O_PnCoupler_2_Disable		    :BOOL;	//Required for machine versions that do not include the variable in their ADDRESS LIST.
        (*{
        #endif	//Fine NO SELEZIONE_DOPPIO_PNPN
        }*)
        ChecksumPlcSafety               :UDINT;
        i_sys_EnableBusy                :BOOL:=FALSE; //Se a TRUE indica che è in corso il programma
                                                    //ENABLE_AX e quindi non può essere eseguito
                                                    //il reset
        i_sys_Busy                      :BOOL:=FALSE;//Viene messa TRUE da BlmMotion al momento
                                                    //del trasferimento dei PM e viene resettata
                                                    //da Simotion alla fine della copia ram/rom
        InitializeIOExecuted            :BOOL; //Serves to execute IO initialization only once per cmd WRITE IN CN
        i_sys_error_blm			        :Alarms;
        TimerSystem                     :ARRAY[0..MAX_TIMER_SYSTEM-1] OF Timer;
        i_sys_activedevice              : INT;  //Dispositivo selezionato per movimenti in JOG
                                        // da 0..31 asse
                                        // da 32 a 64 dispositivo IO
                                        // -1 = INDEFINITO
        i_sys_devicecol_from_vgp : INT:=UNDEF_DEVICE; // richiesta attivazione DEVICE da VGP
                                        // E^ il numero di dispositivo/asse secondo l^ordine della colonna di TABELLA STATI

        //Velocità massime sicure per COMFORT3 nell^ordine di VGP per lettura da parte alta                                
        i_sys_OPMODE                    : INT:=0;	// MODO OPERATIVO
                                        // 0 = JOG
                                        // 1 = REFERENCE
                                        // 2 = MDA
                                        // 3 = AUTO
        (*{
        #ifdef FOUR_RUNNER_PLAUNO
        }*)
        RejectTypeBerg						:BOOL;	//True=esegue scarto spezzone tubo secondo specifiche cliente Berg SW00001253
        (*{
        #endif  //Fine FOUR_RUNNER_PLAUNO
        }*)
        i_sys_swOVR                         : LREAL := 100;  // OVERRIDE SOFTWARE - VALORE 0 - 100
        i_sys_CHANSTATUS                    : INT :=0;	// STATO DEL SISTEMA
                                                        // 0 = Canale in RESET - IN JOG è sempre 0
                                                        // 1 = Canale attivo - IN REF, MDA e AUTO attivi
                                                        // 2 = Canale interrotto
        TaskCyclesActive                     :BOOL;
        i_sys_current_prog	                 :INT; // Programma corrente
        i_sys_OVRSTEP_DEFAULT               : LREAL; //value of OVR_STEP calculated in INITVAR on the^slowest axis
        i_sys_acc_master                    : REAL := 10000;  //Master acceleration
        i_sys_OVRSTEP                       : LREAL := 1000; // Number of steps (*2ms) to be able to make the master accelerate or decelerate
        i_sys_sense_curve                   :INT;		//1=Macchina DX  -1=Macchina SX
        i_sys_control_incongruity_encoder   :BOOL :=TRUE; 
        i_sys_statusencoder                 :ARRAY[(_FIRST_AXIS#FIRST_AXIs)..(_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1)] OF REAL;   
        Configurazione_IO_to_PLC            :SINT;
        Strobe_Config_IO                    :BOOL;
        (*{
        #ifdef ALIMENTATORE_ACTIVE_LINE
        }*)
        error_Alim							:BOOL;	//True se l^alimentatore è ok
        (*{
        #endif	//Fine ALIMENTATORE_ACTIVE_LINE
        }*)
        PercentageTorqueSet          :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1] OF LREAL;
        AxisInLimitationCouple		 :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1] OF BOOL;
        i_sys_Param_RW               :INT;   // 1=READ 2=WRITE
        PM_Modified                  :BOOL:=FALSE;    //Flag per update dei parametri macchina
                                                    //a seguito modifica da BLMMotion
        i_sys_Copy_RAM_ROM           :BOOL:=FALSE;
        i_sys_Param_AX_Number		 :INT;		// da 0..MAXASSI-1   999= ALIMENTATORE
        i_sys_Param_Number			:LREAL; // Numero parametro
        i_sys_Param_Index           :LREAL; // Index parametro
        i_sys_Param_Value			:LREAL; // Valore parametro
        i_sys_InUseReadingSinamics  :BOOL;
        StartInRun                  :BOOL;
        O_Switch_Second_Barrier     :BOOL;
        Strobes_For_Remote          :BOOL;
        FlagEnableCamming			:BOOL;  //True se è possibile mettere in camming gli assi
        CammeInterpolate            :BOOL; // TRUE se le camme sono già state interpolate
        StatusTrace                 :BOOL;
        StatusWaitingMaterial       :BOOL;
        Status_Waiting_Lubricant    :BOOL;
        SpintaB2Eseguita            :BOOL;
        i_sys_Z2_Roland				:LREAL:=-MY_BIG_VALUE_REAL;	//MY_BIG_VALUE_REAL => attiva la sorveglianza in IPO dell^ingresso che indica che la ricerca
                                                                //saldatura Roland si è conclusa.
                                                                //Con valore minore di MY_BIG_VALUE_REAL-1 indica la posizione di Z2 dove è stata rilevata la saldatura
        (*{
        #ifdef CURVATURA
        }*)
        MemoPosAxisPreViolation:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF REAL;  
        (*{
         #endif  //Fine CURVATURA
         }*) 
        TimeReadingMotors          :REAL;
        PosX1TubeOnPalette          :LREAL;  //Posizione X1 quando si e^ attivato il tubo su paletta
        EnableTubeOnPalette          :SINT; //0 disabilita nell^IPO la misura X1 per la misura della posizione del tubo
                                            //1 abilita nell^IPO attesa attivazione del sensore tubo su paletta
                                            //2 indica l^avvenuta misura della posizione X1 dove il tubo e^ stato rilevato
                                            //3 indica l^avvenuto arresto dell^asse X1 dopo la misura del terminale del tubo
        LoadRobotParallel                   :BOOL;
        O_ReleaseTube                       :BOOL;
        i_sys_CmdLowerMultiSupport          :SINT:=SINT#0; 
        (*{
        #ifdef AL_BLM
        }*)
          AL5LoadExecuted				    :BOOL;
          AL42LoadExecuted			        :BOOL;
          AL42PlusLoadExecuted	            :BOOL;
        (*{
        #endif  //Fine AL_BLM
        }*)
        i_sys_AL_Robot                      :BOOL:=FALSE;  //True indica che il robot ha completato il prelevio del tubo dall^AL
        BlockSupport                        :BOOL:=FALSE;	//A TRUE indica che il background non deve interferire
                                                            //con il sostegno tirante in quanto è comandato dalla
                                                            //routine di cambio pila per la SMART-A
        i_sys_Change_DX_SX                  :BOOL:=FALSE;   //TRUE esegue la presa index Y e Q alla partenza del CN

        W3ControlTorque ,        
        W4ControlTorque ,    
        W5ControlTorque,        
        W6ControlTorque                     :BOOL;  //True asse in modulazione di coppia durante curvatura 

        AxesResetInProgress                 :DWORD;
        AllargaStandStill                   :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1] OF BOOL;
        i_sys_PresenceMatrix			    :BOOL;
        i_sys_CalculationBoundariesInCourseX1	    :BOOL;
        i_sys_CalculationBoundariesInCourseX2	    :BOOL;
        i_sys_ResetPressedDriver                    :BOOL;
        OPS_Active                                  :BOOL;  //True gestione speciale pezzo seguente per consentire funzionamento
                                                            //macchina con sistema OPS di Salvagnini
        OPS_StartDate                               :BOOL;  //True se l^operatore ha dato lo START per iniziare il pezzo seguente OPS
        Appo_global                                 :INT;
        (*{
        #ifdef SMART
        }*)

          LatoBasculante            :INT;	//DX=0 SX=1
        
        (*{
        #else  //Fine SMART
        }*)
        i_sys_AutoSetting					:INT;
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

        i_sys_StatusIOoptional    :BOOL:=FALSE;
        i_sys_StatusIOAL42         :BOOL:=FALSE;
        i_sys_StatusIOAL5          :BOOL:=FALSE;
        i_sys_StatusIOAL5Optional :BOOL:=FALSE;

        CountPN_PN_Deactivated					:INT:=0;


        i_sys_hw_ACCESSLEVEL     : INT :=7; // ACCESS_LEVEL da pulsantiera macchina
        i_sys_sw_ACCESSLEVEL     : INT; // ACCESS_LEVEL da PASSWORD SOFTWARE
        s_sys_user_password         :STRING[12];
        i_sys_monitoring            : INT := 0;  // 0 = DISATTIVO
                                                // 1 = ATTIVO 
        i_sys_varinc : INT;	//Variabile di incremento di passo in JOG
                                                // 0 = Non attivo INC
                                                // 1 = 1 um
                                                // 2 = 10 um
                                                // 3 = 100 um
                                                // 4 = 1000 um = 1 mm
                                                // 5 = 10 mm
        R_VarInc        :R_TRIG;  //Fronte di salita del tasto VAR_INC
        (*{
        #ifdef EMULAZIONE
          }*)
        O_PnCoupler_Profisafe    : BOOL;
        O_ModaleHoldToRun        : BOOL;

        Clk_PartenzaPostMortem   :BOOL;
        R_PartenzaPostMortem	 :R_TRIG;	//Fronte salita abilitazione post mortem per servizio
        
        Clk_NastroPieno			 :BOOL;
        R_NastroPieno			 :R_TRIG;	//Fronte salita nastro trasportatore pieno
        
        Clk_AbilitazioneHoldToRun	:BOOL;
        F_AbilitazioneHoldToRun   :F_TRIG;	//Fronte discesa hold to run

        R_SalvaGuardia	:R_TRIG;
        Clk_HoldToRun							:BOOL;
        R_HoldToRun								:R_TRIG;
        F_HoldToRun    						    :F_TRIG;
        R_RiarmoAzionamenti		                :R_TRIG;
        F_RiarmoAzionamenti		                :F_TRIG; 
        Clk_RiarmoAzionamenti	                :BOOL;
        
        R_Plus		                            :R_TRIG; //fronte salita PLUS
        F_Plus                                  :F_TRIG; //fronte discesa PLUS
        Clk_Plus                                :BOOL;
        
        R_Minus                                 :R_TRIG; //fronte salita MINUS
        F_Minus                                 :F_TRIG; //fronte discesa MINUS
        Clk_Minus	                            :BOOL;
        R_Rapid                                 :R_TRIG; //fronte salita RAPID
        F_Rapid                                 :F_TRIG; //fronte discesa RAPID
        R_Start                                 :R_TRIG; //fronte salita START
        F_Start                                 :F_TRIG; //fronte discesa START
          
        R_Reset                                 :R_TRIG; //fronte salita RESET
        F_Reset                                 :F_TRIG; //fronte discesa RESET
        clk_Reset	                            :BOOL;
        R_JOG			                        :R_TRIG; //fronte salita 
        F_JOG			                        :F_TRIG; //fronte discesa 
        R_MDA			                        :R_TRIG; //fronte salita 
        F_MDA			                        :F_TRIG; //fronte discesa 
        R_REF			                        :R_TRIG; //fronte salita 
        F_REF 		                            :F_TRIG; //fronte discesa 
      
        R_AUTO                                  : R_TRIG; //fronte salita 
        F_AUTO                                  : F_TRIG; //fronte discesa 

        F_T0 						            :F_TRIG; //fronte discesa T0 Pulsantiera 
        F_T1						            :F_TRIG; //fronte discesa T1 Pulsantiera 
        F_T2						            :F_TRIG; //fronte discesa T2 Pulsantiera 

        R_ActiveDevice                          :R_TRIG; //Fronte di salita richiesta DEVICE da MCP 
        R_ActiveAsse		                    :R_TRIG;
        R_ActiveDisp                            :R_TRIG;

        R_StartPuls     :R_TRIG;  //Fronte salita START Pulsantiera 
        F_StartPuls     :F_TRIG;  //Fronte discesa START Pulsantiera
        clk_StartPuls   :BOOL;    //On per tutta la durata dell^evento

        O_ShutdownInCourse                        :BOOL;
        kvc_plus						          :BOOL;
        kvc_minus						          :BOOL;
        kvc_reset						          :BOOL;
        kvc_start_cycle			                  :BOOL;
        AreaRadarError							  :BOOL;
        i_sys_OngoingReset                        :BOOL;
        FlgSetFirstStartVelReduced                 :BOOL;	//Serve per decidere se al primo start occorre effettuare i movimenti
        

                                                                        //TRUE quando è stato eseguita l^attivazione del modo operativo automatico		
                                                                        //FALSE quando si disattiva il modo automatico, oppure si conclude il primo start
        i_sys_Carm7CaricoContinuo               :BOOL:=FALSE;

        Clk_carm7caricocontinuo		            :BOOL;
        F_carm7caricocontinuo			        :F_TRIG;	//Fronte discesa del ciclo continuo del CARM7 o degli AL5

        RigaStartLoaderParallelo                :INT:=-1;
        RigaEndLoaderParallelo                  :INT:=-1;
        RichiestaArresto                        :BOOL;
        CreateBlackBox					        :BOOL; 	//Indica al servizio BLMBlackBox se generare log per gli allarmi attivi o no;
        DatiSelExtProd					        :SelExtProd;
        i_sys_PezzoSeguenteDisattivato	        :BOOL;	//Viene messo a TRUE quando si disattiva il pezzo seguente. Serve per 
                                                                                //fare in modo di impostare la variabile i_sys_modopezzoseguente a MODO_PEZZO_STANDARD
                                                                                //al cambio di modo
        F_InpCodiceTabellaOK	                :F_TRIG; //fronte discesa InpCodiceTabellaOK
        singleblock			                    :Flipflop; 
        i_sys_singleblockactive                 : BOOL  :=FALSE;
        i_sys_SenzaTubo                         : BOOL  :=FALSE;
        i_sys_chan_request                      : INT :=0;  // STATO DEL SISTEMA
                                                            // 0 = Non richiesto cambio modo 
                                                            // 1 = Richiesto nuovo modo operativo
        i_SYS_OPMODE_request                    : INT :=0;  // MODO OPERATIVO RICHIESTO
                                                            // 0 = JOG
                                                            // 1 = REFERENCE
                                                            // 2 = MDA
                                                            // 3 = AUTO
        i_sys_MDA_ABILITATO                     : INT := 0; //Abilita la modalità MDA che altrimenti non è attiva
                                                            // Deve essere forzato dalla parte alta
        i_sys_stopcond                          : INT   :=0; // 0=NO STOP, 1=STOP ATTIVO
        Key_Disp                                :ARRAY[0.._MAX_KEY_DISP#MAX_KEY_DISP-1] OF INT;
        kvc_stop_cycle			                :BOOL;
        TaskCicliAttivo                         :BOOL;
        ErroreLubrMacchina                      :USINT;  //0=OK, 1=Errore livello, 2=Errore ciclo lubrificazione
        Log_assi								:Trace_Asse;
        O_Vofa_1_2_Enable               		: BOOL;
        i_sys_ResetPremutoSafety                :BOOL;
        i_sys_ResetPremutoDriver                :BOOL;
        
        i_sys_OVR                               :LREAL:=100; // OVERRIDE   - VALORE 0 - 100
        i_sys_hwOVRdaTastiera                   :INT:=100; // OVERRIDE HARDWARE impostato da tastiera
        i_sys_hwOVRdaRemoto                     :INT:=100; // OVERRIDE HARDWARE impostato da remoto
        i_sys_hwOVR                             :LREAL:=100; // OVERRIDE HARDWARE - VALORE 0 - 100
        i_sys_hwovr_appo					    :LREAL;
        i_sys_UomoMorto						    :BOOL:=FALSE;

        i_sys_OVR2                              : LREAL := 100; // * OVERRIDE MANDRINO
        i_sys_hwOVR2                            : LREAL := 100; // OVERRIDE MANDRINO HARDWARE - VALORE 50 - 100
        i_sys_swOVR2                            : LREAL := 100; // OVERRIDE MANDRINO SOFTWARE - VALORE 0 - 100

        old_i_sys_activedevice:INT; 

    //   {
    //   #ifdef SAFETY_INTEGRATED
    //   }
        i_sys_UomoMortoDelay                    :BOOL:=FALSE;	//Stato dell^uomo morto impostato dal PLC safety
        i_sys_hwOVRLimit                        : REAL := OVERRIDE_STANDARD;
        i_sys_hwOVRLimit_for_scaling            : REAL := OVERRIDE_STANDARD;
        Temperatura                             :LREAL;
        i_sys_tOVR                              : LREAL := 100;  // OVERRIDE PER RIDUZIONE VELOCITA^ DA TEMPERATURA - VALORE 0 - 100
        ovr,ovr2                                :LREAL;
        FlgRiduzVelVGP					        :BOOL;	//True se nel pezzo attivo è presente lo stop point 76 che indica che la traiettoria 
                                                        //è stata creata da VGP3D per non oltrepassare i limiti delle velocità sicure
                                                        //imposti dal COMFORT3
                                                        // {
                                                        // #endif	//Fine SNERVAMENTO
                                                        // }
        FlgPrimoStartFatto				        :BOOL;  //True alla fine del primo start
        FlgPezzoInEsecuzione			        :BOOL;  //True alla fine del primo start, False alla fine del pezzo
        i_sys_OverrideMarcatura				    :BOOL;	//True con marcatura in corso per forzare override 100% al primo start
        i_sys_svuotare_cassa	                :BOOL;	//Vale true quando occorre far comparire il messaggio di svuotare la cassa,
                                                        //invece del classico messaggio di attesa start
        i_sys_Machine_Process		            :WORD;
        Sostegno2StopBackground	                :SINT:=SINT#-1;	//-1=Sostegno2 governato dal background
                                                                        // 0=Sostegno2 bloccato in posizione bassa da stop point mediante 
                                                                        //	 comando STOPP_ABBASSO_SOSTEGNO_TUBO_2
                                                                        // 1=Richiesta di sospensione della gestione del Sostegno2 eseguita
                                                                        //	 dal background
        i_sys_TempoWarning                      :REAL;
        i_sys_flg_attesa_slitta_1	            :BOOL:=FALSE;
        i_sys_flg_attesa_slitta_2	            :BOOL:=FALSE;
        i_sys_PROGSTATUS                        : INT :=0;	// STATO DEL PROGRAMMA
                                                                        // 0 = Programma in RESET - IN tutti i modi operativi <> AUTO è sempre 0
                                                                        // 1 = Programma in fase di PRIMOSTART
                                                                        // 2 = Programma in esecuzione tabella punti
                                                                        // 3 = Programma in attesa START a inizio tabella punti
        i_sys_CR_SENZA_SICUREZZE                :BOOL:=FALSE;
        StopPezzoSeguente                       :BOOL;
        RichJobSospeso                          :BOOL;
        i_sys_FlagCoccodrilloStopPoint          :BOOL:=FALSE;      //True se in automatico viene attivato lo STOP POINT 
        i_sys_PosFinale							:ARRAY[(_FIRST_AXIS#FIRST_AXIS) .. (_FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS -1)] OF LREAL;  //Serve per calcolare la Distance to go 
        AsseInLimitazioneCoppia		            :ARRAY[_FIRST_AXIS#FIRST_AXIS .. _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1] OF BOOL;
        i_sys_AttivazioneNastro		            :BOOL:=FALSE;
        i_sys_ViolazioneProgrammata             :BOOL:=FALSE;
        i_sys_mem_weld                          :ARRAY[0..1] OF LREAL;   //serve per ripartenza ricerca weld dopo violazione barriera
        WebApp                                  :WebApplication;
        MinutiRisparmioEnergetico               :INT;
        PagTabCorrezione                        :BOOL;  //True se è attiva la pagina tabella di correzione
        i_sys_StatoIOAL5_old			        :BOOL:=FALSE;
        i_sys_StatoIOAL5Opzionale_old           :BOOL:=FALSE;
        i_sys_StatoIOAL5                        :BOOL:=FALSE;
        i_sys_StatoIOAL5Opzionale               :BOOL:=FALSE;
        i_sys_StatoFestoOpzionale               :BOOL:=FALSE;
        i_sys_DL_PN1						    :BOOL;	//True se è abilitato il modulo Keyence per la lettura dell^angolo del ritorno elastico
        OPS_Attivo                              :BOOL;  //True gestione speciale pezzo seguente per consentire funzionamento
        OutputSenzaIndirizzo                    :DINT;
/********************************
* VARIABLES FOR SUPPORT LIMITS *
********************************/
       
        i_sys_enable_Alimen 				    :BOOL;
        BlmMxToolsOffPezzo                      :ARRAY[0..MAXTOOLSOFFPEZZO-1] OF RigaUtDispositivo; //Tabella dati utensili per ciascun dispositivo

        
        
/********
* TEST *
********/

Dummy                                         : int;
Start_read								: BOOL;
Dummy_LREAL_1                            :LREAL;
Dummy_LREAL_2                            :LREAL; 
Dummy_LREAL_3                            :LREAL;
Dummy_LREAL_4                            :LREAL; 

Dummy_INT_1                              :INT;
Dummy_INT_2                              :INT;
Dummy_INT_3                              :INT;
Dummy_INT_4                              :INT;
    
END_VAR	

/*************************************
* SINGLE AXIS _MC FUNCTIONAL BLOCKS *
*************************************/

VAR_GLOBAL
mcReset									:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1-1] OF MC_Reset;
mcHome									:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF MC_Home;
mcReset_Master					        :ARRAY[FIRST_AXIS_MASTER..FIRST_AXIS_MASTER+_CONST_VAR_DEF._MAXAXIS_MASTER#MAXAXIS_MASTER-1] OF MC_Reset;
END_VAR

/********************************
* PUSH-BUTTON PANEL BACKGROUND *
********************************/


        VAR_GLOBAL   
        //Drive_address
            Axis_Address                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF DINT;
            Stato_Out                                 :ARRAY[0..MAXDIGITALOUTPUT] OF Struttura_Stato_Out;
        END_VAR   

/***********************
 * VAR_GLOBAL CONSTANT *
 ***********************/

 VAR_GLOBAL
 INC  :ARRAY[0..5] OF LREAL:=[0,0.001,0.01,0.1,1,10];
 
 COL_DEVICE_STATI : ARRAY[0.._MAXCOL_DEVICE#MAXCOL_DEVICE-1] OF INT:=[X1,
                                                                      Y1,
                                                                      Z1,
                                                                      Q1,
                                                                      V1,
                                                                      W3,
                                                                      W4,
                                                                      U4,
                                                                      U2,
                                                                      CLAMP,
                                                                      PALETTA,
                                                                      SUPPORT_TIRANT,
                                                                     
                                                                      
                                                
                                                                      
                                                                      LOCKING_DXSX,
                                                                      DISAPPEARANCE_ROLLERS,
                                                                      CLAMP_CARGE,
                                                                    
                                                                      
                                                                      -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
                                                                      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
{::ifdef BASCULANTE_ELETTRICO}                                                                      
COL_DEVICE_STATI : ARRAY[0.._MAXCOL_DEVICE#MAXCOL_DEVICE-1] OF INT:=[B2];
{::else}  //Fine BASCULANTE_ELETTRICO
//COL_DEVICE_STATI : ARRAY[0.._MAXCOL_DEVICE#MAXCOL_DEVICE-1] OF INT:=[BASCULANTE];
{::endif}  //Fine NO BASCULANTE_ELETTRICO
{::ifdef BASCULANTE_ELETTRICO}
COL_DEVICE_STATI : ARRAY[0.._MAXCOL_DEVICE#MAXCOL_DEVICE-1] OF INT:=[-1];
{::endif}
MODE_LEVEL                          :ARRAY[-1..5] OF INT:=[INT#7,INT#7,INT#5,INT#4,INT#7,INT#7,INT#7];
                                                                      //Costanti per definizione dei timer 
ADDRES_OUT_STOPP                    : ARRAY[PRIMO_STOP_POINT_SET_OUT..ULTIMO_STOP_POINT_SET_OUT] OF INT:=[OUT_01_STOP_POINT,OUT_02_STOP_POINT,OUT_03_STOP_POINT,OUT_04_STOP_POINT,OUT_05_STOP_POINT,OUT_06_STOP_POINT,
                                                                      OUT_07_STOP_POINT,OUT_08_STOP_POINT,OUT_09_STOP_POINT,OUT_10_STOP_POINT,OUT_11_STOP_POINT,OUT_12_STOP_POINT,
                                                                      OUT_13_STOP_POINT,OUT_14_STOP_POINT,OUT_15_STOP_POINT,OUT_16_STOP_POINT,OUT_17_STOP_POINT,OUT_18_STOP_POINT,
                                                                      OUT_19_STOP_POINT,OUT_20_STOP_POINT,OUT_21_STOP_POINT,OUT_22_STOP_POINT,OUT_23_STOP_POINT,OUT_24_STOP_POINT]; 
TOLLERANZA_SINCRONIZZAZIONE : ARRAY[_FIRST_AXIS#FIRST_AXIS.._CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF INT:=[100,
                                                                      5,
                                                                      5,
                                                                      20,
                                                                      5,
                                                                      20,
                                                                      20,
                                                                      5,
                                                                      5,
                                                                      5,0,0,0,0,0,0,0]; 
KEY_ASSI : ARRAY[0..MAX_TASTI_ASSI-1] OF INT:=[X1,
                                                                      Y1,
                                                                      Z1,
                                                                      Q1,
                                                                      V1,
                                                                      W3,
                                                                      W4,
                                                                      U4,
                                                                      U2,
                                                                     
                                                                      -1,
                                                                      -1,
                                                                      -1,
                                                                      -1,
                                                                      L1,0]; 
{::ifdef ASSE_Z2}                                                                      
//KEY_ASSI : ARRAY[0..MAX_TASTI_ASSI-1] OF INT:=[Z2];
{::else}  //Fine ASSE_Z2
KEY_ASSI : ARRAY[0..MAX_TASTI_ASSI-1] OF INT:=[-1];
{::endif}  //Fine ASSE_Z2
    // GRAY TO DEC CONVERTER      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25, 26,27, 28, 29, 30, 31
    GRAY_OVR:ARRAY[0..31] OF INT:=[0, 0, 2, 1,10, 8, 4, 6,80,75,60,70,20,30,50,40, 0, 0, 0, 0, 0, 0, 0, 0,85,90,100,95,100,100,100,100];
    
    // GRAY TO DEC CONVERTER2     0, 1, 2, 3, 4, 5, 6, 7,  8,  9, 10, 11,12,13, 14,15
    GRAY_OVR2:ARRAY[0..15] OF INT:=[50,50,60,55,80,75,65,70,120,115,105,110,85,90,100,95];                                                                                                                                
 END_VAR

 VAR_GLOBAL RETAIN
  counter: Lreal;
 End_var



END_CONFIGURATION