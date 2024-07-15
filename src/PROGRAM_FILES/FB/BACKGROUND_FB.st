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
USING _FC_ACCESSLEVEL_FC_NAMESPACE;
USING _UTILITIES_FB;
USING _COMM_GLOBAL_FC;
USING _FB;
USING _AUTO_FC;
USING _LoadConfigurationTest;

NAMESPACE BACKGROUND_NAMESPACE_FB

/**********************************************************************
 * MACHINE BUTTON PANEL READOUT FUNCTIONS AND SYSTEM VARIABLE SETTING *
 **********************************************************************/

FUNCTION_BLOCK FB_Mcp483


    VAR_EXTERNAL
        
        mcpi_spindle_stop : BOOL;
        mcpo_spindle_stop  : BOOL;
        mcpi_10000: BOOL;
        mcpi_1000  : BOOL;
        mcpi_100 : BOOL;
        mcpi_10  : BOOL;
        mcpi_1  : BOOL;
        mcpi_NCstart  : BOOL;
        mcpi_minus  : BOOL;
        mcpi_plus  : BOOL;
        mcpi_rapid  : BOOL;
        mcpi_reset  : BOOL;
        mcpi_AUTO  : BOOL;
        mcpi_JOG : BOOL;
        mcpi_REF  : BOOL;
        mcpi_MDA  : BOOL;
        mcpi_VARINC  : BOOL;
        mcpi_ax0  : BOOL;
        mcpi_ax1  : BOOL;
        mcpi_ax2  : BOOL;
        mcpi_ax3  : BOOL;
        mcpi_ax4  : BOOL;
        mcpi_ax5  : BOOL;
        mcpi_ax6  : BOOL;
        mcpi_ax7  : BOOL;
        mcpi_ax8  : BOOL;
        mcpi_ax9  : BOOL;
        mcpi_ax10  : BOOL;
        mcpi_ax11  : BOOL;
        mcpi_t0  : BOOL;
        mcpi_t1  : BOOL;
        mcpi_t2  : BOOL;
        mcpi_t3 : BOOL;
        mcpi_t4  : BOOL;
        mcpi_t5  : BOOL;
        mcpi_t6  : BOOL;
        mcpi_t7 : BOOL;
        mcpi_t8  : BOOL;
        mcpi_t9  : BOOL;
        mcpi_t10 : BOOL;
        mcpi_t11  : BOOL;
        mcpi_t12  : BOOL;
        mcpi_t13 : BOOL;
        mcpi_t14  : BOOL;

        i_sys_ACCESSLEVEL     : INT;	// LIVELLO DI ACCESSO
                                        // è il minore fra sw_ACCESSLEVEL e hw_ACCESSLEVEL
                                        // 0 = PROGRAMMATORE COSTRUTTORE
                                        // 1 = COSTRUTTORE
                                        // 2 = MIS COSTRUTTORE
                                        // 3 = SAT COSTRUTTORE
                                        // 4 = MANUTENTORE CLIENTE
                                        // 5 = OPERATORE QUALIFICATO
                                        // 6 = OPERATORE CLIENTE
                                        // 7 = UTENTE QUALSIASI
        i_sys_hw_ACCESSLEVEL     : INT; // ACCESS_LEVEL da pulsantiera macchina
        i_sys_sw_ACCESSLEVEL     : INT; // ACCESS_LEVEL da PASSWORD SOFTWARE
        i_sys_monitoring            : INT;  // 0 = DISATTIVO
                                            // 1 = ATTIVO
        i_sys_error_number	:DINT;	//Numero di errori presenti nel sistema
        i_sys_varinc : INT;	//Variabile di incremento di passo in JOG
                                              // 0 = Non attivo INC
                                              // 1 = 1 um
                                              // 2 = 10 um
                                              // 3 = 100 um
                                              // 4 = 1000 um = 1 mm
                                              // 5 = 10 mm
        R_VarInc        :R_TRIG;  //Fronte di salita del tasto VAR_INC
        O_PnCoupler_Profisafe    : BOOL;
        UsagePnCouplerInProfisafe   :BOOL;
        O_ModaleHoldToRun        : BOOL;
        Clk_PartenzaPostMortem   :BOOL;
        R_PartenzaPostMortem	 :R_TRIG;	//Fronte salita abilitazione post mortem per servizio
         Clk_AbilitazioneHoldToRun	:BOOL;
        F_AbilitazioneHoldToRun   :F_TRIG;	//Fronte discesa hold to run

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
        TimerSistema                            :ARRAY[0..MAX_TIMER_SISTEMA-1] OF Timer;
        O_ShutdownInCourse                      :BOOL;
        kvc_plus						        :BOOL;
        kvc_minus						        :BOOL;
        i_sys_error                             :DINT;	// codice errore
        kvc_start_cycle			                :BOOL;
        I_RiarmoAzionamenti                     :BOOL;
        AreaRadarError							:BOOL;
        SpintaB2Eseguita                        :BOOL;
        i_sys_OngoingReset                      :BOOL;
        i_sys_DpCoupler                         :BOOL;
        i_sys_PnCoupler                         :BOOL;
        kvc_reset						        :BOOL;
        F_T0 						            :F_TRIG; //fronte discesa T0 Pulsantiera
        F_T1						            :F_TRIG; //fronte discesa T1 Pulsantiera
        F_T2						            :F_TRIG; //fronte discesa T2 Pulsantiera
        i_sys_OPMODE                            : INT;	// MODO OPERATIVO
                                                        // 0 = JOG
                                                        // 1 = REFERENCE
                                                        // 2 = MDA
                                                        // 3 = AUTO
        FlgSetFirstStartVelReduced                 :BOOL;	//Serve per decidere se al primo start occorre effettuare i movimenti
                                                        //a velocità ridotta.
                                                        //TRUE quando è stato eseguita l^attivazione del modo operativo automatico
                                                        //FALSE quando si disattiva il modo automatico, oppure si conclude il primo start
        Rc                                 :INT;//Riga verso cui ci si sta muovendo
        i_sys_current_prog	                 :INT; // Programma corrente
        Programmi                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS - UINT#1] OF Programma;
        i_sys_Carm7CaricoContinuo       :BOOL;
        Clk_carm7caricocontinuo		    :BOOL;
        F_carm7caricocontinuo			:F_TRIG;	//Fronte discesa del ciclo continuo del CARM7 o degli AL5
    	RigaStartLoaderParallelo        :INT;
	    RigaEndLoaderParallelo          :INT;
        RichiestaArresto                :BOOL;
        R_ActiveDevice                          :R_TRIG; //Fronte di salita richiesta DEVICE da MCP
        R_ActiveAsse		                    :R_TRIG;
        R_ActiveDisp                            :R_TRIG;
        i_sys_activedevice                      : INT;  //Dispositivo selezionato per movimenti in JOG
                                                        // da 0..31 asse
                                                        // da 32 a 64 dispositivo IO
                                                        // -1 = INDEFINITO
        i_sys_devicecol_from_vgp        : INT; // richiesta attivazione DEVICE da VGP
                                                        // E^ il numero di dispositivo/asse secondo l^ordine della colonna di TABELLA STATI
        singleblock			            :Flipflop;
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
        mcpi_singleblock                : BOOL;
        i_sys_singleblockactive         : BOOL;
        mcpo_singleblock                :BOOL;
        senzatubo				        :Flipflop;
        mcpi_repos                      : BOOL;
        i_sys_SenzaTubo                 : BOOL;
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        i_sys_chan_request              : INT;  // STATO DEL SISTEMA
                                            // 0 = Non richiesto cambio modo
                                            // 1 = Richiesto nuovo modo operativo
        i_SYS_OPMODE_request            : INT;  // MODO OPERATIVO RICHIESTO
                                            // 0 = JOG
                                            // 1 = REFERENCE
                                            // 2 = MDA
                                            // 3 = AUTO
        i_sys_MDA_ABILITATO             : INT; //Abilita la modalità MDA che altrimenti non è attiva
                                            // Deve essere forzato dalla parte alta
        i_sys_CHANSTATUS                : INT;	// STATO DEL SISTEMA
                                            // 0 = Canale in RESET - IN JOG è sempre 0
                                            // 1 = Canale attivo - IN REF, MDA e AUTO attivi
                                            // 2 = Canale interrotto
        i_sys_stopcond                  : INT; // 0=NO STOP, 1=STOP ATTIVO
        access                          :Access_Level;
        mcpo_repos                      :BOOL;
        i_sys_CicloContinuoSenzaCoil	:BOOL;
        i_sys_CicloContinuo             : LREAL;
        CammeInterpolate                :BOOL; // TRUE se le camme sono già state interpolate
        R_SalvaGuardia	:R_TRIG;
        // {
        // #ifdef CURVATURA
        // }
          CoppiaY1TuboRotto             :USINT;  //Percentuale coppia per verifica rottura tubo
        // {
        // #endif  //Fine CURVATURA
        // }
        i_sys_OVR_act                   : LREAL; // OVERRIDE SOFTWARE TEMPORANEO - NON MODIFICARE -  VALORE 0 - 100
        i_sys_current_msg               :DINT; 			// Messaggio attivo
        i_sys_devicecol_from_simotion   : INT; // richiesta attivazione DEVICE da SIMOTION (MCP)
                                                // E^ il numero di dispositivo/asse secondo l^ordine della colonna di TABELLA STATI
        mcpo_ncstop                     : BOOL;
        mcpo_ncstart                    :BOOL;
        TaskCicliAttivo                 :BOOL;
        StartDato                       :BOOL;
        i_sys_BarrieraViolata           :BOOL;	//TRUE quando la barriera è KO e la macchina non può funzionare,
                                      //FALSE quando tutti gli assi sono stati riabilitati
        mcpo_plus  : BOOL;
        mcpo_minus  : BOOL;
        Curv_Man                       :CURVATURA_MANUALE_STRUCT; //Dati per il ciclo di curvatura manuale
        i_sys_swOVR                    : LREAL;  // OVERRIDE SOFTWARE - VALORE 0 - 100
        O_Reset,
        O_StatoEmergenza,
        O_StatoBarriera                        : BOOL;
        IAddress                        : ARRAY [0..MAXDIGITALINPUT] OF Digital_Input;
        // {
        // #ifdef PROFINET
        // }
          O_Comfort3            :BOOL;  //Serve per evitare di visualizzare l^errore 100076
        // {
        // #endif  //Fine PROFINET
        // }

        // {
        // #ifdef THREE_RUNNER
        // }
          F_Feedback_Carico_Tubo 	:F_TRIG;
          R_Feedback_Carico_Tubo 	:R_TRIG;
          F_Pedale_Carico_Tubo   	:F_TRIG;
          R_Pedale_Carico_Tubo   	:R_TRIG;
          BackgroundCycles       	:INT;
          ShouldBeInOperation		:BOOL;
          ShouldNotBeInOperation	:BOOL;
          IsInOperation				:BOOL;
        // {
        // #endif  //Fine THREE_RUNNER
        // }
        i_sys_CMD_Reset             :BOOL;  //True quando viene dato il comando di reset da comando di BlmMotion
        OPS_StartDato               :BOOL;  //True se l^operatore ha dato lo START per iniziare il pezzo seguente OPS
        Appo_i_sys_Error		    :DINT;	//Memorizzazione di particolari codici di errore che debbono essere
                                        //comunque visualizzati in BlmMotion in quanto hanno generato il
                                        //problema e sarebbero poi stati sovrascritti da altri codici di errore
        i_sys_InUsoScritturaSinamics:BOOL;
        i_sys_InUsoLetturaSinamics  :BOOL;
        ErroreLubrMacchina          :USINT;  //0=OK, 1=Errore livello, 2=Errore ciclo lubrificazione
        Calibr                      :Calibraz_Struct;
        i_sys_EnableBusy                :BOOL; //Se a TRUE indica che è in corso il programma
                                                //ENABLE_AX e quindi non può essere eseguito
                                                //il reset
        PM_Modified                  :BOOL;    //Flag per update dei parametri macchina
        i_sys_RunOk                  : INT;  // 0 = NON IN RUN
                                                // 1 = INIZIALIZZAZIONE E RUN ATTIVATI
                                                // 2 = INIZIALIZZAZIONE COMPLETATA
        i_sys_ResetPremutoSafety                :BOOL;
        i_sys_ResetPremutoDriver                :BOOL;
        i_sys_hwOVRdaTastiera                   :INT; // OVERRIDE HARDWARE impostato da tastiera
        i_sys_hwOVRdaRemoto                     :INT; // OVERRIDE HARDWARE impostato da remoto
        V_dbcoupler_i                           : dbcoupler_i;
        V_pncoupler_i                           : pncoupler_i;
        i_sys_MacchinaMaster				    :BOOL;
        EnableOverrideRemoto                :BOOL;  //True se è possibile usare l^override impostato da remoto. Diventa TRUE
                                                            //la prima volta dopo il run della CPU che ha valore maggiore di 1. Serve
                                                            //per capire che la comunicazione è stata attivata ed il valore (anche 0)
                                                            //è consistente
        
                // {
        // #ifdef DH4010  
        // }                                                    i_sys_hwOVR                             :LREAL; // OVERRIDE HARDWARE - VALORE 0 - 100
        // Rc                                      :ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;//Riga verso cui ci si sta muovendo
                // {
        // #endif  
        // }
        i_sys_hwovr_appo					    :LREAL;
        InibisciOverrideTastiera		        :BOOL;
        MinimumKeyboardOverride			        :INT;
        OverrideParametrico					    :INT;
        i_sys_UomoMorto						    :BOOL;
        //   {
        //   #ifdef SAFETY_INTEGRATED
        //   }
        i_sys_UomoMortoDelay                    :BOOL;	//Stato dell^uomo morto impostato dal PLC safety
        i_sys_override_uomomorto                :USINT;	//Override per pressione uomo morto
        i_sys_hwOVRLimit                        : REAL;
        i_sys_hwOVRLimit_for_scaling            : REAL;
        i_sys_Asse_Master_per_riduzT            :INT;   			//Numero dell^asse che comanda la riduzione di velocità in caso di bassa temperatura (-1 disabilita il ciclo)
        Temperatura                             :LREAL;
        i_sys_T_max_per_riduzT                  :LREAL;				//Temperatura massima per velocità ridotta
        i_sys_tOVR                              : LREAL;  // OVERRIDE PER RIDUZIONE VELOCITA^ DA TEMPERATURA - VALORE 0 - 100
        i_sys_T_min_per_riduzT                  :LREAL;				//Temperatura minima per velocità ridotta
        i_sys_vel_min_per_riduzT                :LREAL;				//% di velocità a bassa temperatura (in alta temperatura = 100)
        ovr,ovr2                                :LREAL;
        i_sys_override_HoldToRun_Auto	        :USINT;
        FlgRiduzVelVGP					        :BOOL;	//True se nel pezzo attivo è presente lo stop point 76 che indica che la traiettoria
                                                        //è stata creata da VGP3D per non oltrepassare i limiti delle velocità sicure
                                                        //imposti dal COMFORT3
                                                        // {
                                                        // #endif	//Fine SNERVAMENTO
                                                        // }
        FlgPezzoInEsecuzione			        :BOOL;  //True alla fine del primo start, False alla fine del pezzo
        I_Barriera                              :BOOL;
        i_sys_acc_master                        : REAL;
        i_sys_OVR                               :LREAL; // OVERRIDE   - VALORE 0 - 100
        i_sys_OverrideMarcatura				    :BOOL;	//True con marcatura in corso per forzare override 100% al primo start
        i_sys_OVR2                              : LREAL ; // * OVERRIDE MANDRINO
        i_sys_hwOVR2                             : LREAL ; // OVERRIDE MANDRINO HARDWARE - VALORE 50 - 100
        i_sys_swOVR2                            : LREAL ; // OVERRIDE MANDRINO SOFTWARE - VALORE 0 - 100
        Disp                                  :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
        i_sys_hwOVR                             :LREAL;
        MODE_LEVEL                              :ARRAY[-1..5] OF INT;
    END_VAR

    VAR_EXTERNAL CONSTANT
        TRC_JOG_NON_IN_MONITORING               :INT;  //Macchina non in monitoring o livello di accesso troppo basso
        INP_ENABLE_HOLD_TO_RUN                  :INT;
        MAX_TIMER_SISTEMA 	                    :USINT;
        T_LOG_AXIS	                            :INT;   //Log della trace degli assi
        INP_HOLD_TO_RUN                         :INT;
        INP_JOG_PLUS                            :INT;
        INP_JOG_MINUS                           :INT;
        INP_PHOTOCELLS_NOT_VIOLATED             :INT;
        ERR_ENABLE_HOLD_TO_RUN                  :DINT;
        BUTTON_PLUS 			                :USINT;
        BUTTON_MINUS			                :USINT;
        INP_AUTO_DPCOUPLER                      :INT;
        INP_JOG_DPCOUPLER                       :INT;
        INP_RESET_DPCOUPLER                     :INT;
        INP_AUTO                                :INT;
        RESET							        :USINT;
        STOPP_START_LOADING_UNLOADING_PARALLEL 	:INT;
        INP_CONTINUOUS_CYCLE_DPCOUPLER          :INT;
        A1 :    INT;
        A2 :    INT;
        B1 :    INT;
        B2 :    INT;
        X1 :    INT;    //Cart X
        X2 :    INT;
        X3 :    INT;
        W1 :    INT;
        W3 :    INT;
        W4 :    INT;
        L1 :    INT;
                (*{
                #ifdef ASSE_Z2
                }*)
        Z2  :INT;//Welding research on AL5
                (*{
                #endif  //Fine ASSE_Z2
                }*)
        JOGMODE                               :INT;
        REFMODE                               :INT;
        MDAMODE                               :INT;
        AUTOMODE                              :INT;
        TEACH_IN                              :INT;
        ERR_CMD_NON_DISPONIBILE              :DINT;
        X6                                   :INT;
        MODO_MDA_1        	                :USINT;  //1=Gestione standard
        MODO_MDA_2        	                :USINT;  //2=Gestione come tabella di posizioni
        MODO_MDA_3        	                :USINT;  //3=Gestione come tabella di posizioni e attesa start
        CHANNELRESET                        :INT;   //PROGRAMMI TUTTI RESETTATI
        ERR_OPMODE_NOT_ALLOWED              :DINT; 	//Richiesta modo operativo non ammessa per livello di accesso troppo basso
        CHANNELACTIVE :INT;								//PROGRAMMA ATTIVO
        CHANNELINTERRUPTED:INT;						//PROGRAMMA FERMO IN STOP CICLO
        VAR_INC_INIT :INT;  //Stato iniziale per gestione VAR_INC
        UNDEF_DEVICE                       	:INT;
        CMD_SERVICING                       :INT;  //115
        CMD_MANUAL_CURVE                    :INT;  //116
        OUT_TEST_PIECE                      :INT;
        CMD_END_SERVICE                     :INT;  //117
        ERR_CONTINUOUS_CYCLE_NOT_POSSIBLE   :DINT;
        ERR_EXECUTE_CMD_233                 :DINT;
        C_RESTORE_MACHINE				    :INT;
        C_SHIPPING_MACHINE					:INT;	//223;
        MSG_NO_MEMORY_OVERRIDE			    :INT;  //Memorizzazione tabella non possibile per override <100
        MSG_NOMESSAGE						:INT;   //Cancellazione messaggio
        FIRST_COMPLEX_OPERATION             :INT;
        CLAMP                               :INT;  //32
        FIRST_DEVICE                        :INT;
        PALETTA                             :INT;
        MANUAL_BENDING                      :INT;   //100
        C_CUT_KNIFE                         :INT;
        MAXDIGITALINPUT                     :INT;
        INP_CHECK_EMERGENCY                 :INT;
        INP_CHECK_BARRIER                   :INT;
        INP_CHECK_COMFORT3                  :INT;
        ERR_NOERROR							:SINT;
        INP_FEEDBACK_LOAD_TUBE              :INT;
        INP_PEDAL_LOAD_TUBE                 :INT;
        CMD_OUTSIDE_CUTTING                 :INT;  //123
        C_CALIBRATION                       :INT;  //112
        ACCESS_MANUTENTORE   	            :SINT;
        TRC_OVERRIDE_MIN_TEMP               :INT;
        AXES_MAX_TEMPERATURE                :DWORD;
          (*{
            #ifdef PLC_S7_1500
            }*)
        TEMPO_ARRESTO_BARRIERA  :REAL;	//Tempo di arresto assi durante violazione barriera di sicurezza
                                                //Se è 0=>Si ferma con il tempo dell^asse più lento
                       (*{
            #endif PLC_S7_1500
            }*)
        COL_RESERVED1             :INT;
        //         {
// #endif  //Fine ELECTM
// }
// {
// #ifdef ORBITALE
// }
    BIT_NO_OVERRIDE						:INT;  //Evita riduzione override per taglio sia 3-RUNNER, 4-RUNNER ed EFLEX
    ACCELERATION_MASTER                 :REAL;
    MAX_MASTER_ACCELERATION             :INT;
        ERR_AXES_NO_INDEX         		:DINT 	;	//Errore asse non indexato
        ERR_AXES_CREAZIONE_CAMMA  		:DINT 	;	//Errore creazione camma
        ERR_VETT_Y1_U4            		:DINT 	;	//Errore sull^oggetto vettoriale Y1_U4
        ERR_VECT_CARRO_ANIMA      		:DINT 	;	//Errore sull^oggetto vettoriale Carro-Anima
        ERR_VECT_MORSA_RULLI_Q    		:DINT 	;	//Errore sull^oggetto vettoriale Morsa-Rulli-Q
        ERR_VECT_TESTA_Q_T        		:DINT 	;	//Errore sull^oggetto vettoriale Testa-Q-V
        ERR_VECT_Y1_U4_Y2         		:DINT 	;	//Errore sull^oggetto vettoriale Braccio-SlittaMobile-Matrice
        ERR_VECT_Q_V1_V4          		:DINT 	;	//Errore sull^oggetto vettoriale Q-V1_V4
        ERR_VECT_A1_A2            		:DINT 	;	//Errore sull^oggetto vettoriale A1_A2
        ERR_VETT_Y1_X1            		:DINT 	;	//Errore sull^oggetto vettoriale Y1_X1
        ERR_VECT_Y1_Y2            		:DINT 	;	//Errore sull^oggetto vettoriale Braccio-Matrice
        ERR_VETT_X3_X6           		:DINT 	;	//Errore sull^oggetto vettoriale X3_X6
        ERR_VETT_X2_X6            		:DINT 	;	//Errore sull^oggetto vettoriale X2_X6
        ERR_VETT_X1_Y1            		:DINT 	;	//Errore sull^oggetto vettoriale X1_Y1
        ERR_VETT_X2_Y2            		:DINT 	;	//Errore sull^oggetto vettoriale X2_Y2
        SN_SP                           :SINT;
        SN                              :SINT;
        SP                              :SINT;
        TRC_SAFETY_JOG_DEVICES          :INT;

        C_DISATTIVAZIONE_BOOSTER_X1			:INT;
        C_IMPOSTAZIONI_ASSI_DISATTIVABILI	:INT;
        C_TENSIONE_FREQUENZA              	:INT;
        C_SOTTOTESTA                      	:INT;
        C_ATTIVAZIONE_V1                  	:INT;
        C_DISATTIVAZIONE_V1               	:INT;
        C_IMPOSTA_MANIPOLATORE2D			:INT;
        C_MACCHINA_DX                     	:INT;
          C_MACCHINA_SX                    	:INT;
          C_RESET_SIMOTION                  :INT;
          C_ON_4RFORM                       :INT;
          C_ASSE_ANIMA						:INT;	//Comandando attivazione/disattivazione asse anima
          ULTIMA_OPERAZIONE_COMPLESSA       :INT;
          CMD_TAGLIO_EFLEX                  :INT;  //118
          POSIZIONE_MASTER_RFI              :REAL;  //Posizione del master durante l^operazione di rientro fuori ingombri/primo start
          KinAxesW3_W4_Q1  					:UINT;
          KinAxesX1_U2                      :UINT;
          KinAxesY1_U4  					:UINT;
          KinAxesQ1_V1_Z1                   :UINT;
            LAVORO                  :SINT;
          RIPOSO                  :SINT;
          TRC_LIVELLO_DI_ACCESSO_BASSO            :INT; //Livello di accesso troppo basso per qualsiasi operazione in macchina

    END_VAR

    VAR
      KinAxesX1_U2_def    : DB_ANY;
      X1_U2               : REF_TO TO_Kinematics;
      KinAxesW3_W4_Q1_def     : DB_ANY;
      W3_W4_Q1             : REF_TO TO_Kinematics;
      KinAxesW3_W4_W6_Q1_def     : DB_ANY;
      W3_W4_W6_Q1              : REF_TO TO_Kinematics;
      KinAxesY1_U4_def      : DB_ANY;
      Y1_U4             : REF_TO TO_Kinematics;
      KinAxesY1_X1_def     : DB_ANY;
      Y1_X1             : REF_TO TO_Kinematics;
      KinAxesX1_Y1_def     : DB_ANY;
      X1_Y1             : REF_TO TO_Kinematics;
      KinAxesX2_Y2_def     : DB_ANY;
      X2_Y2             : REF_TO TO_Kinematics;
      KinAxesX3_X6_def     : DB_ANY;
      X3_X6             : REF_TO TO_Kinematics;
      KinAxesX2_X6_def     : DB_ANY;
      X2_X6             : REF_TO TO_Kinematics;
      KinAxesY1_U5_def     : DB_ANY;
      Y1_U5             : REF_TO TO_Kinematics;
      KinAxesY1_U4_Y2_def     : DB_ANY;
      Y1_U4_Y2             : REF_TO TO_Kinematics;
      KinAxesY1_Y2_def     : DB_ANY;
      Y1_Y2             : REF_TO TO_Kinematics;
      KinAxesQ1_V1_V4_def     : DB_ANY;
      Q1_V1_V4            : REF_TO TO_Kinematics;
      KinAxesQ1_V1_def     : DB_ANY;
      Q1_V1             : REF_TO TO_Kinematics;
      KinAxesQ1_V1_Z1_def     : DB_ANY;
      Q1_V1_Z1             : REF_TO TO_Kinematics;

      masteraxis          :REF_TO TO_PositioningAxis;
      PosAx               :REF_TO TO_SynchronousAxis;
      Sincroasse          :REF_TO TO_SynchronousAxis;
      Camma               :REF_TO TO_Cam;
      iret                :INT;
      dev                 :INT;
      OpMode              :INT;
      dwret               :DWORD;
      dret                :DINT;
      AssiFermi           :INT;
      AppoDirezione       :INT;
      Esito               :BOOL;
      AccelerazioneMaster :REAL;
      Appo                :REAL;
      NoErrore            :BOOL;
      StatePower          :BOOL;
      AsseMaxTemp         :INT;

          CURRENT_STATE										:INT:=GENERAL_STATE#STATE_INIT;
          CURRENT_AXIS										:INT;
          EnableAsseInCoppia                                : FB_EnableAsseInCoppia;
          DisableAsseInCoppia		                        :FB_DisableAsseInCoppia;
          Read_ActiveDevice								    :FB_Read_ActiveDevice;
          Change_OpMode										:FB_Change_OpMode;
          StopAssiMaster									:FB_StopAssiMaster;
          Homing_Absolute									:FB_Homing_Absolute; (*next to work *)
          EliminaCodaComandi							    :FB_EliminaCodaComandi;
          Set_Condizione_Move_Asse				            :FB_Set_Condizione_Move_Asse;
          Set_Condizione_Move_Dispositivo	                :FB_Set_Condizione_Move_Dispositivo;
          EnableAxisBLM										:FB_EnableAxisBLM;
          Jog_Asse											:FB_Jog_Asse;
          ResetMotionResetAxis						        :FB_ResetMotionResetAxis;
          MuoviDispositivo								    :FB_MuoviDispositivo;
          PosizionaSostegno								    :FB_PosizionaSostegno;
          SpegneAssi										:FB_SpegneAssi;
          CambiaSetParametri                                : FB_ChangeSetParameters;
    END_VAR

    VAR_TEMP
        i                   :INT;
    END_VAR

      VAR_OUTPUT
          OutputState	:INT;
      END_VAR

      CASE CURRENT_STATE OF
          GENERAL_STATE#STATE_INIT:
              (*{#ifndef MCP_1900}*)
                  mcpo_spindle_stop:=NOT(mcpi_spindle_stop);
              (*{#endif};*)
              //Legge il livello di accesso hardware dalla chiave della MCP
              i_sys_hw_AccessLevel:=Read_hw_AccessLevel();
              i_sys_sw_AccessLevel:=read_sw_accessLevel();

              // Imposta il livello di accesso scegliendo il minore fra hw e sw
              i_sys_AccessLevel:=MIN(i_sys_sw_AccessLevel , i_sys_hw_AccessLevel);

              IF i_sys_monitoring<>0 THEN
                  reset_trace(TRC_JOG_NON_IN_MONITORING);
              END_IF;

              (*IF i_sys_Accesslevel>ACCESS.jog_in_monitoring OR MCPI_FEED_STOP {#ifndef MCP_1900 }OR MCPI_SPINDLE_STOP {#endif} THEN
                  reset_trace(Code:=TRC_MONITORING_DISABILITATO);
              END_IF;*)

              IF i_sys_error_number>0 THEN
                  //Reset delle uscite che debbono essere spente, anche a seguito errore
                  OnStopCicloDispositivi();
              END_IF;

              //Legge il livello dei pulsanti INC_VAR
              IF mcpi_10000 THEN
                  var_inc(5);
              ELSE
                  IF mcpi_1000 THEN
                      var_inc(4);
                  ELSE
                      IF mcpi_100 THEN
                          var_inc(3);
                      ELSE
                          IF mcpi_10 THEN
                              var_inc(2);
                          ELSE
                              IF mcpi_1 THEN
                                  var_inc(1);
                              ELSE
                                  IF R_VARINC.Q THEN
                                      IF i_sys_varinc=6 THEN //reset VARINC
                                          var_inc(-1);
                                      ELSE
                                          var_inc(6);
                                      END_IF;
                                  END_IF;
                              END_IF;
                          END_IF;
                      END_IF;
                  END_IF;
              END_IF;
              //Legge i fronti di salita e discesa dei tasti, condizionati dal fatto che le barriere non siano violate
              (*{#ifdef PROFINET}*)
                  O_PnCoupler_Profisafe:=UsagePnCouplerInProfisafe;
                  //Da attivare se mai servirà
                  //O_MasterImpianto:=i_sys_MacchinaMaster;
                  (*{#ifndef ORBITALE}*)
                      //O_ModaleHoldToRun:=readdigitalinput(NumInput:=INP_ENABLE_HOLD_TO_RUN, Verifica:=FALSE);%%%%
                  (*{#endif}  //Fine O ORBITALE
              {#endif}  //Fine PROFINET *)
              Clk_PartenzaPostMortem:=TimerSistema[T_LOG_AXIS].Executed;
              R_PartenzaPostMortem(clk:=Clk_PartenzaPostMortem);
              Clk_AbilitazioneHoldToRun:=ReadDigitalInput(NumInput:=INP_ENABLE_HOLD_TO_RUN);
              F_AbilitazioneHoldToRun(clk:=Clk_AbilitazioneHoldToRun);
              Clk_HoldToRun:=ReadDigitalInput(NumInput:=INP_HOLD_TO_RUN);
              R_HoldToRun(clk:=Clk_HoldToRun);
              F_HoldToRun(clk:=Clk_HoldToRun);
              IF readdigitalinput(NumInput:=INP_ENABLE_HOLD_TO_RUN)=TRUE THEN
                  //Selettore abilitazione Hold to Run è arrivo
                  Clk_Plus:=ReadDigitalInput(NumInput:=INP_JOG_PLUS, Verifica:=FALSE) OR kvc_plus;
                  R_Plus(clk:=Clk_Plus);
                  F_Plus(clk:=Clk_Plus);
                  Clk_Minus:=ReadDigitalInput(NumInput:=INP_JOG_MINUS, Verifica:=FALSE) OR kvc_minus;
                  R_Minus(clk:=Clk_Minus);
                  F_Minus(clk:=Clk_Minus);
                  (*{#ifdef PROFINET}*)
                      IF R_HoldToRun.q=TRUE THEN
                          //Fronte di salita Hold to run si mette a TRUE O_ShutdownInCourse per abilitare SS1 nel PLC di sicurezza
                          O_ShutdownInCourse:=TRUE;
                      END_IF;
                      IF f_HoldToRun.q=TRUE THEN
                          //Fronte di discesa Hold to run si ripristina O_ShutdownInCourse
                          O_ShutdownInCourse:=FALSE;
                      END_IF;
                      (*{#ifdef EMULAZIONE}*)
                          //Per far partire le operazioni complesse senza lo START dal plc di sicurezza
                          R_Start(clk:=(R_Plus.q AND ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED, Verifica:=TRUE)));
                          F_Start(clk:=(F_Plus.q AND ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED, Verifica:=TRUE)));
                      (*{#endif}  //Fine EMULAZIONE
                  {#endif}  //Fine PROFINET *)
              ELSE
                  IF F_HoldToRun.Q=TRUE THEN
                      //Alla disattivazione dell^ingresso si cancella il messaggio rosso di avvertimento
                      init_error();
                  END_IF;
                  IF F_AbilitazioneHoldToRun.q=TRUE THEN
                      IF i_sys_error=ERR_ENABLE_HOLD_TO_RUN THEN
                          //Cancellazione messaggio rosso di abilitazione Hold To Run
                          init_error();
                      END_IF;
                  END_IF;

                  R_Start(clk:=((mcpi_NCstart OR KVC_start_cycle) AND ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED, Verifica:=TRUE)));
                  F_Start(clk:=((mcpi_NCstart OR KVC_start_cycle) AND ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED, Verifica:=TRUE)));
                  Clk_Plus:=(mcpi_plus OR kvc_plus) AND ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED, Verifica:=TRUE);
                  R_Plus(clk:=Clk_Plus);
                  F_Plus(clk:=Clk_Plus);
                  Clk_Minus:=(mcpi_minus OR kvc_minus) AND ReadDigitalInput(NumInput:=INP_PHOTOCELLS_NOT_VIOLATED, Verifica:=TRUE);
                  R_Minus(clk:=Clk_Minus);
                  F_Minus(clk:=Clk_Minus);
                  {::ifdef PROFINET} 
                      Clk_RiarmoAzionamenti:=I_RiarmoAzionamenti;
                      R_RiarmoAzionamenti(clk:=(Clk_RiarmoAzionamenti AND AreaRadarError=FALSE));
                      F_RiarmoAzionamenti(clk:=(Clk_RiarmoAzionamenti AND AreaRadarError=FALSE));
                      IF R_RiarmoAzionamenti.q=TRUE THEN
                          //Fronte di salita riarmo azionamenti si mette a TRUE O_ShutdownInCourse per abilitare SS1 nel PLC di sicurezza
                          O_ShutdownInCourse:=TRUE;
                          //Al riarmo azionamenti si riabilita la spinta del basculante elettrico sugli estremi
                          SpintaB2Eseguita:=FALSE;
                          //Con il riarmo degli azionamenti si resetta il flag che consente l^accensione della luce blue di
                          //riarmo a seguito di un movimento in JOG di un asse
                          i_sys_OngoingReset:=TRUE;
                      END_IF;
                      IF F_RiarmoAzionamenti.q=TRUE THEN
                          //Fronte di discesa riarmo azionamenti si ripristina O_ShutdownInCourse
                          O_ShutdownInCourse:=FALSE;
                      END_IF;
                  {::endif}  //Fine PROFINET *)
              END_IF;
              IF R_Plus.q=TRUE THEN
                  ConteggiaEvento(Event:=BUTTON_PLUS);
              END_IF;
              IF R_Minus.q=TRUE THEN
                  ConteggiaEvento(Event:=BUTTON_MINUS);
              END_IF;
              R_Rapid(clk:=mcpi_rapid);
              F_Rapid(clk:=mcpi_rapid);
              IF (i_sys_DpCoupler=TRUE OR i_sys_PnCoupler=TRUE) AND GetCmdCentraleCoupler()=TRUE THEN
                  //Comandi remoti
                  R_JOG(clk:=(readdigitalinput(NumInput:=INP_JOG_DPCOUPLER)=TRUE));
                  F_JOG(clk:=(readdigitalinput(NumInput:=INP_JOG_DPCOUPLER)=TRUE));
                  R_AUTO(clk:=(readdigitalinput(NumInput:=INP_AUTO_DPCOUPLER)=TRUE));
                  F_AUTO(clk:=(readdigitalinput(NumInput:=INP_AUTO_DPCOUPLER)=TRUE));
                  R_Reset(clk:=kvc_reset OR (readdigitalinput(NumInput:=INP_RESET_DPCOUPLER)=TRUE));
                  F_Reset(clk:=kvc_reset OR (readdigitalinput(NumInput:=INP_RESET_DPCOUPLER)=TRUE));
                  clk_Reset:=kvc_reset OR readdigitalinput(NumInput:=INP_RESET_DPCOUPLER);
              ELSE
                  //Comandi da pulsantiera
                  R_Reset(clk:=mcpi_reset OR kvc_reset);
                  F_Reset(clk:=mcpi_reset OR kvc_reset);
                  clk_Reset:=mcpi_reset OR kvc_reset;
                  IF readdigitalinput(NumInput:=INP_AUTO)=TRUE THEN
                      //Comando AUTOMODE da ingresso digitale, quando è attivo l^ingresso
                      //non si può abilitare il JOGMODE
                      R_AUTO(clk:=(readdigitalinput(NumInput:=INP_AUTO)=TRUE));
                      F_AUTO(clk:=(readdigitalinput(NumInput:=INP_AUTO)=TRUE));
                  ELSE
                      R_AUTO(clk:=mcpi_AUTO);
                      F_AUTO(clk:=mcpi_AUTO);
                      R_JOG(clk:=mcpi_JOG);
                      F_JOG(clk:=mcpi_JOG);
                  END_IF;
              END_IF;
              IF R_AUTO.q=TRUE OR i_sys_OPMODE<>AUTOMODE THEN
                  FlgSetFirstStartVelReduced:=TRUE;
              END_IF;
              IF R_Reset.q=TRUE THEN
                  ConteggiaEvento(Event:=RESET);
              END_IF;
              R_REF(clk:=mcpi_REF);
              F_REF(clk:=mcpi_REF);
              R_MDA(clk:=mcpi_MDA);
              F_MDA(clk:=mcpi_MDA);
              F_T0(clk:=mcpi_T0);
              F_T1(clk:=mcpi_T1);
              F_T2(clk:=mcpi_T2);
              {::ifndef ORBITALE}
                  ArrestoNastroPieno();
              {::endif}	//Fine NO ORBITALE
            //   {#ifdef CARICO_SCARICO_PARALLELI}
                //   IF GetCmdCentraleCoupler()=TRUE THEN %%%%
                //       IF rc<=RigaStopPoint(Programma:=i_sys_current_prog, PuntoArresto:=STOPP_START_LOADING_UNLOADING_PARALLEL) OR
                //            programmi[i_sys_current_prog].stato_corrente=0 THEN
                //           //Per evitare errori di sincronizzazione camma, il ciclo continuo può essere attivato solo se non è stato oltrepassato
                //           //il punto di inizio del carico parallelo, oppure è iniziato il primo start
                //           i_sys_carm7caricocontinuo:=readdigitalinput(NumInput:=INP_CONTINUOUS_CYCLE_DPCOUPLER);
                //       END_IF;
                //   END_IF;
                //   //Fronte di discesa ciclo continuo carico con CARM7 o AL
                //   F_carm7caricocontinuo(clk:=i_sys_carm7caricocontinuo);
                //   //L^interruzione del ciclo continuo durante il carico e lo scarico paralleli non è possibile.
                //   //Quindi si memorizza la richiesta di disattivazione e lo si gestisce al termine delle contemporaneità.
                //   IF F_carm7caricocontinuo.q=TRUE THEN
                //       //Disattivazione ciclo continuo CARM7 o AL5
                //       IF ((_getStateOfTaskId(_task.motiontask_LoaderParallelo)<>TASK_STATE_STOPPED) OR
                //               (RigaStartLoaderParallelo>-1 AND Rc>=RigaStartLoaderParallelo)) AND RichiestaArresto=FALSE THEN
                //           //Ripristino ciclo continuo e memorizzazione della richiesta di disattivazione
                //           i_sys_carm7caricocontinuo:=TRUE;
                //           RichiestaArresto:=TRUE;
                //       END_IF;
                //   END_IF;
                //   IF RichiestaArresto=TRUE AND _getStateOfTaskId(_task.motiontask_LoaderParallelo)=TASK_STATE_STOPPED AND
                //        Rc>RigaEndLoaderParallelo AND Rc<RigaStartLoaderParallelo THEN
                //       //Con la richiesta di disattivazione del ciclo continuo e con le operazioni di carico parallelo e scarico
                //       //parallelo terminatati è possibile interrompere la produzione
                //       i_sys_carm7caricocontinuo:=FALSE;
                //       RichiestaArresto:=FALSE;
                //   END_IF;
            //   {#else}	//Fine CARICO_SCARICO_PARALLELI
                //   IF GetCmdCentraleCoupler()=TRUE THEN
                //       i_sys_carm7caricocontinuo:=readdigitalinput(NumInput:=INP_CONTINUOUS_CYCLE_DPCOUPLER);
                //   END_IF;
            //   {#endif}	//Fine CARICO_SCARICO_PARALLELI
              GestioneTabelleProduzione();

              R_VarInc(clk:=mcpi_VARINC);

              R_ActiveAsse(clk:=(mcpi_ax0 OR mcpi_ax1 OR mcpi_ax2 OR mcpi_ax3 OR mcpi_ax4 OR
                                                   mcpi_ax5 OR mcpi_ax6 OR mcpi_ax7 OR mcpi_ax8 OR mcpi_ax9 OR
                                                   mcpi_ax10 OR mcpi_ax11));

            //   {#ifdef DH4010}
                  //Tolto mcpi_t13 perchè non è un dispositivo ma il tasto che abilita i comandi
                  //dei dispositivi della testa sinistra
            //       R_ActiveDisp(clk:=(mcpi_t0 OR mcpi_t1 OR mcpi_t2 OR mcpi_t3 OR mcpi_t4 OR %%%%
            //                                            mcpi_t5 OR mcpi_t6 OR mcpi_t7 OR mcpi_t8 OR mcpi_t9 OR
            //                                            mcpi_t10 OR mcpi_t11 OR mcpi_t12 OR mcpi_t14));
            // //   {#else}  //Fine DH4010
            //       R_ActiveDisp(clk:=(mcpi_t0 OR mcpi_t1 OR mcpi_t2 OR mcpi_t3 OR mcpi_t4 OR %%%%
            //                                            mcpi_t5 OR mcpi_t6 OR mcpi_t7 OR mcpi_t8 OR mcpi_t9 OR
            //                                            mcpi_t10 OR mcpi_t11 OR mcpi_t12 OR mcpi_t13 OR mcpi_t14));
            //   {#endif}  //Fine NO DH4010

              R_ActiveDevice(clk:=(R_ActiveAsse.Q OR R_ActiveDisp.Q));

            //   {#ifdef ASSE_Z2}
            //       {#ifndef MS0017}
                      //Non è disponibile il tasto per selezionare l^asse Z2 si utilizza un comando manuale di BlmMotion
                      IF i_sys_devicecol_from_vgp=Z2 THEN
                          i_sys_activedevice:=Z2;
                      END_IF;
            //       {#endif}  //Fine NO MS0017
            //   {#endif}  //Fine ASSE_Z2

              // Legge lo stato hardware di SINGLE BLOCK solo in modo MDA e AUTO
              IF i_sys_OPMODE=MDAMODE OR i_sys_OPMODE=AUTOMODE THEN
                  singleblock(s1:=mcpi_singleblock, r1:=BOOL#0, q1=> i_sys_singleblockactive);
              END_IF;
            //   {#ifdef DH4010  }
                  IF i_sys_singleblockactive=TRUE THEN
                      dret:=Push_error(code:=ERR_CMD_NON_DISPONIBILE, Arresto:=TRUE, Attesa:=FALSE);
                  END_IF;
                  i_sys_singleblockactive:=FALSE;
            //   {#endif}    //Fine DH4010
              mcpo_singleblock:=i_sys_singleblockactive;
              //Legge lo stato hardware di SENZATUBO
            //   {#ifdef ORBITALE}
                  IF i_sys_OpMode=JOGMODE THEN
                      senzatubo(s1:=mcpi_repos, r1:=BOOL#0, q1=>i_sys_SenzaTubo);
                    //   {#ifdef DH4010}
                          Axes[X6].Agganciato_Tabella_Stati:=NOT(i_sys_SenzaTubo);
                    //   {#endif}    //Fine DH4010
                  END_IF;
            //   {#else}  //Fine ORBITALE
                  senzatubo(s1:=mcpi_repos, r1:=BOOL#0, q1=>i_sys_SenzaTubo);
            //   {#endif}  //Fine NO ORBITALE
            //   {#ifdef ORBITALE}
                  IF i_sys_opmode<>AUTOMODE THEN
                      CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETERS_LOOP;
                  ELSE
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SET_MCPO_REPOS;
                  END_IF;
            //   {#else}
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SET_MCPO_REPOS;
            //   {#endif};

          _FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETERS_LOOP:
            //   {#ifdef ORBITALE}
                  CURRENT_AXIS:=CURRENT_AXIS+1;
                  IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
                      IF pm[CURRENT_AXIS].ma_num_encs>1 THEN
                          IF i_sys_SenzaTubo=TRUE AND TestEncoderEsterno(Asse:=CURRENT_AXIS)=TRUE THEN
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETRI_SET_1;
                          ELSE
                              IF i_sys_SenzaTubo=FALSE THEN
                                //   {#ifdef EFLEX}
                                      IF i_sys_activedevice=CMD_MANUAL_CURVE OR i_sys_activedevice=CMD_SERVICING  OR i_sys_activedevice=CMD_END_SERVICE THEN
                                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETRI_SET_1;
                                      ELSE
                                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETRI_SET_2;
                                      END_IF;
                                //   {#else}
                                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETRI_SET_2;
                                //   {#endif};
                              END_IF;
                          END_IF;
                      END_IF;
                  ELSE
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SET_MCPO_REPOS;
                  END_IF;
            //   {#endif};

          _FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETRI_SET_1:
            //   {#ifdef ORBITALE}
                  IF CambiaSetParametri.outputState<>GENERAL_STATE#STATE_END THEN
                      CambiaSetParametri(Axis:=Axes[CURRENT_AXIS].PosAx,
                                                           NumeroAsse:=CURRENT_AXIS,
                                                           NumeroSetParametri:=1,
                                                           ProssimoComando:=EnumNextCommandEnable#IMMEDIATELY);
                  ELSE
                      CambiaSetParametri();
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETERS_LOOP;
                  END_IF;
            //   {#endif};

          _FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETRI_SET_2:
            //   {#ifdef ORBITALE}
                  IF CambiaSetParametri.outputState<>GENERAL_STATE#STATE_END THEN
                      CambiaSetParametri(Axis:=Axes[CURRENT_AXIS].PosAx,
                                                           NumeroAsse:=CURRENT_AXIS,
                                                           NumeroSetParametri:=2,
                                                           ProssimoComando:=EnumNextCommandEnable#IMMEDIATELY);
                  ELSE
                      CambiaSetParametri();
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_SET_PARAMETERS_LOOP;
                  END_IF;
            //   {#endif};

          _FB_MCP483#STATE_MCP483_SET_MCPO_REPOS:
              mcpo_repos:=i_sys_SenzaTubo;
              WriteDigitalOutput(NumOutput:=OUT_TEST_PIECE, Value:=i_sys_SenzaTubo);
              {::ifdef DH4010}
                  IF TestCaricoAspo(Programma:=i_sys_current_prog)=FALSE AND i_sys_CicloContinuoSenzaCoil=FALSE THEN
                      IF i_sys_CicloContinuo>0.001 AND i_sys_opmode=AUTOMODE AND CammeInterpolate=TRUE THEN
                          dret:=Push_error(code:=ERR_CONTINUOUS_CYCLE_NOT_POSSIBLE, Attesa:=FALSE);
                      END_IF;
                      i_sys_CicloContinuo:=0;
                      SetLedCicloContinuo(StaTO:=FALSE);
                  END_IF;
                  IF (TestCicloSpedizioneEseguito()=TRUE AND i_sys_opmode<>JOGMODE) OR
                       (TestCicloSpedizioneEseguito()=TRUE AND i_sys_activedevice>UNDEF_DEVICE AND
                          i_sys_activedevice<>C_RESTORE_MACHINE AND i_sys_activedevice<>C_SHIPPING_MACHINE) THEN
                      //Durante o dopo il ciclo di spedizione può essere eseguito solo il ciclo di spedizione o quello di ripristino
                      dret:=Push_error(code:=ERR_EXECUTE_CMD_233, Attesa:=FALSE);
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_DH4010_CHANGE_OPMODE_JOGMODE;
                  ELSE
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SAVE;
                  END_IF;
              {::else}
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SAVE;
              {::endif};

          _FB_MCP483#STATE_MCP483_DH4010_CHANGE_OPMODE_JOGMODE:
            //   {#ifdef DH4010}
                  IF Change_OpMode.outputState<>GENERAL_STATE#STATE_END THEN
                      change_OpMode(JOGMODE);
                  ELSE
                      change_OpMode();
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SAVE;
                  END_IF;
            //   {#endif};

          _FB_MCP483#STATE_MCP483_SAVE:
            //   {#ifndef MCP_1900}
                //   R_SalvaGuardia(clk:=mcpi_teachin);//check in new in/out
            //   {#endif};
              IF R_SalvaGuardia.q THEN
                //   {#ifdef CURVATURA}
                //       {#ifdef EFLEX_DH4010}
                          ;
                    //   {#else} //Fine EFLEX_DH4010
                          IF CoppiaY1TuboRotto>0 THEN
                              //Ciclo di verifica rottura tubo abilitato
                              IF i_sys_ovr_act=100 THEN
                                  IF i_sys_SenzaTubo=FALSE THEN
                                      FOR i:=1 TO TO_INT(_MAXSEQ#MAXSEQ) DO
                                          Programmi[i_sys_current_prog].CoppiaMediaY1FrenaturaAppo[i]:=Programmi[i_sys_current_prog].CoppiaMediaY1Frenatura[i];
                                      END_FOR;
                                  END_IF;
                              ELSE
                                  //Non è possibile programmare la tabella campione in quanto l^override è <100
                                  i_sys_current_msg:=MSG_NO_MEMORY_OVERRIDE;
                              END_IF;
                          ELSE
                              dret:=Push_error(Code:=ERR_CMD_NON_DISPONIBILE, Arresto:=FALSE);
                          END_IF;
                //       {#endif}  //Fine EFLEX_DH4010
                //   {#else}  //Fine CURVATURA
                      dret:=Push_error(Code:=ERR_CMD_NON_DISPONIBILE, Arresto:=FALSE);
                //   {#endif}  //Fine NO CURVATURA
                  ;
              END_IF;
              IF i_sys_ChanStatus=CHANNELRESET THEN
                  // Legge il dispositivo attivo SOLO
                  // nella condizione di canale resettato
                  IF i_sys_monitoring=0 THEN
                      // non sono in monitoring
                      IF R_ActiveDevice.Q THEN
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_READ_ACTIVE_DEVICE_1;
                      ELSE
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SET_ACTIVE_DEVICE;
                      END_IF;
                  ELSE
                      // siamo in monitoring
                      // se esiste una richiesta di NUOVO device dalla MCP
                      IF R_ActiveDevice.Q THEN
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_READ_ACTIVE_DEVICE_2;
                      ELSE
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SET_ACTIVE_DEVICE;
                      END_IF;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SET_ACTIVE_DEVICE;
              END_IF;

          _FB_MCP483#STATE_MCP483_READ_ACTIVE_DEVICE_1:
              IF Read_ActiveDevice.OutputState<>GENERAL_STATE#STATE_END THEN
                  read_ActiveDevice();
              ELSE
                  dev:=read_ActiveDevice.ReturnValue;
                  read_ActiveDevice();
                  IF dev>=0 THEN
                      //Cancella eventuale messaggio verde
                      i_sys_current_msg:=MSG_NOMESSAGE;
                      set_Activedevice(iDeviceNumber:=dev);
                      i_sys_devicecol_from_vgp:=UNDEF_DEVICE;
                  END_IF;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SET_ACTIVE_DEVICE;
              END_IF;

        _FB_MCP483#STATE_MCP483_READ_ACTIVE_DEVICE_2:
              IF Read_ActiveDevice.OutputState<>GENERAL_STATE#STATE_END THEN
                  read_ActiveDevice();
              ELSE
                  dev:=read_ActiveDevice.ReturnValue;
                  // cancella eventuale messaggio verde
                  i_sys_current_msg:=MSG_NOMESSAGE;
                  IF dev<FIRST_COMPLEX_OPERATION  THEN
                      i_sys_Activedevice:=UNDEF_DEVICE; // Indefinito
                      i_sys_devicecol_from_vgp:=UNDEF_DEVICE;
                      IF Get_Colonna_Device_stati(dev)>-1 THEN
                          i_sys_devicecol_from_simotion:=Get_Colonna_Device_stati(dev);
                      ELSE
                          set_Activedevice(iDeviceNumber:=dev);
                      END_IF;
                  ELSE
                      set_Activedevice(iDeviceNumber:=dev);
                      i_sys_devicecol_from_vgp:=UNDEF_DEVICE;
                  END_IF;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_SET_ACTIVE_DEVICE;
              END_IF;

        _FB_MCP483#STATE_MCP483_SET_ACTIVE_DEVICE:
              IF i_sys_accesslevel>access.jog_in_monitoring THEN
                  set_Activedevice(iDeviceNumber:=UNDEF_DEVICE);
              END_IF;
              // Se non stiamo già processando una richiesta di cambio modo
              IF i_sys_Chan_Request=0 THEN
                  opmode:=Read_OpMode();
                  IF opmode=JOGMODE AND i_sys_OPMODE=JOGMODE THEN
                      // ripremuto tasto JOG
                      // azzero VAR_INC
                      var_inc(VAR_INC_INIT);
                  END_IF;
                  IF opmode<>-1 THEN
                      //Se la richiesta di nuovo modo operativo può essere accettata
                      IF i_sys_OPMODE<>opmode THEN
                          // Se è richiesto un passaggio in MANUALE oppure
                          // se il canale è in RESET
                          IF opmode=JOGMODE OR i_sys_chanstatus<>CHANNELACTIVE THEN
                              IF i_Sys_Accesslevel<=MODE_LEVEL[opmode] THEN
                                  // attivata la richiesta di cambio modo
                                  i_sys_chan_request:=1;
                                  i_SYS_OPMODE_request:=opmode;
                                  //ferma gli assi in movimento
                                  i_sys_stopcond:=1;
                              ELSE
                                  //ALLARME per non aver accettato la richiesta
                                  // per livello di accesso inadeguato
                                  iret:=Push_error(Code:=ERR_OPMODE_NOT_ALLOWED);
                              END_IF;
                          END_IF;
                      END_IF;
                  END_IF;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_NCSTOP;
              ELSE
                  //Gli assi sono tutti fermi?
                  assifermi:=0;
                  FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
                      IF TestVelRealZero(NomeAsse:=Axes[i].PosAx, NumeroAsse:=i, ErroreInseguimento:=FALSE) THEN
                          assifermi:=assifermi+1;
                      END_IF;
                  END_FOR;
                  IF assifermi=_MAXIS#MAXAXIS THEN
                      // Arresta e resetta i movimenti attivi
                      FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
                          IF pm[i].ma_asse_abilitato=TRUE THEN
                              StopAxis(Asse:=i);
                          END_IF;
                      END_FOR;
                      // resetta il canale e cambia il modo operativo
                      i_sys_CHANSTATUS:=CHANNELRESET;
                      IF i_SYS_OPMODE_request=JOGMODE THEN
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_OPMODE_JOG;
                      ELSE
                          //Imposta i limiti software degli assi ai valori swLEVEL1
                        //   {#ifdef EFLEX_DH4010}
                        //       {#ifdef EFLEX}
                                  SetAxSwLimit1();
                            //   {#else} //FINE EFLEX
                                  //Per la DH4010 non si debbono toccare i limiti di X1, X2 e X3 in quanto impostati
                                  //dal backgroud in accordo con i limiti calcolati e trasferiti da VGP
                                  SetAxSwLimit1(PrimoAsseNoGestione:=X1, SecondoAsseNoGestione:=X2, TerzoAsseNoGestione:=X3);
                        //       {#endif};  //Fine NO EFLEX
                        //   {#else}  //Fine EFLEX_DH4010
                        //       {#ifdef CURVATURA}
                        //           {#ifdef SPALLAMENTO_DX}
                                      SetAxSwLimit1(PrimoAsseNoGestione:=W3, SecondoAsseNoGestione:=W4);
                                //   {#else}  //Fine SPALLAMENTO_DX
                                      SetAxSwLimit1(PrimoAsseNoGestione:=W3);
                            //       {#endif};  //Fine NO SPALLAMENTO_DX
                            //   {#else};  //Fine CURVATURA
                                  SetAxSwLimit1();
                        //       {#endif};  //Fine NO CURVATURA
                        //   {#endif};  //Fine NO EFLEX_DH4010
                          IF i_SYS_OPMODE_request=REFMODE AND i_sys_OPMODE=jogmode THEN
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_OPMODE_REF;
                          ELSE
                              IF i_sys_OPMODE_request=AUTOMODE THEN
                                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_OPMODE_AUTO;
                              ELSE
                                  IF i_sys_OPMODE_request=MDAMODE AND i_sys_OPMODE=jogmode AND
                                       i_sys_MDA_ABILITATO>=MODO_MDA_1 AND i_sys_MDA_ABILITATO<=MODO_MDA_3 THEN
                                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHANGE_OPMODE_MDA;
                                  ELSE
                                      i_sys_chan_request:=0;
                                      i_SYS_OPMODE_request:=-1;
                                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_NCSTOP;
                                  END_IF;
                              END_IF;
                          END_IF;
                      END_IF;
                  ELSE
                      i_sys_chan_request:=0;
                      i_SYS_OPMODE_request:=-1;
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_NCSTOP;
                  END_IF;
              END_IF;

            _FB_MCP483#STATE_MCP483_CHANGE_OPMODE_REF:
              IF Change_OpMode.outputState<>GENERAL_STATE#STATE_END THEN
                  change_OpMode(REFMODE);
              ELSE
                  change_OpMode();
                  i_sys_chan_request:=0;
                  i_SYS_OPMODE_request:=-1;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_NCSTOP;
              END_IF;

            _FB_MCP483#STATE_MCP483_CHANGE_OPMODE_MDA:
              IF Change_OpMode.outputState<>GENERAL_STATE#STATE_END THEN
                  change_OpMode(MDAMODE);
              ELSE
                  change_OpMode();
                  i_sys_chan_request:=0;
                  i_SYS_OPMODE_request:=-1;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_NCSTOP;
              END_IF;

            _FB_MCP483#STATE_MCP483_CHANGE_OPMODE_AUTO:
              IF Change_OpMode.outputState<>GENERAL_STATE#STATE_END THEN
                  change_OpMode(AUTOMODE);
              ELSE
                  change_OpMode();
                  i_sys_chan_request:=0;
                  i_SYS_OPMODE_request:=-1;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_NCSTOP;
              END_IF;

            _FB_MCP483#STATE_MCP483_CHANGE_OPMODE_JOG:
              IF Change_OpMode.outputState<>GENERAL_STATE#STATE_END THEN
                  change_OpMode(JOGMODE);
              ELSE
                  change_OpMode();
                  i_sys_chan_request:=0;
                  i_SYS_OPMODE_request:=-1;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_NCSTOP;
              END_IF;

            _FB_MCP483#STATE_MCP483_NCSTOP:
              //LEGGE LO STOP
              IF Read_NCSTOP()=TRUE THEN
                  //Feed a zero
                  i_sys_stopcond:=1;
                  i_sys_CHANSTATUS:=CHANNELINTERRUPTED;
                  OnStopCicloDispositivi();
                  IF i_sys_OPMODE=AUTOMODE THEN
                    //   IF NOT _getStateOfTaskId(_task.motiontask_AUTO)=TASK_STATE_SUSPENDED THEN
                    //       dwret:=_suspendTaskId(_task.motiontask_AUTO);
                    //   END_IF;
                      mcpo_NCSTOP:=TRUE;
                      mcpo_NCSTART:=FALSE;
                  END_IF;
                  IF i_sys_OPMODE=MDAMODE THEN
                    //   IF NOT _getStateOfTaskId(_task.motiontask_MDA)=TASK_STATE_SUSPENDED THEN
                    //       dwret:=_suspendTaskId(_task.motiontask_MDA);
                    //   END_IF;
                      mcpo_NCSTOP:=TRUE;
                      mcpo_NCSTART:=FALSE;
                  END_IF;
                  IF i_sys_OPMODE=JOGMODE AND taskcicliattivo THEN
                    //   IF NOT _getStateOfTaskId(_task.motiontask_CICLI)=TASK_STATE_SUSPENDED THEN
                    //       dwret:=_suspendTaskId(_task.motiontask_CICLI);
                    //   END_IF;
                      mcpo_NCSTOP:=TRUE;
                      mcpo_NCSTART:=FALSE;
                  END_IF;
                  // REFERENCE per assi con encoder incrementale
                  IF i_sys_OPMODE=REFMODE AND taskcicliattivo THEN
                    //   IF NOT _getStateOfTaskId(_task.motiontask_CICLI)=TASK_STATE_SUSPENDED THEN
                    //       dwret:=_suspendTaskId(_task.motiontask_CICLI);
                    //   END_IF;
                      mcpo_NCSTOP:=TRUE;
                      mcpo_NCSTART:=FALSE;
                  END_IF;
                  FOR i:=FIRST_DEVICE TO (_MAXDEVICES#MAXDEVICES+FIRST_DEVICE-1) DO
                    //   {#ifdef ORBITALE}
                          AbilitazioneDispositivi(Dispositivo:=i, Stato:=FALSE);
                    //   {#else}
                          IF i<>CLAMP AND i<>PALETTA THEN
                              //Disabilitazione movimento dispositivi tranne la pinza e la paletta che possono
                              //essere movimentate anche con lo stop attivo (barriera di sicurezza violata)
                              AbilitazioneDispositivi(Dispositivo:=i, Stato:=FALSE);
                          END_IF;
                    //   {#endif};
                  END_FOR;
              ELSE
                  //Legge lo START solo nel caso in cui non ci siano allarmi attivi
                  IF Read_NCStart() THEN
                      FOR i:=FIRST_DEVICE TO (_MAXDEVICES#MAXDEVICES+FIRST_DEVICE-1) DO
                          //Disabilitazione movimento dispositivi
                          AbilitazioneDispositivi(Dispositivo:=i, Stato:=TRUE);
                      END_FOR;
                      StartDato:=TRUE;
                  END_IF;
                  IF i_sys_Error_number=0 AND StartDato=TRUE AND i_sys_BarrieraViolata=FALSE THEN
                      //Se è la prima volta che viene premuto lo start dopo il cambio modo
                      //non effettua il RESUME del task
                      IF i_sys_OPMODE=AUTOMODE AND i_sys_CHANSTATUS=CHANNELINTERRUPTED THEN
                        ;//   dwret:=_resumeTaskId(_task.motiontask_AUTO);
                      END_IF;
                      IF i_sys_OPMODE=MDAMODE AND i_sys_CHANSTATUS=CHANNELINTERRUPTED THEN
                          ;//dwret:=_resumeTaskId(_task.motiontask_MDA);
                      END_IF;
                      IF (i_sys_OPMODE=JOGMODE OR i_sys_OPMODE=REFMODE) AND taskcicliattivo AND i_sys_CHANSTATUS=CHANNELINTERRUPTED THEN
                          ;//dwret:=_resumeTaskId(_task.motiontask_CICLI);
                      END_IF;
                      i_sys_stopcond:=0;
                      IF i_sys_OPMODE=MDAMODE OR i_sys_OPMODE=AUTOMODE OR
                           (i_sys_OPMODE=JOGMODE AND taskcicliattivo=TRUE) OR
                           (i_sys_OPMODE=REFMODE AND taskcicliattivo=TRUE) THEN
                          mcpo_NCSTOP:=FALSE;
                          mcpo_NCSTART:=TRUE;
                          i_sys_CHANSTATUS:=CHANNELACTIVE;
                          OffStopCicloDispositivi();
                      END_IF;
                      //Reset dello stop ciclo dopo la riattivazione delle uscite digitali
                      StartDato:=FALSE;
                  END_IF;
              END_IF;
            //   {#ifdef CURVATURA}
                  //Verifica se attivo un CICLO in cui necessaria la presenza costante
                  IF i_sys_activedevice=MANUAL_BENDING THEN
                      mcpo_minus:=TRUE;
                      mcpo_plus:=TRUE;
                      IF (Clk_Plus=TRUE OR Clk_minus=TRUE) AND i_sys_BarrieraViolata=FALSE THEN
                          IF (Curv_Man.Avanti=TRUE AND Clk_minus=TRUE) OR
                               (Curv_Man.Indietro=TRUE AND Clk_Plus=TRUE) THEN
                              //Cambio comando andata/ritorno si fa ripartire il task cicli per
                              //iniziare un nuovo ciclo
                              Curv_Man.Avanti:=FALSE;
                              Curv_Man.Indietro:=FALSE;
                            //   dwret:=_restartTaskId(_task.motiontask_CICLI);
                          ELSE
                              IF read_NCSTOP()=FALSE THEN
                                  //Esecuzione dell^operazione richiesta
                                  i_sys_stopcond:=0;
                              END_IF;
                          END_IF;
                      ELSE
                          //Tasto piu^ o memo rilasciato si ferma in stop ciclo per eseguire
                          //la presenza costante
                          i_sys_stopcond:=1;
                      END_IF;
                      IF TestVelHoldToRun()=FALSE THEN
                          IF mcpi_rapid=TRUE THEN
                              //Imposta l^override in funzione del tasto rapid
                              i_sys_swOVR:=Curv_Man.Override_high_vel;
                          ELSE
                              i_sys_swOVR:=Curv_Man.Override_low_vel;
                          END_IF;
                      END_IF;
                  ELSE
                      //Reset flag curvatura in caso di abbandono curvatura manuale
                      Curv_Man.Avanti:=FALSE;
                      Curv_Man.Indietro:=FALSE;
                  END_IF;
            //   {#else}  //Fine CURVATURA
                  //Reset flag curvatura in caso di abbandono curvatura manuale
                  Curv_Man.Avanti:=FALSE;
                  Curv_Man.Indietro:=FALSE;
            //   {#endif}  //Fine no CURVATURA
              CASE i_sys_activedevice OF
                  C_CUT_KNIFE :
                      IF TaskCicliAttivo=FALSE THEN
                        ;//   dwret:=_restartTaskId(_task.motiontask_CICLI);
                      END_IF;
                      IF Clk_plus=FALSE AND Clk_minus=FALSE THEN
                          //Tasto più o memo rilasciato si ferma in stop ciclo per eseguire
                          //la presenza costante
                          i_sys_stopcond:=1;
                      ELSE
                          i_sys_stopcond:=0;
                      END_IF;
              END_CASE;
            //   {#ifdef PROFINET}
            //       {#ifndef EMULAZIONE}
                      IF F_Reset.Q=TRUE OR (R_RiarmoAzionamenti.Q AND O_Reset=TRUE) THEN
                          //Cancellazione della richiesta di reset al plc di sicurezza mediante il fronte di
                          //discesa del pulsante di reset della tastiera, oppure mediante il fronte di discesa
                          //del riarmo azionamenti.
                          O_Reset:=FALSE;
                          O_ShutdownInCourse:=FALSE;
                      END_IF;
                      O_StatoBarriera:=ReadDigitalInput(NumInput:=INP_CHECK_BARRIER, Verifica:=TRUE);
                      O_StatoEmergenza:=ReadDigitalInput(NumInput:=INP_CHECK_EMERGENCY , Verifica:=TRUE);
                    //   {#ifndef RUNNER}
                    //       {#ifndef EFLEX}
                              IF IAddress[INP_ENABLE_HOLD_TO_RUN].port=-1 OR
                                   IAddress[INP_ENABLE_HOLD_TO_RUN].bit=-1 THEN
                                  //In caso di mancanza di hold TO run (indirizzo non definito) lo stato del feedback
                                  //deve essere inviato al plc di sicurezza a TRUE per soddisfare il test di congruenza
                                  //dell^uscita di sicurezza che comanda il contattore per la selezione della zona radar
                                  //numero 4 (quella attiva con l^hold TO run premuto)
                                  O_Comfort3:=TRUE;
                              ELSE
                                  O_Comfort3:=ReadDigitalInput(NumInput:=INP_CHECK_COMFORT3, Verifica:=TRUE);
                              END_IF;
            //               {#endif} //FINE NO EFLEX
            //           {#endif} //FINE NO RUNNER
            //       {#endif}  //Fine EMULAZIONE
            //   {#endif}  //Fine PROFINET
            //   {#ifdef THREE_RUNNER}
                  F_Pedale_Carico_Tubo(clk:=ReadDigitalInput(NumInput:=INP_PEDAL_LOAD_TUBE, Verifica:=FALSE));
                  R_Pedale_Carico_Tubo(clk:=ReadDigitalInput(NumInput:=INP_PEDAL_LOAD_TUBE, Verifica:=FALSE));
                  F_FeedBack_Carico_Tubo(clk:=ReadDigitalInput(NumInput:=INP_FEEDBACK_LOAD_TUBE, Verifica:=FALSE));
                  R_Feedback_Carico_Tubo(clk:=ReadDigitalInput(NumInput:=INP_FEEDBACK_LOAD_TUBE, Verifica:=FALSE));
            //   {#endif} //Fine THREE_RUNNER
              IF R_Reset.Q=TRUE OR i_sys_CMD_Reset=TRUE OR
                   ((i_sys_error<>0 OR (i_sys_opmode<>AUTOMODE AND i_sys_activedevice=UNDEF_DEVICE)) AND R_RiarmoAzionamenti.Q) THEN
                  //Reset mediante apposito pulsante della tastiera, oppure con comando, oppure con pulsante di riarmo se ci sono
                  //errori attivi, oppure se non siamo in automatico ma non ci deve essere nessuna dispositivo/comando attivo
                  ops_startdato:=FALSE;
                  Appo_i_sys_Error:=ERR_NOERROR;
                  i_sys_InUsoScritturaSinamics:=FALSE;
                  i_sys_InUsoLetturaSinamics:=FALSE;
                  FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
                      Axes[i].statocamming:=FALSE;
                  END_FOR;
                  IF i_sys_activedevice=CMD_OUTSIDE_CUTTING THEN
                      i_sys_activedevice:=UNDEF_DEVICE;
                  END_IF;
                  IF i_sys_OPMODE=AUTOMODE THEN
                      IF (i_sys_DpCoupler=TRUE OR i_sys_PnCoupler=TRUE) AND GetCmdCentraleCoupler()=TRUE THEN
                          iret:=reset_error();
                          //Riabilitazione assi
                           ;//dwret:=_restartTaskId(_task.motiontask_ENABLEAX);
                          IF i_sys_error=0 THEN
                              //Ripartenza della gestione del post mortem solo se il reset ha eliminato tutti gli errori attivi
                              TimerSistema[T_LOG_AXIS].Executed:=FALSE;
                              TimerSistema[T_LOG_AXIS].Value:=0;
                              TimerSistema[T_LOG_AXIS].Status:=SINT#0;
                          END_IF;
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_RESTART_ENABLEAX;
                      ELSE
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_RESET_CHANGE_OPMODE_JOG;
                      END_IF;
                  ELSE
                      i_sys_stopcond:=1;    //NON TOGLIERE SERVE PER EVITARE DI FAR RIPARTIRE
                                                                  //L^ASSE MOVIMENTATO DA SOLO SE VIENE PREMUTO SOLO IL RESET
                      ResetRc();
                      iret:=reset_error();
                      ErroreLubrMacchina:=USINT#0;
                      IF i_sys_activedevice=C_CALIBRATION THEN
                          calibr.numeroasse:=TO_SINT(UNDEF_DEVICE);
                          i_sys_activedevice:=UNDEF_DEVICE;
                      END_IF;
                    //   {#ifdef PROFINET}
                          O_Reset:=TRUE;
                          O_ShutdownInCourse:=TRUE;
                    //   {#endif}	//Fine PROFINET
                    //   {#ifdef EFLEX_DH4010}
                          //Arresto del motion task cicli quando di preme reset
                        //   IF _getStateOfTaskId(_task.motiontask_CICLI)<>TASK_STATE_STOPPED THEN
                        //       set_Activedevice(iDeviceNumber:=UNDEF_DEVICE);
                        //       dwret:=_resetTaskId(_task.motiontask_CICLI);
                        //   END_IF;
                    //   {#endif}    //Fine EFLEX_DH4010
                      IF i_sys_EnableBusy=FALSE THEN
                          //Arresta e resetta i movimenti attivi
                          //cancella eventuale messaggio verde
                          i_sys_current_msg:=MSG_NOMESSAGE;
                          Abort_All();
                          PM_Modified:=FALSE;
                          FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
                              IF Axes[i].Request_Restart_Oggetto_Simotion=TRUE THEN
                                  PM_Modified:=TRUE;
                              END_IF;
                          END_FOR;
                          FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
                              IF Axes[i].Request_Restart_Oggetto_Sinamics=TRUE THEN
                                  PM_Modified:=TRUE;
                              END_IF;
                          END_FOR;
                          //cancella il log degli assi
                          Init_LOG_Assi();
                          //Riabilita gli assi
                          i_sys_RUNOK:=0;
                        //   IF _getStateOfTaskId(_task.motiontask_ENABLEAX)<>TASK_STATE_RUNNING THEN
                        //       dwret:=_restartTaskId(_task.motiontask_ENABLEAX);
                        //       i_sys_stopcond:=0;
                        //   END_IF;
                      END_IF;
                      i_sys_ResetPremutoSafety:=TRUE;
                      i_sys_ResetPremutoDriver:=TRUE;
                      IF i_sys_error=0 THEN
                          //Ripartenza della gestione del post mortem solo se il reset ha eliminato tutti gli errori attivi
                          TimerSistema[T_LOG_AXIS].Executed:=FALSE;
                          TimerSistema[T_LOG_AXIS].Value:=0;
                          TimerSistema[T_LOG_AXIS].Status:=SINT#0;
                      END_IF;
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_RESTART_ENABLEAX;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_RESTART_ENABLEAX;
              END_IF;

            _FB_MCP483#STATE_MCP483_RESET_CHANGE_OPMODE_JOG:
              IF Change_OpMode.outputState<>GENERAL_STATE#STATE_END THEN
                  Change_OpMode(JOGMODE);
              ELSE
                  Change_OpMode();
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_RESTART_ENABLEAX;
              END_IF;

            _FB_MCP483#STATE_MCP483_RESTART_ENABLEAX:
              IF (R_Reset.Q=TRUE OR ((i_sys_error<>0 OR i_sys_opmode<>AUTOMODE) AND R_RiarmoAzionamenti.Q)) AND
                   i_sys_EnableBusy=TRUE (*AND _getStateOfTaskId(_task.motiontask_ENABLEAX)=TASK_STATE_RUNNING *)THEN
                  //Ripartenza del programa ENABLE_AX per eseguire l^eventuale operazione di scrittura dei parametri
                  //se siamo bloccati in fondo alla funzione
               ; //   dwret:=_restartTaskId(_task.motiontask_ENABLEAX);
              END_IF;
              //Legge l^override della tastiera
            //   {#ifndef MCP_1900} %%%%
                //   i_sys_hwOVRdaTastiera:=Read_Ovr(mcpi_feed_a,mcpi_feed_b,mcpi_feed_c,mcpi_feed_d,mcpi_feed_e);
            //   {#else}
                  //i_sys_hwOVRdaTastiera:=Read_Ovr_MCP1900();
                  //SetPowerrideIncrementLEDs();
                  //spostato nell'ipo
            //   {#endif};
            //   {#ifdef PROFINET}
                  //Lettura dell^override da remoto
                //   {#ifndef EMULAZIONE}
                      IF i_sys_PnCoupler=TRUE THEN
                          i_sys_hwOVRdaRemoto:=(MAX(0, MIN(100, TO_INT(V_pncoupler_i.address_pn_i[43]))));
                      ELSE
                          IF i_sys_DpCoupler=TRUE THEN
                              i_sys_hwOVRdaRemoto:=(MAX(0, MIN(100, TO_INT(V_dbcoupler_i.address_db_i[43]))));
                          ELSE
                              i_sys_hwOVRdaRemoto:=0;
                          END_IF;
                      END_IF;
                //   {#endif}	//Fine NO EMULAZIONE
                  IF (i_sys_PnCoupler=TRUE OR i_sys_DpCoupler=TRUE) AND
                       i_sys_MacchinaMaster=FALSE AND
                       GetCmdCentraleCoupler()=TRUE THEN
                      IF i_sys_hwOVRdaRemoto>0 THEN
                          //L^abilitazione ad usare l^override da remoto occorre che il valore sia stato impostato
                          //diverso da 0 almeno una volta dalla messa in run del Simotion, ci debbono essere un Coupler
                          //la macchina deve essere slave nell^impianto e deve essere connessa ad un sistema centrale
                          EnableOverrideRemoto:=TRUE;
                      END_IF;
                  ELSE
                      //Disabilitazione override da remoto senza nessun Coupler e se la macchina
                      EnableOverrideRemoto:=FALSE;
                  END_IF;
            //   {#else}  //Fine PROFINET
                  IF i_sys_DpCoupler=TRUE THEN
                      i_sys_hwOVRdaRemoto:=MAX(0, MIN(100, TO_INT(V_dbcoupler_i.address_db_i[43])));
                  ELSE
                      i_sys_hwOVRdaRemoto:=0;
                  END_IF;
                  IF i_sys_DpCoupler=TRUE AND
                       i_sys_MacchinaMaster=FALSE AND
                       GetCmdCentraleCoupler()=TRUE THEN
                      IF i_sys_hwOVRdaRemoto>0 THEN
                          //L^abilitazione ad usare l^override da remoto occorre che il valore sia stato impostato
                          //diverso da 0 almeno una volta dalla messa in run del Simotion, ci debbono essere un Coupler
                          //la macchina deve essere slave nell^impianto e deve essere connessa ad un sistema centrale
                          EnableOverrideRemoto:=TRUE;
                      ELSE
                          //Disabilitazione override da remoto senza nessun Coupler e se la macchina
                          EnableOverrideRemoto:=FALSE;
                      END_IF;
                  END_IF;
            //   {#endif}  //Fine NO PROFINET
              IF EnableOverrideRemoto=TRUE THEN
                  //Impostazione override considerando anche la connessione remota
                  i_sys_hwOVR:=MIN(i_sys_hwOVRdaTastiera, i_sys_hwOVRdaRemoto);
              ELSE
                  i_sys_hwOVR:=i_sys_hwOVRdaTastiera;
              END_IF;

              IF InibisciOverrideTastiera THEN
                  IF i_sys_opmode=AUTOMODE AND i_sys_ACCESSLEVEL>ACCESS_MANUTENTORE THEN
                      IF (i_sys_hwOVR>=MinimumKeyboardOverride) THEN
                          i_sys_hwOVR_appo:=OverrideParametrico;
                      ELSE
                          i_sys_hwOVR_appo:=0;
                      END_IF;
                  END_IF;
              END_IF;
            //   {#ifdef SAFETY_INTEGRATED}
                  //Impostazione override per gestione uomo morto
                  IF i_sys_UomoMorto=TRUE OR i_sys_UomoMortoDelay=TRUE THEN
                      i_sys_HWOvr_appo:=i_sys_HWOvr_appo*i_sys_override_uomomorto/100;
                  END_IF;
            //   {#else}  //Fine SAFETY_INTEGRATED
                  (*{#ifndef PROFINET}
                      {#ifdef RIDUZIONE_OVERRIDE}
                          IF readdigitalinput(NumInput:=INP_CH1_RID_OVERRIDE, Verifica:=FALSE)=TRUE THEN
                              IF readdigitalinput(NumInput:=INP_CH2_RID_OVERRIDE, Verifica:=FALSE)=TRUE THEN
                                  //Entrambi ingressi a ON riduzione override attiva
                                  i_sys_HWOvr_appo:=i_sys_HWOvr_appo*i_sys_override_uomomorto/100;
                              ELSE
                                  //Un ingresso a ON e uno a OFF override a 0
                                  i_sys_HWOvr_appo:=0;
                              END_IF;
                          ELSE
                              IF readdigitalinput(NumInput:=INP_CH2_RID_OVERRIDE, Verifica:=FALSE)=TRUE THEN
                                  //Un ingresso a ON e uno a OFF override a 0
                                  i_sys_HWOvr_appo:=0;
                              END_IF;
                          END_IF;
                      {#endif}  //Fine RIDUZIONE_OVERRIDE
                  {#endif}  //Fine NO PROFINET*)
            //   {#endif}  //Fine NO SAFETY_INTEGRATED
              i_sys_HWOvr_appo:=MIN(TO_REAL(i_sys_HWOvr),i_sys_HWOvrLimit);
              i_sys_HWOvr_appo:=MIN(TO_REAL(i_sys_HWOvr_appo), i_sys_HWOvrLimit_for_Scaling);

              //Calcolo riduzione di velocità per basse temperature
              IF i_sys_Asse_Master_per_riduzT>=0 AND
                   i_sys_Asse_Master_per_riduzT<_MAXIS#MAXAXIS AND
                   TestVelHoldToRun()=FALSE THEN
                  //Ciclo di riduzione velocità in caso di bassa temperatura
                  //legge la temperatura dell^asse che comanda
                  Temperatura:=Axes[i_sys_Asse_Master_per_riduzT].Temperatura_Motore;
                  IF Temperatura>=i_sys_T_max_per_riduzT THEN
                      reset_trace(Code:=TRC_OVERRIDE_MIN_TEMP);
                      i_sys_tOVR:=100;
                  ELSE
                      set_trace(Code:=TRC_OVERRIDE_MIN_TEMP, Axis:=i_sys_Asse_Master_per_riduzT);
                      IF Temperatura<=i_sys_T_min_per_riduzT THEN
                          i_sys_tOVR:=i_sys_vel_min_per_riduzT;
                      ELSE
                          //Valore di temperatura compresa fra min e max
                          i_sys_tOVR:=100-(100-i_sys_vel_min_per_riduzT)*(i_sys_T_max_per_riduzT-Temperatura)/(i_sys_T_max_per_riduzT-i_sys_T_min_per_riduzT);
                      END_IF;
                  END_IF;
              ELSE
                  reset_trace(Code:=TRC_OVERRIDE_MIN_TEMP);
                  i_sys_tOVR:=100;
              END_IF;
              //Calcolo abbassamento feed per temperatura motori troppo alta
              Temperatura:=0;
              //Trova max temperatura degli assi che concorrono ad abbassare l^override
              FOR i:=0 TO _MAXIS#MAXAXIS-1 DO
                  IF pm[i].ma_asse_abilitato=TRUE THEN
                      IF getBit(AXES_MAX_TEMPERATURE, TO_USINT(i))=TRUE THEN
                          IF Temperatura<TO_LREAL(Axes[i].Temperatura_Motore) THEN
                              Temperatura:=TO_LREAL(Axes[i].Temperatura_Motore);
                              AsseMaxTemp:=i;
                          END_IF;
                      END_IF;
                  END_IF;
              END_FOR;

              //Modifica per gestione override come limitazione e non come percentuale
              ovr:=(i_sys_hwOvr_appo*i_sys_swOvr)/100;
              IF ovr>i_sys_tOVR THEN
                  ovr:=i_sys_tOVR;
              END_IF;
            //   {#ifdef PROFINET}
            //       {#ifndef ORBITALE}
                      IF TestVelHoldToRun()=TRUE AND i_sys_OPMODE=AUTOMODE AND FlgRiduzVelVGP=FALSE AND (TestMasterLavoro()=TRUE OR FlgPezzoInEsecuzione=TRUE) THEN
                          //Impostazione override per gestione hold TO run in automatico immediatamente al termine del primo start, così
                          //che al momento di iniziare la movimentazione degli assi l^override attivo sia già impostato al valore definito
                          //nel parametro Q4100
                          IF ovr>i_sys_override_HoldToRun_Auto THEN
                              ovr:=i_sys_override_HoldToRun_Auto;
                          END_IF;
                      END_IF;
            //       {#endif}  //Fine NO ORBITALE
            //   {#endif}	//Fine PROFINET
              IF i_sys_stopcond<>0 THEN
                //   {#ifdef PROFINET}
                      IF (i_Barriera=FALSE OR i_sys_BarrieraViolata=TRUE) AND TEMPO_ARRESTO_BARRIERA>0.01 THEN
                          //In caso di violazione della barriera di sicurezza se definita
                          //si attiva l^apposita rampa di decelerazione
                          Appo:=TEMPO_ARRESTO_BARRIERA;
                          AccelerazioneMaster:=1/Appo;
                      ELSE
                          AccelerazioneMaster:=i_sys_acc_master;
                      END_IF;
                //   {#else}  //Fine PROFINET
                      IF i_sys_BarrieraViolata=TRUE AND TEMPO_ARRESTO_BARRIERA>0.01 THEN
                          //In caso di violazione della barriera di sicurezza se definita
                          //si attiva l^apposita rampa di decelerazione
                          Appo:=TEMPO_ARRESTO_BARRIERA;
                          AccelerazioneMaster:=1/Appo;
                      ELSE
                          AccelerazioneMaster:=i_sys_acc_master;
                      END_IF;
                //   {#endif}  //Fine NO PROFINET
                  i_sys_ovr:=0;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_STOP_ASSI_MASTER;
              ELSE
                //   {#ifdef ORBITALE}
                //       {#ifdef DH4010}
                          i_sys_ovr:=ovr;
                    //   {#else}  //Fine DH4010
                          IF i_sys_OPMODE=AUTOMODE THEN
                              IF TestMasterLavoro()=TRUE THEN
                                  IF getBit(TO_DWORD(ReadTabStati(Riga:=rc, Colonna:=COL_RESERVED1, Programma:=I_SYS_Current_Prog, Tabella:=SINT#-1)), TO_USINT(BIT_NO_OVERRIDE))=TRUE THEN
                                      //Gestione forzatura OVERRIDE nel ciclo automatico nelle linee della tabella
                                      //stati dove e^ previsto (ad esempio per il taglio)
                                      i_sys_ovr_act:=100;
                                      i_sys_ovr:=100;
                                      masteraxis:=AsPositioningAxisRef(Axes[X1].masteraxis);
                                      masteraxis^._override.velocity:=100;
                                  ELSE
                                      i_sys_ovr:=ovr;
                                  END_IF;
                              ELSE
                                  //Primo start
                                //   {#ifdef FOUR_RUNNER_PLAUNO}
                                      IF i_sys_OverrideMarcatura=TRUE THEN
                                          //Forzatura override al 100% per mantenere costante la lunghezza della scritta
                                          i_sys_ovr_act:=100;
                                          i_sys_ovr:=100;
                                          masteraxis:=AsPositioningAxisRef(Axes[X1].masteraxis);
                                          masteraxis^._override.velocity:=100;
                                          //Axes[X1].masteraxis._override.velocity:=100;
                                      ELSE
                                          //Ripristiono override al termine della marcatura
                                          i_sys_ovr:=ovr;
                                      END_IF;
                                //   {#else}	//Fine FOUR_RUNNER_PLAUNO
                                      i_sys_ovr:=ovr;
                                //   {#endif};    //Fine NO FOUR_RUNNER_PLAUNO
                              END_IF;
                          ELSE
                              i_sys_ovr:=ovr;
                          END_IF;
                //       {#endif}  //Fine NO DH4010
                //   {#else}  //Fine ORBITALE
                      i_sys_ovr:=ovr;
                //   {#endif}  //Fine NO ORBITALE
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_READ_OVR_2;
              END_IF;

            _FB_MCP483#STATE_MCP483_STOP_ASSI_MASTER:
              IF StopAssiMaster.outputState<>GENERAL_STATE#STATE_END THEN
                  StopAssiMaster(AccelerazioneMaster:=AccelerazioneMaster (* , ProssimoComando:=IMMEDIATELY*));
              ELSE
                  StopAssiMaster();
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_READ_OVR_2;
              END_IF;

            _FB_MCP483#STATE_MCP483_READ_OVR_2:
               {::ifndef MCP_1900}
                  //Legge il II Override
                  i_sys_hwOVR2:=read_ovr2(mcpi_spindleoverride_a,mcpi_spindleoverride_b,mcpi_spindleoverride_c,mcpi_spindleoverride_d);
                  ovr2:=((i_sys_hwOvr2)/100) * ((i_sys_swOvr2)/100) * 100;
                  IF i_sys_stopcond<>0 THEN
                      i_sys_ovr2:=0;
                  ELSE
                      i_sys_ovr2:=ovr2;
                  END_IF;
               {::else}	//FINE NO MCP_1900
                  IF i_sys_stopcond<>0 THEN
                      i_sys_ovr2:=0;
                  ELSE
                      i_sys_ovr2:=100;
                  END_IF;
            {::endif} //FINE MCP_1900
              ovr2:=((i_sys_hwOvr2)/100) * ((i_sys_swOvr2)/100) * 100;
              IF i_sys_stopcond<>0 THEN
                  i_sys_ovr2:=0;
              ELSE
                  i_sys_ovr2:=ovr2;
              END_IF;
              //Assegnazione override all^asse master, per fare in modo di gestire la velocità
              //degli assi collegati in camma
              IF i_sys_stopcond=0 THEN
                  //Non deve essere attivo lo stop ciclo, in quanto in quella fase
                  //l^arresto dell master avviene con un normale comando di stop, associato
                  //alla rampa di decelerazione definita nella variabile AccelerazioneMaster
                  IF ABS(GetMasterOverride()-i_sys_ovr)>0.9 AND TestMasterVeloc()=TRUE THEN
                      //Al cambio di override corrisponde una immediata impostazione dell^accelerazione
                      //dell^asse master pari a 3 (quindi siamo a 0.3 secondi di rampa)
                      //Questo per evitare che ad ogni gradino venga causata sugli assi veri una
                      //accelerazione/decelerazione troppo elevata
                      IF ABS(GetMasterMaxAcceleration()-ACCELERATION_MASTER)>0.1 THEN
                          SetMasterMaxAcceleration(Value:=ACCELERATION_MASTER);
                      END_IF;
                  ELSE
                      //Senza nessun cambio di override si ripristina l^accelerazione standard del master
                      //molto elavata
                      IF ABS(GetMasterMaxAcceleration()-MAX_MASTER_ACCELERATION)>0.1 THEN
                          SetMasterMaxAcceleration(Value:=MAX_MASTER_ACCELERATION);
                      END_IF;
                  END_IF;
                  //Impostazione dell^override dell^asse master
                  IF ABS(GetMasterOverride()-i_sys_ovr_Act)>0.001 THEN
                      SetMasterVelocity(Value:=i_sys_ovr_Act);
                    //   {#ifdef RUNNER}
                    //       {#ifdef FOUR_RUNNER_PLAUNO}
                    //           {#ifndef PLAUNO_LIGHT}
                                masteraxis:=AsPositioningAxisRef(Axes[A1].masteraxis);
                                masteraxis^._override.velocity:=i_sys_ovr_Act;
                        //       {#endif} //FINE NO PLAUNO_LIGHT
                        //   {#endif}  //Fine FOUR_RUNNER_PLAUNO
                        //   {#ifdef B1_B2}
                              masteraxis:=AsPositioningAxisRef(Axes[B1].masteraxis);
                              masteraxis^._override.velocity:=i_sys_ovr_Act;
                    //       {#endif}  //Fine B1_B2
                    //   {#endif}  //Fine RUNNER
                  END_IF;
              ELSE
                  //Lo stop ciclo è attivo
                  IF TestMasterVeloc()=FALSE THEN
                      //Con stop ciclo attivo si imposta l^override desiderato quando il master si è fermato
                      IF ABS(GetMasterOverride()-i_sys_ovr_Act)>0.001 THEN
                          SetMasterVelocity(Value:=i_sys_ovr_Act);
                        //   {#ifdef RUNNER}
                        //       {#ifdef FOUR_RUNNER_PLAUNO}
                        //           {#ifndef PLAUNO_LIGHT}
                                      masteraxis:=AsPositioningAxisRef(Axes[A1].masteraxis);
                                      masteraxis^._override.velocity:=i_sys_ovr_Act; 
                            //       {#endif} //FINE NO PLAUNO_LIGHT
                            //   {#endif}  //Fine FOUR_RUNNER_PLAUNO
                            //   {#ifdef B1_B2}
                                    masteraxis:=AsPositioningAxisRef(Axes[B1].masteraxis);
                                    masteraxis^._override.velocity:=i_sys_ovr_Act;
                        //       {#endif}  //Fine B1_B2
                        //   {#endif}  //Fine RUNNER
                      END_IF;
                  END_IF;
                  //Lo stop ciclo si esegue con l^accelerazione standard del master
                  IF ABS(GetMasterMaxAcceleration()-MAX_MASTER_ACCELERATION)>0.1 THEN
                      SetMasterMaxAcceleration(Value:=MAX_MASTER_ACCELERATION);
                  END_IF;
              END_IF;
              FOR i:=_FIRST_AXIS#FIRST_AXIS TO _FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1 BY 1 DO
                  IF CheckIfTOactive(Axis:=i)=TRUE THEN
                      //Se l^asse non è in CAMMA
                      PosAx:=AsSynchronousAxisRef(Axes[i].PosAx);
                      Camma:= AsCamRef(PosAx^.StatusSynchronizedMotion.ActualCam);
                      IF   PosAx^.StatusSynchronizedMotion.FunctionState <>3 THEN
                          {::ifdef AL_BLM}
                              IF i=L1 THEN
                                  //Essendo L1 un asse del caricatore, non deve essere gestito normalmente
                                  //in quanto deve consentire il funzionamento del caricatore, anche quando
                                  //l^override è impostato a 0 in quanto il dispositivo è fuori posizione
                                  IF i_sys_stopcond=1 THEN
                                    ;
                                      PosAx^._override.velocity:=i_sys_ovr_act;
                                  ELSE
                                    ;
                                      PosAx^._override.velocity:=i_sys_hwovr;
                                  END_IF;
                              ELSE
                            //     //   {#ifdef ASSE_Z2}
                                      IF i=Z2 THEN
                                        PosAx:=AsSynchronousAxisRef(Axes[i].PosAx);
                                          IF i_sys_stopcond=1 THEN ;
                                            PosAx^._override.velocity:=i_sys_ovr_act;
                                          ELSE ;
                                            PosAx^._override.velocity:=100;
                                          END_IF;
                                      ELSE ;
                                            PosAx^._override.velocity:=i_sys_ovr_act;
                                      END_IF;
                                //   {#else}  //Fine ASSE_Z2
                                            PosAx^._override.velocity:=i_sys_ovr_act;
                                //   {#endif}  //Fine NO ASSE_Z2
                              END_IF;
                          {::else}  //Fine AL_BLM
                        //       {#ifdef EFLEX}
                                  IF i=W1 THEN
                                        PosAx:=AsSynchronousAxisRef(Axes[i].PosAx);
                                      IF (i_sys_OpMode=JOGMODE AND i_sys_activedevice=CMD_TAGLIO_EFLEX) OR
                                           (i_sys_OpMode=AUTOMODE AND PosizioneMaster()=POSIZIONE_MASTER_RFI) THEN
                                          //Asse W1 (trancia) non deve sentire override quando taglia con
                                          //il comando manuale
                                          PosAx^._override.velocity:=100;
                                      ELSE
                                          PosAx^._override.velocity:=i_sys_ovr_act;
                                      END_IF;
                                  ELSE
                                      //Gestione standard asse
                                      PosAx^._override.velocity:=i_sys_ovr_act;
                                  END_IF;
                            //   {#else}  //Fine EFLEX
                            //       {#ifdef RUNNER}
                            //           {#ifdef FOUR_RUNNER_PLAUNO}
                            //               {#ifdef B1_B2}
                                            //   IF {#ifndef PLAUNO_LIGHT}i=A1 OR i=A2 OR {#endif} i=B1 OR i=B2 THEN
                                            //       IF i_sys_ovr_act>0.1 THEN
                                            //           assi[i].PosAx._override.velocity:=100;
                                            //       ELSE
                                            //           assi[i].PosAx._override.velocity:=i_sys_ovr_act;
                                            //       END_IF;
                                            //   ELSE
                                            //       //Gestione standard asse
                                            //       assi[i].PosAx._override.velocity:=i_sys_ovr_act;
                                            //   END_IF;
                                        //   {#else}  //Fine B1_B2
                                        //       {#ifndef PLAUNO_LIGHT}
                                                  IF i=A1 OR i=A2 THEN
                                                    PosAx:=AsSynchronousAxisRef(Axes[i].PosAx);
                                                      IF i_sys_ovr_act>0.1 THEN ;
                                                          //PosAx^._override.velocity:=100;
                                                      ELSE ;
                                                          //PosAx^._override.velocity:=i_sys_ovr_act;
                                                      END_IF;
                                                  ELSE ;
                                            //   {#endif} //FINE NO PLAUNO_LIGHT
                                              //Gestione standard asse
                                                   // PosAx^._override.velocity:=i_sys_ovr_act;
                                            //   {#ifndef PLAUNO_LIGHT}
                                                  END_IF;
                                    //           {#endif}	//Fine PLAUNO_LIGHT
                                    //       {#endif}  //Fine NO B1_B2
                                    //   {#endif}  //Fine FOUR_RUNNER_PLAUNO
                                    //   {#ifdef THREE_RUNNER_TWO_RUNNER}
                                    //       {#ifdef B1_B2}
                                              IF i=A1 OR i=B1 OR i=B2 THEN
                                                PosAx:=AsSynchronousAxisRef(Axes[i].PosAx);
                                                  IF i_sys_ovr_act>0.1 THEN ;
                                                      //PosAx^._override.velocity:=100;
                                                  ELSE ;
                                                      //PosAx^._override.velocity:=i_sys_ovr_act;
                                                  END_IF;
                                              ELSE ;
                                                  //Gestione standard asse
                                                  //PosAx^._override.velocity:=i_sys_ovr_act;
                                              END_IF;
                                        //   {#else}  //Fine B1_B2
                                              IF i=A1 THEN
                                                  IF i_sys_ovr_act>0.1 THEN ;
                                                      //PosAx^._override.velocity:=100;
                                                  ELSE ;
                                                      //PosAx^._override.velocity:=i_sys_ovr_act;
                                                  END_IF;
                                              ELSE ;
                                                  //Gestione standard asse
                                                  //PosAx^._override.velocity:=i_sys_ovr_act;
                                              END_IF;
                                //           {#endif}  //Fine NO B1_B2
                                //       {#endif}  //Fine THREE_RUNNER_TWO_RUNNER
                                //   {#else}  //Fine RUNNER
                                      //Gestione standard asse
                                      //PosAx^._override.velocity:=i_sys_ovr_act;
                        //           {#endif}  //Fine NO RUNNER
                        //       {#endif}  //Fine NO EFLEX
                          {::endif}  //Fine NO AL_BLM
                          ;
                      END_IF;
                  END_IF;
              END_FOR;
              //Assegna l^override agli oggetti interpolatore
              {::ifndef ORBITALE}
                KinAxesX1_U2_def:= KinAxesX1_U2;
                X1_U2:= AsKinematicsRef(KinAxesX1_U2_def);
                X1_U2^._override.velocity:=i_sys_ovr_act;
              {::endif}  //Fine ORBITALE
                //{::ifndef ETURN}
                    // {::ifndef ORBITALE}
                    //   {::ifndef ELECTMRL}
                    //         KinAxesW3_W4_Q1_def:= KinAxesW3_W4_Q1;
                    //         W3_W4_Q1:= AsKinematicsRef(KinAxesW3_W4_Q1_def);
                    //       W3_W4_Q1^._override.velocity:=i_sys_ovr_act;
                    // //   {#else}
                    //         KinAxesW3_W4_W6_Q1_def:= KinAxesW3_W4_W6_Q1;
                    //         W3_W4_W6_Q1:= AsKinematicsRef(KinAxesW3_W4_W6_Q1_def);
                    //         W3_W4_W6_Q1^._override.velocity:=i_sys_ovr_act;
            //           {#endif} //FINE ELECTMRL
            //       {#endif}	//Fine ORBITALE
            //   {#endif}  //Fine ETURN
            //   {#ifdef INTERPOLA_Y1_X1}
                  //Y1_X1._override.velocity:=i_sys_ovr_act;//
            //   {#endif}    //Fine INTERPOLA_Y1_X1
            //   {#ifdef INTERPOLA_X1_Y1}
                  //X1_Y1._override.velocity:=i_sys_ovr_act;//
            //   {#endif}	//Fine INTERPOLA_X1_Y1
            //   {#ifdef INTERPOLA_X2_Y2}
                  //X2_Y2._override.velocity:=i_sys_ovr_act;//
            //   {#endif}	//Fine INTERPOLA_X2_Y2
            //   {#ifdef INTERPOLA_X3_X6}
                  //X3_X6._override.velocity:=i_sys_ovr_act;//
            //   {#endif}	//Fine INTERPOLA_X3_X6
            //   {#ifdef INTERPOLA_X2_X6}
                  //X2_X6._override.velocity:=i_sys_ovr_act;//
            //   {#endif}	//Fine INTERPOLA_X2_X6
            {::ifdef SLITTA_MOBILE_DX}
             KinAxesY1_U4_def:= KinAxesY1_U4;
             Y1_U4:= AsKinematicsRef(KinAxesY1_U4_def);
             Y1_U4^._override.velocity:=i_sys_ovr_act;
                  //Y1_U4._override.velocity:=i_sys_ovr_act;//
            {::endif}	//Fine SLITTA_MOBILE_DX
            //   {#ifdef SLITTA_MOBILE_SX}
                  //Y1_U5._override.velocity:=i_sys_ovr_act;//
            //   {#endif}	//Fine SLITTA_MOBILE_SX
            //    {::ifdef MATRICE_ELETTRICA}
            //       IF TestAxesY2()=TRUE THEN
            //           Y1_U4_Y2._override.velocity:=i_sys_ovr_act;//
            //            Y1_Y2._override.velocity:=i_sys_ovr_act;//
            //        END_IF;
            //    {::endif}  //Fine MATRICE_ELETTRICA
            //   {#ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO}
                //   Q1_V1_V4._override.velocity:=i_sys_ovr_act;//
                //   Q1_V1._override.velocity:=i_sys_ovr_act;
            //   {#endif}	//Fine SOLLEVAMENTO_SLITTA_SPALLAMENTO
               {::ifdef INTERPOLA_Q1_V1_Z1}
            KinAxesQ1_V1_Z1_def:= KinAxesQ1_V1_Z1;
            Q1_V1_Z1:= AsKinematicsRef(KinAxesY1_U4_def);
            Q1_V1_Z1^._override.velocity:=i_sys_ovr_act;
                  //Q1_V1_Z1._override.velocity:=i_sys_ovr_act;
               {::endif}    //Fine INTERPOLA_Q1_V1_Z1
              IF i_sys_current_prog>=0 THEN
                  //Rinfresca il MASTER
                  SetPosizioneMaster(Programma:=i_sys_current_prog);
              END_IF;
              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
              CURRENT_STATE:= _FB_MCP483#STATE_MCP483_LOOP_HOMING;

        _FB_MCP483#STATE_MCP483_LOOP_HOMING:
              FOR i:=CURRENT_AXIS+1 TO _MAXIS#MAXAXIS-1 BY 1 DO
                  IF TestToAttivo(Asse:=i)=TRUE THEN
                    PosAx:=AsSynchronousAxisRef(Axes[i].PosAx);
                      IF  Posax^.StatusWord.%x1 = TRUE THEN
                          IF Posax^.ErrorDetail.Reaction<>0 THEN
                              //Ferma tutti gli altri assi
                              i_sys_stopcond:=1;
                              IF Posax^.StatusWord.%X0=TRUE THEN
                                  Abort_All();
                              END_IF;
                          END_IF;
                      END_IF;

                      //Verifica eventuali errori di creazione della camma
                      IF PosAx^.StatusSynchronizedMotion.ActualMaster <> UDINT#0 THEN
                     // IF Axes[i].sincroasse.activemaster<> NULL THEN
                        Camma:= AsCamRef(Axes[i].Camma);
                            IF Camma^.StatusWord.%X1 =TRUE THEN
                            //IF Axes[i].Camma.error=YES THEN
                               IF Camma^.ErrorDetail.Reaction <> 0 THEN
                              //IF assi[i].Camma.errorreaction<>NONE THEN
                                  i_sys_stopcond:=1;
                                    IF Posax^.StatusWord.%X0=TRUE THEN
                                  //IF assi[i].Posax.control=ACTIVE THEN
                                      Abort_All();
                                  END_IF;
                              END_IF;
                          END_IF;
                      END_IF;
                      IF posax^.VirtualAxis.Mode = UDINT#0 THEN
                          IF pm[i].ma_verifica_num_serie_encoder=TRUE THEN
                              Axes[i].Homed:=Axes[i].numserieencoder=pm[i].ma_num_serie_encoder AND
                                                           Axes[i].numserieencoder>0 AND
                                                           PosAx^.StatusWord.%X5= TRUE AND
                                                           pm[i].ma_stato_index=2;
                          ELSE
                              Axes[i].Homed:=PosAx^.StatusWord.%X5=TRUE AND 
                                                           pm[i].ma_stato_index=2;
                        ;  END_IF;
                      ELSE
                          IF PosAx^.StatusWord.%X5=TRUE THEN
                              //Asse virtuale l^asse viene indexato di default
                              pm[i].ma_stato_index:=SINT#0;
                              CURRENT_AXIS:=i;
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_LOOP_HOMING_ABSOLUTE;
                              EXIT;
                          END_IF;
                      END_IF;
                  END_IF;
              END_FOR;
              IF CURRENT_STATE<>_FB_MCP483#STATE_MCP483_LOOP_HOMING_ABSOLUTE THEN
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHECK_HOMING_ERRORS;
              END_IF;

            //   CURRENT_AXIS:=CURRENT_AXIS+1;
            //   IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
            //     PosAx:= AsSynchronousAxisRef(Axes[CURRENT_AXIS].Posax);
            //       IF TestToAttivo(Asse:=CURRENT_AXIS)=TRUE THEN
            //           IF Posax^.ErrorWord=TRUE THEN
            //             IF Posax^.ErrorDetail.Reaction<>0 THEN
            //               //IF assi[CURRENT_AXIS].Posax.errorreaction<>NONE THEN
            //                   //Ferma tutti gli altri assi
            //                   i_sys_stopcond:=1;
            //                   IF Posax^.StatusWord.%X0=TRUE THEN
            //                   //IF assi[CURRENT_AXIS].Posax.control=ACTIVE THEN
            //                       Abort_All();
            //                   END_IF;
            //               END_IF;
            //           END_IF;
            //           sincroasse:= AsSynchronousAxisRef( assi[CURRENT_AXIS].Sincroasse);
            //           IF Sincroasse^.ErrorWord=TRUE THEN
            //           //IF.error=YES THEN
            //             IF Sincroasse^.ErrorDetail.Reaction<>0 THEN
            //               //IF Sincroasse^.errorreaction<>NONE THEN
            //                   i_sys_stopcond:=1;
            //                   PosAx:= AsSynchronousAxisRef(Axes[i].Posax);
            //                   IF Posax^.StatusWord.%X0=TRUE THEN
            //                       Abort_All();
            //                   END_IF;
            //               END_IF;
            //           END_IF;
            //           //Verifica eventuali errori di creazione della camma
            //           sincroasse:= AsSynchronousAxisRef(Axes[CURRENT_AXIS].sincroasse);
            //           IF sincroasse^.StatusSynchronizedMotion.ActualMaster <> UDINT#0 THEN
            //           //IF Axes[CURRENT_AXIS].sincroasse.activemaster<>NULL THEN
            //           Camma:= AsCamRef(Axes[i].Camma);
            //           IF Camma^.ErrorWord=TRUE THEN
            //           //IF Axes[i].Camma.error=YES THEN
            //              IF Camma^.ErrorDetail.Reaction <> 0 THEN
            //               //IF assi[CURRENT_AXIS].Camma.error=YES THEN
            //                   //IF assi[CURRENT_AXIS].Camma.errorreaction<>NONE THEN
            //                       i_sys_stopcond:=1;
            //                       IF assi[CURRENT_AXIS].Posax.control=ACTIVE THEN
            //                           Abort_All();
            //                       END_IF;
            //                   END_IF;
            //               END_IF;
            //           END_IF;
            //           IF Axes[CURRENT_AXIS].posax.typeofaxis<>VIRTUAL_AXIS THEN
            //               IF pm[CURRENT_AXIS].ma_verifica_num_serie_encoder=TRUE THEN
            //                   Axes[CURRENT_AXIS].Homed:=Axes[CURRENT_AXIS].numserieencoder=pm[CURRENT_AXIS].ma_num_serie_encoder AND
            //                                                Axes[CURRENT_AXIS].numserieencoder>0 AND
            //                                                Axes[CURRENT_AXIS].PosAx.positioningstate.homed=YES AND
            //                                                pm[CURRENT_AXIS].ma_stato_index=2;
            //               ELSE
            //                   Axes[CURRENT_AXIS].Homed:=Axes[CURRENT_AXIS].PosAx.positioningstate.homed=YES AND
            //                                                pm[CURRENT_AXIS].ma_stato_index=2;
            //               END_IF;
            //           ELSE
            //               IF Axes[CURRENT_AXIS].PosAx.positioningstate.homed<>YES THEN
            //                   //Asse virtuale l^asse viene indexato di default
            //                   pm[CURRENT_AXIS].ma_stato_index:=0;
            //                   CURRENT_STATE:=STATE_MCP483_LOOP_HOMING_HOMING_ABSOLUTE;
            //               END_IF;
            //           END_IF;
            //       END_IF;
            //   ELSE
            //       CURRENT_STATE:=_FB_MCP483#STATE_MCP483_CHECK_HOMING_ERRORS;
            //   END_IF;

          _FB_MCP483#STATE_MCP483_LOOP_HOMING_ABSOLUTE:
              IF Homing_Absolute.outputState<>GENERAL_STATE#STATE_END THEN
                  Homing_absolute(Asse:=CURRENT_AXIS, Position:=0);
              ELSE
                  Homing_absolute();
                  Axes[CURRENT_AXIS].homed:=TRUE;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_LOOP_HOMING;
              END_IF;

          _FB_MCP483#STATE_MCP483_CHECK_HOMING_ERRORS:
            //   {#ifdef INTERPOLA_Y1_X1}
                //   IF Y1_X1.error=YES THEN %%%%
                //       IF Y1_X1.errorreaction<>NONE THEN
                //           //Ferma tutti gli altri assi
                //           i_sys_stopcond:=1;
                //           IF Y1_X1.linearPathCommand.state=ACTIVE THEN //OT.StatusWord.%X8
                //               iret:=Push_error(code:=ERR_VETT_Y1_X1, Arresto:=TRUE);
                //           ELSE
                //               iret:=Push_error(code:=ERR_VETT_Y1_X1, Arresto:=FALSE);
                //           END_IF;
                //       END_IF;
                //   END_IF;
            //   {#endif}	//Fine INTERPOLA_Y1_X1
            //   {#ifdef INTERPOLA_X1_Y1}
                //   IF X1_Y1.error=YES AND X1_Y1.errorreaction<>NONE AND I_Barriera=TRUE AND i_sys_BarrieraViolata=FALSE THEN %%%%
                //       //Ferma tutti gli altri assi
                //       i_sys_stopcond:=1;
                //       IF X1_Y1.linearPathCommand.state=ACTIVE THEN
                //           iret:=Push_error(code:=ERR_VETT_X1_Y1, Arresto:=TRUE);
                //       ELSE
                //           iret:=Push_error(code:=ERR_VETT_X1_Y1, Arresto:=FALSE);
                //       END_IF;
                //   END_IF;
            //   {#endif}	//Fine INTERPOLA_X1_Y1
            //   {#ifdef INTERPOLA_X2_Y2}
                //   IF X2_Y2.error=YES AND X2_Y2.errorreaction<>NONE AND I_Barriera=TRUE AND i_sys_BarrieraViolata=FALSE THEN %%%%


                //       //Ferma tutti gli altri assi
                //       i_sys_stopcond:=1;
                //       IF X2_Y2.linearPathCommand.state=ACTIVE THEN
                //           iret:=Push_error(code:=ERR_VETT_X2_Y2, Arresto:=TRUE);
                //       ELSE
                //           iret:=Push_error(code:=ERR_VETT_X2_Y2, Arresto:=FALSE);
                //       END_IF;
                //   END_IF;
            //   {#endif}	//Fine INTERPOLA_X1_Y1
            //   {#ifdef INTERPOLA_X3_X6}
                //   IF X3_X6.error=YES AND X3_X6.errorreaction<>NONE AND I_Barriera=TRUE AND i_sys_BarrieraViolata=FALSE THEN %%%%
                //       //Ferma tutti gli altri assi
                //       i_sys_stopcond:=1;
                //       IF X3_X6.linearPathCommand.state=ACTIVE THEN
                //           iret:=Push_error(code:=ERR_VETT_X3_X6, Arresto:=TRUE);
                //       ELSE
                //           iret:=Push_error(code:=ERR_VETT_X3_X6, Arresto:=FALSE);
                //       END_IF;
                //   END_IF;
            //   {#endif}	//Fine INTERPOLA_X3_X6
            //   {#ifdef INTERPOLA_X2_X6}
                //   IF X2_X6.error=YES AND X2_X6.errorreaction<>NONE AND I_Barriera=TRUE AND i_sys_BarrieraViolata=FALSE THEN %%%%
                //       //Ferma tutti gli altri assi
                //       i_sys_stopcond:=1;
                //       IF X2_X6.linearPathCommand.state=ACTIVE THEN
                //           iret:=Push_error(code:=ERR_VETT_X2_X6, Arresto:=TRUE);
                //       ELSE
                //           iret:=Push_error(code:=ERR_VETT_X2_X6, Arresto:=FALSE);
                //       END_IF;
                //   END_IF;
            //   {#endif}	//Fine INTERPOLA_X2_X6
            //   {#ifdef SLITTA_MOBILE_DX}

            //   {#endif}	//Fine SLITTA_MOBILE_DX
            //   {#ifdef SLITTA_MOBILE_SX}
                //   IF Y1_U5.error=YES THEN
                //       IF Y1_U5.errorreaction<>NONE THEN
                //           //Ferma tutti gli altri assi
                //           i_sys_stopcond:=1;
                //           IF Y1_U5.linearPathCommand.state=ACTIVE THEN
                //               iret:=Push_error(code:=ERR_VETT_Y1_U4, Arresto:=TRUE);
                //           ELSE
                //               iret:=Push_error(code:=ERR_VETT_Y1_U4, Arresto:=FALSE);
                //           END_IF;
                //       END_IF;
                //   END_IF;
            //   {#endif}  //Fine SLITTA_MOBILE_SX
            //   {#ifndef ORBITALE}
                //   IF X1_U2.error=YES THEN
                //       IF X1_U2.errorreaction<>NONE THEN
                //           //Ferma tutti gli altri assi
                //           i_sys_stopcond:=1;
                //           IF X1_U2.linearPathCommand.state=ACTIVE THEN
                //               iret:=Push_error(code:=ERR_VECT_CARRO_ANIMA, Arresto:=TRUE);
                //           ELSE
                //               iret:=Push_error(code:=ERR_VECT_CARRO_ANIMA, Arresto:=FALSE);
                //           END_IF;
                //       END_IF;
                //   END_IF;
            //   {#endif}  //Fine NO ORBITALE
            //   {#ifndef ETURN}
            //       {#ifndef ORBITALE}
            //           {#ifndef ELECTMRL}
                        //   IF W3_W4_Q1.error=YES THEN
                        //       IF W3_W4_Q1.errorreaction<>NONE THEN
                        //           //Ferma tutti gli altri assi
                        //           i_sys_stopcond:=1;
                        //           IF W3_W4_Q1.linearPathCommand.state=ACTIVE THEN
                        //               iret:=Push_error(code:=ERR_VECT_MORSA_RULLI_Q, Arresto:=TRUE);
                        //           ELSE
                        //               iret:=Push_error(code:=ERR_VECT_MORSA_RULLI_Q, Arresto:=FALSE);
                        //           END_IF;
                        //       END_IF;
                        //   END_IF;
                    //   {#else}    //FINE NO ELECTMRL
                        //   IF W3_W4_W6_Q1.error=YES THEN
                        //       IF W3_W4_W6_Q1.errorreaction<>NONE THEN
                        //           //Ferma tutti gli altri assi
                        //           i_sys_stopcond:=1;
                        //           IF W3_W4_W6_Q1.linearPathCommand.state=ACTIVE THEN
                        //               iret:=Push_error(code:=ERR_VECT_MORSA_RULLI_Q, Arresto:=TRUE);
                        //           ELSE
                        //               iret:=Push_error(code:=ERR_VECT_MORSA_RULLI_Q, Arresto:=FALSE);
                        //           END_IF;
                        //       END_IF;
                        //   END_IF;
            //           {#endif} //FINE ELECTMRL
            //       {#endif}  //Fine NO ORBITALE
            //   {#endif}  //Fine NO ETURN
            //   {#ifdef MATRICE_ELETTRICA}
                //   IF TestAxesY2()=TRUE THEN
                //       IF Y1_U4_Y2.error=YES THEN
                //           IF Y1_U4_Y2.errorreaction<>NONE THEN
                //               //Ferma tutti gli altri assi
                //               i_sys_stopcond:=1;
                //               IF Y1_U4_Y2.linearPathCommand.state=ACTIVE THEN
                //                   iret:=Push_error(code:=ERR_VECT_Y1_U4_Y2, Arresto:=TRUE);
                //               ELSE
                //                   iret:=Push_error(code:=ERR_VECT_Y1_U4_Y2, Arresto:=FALSE);
                //               END_IF;
                //           END_IF;
                //       END_IF;
                //       IF Y1_Y2.error=YES THEN
                //           IF Y1_Y2.errorreaction<>NONE THEN
                //               //Ferma tutti gli altri assi
                //               i_sys_stopcond:=1;
                //               IF Y1_Y2.linearPathCommand.state=ACTIVE THEN
                //                   iret:=Push_error(code:=ERR_VECT_Y1_Y2, Arresto:=TRUE);
                //               ELSE
                //                   iret:=Push_error(code:=ERR_VECT_Y1_Y2, Arresto:=FALSE);
                //               END_IF;
                //           END_IF;
                //       END_IF;
                //   END_IF;
            //   {#endif}  //Fine MATRICE_ELETTRICA
            //   {#ifdef SOLLEVAMENTO_SLITTA_SPALLAMENTO}
                //   IF Q1_V1_V4.error=YES THEN
                //       IF Q1_V1_V4.errorreaction<>NONE THEN
                //           //Ferma tutti gli altri assi
                //           i_sys_stopcond:=1;
                //           IF Q1_V1_V4.linearPathCommand.state=ACTIVE THEN
                //               iret:=Push_error(code:=ERR_VECT_Q_V1_V4, Arresto:=TRUE);
                //           ELSE
                //               iret:=Push_error(code:=ERR_VECT_Q_V1_V4, Arresto:=FALSE);
                //           END_IF;
                //       END_IF;
                //   END_IF;
                //   IF Q1_V1.error=YES THEN
                //       IF Q1_V1.errorreaction<>NONE THEN
                //           //Ferma tutti gli altri assi
                //           i_sys_stopcond:=1;
                //           IF Q1_V1.linearPathCommand.state=ACTIVE THEN
                //               iret:=Push_error(code:=ERR_VECT_Q_V1_V4, Arresto:=TRUE);
                //           ELSE
                //               iret:=Push_error(code:=ERR_VECT_Q_V1_V4, Arresto:=FALSE);
                //           END_IF;
                //       END_IF;
                //   END_IF;
            //   {#endif}	//Fine SOLLEVAMENTO_SLITTA_SPALLAMENTO
            
            //    {::ifdef INTERPOLA_Q1_V1_Z1}
                        Q1_V1_Z1:= AsKinematicsRef(KinAxesQ1_V1_Z1_def);
                       IF Q1_V1_Z1^.StatusWord.%X1=TRUE THEN
                          IF Q1_V1_Z1^.ErrorDetail.Reaction<>0 THEN
                          //Ferma tutti gli altri assi
                          i_sys_stopcond:=1;
                          IF Q1_V1_Z1^.StatusWord.%X8 THEN
                              iret:=Push_error(code:=ERR_VECT_TESTA_Q_T, Arresto:=TRUE);
                          ELSE
                              iret:=Push_error(code:=ERR_VECT_TESTA_Q_T, Arresto:=FALSE);
                          END_IF;
                      END_IF;
                  END_IF;
            //    {::endif}	//Fine INTERPOLA_Q1_V1_Z1
              IF i_sys_OPMODE=JOGMODE AND
                   i_sys_activedevice<FIRST_COMPLEX_OPERATION AND
                   TaskCicliAttivo THEN
                  (*dwret:=_resetTaskId(_task.motiontask_CICLI);*)
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_ELIMINATE_HOMING_COMMANDS;
              ELSE
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG;
              END_IF;

          _FB_MCP483#STATE_MCP483_ELIMINATE_HOMING_COMMANDS:
              IF EliminaCodaComandi.outputState<>GENERAL_STATE#STATE_END THEN
                  EliminaCodaComandi();
              ELSE
                  EliminaCodaComandi();
                  i_sys_stopcond:=0;
                  TaskCicliAttivo:=FALSE;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG;
              END_IF;

        _FB_MCP483#STATE_MCP483_GEST_JOG:
              //Gestisce il movimento in JOG
              IF i_sys_OPMODE=JOGMODE AND i_sys_activedevice>=_FIRST_AXIS#FIRST_AXIS AND i_sys_STOPCOND=0 THEN
                  //Verifica che non sia rimasto nessun asse in movimento
                  IF i_sys_activedevice>=_FIRST_AXIS#FIRST_AXIS AND i_sys_Activedevice<_MAXIS#MAXAXIS+_FIRST_AXIS#FIRST_AXIS THEN
                      //Verifica che non sia rimasto nessun asse in movimento
                      IF i_sys_activedevice>=_FIRST_AXIS#FIRST_AXIS AND i_sys_Activedevice<_MAXIS#MAXAXIS+_FIRST_AXIS#FIRST_AXIS THEN
                          //E^ un asse
                          // verifica e gestisce il JOG per assi reali
                        //   {#ifdef BLOCCAGGIO_DX}
                              IF i_sys_activedevice=W3 THEN
                                  AppoDirezione:=SN;
                              ELSE
                                  AppoDirezione:=SN_SP;
                              END_IF;
                        //   {#else}  //Fine BLOCCAGGIO_DX
                              AppoDirezione:=SN_SP;
                        //   {#endif}  //Fine NO BLOCCAGGIO_DX
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_SET_CONDITION_MOVE_AXIS;
                      ELSE
                          IF i_sys_activedevice>=FIRST_DEVICE AND i_sys_Activedevice<FIRST_DEVICE+_MAXDEVICES#MAXDEVICES THEN
                              //E^ un dispositivo
                              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_AXIS_RESET_MOTION;
                          ELSE
                              IF TaskCicliAttivo=FALSE THEN
                                  //Comandi manuali complessi
                                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_TASK_ACTIVE;
                              ELSE
                                  CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                              END_IF;
                          END_IF;
                      END_IF;
                  ELSE
                      CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                  END_IF;
              ELSE
                  CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_SET_CONDITION_MOVE_AXIS:
              IF Set_Condizione_Move_Asse.outputState<>GENERAL_STATE#STATE_END THEN
                  SET_Condizione_Move_Asse(Asse:=i_sys_activedevice);
              ELSE
                  SET_Condizione_Move_Asse();
                  IF OK_Condizione_Move_Asse(Asse:=i_sys_activedevice, Direzione:=AppoDirezione)=TRUE THEN
                      reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
                      IF i_sys_activedevice>=_FIRST_AXIS#FIRST_AXIS AND i_sys_Activedevice<_MAXIS#MAXAXIS+_FIRST_AXIS#FIRST_AXIS THEN
                          IF TestAssePronto(Asse:=i_sys_activedevice)=FALSE THEN
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_ENABLE_AXIS_BLM;
                          ELSE
                              StatePower:=TRUE;
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_CHECK_ENABLE_AXIS_BLM;
                          END_IF;
                      ELSE
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_CHECK_ENABLE_AXIS_BLM;
                      END_IF;
                  ELSE
                      TaskCicliAttivo:=FALSE;
                      CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                  END_IF;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_ENABLE_AXIS_BLM:
              IF EnableAxisBLM.outputState<>GENERAL_STATE#STATE_END THEN
                  EnableAxisBlm(Axis:=Axes[i_sys_activedevice].PosAx,
                                              NumeroAsse:=i_sys_activedevice
                                              (*ModoAttesa:=IMMEDIATELY*));
              ELSE
                  EnableAxisBlm();
                  StatePower:=FALSE;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_CHECK_ENABLE_AXIS_BLM;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_CHECK_ENABLE_AXIS_BLM:
              IF StatePower=TRUE THEN ;
                // //   {#ifdef PINZA_ELETTRICA}%%%%
                //     PosAx:=AsSynchronousAxisRef(Axes[W1].PosAx);
                //       IF i_sys_activedevice=W1 THEN
                //           IF PosAx^.StatusWord.%X7 THEN
                //               //Impostazione coppia pinza chiusa
                //               CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_ENABLE_COUPLE_W1_GENERIC;
                //           ELSE
                //             //   {#ifdef ELECTXL}
                //                   //Movimento manuale pinza a coppia ridotta (di calibrazione)
                //                   CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_ENABLE_PAIR_W1_ELECTXL;
                //             //   {#else}
                //             //       {#ifdef ETURN_MR}
                //                       //Movimento manuale pinza a coppia massima
                //                       CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_DISABLE_COUPLE_W1_ETURN;
                //                 //   {#else}
                //                       CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_JOG_ASSE;
                //             //       {#endif};
                //             //   {#endif};
                //           END_IF;
                //       ELSE
                //           CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_JOG_ASSE;
                //       END_IF;
                // //   {#else}
                //       CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_JOG_ASSE;
                // //   {#endif};
              ELSE
                  TaskCicliAttivo:=FALSE;
                  CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_ENABLE_COUPLE_W1_GENERIC:
          ;  //   {#ifdef PINZA_ELETTRICA}
                //   IF EnableAsseInCoppia.outputState<>GENERAL_STATE#STATE_END THEN %%%%
                //       EnableAsseInCoppia(Coppia:=i_sys_CoppiaPinzaChiusa, NumeroAsse:=W1, Attesa:=FALSE);
                //   ELSE
                //       EnableAsseInCoppia();
                //       CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_JOG_ASSE;
                //   END_IF;
            //   {#endif};

          _FB_MCP483#STATE_MCP483_GEST_JOG_ENABLE_PAIR_W1_ELECTXL:
            ;//   {#ifdef ELECTXL}
                //   IF EnableAsseInCoppia.outputState<>GENERAL_STATE#STATE_END THEN %%%%
                //       EnableAsseInCoppia(Coppia:=BlmMxToolsOffPezzo[W1].CoppiaRidotta*10,
                //                                            NumeroAsse:=W1,
                //                                            Attesa:=FALSE);
                //   ELSE
                //       EnableAsseInCoppia();
                //       CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_JOG_ASSE;
                //   END_IF;
            //   {#endif};

          _FB_MCP483#STATE_MCP483_GEST_JOG_DISABLE_COUPLE_W1_ETURN:
            //   {#ifdef ETURN_MR}
                  IF DisableAsseInCoppia.outputState<>GENERAL_STATE#STATE_END THEN
                      DisableAsseInCoppia(NumeroAsse:=W1, Verifica:=FALSE);
                  ELSE
                      DisableAsseInCoppia();
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_JOG_ASSE;
                  END_IF;
            //   {#endif};

          _FB_MCP483#STATE_MCP483_GEST_JOG_JOG_ASSE:
              IF Jog_Asse.outputState<>GENERAL_STATE#STATE_END THEN
                  Jog_Asse(Asse:=i_sys_activedevice);
              ELSE
                  Jog_Asse();
                  TaskCicliAttivo:=FALSE;
                  CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_AXIS_RESET_MOTION:
              CURRENT_AXIS:=CURRENT_AXIS+1;
              IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
                  IF TestDispConAsse(Disp:=i_sys_activedevice, Asse:=CURRENT_AXIS)=FALSE THEN
                      //Verifica che la movimentazione del dispositivo non debba anche essere associata ad una posizionamento di un asse,
                      //in tale caso non sarà possibile effettuare il reset di sicurezza in quanto si annullerebbe il necessario
                      //posizionamento dell^asse
                      PosAx:= AsSynchronousAxisRef(Axes[CURRENT_AXIS].PosAx);
                      PosAx:=AsSynchronousAxisRef(Axes[CURRENT_AXIS].posax);
                      IF PosAx^.StatusPositioning.Distance >0.01 AND posax^.simulation.Mode=UDINT#0 THEN
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_RESET_MOTION_RESET_AXIS;
                      END_IF;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_SET_CONDITION_MOVE_DEVICE ;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_RESET_MOTION_RESET_AXIS:
              IF ResetMotionResetAxis.outputState<>GENERAL_STATE#STATE_END THEN
                  //Per sicurezza si resettano gli assi che hanno ancora un percorso residuo
                  ResetMotionResetAxis(Asse:=CURRENT_AXIS, Attesa:=FALSE);
              ELSE
                  ResetMotionResetAxis();
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_AXIS_RESET_MOTION;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_SET_CONDITION_MOVE_DEVICE :
              IF Set_Condizione_Move_Dispositivo.outputState<>GENERAL_STATE#STATE_END THEN
                  SET_Condizione_Move_Dispositivo(i_sys_activedevice);
              ELSE
                  SET_Condizione_Move_Dispositivo();
                  IF R_Plus.Q=TRUE AND Clk_minus=FALSE THEN
                      IF i_sys_monitoring<>0 OR i_sys_Accesslevel<=ACCESS_MANUTENTORE OR SbloccaMonitoring(Asse:=i_sys_Activedevice) THEN
                          Esito:=OK_Condizione_Move_Dispositivo(Dispositivo:=i_sys_activedevice,
                                                                                                      Stato:=LAVORO,
                                                                                                      TastoPiu:=TRUE,
                                                                                                      TastoMeno:=FALSE);
                          IF Esito=TRUE THEN
                              reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_MOVE_DEVICE ;
                          ELSE
                              TaskCicliAttivo:=FALSE;
                              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                          END_IF;
                      ELSE
                          TaskCicliAttivo:=FALSE;
                          set_trace(Code:=TRC_JOG_NON_IN_MONITORING);
                          CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                      END_IF;
                  ELSE
                      IF R_Minus.Q=TRUE AND Clk_plus=FALSE THEN
                          IF i_sys_monitoring<>0 OR i_sys_Accesslevel<=ACCESS_MANUTENTORE OR SbloccaMonitoring(Asse:=i_sys_Activedevice) THEN
                              Esito:=OK_Condizione_Move_Dispositivo(Dispositivo:=i_sys_activedevice,
                                                                                                          Stato:=RIPOSO,
                                                                                                          TastoPiu:=FALSE,
                                                                                                          TastoMeno:=TRUE);
                              IF Esito=TRUE THEN
                                  reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
                                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_MOVE_DEVICE_REST;
                              ELSE
                                  TaskCicliAttivo:=FALSE;
                                  CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                              END_IF;
                          ELSE
                              TaskCicliAttivo:=FALSE;
                              set_trace(Code:=TRC_JOG_NON_IN_MONITORING);
                              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                          END_IF;
                      ELSE
                          IF Clk_minus=FALSE AND Clk_plus=FALSE THEN
                              IF Disp[i_sys_activedevice].PresenzaCostante=TRUE THEN
                                  WriteDigitalOutput(NumOutput:=Disp[i_sys_activedevice].out_lavoro, Value:=FALSE);
                                  WriteDigitalOutput(NumOutput:=Disp[i_sys_activedevice].out_riposo, Value:=FALSE);
                              END_IF;
                              reset_trace(Code:=TRC_JOG_NON_IN_MONITORING);
                              TaskCicliAttivo:=FALSE;
                              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                          ELSE
                              IF Disp[i_sys_activedevice].spegnimentouscita=TRUE THEN
                                  IF Disp[i_sys_activedevice].Stato=Disp[i_sys_activedevice].StatoRichiesto THEN
                                      WriteDigitalOutput(NumOutput:=Disp[i_sys_activedevice].out_lavoro, Value:=FALSE);
                                      WriteDigitalOutput(NumOutput:=Disp[i_sys_activedevice].out_riposo, Value:=FALSE);
                                  END_IF;
                              END_IF;
                              TaskCicliAttivo:=FALSE;
                              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
                          END_IF;
                      END_IF;
                  END_IF;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_MOVE_DEVICE :
              IF MuoviDispositivo.outputState<>GENERAL_STATE#STATE_END THEN
                  MuoviDispositivo(Dispositivo:=i_sys_activedevice, Stato:=LAVORO);
              ELSE
                  MuoviDispositivo();
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_MOVE_DEVICE_REST:
              IF MuoviDispositivo.outputState<>GENERAL_STATE#STATE_END THEN
                  MuoviDispositivo(Dispositivo:=i_sys_activedevice, Stato:=RIPOSO);
              ELSE
                  MuoviDispositivo();
                  CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_TASK_ACTIVE:
              IF TaskCicliAttivo=FALSE THEN
                  //Comandi manuali complessi
                  IF i_sys_activedevice>=FIRST_COMPLEX_OPERATION AND
                       i_sys_activedevice<>C_DISATTIVAZIONE_BOOSTER_X1 AND
                       i_sys_activedevice<>C_IMPOSTAZIONI_ASSI_DISATTIVABILI AND
                       i_sys_activedevice<>C_TENSIONE_FREQUENZA AND
                       i_sys_activedevice<>C_ATTIVAZIONE_V1 AND
                       i_sys_activedevice<>C_DISATTIVAZIONE_V1 AND
                       i_sys_activedevice<>C_IMPOSTA_MANIPOLATORE2D AND
                       i_sys_activedevice<>C_MACCHINA_DX AND
                       i_sys_activedevice<>C_MACCHINA_SX AND
                       i_sys_activedevice<>C_RESET_SIMOTION AND
                       i_sys_activedevice<>C_ON_4RFORM AND
                       i_sys_activedevice<=ULTIMA_OPERAZIONE_COMPLESSA THEN
                      (*dwret:=_restartTaskId(_task.motiontask_CICLI);%%% rewrite with var the action*);
                  END_IF;
              END_IF;
              CURRENT_AXIS:=_FIRST_AXIS#FIRST_AXIS-1;
              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS;

          _FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS:
              IF i_sys_OPMODE=JOGMODE AND i_sys_activedevice<_FIRST_AXIS#FIRST_AXIS THEN
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_AXES_ELIMINATES_TAIL_COMMANDS_LOOP;
              ELSE
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_HOME_ASSI_REALI;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_JOG_LOOP_AXES_ELIMINATES_TAIL_COMMANDS_LOOP:

              FOR i:=0 TO _MAXIS#MAXAXIS-1 BY 1 DO
                  IF TestToAttivo(Asse:=i)=TRUE THEN
                    Posax:= AsSynchronousAxisRef(Axes[i].Posax);
                    // Posax:= AsSynchronousAxisRef(UINT#100);
                      IF Posax^.Velocity <>0 THEN //Velocity
                          IF Posax^.StatusWord.%X6 THEN
                          //IF Axes[i].Posax.motionstatedata.motioncommand<>MOTION_DONE THEN
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_ELIMINATES_CONTROL_LOOP;
                              EXIT;
                          END_IF;
                      END_IF;
                  END_IF;
              END_FOR;
              IF CURRENT_STATE<>_FB_MCP483#STATE_MCP483_GEST_JOG_ELIMINATES_CONTROL_LOOP THEN
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_HOME_ASSI_REALI;
              END_IF;
              (*CURRENT_AXIS:=CURRENT_AXIS+1;
              IF CURRENT_AXIS<_MAXIS#MAXAXIS THEN
                  IF TestToAttivo(Asse:=CURRENT_AXIS)=TRUE THEN
                      IF Axes[CURRENT_AXIS].Posax.motionstatedata.commandvelocity<>0 THEN
                          IF Axes[CURRENT_AXIS].Posax.motionstatedata.motioncommand<>MOTION_DONE THEN
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_JOG_ELIMINATES_CONTROL_LOOP;
                          END_IF;
                      END_IF;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_HOME_ASSI_REALI;
              END_IF;*)

          _FB_MCP483#STATE_MCP483_GEST_JOG_ELIMINATES_CONTROL_LOOP:
              IF EliminaCodaComandi.outputState<>GENERAL_STATE#STATE_END THEN
                  //Arresto assi se rimane attivo un comando ma l^asse è stato disattivato
                  EliminaCodaComandi();
              ELSE
                  EliminaCodaComandi();
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_HOME_ASSI_REALI;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_HOME_ASSI_REALI:
              //Gestisce l^HOME solo per assi reali
              IF i_sys_OPMODE=REFMODE THEN
                  IF i_sys_activedevice>=_FIRST_AXIS#FIRST_AXIS THEN
                      IF i_sys_Activedevice<_MAXIS#MAXAXIS+_FIRST_AXIS#FIRST_AXIS THEN
                        If Posax = NULL THEN
                        Posax:= AsSynchronousAxisRef(Axes[i_sys_Activedevice].PosAx);
                        If Posax = NULL THEN;
                        END_IF;
                    END_IF;
                          IF PosAx^.VirtualAxis.Mode = UDINT#0 THEN
                              IF GetEncoderAbsolute(i_sys_Activedevice) THEN
                                  IF mcpi_plus=TRUE THEN
                                      //Serve per poter eseguire l^index dell^asse anche con barriere violate
                                      IF i_sys_accesslevel<=access.index THEN
                                          reset_trace(TRC_LIVELLO_DI_ACCESSO_BASSO);
                                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_HOME_HOMING_ABSOLUTE;
                                      ELSE
                                          //Errore livello troppo basso
                                          set_trace(code:=TRC_LIVELLO_DI_ACCESSO_BASSO);
                                      END_IF;
                                  ELSE
                                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_WATCHDOG_STELIA;
                                  END_IF;
                              ELSE
                                  IF R_PLUS.Q THEN
                                      // lancia task cicli per la presa index di encoder incrementali
                                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_GEST_HOME_INCREMENTAL_SET_MOVE_AXIS;
                                  ELSE
                                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_WATCHDOG_STELIA;
                                  END_IF;
                              END_IF;
                          ELSE
                              CURRENT_STATE:=_FB_MCP483#STATE_MCP483_WATCHDOG_STELIA;
                          END_IF;
                      ELSE
                          CURRENT_STATE:=_FB_MCP483#STATE_MCP483_WATCHDOG_STELIA;
                      END_IF;
                  ELSE
                      CURRENT_STATE:=_FB_MCP483#STATE_MCP483_WATCHDOG_STELIA;
                  END_IF;
              ELSE
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_WATCHDOG_STELIA;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_HOME_HOMING_ABSOLUTE:
              IF Homing_Absolute.outputState<>GENERAL_STATE#STATE_END THEN
                  Homing_Absolute(Asse:=i_sys_activedevice, Position:=pm[i_sys_Activedevice].MA_Refp_Set_Pos);
              ELSE
                  Homing_Absolute();
                  i_sys_chan_request:=1;
                  i_SYS_OPMODE_request:=JOGMODE;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_WATCHDOG_STELIA;
              END_IF;

          _FB_MCP483#STATE_MCP483_GEST_HOME_INCREMENTAL_SET_MOVE_AXIS:
              IF Set_Condizione_Move_Asse.outputState<>GENERAL_STATE#STATE_END THEN
                  SET_Condizione_Move_Asse(Asse:=i_sys_activedevice);
              ELSE
                  SET_Condizione_Move_Asse();
                  IF OK_Condizione_Move_Asse(Asse:=i_sys_activedevice, Direzione:=SN_SP) THEN
                      reset_trace(Code:=TRC_SAFETY_JOG_DEVICES);
                      (*dwret:=_restartTaskId(_task.motiontask_CICLI);*)
                  END_IF;
                  CURRENT_STATE:=_FB_MCP483#STATE_MCP483_WATCHDOG_STELIA;
              END_IF;

          _FB_MCP483#STATE_MCP483_WATCHDOG_STELIA:
              WatchDogStelia();
              CURRENT_STATE:=GENERAL_STATE#STATE_END;

          GENERAL_STATE#STATE_END:
              CURRENT_STATE:=GENERAL_STATE#STATE_INIT;

      END_CASE;
      OutputState:=CURRENT_STATE;
  END_FUNCTION_BLOCK

/**************************************************************************************************
 * WHEN THE TIMER T_NASTRO_FULL EXPIRES, IT CHECKS IF THE FULL BELT INPUT IS STILL ON AND IN THAT *
 *                                CASE IT STOPS THE CONVEYOR BELT                                 *
 **************************************************************************************************/
(*{
#ifndef ORBITALE
}*)
	FUNCTION ArrestoNastroPieno

        VAR_EXTERNAL
            TimerSistema                    :ARRAY[0..MAX_TIMER_SISTEMA-1] OF Timer;
            OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
        	Clk_NastroPieno					:BOOL;
	        R_NastroPieno							:R_TRIG;	//Fronte salita nastro trasportatore pieno
            TimeNastFull				    :INT;
        END_VAR

        VAR_EXTERNAL CONSTANT
            MAX_TIMER_SISTEMA 	        :USINT;
            MAXDIGITALOUTPUT            :INT;
            T_NAST_FULL 				:INT;
            OUT_NAST_TRANSPORTER        :INT;
            INP_NAST_TRANSPORTER_FULL	:INT;
        END_VAR

		Clk_NastroPieno:=TimerSistema[T_NAST_FULL].Executed;
		R_NastroPieno(clk:=Clk_NastroPieno);
		IF R_NastroPieno.q=TRUE AND ReadDigitalInput(NumInput:=INP_NAST_TRANSPORTER_FULL)=TRUE THEN
			ComandoNastroTrasportatore(Stato:=FALSE);
		END_IF;
		IF ReadDigitalInput(NumInput:=INP_NAST_TRANSPORTER_FULL)=TRUE AND
			 TimerSistema[T_NAST_FULL].Status=0 AND
			 OAddress[OUT_NAST_TRANSPORTER].Stato=TRUE THEN
			//Partenza timer se è attivo l^ingresso di nastro pieno e l^uscita di comando del nastro
			TimerSistema[T_NAST_FULL].Value:=TimeNastFull;
			TimerSistema[T_NAST_FULL].Status:=SINT#1;
		END_IF;
	END_FUNCTION
(*{
#endif	//Fine NO ORBITALE
}*)

/******************************************************************************************
 * PERFORMS SELECTION MANAGEMENT VIA PNPN OR DPDP OF THE PRODUCTION TABLE TO BE ACTIVATED *
 ******************************************************************************************/

FUNCTION GestioneTabelleProduzione

    VAR_EXTERNAL
        i_sys_OPMODE                    : INT;	// MODO OPERATIVO
                                                // 0 = JOG
                                                // 1 = REFERENCE
                                                // 2 = MDA
                                                // 3 = AUTO
        DatiSelExtProd					:SelExtProd;
        F_InpCodiceTabellaOK	        :F_TRIG; //fronte discesa InpCodiceTabellaOK
    END_VAR

    VAR_EXTERNAL CONSTANT
        JOGMODE                               :INT;
    END_VAR
	VAR
		AppoOutCodiceTabellaOK    :BOOL:=FALSE;
		AppoCodiceErroreTabExt    :INT:=0;
		i                                                :INT;
	END_VAR
	IF i_sys_OPMODE=JOGMODE AND DatiSelExtProd.AbilSelExtTabella=TRUE THEN
		//Lettura dal bus di campo delle informazioni provenienti dal supervisore
		LetturaDatiTabellaProduzione(CodiceTabella:=DatiSelExtProd.CodiceTabellaDaCaricare,
																 AbilPezSegTabExt:=DatiSelExtProd.AbilPezSegTabExt,
																 InpCodiceTabellaOK:=DatiSelExtProd.InpCodiceTabellaOK);
		//Abilitazione sorveglianza sul cambio di stato del segnale di strobe di convalida
		F_InpCodiceTabellaOK(clk:=DatiSelExtProd.InpCodiceTabellaOK);
		IF F_InpCodiceTabellaOK.q=TRUE THEN
			//Il fronte di discesa della variabile "InpCodiceTabellaOK" indica che la codifica della tabella di produzione non è più coerente.
			//Quindi si resetta la variabile "OutCodiceTabellaOK" che indica al supervissore lo stato della tabella di produzione.
			DatiSelExtProd.OutCodiceTabellaOK:=FALSE;
			//Si resetta (-1) anche il codice di un eventuale errore di attivazione della tabella di produzione
			DatiSelExtProd.CodiceErroreTabExt:=-1;
		END_IF;
		IF DatiSelExtProd.CodiceErroreTabExt=0 THEN
			DatiSelExtProd.OutCodiceTabellaOK:=TRUE;
		ELSE
			IF DatiSelExtProd.CodiceErroreTabExt>0 THEN
				//In caso di errore viene resetta la variabile che indica che la tabella di produzione richiesta è stata trasferita al Simotion
				DatiSelExtProd.OutCodiceTabellaOK:=FALSE;
				//Invalida i dati dei pezzi in quanto dovrà essere inviata la produzione
				InvalidaProgrammi();
			END_IF;
		END_IF;
		IF DatiSelExtProd.InpCodiceTabellaOK=FALSE THEN
			DatiSelExtProd.OutCodiceTabellaOK:=FALSE;
		END_IF;
		ScritturaDatiTabellaProduzione(CodiceErroreTabExt:=DatiSelExtProd.CodiceErroreTabExt, OutCodiceTabellaOK:=DatiSelExtProd.OutCodiceTabellaOK);
	ELSE
		AppoOutCodiceTabellaOK:=FALSE;
		AppoCodiceErroreTabExt:=0;
		ScritturaDatiTabellaProduzione(CodiceErroreTabExt:=AppoCodiceErroreTabExt, OutCodiceTabellaOK:=AppoOutCodiceTabellaOK);
	END_IF;
END_FUNCTION

/**********************************************************************************************************************
 *                  VIA DPDP OR PNPN READS THE FOLLOWING INFORMATION WHICH IS SET BY THE SUPERVISOR:                  *
 *                  -BYTE X05 (LOW) AND X06(HIGH) => NUMBER OF THE PRODUCTION TABLE TO BE ACTIVATED                   *
 *               -BYTE X04 BIT 0 => STROBE TO VALIDATE THE CODE OF THE PRODUCTION TABLE TO BE ACTIVATED               *
 * -BYTE X04 BIT 1 => TRUE IN THE ACTIVE TABLE ENABLES THE FOLLOWING WORKPIECE MODE, FALSE ENABLES EXTERNAL SELECTION *
 **********************************************************************************************************************/

FUNCTION LetturaDatiTabellaProduzione

    VAR_EXTERNAL
        i_sys_DpCoupler                 :BOOL;
        V_pncoupler_i                   : pncoupler_i;
        V_dbcoupler_i                   : dbcoupler_i;
        i_sys_PnCoupler                 :BOOL;
    END_VAR

	VAR_IN_OUT
		CodiceTabella			:DINT;	//Numero della tabella di produzione che deve essere attivata
		AbilPezSegTabExt	    :BOOL;	//TRUE abilita il pezzo seguente, FALSE abilita la selezione esterna
		InpCodiceTabellaOK      :BOOL;	//Strobe di convalida dei dati di input
	END_VAR

    VAR
		AppoByteBasso	:WORD;
		AppoByteAlto	:WORD;
        tmpword:word;
	END_VAR
	// {
	// #ifndef EMULAZIONE
	// }
		AppoByteBasso:=WORD#0;
		AppoByteAlto:=WORD#0;
		IF i_sys_DpCoupler=TRUE THEN
			AppoByteBasso:=V_dbcoupler_i.address_db_i[5];
			AppoByteAlto:=V_dbcoupler_i.address_db_i[6];
			InpCodiceTabellaOK:=getbit(V_dbcoupler_i.address_db_i[4], USINT#0);
			AbilPezSegTabExt:=getbit(V_dbcoupler_i.address_db_i[4], USINT#1);
		ELSE
			// {
			// #ifdef PROFINET
			// }
				IF i_sys_PnCoupler=TRUE THEN
					AppoByteBasso:=V_pncoupler_i.address_pn_i[5];
					AppoByteAlto:=V_pncoupler_i.address_pn_i[6];
					InpCodiceTabellaOK:=getbit(V_pncoupler_i.address_pn_i[4], USINT#0);
					AbilPezSegTabExt:=getbit(V_pncoupler_i.address_pn_i[4], USINT#1);
				END_IF;
			// {
			// #endif	//Fine PROFINET
			// }
			;
		END_IF;


		CodiceTabella:=TO_DINT(SHL(AppoByteAlto,uint#8) or AppoByteBasso);


	// {
	// #endif	//Fine NO EMULAZIONE
	// }
	;
END_FUNCTION

/*****************************************************************************************************
 *            WRITES INFORMATION TO DPDP OR PNPN WHICH IS TRANSFERRED TO THE SUPERVISOR:             *
 * -BYTE X42 BIT 3 => TRUE THE PRODUCTION TABLE CANNOT BE ACTIVATED DUE TO AN ERROR INDICATED BY VGP *
 *      -BYTE X42 BIT 5 => TRUE THE PRODUCTION TABLE HAS BEEN ACTIVATED AND NO ERROR IS PRESENT      *
 *****************************************************************************************************/
(* *)
FUNCTION ScritturaDatiTabellaProduzione

    VAR_EXTERNAL
        i_sys_DpCoupler                 :BOOL;
        i_sys_PnCoupler                 :BOOL;
        V_pncoupler_o                   : pncoupler_o;
        V_dbcoupler_o                   : dbcoupler_o;

    END_VAR

	VAR_IN_OUT
		CodiceErroreTabExt	:INT;		//True errore nell^attivazione della tabella di produzione, False nessun errore
		OutCodiceTabellaOK	:BOOL;	//True la tabella di produzione è stata attivata
	END_VAR

    VAR_TEMP
        Local_dbcoupler_o42                   : BYTE;
        Local_pncoupler_o42                   : BYTE;
    END_VAR
	IF i_sys_DpCoupler=TRUE THEN
        Local_dbcoupler_o42:=V_dbcoupler_o.address_db_o[42];
		V_dbcoupler_o.address_db_o[42]:=setbit(Local_dbcoupler_o42, USINT#3, TO_BOOL(MAX(0,CodiceErroreTabExt)));
	ELSE
		// {
		// #ifdef PROFINET
		// }
			IF i_sys_PnCoupler=TRUE THEN
                Local_pncoupler_o42:=V_pncoupler_o.address_pn_o[42];
				V_pncoupler_o.address_pn_o[42]:=setbit(Local_pncoupler_o42, USINT#3, TO_BOOL(MAX(0,CodiceErroreTabExt)));
				V_pncoupler_o.address_pn_o[42]:=setbit(Local_pncoupler_o42, USINT#5, OutCodiceTabellaOK);
			END_IF;
		// {
		// #endif    //Fine PROFINET
		// }
		;
	END_IF;
END_FUNCTION

// {
// #ifdef DH4010
// }
	FUNCTION TestCicloSpedizioneEseguito:BOOL

    VAR_EXTERNAL
        pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters;
        Axes                            :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF ax;/// ax as structure is defined in _TYPE
        i_sys_CicloSpedizioneEseguito   :BOOL;     //True se è stato eseguito il ciclo di spedizione
    END_VAR

    VAR_EXTERNAL CONSTANT

        X1:INT;
        X2:INT;
    END_VAR

    VAR
        PosAx : REF_TO TO_SynchronousAxis;
    END_VAR

        PosAx:= AsSynchronousAxisRef(Axes[X2].PosAx);
		// IF (ABS(pm[X2].MA_Spedizione_BLM-PosAx^.actualPosition)<pm[X2].MA_Tolerance AND   //takeBack
		// 		pm[X2].MA_Spedizione_BLM<pm[X2].Ma_Pos_Limit_Minus AND pm[X2].MA_Spedizione_BLM>pm[X2].Ma_Pos_Limit_Plus) OR
		// 	 i_sys_CicloSpedizioneEseguito=TRUE THEN
		// 	TestCicloSpedizioneEseguito:=TRUE;
		// ELSE
		// 	TestCicloSpedizioneEseguito:=FALSE;
		// END_IF;
        // PosAx:= AsSynchronousAxisRef(Axes[X1].PosAx);
		// IF (ABS(pm[X1].MA_Spedizione_BLM-PosAx^.actualPosition)<pm[X1].MA_Tolerance AND
		// 		pm[X1].MA_Spedizione_BLM<pm[X1].Ma_Pos_Limit_Minus AND pm[X1].MA_Spedizione_BLM>pm[X1].Ma_Pos_Limit_Plus) OR
		// 		i_sys_CicloSpedizioneEseguito=TRUE THEN
		// 	TestCicloSpedizioneEseguito:=TRUE;
		// ELSE
		// 	TestCicloSpedizioneEseguito:=FALSE;
		// END_IF;
	END_FUNCTION

/******************************************************************************************************
 * RETURNS TRUE IF IT IS INTENDED THAT THE MOVEMENT IN JOG OF THE DEVICE PASSED AS A PARAMETER CAN BE *
 *                              ASSOCIATED WITH A POSITIONING OF AN AXIS                              *
 ******************************************************************************************************/
FUNCTION TestDispConAsse:BOOL

    VAR_EXTERNAL CONSTANT
        Z1:INT;
        PALETTA                               :INT;
    END_VAR

	VAR_INPUT
		Disp    :INT;
		Asse    :INT;
	END_VAR
	TestDispConAsse:=FALSE;
	// // { %%%%
	// // #ifdef ORBITALE
	// // }
	// // 	{
	// // 	#ifdef DH4010
	// // 	}
	// 		IF Disp=SCOMP_MANIPOLATORE_3D AND Asse=Z1 THEN
	// 			//Il dispositivo SCOMP_MANIPOLATORE_3D per potersi muovere deve prima posizionare a 65° l^asse Z1
	// 			TestDispConAsse:=TRUE;
	// 		END_IF;
	// // 	{
	// // 	#endif  //Fine DH4010
	// // 	}
	// // {
	// // #else    //Fine ORBITALE
	// // }
		IF Disp=PALETTA AND Asse=Z1 THEN
			//Il dispositivo PALETTA per potersi muovere deve prima posizionare a 0° l^asse Z1
			TestDispConAsse:=TRUE;
		END_IF;
	// {
	// #endif  //Fine NO ORBITALE
	// }
END_FUNCTION

    FUNCTION WatchdogStelia

        VAR_EXTERNAL
            OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
        END_VAR

        VAR_EXTERNAL CONSTANT
            MAXDIGITALOUTPUT                    :INT;
            OUT_01_STOP_POINT                             :INT;
            OUT_02_STOP_POINT                             :INT;
            OUT_03_STOP_POINT                             :INT;
            OUT_04_STOP_POINT                             :INT;

            INP_01_STOP_POINT                           :INT;
            INP_02_STOP_POINT                           :INT;
            INP_03_STOP_POINT                           :INT;
        END_VAR


        VAR
          iret  :INT;
        END_VAR
        ;
        // IF i_sys_CM_1243_5=TRUE THEN
        // //   {
        // //   #ifdef ELECTM
        // //   }
        // //     {
        // //     #ifndef PROFINET
        // //     }
        //       o_defects_acknoledge:=mcpi_reset;
        //       o_transmitters_1_2_used:=OAddress[OUT_01_STOP_POINT].stato;
        //       o_transmitters_3_4_used:=OAddress[OUT_02_STOP_POINT].stato;
        //       o_transmitters_5_6_used:=OAddress[OUT_03_STOP_POINT].stato;
        //               (* Tolte in base al documento di Stelia inviato con email di Luca Proserpio di martedì 03/10/2017 16:50
        //                   o_transmitters_7_8_used:=OAddress[OUT_04_STOP_POINT].stato;
        //                   o_override:=LREAL_VALUE_TO_BYTE(i_sys_hwovr);
        //                   o_machine_state:=LREAL_VALUE_TO_WORD(Protube.MachineState.MachineState);
        //               *)
        //       IAddress[INP_01_STOP_POINT].enable_forzatura:=TRUE;
        //       IAddress[INP_01_STOP_POINT].stato_forzatura:=i_temperature_ok;
        //       IF i_running_mode<1 OR i_running_mode>2 THEN
        //         IF i_sys_error<>ERR_HEATING_MODE THEN
        //           iret:=Push_error(code:=ERR_HEATING_MODE, Arresto:=FALSE);
        //         END_IF;
        //       END_IF;
        //       IF i_general_defect=TRUE THEN
        //         IF i_sys_error<>ERR_HEATING_ERROR THEN
        //           iret:=Push_error(code:=ERR_HEATING_ERROR, Arresto:=FALSE);
        //         END_IF;
        //       END_IF;
        //       IF EnableWatchdog=TRUE THEN
        //         o_watchdog:=i_watchdog;
        //         IF Old_I_WatchDog=i_watchdog THEN
        //           //Valore non cambiato si verifica che il timer non sia arrivato
        //           //a zero, altrimenti errore
        //           IF TimerSistema[T_WATCHDOG].valore<1 THEN
        //             IF i_sys_error<>ERR_WATCHDOG THEN
        //               iret:=Push_error(code:=ERR_WATCHDOG, Arresto:=FALSE);
        //             END_IF;
        //           END_IF;
        //         ELSE
        //           //Input cambiato si fa ripartire il timer (1 sec)
        //           Old_I_WatchDog:=o_watchdog;
        //           TimerSistema[T_WATCHDOG].valore:=1000;
        //           TimerSistema[T_WATCHDOG].stato:=1;
        //         END_IF;
        //       ELSE
        //         IF i_watchdog=TRUE THEN
        //           //Il watchdog parte alla prima attivazione di i_watchdog
        //           EnableWatchdog:=TRUE;
        //         END_IF;
        //       END_IF;
        //               //Copia variabili su area di scambio
        //               Heating.input.working_status:=i_working_status;
        //               Heating.input.running_mode:=i_running_mode;
        //               Heating.input.temperature_measurement:=i_temperature_measurement;
        //               Heating.input.pyrometer_position_measurement:=i_pyrometer_position_measurement;
        //               Heating.input.general_defect:=i_general_defect;
        //               Heating.input.watchdog:=i_watchdog;
        //               Heating.input.temperature_ok:=i_temperature_ok;
        //               Heating.output.temperature_setpoint:=o_temperature_setpoint;
        //               Heating.output.tube_diameter:=o_tube_diameter;
        //               Heating.output.first_heat:=o_first_heat;
        //               Heating.output.last_heat:=o_last_heat;
        //               Heating.output.defects_acknoledge:=o_defects_acknoledge;
        //               Heating.output.watchdog:=o_watchdog;
        //               Heating.output.transmitters_1_2_used:=o_transmitters_1_2_used;
        //               Heating.output.transmitters_3_4_used:=o_transmitters_3_4_used;
        //               Heating.output.transmitters_5_6_used:=o_transmitters_5_6_used;
        //               Heating.output.transmitters_7_8_used:=OAddress[OUT_04_STOP_POINT].stato;
        //               Heating.output._override:=LREAL_VALUE_TO_BYTE(i_sys_hwovr);
        //               Heating.output.machine_state:=LREAL_VALUE_TO_WORD(Protube.MachineState.MachineState);
        // //     {
        // //     #endif  //Fine NO PROFINET
        // //     }
        // //   {
        // //   #endif  //Fine ELECTM
        // //   }
        // //   ;
        // END_IF;

      END_FUNCTION

  END_NAMESPACE