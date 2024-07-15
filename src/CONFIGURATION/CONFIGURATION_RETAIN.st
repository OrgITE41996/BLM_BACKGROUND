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

CONFIGURATION _Configuration_Retain
    VAR_GLOBAL RETAIN
        ///ENABLEAX
            //ABSOLUTELY DO NOT MODIFY THE RETAIN IN THIS MODULE
            OldPosAxe                       :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_MAXIS#MAXAXIS-1] OF LREAL;

        ///_VARRETAIN
            Disp                            :ARRAY[(FIRST_DEVICE)..(FIRST_DEVICE+_MAXDEVICES#MAXDEVICES-1)] OF Device;
            pm                              : ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF AxisParameters; 
            ControlledStop				    :BOOL:=TRUE;	//True se il Simotion è stato spento in modo controllato
            Presence4RFORM                  :SINT:=SINT#-1;
            IAddress                        : ARRAY [0..MAXDIGITALINPUT] OF Digital_Input;
            OAddress                        : ARRAY [0..MAXDIGITALOUTPUT] OF Digital_Output;
            cfg                             :_TYPE.Config_Struct;
            PulserTwoHands				    :BOOL:=TRUE;//TRUE enables the hand control with reset and start functions if provided simultaneously
            {
            ::ifdef PROFINET
            }
            Configuration_IO                :SINT:=SINT#0; //preset of IO configurations (0 = standard, 1 = optional io);
            PresencePnCoupler               :BOOL; //TRUE if the first PnCoupler must be present.
            (*{
            #ifndef PLC_S7_1500
            }*)
            PresencePnCoupler2			    :BOOL;			//TRUE se deve essere presente il secondo PnCoupler
            (*{
            #endif	//Fine PLC_S7_1500
            }*)
        	{
            ::endif  //FINE PROFINET 
            }
            HistoricalAlarms                :ARRAY[0..MAXALARMS_FIFO-1] OF Alarms;
            HistoricalIN                    :INT; // Puntatore al primo allarme nello storico
            HistoricalOUT                   :INT; //Puntatore all^ultimo allarme nello storico
            RifDtPrecommand                 :REAL:=0; //Reference diameter on which the reference precommand is calculated
            RefPrecommand                   :REAL:=100; //Precommand matched to reference diameter
            SlopePrecommand                 :REAL:=1; //Slope for calculating the precommand as a function of the diameter
            TubeIntestate                   :SINT; //TUBE_NO_INTESTATE=Tube not intestate
                                                    //TAGLIO_INTERNO=1Hose headed with IPC
                                                    //TAGLIO_ESTERNO+TAGLIO_4RTL=3 Tube headed with 4RTL
                                                    //TAGLIO_ESTERNO+TAGLIO_4RC=4 Tube headed with 4RC
        (*{
        #ifdef ORBITALE
        }*)
            ThreadIntext          	            :BOOL;
            WireSmussed					        :BOOL;
            ChamferPresent                      :BOOL; 
            PosTestWireInHeader 	            :LREAL;//Position of the wire head at the time of intesting
        (*{
        #else  //Fine ORBITALE
        }*)
            UsoPLCOpen						   :BOOL:=FALSE;
            Default_Second_Barrier             :BOOL:=FALSE;

           (*{
            #ifdef PROFINET
            }*)
              //I nuovi hardware funzionano meglio con il nuovo sistema di reset (tutti gli assi contemporaneamente)
              EnablingOptimizedAxisReset       :BOOL:=TRUE;
           (* {
            #else //Fine PROFINET
            }*)

            (*{
            #ifdef LINEARIZZA_ASSE
            }*)
              TabLinearizzaAsse		            :ARRAY [0.._MAX_AXIS_LINEARISES#MAX_AXIS_LINEARISES-1] OF Linearizza;
           (* {
            #endif  //Fine LINEARIZZA_ASSE
            }*)
            i_sys_optimize_Z_preset             :INT:=1;
            MinuteSavingEnergy                  :INT;
            i_sys_Alimen_Ok 			        :BOOL;
            ModeSpecialReturnCuttingOrbital 	:INT:=0;	//Se >0 abilita la gestione speciale che invece di eseguire il modulo di A1/A2 o B1/b2
                                                            //esegue un ritorno completo a zero per entrambi gli assi, indica la percentuale con cui
                                                             //si esegue il riposizionamento a zero dei due assi
            Manipolatore2D_Attivo				:BOOL:=FALSE;			//True se il manipolatore 2D è stato abilitato
            (*{
            #ifdef SMART
            }
              {
              #ifdef BASCULANTE_ELETTRICO
              }*)
                i_sys_coppiaB2                  :USINT:=USINT#15;  //Percentuale coppia spinta B2 sugli estremi
                BasculantePosCarm7_DX           :REAL;
                BasculantePosCarm7_SX           :REAL;
              (*{
              #else  //Fine BASCULANTE_ELETTRICO
              }*)
              TimeNastFull				        :INT:=1000;
              Rc                                :INT;//Riga verso cui ci si sta muovendo
              senzatubo				            :Flipflop;
            // {
            // #ifdef DH4010  
            // }
            //Rc                                  :ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;//Riga verso cui ci si sta muovendo
           
            //   {
            //   #ifdef DH4010
            //   }
              i_sys_CicloContinuoSenzaCoil	    :BOOL;	//Viene messo a True dal comando 328 e serve al personale
              i_sys_CicloContinuo               : LREAL :=0;
            //   {
            //   #endif  //Fine DH4010
            //   }
            // {
            // #ifdef CURVATURA
            // }
              CoppiaY1TuboRotto                 :USINT;  //Percentuale coppia per verifica rottura tubo
            // {
            // #endif  //Fine CURVATURA
            // }
            i_sys_OVR_act                       : LREAL := 100; // OVERRIDE SOFTWARE TEMPORANEO - NON MODIFICARE -  VALORE 0 - 100
            i_sys_current_msg                   :DINT; 			// Messaggio attivo
            i_sys_devicecol_from_simotion       : INT :=UNDEF_DEVICE; // richiesta attivazione DEVICE da SIMOTION (MCP)
                                                                // E^ il numero di dispositivo/asse secondo l^ordine della colonna di TABELLA STATI
            AsseCheTieneTubo                    :INT;
            FlagStopPointLavoro					:BOOL;  //True se la macchina in automatico sta eseguendo il pezzo ma il Master è fermo
            i_sys_AttesaOperatore		        :BOOL;	//True se la macchina è in attesa che l^operatore dia start per proseguire
                                                        //il rientro fuori ingombri dopo aver tolto il tubo dalla macchina
            StatusPrimoStart,
            StatusManualeCarico,
            StatusRobotCarico                   : BOOL;
            AssiSpenti                          :ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF BOOL;  //True se asse è stato spento
            i_sys_MAX_X_SOSTEGNO_ALTO		    :REAL:=1600;  //Quota massima X con sostegno alto, dopo si deve abbassare
        END_VAR

/*************************
 * _VARRETAIN BACKGROUND *
 *************************/
VAR_GLOBAL RETAIN
    managementUsers					:BOOL:=FALSE;
    i_sys_password_SUPERUSER        :STRING:='SHARON';
	i_sys_password_UT				:STRING:='MARYLIN';
	i_sys_password_MIS			    :STRING:='ANGELINA';
	i_sys_password_SAT			    :STRING:='JENNIFER';
	i_sys_password_SERVICE	        :STRING:='BEATRICE';
	i_sys_password_PROG			    :STRING:='GIULIA';
	i_sys_password_USER			    :STRING:='SCARLET';
     (*{
            #ifdef PROFINET
            }*)
    UsagePnCouplerInProfisafe       :BOOL;			//TRUE se il PnCoupler deve far comunicare i dati di sicurezza in profisafe
	i_sys_buttonEventCounter        :DWORD;	//da bit 00 a 03 contatore attivazione Start ciclo 
																	//da bit 04 a 07 contatore attivazione Stop ciclo
																	//da bit 08 a 11 contatore attivazione Reset
																	//da bit 12 a 15 contatore attivazione Pulsante +
																	//da bit 16 a 19 contatore attivazione Pulsante -
    i_sys_MachineMaster             :BOOL;
      //Rappresenta il livello MINIMO di accesso per poter effettuare la funzione richiesta
  access                            :Access_Level;
  i_sys_CicloSpedizioneEseguito     :BOOL:=FALSE;     //True se è stato eseguito il ciclo di spedizione
  StartDato                         :BOOL;
  i_sys_BarrieraViolata             :BOOL;	//TRUE quando la barriera è KO e la macchina non può funzionare, 
                                        //FALSE quando tutti gli assi sono stati riabilitati
    Curv_Man                        :CURVATURA_MANUALE_STRUCT; //Dati per il ciclo di curvatura manuale 
    O_Reset,
    O_StatoEmergenza,
    O_StatoBarriera                 : BOOL;
    // {
    // #ifdef PROFINET
    // }
      O_Comfort3                    : BOOL:=FALSE;  //Serve per evitare di visualizzare l^errore 100076
    // {
    // #endif  //Fine PROFINET
    // }

    // {
    // #ifdef THREE_RUNNER
    // }
      F_Feedback_Carico_Tubo 	    :F_TRIG;
      R_Feedback_Carico_Tubo 	    :R_TRIG;
      F_Pedale_Carico_Tubo   	    :F_TRIG;
      R_Pedale_Carico_Tubo   	    :R_TRIG;
      BackgroundCycles       	    :INT:=0;
      ShouldBeInOperation		    :BOOL:=FALSE;
      ShouldNotBeInOperation	    :BOOL:=FALSE;
      IsInOperation				    :BOOL:=FALSE;
    // {
    // #endif  //Fine THREE_RUNNER
    // }
    i_sys_CMD_Reset                 :BOOL;  //True quando viene dato il comando di reset da comando di BlmMotion
    
    OPS_StartDato                   :BOOL;  //True se l^operatore ha dato lo START per iniziare il pezzo seguente OPS
    Appo_i_sys_Error		        :DINT;	//Memorizzazione di particolari codici di errore che debbono essere
                                        //comunque visualizzati in BlmMotion in quanto hanno generato il
                                        //problema e sarebbero poi stati sovrascritti da altri codici di errore
    i_sys_InUsoScritturaSinamics    :BOOL;
    i_sys_InUsoLetturaSinamics      :BOOL;
    Calibr                          :Calibraz_Struct;
    i_sys_MacchinaMaster		    :BOOL;
    EnableOverrideRemoto            :BOOL;  //True se è possibile usare l^override impostato da remoto. Diventa TRUE
                                    //la prima volta dopo il run della CPU che ha valore maggiore di 1. Serve
                                    //per capire che la comunicazione è stata attivata ed il valore (anche 0)
                                    //è consistente
    InibisciOverrideTastiera		:BOOL:=FALSE;
    MinimumKeyboardOverride			:INT:=99;
    OverrideParametrico				:INT:=100;
    i_sys_override_uomomorto        :USINT:=USINT#20;	//Override per pressione uomo morto
    
    i_sys_Asse_Master_per_riduzT    :INT;   			//Numero dell^asse che comanda la riduzione di velocità in caso di bassa temperatura (-1 disabilita il ciclo)
    i_sys_T_max_per_riduzT          :LREAL;				//Temperatura massima per velocità ridotta
    i_sys_T_min_per_riduzT          :LREAL;				//Temperatura minima per velocità ridotta
    i_sys_vel_min_per_riduzT        :LREAL;				//% di velocità a bassa temperatura (in alta temperatura = 100)
    i_sys_override_HoldToRun_Auto	:USINT:=USINT#5;
    AL5_AperturaPinze_JOG			:BOOL;	//TRUE => Apertura pinze AL5 all^attivazione del JOG
    i_sys_ModoPezzoSeguente         : INT;
    AL_LunghezzaTubo                :LREAL:=-MY_BIG_VALUE_REAL;
    ProgrammiX                      :ARRAY[0.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS-UINT#1] OF ProgrammaX; // dati di produzione in varretain
    
END_VAR   
END_CONFIGURATION