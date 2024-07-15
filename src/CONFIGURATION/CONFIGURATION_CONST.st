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

CONFIGURATION _Configuration_Const
 
    /******************
 * ERROR_MESSAGES *
 ******************/
 VAR_GLOBAL CONSTANT
    ERR_NOERROR							:SINT:= SINT#0;
    ERR_L1_TRASLATORE                   :DINT := 15;
    ERR_INDICATION_DATE_SET             :DINT := 26;
    ERR_CMD_NON_DISPONIBILE              	:DINT := 32;
    ERR_ENABLE_HOLD_TO_RUN              :DINT := 41;
    ERR_STOP_CYCLE_BUTTONS              :DINT := 48;
    ERR_CHECKSUM_SAFETY                 :DINT:= 50;
    ERR_EXECUTE_CMD_233                 :DINT := 59;
    ERR_INCONGRUENZA_NUMENCODER         :DINT := DINT#100015; //Errore grave per incongruenza fra encoder software e encoder hardware
    ERR_FIRST_BLM                       :DINT:=DINT#100000; // Primo codice di errore BLM
    ERR_OPMODE_NOT_ALLOWED              :DINT := DINT#100001; 	//Richiesta modo operativo non ammessa per livello di accesso troppo basso
    ERR_FIRST_ERROR_SIEMENS			    :DINT:=DINT#20001;
    ERR_CONFIG_MECHANICS_SAFETY          :DINT := 37;
    ERR_SAFETY                          :DINT := DINT#100061;
    ERR_NUMSTATES                       :DINT := DINT#100003; //Numero stati <=0
    ERR_GRAVE_DOWNLOAD                  :DINT := DINT#100037; //Errore grave nel download del programma
                                                      //Superato limite trasferimenti per errore_a_tempo
    ERR_VERSION_SAFETY                 :DINT := 28;
    ERR_IO_NOT_DEFINED                 :DINT := DINT#100032; //Manca associazione virtuale/fisico
    ERR_CONTINUOUS_CYCLE_NOT_POSSIBLE   :DINT := DINT#100083;
    ERR_SLITTA_MOBILE_NON_A_ZERO        :DINT := DINT#100018; 	//Movimento del braccio non consentito per slitta mobile non a 0
    ERR_BRACCIO_SOTTO_90                :DINT :=DINT#100019; 	//Movimento della slitta non consentito per posizone del braccio <90
    ERR_APRIRE_SLITTA_SPALLAMENTO       :DINT :=DINT#100030;
    ERR_TASSELLO_BLOCCAGGIO_DXSX        :DINT := DINT#100029; 	//Pos. tassello bloccaggio DX/SX in posizione errata 	//Slitta di spallamento non aperta

END_VAR

/**************
* ERROR_AXES *
**************/

VAR_GLOBAL CONSTANT
ERR_AXES_NO_INDEX         		:DINT 	:= DINT#100400;	//Errore asse non indexato 
ERR_AXES_CREAZIONE_CAMMA  		:DINT 	:= DINT#100500;	//Errore creazione camma
ERR_VETT_Y1_U4            		:DINT 	:= DINT#100600;	//Errore sull^oggetto vettoriale Y1_U4
ERR_VECT_CARRO_ANIMA      		:DINT 	:= DINT#100601;	//Errore sull^oggetto vettoriale Carro-Anima
ERR_VECT_MORSA_RULLI_Q    		:DINT 	:= DINT#100602;	//Errore sull^oggetto vettoriale Morsa-Rulli-Q
ERR_VECT_TESTA_Q_T        		:DINT 	:= DINT#100603;	//Errore sull^oggetto vettoriale Testa-Q-V
ERR_VECT_Y1_U4_Y2         		:DINT 	:= DINT#100604;	//Errore sull^oggetto vettoriale Braccio-SlittaMobile-Matrice
ERR_VECT_Q_V1_V4          		:DINT 	:= DINT#100605;	//Errore sull^oggetto vettoriale Q-V1_V4
ERR_VECT_A1_A2            		:DINT 	:= DINT#100606;	//Errore sull^oggetto vettoriale A1_A2
ERR_VETT_Y1_X1            		:DINT 	:= DINT#100607;	//Errore sull^oggetto vettoriale Y1_X1
ERR_VECT_Y1_Y2            		:DINT 	:= DINT#100608;	//Errore sull^oggetto vettoriale Braccio-Matrice
ERR_VETT_X3_X6           		:DINT 	:= DINT#100609;	//Errore sull^oggetto vettoriale X3_X6
ERR_VETT_X2_X6            		:DINT 	:= DINT#100610;	//Errore sull^oggetto vettoriale X2_X6
ERR_VETT_X1_Y1            		:DINT 	:= DINT#100611;	//Errore sull^oggetto vettoriale X1_Y1
ERR_VETT_X2_Y2            		:DINT 	:= DINT#100612;	//Errore sull^oggetto vettoriale X2_Y2
END_VAR
/***************************************
* TRACE DECLARATION (YELLOW MESSAGES) *
***************************************/

VAR_GLOBAL CONSTANT
    TRC_MISSING_ADDRESSES_IO            :INT:=28;  //Mancano indirizzi hardware degli I/O
    TRC_DRIVE                           :INT:=37;  //Azionamento non attivato + numero dellasse
    TRC_POWER                           :INT:=38;  //Potenza non attivata + numero dellasse
    TRC_COPY_RAM_ROM                    :INT:=47; // Waiting for copy from RAM to ROM 
    TRC_AL5_SOLLEVATORE_FASCIO_ALTO     :INT:=61;
    TRC_AL5_SOLLEVATORE_FASCIO_BASSO    :INT:=62;
    TRC_JOG_NON_IN_MONITORING           :INT:=48;  //Macchina non in monitoring o livello di accesso troppo basso
    TRC_CONFIGURATION_CHANGE            :INT:=77; // Configuration change performed, turn off and on the CN 
    TRC_ATTESA_AZIONAMENTO              :INT:=71;  //Attesa comunicazione con azionamento
    TRC_DEACTIVATION_SAFETY             :INT:=72;
    TRC_START_CMD                       :INT:=151; 
    TRC_CMD_VGP_AXIS_DEACTIVATION       :INT:=152; 
    TRC_CMD_SETTING_V_HZ                :INT:=153; 
    TRC_RELEASE_EMERGENCY               :INT:=165; 
    TRC_EXECUTE_MOTOR_SETTING           :INT:=166; // Traces from 200 to 299 are to be used for device waiting, BlmMotion displays 
    // the device number by adding it to trace 39 (TRC_DISPOSITIVI_BLMMOTION)
    // the next available trace should be 300;
    
    TRC_SAFETY_JOG_DEVICES              :INT:=190;
    TRC_ASSE_HOLD                       :INT:=195;
    TRC_DISPOSITIVI                     :INT:=200;  //Attesa fine posizionamento + Numero dispositivo
                                                    //Le trace da 200 a 299 sono da utilizzare per attesa dispositivi, BlmMotion visualizza
                                                    //il numero del dispositivo sommandolo alla trace 39 (TRC_DISPOSITIVI_BLMMOTION)
                                                    //la prossima trace utilizzabile deve avere il 300;
    TRC_MOTOR_SLIDE_RIGHT               :INT:=300; 
    TRC_MOTOR_SLIDE                     :INT:=306; 
    TRC_MOTOR_SLIDE_LEFT                :INT:=301; 
    TRC_MOTOR_AL5                       :INT:=302; 
    TRC_MOTOR_DXSX_4RUNNER              :INT:=303; 
    TRC_MOTOR_4RTL                      :INT:=304;
    TRC_MOTOR_4RC                       :INT:=305;
    TRC_ACTIVATION_4RFORM               :INT:=178; 
    TRC_WAIT_SYSTEM_STARTUP             :INT:=192; 
    TRC_CMD_DXSX                        :INT:=307; // + Enable DXSX (W4 On), - Disable DXSX (W4 Off) 
    TRC_FLOW_DRILL                      :INT:=315; // + Enable FLOW_DRILL (V2 On), - Disable FLOW_DRILL (V2 Off) 
    TRC_MOTOR_DRILL                     :INT:=316;
    TRC_CMD_4RTL                        :INT:=308; // + Enable 4RTL (A1 On, A2 Off), - Disable DXSX (A1 Off, A2 Off) 
    TRC_CMD_4RC                         :INT:=309; // + Enable 4RTC (A1 On, A2 On), - Disable DXSX (A1 Off, A2 Off) 
    TRC_CMD_4RIPC                       :INT:=341;
    TRC_CMD_3RIPC                       :INT:=342; 
    TRC_CMD_U4                          :INT:=310; // + Enable U4, - Disable U4 
    TRC_CMD_U5                          :INT:=311; // + Enable U5, - Disable U5 
    TRC_CMD_L1                          :INT:=312; // + Enable L1, - Disable L1 
    TRC_SET_MACHINE_NAME                :INT:=317; 
    TRC_SET_4RFORM                      :INT:=318; 
    TRC_OPERATION_OK                    :INT:=319; 
    TRC_SETTING_Q5004_DX                :INT:=323; 
    TRC_SETTING_Q5004_SX                :INT:=324; 
    TRC_FUNGO_LOADER                    :INT:=331; 
    TRC_FUNGO_ROBOT                     :INT:=332; 
    TRC_FUNGO_CONTROL_PANEL             :INT:=333;
    TRC_OVERRIDE_MIN_TEMP               :INT:=338; 
    TRC_SET_TILTING_SIDE                :INT:=347; 
    TRC_TILTING_SIDE_NOT_SET            :INT:=346; 
    TRC_FUNGO_CONSOLE                   :INT:=334; 
    TRC_WAIT_FOR_DRIVE_COMMUNICATION    :INT:=71; // Waiting for communication with the drive
    TRC_WAIT_FOR_POWER_SUPPLY_ACTIVATION :INT:=9; // Waiting for power supply activation
    TRC_WAIT_FOR_SYSTEM_STARTUP_KEYBOARD :INT:=374; 
    TRC_WAIT_FOR_SYSTEM_STARTUP_PLC_SAFETY :INT:=373; 
    TRC_WAIT_CU32_1                     :INT:=375; 
    TRC_WAIT_CU32_2                     :INT:=376; 
    TRC_WAIT_AFTER_UNCONTROLLED_STOP    :INT:=377;
    TRC_READ_DRIVE_PARAMETERS           :INT:=35;  //Waiting for drive parameters reading
    TRC_NOTRACE                         :INT:=0;   //Cancellazione trace
            //SF [21/04/22]: trace che indica un cambio di configurazione da inviare al PLC di sicurezza di tipo PLC_S7_1500
    TRC_CHANGE_CONFIGURATION_IO			:INT:=410;
    TRC_WAITING_ACTIVATION_DATASET      :INT:=85;  //Attesa attivazione del dataset desiderato
    TRC_INCONGRUENCE_MANIPULATOR2D	    :INT:=357;
    TRC_MACHINE_DXSX                    :INT:=10;  //Configurazione errata macchina DX SX
    TRC_BASCULANTE_DXSX                 :INT:=344;
    TRC_BASCULANTE_SXDX                 :INT:=345;
    TRC_BASCULANTE_NON_IMPOSTATO        :INT:=346;
    TRC_VIOLATION_BARRIER               :INT:=102;  //Attenzione help trace in comune con DH
    TRC_LIVELLO_DI_ACCESSO_BASSO        :INT:=53; //Livello di accesso troppo basso per qualsiasi operazione in macchina
    TRC_MONITORING_DISABILITATO         :INT:=57; // ATTENZIONE: Controlli disabilitati
    TRC_SICUREZZA_JOG_DISPOSITIVI       :INT:=190;
    TRC_MACCHINA_PEZZO_DXSX             :INT:=12;  //Incompatibilità DX/SX fra pezzo/utensili e stato macchina 
END_VAR

/****************
* MACHINE AXES *
****************/

VAR_GLOBAL CONSTANT
        // MACHINE AXES
        X1 :    INT :=_FIRST_AXIS#FIRST_AXIS;    //Cart X
        Y1 :    INT :=_FIRST_AXIS#FIRST_AXIS+1;  //Y-axis bending arm
        Z1 :    INT :=_FIRST_AXIS#FIRST_AXIS+2;  //Rotation Z
        Q1 :    INT :=_FIRST_AXIS#FIRST_AXIS+3;  //Horizontal head
        V1 :    INT :=_FIRST_AXIS#FIRST_AXIS+4;  //Vertical head
        W3 :    INT :=_FIRST_AXIS#FIRST_AXIS+5;  //Vise
        W4 :    INT :=_FIRST_AXIS#FIRST_AXIS+6;  //Rollers
        U4 :    INT :=_FIRST_AXIS#FIRST_AXIS+7;  //Mobile slide
        U2 :    INT :=_FIRST_AXIS#FIRST_AXIS+8;  //Core
        L1 :    INT :=_FIRST_AXIS#FIRST_AXIS+9;  //AL5

        //X1    :INT :=PRIMO_ASSE;    //Carro X
        //Y1    :INT :=PRIMO_ASSE+1;  //Braccio di curvatura Y
        //Z1    :INT :=PRIMO_ASSE+2;  //Rotazione Z
        //Q1    :INT :=PRIMO_ASSE+3;  //Orizzontale testa
        //V1    :INT :=PRIMO_ASSE+4;  //Sollevamento testa
        //W3    :INT :=PRIMO_ASSE+5;  //Bloccaggio
        W1      :INT :=_FIRST_AXIS#FIRST_AXIS+6;  //Morsa 4RFORM
        W2      :INT :=_FIRST_AXIS#FIRST_AXIS+7;  //Spintore 4RFORM
        U1      :INT :=_FIRST_AXIS#FIRST_AXIS+8;  //Scomparsa 4RFORM
        //U2    :INT :=PRIMO_ASSE+9;  //Cambio punzoni 4RFORM
        A1      :INT :=_FIRST_AXIS#FIRST_AXIS+10;  //Rotation 4RTL/"Rotazione 4RC
        A2      :INT :=_FIRST_AXIS#FIRST_AXIS+11;  //Rotation 4RC
        B1      :INT :=_FIRST_AXIS#FIRST_AXIS+12;  //Rotation 4RIPC
        B2      :INT :=_FIRST_AXIS#FIRST_AXIS+13;  //Rotation 4RIPC

    (*{
    #ifdef TAVOLI_ELETTRICI
    }*)
        T1	    :INT:=_FIRST_AXIS#FIRST_AXIS+11;	//Axis table 1
        T2	    :INT:=_FIRST_AXIS#FIRST_AXIS+12; 	//Axis table 2
    (*{
    #endif //TAVOLI_ELETTRICI
    }*)

        X2      :INT:=_FIRST_AXIS#FIRST_AXIS+1;  

        V2      :INT :=_FIRST_AXIS#FIRST_AXIS+14;  //Flow-Drill
        U5      :INT :=_FIRST_AXIS#FIRST_AXIS+9;   //Extra mobile sled
        X6      :INT :=_FIRST_AXIS#FIRST_AXIS+8;

        (*{
        #ifdef ASSE_Z2
        }*)
        Z2  :INT:=_FIRST_AXIS#FIRST_AXIS+11;//Welding research on AL5
        (*{
        #endif  //Fine ASSE_Z2
        }*)

        (*{
        #ifdef MATRICE_ELETTRICA
        }*)
         Y2 : INT :=_FIRST_AXIS#FIRST_AXIS+10;	//Matrice
        (*{
        #endif	//Fine MATRICE_ELETTRICA
        }*)
        V4 : INT :=_FIRST_AXIS#FIRST_AXIS+11; //Sollevamento slitta
        X3 :INT:=_FIRST_AXIS#FIRST_AXIS+7;
END_VAR


/*******************
* PI COMMAND CODE *
*******************/
VAR_GLOBAL CONSTANT
    PIPE_NO_INTESTATE                     :SINT:=SINT#0;
END_VAR

/**************************************************
* MAXIMUM NUMBER OF STATES OF THE 4-COLOR BEACON *
**************************************************/
VAR_GLOBAL CONSTANT
    AL42PLUS_STATO_CARICO              	:SINT:=SINT#0;   //Carico tubo, traslatore avanti
    PARAMETER_COPY_RAM_ROM				:UDINT:=UDINT#977;
    NO_HARDWARE                        	:SINT:=SINT#-1;
    MAX_AXIS_LINEARISES                	:INT:=3;
    UNDEF_DEVICE                       	:INT:=-1;
    AT_STATE_UNDEFINED                	:SINT:=SINT#33;
    OVERHEAD_LEFT_SIDE      		    :SINT:=SINT#1;
    TIPO_AL42                          	:SINT:=SINT#2;
    TIPO_AL42PLUS                      	:SINT:=SINT#4;
    AL42_STATO_CARICO                  	:SINT:=SINT#0;
    TIPO_AL5                          	:SINT:=SINT#5;
    TIPO_AL5_ROLAND                    	:SINT:=SINT#8;		//AL5 con sistema di misura ROLAND
    TIPO_AL5_LUNGO                    	:SINT:=SINT#10;	//AL5 speciale lungo 6mt
    AL5_CARICO_MACCHINA                	:SINT:=SINT#0;	//AL con tubo pronto in posizione di 
                                                        //carico tubo in macchina con pinze 
                                                        //chiuse, se presente traslatore avanti 
    TIPO_AL5_BUNDLE                    	:SINT:=SINT#6;
    TIPO_AL5_ROLAND_BUNDLE            	:SINT:=SINT#9;		//AL5+BUNDLE con sistema di misura ROLAND
    TIPO_AL5_LUNGO_BUNDLE              	:SINT:=SINT#11;	//AL5 speciale lungo 6mt+BUNDLE con sistema di misura ROLAND
    TIPO_MS0017                        	:SINT:=SINT#12;	//Caricatore per ditta Sogefi
    TIPO_AL5_FUTABA                    	:SINT:=SINT#13;	//Caricatore per ditta Futaba con ricerca saldatura Roland
END_VAR



VAR_GLOBAL CONSTANT
    TILTING_POS_CURVATURE                 :REAL:=0;
    TILTING_DIST_POS_PUSH_JOG             :REAL:=1;
    //{::ifdef PLC_S7_1500}
      ADDRESS_SINAMICS        :INT:=672;
      ADDRESS_CX1             :INT:=288;
      TEMPO_ARRESTO_BARRIERA  :REAL:=REAL#0.2;	//Tempo di arresto assi durante violazione barriera di sicurezza
                                                                              //Se è 0=>Si ferma con il tempo dell^asse più lento
    //{::else}
    
    //{::endif PLC_S7_1500}
    AUTOSETTING_SETTING_SIDE_BASCULANT    :SINT:=SINT#16;

    JOGMODE                               :INT:=0;
    REFMODE                               :INT:=1;
    MDAMODE                               :INT:=2;
    AUTOMODE                              :INT:=3;
    TEACH_IN                              :INT:=4;

    BRAKE_ENABLED                    	  :SINT:=SINT#1;
    BRAKE_DISABLED                  	  :SINT:=SINT#2;
    MY_BIG_VALUE_REAL                     :REAL  :=REAL#99999.0;
    //_DEFMACCHINA
    ADDRESS_CU2                           :INT:=NO_HARDWARE;
    ADDRESS_POWER_SUPPLY                  :INT:=NO_HARDWARE;
    FIRST_AXIS_MASTER				      :INT:=0;
    MAXTRACE                              :INT:=20;
    FIRST_DEVICE                          :INT:=32;
    COL_STOP_POINT                        :INT:=23;
    COL_NUM_SEQ                           :INT:=27;
    LOCK_2D                               :INT:=FIRST_DEVICE+8;  //40
    MY_BIG_VALUE_INT                      :INT:=32767;
        (*{
        #ifdef PLC_S7_1500
        }*)
            //Macchina con nuovo PLC
        VERSIONE_SAFETY                   :BYTE:=BYTE#1;
        K_CHECKSUM_PLC_SAFETY             :UDINT:=UDINT#16#A8B373C2;
        (*{
        #else
        }
            {
            #ifdef SELEZIONE_BARRIERA_1_2
            }
                //Macchina speciale con la commutazione di due diverse barriere
                VERSIONE_SAFETY            :BYTE:=;
                K_CHECKSUM_PLC_SAFETY      :UDINT:=16#;
            {
            #else //Fine SELEZIONE_BARRIERA_1_2
            }
                //Macchina senza selezione della barriera da utilizzare
                {
                #ifdef RIARMO_NON_EQUIVALENTE
                }
                    //Macchina con riarmo in ingresso doppio canale non equivalente
                    {
                    #ifdef ABILITAZIONE_PULSANTIERA_2MANI
                    }
                        //Macchina con riarmo in ingresso doppio canale non equivalente la pulsantiera a due mani può essere abilitata o meno
                        //in funzione dello stato della variabile ritentiva "PulsantieraDueMani"
                        VERSIONE_SAFETY          :BYTE:=23;
                        K_CHECKSUM_PLC_SAFETY    :UDINT:=16#2B6CA313;
                    {
                    #else  //FINE ABILITAZIONE_PULSANTIERA_2MANI
                    }
                        //Macchina con riarmo in ingresso doppio canale non equivalente la pulsantiera a due mani deve essere sempre presente
                        {
                        #ifdef ASSE_Z2
                        }
                            //Macchina con asse Z2 e riarmo con ingresso in doppio canale non equivalente
                            VERSIONE_SAFETY          :BYTE:=18;
                            K_CHECKSUM_PLC_SAFETY    :UDINT:=16#59131FE7;
                        {
                        #else  //Fine ASSE_Z2
                        }
                            //Macchina con riarmo con ingresso in doppio canale non equivalente
                            VERSIONE_SAFETY          :BYTE:=14;
                            K_CHECKSUM_PLC_SAFETY    :UDINT:=16#7D0BE64D;
                        {
                        #endif  //Fine NO ASSE_Z2
                        }
                    {
                    #endif  //FINE NO ABILITAZIONE_PULSANTIERA_2MANI
                    }
                {
                #else  //Fine RIARMO_NON_EQUIVALENTE
                }
                    //Macchina con riarmo in ingresso doppio canale equivalente
                    VERSIONE_SAFETY          :BYTE:=17;
                    K_CHECKSUM_PLC_SAFETY    :UDINT:=16#8711A629;
                {
                #endif  //Fine NO RIARMO_NON_EQUIVALENTE
                }
            {
            #endif //Fine NO SELEZIONE_BARRIERA_1_2
            }
        {
        #endif // FINE NO PLC_S7_1500
        }*)

        (*{
        #ifdef PLC_S7_1500
        }*)
        ADDRESS_CU1                       :INT:=288;
        (*{
        #endif //FINE NO PLC_S7_1500
        }*)
    ///_CONST
    MODO_MDA_1        	:USINT:=USINT#1;  //1=Gestione standard
    MODO_MDA_2        	:USINT:=USINT#2;  //2=Gestione come tabella di posizioni
    MODO_MDA_3        	:USINT:=USINT#3;  //3=Gestione come tabella di posizioni e attesa start
    EXTRALIMIT         	:REAL :=REAL#0.1; // extralimite per limiti fisici
    CHANNELRESET        :INT:=0;									//PROGRAMMI TUTTI RESETTATI
    CHANNELACTIVE :INT:=1;								//PROGRAMMA ATTIVO
    CHANNELINTERRUPTED:INT:=2;						//PROGRAMMA FERMO IN STOP CICLO
      //Rappresenta la numerazione degli assi/dispositivi per VGP
      CARICO_ASPO                        	:SINT:=SINT#5;  //Carico della DH4010 con aspo
   
/***************************************
* DRIVE_OBJECT_PARAM_ACTIVE/DISACTIVE *
***************************************/
    SETTING_OBJECT_OPERATION		    :UDINT:=UDINT#105;//DriveObject Activated/disactivated, check if it is active
    OPERATION_OBJECT_STATE              :UDINT:=UDINT#106;//Check if it is disactive
    ELECTRIC_SLED_TYPE              	:SINT:=SINT#0;		//Slitta mobile di tipo elettrico
    
   // MY_BIG_VALUE_REAL                   :REAL:=REAL#99999.0;

    STATE_CNC_SHUTDOWN				    :INT:=16#101;	//257//Gruppo 16#100 Macchina spenta;

    NON_DEACTIVATABLE_AXIS              :SINT:=SINT#0;
    ACTIVE_DEACTIVATABLE_AXIS           :SINT:=SINT#1;
    
    _DEACTIVATABLE_AXIS         :SINT:=SINT#2;

    AXIS_ACTIVE                         :REAL:=1;
    OUT_RESET_POWER_SUPPLY				:INT:=174;
    MAXDIGITALOUTPUT                    :INT:=400;
    MAXDIGITALINPUT                     :INT:=400;
    
   

   (*  {
    #else  //Fine DH4010  
    }*)
      NUMERO_TABELLE                    :USINT:=USINT#1;
   (* {
    #endif  //Fine NO DH4010  
    }*)

    PROGRAM_JOG                      	:SINT:=SINT#1;
    MAINTENANCE_ACCESS   	            :SINT:=SINT#4;
    INP_VOFA_1							:INT:=116;
    INP_VOFA_2							:INT:=117;

    //Linear machines
    OUT_BOOSTER_JOB                     :INT:=00;
    MAXALARMS_LIFO                      :INT:=16;
    MAXALARMS_FIFO                      :INT:=512;
                                                                        // LIVELLO DI ACCESSO PER     INDEF JOG  REF  MDA  AUTO TEACH
    VAR_INC_INIT                        :INT:=6;  //Stato iniziale per gestione VAR_INC
    FIRST_COMPLEX_OPERATION             :INT:=100; 
    CMD_SERVICING                       :INT:=FIRST_COMPLEX_OPERATION+15;  //115
    CMD_MANUAL_CURVE                    :INT:=FIRST_COMPLEX_OPERATION+16;  //116
    CMD_END_SERVICE                     :INT:=FIRST_COMPLEX_OPERATION+17;  //117

    MAX_TIMER_SYSTEM 	                :USINT:=USINT#47; 
    T_POST_MORTEM						:INT:=21;
    MaxParameterNumber                  : UINT := UINT#18;
    C_RESTORE_MACHINE				    :INT:=233;
    C_SHIPPING_MACHINE					:INT:=FIRST_COMMAND_BLMMOTION+23;	//223;
    FIRST_COMMAND_BLMMOTION				:INT:=200;
    MAX_KEYS_AXIS                       :USINT:=USINT#15;
      //Rappresenta la mappatura degli assi sulla pulsantiera macchina
MANUAL_BENDING                          :INT:=FIRST_COMPLEX_OPERATION+0;   //100
C_CUT_KNIFE                             :INT:=260;
C_CALIBRATION                           :INT:=FIRST_COMPLEX_OPERATION+12;  //112
CMD_OUTSIDE_CUTTING                     :INT:=FIRST_COMPLEX_OPERATION+23;  //123

AXES_MAX_TEMPERATURE                    :DWORD:=DWORD#4294967295;  //Assi su cui eseguire il controllo di massima temperatura
POSITION_EXTRA_MASTER                   :REAL:=REAL#0.02;
COL_RESERVED1                           :INT:=18;
ACCELERATION_MASTER                     :REAL:=3;
MAX_MASTER_ACCELERATION                 :INT:=10000;
PRIMO_STOP_POINT_SET_OUT              	:INT:=228;
MAX_INP_OUT_STOP_POINT                  :INT:=24;
PRIMO_STOP_POINT_RESET_OUT            	:INT:=ULTIMO_STOP_POINT_SET_OUT+1; //252
ULTIMO_STOP_POINT_SET_OUT             	:INT:=PRIMO_STOP_POINT_SET_OUT+MAX_INP_OUT_STOP_POINT-1;//251
ULTIMO_STOP_POINT_RESET_OUT           	:INT:=PRIMO_STOP_POINT_RESET_OUT+MAX_INP_OUT_STOP_POINT-1;//275
MODO_PEZZO_STANDARD                     :SINT:=SINT#0;
//         {
// #endif  //Fine ELECTM
// }
// {
// #ifdef ORBITALE
// }
BIT_NO_OVERRIDE						:INT:=10;  //Evita riduzione override per taglio sia 3-RUNNER, 4-RUNNER ed EFLEX
SN_SP           :SINT:=   SINT#0;
SN              :SINT:=SINT#1;
SP              :SINT:=SINT#-1;


C_DISATTIVAZIONE_BOOSTER_X1			:INT:=241;
C_IMPOSTAZIONI_ASSI_DISATTIVABILI	    :INT:=242;
C_TENSIONE_FREQUENZA              	:INT:=243;
C_SOTTOTESTA                      	:INT:=244;
C_ATTIVAZIONE_V1                  	:INT:=245;
C_DISATTIVAZIONE_V1               	:INT:=246;
C_IMPOSTA_MANIPOLATORE2D			    :INT:=320;
C_MACCHINA_DX                     	:INT:=247;
C_MACCHINA_SX                    	:INT:=248;
C_RESET_SIMOTION                  	:INT:=261;
C_ON_4RFORM                       	:INT:=272;
C_ASSE_ANIMA						:INT:=332;	//Comandando attivazione/disattivazione asse anima
ULTIMA_OPERAZIONE_COMPLESSA       	:INT:=C_ASSE_ANIMA;
CMD_TAGLIO_EFLEX                    :INT:=FIRST_COMPLEX_OPERATION+18;  //118
POSIZIONE_MASTER_RFI                :REAL:=-1;  //Posizione del master durante l^operazione di rientro fuori ingombri/primo start
                                                    //Il seguente array serve per associare le uscite agli stop point di comando

OUT_MACCHINA_IN_CICLO_AUTOMATICO                :INT:=23;
PROGSTATUS_RESET 							    :INT:=0;
JOB_STATE_PRODUZIONE                            :SINT:=SINT#5;
JOB_STATE_SOSPESO                               :SINT:=SINT#7;
MODO_PEZZO_SEGUENTE                             :SINT:=SINT#1;
SALVA_DATI_SIMOTION                   	        :SINT:=SINT#4;
TIPO_CARM                          	            :SINT:=SINT#1;
CARICO_AL_BLM_ROBOT                	            :SINT:=SINT#3;  //Carico Robot che preleva il tubo dall^AL
MODO_PEZZO_SEGUENTE_SINGOLO                     :SINT:=SINT#3;

PROGRAMMA_JOG                      	            :SINT:=SINT#1;
MAXTOOLSOFFPEZZO                                :INT:=32;
POSY_SCOMPBLOCC_DX                	            :REAL:=35;
POSY_SCOMPBLOCC_SX                	            :REAL:=145;
INACTIVE_DEACTIVATABLE_AXIS                    :SINT:=SINT#2;
DX                                              :INT:=1;
SX                                              :INT:=-1;


END_VAR

/***************
* PLC_TIA_NEW *
***************/

VAR_GLOBAL CONSTANT
driveObjectIdConstant                   : UINT := UINT#2;
hardwareIdConstant                      :WORD := WORD#264;
END_VAR

/*******************
* LINEAR MACHINES *
*******************/

VAR_GLOBAL CONSTANT
//IN
INP_OUT_TIPO_SAFETY					            :SINT:=SINT#-2;
INP_STOP_CYCLE                                  :INT:=01;
INP_EMERGENCY                                   :INT:=02;
INP_PHOTOCELLS_NOT_VIOLATED                     :INT:=03;
INP_SEARCH_WELD                                 :INT:=25;
INP_MEASURE_TERMINAL_TUBE                       :INT:=40;
INP_FEEDBACK_LOAD_TUBE                          :INT:=60;
INP_PEDAL_LOAD_TUBE                             :INT:=61;
INP_STOP_CYCLE_DPCOUPLER                        :INT:=81;
INP_PRESENCE_PIECE_1_AL42                       :INT:=102;
INP_PRESENCE_PIECE_2_AL42                       :INT:=103;
INP_INTEGRATED_SOUL                             :INT:=116;
INP_CHECK_EMERGENCY                             :INT:=128;
INP_CHECK_BARRIER                               :INT:=129;
INP_RESET_DRIVES                                :INT:=131;
INP_AL5_TRASLATORE_INDIETRO                     :INT:=133;
INP_AUTO                                        :INT:=134;
INP_AL5_SINCR_CATENA_SX                         :INT:=139;
INP_AL5_SINCR_CATENA_DX                         :INT:=140;
INP_RADAR_ZONA_1                                :INT:=143;
INP_RADAR_ZONA_2                                :INT:=144;
INP_RADAR_ZONA_3                                :INT:=145;
INP_HOLD_TO_RUN                                 :INT:=173;
INP_PEDALE_SAFETY                               :INT:=176;
INP_BARRIER_2                                   :INT:=177;
INP_RADAR_ZONA_BIT_0                            :INT:=178;
INP_RADAR_ZONA_BIT_1                            :INT:=179;
INP_ENABLE_HOLD_TO_RUN                          :INT:=180;
INP_EMERGENCY_PLC_SAFETY						:INT:=181;
INP_JOG_PLUS                                    :INT:=188;
INP_JOG_MINUS                                   :INT:=189;
INP_CHECK_COMFORT3                              :INT:=235;
INP_PHOTOCELLS_NOT_VIOLATED_PLC_SAFETY	        :INT:=291;
INP_HOLD_TO_RUN_PLC_SAFETY						:INT:=294;
INP_PEDALE_SAFETY_PLC_SAFETY					:INT:=295;
INP_RESET_DRIVE_PLC_SAFETY				        :INT:=292;
INP_BARRIER_2_PLC_SAFETY						:INT:=293;
INP_RADAR_ZONA_BIT_0_PLC_SAFETY					:INT:=296;
INP_RADAR_ZONA_BIT_1_PLC_SAFETY					:INT:=297;
INP_NAST_TRANSPORTER_FULL					    :INT:=298;
INP_OPTICAL_REARMING_PLC_SAFETY					:INT:=301;
INP_BARRIERA_3_PLC_SAFETY						:INT:=302;
INP_EMERGENCY_EXTERIOR_PLC_SAFETY				:INT:=303;
INP_EXTERNAL_BARRIER_PLC_SAFETY					:INT:=304;
INP_RESET_EXTERNAL_DRIVES_PLC_SAFETY	        :INT:=305;
INP_CONTINUOUS_CYCLE_DPCOUPLER                  :INT:=201;


//OUT

 OUT_NAST_TRANSPORTER                          :INT:=15;
 OUT_CARICO_MACCHINA_PRONTA                    :INT:=19;
 OUT_CHIUSURA_CILINDRO_SOTTOTESTA              :INT:=24;
 OUT_APERTURA_CILINDRO_SOTTOTESTA              :INT:=25;
 OUT_APERTURA_INTERMEDIA_CILINDRO_SOTTOTESTA   :INT:=177;
 OUT_CARICO_MACCHINA_PRONTA_DPCOUPLER          :INT:=65;
 OUT_FCY                                       :INT:=138;
 OUT_CARICO_MACCHINA_TUBO_PRESO                :INT:=50;
 OUT_CARICO_MACCHINA_TUBO_PRESO_DPCOUPLER      :INT:=66;
 OUT_SCARICO_MACCHINA_PRONTA                   :INT:=21;
 OUT_SCARICO_MACCHINA_PRONTA_DPCOUPLER         :INT:=67;
 OUT_SCARICO_MACCHINA_TUBO_LIBERO			    :INT:=121;
 OUT_SCARICO_MACCHINA_TUBO_LIBERO_DPCOUPLER      :INT:=310;
 OUT_CODIFICA_PROGRAMMA_OK_DPCOUPLER             :INT:=309;

 OUT_CARICO_TUBO_PRESENTE_AL                   :INT:=91;
 OUT_CARICO_TUBO_PRESENTE_AL_DPCOUPLER         :INT:=222;

 OUT_CARICO_TUBO_LIBERO_AL                     :INT:=92;
 OUT_AL5_ALZO_FASCIO                           :INT:=93;
 OUT_AL5_ABBASSO_FASCIO                        :INT:=94; 
 OUT_CARICO_TUBO_LIBERO_AL_DPCOUPLER           :INT:=223;


 OUT_PRESSIONE_PNEUMATICA_AL                   :INT:=96;
 OUT_ALZO_SOSTEGNO_TUBO_AL42                   :INT:=100;
 OUT_ABBASSO_SOSTEGNO_TUBO_AL42                :INT:=101;
 OUT_AL5_FRENO_FASCIO                          :INT:=110;
 OUT_AL5_RICHIESTA_CARICO_FASCIO               :INT:=118;
 OUT_CUTTING_KNIFE_HORIZONTAL                  :INT:=127;
 OUT_AL5_AVANTI_TRASLATORE                     :INT:=136;
 OUT_AL5_INDIETRO_TRASLATORE                   :INT:=137;
 OUT_CUTTING_KNIFE_VERTICAL                    :INT:=128;
 OUT_TEST_PIECE                                :INT:=139;

 OUT_AL5_REGOLAZIONE_PRESS_PNEUMATICA         :INT:=151;
OUT_AL5_ENABLE_MOTORE_ROTAZIONE_TB            :INT:=152;
OUT_AL5_SELEZ_VEL_MAX_ROT_TB                  :INT:=153;
OUT_AL5_SELEZ_VEL_MIN_ROT_TB                  :INT:=154;
OUT_AL5_START_ROLAND                          :INT:=155;
OUT_AL5_STROBE_SELEZ_ROLAND                   :INT:=156;
OUT_AL5_SELEZ_M1_ROLAND                       :INT:=157;
OUT_AL5_SELEZ_M2_ROLAND                       :INT:=158;
OUT_AL5_SELEZ_M4_ROLAND                       :INT:=159;
OUT_AL5_SELEZ_M8_ROLAND                       :INT:=160;
OUT_AL5_SELEZ_M16_ROLAND                      :INT:=161;
OUT_AL5_ESCLUSIONE_MAGNETIZZAZIONE            :INT:=162;
OUT_AL5_MARCATURA_POWER_OFF                   :INT:=196;
OUT_AL5_MARCATURA_START_CICLO                 :INT:=197;
OUT_AL5_MARCATURA_SEL_PROG_BIT_0              :INT:=198;
OUT_AL5_MARCATURA_SEL_PROG_BIT_1              :INT:=199;
//Non usare da 200 a 213 in quanto indirizzi comuni a tutte le macchine !!!!
OUT_AL5_MARCATURA_SEL_PROG_BIT_2              :INT:=214;
OUT_AL5_MARCATURA_SEL_PROG_BIT_3              :INT:=215;
OUT_AL5_MARCATURA_SEL_PROG_BIT_4              :INT:=216;
OUT_AL5_MARCATURA_SEL_PROG_BIT_5              :INT:=217;
OUT_ATTESA_ENABLE_SELEZ_ESTERNA_DPCOUPLER     :INT:=227;
OUT_ATTESA_ENABLE_SELEZ_ESTERNA               :INT:=226;
OUT_MACCHINA_ATTESA_START_DPCOUPLER           :INT:=205;
OUT_NAST_TRANSPORTER_DPCOUPLER                :INT:=307;
OUT_TUBO_OK								      :INT:=312;
OUT_AL5_CATENA_DX_AVANTI                      :INT:=106;
OUT_AL5_CATENA_DX_INDIETRO                    :INT:=107;    
OUT_AL5_CATENA_SX_AVANTI                      :INT:=108;
OUT_AL5_CATENA_SX_INDIETRO                    :INT:=109; 

END_VAR

/*********************************************************
* OUTPUTS COMMON TO ALL MACHINES FOR COUPLER MANAGEMENT *
*********************************************************/
VAR_GLOBAL CONSTANT
OUT_MACCHINA_IN_WARNING_DPCOUPLER               :INT:=70;
END_VAR

/***************************************************************
* LOCATES IN THE OADDRES STRUCTURE THE LOCATION OF THE OUTPUT *
*    OUTPUTS COMMON TO ALL MACHINES FOR COUPLER MANAGEMENT    *
***************************************************************/

VAR_GLOBAL CONSTANT
OUT_ANTICIPATION_ENTRY_ROBOT_DPCOUPLER         :INT:=200;
END_VAR

/******************
* DEVICES STATUS *
******************/

VAR_GLOBAL CONSTANT
INTERMEDIATE            :SINT:=SINT#50;
RIPOSO                  :SINT:=SINT#0;
ARRESTO                 :SINT:=SINT#-1;
INDEFINITO              :SINT:=SINT#33;
LAVORO                  :SINT:=SINT#100;
PREAPERTO               :SINT:=SINT#25;
VERSO_LAVORO            :SINT:=SINT#10;
VERSO_RIPOSO            :SINT:=SINT#90;
INTERMEDIO              :SINT:=SINT#50;
INTERMEDIO_PINZA        :SINT:=SINT#80;

END_VAR

/***************
* DEVICE TYPE *
***************/

VAR_GLOBAL CONSTANT

DISP_IN_TT  :SINT:=SINT#1;		//01 - TEMPO LAVORO - TEMPO RIPOSO   xxxxxxx0001
DISP_IN_TM  :SINT:=SINT#3;		//03 - TEMPO LAVORO - MICRO RIPOSO   xxxxxxx0011
DISP_IN_MT  :SINT:=SINT#5;		//05 - MICRO LAVORO - TEMPO RIPOSO   xxxxxxx0101
DISP_IN_MM  :SINT:=SINT#7;		//07 - MICRO LAVORO - MICRO RIPOSO   xxxxxxx0111
DISP_IN_AI	:SINT:=SINT#9;		//09 - INGRESSO ANALOGICO      xxxxxxx1001 
DISP_IN_V   :SINT:=SINT#16;	//16 - INGRESSO VIRTUALE       xxxxxx10000 COMANDATO DALLA LOGICA INTERNA (es: un altro task)
DISP_IN_V_C :SINT:=SINT#17;	//17 - Dispositivo virtuale composto da più dispositivi reali

DISP_OUT_DO :SINT:=SINT#1;		//1 - USCITA DIGITALE         xxxxxxxxx01
DISP_OUT_AO :SINT:=SINT#3; 	//3 - USCITA ANALOGICA        xxxxxxxxx11
DISP_OUT_V_C:SINT:=SINT#5; 	//Dispositivo virtuale composto da più dispositivi reali
END_VAR

/***************
* DEVICE LIST *
***************/

VAR_GLOBAL CONSTANT
CLAMP                         :INT:=FIRST_DEVICE+0;  //32
PALETTA                       :INT:=FIRST_DEVICE+1;  //33
SUPPORT_TIRANT                :INT:=FIRST_DEVICE+2;  //34
BOOSTER                       :INT:=FIRST_DEVICE+3;  //35
CILSOTTOTESTA                 :INT:=FIRST_DEVICE+4;  //36
BLOCCAGGIO_DXSX               :INT:=FIRST_DEVICE+5;  //37
AL42SPINTORE                  :INT:=FIRST_DEVICE+6;  //38
AL42FERMOTUBO                 :INT:=FIRST_DEVICE+7;  //39
AL42SOSTEGNO                  :INT:=FIRST_DEVICE+8;  //40
AL42PLUSTRASLATORE            :INT:=FIRST_DEVICE+9;  //41
AL5_SOLLEVATORE_FASCIO        :INT:=FIRST_DEVICE+10;  //42
CLAMP_CARGE                   :INT:=FIRST_DEVICE+11;  //43
WELD                          :INT:=FIRST_DEVICE+12;  //44
AL5_SINGOLIZZATORE_DX         :INT:=FIRST_DEVICE+13;  //45
AL5_SINGOLIZZATORE_SX         :INT:=FIRST_DEVICE+14;  //46    
AL5_PINZA_DX                  :INT:=FIRST_DEVICE+15;  //47
AL5_PINZA_SX                  :INT:=FIRST_DEVICE+16;  //48
AL5_ALLINEATORE               :INT:=FIRST_DEVICE+17;  //49
AL5_TRASLATORE                :INT:=FIRST_DEVICE+18;  //50
AL42CADENZATORE               :INT:=FIRST_DEVICE+19;  //51
AL42PLUSPINZE                 :INT:=FIRST_DEVICE+20;  //52
AL5_SUPPORTO_DX_ROLAND        :INT:=FIRST_DEVICE+21;  //53
AL5_SUPPORTO_SX_ROLAND        :INT:=FIRST_DEVICE+22;  //54
AL5_PALETTA_ROLAND            :INT:=FIRST_DEVICE+23;  //55
AL5_RULLI_ROLAND              :INT:=FIRST_DEVICE+24;  //56
AL5_SLITTA_ROLAND             :INT:=FIRST_DEVICE+25;  //57
AL5_SINGOLIZZATORE_3          :INT:=FIRST_DEVICE+26;  //58
AL5_SINGOLIZZATORE_4          :INT:=FIRST_DEVICE+27;  //59
AL5_PINZA_3                   :INT:=FIRST_DEVICE+28;  //60
AL5_PINZA_4                   :INT:=FIRST_DEVICE+29;  //61
TAGLIO_LAMA                   :INT:=FIRST_DEVICE+30;  //62
VERTICALE_1_REGGI_TUBO        :INT:=FIRST_DEVICE+31;  //63
VERTICALE_2_REGGI_TUBO        :INT:=FIRST_DEVICE+32;  //64
PINZA_REGGI_TUBO              :INT:=FIRST_DEVICE+33;  //65
RISCALDAMENTO                 :INT:=FIRST_DEVICE+34;  //66
CARICO_EUROTUBI               :INT:=FIRST_DEVICE+35;  //67
ANIMA_SOSPESA									:INT:=FIRST_DEVICE+36;  //68


MAX_TASTI_ASSI      :USINT:=USINT#15;
//Rappresenta la mappatura degli assi sulla pulsantiera macchina


Y1_U4 : INT  := _FIRST_KIN_AXES#FIRST_KIN_AXES;
Q1_V1_Z1 : INT :=  _FIRST_KIN_AXES#FIRST_KIN_AXES + 1;
END_VAR

VAR_GLOBAL CONSTANT
HEAD_1                          :INT := FIRST_DEVICE  + 0;  //32
HEAD_2                          :INT := FIRST_DEVICE + 1;  //33
HEAD_1_SX                       :INT:=FIRST_DEVICE  +0;  //32
HEAD_2_SX                       :INT:=FIRST_DEVICE  +1;  //33
HEAD_1_DX                       :INT:=FIRST_DEVICE  +2;  //34
HEAD_2_DX                       :INT:=FIRST_DEVICE  +3;  //35
DISAPPEARANCE_SHOULDER_LEFT     :INT:=FIRST_DEVICE  +4;  //36
DISAPPEARANCE_SHOULDER_RIGHT    :INT:=FIRST_DEVICE  +5;  //37
MORSA_FOUR_RLFORM               :INT := FIRST_DEVICE + 15;	//47
// {
// #ifndef BASCULANTE_ELETTRICO
// }
BASCULANTE                    :INT := FIRST_DEVICE + 7;  //39
// {
// #endif  //Fine NO BASCULANTE_ELETTRICO
// }
LOCKING_DXSX                    :INT := FIRST_DEVICE + 5;  //37 Posizionamento Bloccaggio lato DX/SX
DISAPPEARANCE_ROLLERS           :INT := FIRST_DEVICE + 6;  //38 
END_VAR
/*************************************
* CONSTANTS BY DEFINITION OF TIMERS *
*************************************/

VAR_GLOBAL CONSTANT
MAX_TIMER_SISTEMA 	        :USINT:=USINT#47; 
T_LOG_AXIS	                :INT:=4;   //Log della trace degli assi        
T_ENERGY_SAVING    	        :INT:=18;
T_NAST_FULL 				:INT:=41;
T_SINCR_CATENE_AL5	        :INT:=6;   //Tempo sincronizzazione catene AL5
END_VAR

/*********************************************************
* OUTPUTS COMMON TO ALL MACHINES FOR COUPLER MANAGEMENT *
*********************************************************/
VAR_GLOBAL CONSTANT
OUT_DRIVES_ON_DPCOUPLER										:INT:=313;
END_VAR

/**************
* BACKGROUND *
**************/

VAR_GLOBAL CONSTANT
ACCESS_NO				:SINT:=SINT#7;
ACCESS_OPERATORE     	:SINT:=SINT#6;
ACCESS_PROGRAMMATORE	:SINT:=SINT#5;
ACCESS_MANUTENTORE   	:SINT:=SINT#4;
ACCESS_SAT           	:SINT:=SINT#3;
ACCESS_MIS           	:SINT:=SINT#2;
ACCESS_UT           	:SINT:=SINT#1;
ACCESS_SUPERUSER     	:SINT:=SINT#0;
END_VAR

/************************************
* ADDRESSES VALID FOR ALL MACHINES *
************************************/

VAR_GLOBAL CONSTANT

INP_LOAD_COMMAND_CENTRAL_DPCOUPLER        :INT:=79;
INP_RESET_DPCOUPLER                       :INT:=82;
INP_AUTO_DPCOUPLER                        :INT:=83;
INP_JOG_DPCOUPLER                         :INT:=84;
END_VAR

/***********************************************************************
* CONSTANTS FOR VARIABLE I_SYS_BUTTONEVENTCOUNTER MACHINE EVENT COUNT *
***********************************************************************/
VAR_GLOBAL CONSTANT
RESET							:USINT:=USINT#8;
BUTTON_PLUS 			        :USINT:=USINT#12;
BUTTON_MINUS			        :USINT:=USINT#16;
END_VAR

/**************
* STOP POINT *
**************/
VAR_GLOBAL CONSTANT
STOPP_START_LOADING_UNLOADING_PARALLEL 	:INT:=26;	//Inizio carico nuovo tubo contemporaneamente allo scarico del tubo appena curvato
END_VAR

/*******************************************
* MESSAGE DECLARATION (MESSAGES IN GREEN) *
*******************************************/
VAR_GLOBAL CONSTANT
MSG_NOMESSAGE							:INT:=0;   //Cancellazione messaggio  
MSG_NO_MEMORY_OVERRIDE			        :INT:=37;  //Memorizzazione tabella non possibile per override <100
END_VAR

/*****************************
* CONSTANTS FOR WELD SEARCH *
*****************************/

VAR_GLOBAL CONSTANT
OVERRIDE_STANDARD                          	:INT:=100;
END_VAR

/********
* AXES *
********/

VAR_GLOBAL CONSTANT
//KinAxesX1_U2  														:UINT :=UINT#500;
KinAxesW3_W4_Q1  												    :UINT :=UINT#300;
//KinAxesW3_W4_W6_Q1  												:UINT :=UINT#504;
KinAxesY1_U4  														:UINT :=UINT#301;
KinAxesQ1_V1_Z1  												    :UINT :=UINT#302;
KinAxesX1_U2                                                        :UINT :=UINT#303;
// KinAxesY1_X1  												        :UINT :=UINT#505;
// KinAxesX1_Y1  												        :UINT :=UINT#506;
// KinAxesX2_Y2  												        :UINT :=UINT#507;
// KinAxesX3_X6  												        :UINT :=UINT#508;
// KinAxesX2_X6  												        :UINT :=UINT#509;
// KinAxesY1_U5  												        :UINT :=UINT#510;
// KinAxesY1_U4_Y2 												    :UINT :=UINT#511;
// KinAxesY1_Y2  												        :UINT :=UINT#512;
// KinAxesQ1_V1_V4  												    :UINT :=UINT#514;
// KinAxesQ1_V1  												        :UINT :=UINT#515;


END_VAR

/*******
* OUT *
*******/
VAR_GLOBAL CONSTANT

OUT_01_STOP_POINT                             :INT:=163;
OUT_02_STOP_POINT                             :INT:=164;
OUT_03_STOP_POINT                             :INT:=165;
OUT_04_STOP_POINT                             :INT:=166;
OUT_05_STOP_POINT                             :INT:=98;
OUT_06_STOP_POINT                             :INT:=99;
OUT_07_STOP_POINT                             :INT:=100;
OUT_08_STOP_POINT                             :INT:=101;
OUT_09_STOP_POINT                             :INT:=102;
OUT_10_STOP_POINT                             :INT:=103;
OUT_11_STOP_POINT                             :INT:=104;
OUT_12_STOP_POINT                             :INT:=105;
OUT_13_STOP_POINT                             :INT:=106;
OUT_14_STOP_POINT                             :INT:=107;
OUT_15_STOP_POINT                             :INT:=108;
OUT_16_STOP_POINT                             :INT:=109;
OUT_17_STOP_POINT                             :INT:=110;
OUT_18_STOP_POINT                             :INT:=111;
OUT_19_STOP_POINT                             :INT:=112;
OUT_20_STOP_POINT                             :INT:=113;
OUT_21_STOP_POINT                             :INT:=114;
OUT_22_STOP_POINT                             :INT:=115;
OUT_23_STOP_POINT                             :INT:=116;
OUT_24_STOP_POINT                             :INT:=117;

//  {
//  #else  //Fine PROFINET
//  }
OUT_RADAR_0                                 :INT:=42; //Gestione RADAR 
OUT_RADAR_1                                 :INT:=43; //Gestione RADAR 
OUT_RADAR_2                                 :INT:=44; //Gestione RADAR 
OUT_RADAR_3                                 :INT:=45; //Gestione RADAR
//  {
//  #endif  //Fine NO PROFINET
//  }

END_VAR

/*******
* IN *
*******/
VAR_GLOBAL CONSTANT

INP_01_STOP_POINT                           :INT:=170;
INP_02_STOP_POINT                           :INT:=171;
INP_03_STOP_POINT                           :INT:=172;

END_VAR

/********************************
* ADDRESS FOR I/O ON 4X5 BOARD *
********************************/

VAR_GLOBAL CONSTANT
MIN_ZONA_RADAR_VGP                  :SINT:=SINT#-2;
END_VAR

/***********************************
* INIDIRIZZI FOR I/O ON 4X5 BOARD *
***********************************/
VAR_GLOBAL CONSTANT
INDIRIZZO_X142_SIMOTION             :INT:=256;
INDIRIZZO_X122_SINAMICS             :INT:=257;
INDIRIZZO_X132_SINAMICS             :INT:=258;
END_VAR

/**********************************************
* ADDRESSES FOR FESTO PROFINET INPUT MODULES *
**********************************************/

VAR_GLOBAL CONSTANT

FESTO_INP_LINEA_1_MODULO_1          :INT:=156;  //bit 0÷15  16 input
FESTO_INP_LINEA_1_MODULO_3          :INT:=157;  //bit 0÷07  08 input
FESTO_INP_LINEA_2_MODULO_2          :INT:=154;  //bit 0÷15  16 input
FESTO_INP_LINEA_2_MODULO_3          :INT:=155;  //bit 0÷07  08 input
FESTO_INP_LINEA_3_MODULO_2          :INT:=152;  //bit 0÷15  16 input
FESTO_INP_LINEA_3_MODULO_3          :INT:=153;  //bit 0÷07  08 input
FESTO_INP_LINEA_4_MODULO_2          :INT:=150;  //bit 0÷15  16 input
FESTO_INP_LINEA_4_MODULO_3          :INT:=151;  //bit 0÷07  08 input
END_VAR



END_CONFIGURATION