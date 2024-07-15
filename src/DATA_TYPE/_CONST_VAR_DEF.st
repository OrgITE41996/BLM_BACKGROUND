USING _FC_IO_FC;

NAMESPACE _CONST_VAR_DEF

    
    TYPE
        BIG_VALUE_REAL :  REAL
        (MY_BIG_VALUE_REAL					                       := REAL#99999.0);
    END_TYPE

    TYPE
        J_ID_RESET :SINT
        (JOB_ID_RESET                                              :=SINT#-1);
    END_TYPE

    TYPE
        _MPROGRAMS :UINT
        (MAXPROGRAMS                                               :=UINT#15+UINT#1);
    END_TYPE

    TYPE
        _P_ID_RESET :SINT
        (P_ID_RESET                                                 :=SINT#0);
    END_TYPE

    TYPE 
        _MAXIS : INT
                (*{
            #ifdef B1_B2
            }*)
            (MAXAXIS							:=17); // !!!!!!! (ifdef)
            (*{
            #else  //Fine B1_B2
        
            {
            #endif  //Fine D445
            }
            MAXAXIS                         :INT:=10;
            (*{
            #ifdef MATRICE_ELETTRICA
            }
                {
                #ifdef ASSE_Z2
                }
                MAXAXIS             :INT:=12;
                {
		        #else  //Fine ASSE_Z2
		        }
                MAXAXIS 			:INT:=11;
            {
            #else	//Fine MATRICE_ELETTRICA
            }
            MAXAXIS					:INT:=10;
            /////////////////////////////////////////////////////////////////////////////////////////
            {
            #ifdef ELECTM  
            }
            {
            #ifdef ASSE_Z2
            }
            MAXAXIS                 :INT:=11
              {
            #else  //Fine ASSE_Z2
            }
            MAXAXIS                 :INT:=10;
              {
            #else  //Fine ASSE_Z2
            }
            MAXAXIS               :INT:=10;
            /////////////////////////////////////////////////////////////////////////////////////////
            {
            #ifdef ELECTMRL
            }
	        {
	        #ifdef TAVOLI_ELETTRICI
	        }
            MAXAXIS                 :INT:=13;
	        {
	        #else //TAVOLI_ELETTRICI
	        }
		    MAXAXIS					:INT:=11;
	        {
	        #endif //FINE NO TAVOLI_ELETTRICI
	        }
            MAXAXIS                   :INT:=9;
            MAXAXIS                   :INT:=5;
              //Elenco assi
            MAXAXIS                 :INT:=14;
            //Elenco assi
            MAXAXIS                 :INT:=11;
            //Elenco assi
            }
                MAXAXIS								:INT:=15;
            {
            #endif  //Fine NO B1_B2
            }
            //Elenco assi
	        MAXAXIS               	:INT:=6;
             //Elenco assi
            {
            #ifdef B1_B2
            }
                MAXAXIS								:INT:=8;
            {
            #else  //Fine B1_B2
            }
                MAXAXIS               :INT:=6;
            {
            #endif  //Fine NO B1_B2
            }
            {
            #ifdef ASSE_Z2
            }
                MAXAXIS								:INT:=14;  
            {
            #else	//Fine ASSE_Z2
            }
            MAXAXIS								:INT:=13;  
            {
            #endif	//Fine NO ASSE_Z2
            }
            	{
            #ifdef ASSE_Z2
            }
                MAXAXIS								:INT:=14;  
            {
            #else	//Fine ASSE_Z2
            }
            MAXAXIS								:INT:=13;  
            {
            #endif	//Fine NO ASSE_Z2
            }
            {
            #ifdef ASSE_Z2
            }
                MAXAXIS             :INT:=16; 
            {
            #else  //Fine ASSE_Z2
            }
                MAXAXIS             :INT:=15;  
            {
            #endif  //Fine NO ASSE_Z2
            }
              {
            #ifdef BASCULANTE_ELETTRICO
            }
                MAXAXIS                 :INT:=10;
            {
            #else  //Fine BASCULANTE_ELETTRICO
            }
                MAXAXIS                 :INT:=9;
            {
            #endif  //Fine NO BASCULANTE_ELETTRICO
            }
              {
            #ifdef BASCULANTE_ELETTRICO
            }
                MAXAXIS                 :INT:=10;
            {
            #else  //Fine BASCULANTE_ELETTRICO
            }
                MAXAXIS                 :INT:=9;
            {
            #endif  //Fine NO BASCULANTE_ELETTRICO
            }
            /////////////////////////////////////////////////////////////////////////////////////////
            {
            #ifdef SMART_A
            }
            {
                #define BLOCCAGGIO_DX
                #define SPALLAMENTO_DX
                #define SLITTA_MOBILE_DX
                #define SICUREZZA_TIRANTE_ANIMA
                #define DXSX_PROCESSO
                #define SMART_SMART_A
                #define MDASPECIALE
                #define LINEARIZZA_ASSE  
                #define TIRANTE_ANIMA_FC
                #define INTERPOLA_Q1_V1_Z1
                #define CURVATURA
                #define CARICO_AUTOMATICO
            }
            MAXAXIS                 : INT := 9;
            MAXAXIS               :INT:=12;*)

                // ASSI MACCHINA
    END_TYPE
    TYPE
        (*MY_B_Value : REAL
        (MY_BIG_VALUE_REAL                   :=REAL#99999.0);*)
    END_TYPE

    TYPE
        _MAX_TYPES_WORKS :USINT
        (MAX_TYPES_WORKS                       :=USINT#14);
    END_TYPE


    TYPE
    DEVICES_STATUS : SINT
    (RIPOSO             :=SINT#0,
    INDEFINITO          :=SINT#33,
    LAVORO              :=SINT#100,
    ARRESTO             :=SINT#-1,
    VERSO_LAVORO        :=SINT#10,
    VERSO_RIPOSO        :=SINT#90);

    END_TYPE

    TYPE
        _MAX_AXIS_LINEARISES : INT
        (MAX_AXIS_LINEARISES                	:=1);
    END_TYPE

    TYPE
        _MAXAXIS_MASTER     : INT
        (MAXAXIS_MASTER					:=10);
    END_TYPE

             (*  {
            #else  //Fine DH4010  
            }*)
    TYPE
        TABLE :SINT
            (TABLE_LX                          :=SINT#0,
            TABLE_RX                           :=SINT#0);
    END_TYPE
         (* {
          #endif  //Fine NO DH4010  
          }*)
    TYPE
    _MAXSTATES :INT
          (MAXSTATES                             :=10000);
    END_TYPE
    TYPE
    _MAXCOL_STATES :INT
          (MAXCOL_STATES                         :=29);
    END_TYPE
    
    TYPE
        _MAX_AXES :USINT
        (MAX_AXES_KEYS                      :=USINT#15);
    END_TYPE

    // TYPE
    //     _MYARRAYINT : Struct
    //     MYARRAY : ARRAY [0..5] OF INT;
    //     END_struct;
    // END_TYPE

    TYPE
        _MAXCOL_DEVICE :INT
        (MAXCOL_DEVICE  :=_MAXIS#MAXAXIS+_MAXDEVICES#MAXDEVICES); 
    END_TYPE

    TYPE
        _MAXDEVICES : INT
        (MAXDEVICES                      :=37);
    END_TYPE

    TYPE
        _MAXSEQ:UINT
        (MAXSEQ                              :=UINT#350);
    END_TYPE

    TYPE
        _MAX_KEY_DISP :USINT
        (MAX_KEY_DISP                    :=USINT#27);
    END_TYPE

    TYPE
        _FIRST_AXIS :INT
       (FIRST_AXIS                      :=0);
    END_TYPE

    TYPE
        _MAX_LOG : INT
        (MAX_LOG_SIZE                       	:=500);  //Lunghezza del buffer di log
    END_TYPE

    // TYPE   ???? 
    //     _ULTIMO_STOP_POINT_SET_OUT: INT
    //     (ULTIMO_STOP_POINT_SET_OUT             	:=_PRIMO_STOP_POINT_SET_OUT#PRIMO_STOP_POINT_SET_OUT+_MAX_INP_OUT_STOP_POINT#MAX_INP_OUT_STOP_POINT-1);//251
    // END_TYPE

    // TYPE
    //     _PRIMO_STOP_POINT_SET_OUT : INT
    //     (PRIMO_STOP_POINT_SET_OUT              	:=228);
    // END_TYPE

    // TYPE
    //     _MAX_INP_OUT_STOP_POINT : INT
    //     (MAX_INP_OUT_STOP_POINT                  :=24);

    // END_TYPE

    TYPE
        _FIRST_KIN_AXES : INT
        (FIRST_KIN_AXES := 0);
    END_TYPE

    TYPE
        _MAX_KIN_AXES   : INT
        (MAX_KIN_AXES  := 4);
    END_TYPE
END_NAMESPACE