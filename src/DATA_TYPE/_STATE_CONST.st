USING _TYPE;
NAMESPACE _STATE_CONST

    TYPE
        GENERAL_STATE : INT 
                    (STATE_INIT										 	:=0,			//stato iniziale generico. tutte le funzioni iniziano da questo stato ,
                    STATE_END										  	:=9999); 		//stato finale generico. tutte le funzioni finiscono con questo stato ,
    END_TYPE

    TYPE
        SettingsTO :INT	
                    (STATE_GET_POSAX_TO_STATE									:=210 ,
                    STATE_ACTIVATE_DEACTIVATE_POSAX_TO					 		:=211 ,
                    STATE_ACTIVATE_POSAX_TO										:=212 ,
                    STATE_DEACTIVATE_POSAX_TO									:=213 ,
                    STATE_GET_SYNCHRO_TO_STATE									:=214 ,
                    STATE_ACTIVATE_DEACTIVATE_SYNCHRO_TO				 		:=215 ,
                    STATE_ACTIVATE_SYNCHRO_TO									:=216 ,
                    STATE_DEACTIVATE_SYNCHRO_TO									:=217);
    END_TYPE
    
    TYPE
        _FB_ReadSafetyData : INT
                    (STATE_READ_SAFETY_DATA_LOOP_AXES							:=2420 ,
                    STATE_READ_SAFETY_DATA_READ_MAXIMUM_TEMPERATURE				:=2421 ,
                    STATE_READ_SAFETY_DATA_READ_R9398_0							:=2422 ,
                    STATE_READ_SAFETY_DATA_READ_R9398_1							:=2423 ,
                    STATE_READ_SAFETY_DATA_READ_R9728_0							:=2424 ,
                    STATE_READ_SAFETY_DATA_READ_R9728_1							:=2425 ,
                    STATE_READ_SAFETY_DATA_READ_R9728_2							:=2426 ,
                    STATE_READ_SAFETY_DATA_READ_R9798_0							:=2427 ,
                    STATE_READ_SAFETY_DATA_READ_R9898_0							:=2428 ,
                    STATE_READ_SAFETY_DATA_READ_R9601_0							:=2429 ,	
                    STATE_READ_SAFETY_DATA_READ_NUMBER_ERRORS					:=2430 ,
                    STATE_READ_SAFETY_DATA_LOOP_ERROR_CODES					    :=2431 ,
                    STATE_READ_SAFETY_DATA_READ_ERROR							:=2432 ,
                    STATE_READ_SAFETY_DATA_READ_AXIS_TYPE						:=2433 ,
                    STATE_READ_SAFETY_DATA_READ_LOAD_REVOLUTIONS_AZ				:=2434 ,
                    STATE_READ_SAFETY_DATA_READ_LEAD_AZ					        :=2435 ,
                    STATE_READ_SAFETY_DATA_READ_MOTOR_REVOLUTIONS_AZ			:=2436 );
    END_TYPE

    TYPE
        ReadingSinamicsREAL : INT
                    (STATE_READ_SINAMICS_REAL_READ_PARAMETER					:=1290,
                    STATE_READ_SINAMICS_REAL_READ_DESCRIPTION				    :=1291);
    END_TYPE

    TYPE
    	_PROGRAM_ENABLEAX : INT
		                (STATE_ENABLEAX_WAIT_AXES_VERIFIED                                  :=5450,
		                STATE_ENABLEAX_CHECK_RUN_RESET                                      :=5451,
		                STATE_ENABLEAX_SET_SYSTEM_STARTED                                   :=5452,
		                STATE_ENABLEAX_ENDLESS_LOOP_CHECKSUM                                :=5453,
		                STATE_ENABLEAX_CALCULATE_FACTOR_PRECOMMAND                          :=5454,
		                STATE_ENABLEAX_ENABLE_BUSY                                          :=5455,
		                STATE_ENABLEAX_RESET_SMART_LINE                                     :=5456,
		                STATE_ENABLEAX_LOOP_AXIS_REVERSAL_DIRECTION                         :=5457,
		                STATE_ENABLEAX_LOOP_AXIS_REVERSE_SENSE_REVERSES_AXIS                :=5458,
		                STATE_ENABLEAX_WAITING_STARTUP_SYSTEM                               :=5459,
		                STATE_ENABLEAX_WAIT_SINAMIC_READY                                   :=5460,
		                STATE_ENABLEAX_LOOP_AXIS_CHANGE_DATASET                             :=5461,
		                STATE_ENABLEAX_READ_ENGINE_STATUS                                   :=5462,
		                STATE_ENABLEAX_CHANGE_SIMULATION_PARAMETERS_SET                     :=5463,
		                STATE_ENABLEAX_NON_SIMULATION_AXIS                                  :=5464,
		                STATE_ENABLEAX_CHANGE_TRUE_ENCODER_PARAMETERS_SET                   :=5465,
		                STATE_ENABLEAX_READ_ENCODER_NUMBER                                  :=5466,
		                STATE_ENABLEAX_READ_SINAMICS_ENCODER_NUMBER                         :=5467,
		                STATE_ENABLEAX_SET_MOBILE_SLIDE_TYPE                                :=5468,
		                STATE_ENABLEAX_CHECK_SERIOUS_ERROR                                  :=5469,
		                STATE_ENABLEAX_AXIS_ENCODER_LOOP_OK                                 :=5470,
		                STATE_ENABLEAX_WAIT_REAL_VELOCITY_ZERO                              :=5471,
		                STATE_ENABLEAX_SET_VECTORIAL_OBJECTS_ACCELERATION                   :=5472,
		                STATE_ENABLEAX_ENABLE_AXES                                          :=5473,
		                STATE_ENABLEAX_DISABLE_RESTART_LOOP                                 :=5474,
		                STATE_ENABLEAX_DISABLE_RESTART_LOOP_EXECUTION                       :=5475,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE                     :=5476,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE_1441                :=5477,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMETERS_WRITE_1460                :=5478,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1462                    :=5479,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1520                    :=5480,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1521                    :=5481,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1715                    :=5482,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_1717                    :=5483,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_2175                    :=5484,
		                STATE_ENABLEAX_LOOP_AXIS_WRITE_PARAMS_WRITE_2177                    :=5485,
		                STATE_ENABLEAX_CHECK_PARAM_RW_2                                     :=5486,
		                STATE_ENABLEAX_SYSTEM_STARTUP_WAIT_2                                :=5487,
		                STATE_ENABLEAX_COPY_RAM_ROM_2                                       :=5488,
		                STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS                                :=5489,
		                STATE_ENABLEAX_LOOP_AXIS_WAIT_SINAMICS_FREE_READ                    :=5490,
		                STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_SIMULATION_VERIFICATION        :=5491,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_35                        :=5492,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_1441                      :=5493,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMS_READ_1460                      :=5494,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1462                  :=5495,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1520                  :=5496,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1521                  :=5497,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1538                  :=5498,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1539                  :=5499,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1715                  :=5500,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1717                  :=5501,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_2175                  :=5502,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_2177                  :=5503,
                        STATE_ENABLEAX_LOOP_AXIS_READ_PARAMETERS_READ_1215                  :=5504,
                        STATE_ENABLEAX_LOOP_RESTART_ACTIVATION_NO_PROFINET                  :=5505,
                        STATE_ENABLEAX_LOOP_ALIGN_ACCELERATION_VELOCITY                     :=5506,
                        STATE_ENABLEAX_ENGINE_SPEED_NO_PROFINET_READ                        :=5507,
                        STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_READ              :=5508,
                        STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_WAIT_SINAMICS_FREE_READ :=5509,
                        STATE_ENABLEAX_LOOP_ENGINE_SPEED_READ_NO_PROFINET_READ_2000         :=5510,
                        STATE_ENABLEAX_LOOP_ENGINE_SPEED_NO_PROFINET_RESTART_READ           :=5511,
                        STATE_ENABLEAX_ENGINE_SPEED_READ_COPY_RAM_ROM_NO_PROFINET           :=5512,
                        STATE_ENABLEAX_GENERIC_DRIVE_READ                                   :=5513,
                        STATE_ENABLEAX_GENERIC_DRIVE_READ_PARAM                             :=5514,
                        STATE_ENABLEAX_LOOP                                                 :=5515,
                        STATE_ENABLEAX_ORBITAL_XX_RESET                                     :=5516,
                        STATE_ENABLEAX_DISABLEABLE_AXIS_STATE_TEST                          :=5517,
                        STATE_ENABLEAX_MODULE_Z1                                            :=5518,
                        STATE_ENABLEAX_MODULE_B1_B2                                         :=5519,
                        STATE_ENABLEAX_MODULE_A1_A2                                         :=5520,
                        STATE_ENABLEAX_CHANGE_FOUR_RUNNER_PARAMETERS_SET                    :=5521,
                        STATE_ENABLEAX_SYSTEM_STARTUP_3_WAIT                                :=5522,
                        STATE_ENABLEAX_ENCODER_LOOP                                         :=5523,
                        STATE_ENABLEAX_ENCODER_LOOP_CHANGE_PARAMETERS                       :=5524,
                        STATE_ENABLEAX_DISABLE_LOOP_L1                                      :=5525,
                        STATE_ENABLEAX_READ_SAFETY_DATA_BEFORE_STARTUP                      :=5526,
                        STATE_ENABLEAX_RESET_DRIVER_PRESSED                                 :=5527,
                        STATE_ENABLEAX_RESET_DRIVER_PRESSED_MECHANICAL_SAFETY_TEST          :=5528,
                        STATE_ENABLEAX_RESET_DRIVER_PRESSED_ENDLESS_LOOP_MECHANICAL_SAFETY  :=5529,
                        STATE_ENABLEAX_MODULE_0_360_TESTHEADS                               :=5530,
                        STATE_ENABLEAX_MODULE_0_360_TESTHEADS_2                             :=5531,
                        STATE_ENABLEAX_SET_STARTUP_FALSE                                    :=5532,
                        STATE_ENABLEAX_NUM_STATES_TEST                                      :=5533,
                        STATE_ENABLEAX_MASTER_VELOCITY_CHECK                                :=5534,
                        STATE_ENABLEAX_HOMING_CHECK_U2                                      :=5535,
                        STATE_ENABLEAX_DEVICES_ENABLE_LOOP                                  :=5536,
                        STATE_ENABLEAX_SET_STARTUP_FALSE_FINAL                              :=5537);
    END_TYPE

    TYPE
        SF :INT ///Ask how to def type
            ( C_RESET_SIMOTION                  :=261);
    END_TYPE

    TYPE
        PI_COMMANDS_CODE : INT
                        (REFMODE                                :=1,
                        MDAMODE                                 :=2,
                        AUTOMODE                                :=3,
                        CHANNELRESET                            :=0	(*PROGRAMMI TUTTI RESETTATI*));
    END_TYPE

    TYPE
        MAX_NUMBER_STATES_FLASHING_4COLOR :INT
                                        (UNDEF_DEVICE                       	:=-1);

    END_TYPE

    TYPE
        _ERR : DINT
                (ERR_FIRST_BLM                                  :=DINT#100000, // Primo codice di errore BLM
		        ERR_FIRST_ERROR_SIEMENS						:=DINT#20001);
    END_TYPE

    TYPE
        ERROR_TYPE : INT
                    (ERROR_BLM					:=9);   //Errore )
    END_TYPE

    TYPE
        CONSTANTS_WELD_SEARCH : INT
        (OVERRIDE_STANDARD                          	:=100);
    END_TYPE
    
    TYPE
        FB_COPY_RAM_ROM : INT
		                (STATE_COPY_RAM_ROM_WAIT_SAVE_FINISHED	    :=5560);
    END_TYPE

    TYPE
        _FB_CHANGE_SET_PARAMETERS : INT
		(STATE_CHANGE_SET_PARAMETERS_SET_DATASET							                    :=1400,
		STATE_CHANGE_SET_PARAMETERS_WAITING_SET_DATASET									        :=1401,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_SIMUL													:=1402,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_RESET_AXIS										        :=1403,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_DISABLE_AXES									        :=1404,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_SET_DATASET										        :=1405,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_HOMING												    :=1406,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_RESET_ASSE_2									        :=1407,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_ENABLE_AXIS										        :=1408,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_AXIS													:=1409,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_RESET_AXIS							            :=1410,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_SET_DATASET							            :=1411,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_DISABLE_AXIS						                :=1412,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_HOMING									        :=1413,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_RESET_AXIS_2						                :=1414,
		STATE_CHANGE_SET_PARAMETERS_ORBITAL_REAL_ENABLE_AXIS							            :=1415,
		STATE_CHANGE_SET_PARAMETERS_EMERGENCY_RESET												    :=1416,
		STATE_CHANGE_SET_PARAMETERS_ELIMINATE_QUEUE_COMMANDS										        :=1417);
    END_TYPE

    TYPE
        _FB_WaitingStartUpSystem :INT
        (STATE_WAIT_START_SYSTEM_GET_STATE_MASTER_1                  :=1200,
        STATE_WAIT_START_SYSTEM_WAITE_MASTER_1                      :=1201,
        STATE_WAIT_START_SYSTEM_CHECK_EMULATION                         :=1202,
        STATE_WAIT_START_SYSTEM_GET_STATE_MASTER_2                      :=1203,
        STATE_WAITING_START_SYSTEM_WAITING_MASTER_2                      :=1204,
        STATE_WAIT_START_SYSTEM_CHECK_FEEDER_ACTIVE_LINE                :=1205,
        STATE_WAIT_START_SYSTEM_GET_CU32_1_STATE                        :=1206,
        STATE_WAIT_START_SYSTEM_GET_CU32_1_FIRMWARE                     :=1207,
        STATE_WAIT_START_SYSTEM_GET_CU32_2_STATE                        :=1208,
        STATE_WAIT_START_SYSTEM_GET_CU32_2_FIRMWARE                     :=1209,
        STATE_WAIT_START_SYSTEM_CHECK_AXES                              :=1210,
        STATE_WAIT_START_SYSTEM_WAIT_POWER_SUPPLY                       :=1211,
        STATE_WAIT_START_SYSTEM_WAIT_START_POSAXIS                            :=1212,
        STATE_WAIT_START_SYSTEM_WAIT_OBJECTS_VECTORS                    :=1213,
        STATE_WAIT_START_SYSTEM_WAIT_STATE_TO_POSAXIS                   :=1214,
        STATE_WAIT_START_SYSTEM_WAIT_TO_SYNCHRO                         :=1215,
        STATE_WAITING_START_SYSTEM_WAITING_STATE_TO_SYNCHRO             :=1216,
        STATE_WAIT_START_SYSTEM_WAITING_TO_CAMMA                  :=1217,
        STATE_WAIT_START_SYSTEM_WAITING_STATE_TO_CAMMA                  :=1218,
        STATE_WAIT_START_SYSTEM_READING_LOGADDRESSIN                    :=1219,
        STATE_WAIT_START_SYSTEM_WAIT_MASTER_AXIS                        :=1220,
        STATE_WAIT_START_SYSTEM_WAITING_STATE_TO_MASTER                 :=1221,
        STATE_WAIT_START_SYSTEM_WAIT_READ_LOGADDRESSIN                  :=1222,
        STATE_WAIT_START_SYSTEM_WAITING_WAIT_ACTION                  :=1223,
        STATE_WAIT_START_SYSTEM_WAIT_LOGADDRESSIN_RETANY_OK             :=1224,
        STATE_WAIT_WAIT_SYSTEM_READ_NUMBEROFENCODERS                    :=1225,
        STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFENCODERS_RETANY_OK    :=1226,
        STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFDATASETS              :=1227,
        STATE_WAIT_START_SYSTEM_WAIT_READ_NUMBEROFDATASETS_RETANY_OK    :=1228,
        STATE_WAIT_START_SYSTEM_WAIT_READ_ENCODERIDENTIFICATION         :=1229,
        STATE_WAIT_START_SYSTEM_WAIT_READ_ENCODERIDENTIFICATION_RETANY_OK   :=1230,
        STATE_WAIT_START_SYSTEM_WAIT_READ_SETPOINTDRIVERINFO            :=1231,
        STATE_WAIT_START_SYSTEM_WAIT_SETPOINTDRIVERINFO_RETANY_OK       :=1232,
        STATE_WAIT_START_SYSTEM_WAIT_READ_CYCLICINTERFACE               :=1233,
        STATE_WAIT_START_SYSTEM_WAIT_READ_CYCLICINTERFACE_RETANY_OK    :=1234,
        STATE_WAIT_START_SYSTEM_WAIT_POSAX_NOT_NULL                     :=1235,
        STATE_WAIT_START_SYSTEM_WAIT_END_RESTART_TO                     :=1236,
        STATE_WAIT_START_SYSTEM_WAIT_AXIS_VALID                         :=1237,
        STATE_WAIT_START_SYSTEM_PARAMETER_2_105                         :=1238,
        STATE_WAIT_START_SYSTEM_READING_PARAM_105                       :=1239,
        STATE_WAIT_START_SYSTEM_WAIT_X1_U2                              :=1240,
        STATE_WAIT_START_SYSTEM_GET_STATE_Y1_X1                         :=1241,
        STATE_WAIT_START_SYSTEM_WAIT_Y1_X1                         :=1242,
        STATE_WAIT_START_SYSTEM_GET_WAIT_STATE_Y1_U4                    :=1243,
        STATE_WAIT_START_SYSTEM_WAIT_Y1_U4                               :=1244,
        STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U5                         :=1245,
        STATE_WAIT_START_SYSTEM_WAIT_Y1_U5                         :=1246,
        STATE_WAIT_START_SYSTEM_GET_STATE_W3_W4_Q1                      :=1247,
        STATE_WAIT_START_SYSTEM_WAIT_W3_W4_Q1                           :=1248,
        STATE_WAIT_START_SYSTEM_GET_STATE_Y1_U4_Y2                      :=1249,
        STATE_WAIT_START_SYSTEM_WAIT_Y1_U4_Y2                           :=1250,
        STATE_WAIT_START_SYSTEM_GET_STATE_W3_W4_W6_Q1                   :=1251,
        STATE_WAIT_START_SYSTEM_WAIT_W3_W4_W6_Q1                        :=1252,
        STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_V4                     :=1253,
        STATE_WAIT_START_SYSTEM_WAIT_Q1_V1_V4                           :=1254,
        STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1                          :=1255,
        STATE_WAIT_START_SYSTEM_WAIT_Q1_V1                             :=1256,
        STATE_WAIT_START_SYSTEM_GET_STATE_Y1_Y2                         :=1257,
        STATE_WAIT_START_SYSTEM_WAIT_Y1_Y2                              :=1258,
        STATE_WAIT_START_SYSTEM_GET_STATE_Q1_V1_Z1                      :=1259,
        STATE_WAIT_START_SYSTEM_WAIT_Q1_V1_Z1                           :=1260,
        STATE_WAIT_START_SYSTEM_WAIT_SYNAMICS                           :=1261,
        STATE_WAIT_START_SYSTEM_LOOP_AXIS_ACTIVE                        :=1262,
        STATE_WAIT_START_SYSTEM_WAIT_ACTION_ACTIVE                     :=1263,
        STATE_WAIT_START_SYSTEM_RESTART_ACTIVE_AXIS                     := 1264
        );
    END_TYPE

    TYPE
        _FB_AttendedStatusTO : INT 
		(STATE_WAITING_TO_GET_STATE							  :=1280);
    END_TYPE

    TYPE
        _FB_RestartActivationAxis :INT
		(STATE_RESTART_AXIS_WAITING								:=1108);
    END_TYPE

    TYPE
    	_FB_ResetAxisBLM :INT
		(STATE_RESET_AXIS_BLM__WAIT							:=1320);
    END_TYPE

    TYPE
        _FB_DisableAxes :INT
		(STATE_DISABLE_AXIS_DISABLE																:=999);
    END_TYPE

    TYPE
    	_FB_CopyRomSinamic : INT
		(STATE_COPY_ROM_SINAMICS_CU																									:=1840,
		STATE_COPY_ROM_SINAMICS_READ_CX1_STATE																			            :=1841,
		STATE_COPY_ROM_SINAMICS_WRITE_CX1_PARAM																			            :=1842,
		STATE_COPY_ROM_SINAMICS_READ_CX2_STATE																			            :=1843,
		STATE_COPY_ROM_SINAMICS_WRITE_CX2_PARAM																			            :=1844,
		STATE_COPY_ROM_SINAMICS_WAIT_CU_AVAILABLE																		            :=1845,
		STATE_COPY_ROM_SINAMICS_WAIT_CU_COPY_COMPLETED															                    :=1846,
		STATE_COPY_ROM_SINAMICS_WAIT_CX1_AVAILABLE																	                :=1847,
		STATE_COPY_ROM_SINAMICS_WAIT_CX1_COPY_COMPLETED															                    :=1848,
		STATE_COPY_ROM_SINAMICS_WAIT_CX2_AVAILABLE																	                :=1849,
		STATE_COPY_ROM_SINAMICS_WAIT_CX2_COPY_COMPLETED															                    :=1850);
    END_TYPE

    TYPE
        _FB_Disable_Axis_In_Torque : INT
		(STATE_DISABLE_AXIS_IN_TORQUE_WAIT																	:=1375);
    END_TYPE

    TYPE
        _FB_ManagementBrake : INT
		(STATE_MANAGEMENT_BRAKE_WAIT_READING_SYNAMICS								:=1109,
		STATE_MANAGEMENT_BRAKE_WRITE_SYNAMICS									    :=1110);
    END_TYPE

    TYPE
        _FB_DeleteQueueCommands :INT 
		(STATE_ELIMINATE_QUEUE_COMMANDS_SET_LINEARIZE_AXES										:=1350,
		STATE_ELIMINATE_QUEUE_COMMANDS_SET_STOP_AXIS_MASTER										:=1351,
		STATE_ELIMINATE_QUEUE_COMMANDS_RESET_AXES_MASTER										:=1352,
		STATE_ELIMINATE_QUEUE_COMMANDS_RESET_VECTOR_OBJECTS								        :=1353,
		STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_STOP_AXIS											:=1354,
		STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_STOP_SINGLE_AXIS										:=1355,
		STATE_ELIMINATE_QUEUE_COMMANDS_LOOP_AXIS_RESET										    :=1356,
		STATE_ELIMINATE_QUEUE_COMMANDS_WAITING_AXES_IN_RESET								    :=1357,
		STATE_ELIMINATE_QUEUE_COMMANDS_CANCEL_RESET											    :=1358,
		STATE_ELIMINATE_QUEUE_COMMANDS_DISABLE_TORQUE											:=1359,
		STATE_ELIMINATE_QUEUE_COMMANDS_EXECUTE_RESET_AXIS										:=1360,
		STATE_ELIMINATE_QUEUE_COMMANDS_AXES_IN_RESET											:=1361,
		STATE_ELIMINATE_QUEUE_COMMANDS_WIDEN_TIME_STANDSTILL								    :=1362,
		STATE_ELIMINATE_QUEUE_COMMANDS_RESET_ALL_CAMS											:=1363,
		STATE_ELIMINATE_QUEUE_COMMANDS_FINAL_SETTINGS											:=1364);
    END_TYPE
    
    TYPE
        _FB_StopMasterAxes :INT
            (STATE_STOP_ASSI_MASTER_STOP_MASTER_2															:=1331);
    END_TYPE

    TYPE
        _FB_Form_Axis : INT
		(STATE_MODULE_WAIT_AXIS_HOMED				:=4280);
    END_TYPE

    TYPE
        _FB_EnableAxis : INT
		(STATE_ENABLEASSI_WAIT_CAMBIO_DX_SX											:=950,
		STATE_ENABLEASSI_WAIT_VERIFICA_IO											:=951,
		STATE_ENABLEASSI_WAIT_VERIFICA_CONFIG_DX_SX							        :=952,
		STATE_ENABLEASSI_VERIFICA_SIMULAZIONE										:=953,
		STATE_ENABLEASSI_VERIFICA_BASCULANTE										:=954,
		STATE_ENABLEASSI_WAIT_BARRIERE_NON_VIOLATE							        :=955,
		STATE_ENABLEASSI_GET_NEXT_STEP												:=956,
		STATE_ENABLEASSI_CHECK_VER_SAFETY											:=957,
		STATE_ENABLEASSI_CHECK_DH4010												:=958,
		STATE_ENABLEASSI_WAIT_ALIMENTATORE_ON										:=959,
		STATE_ENABLEASSI_ENABLE_MASTER												:=960,
		STATE_ENABLEASSI_ENABLE_MASTER_1_DH4010									    :=961,
		STATE_ENABLEASSI_ENABLE_MASTER_2_DH4010									    :=962,
		STATE_ENABLEASSI_CHECK_FOUR_RUNNER											:=963,
		STATE_ENABLEASSI_ENABLE_MASTER_TAGLIO_ESTERNO						        :=964,
		STATE_ENABLEASSI_CHECK_RUNNER												:=965,
		STATE_ENABLEASSI_ENABLE_MASTER_TAGLIO_INTERNO						        :=966,
		STATE_ENABLEASSI_ENABLE_LOOP												:=967,
		STATE_ENABLEASSI_ENABLE_SINGLE_AXIS											:=968,
		STATE_ENABLEASSI_ENABLE_SIMULATION_SINGLE_AXIS					            :=969,
		STATE_ENABLEASSI_WAIT_DRIVESTATE_ACTIVE									    :=970,
		STATE_ENABLEASSI_WAIT_POWER_ACTIVE											:=971,
		STATE_ENABLEASSI_WAIT_CONTROL_ACTIVE										:=972,
		STATE_ENABLEASSI_RETURNVALUE												:=973);
    END_TYPE

    TYPE
    	_FB_EnableSingleAxisBLM :INT
		(STATE_ENABLESINGLEAXIS_PROFINET													:=990,
		STATE_ENABLESINGLEAXIS_WAIT_EMERGENZA										        :=991,
		STATE_ENABLESINGLEAXIS_WAIT_SAFETY_OFF									            :=992,
		STATE_ENABLESINGLEAXIS_CHECK_SAFETY_INTEGRATED					                    :=993,
		STATE_ENABLESINGLEAXIS_INTEGRATED_WAIT_SAFETY_OFF				                    :=994,
		STATE_ENABLESINGLEAXIS_TESTL1														:=995,
		STATE_ENABLESINGLEAXIS_ENABLE														:=996,
		STATE_ENABLESINGLEAXIS_DISABLE												        :=997);
    END_TYPE

    TYPE
		_FB_StopMaster :INT
		(STATE_STOP_MASTER_STOP																						:=1330);
    END_TYPE

    TYPE
        _FB_ResetAssiMaster :INT
		(STATE_RESET_ASSI_MASTER_RESET_MASTER_1														:=1372,
		STATE_RESET_ASSI_MASTER_RESET_BUFFER_2														:=1373,
		STATE_RESET_ASSI_MASTER_RESET_MASTER_2														:=1374);
    END_TYPE
    
    TYPE
        _FB_ResetMasterAxisBLM :INT
		(STATE_RESET_MASTER_AXIS_BLM_WAIT_PLCOPEN													:=1370,
		STATE_RESET_MASTER_AXIS_BLM_NO_PLCOPEN														:=1371);
    END_TYPE

    TYPE
        _Module_2Axes : INT
        (STATE_MODULE_2AXIS_LOOP_WAITING                                                             :=5670,
        STATE_MODULE_2AXIS_HOMING                                                                   :=5671,
        STATE_MODULE_2AXIS_RESET_AXIS1                                                              :=5672,
        STATE_MODULE_2AXIS_RESET_AXIS2                                                              :=5673,
        STATE_MODULE_2AXIS_WAITING_HOMING_AXIS1                                                      :=5674,
        STATE_MODULE_2AXIS_WAITING_HOMING_AXIS2                                                      :=5675);
    END_TYPE
     
    TYPE
        _EnumCamPositionMode : INT
        (BASIC := 16,
        ACTUAL := 7);
    END_TYPE

    TYPE
        _FB_VerificaConfiguraMacchinaDxSx :INT
		(STATE_VERIFICATION_CONFIG_DXSX_WAITING_END_RESTART_Y1									        :=1270,
		STATE_VERIFY_CONFIG_DXSX_WAIT_RESTART_SYSTEM										                :=1271,
		STATE_VERIFY_CONFIG_DXSX_RETURN_VALUE														    :=1272);
    END_TYPE

    TYPE
        _EnumYesNo : INT
        (YES:=173,
        NO:=91);
    END_TYPE

    TYPE
        _FB_MCP483 :INT
		(STATE_MCP483_CHANGE_SET_PARAMETERS_LOOP    :=5320,
		STATE_MCP483_CHANGE_SET_PARAMETRI_SET_1     :=5321,
		STATE_MCP483_CHANGE_SET_PARAMETRI_SET_2     :=5322,
		STATE_MCP483_SET_MCPO_REPOS                 :=5323, //5
		STATE_MCP483_DH4010_CHANGE_OPMODE_JOGMODE   :=5324,
		STATE_MCP483_SAVE                  :=5325,
		STATE_MCP483_READ_ACTIVE_DEVICE_1           :=5326,
		STATE_MCP483_READ_ACTIVE_DEVICE_2           :=5327,
		STATE_MCP483_SET_ACTIVE_DEVICE              :=5328,//
		STATE_MCP483_CHANGE_OPMODE_REF              :=5329,
		STATE_MCP483_CHANGE_OPMODE_MDA              :=5330,
		STATE_MCP483_CHANGE_OPMODE_AUTO             :=5331,
		STATE_MCP483_CHANGE_OPMODE_JOG              :=5332,
		STATE_MCP483_NCSTOP                         :=5333, //9
		STATE_MCP483_RESET_CHANGE_OPMODE_JOG        :=5334,
		STATE_MCP483_RESTART_ENABLEAX               :=5335,
		STATE_MCP483_STOP_ASSI_MASTER               :=5336, //8
		STATE_MCP483_READ_OVR_2                     :=5337, //7
		STATE_MCP483_LOOP_HOMING                    :=5338, //2
		STATE_MCP483_LOOP_HOMING_ABSOLUTE           :=5339,
		STATE_MCP483_CHECK_HOMING_ERRORS            :=5340, //6
		STATE_MCP483_ELIMINATE_HOMING_COMMANDS      :=5341,
		STATE_MCP483_GEST_JOG                       :=5342,
		STATE_MCP483_GEST_JOG_SET_CONDITION_MOVE_AXIS    :=5343,
		STATE_MCP483_GEST_JOG_ENABLE_AXIS_BLM       :=5344,
		STATE_MCP483_GEST_JOG_CHECK_ENABLE_AXIS_BLM :=5345,
		STATE_MCP483_GEST_JOG_ENABLE_COUPLE_W1_GENERIC :=5346,
		STATE_MCP483_GEST_JOG_ENABLE_PAIR_W1_ELECTXL   :=5347,
		STATE_MCP483_GEST_JOG_DISABLE_COUPLE_W1_ETURN  :=5348,
        STATE_MCP483_GEST_JOG_JOG_ASSE                 :=5349,
        STATE_MCP483_GEST_JOG_LOOP_AXIS_RESET_MOTION   :=5350,
        STATE_MCP483_GEST_JOG_RESET_MOTION_RESET_AXIS  :=5351,
        STATE_MCP483_GEST_JOG_SET_CONDITION_MOVE_DEVICE       :=5352,
        STATE_MCP483_GEST_JOG_MOVE_DEVICE              :=5353,
        STATE_MCP483_GEST_JOG_MOVE_DEVICE_REST         :=5354,
        STATE_MCP483_GEST_JOG_TASK_ACTIVE              :=5355,
        STATE_MCP483_GEST_JOG_LOOP_STEPS_ELIMINATE_COMMANDS :=5356, //10
        STATE_MCP483_GEST_JOG_LOOP_AXES_ELIMINATES_TAIL_COMMANDS_LOOP :=5357, //1
        STATE_MCP483_GEST_JOG_ELIMINATES_CONTROL_LOOP :=5358,
        STATE_MCP483_GEST_HOME_ASSI_REALI             :=5359, //3
        STATE_MCP483_GEST_HOME_HOMING_ABSOLUTE        :=5360,
        STATE_MCP483_GEST_HOME_INCREMENTAL_SET_MOVE_AXIS:=5361,
        STATE_MCP483_WATCHDOG_STELIA :=5362); //4
    END_TYPE

    TYPE
    	_FB_Read_ActiveDevice : INT
		(STATE_READ_ACTIVE_DEVICE_OPERAZIONE_COMPLESSA											:=1420,
		STATE_READ_ACTIVE_DEVICE_RESET_AXIS														:=1421,
		STATE_READ_ACTIVE_DEVICE_ELIMINA_CODA_COMANDI											:=1422,
		STATE_READ_ACTIVE_DEVICE_STOPCOND														:=1423,
		STATE_READ_ACTIVE_DEVICE_OPERAZIONE_COMPLESSA_2										    :=1424,
		STATE_READ_ACTIVE_DEVICE_ELIMINA_CODA_COMANDI_2										    :=1425,
		STATE_READ_ACTIVE_DEVICE_RETURN															:=1426);
    END_TYPE

    TYPE
        _FB_Change_OpMode :INT
		(STATE_CHANGE_OPMODE_ELIMINA_CODA_COMANDI													:=1450,
		STATE_CHANGE_OPMODE_RESET_AL																:=1451,
		STATE_CHANGE_OPMODE_ARRESTA_MOTORI															:=1452,
		STATE_CHANGE_OPMODE_POST_ARRESTO															:=1453,
		STATE_CHANGE_OPMODE_JOGMODE																	:=1454,
		STATE_CHANGE_OPMODE_REFMODE																	:=1455,
		STATE_CHANGE_OPMODE_MDAMODE																	:=1456,
		STATE_CHANGE_OPMODE_AUTOMODE																:=1457,
		STATE_CHANGE_OPMODE_LOOP_ASSI_JOGMODE														:=1458,
		STATE_CHANGE_OPMODE_RESET_AXIS_JOGMODE														:=1459,
		STATE_CHANGE_OPMODE_DISABLE_AXIS_JOGMODE													:=1460);
    
    END_TYPE

    TYPE
        _FB_ResetAL : INT
		(STATE_RESET_AL_RESET_USCITE_CARICO									:=617,
		STATE_RESET_AL_AL5_SOLLEVATORE										:=618,
		STATE_RESET_AL_AL5_CATENE											:=619);
    END_TYPE

    TYPE
        _FB_AL5CATENE : INT
		(STATE_AL5_CATENE_LAVORO																:=1430,
		STATE_AL5_CATENE_MS0017_CATENA_DX													    :=1431,
		STATE_AL5_CATENE_MS0017_CATENA_SX														:=1432,
		STATE_AL5_CATENE_MS0017_CATENA_3													    :=1433,
		STATE_AL5_CATENE_CATENA_DX_SINCRONIZZATE												:=1434,
		STATE_AL5_CATENE_NO_SINCRONIZZATE														:=1435,
		STATE_AL5_CATENE_CATENA_SX_SINCRONIZZATE												:=1436,
		STATE_AL5_CATENE_NO_SYNC_CATENA_DX_ARRESTO												:=1437,
		STATE_AL5_CATENE_NO_SYNC_CATENA_SX_ARRESTO												:=1438,
		STATE_AL5_CATENE_NO_SYNC_CATENA_SX_LAVORO_ATTESA_SYNC							        :=1439,
		STATE_AL5_CATENE_NO_SYNC_CATENA_SX_ARRESTO_SYNC										    :=1440,
		STATE_AL5_CATENE_MANTIENI_CATENE_FERME													:=1441,
		STATE_AL5_CATENE_NO_SYNC_CATENA_DX_LAVORO_ATTESA_SYNC							        :=1442,
		STATE_AL5_CATENE_NO_SYNC_CATENA_DX_ARRESTO_SYNC										    :=1443,
		STATE_AL5_CATENE_CATENA_SX_LAVORO_POST_SYNC												:=1444,
		STATE_AL5_CATENE_CATENA_DX_ALTRI_STATI													:=1445,
		STATE_AL5_CATENE_CATENA_SX_ALTRI_STATI													:=1446);
    END_TYPE

    TYPE
        _FB_Homing_Absolute : INT
		(STATE_HOMING_ABSOLUTE_HOME_SIMULATED_AXIS													:=1300,
		STATE_HOMING_ABSOLUTE_HOME_ACTIVATED_AXIS													:=1301,
		STATE_HOMING_ABSOLUTE_WAIT_PLCOPEN_RESET													:=1302,
		STATE_HOMING_ABSOLUTE_READ_NUMERO_SERIE_ENCODER										        :=1303,
		STATE_HOMING_ABSOLUTE_SET_VALUES															:=1304);
    END_TYPE

    TYPE
        _FB_AL5_Sollevatore :INT
		(STATE_SOLLEVATORE_WAIT_SOLLEVATORE									        :=612,
		STATE_SOLLEVATORE_VERIFICA_STOP											    :=613);
    END_TYPE

    TYPE
    	_FB_WaitDisp : INT
		(STATE_ATTESA_DISPOSITIVO														:=603);
    END_TYPE

    TYPE
        _FB_Jog_Asse : INT
		(STATE_JOG_ASSE_LOOP_RESET_ASSI																		:=1480,
		STATE_JOG_ASSE_RESET_ASSE																			:=1481,
		STATE_JOG_ASSE_JOG_PURO_POSITIVO																	:=1482,
		STATE_JOG_ASSE_POS_ASSE_POSITIVE																	:=1483,
		STATE_JOG_ASSE_JOG_PURO_NEGATIVO																	:=1484,
		STATE_JOG_ASSE_POS_ASSE_NEGATIVE																	:=1485,
		STATE_JOG_ASSE_RILASCIO_PULSANTI																	:=1486);
    END_TYPE

    TYPE
        _FB_SpegneAssi :INT
		(STATE_SPEGNE_ASSI_LOOP_ASSI																		:=3430,
		STATE_SPEGNE_ASSI_DISABLE_AXIS																		:=3431);
    END_TYPE

    TYPE
        _FB_Set_Condizione_Move_Asse :INT
		(STATE_SET_CONDIZIONE_MOVE_ASSE_PRESSIONE_DIREZIONE								:=1470,
		STATE_SET_CONDIZIONE_MOVE_ASSE_WAIT_TIME_500									:=1471,
		STATE_SET_CONDIZIONE_MOVE_ASSE_DH4010_RILASCIO_DIREZIONE					    :=1472);

    END_TYPE

    TYPE
		_FB_AttesaFineResetAssi :INT
		(STATE_ATTESA_FINE_RESET_ASSI_CICLO_ASSI							:=707,
		STATE_WAIT_ATTESA_AZIONAMENTO									    :=708);
    END_TYPE

    TYPE
        _FB_PosAsse : INT
		(STATE_POSASSE_PLCOPEN															:=705,
		STATE_POSASSE_NOPLCOPEN															:=706);
    END_TYPE

    TYPE
        _FB_ResetAllCamme : INT
		(STATE_RESET_ALL_CAMME_STOP_MASTER																	:=1335,
		STATE_RESET_ALL_CAMME_LOOP_ASSI																		:=1336,
		STATE_RESET_ALL_CAMME_DISABILITA_CAMMA														        :=1337,
		STATE_RESET_ALL_CAMME_WAIT_ASSI_ACTIVE														        :=1338,
		STATE_RESET_ALL_CAMME_WAIT_ASSE																		:=1339);
    END_TYPE

    TYPE
        _FB_DisabilitaCamma : INT
		(STATE_DISABILITA_CAMMA_RESET_MOTION																:=1325,
		STATE_DISABILITA_CAMMA_DISABLE_CAMMING														        :=1326,
		STATE_DISABILITA_CAMMA_SGANCIO_CAMMA															    :=1327);
    END_TYPE

    TYPE
        _EnumPositioningMode : INT
        (ABSOLUTE := 1,
        RELATIVE  := 115);
    END_TYPE

    TYPE
        INC_Enum   : LREAL
    (inc_0 := 0,
    inc_1  := 0.001,
    inc_2  := 0.01,
    inc_3  := 0.1,
    inc_4  := 1,
    inc_5  := 10);
    END_TYPE

    // TYPE
    //     INC_Array                                             :ARRAY[0..5] OF INC_Enum;
    // END_TYPE

    // TYPE
    //     INC                                                     : (INC_Array);
    // END_TYPE

    END_NAMESPACE