USING Simatic.Ax.LAcycCom;
USING Siemens.Simatic.S71500.Tasks; 
USING Siemens.Simatic.S71500.MotionControl.Native;
USING EnableAxFunction;
USING _FC_FB;
USING Retain_Namespase;
USING _STATE_CONST;
USING _CONST_VAR_DEF;
USING _FC_IO_FC;
USING FC_N;
USING _PLC_OPEN;
USING _AXIS_CTRL_CLASS;


NAMESPACE _TYPE

TYPE

ax:STRUCT
	Nome															:STRING;//// ?  
	PosAx															:DB_ANY;//POSAXIS; /////DB_ANY TO_SYNchronous Axis
	SincroAsse 												        :DB_ANY;//FOLLOWINGOBJECTTYPE;///DB_ANY; TO_SynchronousAxis
	MasterName											            :DB_ANY;////?
	MasterAxis												        :DB_ANY;////DB_ANY; TO_PositioningAxis
	Camma				   										    :DB_ANY;///DB_ANY; TO_Cam
	QuotaReale												        :LREAL;
	StatoCamming											        :BOOL:=FALSE;	//True indica che l^asse deve essere ricollegato in camma 
																									//alla ripartenza del ciclo automatico
	Request_Restart_Oggetto_Simotion                                :BOOL;/// MC_RESTART
	Request_Restart_Oggetto_Sinamics                                :BOOL;///MC_RESTART
	Temperatura_Motore                                              :REAL;
	ColonnaAsseVgpTabStati          	                            :SINT:=SINT#-1; //Numero colonna dell^asse nella TabStati spedita da VGP
	NumeroAsseVgp         			                                :SINT:=SINT#-1; //Numero asse secondo la numerazione di VGP
	FattoreCoppiaMax          	                                    :REAL; //limite di coppia massima del motore/16384
	CoppiaNominale          		                                :REAL; //Coppia nominale del motore
	CoppiaM0          					                            :REAL; //Coppia nominale a velocità nulla del motore
	CoppiaTargaMax		                                            :REAL; //Coppia massima di targa del motore
	CoppiaPercentuale         	                                    :INT;
	CoppiaIstantanea	                                            :REAL;
	Somma_Coppia_Misurata							                :REAL; //Sommatoria delle coppie misurate in fase di monitoring
	Contatore_Coppia_Misurata 				                        :REAL; //Contatore dei campioni di coppia misurati in fase di monitoring
	Carico				  									        :REAL;			  
	VelNominale			 									        :LREAL;
	DistanceToGo											        :REAL;
	Coeff_Vel_torque									            :REAL; 
	Homing_al_restart	   							                :BOOL:=TRUE;//Forza l^homing degli assi incrementali a seguito di uno STOP/RUN 
	Coppia_Massima_Misurata 					                    :REAL; //Valore massimo di coppia dall^accensione della macchina									   
	Coppia_Massima_Misurata_in_ciclo 	                            :REAL; //Valore massimo di coppia dall^ultimo passaggio in AUTO

	Coppia_Media_Misurata 	 					                    :REAL;
	Temperatura_Massima_Misurata  		                            :INT; //Valore massimo di temperatura dall^accensione della macchina									   
	fattore_carico_cuscinetti_temp		                            :LREAL:=0;
	somma_p_temp			        				                :LREAL:=0;
	somma_n_temp			        				                :LREAL:=0;
	NumSerieEncoder		        				                    :LREAL:=0;
	Tipo_Encoder 			        				                :SINT:=SINT#0; //1=ASSOLUTO 2=INCREMENTALE
	Homed                     				                        :BOOL; //TRUE se fatto HOME (valido anche per encoder assoluti)//// MC_Home
	Agganciato_Tabella_Stati  				                        :BOOL:=TRUE;//Se TRUE viene controllato in tabella stati altrimenti no
	ma_asse_installato_shadow					                    :SINT:=SINT#0;   //0=Asse non disabilitabile
																								//1=Asse disabilitabile, stato abilitato
																								//2=Asse disabilitabile, stato disabilitato
	ma_Asse_Abilitato_shadow					                    :BOOL:=TRUE;
	//KW_ora														    :kwh;
	DatiSafety												        :Sicurezza;
	TemperaturaMaxPossibile						                    :USINT:=USINT#130;
	(*{
	#ifdef CARICO_SCARICO_PARALLELI
	}*)
		Agganciato_Tabella_Stati_Appo                               :BOOL;	// Se TRUE viene controllato in tabella stati altrimenti no
	(*{
	#endif	//Fine CARICO_SCARICO_PARALLELI
	}*)
	SovraccarioI2T										            :USINT;	//Percentuale di incremento di sovraccarico I2T
																						//nell^ultimo pezzo eseguito
	IsSiemensAxis											        :BOOL:=TRUE;
	
	//strutture che contengono i parametri di chiamata dei vari blocchi PLCOpen
	MC_Axes                          : AXIS_CTRL;
	// mcMoveAbsoluteInfo								                : MC_MoveAbsolute;
	// mcMoveRelativeInfo								                : MC_MoveRelative;
	// mcEnableAxisInfo									            : MC_Power;
	// mcResetAxisInfo										            : MC_Reset;
	// mcHomeAxisInfo										            : MC_Home;	
	// mcStopAxis										            : MC_Stop;
	// mcJogAxisInfo											        : MC_MoveJog;
	// mcCamInInfo												        : MC_CamIn;
	// mcCamOutInfo											        : MC_CamOut;
	// mcMoveVelocityInfo								                : MC_MoveVelocity;
END_STRUCT;

Sicurezza: STRUCT
	r9398							:ARRAY[0..1] OF UDINT;
	r9728							:ARRAY[0..2] OF UDINT;
	r9798							:UDINT;
	r9898							:UDINT;
END_STRUCT;

    AxisParameters : STRUCT
	//appoggio per poter essere lette dal background
	max_ax_velo									                    :REAL:=REAL#1.0;	// 32000   mm/min, rev/min maximum axis velocity LREAL NEW CONF
	MA_max_ax_accel							                        :REAL:=REAL#1.0;	// 32300   m/s², rev/s² maximum axis acceleration LREAL NEW CONF
	ma_num_encs									                    :SINT:=SINT#0;	// 30200   Numero di encoder presenti per l^asse (sempre >0)
	MA_JOG_VELO_RAPID						                        :USINT:=USINT#40;	// 32010   % Rapid traverse in jog mode LREAL RESET
	MA_JOG_VELO									                    :USINT:=USINT#20;	// 32020   % Jog axis velocity LREAL RESET
	MA_JOG_ACC									                    :USINT:=USINT#50;	// 32030   % Jog axis acceleration
	MA_CAM_DIR_IS_MINUS					                            :REAL;	// 34010   - Approach reference point in minus direction BOOLEAN RESET
	MA_REFP_VELO_SEARCH_CAM			                                :REAL;	// 34020   mm/min, rev/min Reference point approach velocity LREAL RESET
	MA_REFP_VELO_SEARCH_MARKER	                                    :REAL;	// 34040   mm/min, rev/min Creep velocity LREAL RESET
	MA_REFP_MOVE_FROM_CAM				                            :REAL;	// 34042   mm moviemnto per allontanarsi dalla fine della camma prima della ricerca del marker 
	MA_REFP_VELO_POS						                        :REAL;	// 34070   mm/min, rev/min Reference point positioning velocity LREAL RESET
	MA_REFP_MOVE_DIST						                        :REAL;	// 34080   mm, degrees Reference point distance LREAL NEW CONF
	MA_REFP_CAM_MARKER_DIST			                                :REAL;	// 34093   mm, degrees Reference cam/reference mark distance LREAL POWER ON
	MA_Refp_Set_Pos							                        :REAL;	// 34100   mm, degrees Reference point value/target point for distancecoded system
	
	MA_Refp_Set_Pos_BLM					                            :REAL;	// 34110   Quota index asse originale BLM
	(*{
	#ifdef DH4010
	}*)
	MA_Spedizione_BLM					                            :REAL:=-1*BIG_VALUE_REAL#MY_BIG_VALUE_REAL;	// 34111   Quota per spedizione macchina   // ttakeback
	(*{
	#endif	//Fine dh4010
	}*)
	MA_Tolerance								                    :REAL:=REAL#0.1;	 // 36010	mm, tolleranza per fine movimento
	MA_POS_WIN_TOL_TIME					                            :REAL:=REAL#1.0;	 // 36020   sec, 
	MA_standstill_pos_tol				                            :REAL:=REAL#2.0;	// 36030   mm, degrees Standstill tolerance LREAL NEW CONF
	MA_standstill_DELAY_TIME		                                :REAL:=REAL#0.4;   // 36040   sec, Standstill delay time
	Ma_Pos_Limit_Minus_Attivo		                                :REAL;  	 // Limite  minimo asse utilizzando il 36100 o 36100-1
	Ma_Pos_Limit_Minus					                            :REAL;	// 36100   limite minimo asse impostato dal paraetro 36100 
	//Ma_Pos_Limit_Minus_1				                            :REAL:=-MY_BIG_VALUE_REAL;	// 36100-1 limite minimo asse impostato dal paraetro 36100-1 
	Ma_Pos_Limit_Minus2					                            :REAL;	// 36120   limite minimo asse in monitoring
	Ma_Pos_Limit_Plus_Attivo		                                :REAL;  	 // Limite  massimo asse utilizzando il 36110 o 36110-1
	Ma_Pos_Limit_Plus						                        :REAL;	// 36110   limite massimo asse impostato dal paraetro 36110 
	//Ma_Pos_Limit_Plus_1					                            :REAL:=-MY_BIG_VALUE_REAL	;	// 36110-1 limite massimo asse impostato dal paraetro 36110-1 
	Ma_Pos_Limit_Plus2					                            :REAL;	// 36130   limite massimo asse in monitoring
	MA_MDA_PLUS_POSITION				                            :REAL;	// 39020  Posizione positiva per MDA
	MA_MDA_MINUS_POSITION				                            :REAL;	// 39010  Posizione negativa per MDA
	MA_MDA_DELAY_TIME						                        :LREAL:=0;	// 39030  Tempo di attesa fra due movimenti in MDA
	MA_MDA_VELOCITY							                        :USINT:=USINT#100;	// 39040  Velocità per movimenti MDA
	MA_MDA_ACCEL								                    :USINT:=USINT#100;  // 39045  Accelereazione per movimenti MDA
	MA_MDA_TORQUE								                    :USINT:=USINT#100;  // 39050  Limitazione di coppia in modalità MDA
	MA_JOG_TARGET			 					                    :LREAL;	// 39060  Quota target per JOG
	//MA_STATO_ENCODER		  			                            : EnumAxisEncoderIdentification	;	//  39070
	MA_LIMITE_ATTIVO		  			                            :INT;   
	AZ_P1441									                    :REAL:=40;//-REAL#99999.0;//-MY_B_Value#MY_BIG_VALUE_REAL;
	AZ_P1460				  					                    :REAL:=-REAL#99999.0;//-MY_B_Value#MY_BIG_VALUE_REAL;
	AZ_P1462				  					                    :REAL:=-REAL#99999.0;//-MY_BIG_VALUE_REAL;
	AZ_P1715									                    :REAL:=-REAL#99999.0;//-MY_BIG_VALUE_REAL;
	AZ_P1717				  					                    :REAL:=-REAL#99999.0;//-MY_BIG_VALUE_REAL;
	AZ_P1520									                    :REAL;	//Coppia massima richiesta da BlmMotion in senso positivo
	AZ_P1521									                    :REAL;	//Coppia massima richiesta da BlmMotion in senso negativo
	az_r1538										                :REAL;	//Coppia massima realmente attiva in senso positivo
	az_r1539										                :REAL;	//Coppia massima realmente attiva in senso negativo
	AZ_P2175									                    :REAL;
	AZ_P2177									                    :REAL;
	MA_TORQUE_CALIBRAZIONE_RICERCA_TACCA	                        :INT:=10;	//Coppia per la ricerca della tacca durante la calibrazione
	Ma_Torque_Calibrazione_Tenuta_Tubo		                        :INT:=40;	//Coppia di tenuta del tubo da usare durante le
																									//operazioni di scarico con pedale/robot
	MA_VEL_CALIBRAZIONE_RICERCA_TACCA			                    :INT:=10;	//Velocità dell^asse durante la RICERCA TACCA
	MA_ACC_CALIBRAZIONE_RICERCA_TACCA			                    :INT:=30;	//Accelerazione/decelerazione dell^asse durante la RICERCA TACCA
	MA_CORSA_CALIBRAZIONE_USCITA_TACCA	                            :INT:=50;	//Corsa di apertura una volta trovata la tacca
	MA_VEL_CALIBRAZIONE				 						        :INT:=10; //Velocità dell^asse durante la calibrazione 
	MA_ACC_CALIBRAZIONE	 			  					            :INT:=30; //Accelerazione/decelerazione dell^asse durante la calibrazione
	MA_AGGIUSTAGGIO_POS_CALIBRAZIONE			                    :REAL:=0; //Serve per calcolare l^offset di calibrazione da passare a VGP
																									//-0.1 per la morsa per aumentare la spinta
																									//+0.7 per i rulli per diminuire la spinta
	ma_fattore_coppia_calibrazione                                  :REAL:=REAL#0.6;	//Fattore per riduzione coppia massima durante calibrazione											
	ma_max_differenza		 					                    :INT;       //Massima differenza fra la quota teorica e reale
	MA_EXTRA_COPPIA_MONITORING 		                                :INT:=20; // tolleranza % extra coppia prima dell^allarme
	ma_max_temperatura_raggiunta	                                :INT:=0;
	ma_max_coppia_raggiunta 			                            :INT:=0;
	ma_corsa_totale_asse 					                        :LREAL:=0;
	ma_corsa_parziale_asse 				                            :LREAL:=0;
	ma_fattore_carico_cuscinetti	                                :LREAL:=0;
	ma_somma_p 										                :LREAL:=0;
	ma_somma_n 										                :LREAL:=0;
	ma_num_serie_encoder 					                        :LREAL:=0;
	ma_ReverseringCounter					                        :UDINT:=UDINT#0;
	ma_LimiteSovraccaricoI2T			                            :USINT:=USINT#100;	//Limite massimo sovraccarico I2T quando il valore
																							//calcolato al termine del precedente pezzo non è
																							//ancora disponibile. 
																							//Con 100 il test viene disabilitato.
	(*{
	#ifdef EMULAZIONE
	}*)
	ma_verifica_num_serie_encoder	                            :BOOL:=FALSE;
	(*{
	#else	//Fine EMULAZIONE
	}
		ma_verifica_num_serie_encoder	                            :BOOL:=TRUE;
	{
	#endif	//Fine NO EMULAZIONE
	}*)
	
	ma_ax_motion_dir		  				                        :INT	:=0;	//+1=nessuna inversione conteggio e riferimento
																						//-1=inversione conteggio e riferimento
																						// 0=errore non può funzionare
	ma_stato_index								                    :SINT	:=SINT#2;	//0=Index non eseguito
																						//1=Index in corso di esecuzione
																						//2=Index eseguito
	ma_asse_installato						                        :SINT	:=SINT#0;	//0=Asse non disabilitabile
																						//1=Asse disabilitabile, stato abilitato
																						//2=Asse disabilitabile, stato disabilitato
	ma_pos_win_vel								                    :REAL	:=1;	//Percentuale velocità massima per considerare l^asse fermo
	ma_Temperatura_Assorbimento		                                :INT	:=0;	//39300 Temperatura minima motore per ciclo di assorbimento
	ma_Coppia_Accelerazione_Rif		                                :REAL	:=0;	//39301 Coppia media di riferimento durante accelerazione
	ma_Coppia_VelCostante_Rif			                            :REAL	:=0;	//39302 Coppia media di riferimento durante velocità costante
	ma_Riposo_Assorbimento				                            :REAL	:=0;	//39303 Quota di riposo al termine del ciclo di misura assorbimento
	ma_Coppia_Accelerazione				                            :REAL	:=0;	//39004 Coppia media misurata durante accelerazione
	ma_Coppia_VelCostante					                        :REAL	:=0;	//39005 Coppia media misurata durante velocità costante
	ma_Esito_Assorbimento					                        :BOOL	:=FALSE;//Esito della verifica dell^assorbimento*)
	ma_Asse_Abilitato							                    :BOOL	:=TRUE;	//False se non e^ presente la CU a cui l^azionamento e^ collegato
	ma_Confronta_Asse_Abilitato		                                :BOOL	:=FALSE;//TRUE se si deve confrontare "ma_Asse_Abilitato" con 
																							//"ma_Asse_Abilitato_shadow". Serve solo per gli assi in cui
																							//al trasferimento del pezzo, viene in qualche modo impostato
																							//il valore "ma_Asse_Abilitato_shadow" (esempio A1 e A2)
	ma_Asse_SOS										               :BOOL :=FALSE;//True se l^asse deve mantenere la coppia durante la violazione
																							//della barriera di sicurezza, perchè usa la funzione SOS della
																							//Safety estesa
	ma_Asse_Da_Calibrare					                       :BOOL;				//True se l^asse deve essere calibrato	
	ma_AsseSpegnibile							                    :BOOL:=FALSE;	//True se l^asse può essere spento se durante il pezzo non viene movimentato
	ma_AsseInVGP									                :BOOL;				//True se l^asse è presente in VGP e quindi impostabile			
	ma_Vel_Safety_COMFORT1				                            :REAL:=35;		//Velocita di sicurezza COMFORT1
	ma_Vel_Safety_COMFORT3				                            :REAL;				//Velocita di sicurezza COMFORT3
	ma_Asse_Tenuta_Tubo						                        :BOOL;				//True se l^asse può tenere il tubo e quindi potrebbe impedire 
																							//la disattivazione dell^alimentatore per il risparmio energetico
	ma_NumeroTabellaStati					                        :SINT:=SINT#-1;
	//ma_VelSicuraPrimoStart				                            :REAL:=-MY_BIG_VALUE_REAL;
	ma_LetturaMotoreDriveCliq			                            :USINT:=USINT#0;		//1 attiva la lettura del motore in drive-cliq p300=[1000]
	ma_Tolleranza_Sincronismo			                            :REAL:=0;			//Tolleranza per poter sincronizare l^asse con il master in funzione della sua camma*)
END_STRUCT;

	
	Produzione:STRUCT 
	JobID															:INT:=J_ID_RESET#JOB_ID_RESET;
	JobState														:INT:=0;
	MachineState													: ProduzioneStatoMacchina;
	Pezzo															:ProduzionePezzo;
	Esecuzione_Job_Scarti											:BOOL;									//Messa TRUE da VGP3D ed indica che è in corso la produzione degli scarti
	JobGUID															:STRING[38];	//Guid del job
	JobName 														:STRING[64];	//Nome del job (solo pezzo seguente)
END_STRUCT;

ProduzioneStatoMacchina:STRUCT
	MachineState													:INT;
	Lampeggiante													:INT;
	ByteHight														:WORD;
	ByteLow															:WORD;
	DateTime														:LDATE_AND_TIME ;	
END_STRUCT;

ProduzionePezzo:STRUCT
	NumProgrammi													:UINT:=UINT#0;
	Dati															:ARRAY[UINT#1.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS-UINT#1] OF ProduzionePezzoBis;
END_STRUCT;

ProduzionePezzoBis:STRUCT
	ID																:INT:= _P_ID_RESET#P_ID_RESET;
	NumFatti														:DINT;	
	Guid															:STRING[38];	//Guid del pezzo
	Guid_shadow														:STRING[38];	//Guid del pezzo
END_STRUCT;

Digital_Output:STRUCT
	Port															:INT;   //NUMERO BYTE (Scheda) A PARTIRE DA 0
	Bit				 												:SINT;  //NUMERO BIT (posizione dell^Ingresso/Uscita sulla scheda) PARTIRE DA 0
	Enable_Forzatura												:BOOL;  //TRUE per abilitare la forzatura
	Stato_Forzatura													:BOOL;  //Stato della forzatura 
	Stato			   												:BOOL;  //Stato dell^ingresso/uscita (TRUE o FALSE)
	StatoPreForzatura												:SINT;  //Stato dell^uscita prima della forzatura, serve per 
																			//ripristinare lo stato originale prima della forzatura
																			//0=Stato ripristinato 1=Stato ON -1=Stato OFF
	StatoPreStopCiclo												:BOOL:=FALSE;	//Stato uscita prima dello stop ciclo	
	AbilGestStopCiclo												:BOOL:=FALSE;	//Abilitazione gestione stop ciclo
	Inversione														:BOOL:=FALSE;	//True=inversione logica uscita fisica
END_STRUCT; 

Digital_Input:STRUCT
	Port															:INT;   			//NUMERO BYTE (Scheda) A PARTIRE DA 0
	Bit				 												:SINT;  			//NUMERO BIT (posizione dell^Ingresso/Uscita sulla scheda) PARTIRE DA 0
	Enable_Forzatura												:BOOL;  			//TRUE per abilitare la forzatura
	Stato_Forzatura													:BOOL;  			//Stato della forzatura 
	Stato			   												:BOOL;  			//Stato dell^ingresso/uscita (TRUE o FALSE)
	StatoPreForzatura												:SINT;  			//Stato dell^uscita prima della forzatura, serve per 
																	//ripristinare lo stato originale prima della forzatura
																	//0=Stato ripristinato 1=Stato ON -1=Stato OFF
	Inversione														:BOOL:=FALSE;	//True=inversione logica ingressi fisico
END_STRUCT;

Trace:STRUCT
	 Stato 															:SINT;	//(0= Disattivo, >=1 Attivo)
	 Asse 															:INT;
	 DataOra 														:DATE_AND_TIME; 		//Ora set
	 Codice															:INT;		//Codice del messaggio
END_STRUCT;

//////////////////////////////////////////////////////////////////////////////////////////////////////
//Informazioni su come accedere alla tabella generale dei polinomi
DatiPolinomi:STRUCT
	NumeroPolinomi:INT;
	RigaInizio		:DINT;
	RigaFine		  :DINT;
END_STRUCT; 

Programma: STRUCT
	ProdOrd															:STRING;
	DownloadNumber 													:INT;
	OperationNumber													:INT;
	MaterialCode					 								:STRING;
	InfoPolinomi													:ARRAY [0.._CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF DatiPolinomi;
	ProgrammaValido													:BOOL;	// TRUE se il programma può essere mandato in automatico  
	TipoTubo 														:REAL;	// => Tipo tubo (TONDO=0,ELLISSE=1,OVALE=2,QUADRO=3,
	TempoCicloVGP													:INT;		//Tempo ciclo calcolata da VGP3D
	(*{
	#ifdef DH4010
	}*)
		TabStatiInizio												:ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;
		TabStatiFine												:ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;
		posMasterDx  												:LREAL;
		posMasterSx  												:LREAL;
		NumStati 													:ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;	//Numero stati presenti nella tabella stati
		sequenza    												:ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;	//Numero di sequenza attiva
		maxsequenze    												:ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;  //Numero totale di sequenze
		maxsequenze_export											:ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;  //Numero totale di sequenze
		stato_corrente 												:ARRAY[TABLE#TABLE_LX..TABLE#TABLE_RX] OF INT;	//Stato corrente della tabella stati
	(*{
	#else	//Fine DH4010
	}*)
		(*BlmMxCurve 													:ARRAY[0..MAXSEQ-1] OF AngoloCoppiaX1; //Tabelle per modulazione coppia carro X durante la curvatura
		//TabStatiInizio												:INT;
		//TabStatiFine												:INT; *)
		posMaster 												:LREAL;
		//Numstati 													:INT;	//Numero stati presenti nella tabella stati
		//sequenza													:INT;	//Numero di sequenza attiva
		//maxsequenze												:INT;	//Numero totale di sequenze
		//maxsequenze_export										:INT;	//Numero totale di sequenze
		//stato_corrente											:INT;	//Stato corrente della tabella stati
		(*RigaInizioPrimaCurva										:INT;	//Numero di riga della tabella stati dove inizia la prima curva
		RigaFinePrimaCurva											:INT;	//Numero di riga della tabella stati dove termina la prima curva
		RigaFineUltimaCurva											:INT;	//Numero di riga della tabella stati dove inizia l^ultima curva
	(*{
	#endif	//Fine NO DH4010
	}*)
		Diameter_Tube 												:REAL;
		Diameter_Tube_B 											:REAL;
		Diameter_Tube_Section										:USINT;
		TipoCarico													:SINT;	//0=Carico manuale
																			//1=Carico robot standard
																			//2=Carico con AL
																			//3=Carico robot che preleva tubo da AL
																			//4=Carico da LT722
		TipoScarico													:SINT;	//0=Scarico manuale
																				//1=Scarico robot
		UsoRadar_Pezzo 												:INT;
		UsoRadar_Manuale 											:INT;
		PedaleUltimaRiga											:BOOL;	//Abilitazione pedale apertura pinza ultima riga, solo per terminali
																			//lunghi, Z=0 a prima sequenza (verifiche operate da VGP3D) e carico
																			//nuovo tubo insieme allo scarico di quello eseguito
		seq_registrate 												:INT;		// =0 significa nessun monitoraggio
		//Monitor_Coppie 												:ARRAY[0..MAXSEQ-1] OF CoppieAssi;
		UtPrimaSequenza												:INT:=0;				//Utensile prima sequenza per carico tubo Aerolia
		//PosX1LoaderFI												:LREAL:=MY_BIG_VALUE_REAL;	//Posizione assoluta X1 fuori ingombri per carico con loader										   
		LunghezzaTuboEsecuzione										:REAL;
		LunghezzaTuboTeorica										:REAL;
		SoluzioneTelecameraDalsa									:INT;		
		TipoTelecamera												:INT;		//xxxx1 Sistema di misura lineare Keynence
																				//xxx1x Telecamera Dalsa
		TempoCarico													:REAL;	//Tempo di carico stimato da VGP	
		TempoScarico												:REAL;	//Tempo di carico stimato da VGP
		Spessore													:REAL;	//Spessore del tubo 
		PezzoPieno													:SINT;	//1 quando è pieno, 0 quando è cavo
	(*{
	#ifdef AL_BLM
	}*)
		(*DatiAL														:QuoteAL;
		AzzeramentoTuboLight										:BOOL;	//True se azzera il tubo con pinze AL aperte
		AzzSingolarizzBasso											:BOOL;	//True se azzera il tubo con singolarizzatore basso
		RicSaldRoland												:BOOL;	//True se nel pezzo occorre eseguire la ricerca saldatura Roland su AL5
		ProgrRoland													:SINT;	//Numero programma ricerca saldatura Roland
		RicSaldKeyence												:BOOL;	//True abilita la ricerca saldatura su AL5 con sensore Keyence
	(*{
	#endif	//Fine AL_BLM
	}
	{
	#ifdef ORBITALE
	}*)
		//TabLavorazione												:ARRAY[0..MAXLAVORAZIONI-1] OF RigaLavorazione;
		DatiLavorazione												:ARRAY[0.._MAX_TYPES_WORKS#MAX_TYPES_WORKS-USINT#1] OF LineDataProcessing;
    	ContatorePezziSenzaMisura									:INT;		//Numero di pezzi eseguiti senza aver misurato
		(*{
		#ifdef RUNNER
		}*)
      	(*IntervalloPezziDaMisurare									:INT;		//Intervallo di pezzi fra una misura e la successiva
		TipoTaglio4RUNNER											:INT;		//Tipo taglio del pezzo
																				//-1 = solo per curvatubi
																				// 0 = nessun taglio, errore
																				// 1 = taglio interno IPC
																				// 2 = taglio esterno (4RTL o 4RC)
			ModoAspo												:UINT;	//0=aspo motorizzato
																				//1=aspo in folle sempre
																				//2=aspo in folle in avanzamento, motorizzato in arretramento
			PosZ1SpaccoTubo											:LREAL;
			OffsetW3SpaccoTubo										:LREAL;
			OffsetW4SpaccoTubo										:LREAL;
			PerVelAccY1SpaccoTubo									:USINT;
			Uso4RFORM												:BOOL;	//True se 4RFORM è usato nel pezzo
			_4RUNNER_L_intestatura									:REAL;	
			_4RUNNER_PosY1Spacco									:REAL;
			_4RUNNER_Lubr_Taglio									:UINT;	//Frequenza lubrificazione durante tutti i tipi di taglio
			_4RUNNER_Lubr_IPC_Light									:UINT;	//Frequenza lubrificazione durante l^incisione ipc light
			_4RUNNER_Lubr_Foratura									:UINT;	//Frequenza lubrificazione durante la foratura
			_4RUNNER_MorsaIPC										:BOOL;	//True usa morsa IPC per spacco, False usa W4
			_4RUNNER_TempoIPC										:REAL;
			_4RUNNER_UT_Taglio										:INT;
			_4RUNNER_UT_Spacco										:INT;
			_4RUNNER_VelRotLama										:REAL;	//Velocità rpm rotazione lama di taglio
			_4RUNNER_VelU2Taglio									:SINT;	//Percentuale velocità U2 andata taglio RTL
			_4RUNNER_AllX1DopoTaglio								:REAL;	//Allontanamento X1 dopo taglio 4RTL/4RC
			_4RUNNER_U1SicurezzaY1									:REAL;	//Posizione U1 per muovere Y1 in sicurezza
			_4RUNNER_DiametroTCartella								:REAL;	//Diametro teorico cartella
			_4RUNNER_DiametroRCartella								:REAL;	//Diametro misurato cartella
			_4RUNNER_TollSupCartella								:REAL;	//Tolleranza superiore per misura diametro cartella
			_4RUNNER_TollInfCartella								:REAL;
			_4RUNNER_SpessoreTCartella								:REAL;	//Spessore teorico da inizio cartella a fine tubo
			_4RUNNER_DeltaCartellaTR								:REAL;	//Differenza fra la cartella teorica e quella misurata
			_4RUNNER_PosX1TCartella									:REAL;	//Posizionammento teorica cartella X1
			_4RUNNER_PosX1RCartella									:REAL;	//Posizionammento reale cartella X1
			_4RUNNER_DeltaTuboCartella								:REAL;	//Modifica lunghezza tubo dopo lavorazione 4RFORM
			_4RUNNER_CoppiaChiusuraW1								:INT;		//Percentuale coppia chiusura morsa in automatico
			_4RUNNER_Foratura_Carrello								:BOOL;
			_4RUNNER_PresenzaVibratore								:BOOL; 	//SF [25/01/2018]
			_4RUNNER_GuidaTuboConTaglio								:BOOL; //SF [11/06/2018]: abilita il ciclo Moino per aiutare l^infilaggio del tubo in pinza di scarico con l^ausilio
																				//del 4RC
			UsoSupportiTubo3RUNNER									:BOOL; 			//indica se nel programma sono attivati i supporti tubo per 3-RUNNER (cliente Carpigiani)
      		OffsetX1SpaccoIPC         								:REAL;
			OffsetW3Taglio4RC										:REAL;	//Posizione W3 durante il taglio 4RC
			OffsetW4Taglio4RC										:REAL;	//Posizione W4 durante il taglio 4RC
			GradiVelCost4RC											:INT;		//Gradi a velocità costante taglio 4RC
			SpessoreTaglio4RC										:REAL;
			SpessorePreTaglio4RC									:REAL;
			PassoDiscesaRotelle4RC									:REAL;	//Discesa rotelle 4RC per ogni giro del 4RC
			FlagScarSpezAsseW1										:BOOL;	//True se è attivo lo scarico spezzone tagliato in asse con W1 (U1=U2=0)
			OffsetU1Taglio4RTL4RC									:REAL;	//Offset rispetto alla posizione U1 di taglio (parametro 60 cmd 11)
			AbilRecAllungamentoIPC									:BOOL;	//True recupero allungamento stimato per lavorazioni IPC e IPC_LIGHT
																				//Il campo viene abilitato in VGP con il temp flag 8 bit 12
			Uso3RUNNER_Curvatubi									:BOOL;	//True se il pezzo della 3RUNNER è connsesso con la curvatubi
			RotellaFissaTaglio										:BOOL;
			ModoTaglio3RIPC											:INT;		//1=Base, 2=Con strappo, 3=Seguito da strappo
			SpessoreLight											:REAL;	//Spessore per lavorazione IPC
			CicloSpecialeIPC										:BOOL;	//True abilita il ciclo speciale delle preincisioni con taglio interno che 
																				//vengono eseguite solamente durante la fase di arretramento del tubo e non
																				//durante l^esecuzione della tabella stati
		(*{
		#endif	//Fine RUNNER
		}
		{
		#ifdef EFLEX_DH4010 
		}*)
			(*UsoSmussatore											:BOOL;
			LunghezzaTuboCorretta									:REAL;
			SmussaturaPezzoSingolo									:BOOL;	//True abilita la smussatura per singolo pezzo non viene eseguita al primo start
			CicloCerchio						 					:BOOL;  //Abilitazione ciclo cerchio
			OffsetX1TaglioCerchio 									:REAL;  //Offset per index X1 per gestione pezzo cerchio
			ModoDiRiferimento										:SINT;	
			(*{
			#ifdef DH4010 
			}*)
				(*TabCentrature 										:ARRAY[TABELLA_SX..TABELLA_DX] OF Tab_Centratura;
				NumRigheTabCentratura								:ARRAY[TABELLA_SX..TABELLA_DX] OF INT; // Numero stati presenti nella tabella stati
				OffsetInizialeCentrature							:ARRAY[TABELLA_SX..TABELLA_DX] OF REAL;
				TabSincro 											:Tab_Sincronismo;*)
				NumSincro 											:INT;
				QuotaX6SmussoDopoTaglio								:REAL;
				TaglioPezziDiritti									:BOOL;
				UsoSoloTestaSx										:BOOL;
				DistanzaSmussi										:REAL;
				TipoInfilaggio										:SINT;	//0=Infilaggio manuale, 1=Infilaggio automatico
				OffsetCorrezioneSmusso								:REAL;
				(*CentAuto 											:CentAuto_Struct; //Gestione Centratore Automatico*)
			(*{
			#endif	//Fine DH4010
			}*)
			(*{
			#ifdef EFLEX 
			}*)
				(*ModoMisuraSmusso      								:SINT;  //-1=Disabilitata, 0=Solo misura, 1=Misura e correzione
				LatoTaglio											:SINT;
				QuotaZ1TaglioVGP									:REAL;
				QuotaW1TaglioVGP									:REAL;
				OffsetX1Taglio	      								:REAL;  //Correzione costante (da VGP) della posizione di taglio con smussatore
				OffsetMisurato  									:LREAL;	//Correzione misurata della posizione della smussatura
				CicloSpeciale										:BOOL;  //Abilita il ciclo che esegue le smussature senza arretrare la X1
			(*{
			#endif	//Fine EFLEX 
			}
		{
		#endif	//Fine EFLEX_DH4010
		}
	{
	#else	//Fine ORBITALE
	}*)
		//MAcchine lineari
		(*TaglioLaser													:BOOL;	//Abilitazione taglio laser
		MisRitElastico												:ARRAY[0..MAXSEQ_MISY] OF RitornoElastico;
		PosZ1RicSaldQuadri											:ARRAY[0..3] OF REAL; //Posizioni Z per ricerca saldatura tubi quadri
		OffsetZ1RicSaldatura										:REAL;	//Offset Z1 per ricerca saldatura
		Z1OrientamentoPinza											:REAL;	//Orientamento pinza per tubi non tondi
		PedaleChApPinza												:BOOL;	//True se pedale apertura/chiusura pinza senza safety abilitato
		PedaleConfort												:SINT;	//0=Disabilitato, 1=Carico e scarico, 2=Solo scarico
		TemperatureSetpoint											:INT;
		AC00006623													:BOOL;	//Accessorio anima sospesa
		AC00006842_02												:BOOL;	//Sostegno tubo con motore stepper Festo
	(*{
	#endif	//Fine NO ORBITALE
	}*)
		(*CodicePezzoVGP												:SINT;
		X1ChiusPinzaCarRobot										:LREAL:=-MY_BIG_VALUE_REAL;
	(*{
  #ifdef SELEZIONE_BARRIERA_1_2
  }*)
		AbilSelDueBarriere											:BOOL;	//True abilita la selezione da Simotion della barriera 1 o 2
  (*{
  #endif //Fine SELEZIONE_BARRIERA_1_2
  }
	{
	#ifdef CURVATURA
	}
    {
    #ifndef EFLEX_DH4010
    }*)
		CoppiaMediaY1Frenatura										:ARRAY[1.._MAXSEQ#MAXSEQ] OF REAL;
		CoppiaMediaY1FrenaturaAppo									:ARRAY[1.._MAXSEQ#MAXSEQ] OF REAL;
		(*{
		#endif	//Fine EFLEX_DH4010
		}
	{
	#endif	//Fine CURVATURA
	}*)
		(*NumPezziSvuotCassa											:DINT;		//Numero pezzi per svuotamento cassa, 0 non si ferma mai 
		UsoPaletta 													:BOOL;
	(*{
	#ifdef MISURA_RITORNO_ELASTICO_DUE_SENSORI
	}*)
		(*TolleranzaNegativa											:REAL;
		TolleranzaPositiva											:REAL;
		AumentoFinecorsSlitta										:REAL;
	(*{
	#endif	//MISURA_RITORNO_ELASTICO_DUE_SENSORI
	}

	{
	#ifndef EFLEX_DH4010
	}*)
		(*AllungamentoMisurato										:LREAL;
		AllungamentoTotale											:REAL;	//Allungamento totale misurato*)
		PinzaDiamMinMax												:SINT;	//1=Selezione diametro minimo  2=Selezione diametro massimo
		CaricoTestaBassa											:BOOL;	//Abilitazione carico testa bassa
		CaricoTestaBassaQ1											:LREAL; //Posizione Q1 di testa bassa per il carico
		CaricoTestaBassaV1											:LREAL;	//Posizione V1 di testa bassa per il carico
		ScaricoTestaBassa											:BOOL;	//Abilitazione scarico testa bassa
		ScaricoTestaBassaQ1											:LREAL;	//Posizione Q1 di testa bassa per lo scarico
		ScaricoTestaBassaV1											:LREAL;	//Posizione V1 di testa bassa per lo scarico
		PinzaCoccodrillo3Posizioni									:BOOL:=TRUE;	//True 	=> pinza coccodrillo su 3 posizioni
																							//False => 2 posizioni (Chiuso, Intermedio)
		DisabilCalW1PrimoStart										:BOOL;	//Elect-XL disabilita calibrazione pinza durante chiusura in automatico
		ModoWeld													:INT;
		PosizioneWeld												:BOOL;	//0=Cannotto 1=Sostegno tubo
		PosXAlzoSostegno											:LREAL;	//Quota minim X al di sotto della quale non è possibile alzare
																				//il sostegno tirante per effettuare la ricerca
		//PosQ1LoadTube												:LREAL:=-MY_BIG_VALUE_REAL;	//Posizione teorica Q1 per carico tubo Aerolia
		QuotaQAlCarico												:REAL;	
		QuotaTAlCarico												:REAL;
		Tempo_U4_Pneumatica											:REAL:=0;	//Attesa prima dell^arretramento
		U2TaglioVerticale											:REAL;		//Posizione U2 per taglio coltello lama verticale
		U2TaglioOrizzontale											:REAL;		//Posizione U2 per taglio coltello lama orizzontale
		UsoPressostatoElettronico									:BOOL; 
		CaricoPosterioreAtommash									:BOOL:=BOOL#0;
	(*{
	#endif	//Fine NO EFLEX_DH4010
	}*)
	
	//Creati solo per esportare i dati come li vuole Adige, cioè con la struttura che parte da indice 0
	//anche con il pezzo seguente o la selezione esterna
		(*TipoTubo_export												:REAL;
		Diametro_Tubo_export										:REAL;
		Diametro_Tubo_B_export										:REAL;	
		Spessore_export												:REAL;
		LunghezzaTuboTeorica_export									:REAL;
		PartMaterialCode_export										:STRING[16];
		PezzoPieno_export											:INT;
		PartWeightDensityLinear										:REAL;
		PartWeightDensityLinear_export								:REAL;
		PartMaterialType											:DINT;
		PartMaterialType_export										:DINT;*)
	END_STRUCT;

	Config_Struct:STRUCT
	QuotaAbbassoSostegnoTirante										:LREAL;			//Quota X a cu abbassare il tirante  
	SopralzoUt1Dx				  									:REAL;
	SopralzoUt1Sx				  									:REAL;
	PresenzaTirante													:BOOL;			
	LunghezzaAnima													:LREAL;
	TipoSlittaMobile												:INT:=0;		//0=Slitta elettrica  1=Slitta pneumatica
	Offset_Meccanico_X_U2											:LREAL:=0;
	Elect_DX_SX					 									:SINT:=SINT#0;
	UsoPinzaCoccoPneumatica											:BOOL:=FALSE;	//True=>pinza coccodrillo standard su 3 posizioni
																						//100=aperta, 
																						//0=intermedia,
																						//StopPoint e primo start al carico per apertura totale
	UsoPinzaCoccoElettrica											:BOOL:=FALSE;	
	UsoPinzaCoccoElettrica_shadow									:BOOL:=FALSE;	
	i_sys_IngRulliDXSX												:LREAL:=340;		
	//i_sys_PosTestaBassaXMin										:LREAL:=MY_BIG_VALUE_REAL;
	//i_sys_PosTestaBassaXMax										:LREAL:=MY_BIG_VALUE_REAL;
	i_sys_LatoBasculante											:SINT:=SINT#-1;		//0=Basculante DX  1=Basculante SX
	//i_sys_LimMaxYSX												:LREAL:=-MY_BIG_VALUE_REAL;
	//i_sys_LimMaxYDX												:LREAL:=-MY_BIG_VALUE_REAL;
	i_sys_pinza_due_diametri										:INT;
	//i_sys_DistMinW3W4_DX											:LREAL:=-MY_BIG_VALUE_REAL;
	//i_sys_DistMinW3W4_SX											:LREAL:=-MY_BIG_VALUE_REAL;
	i_sys_PM100_DX				  									:REAL;
	i_sys_PM104_DX				  									:REAL;
	i_sys_PM106_DX				  									:REAL;
	i_sys_PM100_SX				  									:REAL;
	i_sys_PM104_SX				  									:REAL;
	i_sys_PM106_SX				  									:REAL;
	i_sys_POffsetQsx												:LREAL;
	i_sys_POffsetTsx												:LREAL;
	i_sys_MaxSforzo_Tirante											:BOOL:=FALSE;
	//i_sys_Max_Y1_Curvatura										:LREAL:=-MY_BIG_VALUE_REAL;
	i_sys_CaricatoreBLM_DX											:SINT:=SINT#-1;	//1=CARM
																			//2=AL42
																			//3=AL43
																			//4=AL42PLUS
																			//5=AL5													
																			//6=AL5+BUNDLE
																			//7=CARM con FI
																			//8=AL5+ROLAND
																			//9=AL5+BUNDLE+ROLAND
	i_sys_CaricatoreBLM_SX											:SINT:=SINT#-1;	//1=CARM
																			//2=AL42
																			//3=AL43
																			//4=AL42PLUS
																			//5=AL5													
																			//6=AL5+BUNDLE
																			//7=CARM con FI	
																			//8=AL5+ROLAND
																			//9=AL5+BUNDLE+ROLAND
	
	SicurezzaCilindroSottoTesta										:LREAL:=-1;	//Abilitazione/Lunghezza chiavetta bloccaggio matrice del cilindro sottotesta
	OffsetMultiCannottoX1											:LREAL:=0;	
	CaricoEurotubi													:BOOL;	//True abilita il caricatore speciale Eurotubi su ELECT-M
	(*{
	#ifdef SMART_A
	}*)
		LimW4DxSostegnoTirante										:LREAL;
		LimW4SxSostegnoTirante										:LREAL;
	(*{
	#endif	//Fine SMART_A
	}
	{
	#ifdef ELECTXL
	}*)
		LimQ1CausatoDaV1											:REAL:=-REAL#99999.0;//-MY_BIG_VALUE_REAL;
		LimV1CausatoDaQ1											:REAL:=-REAL#99999.0;//-MY_BIG_VALUE_REAL;
	(*{
	#endif	//Fine ELECTXL
	}*)
	TipoMacchinaDXSX												:USINT;	//0=DXSX, 2=Invertibili mediante comando, 3=Solo Dx o solo Sx
	(*{
	#ifdef ORBITALE 
	}
		{
		#ifdef EFLEX
		}*)
			EFROT													:BOOL;	//Presenza smussatore
			EFRDC													:BOOL;	//Presenza correttore raggi
			PresenzaDoppiaMatriceSX									:BOOL;
			PresenzaDoppiaMatriceDX									:BOOL;
			SpezzoneFiloIntestatura									:REAL:=50;				
			SensoDxSx												:SINT;
			PosSmussatore											:REAL:=REAL#-1283.5;	//Posizione dello smussatore rispetto allo 0 della X1
			PosSensMis												:REAL:=REAL#-106.5;	//Posizione del sensore di misura rispetto allo 0 della X1
			VGP_PosSmussatore										:REAL;	//Serve solo a programmare il file INI di VGP da BlmMotion
			VGP_PosSensMis											:REAL;	//Serve solo a programmare il file INI di VGP da BlmMotion
			MaxCorsaMisSmussatura   								:REAL:=1500;  //Massima corsa in S.P. per misura posizione smussatura
			MaxOffsetMisSmussatura									:REAL:=2;	//Massima differenza accettabile misura posizione smussatura
			DistAttSensSmussatura	  								:REAL:=-30;	//Distanza attivazione sensore da posizione smusso teorica
		(*{
		#endif	//Fine EFLEX
		}
		{
		#ifdef DH4010
		}*)
			DH40Rot													:BOOL;	//Presenza smussatore
			RobotPresente											:BOOL;	//Presenza robot di carico e/o scarico (non indica l^uso nel pezzo)
			DistTaglioSmussatore									:REAL;	//Distanza fra taglio e smussatore (entrambi montati sul terzocarro)
			OffsetMeccanico_X2_X3									:REAL;	//Distanza minima fra X2 e X3
			MinX1Manipolatore_Alto									:REAL;	//Quota minima X1 con manipolatore attivo
			MinX2Manipolatore_Alto									:REAL;	//Quota minima X2 con manipolatore alto
			MinX1Manipolatore_Basso									:REAL;	//Quota minima X1 con manipolatore basso
			MinX2Manipolatore_Basso									:REAL;	//Quota minima X2 con manipolatore basso
			MinX1Sostegno1_Alto										:REAL;	//Quota minima X1 con sostegno 1 filo alto
			MinX1Sostegno2_Alto										:REAL;	//Quota minima X1 con sostegno 2 filo alto
			MinX2Sostegno1_Alto										:REAL;	//Quota minima X2 con sostegno 1 filo alto
			MinX2Sostegno2_Alto										:REAL;	//Quota minima X2 con sostegno 2 filo alto
			MaxX3Sostegno2_Alto										:REAL;	//Quota massima X3 con sostegno 2 filo alto
		(*{
		#endif	//Fine DH4010
		}
		{
		#ifdef RUNNER
		}*)  
			ASPO_TempoMaxAvvolgimento								:REAL:=REAL#1.1;		//Tempo di riavvolgimento tubo su aspo
			ASPO_PercentualeTempoOff								:INT:=50;			//Percentuale uscita off del tempo ASPO_TempoMaxAvvolgimento
			ASPO_NuovoModo											:BOOL:=TRUE;	//True abilita la frizione per frenare l^aspo anche quando è in folle
			ASPO_MantieneSvolgimento								:BOOL:=FALSE;	//True mantiene lo svolgimento dell^aspo fino anche in caso
																									//di arresto di X1 fino all^attivazione del sensore di braccio
																									//aperto
			//_4RC													:Dati4RC;	
			//_4RIPC												:Dati4RIPC;
			//_4RFORM												:Dati4RFORM;	
			_4RUNNER_Pos_Fotocellula								:REAL:=-300; 	//Posizione della fotocellula rispetto allo zero matrice
			_4RUNNER_Pos_Rotelle_IPC								:REAL:=-1800; //Posizione delle rotelle IPC rispetto allo zero matrice
			_4RUNNER_Pos_W1											:REAL:=201; 	//Posizione della morsa 4RFORM rispetto allo zero matrice
			_4RUNNER_Pos_W1_A1										:REAL:=-35; 	//Posizione della lama di 4RTL rispetto allo zero morsa
			_4RUNNER_Pos_W1_A1A2									:REAL:=-35; 	//Posizione della lama di 4RC rispetto allo zero matrice
			_4RUNNER_Pos_Encoder									:REAL:=-2500; //Posizione encoder rispetto allo zero matrice
			//_4RUNNER_Pos_Encoder_Accomp							:REAL:=-MY_BIG_VALUE_REAL; //Distanza fra encoder e carrello accompagnamento 
			_4RUNNER_4RForm_4Runner									:REAL:=1000; 	//Distanza di sicurezza fra 4RFORM e 4RUNNER
			_4RUNNER_CorsaCarrAccomp								:REAL:=300;		//Corsa utile encoder carrello accompagnamento a riposo
			_4RUNNER_U1Sicurezza									:REAL:=-1300;	//Posizione U1 in sicurezza rispetto alla macchina
			_4RUNNER_PerVelX1Intestatura							:USINT:=USINT#30;		//Percentuale velocità X1 durante prima intestatura
			_4RUNNER_PerAcCU1Intestatura							:USINT:=USINT#50;		//Percentuale accelerazione X1 durante prima intestatura
			_4RUNNER_PerVelX1Misura									:USINT:=USINT#100;	//Percentuale velocità X1 durante prima intestatura
			_4RUNNER_PerAcCU1Misura									:USINT:=USINT#50;		//Percentuale accelerazione X1 durante prima intestatura
			_4RUNNER_TipoTaglioEsterno								:USINT;				//1=Taglio 4RTL, 2=Taglio 4RC
			_4RUNNER_TipoTaglioInterno								:USINT;				//0=Taglio IPC, 1=Taglio 3RIPC/4RIPC
			_4RUNNER_SpessoreLama4RTL								:REAL:=REAL#1.2;
			//_4RUNNER_U1PosTaglio									:REAL:=-MY_BIG_VALUE_REAL;
			//_4RUNNER_U2PosTaglioRLRC								:REAL:=-MY_BIG_VALUE_REAL;
			//_4RUNNER_U2PosTaglioRL								:REAL:=-MY_BIG_VALUE_REAL;
			_4RUNNER_Pos_U2_LEVA_ALTA								:REAL:=REAL#31.5;	//Posizione limite U2 per leva alta con 4RTL
			_4RUNNER_Pos_U2_LEVA_ALTA_4RC							:REAL:=REAL#0.1;		//Posizione limite U2 per leva alta con 4RC
			_4RUNNER_PosX1SensoreCartella							:REAL:=0;
			_4RUNNER_4RTL_NumDataSet								:SINT:=SINT#1;	//Numero del data set da usare per il taglio lama
			_4RUNNER_Mo1613											:BOOL;	//Abilitazione incisione IPC ridotta
			_4RUNNER_AC4532											:BOOL;	//Abilitazione soffio aria terminale tubo dopo taglio
			_4RUNNER_TavoliSerpentine								:BOOL;	//Abilitazione tavoli serpentine
			_4RUNNER_pos_foratura         							:REAL;  //Distanza fra centro matrice e trapano di foratura
			_4RUNNER_pos_Flow_Drill									:REAL;  //Distanza fra centro matrice e Flow-Drill
			_4RUNNER_LimMinV1PuntaTubo								:REAL;	//Limite minimo V1 punta rispetto al tubo
			_4RUNNER_LimMinV1TrapanoMorsa							:REAL;	//Limite minimo V1 trapano rispetto alla morsa di foratura
			_4RUNNER_PosV1AbbassoTesta								:REAL;	//Quota teorica V1 per passaggio DXSX con 4DRILL
			_4RUNNER_Presenza4DRILL									:BOOL;	//True se abilitato
			_4RUNNER_PresenzaFLOW_DRILL								:BOOL;	//True se abilitato
			_4RUNNER_4RFORM_levaScaricoScivolo						:BOOL;	
			_4RUNNER_Marcatrice										:BYTE;	//00000001	Marcatrice standard (solo una marcatura per programma pezzo)
																						//00000011	Marcatrice con selezione programma di marcatura
			LimitazioneDXSX											:REAL;	//SF [28/03/2018]: indica la presenza del pattino di accompagnamento
																					 //curva su 4-Runner H3 Welbilt
																					 //SF [19/04/2018]: quello che prima era un parametro specifico per la H3 Welbilt,
																					 //ora è comune a tutte le 4RUNNER e indica una limitazione generica nel compiere
																					 //cambi da destro a sinistro
			TipoMAS													:REAL; //Indica che tipo di modulo di sagomatura è montato:
																					 //-1 se versione vecchia di VGP, indica 4RFORM. Incrociare con parametro Presenza4FORM
																					 //0 se non montato, 1 se 4RFORM, 2 se 4RHFORM, 3 se 3RFORM, 4 se 4RLFORM. 
			PresenzaSostegniGuidaTubo								:BOOL; //indica se sono montati (non necessariamente utilizzati) i sostegni di supporto tubo per diametri piccoli su 3-RUNNER (cliente Carpigiani)
			QuotaSostegno3RIPC										:REAL;  //quota alla quale si trova il sostegno montato su 3RIPC						
			QuotaSostegno4RFORM										:REAL;	//quota alla quale si trova il sostegno montato su 4RFORM
			(*{
			#ifdef RUNNER
			} *) 
				_3RUNNER_MaxOffsetX1Taglio							:REAL:=REAL#0.1;	//Massima differenza fra LunghezzaTubo2D e LunghezzaTubo3D	
			(*{
			#endif	//Fine RUNNER
			}
		{
		#endif	//Fine RUNNER
		}
	{
	#else //Fine RUNNER
	}*)
		CorsaEspulsore												:REAL;
		IngombroInternoEspulsore									:REAL;
		LunghezzaPinza												:REAL;
		MaxAngoloRicerca											:LREAL:=10;	//Massimo angolo ricerca
		VelRicerca													:LREAL:=2;	//Percentuale velocita^ ricerca
		LT722														:BOOL;
		TaglioColtello												:SINT;	//Informazione valida sia per taglio versione 1 che versione 2
																			//0=non presente
																			//>0=posizione su utensili DX
																			//0<=posizione su utensili SX
		TipoTaglioColtello											:SINT;	//0=Taglio versione 1
																			//1=Taglio versione 2 solo lama orizzontale
																			//2=Taglio versione 2 lama orizzontale e lama verticale
		LamaVerticale												:BOOL:=TRUE;	//Abilitazione uso lama verticale per taglio a coltello
		SensoreSpezzone												:BOOL;	//True sensore rilevazione spezzone dopo taglio
		AC4332_SM													:BOOL;	//Scomparsa rulli LAVORO a tempo
		Slitta_mobile_lunga											:SINT; 	//Valore che gestisce le collisioni tra slitta mobile lunga e vite dell^asse V1 su XL.
																			//0 = disabilitato, 1 = gestione Midway, 2 = gestione EATON
		AL5_RiferimentoMobile										:SINT;	//1<=Disablitato, 1=Posizione 1, 2=Posizione 2
		BloccoScomparsaBloccaggio									:INT;
		SottoTesta3Pos												:SINT;	//1<=Disablitato, 1=Apertura intermedia, 2=Apertura totale
		GripDev														:BOOL;	//True reggi-tubo riscaldato è presente
		QuotaXIngombroForaturaIdraulica								:LREAL;
		CoppiaU2SalvaTirante										:REAL:=-1;	//NTm asse U2 per sicurezza salvatirante anima
		AC00006842_02												:BOOL;	//True abilita il sostegno speciale comandato da motore stepper Festo	
	(*{
	#endif	//Fine NO RUNNER
	}*)
END_STRUCT;

Alarms:STRUCT
	Status					:SINT; //(0= Disattivo, 1= Attivo)
	DateTime				:DATE_AND_TIME;
	Code					:DINT;
	Code_export				:DINT;
	Axis	 				:SINT;
	TypeAxis				:INT;
	P_DINT  				:ARRAY[1..5]OF DINT;
	P_UDINT					:ARRAY[1..5]OF UDINT;
	P_LREAL					:ARRAY[1..5]OF LREAL;
END_STRUCT;	

Timer :STRUCT			  
	Value	:LREAL;	  
	Status	  :SINT;	//0=Scaduto 1=AttivoConteggio 2=PausaConteggio
	Executed:BOOL;	//True quando il timer ha raggiunto lo zero
END_STRUCT; 

_readDriveParameter: STRUCT 
				functionResult : DINT;
				parameterResult : DINT;
				dataType 		: UDINT;
				dataLength		: UDINT;
				data			:Array [0..233] OF BYTE;
END_STRUCT;
END_TYPE

TYPE
	EnumNextCommandEnable : DINT
	(IMMEDIATELY								:=DINT#60,
	WHEN_COMMAND_DONE                           :=DINT#160      );
END_TYPE

TYPE
	EnumHomingMode :INT
	(DIRECT_HOMING_ABSOLUTE								:=0,
	DIRECT_HOMING_RELATIVE								:=1,
	PASSIVE_HOMING_WITHOUT_RESET						:=2,
	ACTIVE_HOMING										:=3,
	RESERVED											:=4,
	ACTIVE_HOMING_POSITION_PARAMETER_HAS_NO_EFFECT		:=5,
	SET_OFFSET_OF_ABSOLUTE_ENCODER_BY_POSITION			:=6,
	ENABLE_OFFSET_OF_ABSOLUTE_ENCODER					:=7,
	PASSIVE_HOMING										:=8,
	ABORT_PASSIVE_HOMING								:=9,
	PASSIVE_HOMING_POSITION_PARAMETER_HAS_NO_EFFECT		:=10,
	SETTING_OF_POSITION_SETPOINT_ABSOLUTE				:=11,
	SHIFT_THE_SETPOINT_POSITION_RELATIVE                :=12,
	INCREMENTAL_ENCODER_ADJUSTMENT						:=13);

	EnumSyncProfileReference : DINT		
	(SYNC_ADV_DYN_PAR	 := 0, //Synchronization in advance using dynamic parameters
	SYNC_ADV_LEAD_VALUE_DISTANCE := 1,	//Synchronization in advance using leading value distance
	DIRECT_SYNC_SET    := 2, //Direct synchronous setting
	SUBSEQ_SYNC_LEAD_VALUE_DISTANCE := 3,//Subsequent synchronization using leading value distance
	SUBSEQ_SYNC_LEAD_VALUE_DISTANCE_START_LEAD_VAL_POS := 4,//Subsequent synchronization using leading value distance starting from current leading value position
	DIRECT_SYNC_SET_AT_END_CAM := 5, //Direct synchronous setting at the end of the cam
	SYNC_ADV_LEAD_VALUE_DISTANCE_START_FROM_COR_LEAD_VAL_POS := 6);//Synchronization in advance using leading value distance starting from current leading value position


	EnumDirection : INT
	(USER_DEFAULT := 0,// Default value from axis configuration 
	POSITIVE := 1,// Positive direction 
	SHORTEST_WAY := 2, // Shortest path 
	NEGATIVE := 3, //Negative direction 
	EFFECTIVE := 4);
END_TYPE

TYPE
LineDataProcessing:STRUCT
	NumLavPiece			:INT;
	NumLavExecuted	:INT;
	NumLavFirstStart:INT;
END_STRUCT;  
END_TYPE 

TYPE
	Device:STRUCT
	Enable 									:BOOL; 	// Abilitazione
	DispVgp									:BOOL:=TRUE; // Dispositivo riconosciuto da VGP
	Colonna 								:SINT; 	// Rappresenta la colonna della tabella stati
	Agganciato_Tabella_Stati:BOOL;	// Se TRUE viene controllato in tabella stati altrimenti no
	TipoIn 									:SINT;	// 0 - INDEFINITO
																	// 1  - TEMPO LAVORO - TEMPO RIPOSO   xxxxxxx0001
																	// 3  - TEMPO LAVORO - MICRO RIPOSO   xxxxxxx0011
																	// 5  - MICRO LAVORO - TEMPO RIPOSO   xxxxxxx0101
																	// 7  - MICRO LAVORO - MICRO RIPOSO   xxxxxxx0111
																	// 9  - INGRESSO ANALOGICO			xxxxxxx1001 
	TipoOut 							:SINT;	// 0 - INDEFINITO
																// 1  - USCITA ON/OFF DIGITALE		xxxxxxxx001
																// 3  - USCITA ANALOGICA			  xxxxxxxx011 
	Limite_Lavoro 				:REAL;	// Limite positivo (USUALMENTE 100)
	Limite_Riposo 				:REAL;	// Limite negativo (USUALMENTE 0)
	Limite2_Lavoro 				:REAL;	// Limite positivo per monitoring(USUALMENTE 100)
	Limite2_Riposo 				:REAL;	// Limite negativo per monitoring (USUALMENTE 0) 
	Vel_Lavoro 						:REAL:=200; // Velocità per passare da RIPOSO a LAVORO
	Vel_Riposo 						:REAL:=200; // Velocità per passare da LAVORO a RIPOSO
	In_Lavoro 						:INT; // DIGITAL;
	In_Riposo 						:INT; // DIGITAL;
	Out_Lavoro 						:INT; // DIGITAL;
	Out_Riposo 						:INT; // DIGITAL;
	Out_Lavoro_2 					:INT:=-1;	//Seconda uscita digitale del dispositivo;
	Out_Riposo_2 					:INT:=-1;	//Seconda uscita digitale del dispositivo;
	Ritardo_Uscite				:REAL;
	PresenzaCostante			:BOOL:=FALSE;	//Spegne uscite quando in JOG lascia il tasto + o -
	SpegnimentoUscita			:BOOL:=FALSE;	//Spegne uscita quando arriva alla posizione prevista
	Stato  						:INT:=DEVICES_STATUS#INDEFINITO; //Ultimo stato letto
																					//Assume valori come: LAVORO, RIPOSO, INTERMEDIO, VERSO_LAVORO, VERSO_RIPOSO, ecc..
	Timer									:REAL;	// Contatore interno per dispositivi a tempo
	TimerFiltro 					:REAL; 
	Position 							:REAL;	//Posizione del disositivo:
																//può assumere valori compresi fra LIMITE_RIPOSO e LIMITE_LAVORO
	In_Movimento 					:BOOL;	//Indica se il dispositivo è in movimento
	Consenso_Plus		   		:BOOL;
	Consenso_Minus		  	:BOOL;
	StatoRichiesto		  	:REAL;
	TempoInversioneMDA	 	:REAL:=1;
	Mda_Time_Lavoro				:LREAL;
	Mda_Time_Riposo				:LREAL;
	ma_Temperatura_Assorbimento	:INT:=0;	//39300 Temperatura minima motore per ciclo di assorbimento
	ma_Coppia_Accelerazione_Rif	:REAL:=0;	//39301 Coppia media di riferimento durante accelerazione
	ma_Coppia_VelCostante_Rif		:REAL:=0;	//39302 Coppia media di riferimento durante velocità costante
	ma_Riposo_Assorbimento			:REAL:=0;	//39303 Quota di riposo al termine del ciclo di misura assorbimento
	ma_Coppia_Accelerazione			:REAL:=0;	//39004 Coppia media misurata durante accelerazione
	ma_Coppia_VelCostante				:REAL:=0;	//39005 Coppia media misurata durante velocità costante
	ma_Esito_Assorbimento				:BOOL:=FALSE;//Esito della verifica dell^assorbimento
	//ma_NumeroTabellaStati				:SINT:=-1;			
	(*{
	#ifdef CARICO_SCARICO_PARALLELI
	}
		Agganciato_Tabella_Stati_Appo:BOOL;	// Se TRUE viene controllato in tabella stati altrimenti no
	{
	#endif	//Fine CARICO_SCARICO_PARALLELI
	}*)
END_STRUCT;	
END_TYPE

TYPE
	Tabella					:ARRAY [0..1, 0.._MAX_AXIS_LINEARISES#MAX_AXIS_LINEARISES] OF LREAL;  //0=forward 1=backward
END_TYPE

TYPE
	Linearizza						:STRUCT
	Asse						: Tabella;
	OffsetOld						:REAL;
	Ma_Enc_Comp_Enable	:BOOL:=FALSE;
	Ma_Enc_Comp_Step		:REAL;
	Ma_Enc_Comp_Min			:REAL;
	Ma_Enc_Comp_Max			:REAL;
	Ma_Enc_Comp_Module	:REAL;
END_STRUCT;

END_TYPE

TYPE
	//struttura necessaria per abilitare e posizionare gli assi master con i blocchi PLCOpen
MasterAxis:STRUCT
	PosAx															:  DB_ANY;
	MC_Axes                          								: Axis_ctrl;

		//strutture che contengono i parametri di chiamata dei vari blocchi PLCOpen
		// mcMoveAbsoluteInfo								                : MC_MoveAbsolute;
		// mcMoveRelativeInfo								                : MC_MoveRelative;
		// mcEnableAxisInfo									            : MC_Power;
		// mcResetAxisInfo										            : MC_Reset;
		// mcHomeAxisInfo										            : MC_Home;	
		// mcStopAxis										            	: MC_Stop;
		// mcJogAxisInfo											        : MC_MoveJog;
		// mcCamInInfo												        : MC_CamIn;
		// mcCamOutInfo											        : MC_CamOut;

END_STRUCT;
END_TYPE

TYPE
	Kinematics :STRUCT

	KinAx         :DB_ANY;

	mcMoveLinearRelative			: MC_MoveLinearRelative;
	mcMoveLinearAbsolute			: MC_MoveLinearAbsolute;
	mc_GroupStop					: MC_GroupStop;
	END_STRUCT;
END_TYPE

TYPE
Riga_stato : ARRAY[0.._MAXCOL_STATES#MAXCOL_STATES-1, 0.._MAXSTATES#MAXSTATES-1] OF REAL;	
END_TYPE

TYPE
TabStato	:  (Riga_stato);
END_TYPE

TYPE
	SelExtProd: STRUCT
	AbilSelExtTabella				:BOOL;	//TRUE abilita la selezione esterna della tabella di produzione. Viene impostata da VGP3D
	InpCodiceTabellaOK			:BOOL;	//Viene impostato a TRUE dal supervisore per indicare che il codice del pezzo impostato
																	//nel byte Ex05 (parte bassa) e Ex06 (parta alta) è da considerarsi valido.
																	//Indirizzo di comunicazione Ex
  CodiceTabellaDaCaricare	:DINT;	//Codice della tabella composta da 15 programmi da inviare a VGP3D viene valorizzato dopo
																	//che la variabile CodiceTabellaOK=TRUE
	OutCodiceTabellaOK			:BOOL;	//Viene impostato a TRUE dal Simotion al termine del trasferimento della produzione da parte di VGP A942.5
	CodiceErroreTabExt			:INT;		//Codice di errore che ha impedito il trasferimento della tabella di produzione richiesta
																	//-1=inizio trasferimento 0=OK, 1=errore genererico, 2=incongruenza utensili, 5=errore tabella modo, ezzetera
	AbilPezSegTabExt				:BOOL;	//TRUE abilita la modalita selezione esterna della tabella di produzione indicata
																	//FALSE abilita la modalita pezzo seguente della tabella di produzione indicata
END_STRUCT;
END_TYPE

TYPE
	Access_Level : STRUCT
	DOWNLOAD  					:SINT; // possibilità di download del programma
	JOG_IN_MONITORING 	:SINT; // possibilità di muovere in manuale solo in monitoring
	JOG_LIBERO 					:SINT; // possibilità di muovere in manuale anche non in monitoring
	INDEX 							:SINT; // possibilità di effettuare la presa index
	DISATTIVAZIONE_ASSI :SINT;
	CAMBIO_CFG_HW 			:SINT;
	START_PROGRAMMA 		:SINT;
END_STRUCT;   
END_TYPE

TYPE
	Curvatura_Manuale_Struct: STRUCT 
	Override_LOW_VEL 	:REAL; //bassa velocità
	Override_HIGH_VEL	:REAL; //velocità con tasto rapid premuto
	Avanti  					:BOOL; //Flag per sapere se attivo ciclo avanti
	Indietro 					:BOOL; //Flag per sapere se attivo ciclo indietro
END_STRUCT;	
END_TYPE

TYPE
	Calibraz_Struct :STRUCT
	Risultato   				:SINT;	   // -1 = FALLITA
	Utensile						:SINT;
	NumeroAsse  				:SINT;  
	Coppia	  					:LREAL;	 // percentuale di coppia per calibrazione 
	TempOffset  				:LREAL;	 //Quota offset dopo ultima calibrazione (da restituire a VGP)
	TempOffsetLowTorque	:LREAL;	 //Quota offset per chiusure a coppia ridotta (da restituire a VGP)
END_STRUCT; 
END_TYPE

TYPE
	Trace_Asse:STRUCT
	DatiAssi				:ARRAY[_FIRST_AXIS#FIRST_AXIS.._FIRST_AXIS#FIRST_AXIS+_CONST_VAR_DEF._MAXIS#MAXAXIS-1] OF DatiOscilloscopio;
	NumeroRighe				:INT:=_MAX_LOG#MAX_LOG_SIZE;
	Pointer_log 			:INT:=0;
	DatiGenerali_1			:ARRAY[0.._MAX_LOG#MAX_LOG_SIZE-1] OF Stato_Storico_Generale;
	DatiGenerali_2			:ARRAY[0.._MAX_LOG#MAX_LOG_SIZE-1] OF Stato_Storico_Generale;
	NumeroMaster			:USINT:=USINT#1;
	TempoPostMortem			:USINT;
	StartLetturaDati		:BOOL;	//True da Simotion per far partire la creazione del log dal servizio,
													//viene messa a False dal servizio
	RigaErrore				:INT:=-1;
END_STRUCT; 
END_TYPE

TYPE
	DatiOscilloscopio:STRUCT
	DatiAsse			:ARRAY[0.._MAX_LOG#MAX_LOG_SIZE-1] OF Stato_Storico_Asse;
END_STRUCT; 
END_TYPE

TYPE
Stato_Storico_Asse:STRUCT
	FollError 				:REAL:=0;
	Position				:REAL:=0;
	Velocity				:REAL:=0;
	Torque 					:REAL:=0;
	LimitTorque				:USINT:=USINT#0;
	DistanceToGo			:REAL:=0;
	Temperature				:USINT:=USINT#0;
	SovraccaricoI2T			:USINT:=USINT#0;
END_STRUCT; 
END_TYPE

TYPE
	Stato_Storico_Generale:STRUCT
	PosMaster			:REAL:=0;
	VelMaster			:REAL:=0;
	Riga	 			:INT:=0;
	Error				:DINT:=0;
END_STRUCT;
END_TYPE

TYPE
	WebApplication:STRUCT
	wd_job_code						:STRING;	//"JOB"&jobid di protube (se attivo)
	wd_job_state					:INT;			//Stato del Job
	wd_job_est_time					:LREAL;		//Tempo calcolato per fine produzione lotto (sec)
	wd_job_bar_count				:USINT;		//Sempre a 1
	wd_job_part_count				:USINT;		//Numero programmi che compongono il lotto (da 1 a 15)
	wd_bar_quantity					:DINT;		//Numero totale dei pezzi del lotto
	wd_bar_prof_type				:USINT;		//Tipo di profilo di barra (Diametro_Tubo_Sezione)
	wd_bar_dim1						:REAL;		//Dimensione orizzontale (Diametro_Tubo)
	wd_bar_dim2						:REAL;		//Dimensione verticale (Diametro_Tubo_B)
	wd_bar_thick					:REAL;		//Spessore
	wd_bar_length					:REAL;		//Lunghezza barra mettere a 0
	wd_part_cur						:USINT;		//Pezzo corrente in produzione
	wd_job_id						:INT:=0;	//Prende da Protube se valorizzato altrimenti 0
	wd_bar_cur						:USINT;		//Sempre a 0
	wd_bar_part_quant				:USINT;		//Sempre a 0
	wd_bar_part_quant_done			:USINT;		//Sempre a 0
	Pezzo							:ARRAY[1.._CONST_VAR_DEF._MPROGRAMS#MAXPROGRAMS-UINT#1] OF WebAppPezzo;
END_STRUCT;
END_TYPE

TYPE
	WebAppPezzo:STRUCT
	wd_part_name					:STRING[32];//Nome del pezzo senza percorso
	wd_part_quantity				:DINT;		//Quantita di pezzi da produrre*numero lotti
	wd_part_qextra					:DINT;		//Extra da produrre   ?????
	wd_part_length					:REAL;		//Lunghezza del pezzo (LunghezzaTuboEsecuzione)
	wd_part_quantity_done			:DINT;		//Quantità prodotta compresi i lotti precedenti 
	wd_part_prod_time				:LREAL;		//Tempo di produzione singolo pezzo + carico e scarico
	wd_part_prod_id					:INT:=0;	//Prende da Protube se valorizzato altrimenti 0
END_STRUCT;
END_TYPE

TYPE
	Struttura_Stato_Out:STRUCT	 //Struttura Per Memo Stato Output Digitali a seguito Stop Ciclo
	Stato	   :BOOL;	  //0 False 1 True	   
END_STRUCT;
END_TYPE

TYPE
	RigaUtDispositivo 		:STRUCT
	Offset  				:LREAL;	//????
	CoppiaRidotta 			:LREAL; 
	CoppiaNormale			:LREAL; //Coppia standard
	PosChiuso	   			:LREAL; //Posizione teorica (senza offset) chiusa
	PosIntermedio			:LREAL; //Posizione teorica (senza offset) intermedia
	PosAperto	   			:LREAL; //Posizione teorica (senza offset) aperta		
	PosChiusoSx	   			:LREAL; //Posizione teorica (senza offset) chiusa lato SX
	PosIntermedioSx 		:LREAL; //Posizione teorica (senza offset) intermedia lato SX
	PosApertoSx	  			:LREAL; //Posizione teorica (senza offset) aperta lato SX	   
	Free1			   		:REAL; //A disposizione	   
	Free2			   		:REAL; //A disposizione		
END_STRUCT;	
END_TYPE

TYPE
ProgrammaX :STRUCT 
	num_pezzi_fatti 						:DINT;	//Contapezzi di ciascun programma
	num_pezzi_totali 						:DINT;	//Numero pezzi totali da fare per ciascun programma
	num_pezzi_scarti_fatti			:DINT;	//Numero pezzi scarti in corso di produzione
	num_pezzi_scarti_totali			:DINT;	//Numero pezzi scarti da produrre
	num_pezzi_fatti_job 				:DINT; 	//Contapezzi di ciascun programma considerando le ripetizioni della tabella
	num_pezzi_totali_job				:DINT; 	//Numero pezzi totali da fare per ciascun programma considerando le ripetizioni della tabella
	tempo_ciclo 								:LREAL; //Tempo ciclo
	tempo_ciclo_totale					:LREAL; //Tempo ciclo totale
	DataOraInizioProduzione 		: DATE_AND_TIME;//:DT;
	DataOraFineProduzione 			: DATE_AND_TIME;//:DT;
	DataOraDownload 				: DATE_AND_TIME;//:DT;
	TagliEseguiti								:UDINT;
	{::ifdef AL_BLM}
		num_pezzi_fatti_AL5				:DINT;	//Contapezzi pezzi realmente azzerati su AL5
		num_pezzi_totali_AL5			:DINT;	//Numero pezzi totali da fare su AL5
	{::endif} //Fine AL_BLM
	
	Temp_num_pezzi_fatti					:DINT;	//Serve per il file di log di VGP
	Temp_num_pezzi_fatti_vgp			:DINT;	//Serve per il file di log di VGP
	Temp_tempo_ciclo_totale				:LREAL;	//Serve per il file di log di VGP
	Temp_tempo_ciclo_totale_vgp		:LREAL;	//Serve per il file di log di VGP
	num_pezzi_cassa								:DINT;	//Numero pezzi nella cassa
	wd_part_name									:STRING[32];//Nome del pezzo senza percorso
	wd_part_length								:REAL;	//Serve per il file di log di VGP
	//Creati solo per esportare i dati come li vuole Adige, cioè con la struttura che parte da indice 0
	//anche con il pezzo seguente o la selezione esterna
	num_pezzi_fatti_job_export		:DINT;
	num_pezzi_totali_job_export		:DINT;
	num_pezzi_scarti_fatti_export	:DINT; 	
	num_pezzi_scarti_totali_export:DINT;
	tempo_ciclo_export 						:LREAL;
	wd_part_name_export						:STRING[32];
	wd_part_length_export					:REAL;
END_STRUCT;
END_TYPE

TYPE
	pncoupler_i : STRUCT
	address_pn_i AT %B0 : ARRAY [0..63] OF BYTE;
	END_STRUCT;
END_TYPE

TYPE
	dbcoupler_i : STRUCT
	address_db_i AT %B0 :ARRAY [0..63] OF BYTE;
END_STRUCT;

END_TYPE

TYPE
	dbcoupler_o : STRUCT
	address_db_o AT %B0 :ARRAY [0..63] OF BYTE;
END_STRUCT;
END_TYPE

TYPE
	pncoupler_o : STRUCT
	address_pn_o AT %B0 : ARRAY [0..63] OF BYTE;
	END_STRUCT;
END_TYPE

TYPE
	ph_i : STRUCT 
	address_ph_i AT %B0 : ARRAY [0..11] OF BYTE;
	END_STRUCT;
END_TYPE

TYPE
	ph_i_old : STRUCT 
	address_ph_i_old AT %B0 : ARRAY [0..6] OF BYTE;
	END_STRUCT;
END_TYPE 


TYPE
	PHIAL5_Opz  : STRUCT 
	address_PHIAL5_Opz AT %B0 : ARRAY [0..6] OF BYTE;
	END_STRUCT;
END_TYPE

TYPE
	PHIAL5_Opz_old  : STRUCT 
	address_PHIAL5_Opz_old AT %B0 : ARRAY [0..6] OF BYTE;
	END_STRUCT;
END_TYPE

TYPE
	ph_o : STRUCT 
	address_ph_o AT %B0 : ARRAY [0..12] OF BYTE;
	END_STRUCT;
END_TYPE

TYPE
	ph_o_old : STRUCT 
	address_ph_o_old AT %B0 : ARRAY [0..6] OF BYTE;
	END_STRUCT;
END_TYPE 

TYPE
	PHOAL5_Opz  : STRUCT 
	address_PHOAL5_Opz AT %B0 : ARRAY [0..3] OF BYTE;
	END_STRUCT;
END_TYPE

TYPE
	PHOAL5_Opz_old  : STRUCT 
	address_PHOAL5_Opz_old AT %B0 : ARRAY [0..6] OF BYTE;
	END_STRUCT;
END_TYPE

END_NAMESPACE