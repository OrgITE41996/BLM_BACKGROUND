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

NAMESPACE _FC_ACCESSLEVEL_FC_NAMESPACE

/***********************************************************
 * READS THE HARDWARE ACCESS LEVEL FROM THE KEY OF THE MCP *
 ***********************************************************/

FUNCTION Read_hw_AccessLevel : INT

VAR_EXTERNAL
    managementUsers					:BOOL;
    mcpi_key0 : BOOL;
    mcpi_key1  : BOOL;
    mcpi_key2  : BOOL;
    mcpi_key3  : BOOL;
END_VAR

VAR_EXTERNAL CONSTANT
    ACCESS_NO				:SINT;
    ACCESS_OPERATORE     	:SINT;
    ACCESS_PROGRAMMATORE	:SINT;
    ACCESS_MANUTENTORE   	:SINT;
    ACCESS_SAT           	:SINT;
    ACCESS_MIS           	:SINT;
    ACCESS_UT           	:SINT;
    ACCESS_SUPERUSER     	:SINT;
END_VAR
	IF managementUsers THEN
		read_hw_AccessLevel:=ACCESS_NO;
	ELSE 
		IF mcpi_key0 THEN
			Read_hw_AccessLevel:=ACCESS_NO;	
		ELSE IF mcpi_key1 THEN
			Read_hw_AccessLevel:=ACCESS_OPERATORE;	
			ELSE IF mcpi_key2 THEN
				Read_hw_AccessLevel:=ACCESS_PROGRAMMATORE;	
				ELSE IF mcpi_key3 THEN
					Read_hw_AccessLevel:=ACCESS_MANUTENTORE;	
					ELSE
						Read_hw_AccessLevel:=ACCESS_NO;
					END_IF;	
				END_IF;		
			END_IF;		
		END_IF;		
	END_IF;
END_FUNCTION

/***********************************************************
 * READS THE HARDWARE ACCESS LEVEL FROM THE KEY OF THE MCP *
 ***********************************************************/

FUNCTION Read_sw_AccessLevel:INT

VAR_EXTERNAL
    s_sys_user_password         :STRING[12];
    i_sys_password_SUPERUSER        :STRING;
	i_sys_password_UT				:STRING;
	i_sys_password_MIS			    :STRING;
	i_sys_password_SAT			    :STRING;
	i_sys_password_SERVICE	        :STRING;
	i_sys_password_PROG			    :STRING;
	i_sys_password_USER			    :STRING;
END_VAR

VAR_EXTERNAL CONSTANT
    ACCESS_NO				:SINT;
    ACCESS_OPERATORE     	:SINT;
    ACCESS_PROGRAMMATORE	:SINT;
    ACCESS_MANUTENTORE   	:SINT;
    ACCESS_SAT           	:SINT;
    ACCESS_MIS           	:SINT;
    ACCESS_UT           	:SINT;
    ACCESS_SUPERUSER     	:SINT;
END_VAR

	IF s_sys_user_password=i_sys_password_SUPERUSER THEN
		Read_sw_accesslevel:=ACCESS_SUPERUSER;
	ELSE
		IF s_sys_user_password=i_sys_password_UT THEN
			Read_sw_accesslevel:=ACCESS_UT;
		ELSE
			IF s_sys_user_password=i_sys_password_MIS THEN
				Read_sw_accesslevel:=ACCESS_MIS;
			ELSE
				IF s_sys_user_password=i_sys_password_SAT THEN
					Read_sw_accesslevel:=ACCESS_SAT;
				ELSE
					IF s_sys_user_password=i_sys_password_SERVICE THEN
						Read_sw_accesslevel:=ACCESS_MANUTENTORE;
					ELSE
						IF s_sys_user_password=i_sys_password_PROG	THEN
							Read_sw_accesslevel:=ACCESS_PROGRAMMATORE;
						ELSE
							IF s_sys_user_password=i_sys_password_USER THEN
								Read_sw_accesslevel:=ACCESS_OPERATORE;
							ELSE
								Read_sw_accesslevel:=ACCESS_NO;
							END_IF;
						END_IF;
					END_IF;
				END_IF;
			END_IF;		
		END_IF;
	END_IF;
END_FUNCTION	

END_NAMESPACE	