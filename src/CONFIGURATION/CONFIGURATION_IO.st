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

CONFIGURATION _Configuration_IO

    VAR_GLOBAL//IO

        //IN
            mcpi_spindle_stop AT %I1.0 : BOOL;

            mcpi_10000 AT %I6.5 : BOOL;
            mcpi_1000 AT %I6.4 : BOOL;
            mcpi_100 AT %I6.7 : BOOL;
            mcpi_10 AT %I7.0 : BOOL;
            mcpi_1 AT %I7.3 : BOOL;
            mcpi_singleblock AT %I7.1 : BOOL;
            mcpi_repos AT %I7.5 : BOOL;
            mcpi_NCstart AT %I6.2 : BOOL;
            mcpi_ncstop AT %I6.1 : BOOL;
            mcpi_reset AT %I6.0 : BOOL;
            mcpi_AUTO AT %I6.3 : BOOL;
            mcpi_JOG AT %I7.4 : BOOL;
            mcpi_REF AT %I7.6 : BOOL;
            mcpi_MDA AT %I6.6 : BOOL;
            mcpi_VARINC AT %I7.2 : BOOL;
            
            mcpi_minus AT %I14.0 : BOOL;
            mcpi_plus AT %I14.2 : BOOL;
            mcpi_rapid AT %I14.1 : BOOL;
            mcpi_ax0 AT %I15.4 : BOOL;
            mcpi_ax1 AT %I15.5 : BOOL;
            mcpi_ax2 AT %I15.6 : BOOL;
            mcpi_ax3 AT %I15.1 : BOOL;
            mcpi_ax4 AT %I15.2 : BOOL;
            mcpi_ax5 AT %I15.3 : BOOL;
            mcpi_ax6 AT %I14.6 : BOOL;
            mcpi_ax7 AT %I14.7 : BOOL;
            mcpi_ax8 AT %I15.0 : BOOL;
            mcpi_ax9 AT %I14.3 : BOOL;
            mcpi_ax10 AT %I14.4 : BOOL;
            mcpi_ax11 AT %I14.5 : BOOL;

            mcpi_t0 AT %I11.4 : BOOL;
            mcpi_t1 AT %I11.5 : BOOL;
            mcpi_t2 AT %I11.6 : BOOL;
            mcpi_t3 AT %I11.1 : BOOL;
            mcpi_t4 AT %I11.2 : BOOL;
            mcpi_t5 AT %I11.3 : BOOL;
            mcpi_t6 AT %I10.6 : BOOL;
            mcpi_t7 AT %I10.7 : BOOL;
            mcpi_t8 AT %I11.0 : BOOL;
            mcpi_t9 AT %I10.3 : BOOL;
            mcpi_t10 AT %I10.4 : BOOL;
            mcpi_t11 AT %I10.5 : BOOL;
            mcpi_t12 AT %I10.0 : BOOL;
            mcpi_t13 AT %I10.1 : BOOL;
            mcpi_t14 AT %I10.2 : BOOL;


            mcpi_key0 AT %I4.0 : BOOL;
            mcpi_key1 AT %I4.1 : BOOL;
            mcpi_key2 AT %I4.2 : BOOL;
            mcpi_key3 AT %I4.3 : BOOL;

            StatoSafetety_X1:WORD;
            StatoSafetety_Y1:WORD;
            StatoSafetety_Z1:WORD;
            StatoSafetety_Q1:WORD;
            StatoSafetety_R1:WORD;
            StatoSafetety_W3:WORD;
            StatoSafetety_W4:WORD;
            StatoSafetety_U4:WORD;
            StatoSafetety_L1:WORD;
            StatoSafetety_Z2:WORD;

END_VAR

VAR_GLOBAL // OUT
            mcpo_singleblock AT %Q3.1 :BOOL;
            mcpo_repos AT %Q3.5 :BOOL;
            mcpo_reset AT %Q2.0 :BOOL;
            mcpo_ncstop AT %Q2.1 : BOOL;
            mcpo_ncstart AT %Q2.2 :BOOL;  
            mcpo_spindle_stop AT  %Q1.0 : BOOL; ///This needed to be chabge up to clients requirements
            
            mcpo_ax0 AT %Q11.4 : BOOL;
            mcpo_ax1 AT %Q11.5 : BOOL;
            mcpo_ax2 AT %Q11.6 : BOOL;
            mcpo_ax3 AT %Q11.1 : BOOL;
            mcpo_ax4 AT %Q11.2 : BOOL;
            mcpo_ax5 AT %Q11.3 : BOOL;
            mcpo_ax6 AT %Q10.6 : BOOL;
            mcpo_ax7 AT %Q10.7 : BOOL;
            mcpo_ax8 AT %Q11.0 : BOOL;
            mcpo_ax9 AT %Q10.3 : BOOL;
            mcpo_ax10 AT %Q10.4 : BOOL;
            mcpo_ax11 AT %Q10.5 : BOOL;

            mcpo3 AT %QB3 : BYTE;
            mcpo4 AT %QB4 : BYTE;
            mcpo5 AT %QB5 : BYTE;

            mcpo_plus AT %Q10.2 : BOOL;
            mcpo_minus AT %Q10.0 : BOOL;
      
            mcpo_1 AT %Q3.3 : BOOL;
            mcpo_10 AT %Q2.7 : BOOL;
            mcpo_100 AT %Q3.0 : BOOL;
            mcpo_1000 AT %Q2.4 : BOOL;
            mcpo_10000 AT %Q2.5 : BOOL;
            mcpo_varinc AT %Q3.2 : BOOL;
            mcpo_teach_in AT %Q0.2 : BOOL;
            mcpo_t14 AT %Q6.2 : BOOL;

            mcpo_JOG AT %Q3.4 : BOOL;
            mcpo_AUTO AT %Q2.3 : BOOL;
            mcpo_REF AT %Q3.6 : BOOL;
            mcpo_MDA AT %Q2.6 : BOOL;
END_VAR

/**********
* PLC_IO *
**********/

VAR_GLOBAL//IO

//IN
            I_VerPLCSafety AT %IB3000 : BYTE;

            Checksum_Plc_Safety_Hight AT %IW1064 :WORD;
            Checksum_Plc_Safety_Low AT %IW1066 :WORD;

            V_dbcoupler_i AT %IB900         : dbcoupler_i;
            V_pncoupler_i AT %IB1000        : pncoupler_i;

            V_ph_i AT %IB32                    : ph_i;
            V_ph_i_old AT %IB42                : ph_i_old;
            V_PHIAL5_Opz  AT %IB49             : PHIAL5_Opz; 
            V_PHIAL5_Opz_old  AT %IB49         : PHIAL5_Opz_old; 


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

        
        //OUT



        V_dbcoupler_o AT %QB900  : dbcoupler_o;
        V_pncoupler_o AT %QB1000 : pncoupler_o;
        
        V_ph_o AT %QB32                       : ph_o;
        V_ph_o_old AT %QB42                   : ph_o_old;
        V_PHOAL5_Opz AT %QB49                 : PHOAL5_Opz;
        V_PHOAL5_Opz_old AT %QB49             : PHOAL5_Opz_old;

        
        END_VAR
END_CONFIGURATION