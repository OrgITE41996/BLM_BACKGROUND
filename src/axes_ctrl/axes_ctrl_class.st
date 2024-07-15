
USING Siemens.Simatic.S71500.MotionControl.Native;
USING  _TYPE;
USING _CONST_VAR_DEF;
USING _STATE_CONST;

NAMESPACE _AXIS_CTRL_CLASS

    CLASS AXIS_CTRL
        VAR PUBLIC
            ////// IN //////
            stop_Axis_Param     :    StopAxisParamIn;
            move_abs_param      :    MoveAbsoluteParamIn;
            move_rel_param      :    MoveRelativeParamIn;
            reset_axis_param    :    ResetAxisParamIn;
            home_axis_param     :    HomeAxisParamIn;
            cam_out_param       :    CamOutparamIn;
            move_jog_param      :    JogAxisParamIn;

            /////// OUT /////////

            State_enable   : EnableAxisParam;
            State_stop     : StopAxisParamOut;
            State_move_abs : MoveAbsoluteParamOut;
            State_move_rel : MoveRelativeParamOut;
            State_reset    : ResetAxisParamOut;
            State_home     : HomeAxisParamOut;
            State_cam_out  : CamOutparamOut;
            State_move_jog : JogAxisParamOut;

            ///// ENABLE //////////
            
            enable_axes             : BOOL;
            stop_axes               : BOOL;
            move_absolute           : BOOL;
            move_relative           : BOOL;
            reset_axes              : BOOL;
            home_axes               : BOOL; 
            cam_out                 : BOOL;
    

        END_VAR

        VAR PRIVATE
            mcMoveAbsolute								                : MC_MoveAbsolute;
            mcMoveRelative								                : MC_MoveRelative;
            mcEnableAxis									            : MC_Power;
            mcResetAxis									                : MC_Reset;
            mcHomeAxis										            : MC_Home;	
            mcStopAxis										            : MC_Stop;
            mcJogAxis											        : MC_MoveJog;
            mcCamIn												        : MC_CamIn;
            mcCamOut											        : MC_CamOut;
            mcMoveVelocity								                : MC_MoveVelocity;

            MC_CamInSlave                                               : MC_CamIn;
            MC_CamOffsetRelativeSlave                                   : MC_OffsetRelative;
        END_VAR

        METHOD PUBLIC Axes_ctrl
    
            VAR_INPUT
                Axis                    : DB_ANY;
        
        
            END_VAR
        
            VAR_OUTPUT 
        
            END_VAR
        
            VAR
            LocAxis                                                     : REF_TO TO_PositioningAxis;
            LocAxisSlave                                                 : REF_TO TO_SynchronousAxis;
        
            END_VAR
        ;
        LocAxis	:= AsPositioningAxisRef(Axis);
        LocAxisSlave := AsSynchronousAxisRef(Axis);
        
        /////////////////////////////[MC_POWER]////////////////////////////////
        
        
        mcEnableAxis( Axis:=LocAxis^, 
                        Enable:= enable_axes 
                     );
        
        State_enable.Busy   := mcEnableAxis.Busy;
        State_enable.status := mcEnableAxis.Status;
        State_enable.Error  := mcEnableAxis.Error;
        State_enable.ErrorID :=mcEnableAxis.ErrorId;
        
        
        ///////////////////////////////[MC_STOP]////////////////////////////////
        
        mcStopAxis ( Axis:= LocAxis^,
                    Execute:= stop_axes);
        
        State_stop.Done :=mcStopAxis.Done;
        State_stop.Busy := mcStopAxis.Busy;
        State_stop.Error := mcStopAxis.Error;
        State_stop.ErrorID := mcStopAxis.ErrorId;
        
        //////////////////////////////[MC_MOVEABSOLUTE]/////////////////////////
        
        mcMoveAbsolute (Axis:=LocAxis^, 
                       Execute:= move_absolute );
        
        State_move_abs.Done :=mcMoveAbsolute.Done;
        State_move_abs.Busy := mcMoveAbsolute.Busy;
        State_move_abs.Error := mcMoveAbsolute.Error;
        State_move_abs.ErrorID := mcMoveAbsolute.ErrorId;
        
        //////////////////////////////[MC_MOVERELATIVE]/////////////////////////
        
        mcMoveRelative (Axis:=LocAxis^, 
                       Execute:= move_relative );
        
        State_move_rel.Done :=mcMoveRelative.Done;
        State_move_rel.Busy := mcMoveRelative.Busy;
        State_move_rel.Error := mcMoveRelative.Error;
        State_move_rel.ErrorID := mcMoveRelative.ErrorId;
        
        /////////////////////////////[MC_RESET]////////////////////////////////
        
        mcResetAxis (Axis:=LocAxis^, 
                    Execute:= reset_axes );
        
        State_reset.Done  := mcResetAxis.Done;
        State_reset.Busy  := mcResetAxis.Busy;
        State_reset.Error  := mcResetAxis.Error;
        State_reset.ErrorID  := mcResetAxis.ErrorID;
        
        ////////////////////////////[MC_HOME]////////////////////////////////////
        
        mcHomeAxis (Axis:=LocAxis^, 
                    Execute:= home_axes);
        
        State_home.Done  := mcHomeAxis.Done;
        State_home.Busy  := mcHomeAxis.Busy;
        State_home.Error  := mcResetAxis.Error;
        State_home.ErrorID  := mcHomeAxis.ErrorID;
        
        /////////////////////////////[MC_CamOut]////////////////////////////////
        
        mcCamOut   ( Slave := LocAxisSlave^, 
                    Execute:= cam_out );
        
        State_cam_out.Done      := mcCamOut.Done;
        State_cam_out.Busy      := mcCamOut.Busy;
        State_cam_out.Error     := mcCamOut.Error;
        State_cam_out.ErrorID   := mcCamOut.ErrorID;
        
        //////////////////////////////[MC_MoveJog]//////////////////////////////
        
        mcJogAxis (Axis:=LocAxis^);
        
        State_move_jog.InVelocity   :=mcJogAxis.InVelocity;
        State_move_jog.Busy         := mcJogAxis.Busy;
        State_move_jog.Error        :=mcJogAxis.Error;
        State_move_jog.ErrorID      := mcJogAxis.ErrorID;
        
        END_METHOD

        METHOD PUBLIC Cam_segment_scale

            VAR_INPUT
                AX_MASTER     : DB_ANY;
                AX_SLAVE       :DB_ANY;
                CAM_PROFILE    : DB_ANY;
                Change_Offset  : BOOL;
                Correction     : LREAL;
                Segment        : ARRAY [1..2] OF CamSegmentLimit;
            END_VAR

            VAR
                Step_Change_Offset  : INT;
                Master :    REF_TO TO_PositioningAxis;
                Slave  :    REF_TO TO_SynchronousAxis;
                Cam    :    REF_TO TO_Cam;
                SyncSlave   :   BOOL;
                StartChangeOffset   : BOOL;
                OffsetSlave :   LREAL;
                SlaveScaling    : LREAL;
                SyncPosMaster   : LREAL;
                SyncMode        : DINT;
            END_VAR

    IF master= NULL OR slave = NULL OR cam=NULL THEN
        master := AsPositioningAxisRef(AX_MASTER);
        slave := AsSynchronousAxisRef(AX_SLAVE);
        cam   := AsCamRef(CAM_PROFILE);
    END_IF;

        CASE Step_Change_Offset OF

         0: 
            IF Change_Offset THEN
                Step_Change_Offset := Step_Change_Offset +1;
            END_IF;
        1: 
            IF Master^.Position > Segment[1].Segment_Start 
            AND  Master^.Position < Segment[1].Segment_End 
            THEN  
                IF Change_Offset THEN
                    StartChangeOffset:= TRUE;
                        IF MC_CamOffsetRelativeSlave.Busy THEN
                            Step_Change_Offset:= Step_Change_Offset +1;
                        END_IF;
                ELSE
                    SyncMode :=1;
                    SyncPosMaster := Segment[1].Segment_Start + 0.001;
                    OffsetSlave := 0;
                    SlaveScaling := (Segment[1].Segment_End + Correction)/Segment[1].Segment_End;
                    SyncSlave := TRUE;
                END_IF;
            END_IF;
        2: IF Change_Offset THEN
                StartChangeOffset:= FALSE;
            ELSE
                IF Master^.Position > Segment[2].Segment_Start 
                AND Master^.Position < Segment[2].Segment_End
                THEN
                    SyncMode := 1;
                    SyncPosMaster := Segment[2].Segment_Start + 0.001;
                    OffsetSlave := Correction;
                    SlaveScaling := (Segment[2].Segment_End + Correction)/Segment[2].Segment_End;
                    SyncSlave := TRUE;
                    Step_Change_Offset:= Step_Change_Offset +1;
                ELSE
                    SyncSlave := FALSE;
                END_IF;
            END_IF;
            
        
        END_CASE;
        

        MC_CamInSlave(Master:=Master^,
                        Slave:=Slave^,
                        Cam:=Cam^,
                        Execute:=SyncSlave,
                        MasterOffset:=0.0,
                        SlaveOffset:=OffsetSlave,
                        MasterScaling:=1.0,
                        SlaveScaling:=SlaveScaling,
                        MasterSyncPosition:=SyncPosMaster,
                        MasterStartDistance:=10.001,
                        SyncProfileReference:=SyncMode,
                        ApplicationMode:=2);

        MC_CamOffsetRelativeSlave(Slave:=Slave^,
                        Execute:=StartChangeOffset,
                        Offset:=Correction,
                        ProfileReference:=2,
                        OffsetDistance:=50.0,
                        StartPosition:=100.0);
                                

         END_METHOD
    END_CLASS

END_NAMESPACE

