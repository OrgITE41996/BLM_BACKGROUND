
USING Siemens.Simatic.S71500.MotionControl.Native;
USING  _TYPE;
USING _CONST_VAR_DEF;
USING _STATE_CONST;

FUNCTION_BLOCK _Axes_ctrl_fb


    
    VAR_INPUT
        Axis                    : DB_ANY;

        enable_axes             : BOOL;
        stop_axes               : BOOL;
        move_absolute           : BOOL;
        move_relative           : BOOL;
        reset_axes              : BOOL;
        home_axes               : BOOL; 
        cam_out                 : BOOL;

        stop_Axis_Param     :    StopAxisParamIn;
        move_abs_param      :    MoveAbsoluteParamIn;
        move_rel_param      :    MoveRelativeParamIn;
        reset_axis_param    :    ResetAxisParamIn;
        home_axis_param     :    HomeAxisParamIn;
        cam_out_param       :    CamOutparamIn;
        move_jog_param      :    JogAxisParamIn;

    END_VAR

    VAR_OUTPUT 
        State_enable   : EnableAxisParam;
        State_stop     : StopAxisParamOut;
        State_move_abs : MoveAbsoluteParamOut;
        State_move_rel : MoveRelativeParamOut;
        State_reset    : ResetAxisParamOut;
        State_home     : HomeAxisParamOut;
        State_cam_out  : CamOutparamOut;
        State_move_jog : JogAxisParamOut;

    END_VAR

    VAR
    LocAxis                                                     : REF_TO TO_PositioningAxis;
    LocAxisSlave                                                 : REF_TO TO_SynchronousAxis;
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

END_FUNCTION_BLOCK