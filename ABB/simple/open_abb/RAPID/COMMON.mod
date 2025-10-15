MODULE COMMON
    ! === 共有ターゲット/状態 ===
    PERS robtarget gTarget := [[350,0,300],[1,0,0,0],[0,0,1,0],[9E9,9E9,9E9,9E9]];
    PERS bool      gNewTarget := FALSE;
    PERS bool      gStopReq   := FALSE;
    PERS bool      gInMotion  := FALSE;

    ! === 速度・ゾーン（既存SERVERの currentSpeed/currentZone と整合を取るならここも同期）===
    PERS speeddata gVtcp := [100, 50, 0, 0];
    PERS zonedata  gZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03];

    ! === Tool/WObj（既存SERVERの currentTool/currentWobj と同期管理）===
    PERS tooldata gTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    PERS wobjdata gWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    PROC SetToolFrame(tx,ty,tz, qx,qy,qz,qw)
        gTool.tframe.trans := [tx,ty,tz];
        gTool.tframe.rot.q1 := qx;
        gTool.tframe.rot.q2 := qy;
        gTool.tframe.rot.q3 := qz;
        gTool.tframe.rot.q4 := qw;
    ENDPROC

    PROC SetWobjFrame(wx,wy,wz, qx,qy,qz,qw)
        gWobj.oframe.trans := [wx,wy,wz];
        gWobj.oframe.rot.q1 := qx;
        gWobj.oframe.rot.q2 := qy;
        gWobj.oframe.rot.q3 := qz;
        gWobj.oframe.rot.q4 := qw;
    ENDPROC
ENDMODULE
