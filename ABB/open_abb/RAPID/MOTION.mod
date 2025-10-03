MODULE MOTION
    PROC motion_entry()
        ConfL \On;
        SingArea \Wrist;
        TPWrite "MOTION task started";

        WHILE TRUE DO
            IF gStopReq THEN
                TPWrite "Stop request";
                StopMove;
                ClearPath;
                gInMotion := FALSE;
                gStopReq := FALSE;
            ENDIF

            IF (NOT gInMotion) AND gNewTarget THEN
                gNewTarget := FALSE;
                gInMotion := TRUE;
                TPWrite "MoveL start";
                MoveL gTarget, gVtcp, gZone, gTool \WObj:=gWobj;
                gInMotion := FALSE;
                TPWrite "MoveL done";
            ENDIF

            WaitTime 0.01;
        ENDWHILE
    ENDPROC
ENDMODULE
