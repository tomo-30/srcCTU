MODULE SERVER

    !////////////////
    ! GLOBAL VARIABLES
    !////////////////

    ! Robot configuration
    PERS tooldata currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    PERS speeddata currentSpeed;
    PERS zonedata currentZone;

    ! Clock Synchronization
    PERS bool startLog:=TRUE;
    PERS bool startRob:=TRUE;

    ! Mutex between logger and changing the tool and work objects
    PERS bool frameMutex:=FALSE;

    ! PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    VAR num instructionCode;
    VAR num params{10};
    VAR num nParams;

    PERS string ipController:= "192.168.125.1";  ! robot default IP
    !PERS string ipController:= "127.0.0.1";        ! local IP for testing in simulation
    PERS num serverPort:= 5000;

    ! Motion of the robot
    VAR robtarget cartesianTarget;
    VAR jointtarget jointsTarget;
    VAR bool moveCompleted;  ! Set to true after finishing a Move instruction.

    ! Buffered move variables
    CONST num MAX_BUFFER := 512;
    VAR num BUFFER_POS := 0;
    VAR robtarget bufferTargets{MAX_BUFFER};
    VAR speeddata bufferSpeeds{MAX_BUFFER};

    ! External axis position variables
    VAR extjoint externalAxis;

    ! Circular move buffer
    VAR robtarget circPoint;

    ! Correct Instruction Execution and possible return values
    VAR num ok;
    CONST num SERVER_BAD_MSG := 0;
    CONST num SERVER_OK := 1;

    ! ========== OPTIONAL: 状態フラグ（HALT後に参照したい場合） ==========
    VAR bool gHaltReq := FALSE;

    !////////////////
    ! LOCAL METHODS
    !////////////////

    ! Parse the message received from a PC
    PROC ParseMsg(string msg)
        VAR bool auxOk;
        VAR num ind:=1;
        VAR num newInd;
        VAR num length;
        VAR num indParam:=1;
        VAR string subString;
        VAR bool end := FALSE;

        ! Find the end character
        length := StrMatch(msg,1,"#");
        IF length > StrLen(msg) THEN
            ! Corrupt message
            nParams := -1;
        ELSE
            ! Read Instruction code
            newInd := StrMatch(msg,ind," ") + 1;
            subString := StrPart(msg,ind,newInd - ind - 1);
            auxOk:= StrToVal(subString, instructionCode);
            IF auxOk = FALSE THEN
                ! Impossible to read instruction code
                nParams := -1;
            ELSE
                ind := newInd;
                ! Read all instruction parameters (maximum of 8+)
                WHILE end = FALSE DO
                    newInd := StrMatch(msg,ind," ") + 1;
                    IF newInd > length THEN
                        end := TRUE;
                    ELSE
                        subString := StrPart(msg,ind,newInd - ind - 1);
                        auxOk := StrToVal(subString, params{indParam});
                        indParam := indParam + 1;
                        ind := newInd;
                    ENDIF
                ENDWHILE
                nParams:= indParam - 1;
            ENDIF
        ENDIF
    ENDPROC

    ! Handshake: create socket, bind, listen, accept
    PROC ServerCreateAndConnect(string ip, num port)
        VAR string clientIP;
        SocketCreate serverSocket;
        SocketBind serverSocket, ip, port;
        SocketListen serverSocket;
        TPWrite "SERVER: Server waiting for incoming connections ...";
        WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
            SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
            IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
                TPWrite "SERVER: Problem serving an incoming connection.";
                TPWrite "SERVER: Try reconnecting.";
            ENDIF
            WaitTime 0.5;
        ENDWHILE
        TPWrite "SERVER: Connected to IP " + clientIP;
    ENDPROC

    ! Initialize default Tool/Wobj/Zone/Speed
    PROC Initialize()
        currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
        currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
        currentSpeed := [100, 50, 0, 0];
        currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03];  ! z0
        jointsTarget := CJointT();
        externalAxis := jointsTarget.extax;
    ENDPROC

    ! ===== HALT: ソフト停止（CASE 11 用） =====
    PROC DoHalt()
        StopMove;
        WaitTime 0.02;
        gHaltReq := TRUE;
    ENDPROC

    ! ===== MoveL_ZYX: XYZ + OrientZYX(rz,ry,rx)（CASE 21 用） =====
    PROC DoMoveL_ZYX()
        VAR num x; 
        VAR num y; 
        VAR num z;
        VAR num rz;
        VAR num ry;
        VAR num rx;
        VAR robtarget t;
    
        ! 引数は x y z rz ry rx の6個
        IF nParams <> 6 THEN
            ok := SERVER_BAD_MSG;
            RETURN;
        ENDIF
    
        ! 受信パラメータ取得（必ず ASCII 半角の数値）
        x  := params{1};
        y  := params{2};
        z  := params{3};
        rz := params{4};  ! Z [deg]
        ry := params{5};  ! Y [deg]
        rx := params{6};  ! X [deg]
    
        ! robtarget を一括生成（conf は暫定ゼロ、ext は既存の externalAxis を使用）
        t := [[x, y, z],
              OrientZYX(rz, ry, rx),
              [0,0,0,0],
              externalAxis];
    
        moveCompleted := FALSE;
        MoveL t, currentSpeed, currentZone, currentTool \WObj:=currentWobj;
        moveCompleted := TRUE;
    
        ok := SERVER_OK;
    ENDPROC



    !////////////////////////
    ! SERVER: Main procedure
    !////////////////////////
    PROC server_entry()
        VAR string receivedString;
        VAR string sendString;
        VAR string addString;
        VAR bool connected;
        VAR bool reconnected;
        VAR robtarget cartesianPose;
        VAR jointtarget jointsPose;

        ConfL \Off;
        SingArea \Wrist;
        moveCompleted:= TRUE;

        Initialize;

        connected:=FALSE;
        ServerCreateAndConnect ipController,serverPort;
        connected:=TRUE;

        WHILE TRUE DO
            ok:=SERVER_OK;
            reconnected:=FALSE;
            addString := "";

            SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
            ParseMsg receivedString;

            TEST instructionCode

                CASE 0:  ! Ping
                    IF nParams = 0 THEN ok := SERVER_OK; ELSE ok := SERVER_BAD_MSG; ENDIF

                CASE 1:  ! Cartesian Move (x y z qx qy qz qw)
                    IF nParams = 7 THEN
                        cartesianTarget := [[params{1},params{2},params{3}],
                                            [params{4},params{5},params{6},params{7}],
                                            [0,0,0,0], externalAxis];
                        ok := SERVER_OK;
                        moveCompleted := FALSE;
                        MoveL cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                        moveCompleted := TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 2:  ! Joint Move
                    IF nParams = 6 THEN
                        jointsTarget := [[params{1},params{2},params{3},params{4},params{5},params{6}], externalAxis];
                        ok := SERVER_OK;
                        moveCompleted := FALSE;
                        MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                        moveCompleted := TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 3:  ! Get Cartesian
                    IF nParams = 0 THEN
                        cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);
                        addString := NumToStr(cartesianPose.trans.x,2) + " ";
                        addString := addString + NumToStr(cartesianPose.trans.y,2) + " ";
                        addString := addString + NumToStr(cartesianPose.trans.z,2) + " ";
                        addString := addString + NumToStr(cartesianPose.rot.q1,3) + " ";
                        addString := addString + NumToStr(cartesianPose.rot.q2,3) + " ";
                        addString := addString + NumToStr(cartesianPose.rot.q3,3) + " ";
                        addString := addString + NumToStr(cartesianPose.rot.q4,3);
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 4:  ! Get Joints
                    IF nParams = 0 THEN
                        jointsPose := CJointT();
                        addString := NumToStr(jointsPose.robax.rax_1,2) + " ";
                        addString := addString + NumToStr(jointsPose.robax.rax_2,2) + " ";
                        addString := addString + NumToStr(jointsPose.robax.rax_3,2) + " ";
                        addString := addString + NumToStr(jointsPose.robax.rax_4,2) + " ";
                        addString := addString + NumToStr(jointsPose.robax.rax_5,2) + " ";
                        addString := addString + NumToStr(jointsPose.robax.rax_6,2);
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 5:  ! Get external axis positions
                    IF nParams = 0 THEN
                        jointsPose := CJointT();
                        addString := StrPart(NumToStr(jointsTarget.extax.eax_a,2),1,8) + " ";
                        addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_b,2),1,8) + " ";
                        addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_c,2),1,8) + " ";
                        addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_d,2),1,8) + " ";
                        addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_e,2),1,8) + " ";
                        addString := addString + StrPart(NumToStr(jointsTarget.extax.eax_f,2),1,8);
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 6:  ! Set Tool (qx qy qz qw)
                    IF nParams = 7 THEN
                        WHILE (frameMutex) DO
                            WaitTime .01;
                        ENDWHILE
                        frameMutex:= TRUE;
                        currentTool.tframe.trans.x:=params{1};
                        currentTool.tframe.trans.y:=params{2};
                        currentTool.tframe.trans.z:=params{3};
                        currentTool.tframe.rot.q1:=params{4};
                        currentTool.tframe.rot.q2:=params{5};
                        currentTool.tframe.rot.q3:=params{6};
                        currentTool.tframe.rot.q4:=params{7};
                        ok := SERVER_OK;
                        frameMutex:= FALSE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 7:  ! Set WorkObject (qx qy qz qw)
                    IF nParams = 7 THEN
                        currentWobj.oframe.trans.x:=params{1};
                        currentWobj.oframe.trans.y:=params{2};
                        currentWobj.oframe.trans.z:=params{3};
                        currentWobj.oframe.rot.q1:=params{4};
                        currentWobj.oframe.rot.q2:=params{5};
                        currentWobj.oframe.rot.q3:=params{6};
                        currentWobj.oframe.rot.q4:=params{7};
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 8:  ! Set Speed
                    IF nParams = 4 THEN
                        currentSpeed.v_tcp:=params{1};
                        currentSpeed.v_ori:=params{2};
                        currentSpeed.v_leax:=params{3};
                        currentSpeed.v_reax:=params{4};
                        ok := SERVER_OK;
                    ELSEIF nParams = 2 THEN
                        currentSpeed.v_tcp:=params{1};
                        currentSpeed.v_ori:=params{2};
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 9:  ! Set Zone
                    IF nParams = 4 THEN
                        IF params{1}=1 THEN
                            currentZone.finep := TRUE;
                            currentZone.pzone_tcp := 0.0;
                            currentZone.pzone_ori := 0.0;
                            currentZone.zone_ori := 0.0;
                        ELSE
                            currentZone.finep := FALSE;
                            currentZone.pzone_tcp := params{2};
                            currentZone.pzone_ori := params{3};
                            currentZone.zone_ori := params{4};
                        ENDIF
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 11:  ! ====== HALT（新規）======
                    IF nParams = 0 THEN
                        DoHalt;
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 21:  ! ====== MoveL_ZYX（新規）======
                    DoMoveL_ZYX;

                CASE 30:  ! Add Cartesian to buffer
                    IF nParams = 7 THEN
                        cartesianTarget := [[params{1},params{2},params{3}],
                                            [params{4},params{5},params{6},params{7}],
                                            [0,0,0,0], externalAxis];
                        IF BUFFER_POS < MAX_BUFFER THEN
                            BUFFER_POS := BUFFER_POS + 1;
                            bufferTargets{BUFFER_POS} := cartesianTarget;
                            bufferSpeeds{BUFFER_POS} := currentSpeed;
                        ENDIF
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 31:  ! Clear Cartesian Buffer
                    IF nParams = 0 THEN
                        BUFFER_POS := 0;
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 32:  ! Get Buffer Size
                    IF nParams = 0 THEN
                        addString := NumToStr(BUFFER_POS,2);
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 33:  ! Execute buffered MoveL
                    IF nParams = 0 THEN
                        FOR i FROM 1 TO (BUFFER_POS) DO
                            MoveL bufferTargets{i}, bufferSpeeds{i}, currentZone, currentTool \WObj:=currentWobj ;
                        ENDFOR
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 34:  ! External Axis move
                    IF nParams = 6 THEN
                        externalAxis := [params{1},params{2},params{3},params{4},params{5},params{6}];
                        jointsTarget := CJointT();
                        jointsTarget.extax := externalAxis;
                        ok := SERVER_OK;
                        moveCompleted := FALSE;
                        MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                        moveCompleted := TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 35:  ! Set circPoint
                    IF nParams = 7 THEN
                        circPoint := [[params{1},params{2},params{3}],
                                      [params{4},params{5},params{6},params{7}],
                                      [0,0,0,0], externalAxis];
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 36:  ! MoveC to circPoint -> toPoint
                    IF nParams = 7 THEN
                        cartesianTarget := [[params{1},params{2},params{3}],
                                            [params{4},params{5},params{6},params{7}],
                                            [0,0,0,0], externalAxis];
                        MoveC circPoint, cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 98:  ! SysInfo
                    IF nParams = 0 THEN
                        addString := GetSysInfo(\SerialNo) + "*";
                        addString := addString + GetSysInfo(\SWVersion) + "*";
                        addString := addString + GetSysInfo(\RobotType);
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                CASE 99:  ! Close / Reconnect
                    IF nParams = 0 THEN
                        TPWrite "SERVER: Client has closed connection.";
                        connected := FALSE;
                        SocketClose clientSocket;
                        SocketClose serverSocket;
                        ServerCreateAndConnect ipController,serverPort;
                        connected := TRUE;
                        reconnected := TRUE;
                        ok := SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                    ENDIF

                DEFAULT:
                    TPWrite "SERVER: Illegal instruction code";
                    ok := SERVER_BAD_MSG;

            ENDTEST

            ! Compose ACK
            IF connected = TRUE THEN
                IF reconnected = FALSE THEN
                    IF SocketGetStatus(clientSocket) = SOCKET_CONNECTED THEN
                        sendString := NumToStr(instructionCode,0);
                        sendString := sendString + " " + NumToStr(ok,0);
                        sendString := sendString + " " + addString;
                        SocketSend clientSocket \Str:=sendString;
                    ENDIF
                ENDIF
            ENDIF

        ENDWHILE

        ERROR (LONG_JMP_ALL_ERR)
            TPWrite "SERVER: ------";
            TPWrite "SERVER: Error Handler:" + NumtoStr(ERRNO,0);
            TEST ERRNO
                CASE ERR_SOCK_CLOSED:
                    TPWrite "SERVER: Lost connection to the client.";
                    TPWrite "SERVER: Closing socket and restarting.";
                    TPWrite "SERVER: ------";
                    connected:=FALSE;
                    SocketClose clientSocket;
                    SocketClose serverSocket;
                    ServerCreateAndConnect ipController,serverPort;
                    reconnected:= FALSE;
                    connected:= TRUE;
                    RETRY;

                DEFAULT:
                    TPWrite "SERVER: Unknown error.";
                    TPWrite "SERVER: Closing socket and restarting.";
                    TPWrite "SERVER: ------";
                    connected:=FALSE;
                    SocketClose clientSocket;
                    SocketClose serverSocket;
                    ServerCreateAndConnect ipController,serverPort;
                    reconnected:= FALSE;
                    connected:= TRUE;
                    RETRY;
            ENDTEST

    ENDPROC

ENDMODULE
