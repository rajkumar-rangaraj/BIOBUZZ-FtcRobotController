package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BioBuzz HIVE AUTO RED", group = "BioBuzz", preselectTeleOp = "BioBuzz TeleOp")
public class BioBuzzHiveAutoRed extends BioBuzzHiveAutoBase {
    @Override
    protected AutoField.Alliance alliance() {
        return AutoField.Alliance.RED;
    }
}
