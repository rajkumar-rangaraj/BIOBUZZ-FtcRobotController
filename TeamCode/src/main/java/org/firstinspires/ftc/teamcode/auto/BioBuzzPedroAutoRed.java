package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Disabled   // v1 (windmill robot): parked 21 Nov 2026, see v2/

@Autonomous(name = "v1 BioBuzz AUTO RED", group = "BioBuzz", preselectTeleOp = "v1 BioBuzz TeleOp")
public class BioBuzzPedroAutoRed extends BioBuzzPedroAutoBase {
    @Override
    protected AutoField.Alliance alliance() {
        return AutoField.Alliance.RED;
    }
}