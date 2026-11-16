package org.firstinspires.ftc.teamcode.Autonomos;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "BIOBUZZ - ROJO 2 FLORES + PARK", group = "Competencia")
public class AutoRojoDosFlores extends BaseAuto {

    @Override
    protected Pose getStartPose() {
        return FieldRoutes.START_ROJO_DER;
    }

    @Override
    protected List<AutoStep> buildRoutine() {
        return Arrays.asList(
                // 1. Salir y disparar los 3 precargados
                irA(FieldRoutes.start_a_tiroInicial(), "Tiro Inicial"),
                disparar(2.0),

                // 2. Ir por Flor 1, recoger y disparar
                encenderIntake(),
                irA(FieldRoutes.tiroInicial_a_preFlor1(), "Curva a Flor 1"),
                irA(FieldRoutes.preFlor1_a_flor1(), "Agarrar Flor 1"),
                irA(FieldRoutes.flor1_a_preFlor1(), "Retroceder Flor 1"),
                apagarIntake(),
                disparar(2.0),

                // 3. Ir por Flor 2, recoger y disparar
                encenderIntake(),
                irA(FieldRoutes.preFlor1_a_preFlor2(), "Curva a Flor 2"),
                irA(FieldRoutes.preFlor2_a_flor2(), "Agarrar Flor 2"),
                irA(FieldRoutes.flor2_a_preFlor2(), "Retroceder Flor 2"),
                apagarIntake(),
                disparar(2.0),

                // 4. Estacionarse
                irA(FieldRoutes.preFlor2_a_park(), "Park")
        );
    }
}