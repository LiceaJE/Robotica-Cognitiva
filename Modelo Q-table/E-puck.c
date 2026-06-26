#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <math.h>

#define TIME_STEP 64

// --- NUEVA CONFIGURACIÓN EXPANDIDA DE LA Q-TABLE (16 ESTADOS) ---
#define NUM_ESTADOS_SENSORIALES 8  // 0-3: Luz/Suelo (Libre), 4-7: Obstáculos (Paredes)
#define NUM_ESTADOS_INTERNOS 2     // 0: Modo Batería (Hambre), 1: Modo Curiosidad (Aburrido)
#define NUM_ESTADOS (NUM_ESTADOS_SENSORIALES * NUM_ESTADOS_INTERNOS) 

#define NUM_ACCIONES 3  // 0: Avanzar, 1: Girar Derecha, 2: Girar Izquierda

#define EPSILON 0.05       
#define ALPHA 0.1          
#define GAMMA 0.9          
#define UMBRAL_DIST 100.0  // Distancia para activar la esquiva de paredes (ajusta si choca mucho)
#define UMBRAL_LUZ 1000     
#define UMBRAL_NEGRO 400.0  

float curiosity_drive = 0.0;    
float battery_drive = 100.0;       
#define UMBRAL_ESTRES 40.0   
#define UMBRAL_ABURRIMIENTO 70.0 

float q_table[NUM_ESTADOS][NUM_ACCIONES] = {0};

void guardar_q_table() {
    FILE *archivo = fopen("q_table.txt", "w");
    if (archivo == NULL) return;
    fprintf(archivo, "AVANZAR \tGIRAR_DER \tGIRAR_IZQ\n");
    for (int s = 0; s < NUM_ESTADOS; s++) {
        for (int a = 0; a < NUM_ACCIONES; a++) {
            fprintf(archivo, "%8.2f\t", q_table[s][a]);
        }
        fprintf(archivo, "\n");
    }
    fclose(archivo);
}

void cargar_q_table() {
    FILE *archivo = fopen("q_table.txt", "r");
    if (archivo == NULL) return;
    char buffer_cabecera[256];
    if (fgets(buffer_cabecera, sizeof(buffer_cabecera), archivo) == NULL) { fclose(archivo); return; }
    for (int s = 0; s < NUM_ESTADOS; s++) {
        for (int a = 0; a < NUM_ACCIONES; a++) {
            if (fscanf(archivo, "%f", &q_table[s][a]) != 1) { fclose(archivo); return; }
        }
    }
    fclose(archivo);
    printf(">> Q-Table cargada exitosamente (16 estados). ¡Mente cognitiva lista! <<\n");
}

int obtener_estado_cognitivo(double dist_izq, double dist_der, double luz_izq, double luz_der, double suelo_izq, double suelo_der, int estado_interno) {
    int izq_cerca = (dist_izq > UMBRAL_DIST) ? 1 : 0;
    int der_cerca = (dist_der > UMBRAL_DIST) ? 1 : 0;
    
    int sub_estado = 0;

    // --- PRIORIDAD 1: EVITAR OBSTÁCULOS (Sub-estados del 4 al 7) ---
    if (izq_cerca || der_cerca) {
        if (izq_cerca && !der_cerca) sub_estado = 4; // Pared a la izquierda
        else if (!izq_cerca && der_cerca) sub_estado = 5; // Pared a la derecha
        else if (dist_izq > 350.0 && dist_der > 350.0) sub_estado = 7; // Acorralado / Choque inminente
        else sub_estado = 6; // Pared al centro
    }
    else {
        if (estado_interno == 0) {
            // PRIORIDAD 2: MODO HAMBRE (Nos guía la luz de las latas)
            int luz_izq_detectada = (luz_izq < UMBRAL_LUZ) ? 1 : 0;
            int luz_der_detectada = (luz_der < UMBRAL_LUZ) ? 1 : 0;
            
            if (!luz_izq_detectada && !luz_der_detectada) sub_estado = 0; 
            else if (luz_izq_detectada && !luz_der_detectada)  sub_estado = 1; 
            else if (!luz_izq_detectada && luz_der_detectada)  sub_estado = 2; 
            else sub_estado = 3; 
        } 
        else {
            // PRIORIDAD 3: MODO CURIOSIDAD (Nos guía el suelo negro)
            int negro_izq = (suelo_izq < UMBRAL_NEGRO) ? 1 : 0;
            int negro_der = (suelo_der < UMBRAL_NEGRO) ? 1 : 0;
            
            if (!negro_izq && !negro_der) sub_estado = 0; 
            else if (negro_izq && !negro_der)  sub_estado = 1; 
            else if (!negro_izq && negro_der)  sub_estado = 2; 
            else sub_estado = 3; 
        }
    }

    return sub_estado + (estado_interno * 8);
}

int main() {
    wb_robot_init();
    srand(time(NULL));
    cargar_q_table();
    
    WbDeviceTag emitter = wb_robot_get_device("emitter");
    WbDeviceTag ps0 = wb_robot_get_device("ps0"); 
    WbDeviceTag ps7 = wb_robot_get_device("ps7"); 
    wb_distance_sensor_enable(ps0, TIME_STEP);
    wb_distance_sensor_enable(ps7, TIME_STEP);

    WbDeviceTag ls0 = wb_robot_get_device("ls0"); 
    WbDeviceTag ls7 = wb_robot_get_device("ls7"); 
    wb_light_sensor_enable(ls0, TIME_STEP);
    wb_light_sensor_enable(ls7, TIME_STEP);

    WbDeviceTag gs0 = wb_robot_get_device("gs0"); 
    WbDeviceTag gs2 = wb_robot_get_device("gs2"); 
    wb_distance_sensor_enable(gs0, TIME_STEP);
    wb_distance_sensor_enable(gs2, TIME_STEP);

    WbDeviceTag receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, TIME_STEP);

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    int estado_actual = 0;
    long contador_pasos = 0;

    while (wb_robot_step(TIME_STEP) != -1) {
        
        if (battery_drive > 0.0)    battery_drive   -= 0.08; 
        if (curiosity_drive < 100.0) curiosity_drive += 0.05; 
        
        int estado_interno = 0; 
        if (curiosity_drive > UMBRAL_ABURRIMIENTO && battery_drive > UMBRAL_ESTRES) {
            estado_interno = 1; 
        }

        if (battery_drive <= 0.0) {
            printf("\n[SISTEMA FATAL] Bateria agotada. Guardando memoria...\n");
            wb_motor_set_velocity(left_motor, 0.0);
            wb_motor_set_velocity(right_motor, 0.0);
            guardar_q_table();
            char mensaje_muerte[] = "MUERTO";
            wb_emitter_send(emitter, mensaje_muerte, sizeof(mensaje_muerte));
            battery_drive = 100.0;
            curiosity_drive = 0.0; 
            continue; 
        }

        int detecta_lata = 0;
        while (wb_receiver_get_queue_length(receiver) > 0) {
            const char *buffer = wb_receiver_get_data(receiver);
            if (strcmp(buffer, "LATA") == 0) detecta_lata = 1; 
            wb_receiver_next_packet(receiver);
        }

        double dist_der = wb_distance_sensor_get_value(ps0);
        double dist_izq = wb_distance_sensor_get_value(ps7);
        double luz_der  = wb_light_sensor_get_value(ls0);
        double luz_izq  = wb_light_sensor_get_value(ls7);
        double suelo_der = wb_distance_sensor_get_value(gs0);
        double suelo_izq = wb_distance_sensor_get_value(gs2);

        estado_actual = obtener_estado_cognitivo(dist_izq, dist_der, luz_izq, luz_der, suelo_izq, suelo_der, estado_interno);
        
        int accion;
        if ((float)rand() / RAND_MAX < EPSILON) {
            accion = rand() % NUM_ACCIONES; 
        } else {
            int mejor_accion = 0;
            for (int a = 1; a < NUM_ACCIONES; a++) {
                if (q_table[estado_actual][a] > q_table[estado_actual][mejor_accion]) mejor_accion = a;
            }
            accion = mejor_accion;
        }

        if (accion == 0) {        
            wb_motor_set_velocity(left_motor, 6.28);
            wb_motor_set_velocity(right_motor, 6.28);
        } else if (accion == 1) { 
            wb_motor_set_velocity(left_motor, 6.28);
            wb_motor_set_velocity(right_motor, -6.28);
        } else {                  
            wb_motor_set_velocity(left_motor, -6.28);
            wb_motor_set_velocity(right_motor, 6.28);
        }
        
        float recompensa = 0.0;
        float deficit_biologico = 100.0 - battery_drive;

        if (detecta_lata) {
            battery_drive = 100.0;
            recompensa = 250.0; 
            printf(">> [INTRINSECO] ¡Lata devorada! Energía al 100%% <<\n");
        } 
        else {
            
            int es_estado_pared = (estado_actual >= 4 && estado_actual <= 7) || (estado_actual >= 12 && estado_actual <= 15);
            if (es_estado_pared) {
                int estado_relativo = estado_actual % 8;
                if (estado_relativo == 4) {
                    recompensa = (accion == 1) ? 40.0 : -30.0; 
                } 
                else if (estado_relativo == 5) {
                    recompensa = (accion == 2) ? 40.0 : -30.0; 
                } 
                else if (estado_relativo == 7) {
                    if (accion == 0) {
                        recompensa = -150.0;
                    } else {
                        recompensa = 100.0;  
                    }
                } 
                else {
                    recompensa = (accion == 0) ? -50.0 : 20.0; 
                }
            }
            else {
                if (estado_interno == 0) {
                    // MODO METABÓLICO (Busca luz)
                    if (estado_actual == 3)      recompensa = (accion == 0) ? (30.0 + deficit_biologico * 0.5) : -20.0;
                    else if (estado_actual == 1) recompensa = (accion == 2) ? 25.0 : -10.0;
                    else if (estado_actual == 2) recompensa = (accion == 1) ? 25.0 : -10.0;
                    else                         recompensa = (accion == 0) ? 5.0 : -2.0;
                } 
                else {
                    // MODO CURIOSIDAD (Busca suelo negro)
                    if (estado_actual == 11) { 
                        if (accion == 0) {
                            recompensa = -15.0; 
                        } else {
                            recompensa = 50.0; 
                            curiosity_drive -= 2.5; 
                            if (curiosity_drive < 0.0) curiosity_drive = 0.0;
                            printf("[ALOSTASIS] Divirtiéndose en el cuadro negro. Aburrimiento: %.1f%%\n", curiosity_drive);
                        }
                    } 
                    else if (estado_actual == 9)  recompensa = (accion == 2) ? 25.0 : -10.0; // Orientarse al negro
                    else if (estado_actual == 10) recompensa = (accion == 1) ? 25.0 : -10.0;
                    else                          recompensa = (accion == 0) ? 8.0 : -2.0; 
                }
            }
        }
        
        double nueva_dist_der = wb_distance_sensor_get_value(ps0);
        double nueva_dist_izq = wb_distance_sensor_get_value(ps7);
        double nueva_luz_der  = wb_light_sensor_get_value(ls0);
        double nueva_luz_izq  = wb_light_sensor_get_value(ls7);
        double nueva_suelo_der = wb_distance_sensor_get_value(gs0);
        double nueva_suelo_izq = wb_distance_sensor_get_value(gs2);
        
        int nuevo_estado = obtener_estado_cognitivo(nueva_dist_izq, nueva_dist_der, nueva_luz_izq, nueva_luz_der, nueva_suelo_izq, nueva_suelo_der, estado_interno);
        
        float max_futuro = q_table[nuevo_estado][0];
        for (int a = 1; a < NUM_ACCIONES; a++) {
            if (q_table[nuevo_estado][a] > max_futuro) max_futuro = q_table[nuevo_estado][a];
        }
        
        q_table[estado_actual][accion] += ALPHA * (recompensa + GAMMA * max_futuro - q_table[estado_actual][accion]);
        //printf("Sensores Distancia -> Izq (ps7): %.1f | Der (ps0): %.1f\n", dist_izq, dist_der);
        
        contador_pasos++;
        if (contador_pasos % 100 == 0) {
            printf("[MÉTRICAS] Bat: %.1f%% | Aburrimiento: %.1f%% | Modo: %s\n", 
                   battery_drive, curiosity_drive, (estado_interno == 0)?"METABÓLICO":"EXPLORATORIO (Suelo)");
            guardar_q_table();
        }
    }

    wb_robot_cleanup();
    return 0;
}