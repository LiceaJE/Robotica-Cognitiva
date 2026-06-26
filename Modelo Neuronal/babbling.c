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

FILE *archivo_dataset;

void inicializar_dataset() {
    archivo_dataset = fopen("dataset_babbling.csv", "w");
    if (archivo_dataset != NULL) {

        fprintf(archivo_dataset, 
            "DistIzq_t,DistDer_t,LuzIzq_t,LuzDer_t,SueloIzq_t,SueloDer_t,Bat_t,Aburr_t,"
            "Accion,"
            "DistIzq_t1,DistDer_t1,LuzIzq_t1,LuzDer_t1,SueloIzq_t1,SueloDer_t1\n");
    }
}


int main() {
    wb_robot_init();
    srand(time(NULL));
    
    inicializar_dataset();
    
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
    
    float battery_drive = 100.0;
    float curiosity_drive = 0.0;
    
    double prev_dist_izq = 0, prev_dist_der = 0;
    double prev_luz_izq = 0,  prev_luz_der = 0;
    double prev_suelo_izq = 0, prev_suelo_der = 0;
    float prev_bat = 0, prev_aburr = 0;
    int prev_accion = -1; 
    
    printf(">> INICIANDO BALBUCEO MOTOR (MOTOR BABBLING) <<\n");
    
    while (wb_robot_step(TIME_STEP) != -1) {
        
        if (battery_drive > 0.0) battery_drive -= 0.08; 
        if (curiosity_drive < 100.0) curiosity_drive += 0.05;
        
        while (wb_receiver_get_queue_length(receiver) > 0) {
            const char *buffer = wb_receiver_get_data(receiver);
            if (strcmp(buffer, "LATA") == 0) battery_drive = 100.0;
            wb_receiver_next_packet(receiver);
        }
        
        if (battery_drive <= 0.0) {
            char mensaje_muerte[] = "MUERTO";
            wb_emitter_send(emitter, mensaje_muerte, sizeof(mensaje_muerte));
            battery_drive = 100.0;
            curiosity_drive = 0.0; 
            continue; 
        }

        double dist_der = wb_distance_sensor_get_value(ps0);
        double dist_izq = wb_distance_sensor_get_value(ps7);
        double luz_der  = wb_light_sensor_get_value(ls0);
        double luz_izq  = wb_light_sensor_get_value(ls7);
        double suelo_der = wb_distance_sensor_get_value(gs0);
        double suelo_izq = wb_distance_sensor_get_value(gs2);


        if (prev_accion != -1 && archivo_dataset != NULL) {
            fprintf(archivo_dataset, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    prev_dist_izq, prev_dist_der, prev_luz_izq, prev_luz_der, prev_suelo_izq, prev_suelo_der, prev_bat, prev_aburr,
                    prev_accion,
                    dist_izq, dist_der, luz_izq, luz_der, suelo_izq, suelo_der);
            fflush(archivo_dataset); 
        }

        int accion = rand() % 3; 
        
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
        
        prev_dist_izq = dist_izq; prev_dist_der = dist_der;
        prev_luz_izq = luz_izq;   prev_luz_der = luz_der;
        prev_suelo_izq = suelo_izq; prev_suelo_der = suelo_der;
        prev_bat = battery_drive; prev_aburr = curiosity_drive;
        prev_accion = accion;
    }

    if (archivo_dataset) fclose(archivo_dataset);
    wb_robot_cleanup();
    return 0;
}