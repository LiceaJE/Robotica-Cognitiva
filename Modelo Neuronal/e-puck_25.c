#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define TIME_STEP 64
#define PUERTO 5000
#define IP_SERVIDOR "127.0.0.1"

float curiosity_drive = 0.0;    
float battery_drive = 100.0;       

int main() {
    wb_robot_init();
    
    int sock = 0;
    struct sockaddr_in serv_addr;
    
    printf(">> Intentando conectar con el Servidor de Inteligencia (Python)... <<\n");
    
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n [ERROR] No se pudo crear el socket \n");
        return -1;
    }
    
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PUERTO);
    
    if (inet_pton(AF_INET, IP_SERVIDOR, &serv_addr.sin_addr) <= 0) {
        printf("\n [ERROR] Dirección IP inválida o no soportada \n");
        return -1;
    }
    
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("\n [ERROR FATAL] Python no está escuchando. Ejecuta 'server_cerebro.py' primero.\n");
        return -1;
    }
    
    printf(">> ¡CONECTADO AL CEREBRO EN PYTHON EXITOSAMENTE! <<\n");
    
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

    long contador_pasos = 0;

    while (wb_robot_step(TIME_STEP) != -1) {
        
        if (battery_drive > 0.0)     battery_drive   -= 0.08; 
        if (curiosity_drive < 100.0) curiosity_drive += 0.05; 

        if (battery_drive <= 0.0) {
            printf("\n[SISTEMA FATAL] Bateria agotada.\n");
            wb_motor_set_velocity(left_motor, 0.0);
            wb_motor_set_velocity(right_motor, 0.0);
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

        if (detecta_lata) {
            battery_drive = 100.0;
            printf(">> ¡Lata devorada! Energía restaurada al 100%% <<\n");
        }

        double dist_der = wb_distance_sensor_get_value(ps0);
        double dist_izq = wb_distance_sensor_get_value(ps7);
        double luz_der  = wb_light_sensor_get_value(ls0);
        double luz_izq  = wb_light_sensor_get_value(ls7);
        double suelo_der = wb_distance_sensor_get_value(gs0);
        double suelo_izq = wb_distance_sensor_get_value(gs2);

        if (suelo_izq < 400.0 || suelo_der < 400.0) {
            curiosity_drive -= 1.5;
            if (curiosity_drive < 0.0) curiosity_drive = 0.0;
        }

        char buffer_envio[256];
        sprintf(buffer_envio, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                dist_izq, dist_der, luz_izq, luz_der, suelo_izq, suelo_der, battery_drive, curiosity_drive);

        send(sock, buffer_envio, strlen(buffer_envio), 0);

        char buffer_respuesta[16] = {0};
        int bytes_recibidos = recv(sock, buffer_respuesta, 16, 0);
        
        if (bytes_recibidos <= 0) {
            printf("[ERROR] Se perdió la conexión con el Cerebro de Python.\n");
            break;
        }

        int accion_inteligente = atoi(buffer_respuesta);

        if (accion_inteligente == 0) {        
            wb_motor_set_velocity(left_motor, 6.28); // Avanzar
            wb_motor_set_velocity(right_motor, 6.28);
        } else if (accion_inteligente == 1) { 
            wb_motor_set_velocity(left_motor, 6.28); // Girar Derecha
            wb_motor_set_velocity(right_motor, -6.28);
        } else if (accion_inteligente == 2) {                  
            wb_motor_set_velocity(left_motor, -6.28); // Girar Izquierda
            wb_motor_set_velocity(right_motor, 6.28);
        }
        
        contador_pasos++;
        if (contador_pasos % 100 == 0) {
            printf("[VITALES] Energía: %.1f%% | Aburrimiento: %.1f%% | Acción Red: %d\n", 
                   battery_drive, curiosity_drive, accion_inteligente);
        }
    }

    close(sock);
    wb_robot_cleanup();
    return 0;
}