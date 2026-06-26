#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>          
#include <time.h>
#include <math.h>

#define TIME_STEP 64
#define NUM_LATAS 7 // <-- Cambiado de 3 a 7 latas

#define LIMITE_MAX 0.90
#define LIMITE_MIN -0.90

int main() {
  wb_robot_init();
  srand(time(NULL));
  printf("[SUPERVISOR] Sistema Multi-Recurso (7 Latas) Inicializado en Arena 2x2.\n");

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("EPUCK_ENTRENANDO");
  if (robot_node == NULL) {
    printf("[SUPERVISOR] ERROR: No se encontro el DEF 'EPUCK_ENTRENANDO'.\n");
    wb_robot_cleanup();
    return 1;
  }
  WbFieldRef robot_trans = wb_supervisor_node_get_field(robot_node, "translation");
  const double pos_inicial_robot[3] = {0.381927, -0.382479, 0.0001};

  const char *nombres_latas[NUM_LATAS] = {"LATA1", "LATA2", "LATA3", "LATA4", "LATA5", "LATA6", "LATA7"};
  WbNodeRef nodos_latas[NUM_LATAS];
  WbFieldRef trans_latas[NUM_LATAS];
  WbFieldRef rot_latas[NUM_LATAS]; 

  for (int i = 0; i < NUM_LATAS; i++) {
    nodos_latas[i] = wb_supervisor_node_get_from_def(nombres_latas[i]);
    if (nodos_latas[i] == NULL) {
      printf("[SUPERVISOR] AVISO: No se encontro el nodo DEF '%s' en la escena.\n", nombres_latas[i]);
      trans_latas[i] = NULL;
      rot_latas[i] = NULL;
    } else {
      trans_latas[i] = wb_supervisor_node_get_field(nodos_latas[i], "translation");
      rot_latas[i] = wb_supervisor_node_get_field(nodos_latas[i], "rotation"); 
    }
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    const double *pos_robot = wb_supervisor_field_get_sf_vec3f(robot_trans);

    while (wb_receiver_get_queue_length(receiver) > 0) {
      const char *buffer = wb_receiver_get_data(receiver);
      
      if (strcmp(buffer, "MUERTO") == 0) {
        printf("[SUPERVISOR] Robot sin bateria detectado. Reiniciando a posicion inicial...\n");
        wb_supervisor_field_set_sf_vec3f(robot_trans, pos_inicial_robot);
        wb_supervisor_node_reset_physics(robot_node);
      }
      
      wb_receiver_next_packet(receiver);
    }

    for (int i = 0; i < NUM_LATAS; i++) {
      if (nodos_latas[i] == NULL || trans_latas[i] == NULL) continue; 

      const double *pos_lata = wb_supervisor_field_get_sf_vec3f(trans_latas[i]);

      double dx = pos_robot[0] - pos_lata[0];
      double dy = pos_robot[1] - pos_lata[1];
      double dz = pos_robot[2] - pos_lata[2];
      double distancia = sqrt(dx*dx + dy*dy + dz*dz);

      if (distancia < 0.1) {
        printf("[SUPERVISOR] >> ¡Contacto con %s detectado a %.3fm! Recargando... <<\n", nombres_latas[i], distancia);
        
        char mensaje[] = "LATA";
        wb_emitter_send(emitter, mensaje, sizeof(mensaje));

        double nueva_pos[3];
        nueva_pos[0] = pos_lata[0]; 
        
        nueva_pos[1] = ((double)rand() / RAND_MAX) * (LIMITE_MAX - LIMITE_MIN) + LIMITE_MIN;
        nueva_pos[2] = ((double)rand() / RAND_MAX) * (LIMITE_MAX - LIMITE_MIN) + LIMITE_MIN;

        const double nueva_rot[4] = {0.0, 0.0, 1.0, 0.0};

        wb_supervisor_field_set_sf_vec3f(trans_latas[i], nueva_pos);
        if (rot_latas[i] != NULL) {
          wb_supervisor_field_set_sf_rotation(rot_latas[i], nueva_rot);
        }
        
        wb_supervisor_node_reset_physics(nodos_latas[i]);
        
        printf("[SUPERVISOR] %s reubicada en plano YZ: (%.2f, %.2f) con rotacion 0 0 1 0\n", nombres_latas[i], nueva_pos[1], nueva_pos[2]);
      }
    }

    if (pos_robot[1] > LIMITE_MAX + .1 || pos_robot[1] < LIMITE_MIN -.1 || 
        pos_robot[2] > LIMITE_MAX + .1|| pos_robot[2] < LIMITE_MIN - .1) { 
      printf("[SUPERVISOR] ¡Alerta! Robot fuera de la arena. Teletransportando al inicio...\n");
      wb_supervisor_field_set_sf_vec3f(robot_trans, pos_inicial_robot);
      wb_supervisor_node_reset_physics(robot_node);
    }
  }

  wb_robot_cleanup();
  return 0;
}