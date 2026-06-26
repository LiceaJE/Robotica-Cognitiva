import socket
import numpy as np
import tensorflow as tf
import joblib

IP_SERVIDOR = "127.0.0.1"
PUERTO = 5000

ARCHIVO_PESOS_INVERSO  = "pesos_epuck4.weights.h5"
ARCHIVO_SCALER_INVERSO = "scaler_epuck4.pkl"


def construir_red_inversa():
    """Modelo Inverso: (sensores + necesidades) → acción (0/1/2)."""
    inputs  = tf.keras.Input(shape=(8,))
    x       = tf.keras.layers.Dense(64, activation="relu")(inputs)
    x       = tf.keras.layers.Dropout(0.2)(x)
    x       = tf.keras.layers.Dense(32, activation="relu")(x)
    outputs = tf.keras.layers.Dense(3,  activation="softmax")(x)
    return tf.keras.Model(inputs=inputs, outputs=outputs)


def iniciar_servidor_cerebro():
    print(">> Cargando Modelo Inverso y Scaler... <<")

    modelo_inverso = construir_red_inversa()

    try:
        scaler = joblib.load(ARCHIVO_SCALER_INVERSO)
        modelo_inverso.load_weights(ARCHIVO_PESOS_INVERSO, skip_mismatch=True)
        print(f">> [ÉXITO] Cargados '{ARCHIVO_PESOS_INVERSO}' y '{ARCHIVO_SCALER_INVERSO}'. <<")
    except Exception as e:
        print(f"\n[ERROR CRÍTICO] No se pudieron cargar los archivos: {e}")
        print(f"Asegúrate de que '{ARCHIVO_PESOS_INVERSO}' y '{ARCHIVO_SCALER_INVERSO}' "
              f"estén en esta carpeta (los genera Internal_model2.ipynb).")
        return

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((IP_SERVIDOR, PUERTO))
    server_socket.listen(1)
    print(f">> Servidor Cerebro escuchando en {IP_SERVIDOR}:{PUERTO} <<")

    while True:
        print("\nEsperando conexión del e-puck en Webots...")
        conexion, direccion = server_socket.accept()
        print(f"¡Robot conectado desde {direccion}!")

        try:
            while True:
                datos_recibidos = conexion.recv(1024).decode("utf-8").strip()

                if not datos_recibidos:
                    break

                if "MUERTO" in datos_recibidos:
                    print("\n[VITAL] Robot reportó muerte biológica. Esperando reinicio...")
                    continue

                try:
                    valores = [float(x) for x in datos_recibidos.split(",")]
                    if len(valores) != 8:
                        print(f"[AVISO] Se esperaban 8 valores, llegaron {len(valores)}. Ignorando.")
                        continue
                except ValueError:
                    print(f"[AVISO] Datos malformados: '{datos_recibidos}'. Ignorando.")
                    continue

                vector_entrada  = np.array([valores])           # (1, 8)
                entrada_escalada = scaler.transform(vector_entrada)

                prediccion   = modelo_inverso.predict(entrada_escalada, verbose=0)
                accion_optima = int(np.argmax(prediccion[0]))

                nombres = {0: "AVANZAR", 1: "GIRAR_DER", 2: "GIRAR_IZQ"}
                if accion_optima != 0:
                    dist_izq, dist_der, luz_izq, luz_der, suelo_izq, suelo_der, bat, aburr = valores
                    print(f"[DECISIÓN] {nombres[accion_optima]:10s} | "
                          f"Luz=({luz_izq:.0f},{luz_der:.0f}) "
                          f"Dist=({dist_izq:.0f},{dist_der:.0f}) "
                          f"Bat={bat:.1f}% Aburr={aburr:.1f}%")

                conexion.sendall(str(accion_optima).encode("utf-8"))

        except ConnectionResetError:
            print("[SOCKET] Conexión interrumpida por Webots (simulación reiniciada).")
        except Exception as e:
            print(f"[ERROR] Excepción inesperada en el bucle: {e}")
        finally:
            conexion.close()
            print("Conexión con el robot cerrada limpiamente.")


if __name__ == "__main__":
    iniciar_servidor_cerebro()