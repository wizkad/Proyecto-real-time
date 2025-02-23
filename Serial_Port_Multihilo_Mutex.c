#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <bits/termios-baud.h>
#include <asm-generic/termbits-common.h>
#include <pthread.h>

#define SERIAL_PORT "/dev/ttyACM0" 
#define BAUD_RATE B115200          

int serial_port; 
int señal = 0; // Flag para gestionar la escritura
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; // Mutex para proteger 'señal'

void log_data(const char *log_file, const char *data) {
    FILE *file = fopen(log_file, "a"); // Abre el archivo en modo "append"

    // Obtener la fecha y hora actual
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    // Escribir la fecha, hora y datos en el archivo
    fprintf(file, "%02d-%02d-%04d %02d:%02d:%02d: %s\n",
            t->tm_mday, t->tm_mon + 1, t->tm_year + 1900,
            t->tm_hour, t->tm_min, t->tm_sec, data);

    fclose(file);
}

void *read_from_serial(void *arg) {
    char buffer[30];
    char valor[5]; 
    memset(buffer, 0, sizeof(buffer));
    memset(valor, 0, sizeof(valor));
    int num_bytes = 0;

    while (1) {
        pthread_mutex_lock(&mutex); // Bloquea el mutex
        while (señal == 0) {
            pthread_mutex_unlock(&mutex); // Libera el mutex si no puede continuar
        }
        // Leer datos del puerto serie
        num_bytes = read(serial_port, buffer, sizeof(buffer) - 1);
        if (num_bytes < 0) {
            perror("Error al leer del puerto serie");
            pthread_mutex_unlock(&mutex); // Libera el mutex antes de continuar
            continue;
        }

        buffer[num_bytes] = '\0'; 

        // Imprimir todo lo recibido
        printf("Desde el puerto serie: %s\n", buffer);
        fflush(stdout);

        // Buscar números entre delimitadores #
        for (int i = 0; i < num_bytes; i++) {
            if (buffer[i] == '#') {
                int j;
                for (j = 1; j < 5 && (i + j) < num_bytes; j++) {
                    if (buffer[i + j] == '#') {
                        break;
                    }
                    valor[j - 1] = buffer[i + j];
                }

                if (j == 5 && buffer[i + j] == '#') { 
                    valor[4] = '\0'; // Terminar la cadena
                    printf("Número detectado entre #: %s\n", valor);

                    // Guardar en el log
                    log_data("log.txt", valor);
                    fflush(stdout);
                }
            }
        }
        señal = 0;
        pthread_mutex_unlock(&mutex); // Libera el mutex
    }
}

void *write_to_serial(void *arg) {
    char input[256];
    while (1) {
        pthread_mutex_lock(&mutex); // Bloquea el mutex
        while (señal == 1) {
            pthread_mutex_unlock(&mutex); // Libera el mutex si no puede continuar
        }
        printf("Escribe un comando para el puerto serie: ");
        fflush(stdout);
        if (fgets(input, sizeof(input), stdin) != NULL) {
            size_t len = strlen(input);
            if (input[len - 1] == '\n') input[len - 1] = '\r'; // Reemplazar '\n' con '\r'
            write(serial_port, input, strlen(input));          
        } else {
            perror("Error al leer de teclado");
            pthread_mutex_unlock(&mutex); // Libera el mutex antes de salir
            break;
        }
        señal = 1;
        pthread_mutex_unlock(&mutex); // Libera el mutex
    }
    return NULL;
}

int configure_serial_port() {
    struct termios tty;

    // Abrir el puerto serie
    serial_port = open(SERIAL_PORT, O_RDWR);
    if (serial_port < 0) {
        perror("No se pudo abrir el puerto serie");
        return -1;
    }

    // Configurar el puerto serie
    if (tcgetattr(serial_port, &tty) != 0) {
        perror("No se pudieron obtener los atributos del puerto serie");
        close(serial_port);
        return -1;
    }

    cfsetispeed(&tty, BAUD_RATE);
    cfsetospeed(&tty, BAUD_RATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits por palabra
    tty.c_iflag &= ~IGNBRK;                     // Deshabilitar BREAK
    tty.c_lflag = 0;                            // Modo sin canon
    tty.c_oflag = 0;                            // Sin procesamiento de salida
    tty.c_cc[VMIN] = 1;                         // Leer al menos 1 carácter
    tty.c_cc[VTIME] = 1;                        // Esperar 0.1s

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // Deshabilitar control de flujo por software
    tty.c_cflag |= (CLOCAL | CREAD);            // Habilitar lectura
    tty.c_cflag &= ~(PARENB | PARODD);          // Sin paridad
    tty.c_cflag &= ~CSTOPB;                     // 1 bit de parada
    tty.c_cflag &= ~CRTSCTS;                    // Sin control de flujo por hardware

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("No se pudieron establecer los atributos del puerto serie");
        close(serial_port);
        return -1;
    }

    return 0;
}

int main() {
    const char *log_file = "log.txt"; // Nombre del archivo de log
    FILE *file = fopen(log_file, "a"); // Abre el archivo en modo "append"
    if (file == NULL) {
        // Si no se encuentra el archivo, intenta crearlo
        file = fopen(log_file, "w"); // Modo "write" crea el archivo
        if (file == NULL) { // Si tampoco puede crearlo, imprime el error
            perror("Error al abrir o crear el archivo de log");
            return 1;
        }
    }
    if (configure_serial_port() != 0) {
        return 1;
    }

    pthread_t read_thread, write_thread;

    // Crear el hilo para leer del puerto serie
    if (pthread_create(&read_thread, NULL, read_from_serial, NULL) != 0) {
        perror("Error al crear el hilo de lectura");
        close(serial_port);
        return 1;
    }

    // Crear el hilo para escribir al puerto serie
    if (pthread_create(&write_thread, NULL, write_to_serial, NULL) != 0) {
        perror("Error al crear el hilo de escritura");
        close(serial_port);
        return 1;
    }

    // Esperar a que los hilos terminen
    pthread_join(read_thread, NULL);
    pthread_join(write_thread, NULL);

    close(serial_port);
    return 0;
}
