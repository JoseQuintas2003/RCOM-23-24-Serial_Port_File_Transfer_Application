// Link layer protocol implementation

#include "../include/link_layer.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E
#define TT_ADR 0x03
#define REC_ADR 0x01
#define SET 0x03
#define UA 0x07
#define DISC 0x0B

#define BUF_SIZE 256
#define BAUDRATE B38400

struct termios oldtio;

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = connectionParameters.serialPort;

    //Isto n deve ser preciso
    /*if (argc < 2) 
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }
    */

    int fd = open(connectionParameters.serialPort, O_WRONLY | O_NOCTTY);
    if (fd < 0) {
        perror(connectionParameters.serialPort);
        return -1; 
    }

    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }


    LinkLayerState state = START;

    //Transmitter role

    if(!connectionParameters.role){
        unsigned char byte[1];
        
        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        // Create SET packet
        unsigned char buf[BUF_SIZE + 1] = {0};

        buf[0] = FLAG;
        buf[1] = TT_ADR;
        buf[2] = SET;
        buf[3] = buf[1] ^ buf[2];
        buf[5] = FLAG;

        while (alarmCount <= connectionParameters.nRetransmissions && state != STOP_R) {
            if (alarmEnabled == FALSE) {

                int bytes = write(fd, buf, BUF_SIZE);
                printf("SET sent\n"); 
                printf("Bytes written: %d\n", bytes);

                alarm(connectionParameters.timeout); 
                alarmEnabled = TRUE;
            }

            if (read(fd, byte, 1) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == REC_ADR) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == UA) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (REC_ADR ^ UA)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) state = STOP_R;
                        else state = START;
                        break;
                    default: 
                        break;
                }
            }
        }
    }

    //Receiver role

    if(connectionParameters.role){
        unsigned char byte[1];

        // Create UA packet

        unsigned char buf[5] = {0};
        
        buf[0] = FLAG;
        buf[1] = REC_ADR;
        buf[2] = UA;
        buf[3] = buf[1] ^ buf[2];
        buf[5] = FLAG;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        // Auxiliary variables
        int rec_a, rec_c;

        while (state != STOP_R) {
            if (read(fd, byte, 1) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == TT_ADR){
                            state = A_RCV;
                            rec_a = byte;
                        }
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == SET){
                            state = C_RCV;
                            rec_c = byte;
                        }
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (TT_ADR ^ SET)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) state = STOP_R;
                        else state = START;
                        break;
                    default: 
                        break;
                }
            }  
        }

        int bytes = write(fd, buf, 5);
        printf("UA sent\n"); 
        printf("Bytes written: %d\n", bytes);
    }

    /* Isto e suposto estar na funcao llclose()

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    */

    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int fd)
{
    /*
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    int fd = open(connectionParameters.serialPort, O_WRONLY | O_NOCTTY);
    if (fd < 0) {
        perror(connectionParameters.serialPort);
        return -1; 
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    LinkLayerState state = START;
    unsigned char byte;
    int retransmitions = connectionParameters.nRetransmissions;
    (void) signal(SIGALRM, alarmHandler);
    
    while (retransmitions != 0 && state != STOP_R) {
                
        write(fd, ADR, DISC);
        alarm(connectionParameters.timeout);
        alarmEnabled = FALSE;
                
        while (alarmEnabled == FALSE && state != STOP_R) {
            if (read(fd, &byte, 1) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == ADR) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (ADR^DISC)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) state = STOP_R;
                        else state = START;
                        break;
                    default: 
                        break;
                }
            }
        } 
        retransmitions--;
    }

    if (state != STOP_R) return -1;
    write(fd, ADR, UA);
    return close(fd);

    return 1;
    */

   // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }
    close(fd);
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        printf("Usage: %s /dev/ttySxx tx|rx filename\n", argv[0]);
        exit(1);
    }

    const char *serialPort = argv[1];

    int individual; //mudar nome depois
    sscanf(argv[2], "%d", &individual);
    //const char *filename = argv[3];

    LinkLayer linkLayer;
    sprintf(linkLayer.serialPort, "%s", serialPort);

    if (individual)
        linkLayer.role = LlTx;
    else if (!individual)
        linkLayer.role = LlRx;
    else
    {
        printf("Invalid role: %s\n", argv[2]);
        exit(1);
    }

    int fd = llopen(linkLayer);

    llclose(fd);

    return 0;
}