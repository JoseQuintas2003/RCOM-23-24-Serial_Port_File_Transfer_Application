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
#define RR0 0x05
#define RR1 0X85
#define REJ0 0X01
#define REJ1 0X81
#define ESC 0x7D

#define BUF_SIZE 256
#define BAUDRATE B38400

struct termios oldtio;

int fd;
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

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
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
    newtio.c_cc[VMIN] = 1;

    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    LinkLayerState state;
    unsigned char buf[5] = {0};
    unsigned char byte[1];

    //Transmitter role
    if (!connectionParameters.role){
        state = START;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        // Create SET packet
        buf[0] = FLAG;
        buf[1] = TT_ADR;
        buf[2] = SET;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;

        while (alarmCount <= connectionParameters.nRetransmissions && state != STOP_R){
            if (alarmEnabled == FALSE)
            {

                // Send SET
                int bytes = write(fd, buf, 5);
                printf("SET sent\n");
                printf("Bytes written: %d\n", bytes);

                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

            if (read(fd, byte, 1) > 0)
            {
                switch (state)
                {
                case START:
                    if (byte[0] == FLAG)
                        state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte[0] == REC_ADR)
                        state = A_RCV;
                    else if (byte[0] != FLAG)
                        state = START;
                    break;
                case A_RCV:
                    if (byte[0] == UA)
                        state = C_RCV;
                    else if (byte[0] == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case C_RCV:
                    if (byte[0] == (REC_ADR ^ UA))
                        state = BCC1_OK;
                    else if (byte[0] == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case BCC1_OK:
                    if (byte[0] == FLAG){
                        state = STOP_R;
                        printf("UA received successfully\n");
                    }
                    else
                        state = START;
                    break;
                default:
                    break;
                }
            }
        }
    }

    //Receiver role
    if (connectionParameters.role){
        state = START;

        // Create UA packet
        buf[0] = FLAG;
        buf[1] = REC_ADR;
        buf[2] = UA;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);


        while (state != STOP_R){
            if (read(fd, byte, 1) > 0)
            {
                switch (state)
                {
                case START:
                    if (byte[0] == FLAG){
                        state = FLAG_RCV;
                    }
                    break;
                case FLAG_RCV:
                    if (byte[0] == TT_ADR)
                    {
                        state = A_RCV;
                    }
                    else if (byte[0] != FLAG)
                        state = START;
                    break;
                case A_RCV:
                    if (byte[0] == SET)
                    {
                        state = C_RCV;
                    }
                    else if (byte[0] == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case C_RCV:
                    if (byte[0] == (TT_ADR ^ SET)){
                        state = BCC1_OK; 
                    }
                    else if (byte[0] == FLAG)
                        state = FLAG_RCV;
                    else
                        state = START;
                    break;
                case BCC1_OK:
                    if (byte[0] == FLAG){
                        state = STOP_R;
                        printf("SET received successfully\n");
                    }
                    else
                        state = START;
                    break;
                default:
                    break;
                }
            }
        }

        // Send UA
        int bytes = write(fd, buf, 5);
        printf("UA sent\n");
        printf("Bytes written: %d\n", bytes);
    }

    
    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char tramaTx = 0;
    int frameSize = 6+bufSize;
    unsigned char *frame = (unsigned char *) malloc(frameSize);
    if (frame == NULL) {
        // Memory allocation failed
        printf("Memory allocation error");
    }
    frame[0] = FLAG;
    frame[1] = REC_ADR;
    frame[2] = RR0;
    frame[3] = REC_ADR ^ RR0;
    memcpy(frame+4,buf, bufSize);
    unsigned char BCC2 = buf[0];
    for (unsigned int i = 1 ; i < bufSize ; i++) BCC2 ^= buf[i];

    int j = 4;
    for (unsigned int i = 0 ; i < bufSize ; i++) {
        if(buf[i] == FLAG || buf[i] == ESC) {
            frame = realloc(frame,++frameSize);
            frame[j++] = ESC;
        }
        frame[j++] = buf[i];
    }
    frame[j++] = BCC2;
    frame[j++] = FLAG;

    int retransmissions=connectionParameters.nRetransmissions;
    int numTries = 0;
    int rejected = 0, ready = 0;

    while (numTries < retransmissions) { 
        
        alarmEnabled = FALSE;
        alarm(connectionParameters.timeout);
        rejected = 0;
        ready = 0;

        while (alarmEnabled == FALSE && !rejected && !ready) {

            write(fd, frame, j);
            unsigned char byte, C = 0;
            LinkLayerState state = START;
            
            while (state != STOP_R && alarmEnabled == FALSE) {  
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == REC_ADR) state = A_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1 || byte == DISC){
                                state = C_RCV;
                                C = byte; 
                            }
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (REC_ADR ^ C)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG){
                                state = STOP_R;
                            }
                            else state = START;
                            break;
                        default: 
                            break;
                    }
                } 
            } 
            
            if (state == STOP_R) {
                if (byte == REJ0 || byte == REJ1) {
                    rejected = 1;
                } else if (byte == RR0 || byte == RR1) {
                    ready = 1;
                    tramaTx = (tramaTx + 1) % 2;
                }
            }

        }
        if (ready) break;
        numTries++;
    }
    
    free(frame);
    if(ready) return frameSize;
    else{
        llclose(fd);
        return -1;
    }

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
    LinkLayerState state;
    unsigned char buf[5] = {0};
    unsigned char byte[1];
    int retransmissions;

    //Transmitter role
    if (!connectionParameters.role){

        state = START;

        // Create DISC packet
        buf[0] = FLAG;
        buf[1] = TT_ADR;
        buf[2] = DISC;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        retransmissions=connectionParameters.nRetransmissions;

        while ( retransmissions!= 0 && state != STOP_R) {
                    
            // Send Transmitter DISC
            int bytes = write(fd, buf, 5);
                    printf("Transmitter DISC sent\n");
                    printf("Bytes written: %d\n", bytes);

            alarm(connectionParameters.timeout);
            alarmEnabled = FALSE;
                    
            while (alarmEnabled == FALSE && state != STOP_R) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == REC_ADR) state = A_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte == DISC) state = C_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (REC_ADR ^ DISC)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG) state = STOP_R;
                            else state = START;
                            printf("Receiver DISC received successfully\n");
                            break;
                        default: 
                            break;
                    }
                }
            } 
            retransmissions--;
        }

        if (state != STOP_R) return -1;

        // Create UA packet
        buf[0] = FLAG;
        buf[1] = TT_ADR;
        buf[2] = UA;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;

        // Send UA
        int bytes = write(fd, buf, 5);
        printf("UA sent\n");
        printf("Bytes written: %d\n", bytes);
    }

    //Receiver role
    if (connectionParameters.role){

        state = START;

        // Create DISC packet
        buf[0] = FLAG;
        buf[1] = REC_ADR;
        buf[2] = DISC;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        retransmissions=connectionParameters.nRetransmissions;

        while ( retransmissions!= 0 && state != STOP_R) {

            alarm(connectionParameters.timeout);
            alarmEnabled = FALSE;
                    
            while (alarmEnabled == FALSE && state != STOP_R) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == TT_ADR) state = A_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte == DISC) state = C_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (TT_ADR ^ DISC)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG) state = STOP_R;
                            else state = START;
                            printf("Transmitter DISC received successfully\n");
                            break;
                        default: 
                            break;
                    }
                }
            } 
            retransmissions--;
        }
         
        if (state != STOP_R) return -1;

        // Send Receiver DISC
        int bytes = write(fd, buf, 5);
                    printf("Receiver DISC sent\n");
                    printf("Bytes written: %d\n", bytes);

        // Reset state for UA reception
        state = START;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        retransmissions=connectionParameters.nRetransmissions;

        while ( retransmitions!= 0 && state != STOP_R) {

            alarm(connectionParameters.timeout);
            alarmEnabled = FALSE;
                    
            while (alarmEnabled == FALSE && state != STOP_R) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == TT_ADR) state = A_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte == UA) state = C_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (TT_ADR ^ UA)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG) state = STOP_R;
                            else state = START;
                            printf("UA received successfully\n");
                            break;
                        default: 
                            break;
                    }
                }
            } 
            retransmissions--;
        }
    
    if (state != STOP_R) return -1;

    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }

    return close(fd);

}

int main(int argc, char *argv[]){
    if (argc < 3)
    {
        printf("Usage: %s /dev/ttySxx tx|rx filename\n", argv[0]);
        exit(1);
    }

    const char *serialPort = argv[1];

    int individual; //mudar nome depois
    sscanf(argv[2], "%d", &individual);
    //const char *filename = argv[3]; Por enquanto, n precisamos disto

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

    llopen(linkLayer);

    //llwrite(...);

    llclose(fd);

    return 0;
}