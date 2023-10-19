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

#define INF0 0x00
#define INF1 0x40

#define DISC 0x0B
#define RR0 0x05
#define RR1 0X85
#define REJ0 0X01
#define REJ1 0X81
#define ESC 0x7D

#define F_ESC 0x5E // FLAG XOR 0x20
#define E_ESC 0x5D // ESC XOR 0x20

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
    unsigned char byte[5] = {0};
    unsigned char byte[1];

    //Transmitter role
    if (!connectionParameters.role){
        state = START;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        // Create SET packet
        byte[0] = FLAG;
        byte[1] = TT_ADR;
        byte[2] = SET;
        byte[3] = byte[1] ^ byte[2];
        byte[4] = FLAG;

        while (alarmCount <= connectionParameters.nRetransmissions && state != STOP_R){
            if (alarmEnabled == FALSE)
            {

                // Send SET
                int bytes = write(fd, byte, 5);
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
        byte[0] = FLAG;
        byte[1] = REC_ADR;
        byte[2] = UA;
        byte[3] = byte[1] ^ byte[2];
        byte[4] = FLAG;

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
        int bytes = write(fd, byte, 5);
        printf("UA sent\n");
        printf("Bytes written: %d\n", bytes);
    }

    
    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *byte, int bufSize)
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

    if (tramaTx) frame[2] = INF1; 
    else frame[2] = INF0;
    
    frame[3] = frame[1] ^ frame[2];
    
    memcpy(frame+4,byte, bufSize);
    unsigned char BCC2 = byte[0];
    for (unsigned int i = 1 ; i < bufSize ; i++) BCC2 ^= byte[i];

    int j = 4;
    for (unsigned int i = 0 ; i < bufSize ; i++) {
        if(byte[i] == FLAG || byte[i] == ESC) {
            frame = realloc(frame,++frameSize);
            frame[j++] = ESC;
        }
        frame[j++] = byte[i];
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
                if (byte == REJ0 || byte == REJ1) { //Aqui nao devia ser a variavel C ao inves de byte?
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
int llread(int fd, unsigned char * buffer)
{
    LinkLayerState state = START;

    unsigned char writeBuffer[6] = {0}; // 5 bytes for information feedback plus 1 byte for the '\0' character
    unsigned char byte = 0;

    //Auxiliary variables
    int rec_a;
    int rec_c;
    int rec_bcc2;

    int data_check; //Variable to check if the data is correct (aka if the BCC2 is correct)

    int data_position = 0;

    while (state != STOP_R) {
        if (read(fd, &byte, 1) > 0) {
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
                    if (byte == INF0 || byte == INF1){
                        state = C_RCV;
                        rec_c = byte; 
                    }
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == (rec_a ^ rec_c)) state = READING_DATA;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case READING_DATA:
                    if (byte == ESC) state = DATA_FOUND_ESC;
                    else if (byte = FLAG) { //If the byte is a FLAG, it means that the data has ended
                        rec_bcc2 = buffer[data_position-1]; //Stores the recieved BCC2
                        
                        buffer[data_position-1] = '\0'; //Ends the recieved data string

                        
                        data_check = buffer[0]; //Stores the first byte of the data

                        for (int j = 1; j < data_position-1; j++) data_check ^= buffer[j]; //XORs all the bytes of the data to check if the BCC2 is correct


                        if (data_check == rec_bcc2) { //If the BCC2 is correct, send RR[0/1] to the transmitter
                            writeBuffer[0] = FLAG;
                            writeBuffer[1] = REC_ADR;

                            if (rec_c == INF0) writeBuffer[2] = RR1;
                            else writeBuffer[2] = RR0;

                            writeBuffer[3] = writeBuffer[1] ^ writeBuffer[2];
                            writeBuffer[4] = FLAG;
                            writeBuffer[5] = '\0';
                            write(fd, writeBuffer, 5);

                            printf("Data received successfully.\n Recieved data with %d bytes \n", data_position);

                            state = STOP_R;

                            return data_position; //Returns the number of bytes read
                        }
                        else { //If the BCC2 is incorrect, send REJ[0/1] to the transmitter
                            writeBuffer[0] = FLAG;
                            writeBuffer[1] = REC_ADR;

                            if (rec_c == INF0) writeBuffer[2] = REJ0;
                            else writeBuffer[2] = REJ1;

                            writeBuffer[3] = writeBuffer[1] ^ writeBuffer[2];
                            writeBuffer[4] = FLAG;
                            writeBuffer[5] = '\0';
                            write(fd, writeBuffer, 5);

                            printf("An error occured while recieving the data\n");

                            return -1; //Returns -1 to indicate an error
                        }
                    }
                    else {
                        buffer[data_position] = byte;
                        data_position++;
                    }
                    break;
                case DATA_FOUND_ESC:
                    if (byte == F_ESC) {
                        buffer[data_position] = FLAG;
                        data_position++;
                    }
                    else if (byte == E_ESC) {
                        buffer[data_position] = ESC;
                        data_position++;
                    }
                    state = READING_DATA; //Not sure se n falta aqui uma condiçao
                    break;
                default: 
                    break;
            }
        } 
    }

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int fd)
{
    LinkLayerState state;
    unsigned char byte[5] = {0};
    unsigned char byte[1];
    int retransmissions;

    //Transmitter role
    if (!connectionParameters.role){

        state = START;

        // Create DISC packet
        byte[0] = FLAG;
        byte[1] = TT_ADR;
        byte[2] = DISC;
        byte[3] = byte[1] ^ byte[2];
        byte[4] = FLAG;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        retransmissions=connectionParameters.nRetransmissions;

        while ( retransmissions!= 0 && state != STOP_R) {
                    
            // Send Transmitter DISC
            int bytes = write(fd, byte, 5);
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
        byte[0] = FLAG;
        byte[1] = TT_ADR;
        byte[2] = UA;
        byte[3] = byte[1] ^ byte[2];
        byte[4] = FLAG;

        // Send UA
        int bytes = write(fd, byte, 5);
        printf("UA sent\n");
        printf("Bytes written: %d\n", bytes);
    }

    //Receiver role
    if (connectionParameters.role){

        state = START;

        // Create DISC packet
        byte[0] = FLAG;
        byte[1] = REC_ADR;
        byte[2] = DISC;
        byte[3] = byte[1] ^ byte[2];
        byte[4] = FLAG;

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
        int bytes = write(fd, byte, 5);
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