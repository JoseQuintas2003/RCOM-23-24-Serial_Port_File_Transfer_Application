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
int retransmissions;
int timeout = 0;
LinkLayerRole role;


// Alarm function handler
void alarmHandler(int signal)
{
    alarmCount += 1;
    alarmEnabled = TRUE;

    printf("Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = connectionParameters.serialPort;

    alarmEnabled = FALSE;

    fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPortName);
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
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    LinkLayerState state;
    unsigned char byte[5] = {0};
    retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    role = connectionParameters.role;

    //Transmitter role
    if (!role){
        state = START;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        // Create SET packet
        byte[0] = FLAG;
        byte[1] = TT_ADR;
        byte[2] = SET;
        byte[3] = byte[1] ^ byte[2];
        byte[4] = FLAG;

        while (alarmCount <= retransmissions && state != STOP_R){
            if (alarmEnabled == FALSE)
            {
                alarm(timeout);
                alarmEnabled = FALSE;

                // Send SET
                int bytes = write(fd, byte, 5);
                printf("SET sent\n");
                printf("Bytes written: %d\n", bytes);
            }
            int bytes = read(fd, byte, 1);
            if (bytes > 0)
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
                        else {
                            state = START;
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        if (state != STOP_R) {
            printf("Connection failed\n");
            return -1;
        }
    }

    //Receiver role
    if (role){
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

    printf("Connection established successfully\n");
    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char tramaTx = 0;
    int frameSize = 6+bufSize;
    unsigned char *frame = (unsigned char *) malloc(2*frameSize);
    if (frame == NULL) {
        // Memory allocation failed
        perror("Memory allocation error");
        exit(-1);
    }
    frame[0] = FLAG;
    frame[1] = TT_ADR;

    if (tramaTx) frame[2] = INF1; 
    else frame[2] = INF0;
    
    frame[3] = frame[1] ^ frame[2];
    
    memcpy(frame+4, buf, bufSize);
    
    unsigned char BCC2 = buf[0];
    for (unsigned int i = 1 ; i < bufSize ; i++) BCC2 ^= buf[i];

    int j = 4;
    for (unsigned int i = 0 ; i < bufSize ; i++) {
        if(buf[i] == FLAG || buf[i] == ESC) {
            frame[j++] = ESC;
            if(buf[i] == FLAG) frame[j++] = F_ESC;
            else frame[j++] = E_ESC;        
        }
        else frame[j++] = buf[i];
    }
    frame[j++] = BCC2;
    frame[j++] = FLAG;

    int numTries = 0;
    int rejected = 0, ready = 0;
    int bytes_written;
    unsigned char byte = 0;
    unsigned char C = 0;
    LinkLayerState state = START;

    while (numTries < retransmissions) { 
        
        if(rejected == 1) {
            printf("The frame has been rejected\n");
        }
        alarmEnabled = FALSE;
        alarm(timeout);
        rejected = 0;
        ready = 0;

        while (alarmEnabled == FALSE && !rejected && !ready) {

            bytes_written = write(fd, frame, j);
            printf("Bytes written: %d\n", bytes_written);
            byte=0;
            C = 0;
            state = START;
             
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
                if (C == REJ0 || C == REJ1) { //Aqui nao devia ser a variavel C ao inves de byte?
                    printf("Received REJ\n");
                    rejected = 1;
                    ready = 0;
                } else if (C == RR0 || C == RR1) {
                    printf("Received RR\n");
                    ready = 1;
                    tramaTx = (tramaTx + 1) % 2;
                }
            }
            if (rejected) break;
        }
        if (ready) break;
        numTries++;
    }
       
    free(frame);
    if(ready) return frameSize;
    else{
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char * packet)
{
    LinkLayerState state = START;

    unsigned char writeBuffer[6] = {0}; // 5 bytes for information feedback plus 1 byte for the '\0' character
    unsigned char byte[5]={0};

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
                    if (byte[0] == FLAG){
                        state = FLAG_RCV; 
                    } 
                    break;
                case FLAG_RCV:
                    if (byte[0] == TT_ADR){
                        state = A_RCV;
                        rec_a = byte[0];
                    }
                    else if (byte[0] != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte[0] == INF0 || byte[0] == INF1){
                        state = C_RCV;
                        rec_c = byte[0]; 
                    }
                    else if (byte[0] == FLAG) state = FLAG_RCV;
                    //else if (byte[0] == DISC) state = DISCONNECTED;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte[0] == (rec_a ^ rec_c)){
                        state = READING_DATA; 
                    } 
                    else if (byte[0] == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case READING_DATA:
                    if (byte[0] == ESC) {
                        state = DATA_FOUND_ESC;
                    }
                    else if (byte[0] == FLAG) { //If the byte is a FLAG, it means that the data has ended
                        rec_bcc2 = packet[data_position-1]; //Stores the received BCC2
                        
                        packet[data_position-1] = '\0'; //Ends the received data string

                        
                        data_check = packet[0]; //Stores the first byte of the data

                        for (int j = 1; j < data_position-1; j++) data_check ^= packet[j]; //XORs all the bytes of the data to check if the BCC2 is correct

                        if (data_check == rec_bcc2) { //If the BCC2 is correct, send RR[0/1] to the transmitter
                            writeBuffer[0] = FLAG;
                            writeBuffer[1] = REC_ADR;

                            if (rec_c == INF0) writeBuffer[2] = RR1;
                            else writeBuffer[2] = RR0;

                            writeBuffer[3] = writeBuffer[1] ^ writeBuffer[2];
                            writeBuffer[4] = FLAG;
                            writeBuffer[5] = '\0';
                            write(fd, writeBuffer, 5);

                            printf("Data received successfully.\nReceived data with %d bytes \n", data_position);

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
                        packet[data_position++] = byte[0];
                    }
                    break;
                case DATA_FOUND_ESC:
                    if (byte[0] == F_ESC) {
                        packet[data_position++] = FLAG;
                    }
                    else if (byte[0] == E_ESC) {
                        packet[data_position++] = ESC;
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
    int nretransmissions = retransmissions;

    //Transmitter role
    if (!role){

        state = START;

        // Create DISC packet
        byte[0] = FLAG;
        byte[1] = TT_ADR;
        byte[2] = DISC;
        byte[3] = byte[1] ^ byte[2];
        byte[4] = FLAG;
        
        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);
        
        while (nretransmissions!= 0 && state != STOP_R) {
                    
            // Send Transmitter DISC
            int bytes = write(fd, byte, 5);
                    printf("Transmitter DISC sent\n");
                    printf("Bytes written: %d\n", bytes);

            alarm(timeout);
            alarmEnabled = FALSE;

            while (alarmEnabled == FALSE && state != STOP_R) {
                    if (read(fd, byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte[0] == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte[0] == REC_ADR) state = A_RCV;
                            else if (byte[0] != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte[0] == DISC) state = C_RCV;
                            else if (byte[0] == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte[0] == (REC_ADR ^ DISC)) state = BCC1_OK;
                            else if (byte[0] == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte[0] == FLAG) state = STOP_R;
                            else state = START;
                            printf("Receiver DISC received successfully\n");
                            break;
                        default: 
                            break;
                    }
                }
            } 
            nretransmissions--;
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
    if (role){

        state = START;
        nretransmissions = retransmissions;

        // Create DISC packet
        byte[0] = FLAG;
        byte[1] = REC_ADR;
        byte[2] = DISC;
        byte[3] = byte[1] ^ byte[2];
        byte[4] = FLAG;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);
        
        while (nretransmissions!= 0 && state != STOP_R) {
             
            alarm(timeout);
            alarmEnabled = FALSE;
                    
            while (alarmEnabled == FALSE && state != STOP_R) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte[0] == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte[0] == TT_ADR) state = A_RCV;
                            else if (byte[0] != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte[0] == DISC) state = C_RCV;
                            else if (byte[0] == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte[0] == (TT_ADR ^ DISC)) state = BCC1_OK;
                            else if (byte[0] == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte[0] == FLAG) state = STOP_R;
                            else state = START;
                            printf("Transmitter DISC received successfully\n");
                            break;
                        default: 
                            break;
                    }
                }
            } 
            nretransmissions--;
        }
        
        if (state != STOP_R) return -1;

        // Send Receiver DISC
        int bytes = write(fd, byte, 5);
                    printf("Receiver DISC sent\n");
                    printf("Bytes written: %d\n", bytes);

        // Reset state for UA reception
        state = START;
        nretransmissions = retransmissions;

        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);


        while (nretransmissions!= 0 && state != STOP_R) {

            alarm(timeout);
            alarmEnabled = FALSE;
            
            while (alarmEnabled == FALSE && state != STOP_R) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte[0] == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte[0] == TT_ADR) state = A_RCV;
                            else if (byte[0] != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte[0] == UA) state = C_RCV;
                            else if (byte[0] == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte[0] == (TT_ADR ^ UA)) state = BCC1_OK;
                            else if (byte[0] == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte[0] == FLAG) state = STOP_R;
                            else state = START;
                            printf("UA received successfully\n");
                            break;
                        default: 
                            break;
                    }
                }
            } 
            nretransmissions--;
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