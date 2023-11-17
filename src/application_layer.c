// Application layer protocol implementation

#include "../include/application_layer.h"
#include "../include/link_layer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#define RX_START 0x02
#define RX_DATA 0x01
#define RX_END 0x03

double pow(double base, double exponent) {
    double result = 1;
    while (exponent > 0) {
        result *= base;
        exponent--;
    }
    return result;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    
    LinkLayer linkLayer;
    strcpy(linkLayer.serialPort,serialPort);
    if(!strcmp(role, "tx")) linkLayer.role = LlTx;
    else linkLayer.role = LlRx;
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;
    unsigned char* packet;
    
    int fd = llopen(linkLayer);
    if (fd < 0)
    {
        perror("llopen error\n");
        exit(-1);
    }

    switch (linkLayer.role) {
    case LlTx: {

        printf("Opening file: %s\n", filename);

        FILE *file = fopen(filename, "rb");
        if (file == NULL) {
            perror("File not found\n");
            exit(-1);
        }
        
        struct stat st;
        long int fileSize;
        if (stat(filename, &st) == 0) {
            fileSize = st.st_size;
        } 
        else {
            perror("Error getting file size");
            exit(-1);
        }

        packet = malloc(fileSize + 4);
    
        unsigned int ControlPacketSize;
        unsigned char *ControlPacketStart = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        // 2 – start
        ControlPacketStart[0] = 2;
        // 0 – tamanho do ficheiro
        ControlPacketStart[1] = 0;
        ControlPacketStart[2] = 4;

        // Encoding file size
        unsigned char len4 = fileSize / (256 * 256 * 256);
        unsigned char len3 = (fileSize % (256 * 256 * 256)) / (256 * 256);
        unsigned char len2 = (fileSize % (256 * 256)) / 256;
        unsigned char len1 = fileSize % 256;

        ControlPacketStart[3] = len1;
        ControlPacketStart[4] = len2;
        ControlPacketStart[5] = len3;
        ControlPacketStart[6] = len4;

        // 1 – nome do ficheiro
        ControlPacketStart[3 + ControlPacketStart[2]] = 1;
        ControlPacketStart[4 + ControlPacketStart[2]] = strlen(filename);

        for (int i = 0; i < strlen(filename); i++)
        {
            ControlPacketStart[5 + ControlPacketStart[2] + i] = filename[i];
        }

        ControlPacketSize = 5 + ControlPacketStart[2] + strlen(filename);

        //memcpy(ControlPacketStart + 5 + ControlPacketStart[2], filename, strlen(filename));

        if (llwrite(ControlPacketStart, ControlPacketSize) == -1)
        {
            perror("Error in start packet\n");
            exit(-1);
        }
        printf("Start Control Packet sent\n");

        unsigned char sequence = 0;
        long int bytesLeft = fileSize;

        while (bytesLeft >= 0)
        {
            int dataSize = bytesLeft > (long int)MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : bytesLeft;
            unsigned char *data = (unsigned char *)malloc(dataSize);
            fread(data, sizeof(unsigned char), dataSize, file);

            int packetSize = 1 + 1 + 2 + dataSize;
            packet[0] = 1;
            packet[1] = sequence;
            packet[2] = dataSize >> 8 & 0xFF;
            packet[3] = dataSize & 0xFF;
            memcpy(packet + 4, data, dataSize);
            
            int bytes_written = llwrite(packet, packetSize);
            if (bytes_written == -1)
            {
                perror("Error in data packets. Retransmission timeout\n");
            }

            bytesLeft -= (long int)MAX_PAYLOAD_SIZE;
            sequence = (sequence + 1) % 255;
        }
        printf("Data packets sent\n");

        unsigned char *controlPacketEnd = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        // 3 – end
        controlPacketEnd[0] = 3;
        // 0 – tamanho do ficheiro
        controlPacketEnd[1] = 0;
        controlPacketEnd[2] = 4;

        controlPacketEnd[3] = len1;
        controlPacketEnd[4] = len2;
        controlPacketEnd[5] = len3;
        controlPacketEnd[6] = len4;

        // 1 – nome do ficheiro
        controlPacketEnd[3 + controlPacketEnd[2]] = 1;
        controlPacketEnd[4 + controlPacketEnd[2]] = strlen(filename);

        //memcpy(controlPacketEnd + 5 + controlPacketEnd[2], filename, strlen(filename));

        for (int i = 0; i < strlen(filename); i++)
        {
            controlPacketEnd[5 + controlPacketEnd[2] + i] = filename[i];
        }

        if (llwrite(controlPacketEnd, ControlPacketSize) == -1)
        {
            perror("Error in end packet\n");
            exit(-1);
        }
        printf("End Control Packet sent\n");

        free(packet);
        break;
    }
    
    case LlRx: {
        
        FILE *file;
        packet = malloc(256);
        long int fileSize = 0;
        int state = RX_START;

        while (state != RX_END){
            if (llread(packet) < 0){
                perror("An error occured while reading packet\n");
                continue;
            }
            else printf("Packet received\n");
            
            switch (state){
                case RX_START:
                    if (packet[0] == 2){
                        state = RX_DATA;
                        unsigned char filename[packet[4 + packet[2]] + 1];

                        if (packet[1] == 0) {
                            for (int i = 0; i < packet[2]; i++) {
                                fileSize += packet[3 + i] * pow(256,i);
                            }
                            printf("File size: %ld\n", fileSize);
                        }

                        if (packet[7] == 1) {
                            for (int i = 0; i < packet[8]; i++) {
                                filename[i] = packet[9 + i];
                            }
                            filename[packet[8]] = '\0';
                        }
                        printf("File name: %s\n", filename);

                        file = fopen((char *)filename, "wb");

                        if (file == NULL) {
                            perror("Error opening file\n");
                            exit(-1);
                        }

                        packet = malloc(fileSize + 4);
                    }
                    break;
                case RX_DATA:
                    if (packet[0] == 1) {
                        fwrite(packet + 4, packet[2] * 256 + packet[3], 1, file);
                    }
                    else if (packet[0] == 3) {
                        state = RX_END;
                        int fileSize2 = 0;
                        for (int i = 0; i < packet[2]; i++) {
                            fileSize2 += packet[3 + i] * pow(256,i);
                        }

                        printf("Final file size: %d\n", fileSize2);

                        if (fileSize != fileSize2) {
                            perror("Error: file size doesn't match\n");
                            exit(-1);
                        }
                        printf("File received successfully\n");
                        printf("File size: %ld\n", fileSize);
                        printf("File name: %s\n", filename);
                    }
                    break;
                }
            }
            
            fclose(file);
            free(packet);
        }
    }
    
    if (llclose(fd) == -1)
    {
        perror("Error closing connection\n");
        exit(-1);
    }

    printf("Connection closed\n");
}

