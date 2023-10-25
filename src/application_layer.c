// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>


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
    unsigned char *packet;
    
    int fd =  llopen(linkLayer);
    if (fd < 0)
    {
        perror("llopen error\n");
        exit(-1);
    }

    switch (linkLayer.role) {
    case LlTx: {
        printf("Application layer TX\n"); //Apagar depois
        
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
    
        unsigned int ControlPacketSize;
        unsigned char *ControlPacketStart = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        // 2 – start
        ControlPacketStart[0] = 2;
        // 0 – tamanho do ficheiro
        ControlPacketStart[1] = 0;
        ControlPacketStart[2] = (fileSize + 7) / 8;

        for (unsigned char i = 0; i < ControlPacketStart[2]; i++)
        {
            ControlPacketStart[2 + ControlPacketStart[2] - i] = fileSize & 0xFF;
            fileSize >>= 8;
        }

        // 1 – nome do ficheiro
        ControlPacketStart[3 + ControlPacketStart[2]] = 1;
        ControlPacketStart[4 + ControlPacketStart[2]] = strlen(filename);

        memcpy(ControlPacketStart + 5 + ControlPacketStart[2], filename, strlen(filename));

        if (llwrite(ControlPacketStart, ControlPacketSize) == -1)
        {
            perror("Exit: error in start packet\n");
            exit(-1);
        }

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

            if (llwrite(packet, packetSize) == -1)
            {
                perror("error in data packets\n");
                exit(-1);
            }

            bytesLeft -= (long int)MAX_PAYLOAD_SIZE;
            sequence = (sequence + 1) % 255;
        }

        unsigned char *controlPacketEnd = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        // 3 – end
        controlPacketEnd[0] = 3;
        // 0 – tamanho do ficheiro
        controlPacketEnd[1] = 0;
        controlPacketEnd[2] = (fileSize + 7) / 8;

        for (unsigned char i = 0; i < controlPacketEnd[2]; i++)
        {
            controlPacketEnd[2 + controlPacketEnd[2] - i] = fileSize & 0xFF;
            fileSize >>= 8;
        }

        // 1 – nome do ficheiro
        controlPacketEnd[3 + controlPacketEnd[2]] = 1;
        controlPacketEnd[4 + controlPacketEnd[2]] = strlen(filename);

        memcpy(controlPacketEnd + 5 + controlPacketEnd[2], filename, strlen(filename));

        if (llwrite(controlPacketEnd, ControlPacketSize) == -1)
        {
            printf("Exit: error in end packet\n");
            exit(-1);
        }

        llclose(fd);
        break;
    }

    case LlRx: {
        printf("Application layer RX\n"); //Apagar depois
        
        while (1)
        {
            while (llread(packet) < 0){
                
                if (packet[0] == 2)
                {
                    long int rxFileSize = 0;
                    unsigned char *name = (unsigned char *)malloc(packet[4 + packet[3]]);

                    for (unsigned char i = 0; i < packet[3]; i++)
                    {
                        rxFileSize |= (packet[3 + 2 + i] << (8 * i));
                    }

                    memcpy(name, packet + 5 + packet[3], packet[4 + packet[3]]);

                    FILE *newFile = fopen((char *)name, "wb+");
                    while (1)
                    {
                        while (llread(packet) < 0)
                            ;
                        if (packet[0] != 3)
                        {
                            unsigned char *buffer = (unsigned char *)malloc(packet[3]);
                            memcpy(buffer, packet + 4, packet[3]);
                            fwrite(buffer, sizeof(unsigned char), packet[3], newFile);
                            free(buffer);
                        }
                        else
                        {
                            break;
                        }
                    }

                    fclose(newFile);
                    break;
                }
            }
        }

        break;
    }

    default:
        exit(-1);
        break;
    }
}