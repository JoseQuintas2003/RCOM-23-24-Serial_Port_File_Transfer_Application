// Link layer protocol implementation

#include "link_layer.h"

extern int alarmEnabled;
// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E
#define ADR 0x03
#define SET 0x03
#define UA 0x07
#define BCC (ADR^SET)

#define BUF_SIZE 256

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO

    unsigned char buf[BUF_SIZE + 1] = {0};

    buf[0] = FLAG;
    buf[1] = ADR;
    buf[2] = SET;
    buf[3] = BCC;
    buf[5] = FLAG;

    int fd = open(connectionParameters.serialPort, ); //Acabar depois

    if (connectionParameters.role == 0) {
        int bytes = write(fd, buf, BUF_SIZE);
        print("SET sent\n");

        alarm(connectionParameters.timeout);

        if (alarmEnabled == FALSE) { //Mudar 
            buf[0] = FLAG;
            buf[1] = ADR;
            buf[2] = UA;
            buf[3] = BCC;
            buf[5] = FLAG;

            bytes = read(fd, buf, BUF_SIZE);

            if(buf[3] != buf[1]^buf[2]){
                return 0;
            }
        }
        else {

        }
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
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
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
