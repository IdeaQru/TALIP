#ifndef SYNC_SENDING_H
#define SYNC_SENDING_H

#include <Arduino.h>

// --- Konstanta jumlah servo, sesuaikan dengan kebutuhan Anda ---
#define JUMLAH_DXL_PRTOKOL1 10
#define JUMLAH_DXL_PRTOKOL2 14

class SYNC
{
public:
    // Untuk servo Protocol 1.0 (AX/MX)
    void sendGoalPost(int *goalPost);
    int servoIDsXL[14] = {2,3,4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    void debugPacket(unsigned char *packet, int length)
    {
        Serial.print("Packet: ");
        for (int i = 0; i < length; i++)
        {
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    // Untuk servo Protocol 2.0 (XL-320)
    void sendGoalXL(int *positions, int *speeds);
};

// Fungsi CRC untuk Protocol 2.0 (XL-320)
unsigned short update_crc320(unsigned short crc_accum, unsigned char *dat_blk_ptr, unsigned short dat_blk_size);

#endif
