#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// 计算校验和
int calculate_checksum(char *data) {
    int checksum = 0;
    int i;
    for (i = 1; data[i] != '*'; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

int main() {
    char nmea_data[100];
    int time_seconds = 0;
    int second=0;

    while(time_seconds < 100) {
        sprintf(nmea_data, "$GPRMC,%02d%02d%02d.00,A,2237.496474,N,11356.089515,E,0.0,225.5,310518,2.3,W,A*", (time_seconds-1) / 3600,((time_seconds-1) % 3600) / 60,(time_seconds-1) % 60);
        
        // 计算校验和
        int checksum = calculate_checksum(nmea_data);
        printf("%s%02X\n", nmea_data, checksum);

        // // 验证校验和
        // if (validate_checksum(nmea_data)) {
        //     printf("校验通过！\n");
        // } else {
        //     printf("校验未通过！\n");
        // }
 
        time_seconds++;
    }

    return 0;
}
