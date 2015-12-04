#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "frizz_sensor.h"
#include <errno.h>


int main(int argc, char **argv)
{
	int fd;
	char getchars;

	fd = open("/dev/frizz", O_RDWR);
	if(fd < 0) {
		perror("open error\n");
		return -1;
	}

	int sensor_id = 0;
	int test_loop = 0;
	sensor_info test_sensor_info = {0};

	while(1) {

		printf("=============================================================\n"
				"NOTE:\n"
				"1, download frizz firmware.\n"
				"2, check frizz booted and send command, receive sensor data.\n"
				"3, quit this test program.\n"
				"=============================================================\n"
				"Please select:");
		getchars = getchar();
		switch(getchars - 48){  /* ascii change to int number */
		case 1 :
			ioctl(fd, FRIZZ_IOCTL_HARDWARE_DOWNLOAD_FIRMWARE, "/data/frizz/from.bin");
			printf("done.\n");
			break;

		case 2 :
			printf("please input the sensor_id and test_loop:");
			scanf("%x %d", &sensor_id, &test_loop);
			test_sensor_info.sensor_id = sensor_id;
			test_sensor_info.test_loop = test_loop;
			ioctl(fd, FRIZZ_IOCTL_FW_TEST, &test_sensor_info);
			printf("done.\n");
			break;

		case 3:
			close(fd);
			printf("done.\n");
			return 0;
			break;
		default:
			perror("Cannot find the command\n");
			break;
		}
	}

	return 0;
}
