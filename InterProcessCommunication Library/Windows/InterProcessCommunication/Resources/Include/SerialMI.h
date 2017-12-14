#ifndef SerialMI_H_
#define SerialMI_H_


#include <stdlib.h>
#include <Windows.h>
#include <string.h>
#include <time.h>
/* run this program using the console pauser or add your own getch, system("pause") or input loop */
HANDLE fd;

int serialBegin(char port[], long int bdr)
{
	//port name,Read/Write, No Sharing, No Security, Open existing port only, Non Overlapped I/O
	fd = CreateFile(("\\\\.\\%s",port), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0,  NULL);

	if (fd == INVALID_HANDLE_VALUE)
	{
		fprintf(stderr, "\nError connecting to device..!\n");
       	CloseHandle(fd);
       	return 1;
	}
	DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
//	fprintf(stderr, "\nConnecting...\n");
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

	if (GetCommState(fd, &dcbSerialParams) == 0)
	{
    	fprintf(stderr, "Error getting device state..!\n");
    	CloseHandle(fd);
    	return 1;
    }

    switch(bdr)
	{
	case 1200:
		dcbSerialParams.BaudRate = CBR_1200;
		break;
	case 2400:
		dcbSerialParams.BaudRate = CBR_2400;
		break;
	case 4800:
		dcbSerialParams.BaudRate = CBR_4800;
		break;
	case 9600:
		dcbSerialParams.BaudRate = CBR_9600;
		break;
	case 19200:
		dcbSerialParams.BaudRate = CBR_19200;
		break;
	case 38400:
		dcbSerialParams.BaudRate = CBR_38400;
		break;
	case 57600:
		dcbSerialParams.BaudRate = CBR_57600;
		break;
	case 115200:
		dcbSerialParams.BaudRate = CBR_115200;
		break;
	default:
		fprintf(stderr, "\nInvalid BaudRate\n");
		CloseHandle(fd);
		return 1;
		break;
	}
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

	if(SetCommState(fd, &dcbSerialParams) == 0)
    {
        fprintf(stderr, "Error setting device parameters..!\n");
        CloseHandle(fd);
        return 1;
    }
	printf("\n#Successful#\n");
	return 1;
}

void serialWrite(char* write_buff)
{
	DWORD bytes_written;
	WriteFile(fd, write_buff, strlen(write_buff), &bytes_written, NULL);
	if(bytes_written < 0) fprintf(stderr, "Error in writing data\n");
}

const char* serialRead()
{
	char temp_char='\0';
    DWORD bytes_read;
	int i=0;
	static char read_buff[32]="";

    while(bytes_read>0)
    {
		ReadFile (fd, &temp_char, sizeof(temp_char), &bytes_read, NULL);

		if(temp_char == '\n') break;//|| temp_char == '\r'
		read_buff[i] = temp_char;
		i++;
    }

    read_buff[i] = '\0';
	//if(read_buff!="") printf("Rxd: %s\n", read_buff);
	return read_buff;
}

void serialEnd()
{
	fprintf(stderr, "Closing serial port...");
    if (CloseHandle(fd) == 0)
    	{
    	    fprintf(stderr, "Error\n");
    	}
    fprintf(stderr, "OK\n");
}

void delay(unsigned int duration)
{
	unsigned int retTime = time(0) + duration;   // Get finishing time.
    while (time(0) < retTime);
}
#endif
