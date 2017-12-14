#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <tchar.h>
#include <time.h>
#include <stdlib.h>
#pragma comment(lib, "user32.lib")
#include "SerialMI.h"

#define BUF_SIZE 256

TCHAR shmName[] = TEXT("MIGOD");

typedef struct DataExchange
{
    char sendVal[20][32];
    double getVal[20];
}INFO;

INFO *PID;
HANDLE fdshm;
//Definition of the function
int shmAccess()
{
	
	//Create Shared Memory MI_Shared Memory
	
	fdshm = CreateFileMapping(
	INVALID_HANDLE_VALUE,    // use paging file
	NULL,                    // default security
	PAGE_READWRITE,          // read/write access
	0,                       // maximum object size (high-order DWORD)
	BUF_SIZE,                // maximum object size (low-order DWORD)
	shmName);

	//Check if Shared Memory was created
	if (fdshm == NULL)
	{
		_tprintf(TEXT("Could not open file mapping object (%d).\n"),
		GetLastError());
		return(1);
	}

	//Map pBuf to Shared Memory
	PID = (INFO *)MapViewOfFile(fdshm, // handle to map object
		FILE_MAP_ALL_ACCESS,  // read/write permission
		0,
		0,
		BUF_SIZE);

	if (PID == NULL)
	{
		_tprintf(TEXT("Could not map view of file to write (%d).\n"),
			GetLastError());
		CloseHandle(fdshm);
		return(1);
	}
	printf("Shared Memory Created ");
	return (0);
}

void shmWrite(int num1, double tagValue)
{
    //if(num1!=0)printf("%d, %g\n", num1, tagValue);
	PID-> getVal[num1+10] = tagValue;
    //CopyMemory((PVOID)pBuf, PID, sizeof(INFO));
}

char *shmRead(int num2)
{
    char* retrunVal;
    retrunVal= PID->sendVal[num2];
    return retrunVal;
}

int main()
{
	char S_Port[32]="";
    int S_Baud;
    printf("Serial Port (e.g. COM5) : ");
    scanf("%s", S_Port);
    printf("Baud Rate (e.g. 115200): ");
    scanf("%d", &S_Baud);
    serialBegin(S_Port, S_Baud);
    shmAccess();
    while(1)
    {
    	char* outData;
        char someData[32]="";
        const char* inData;
        char val[20]="";
        char addr[10]="";
        int i,j;

        for (i=1; i<10; i++)
        {
            outData = shmRead(i);
            if(outData == "") break;

            printf("%s", outData);
            serialWrite(outData);
            strcpy(PID->sendVal[i], "");
        }
        
		inData = serialRead();
        strcpy(someData, inData);
        for(i=0; i<strlen(someData); i++)
        {
            if(someData[i]==',')
            {
                addr[i]='\0';
                i++;
                break;
            }
            addr[i] = someData[i];
        }

        for(j=i; j<strlen(someData); j++)
        {
            val[j-i] = someData[j];
        }
        shmWrite(atoi(addr), atof(val));
    }
printf("came out of loop, no idea why!");
serialEnd();
CloseHandle(fdshm);
}
