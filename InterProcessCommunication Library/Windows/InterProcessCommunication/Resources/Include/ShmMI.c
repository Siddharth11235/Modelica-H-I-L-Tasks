#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <tchar.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#pragma comment(lib, "user32.lib")

//Defines the name of the Shared Memory
#define BUF_SIZE 256

TCHAR shmName[] = TEXT("MIGOD");

typedef struct DataExchange
{
    char sendVal[20][32];
    double getVal[20];
}INFO;

INFO *PID;
HANDLE fdshm;
void shmAccess()
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
	}
	//printf("Shm Setup Successful\n");
}

//Write onto Shared Memory
double shmWrite(int num1, double tagValue)
{
	shmAccess();
	sprintf(PID->sendVal[num1],"%d,%.2g\n", num1, tagValue);
	//printf("%s", PID->sendVal[num1]);
	//CopyMemory((PVOID)pBuf, PID, sizeof(INFO));
	CloseHandle(fdshm);
	return 0;
}

double shmRead(int num2)
{
	shmAccess();
	double retrunVal=0;
	retrunVal = PID->getVal[num2+10];
	CloseHandle(fdshm);
	return retrunVal;
}

/*void main()
{
	int i;
	double j;
	shmAccess();
	for(i=0; i<5; i++)
	{
		j=i;
		shmWrite(1, j);
		Sleep(1000);
		printf("\"%g\"\n",shmRead(1));
	}
	
	CloseHandle(fdshm);
}*/
