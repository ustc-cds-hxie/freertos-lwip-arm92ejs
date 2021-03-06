/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* Standard includes. */
#include <stdio.h>
#include <time.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+FAT includes. */
#include "ff_headers.h"
#include "ff_stdio.h"
#include "ff_ramdisk.h"

#include "print.h"

/* 
** The number and size of sectors that will make up the RAM disk.
*/ 
#define mainRAM_DISK_SECTOR_SIZE	512UL /* Currently fixed! */
#define mainRAM_DISK_SECTORS		( ( 5UL * 1024UL * 1024UL ) / mainRAM_DISK_SECTOR_SIZE ) /* 5M bytes. */
#define mainIO_MANAGER_CACHE_SIZE	( 15UL * mainRAM_DISK_SECTOR_SIZE )

/* Where the RAM disk is mounted. */
#define mainRAM_DISK_NAME		"/ram"

/* The number of bytes read/written to the example files at a time. */
#define fsRAM_BUFFER_SIZE 		200

/* The number of bytes written to the file that uses f_putc() and f_getc(). */
#define fsPUTC_FILE_SIZE		100

/*
 * Create a set of example files in the root directory of the volume using
 * ff_fwrite().
 */
static void prvCreateDemoFilesUsing_ff_fwrite( const char *pcMountPath );

/*
 * Use ff_fread() to read back and verify the files that were created by
 * prvCreateDemoFilesUsing_ff_fwrite().
 */
static void prvVerifyDemoFileUsing_ff_fread( void );

/*
 * Create an example file in a sub-directory using f_putc().
 */
static void prvCreateDemoFileUsing_ff_fputc( const char *pcMountPath );

/*
 * Use f_getc() to read back and verify the file that was created by
 * prvCreateDemoFileUsing_f_putc().
 */
static void prvVerifyDemoFileUsing_ff_fgetc( const char *pcMountPath );


/* Names of directories that are created. */
static const char *pcDirectory1 = "SUB1", *pcDirectory2 = "SUB2", *pcFullPath = "/SUB1/SUB2";

/*
 * Creates a RAM disk, then creates files on the RAM disk.  The files can then
 * be viewed via the FTP server and the command line interface.
 */
void CreateDiskAndExampleFiles( void );

/*
 * Functions used to create and then test files on a disk.
 */
extern void vCreateAndVerifyExampleFiles( const char *pcMountPath );


/*-----------------------------------------------------------*/
/*
** Code
*/

void CreateRamDisk( void )
{
	static uint8_t ucRAMDisk[ mainRAM_DISK_SECTORS * mainRAM_DISK_SECTOR_SIZE ];
	FF_Disk_t     *pxDisk;

	/* Create the RAM disk. */
        vPrintMsg("Calling FF_RAMDiskInit\n");
	pxDisk = FF_RAMDiskInit( mainRAM_DISK_NAME, ucRAMDisk, mainRAM_DISK_SECTORS, mainIO_MANAGER_CACHE_SIZE );
	configASSERT( pxDisk );
        vPrintMsg("Back from FF_RAMDiskInit\n");

        vPrintMsg("Calling FF_RAMDiskShowPartition\n");
	/* Print out information on the disk. */
	FF_RAMDiskShowPartition( pxDisk );
        vPrintMsg("Back from FF_RAMDiskShowPartition\n");

}

void vCreateAndVerifyExampleFiles( const char *pcMountPath )
{
	/* Create and verify a few example files using both line based and character
	based reads and writes. */
	prvCreateDemoFilesUsing_ff_fwrite( pcMountPath );
	prvVerifyDemoFileUsing_ff_fread();
	prvCreateDemoFileUsing_ff_fputc( pcMountPath );
	prvVerifyDemoFileUsing_ff_fgetc( pcMountPath );
}

static void prvCreateDemoFilesUsing_ff_fwrite( const char *pcMountPath )
{
	BaseType_t xFileNumber, xWriteNumber;
	const BaseType_t xMaxFiles = 5;
	int32_t lItemsWritten;
	int32_t lResult;
	FF_FILE *pxFile;
	char *pcRAMBuffer, *pcFileName;

	/* Allocate buffers used to hold date written to/from the disk, and the
	file names. */
	pcRAMBuffer = ( char * ) pvPortMalloc( fsRAM_BUFFER_SIZE );
	pcFileName = ( char * ) pvPortMalloc( ffconfigMAX_FILENAME );
	configASSERT( pcRAMBuffer );
	configASSERT( pcFileName );

	/* Ensure in the root of the mount being used. */
	lResult = ff_chdir( pcMountPath );
	configASSERT( lResult >= 0 );

	/* Create xMaxFiles files.  Each created file will be
	( xFileNumber * fsRAM_BUFFER_SIZE ) bytes in length, and filled
	with a different repeating character. */
	for( xFileNumber = 1; xFileNumber <= xMaxFiles; xFileNumber++ )
	{
		/* Generate a file name. */
		snprintf( pcFileName, ffconfigMAX_FILENAME, "root%03d.txt", ( int ) xFileNumber );

		/* Obtain the current working directory and print out the file name and
		the	directory into which the file is being written. */
		ff_getcwd( pcRAMBuffer, fsRAM_BUFFER_SIZE );
		FF_PRINTF( "Creating file %s in %s\n", pcFileName, pcRAMBuffer );

		/* Open the file, creating the file if it does not already exist. */
		pxFile = ff_fopen( pcFileName, "w" );
		configASSERT( pxFile );

		/* Fill the RAM buffer with data that will be written to the file.  This
		is just a repeating ascii character that indicates the file number. */
		memset( pcRAMBuffer, ( int ) ( '0' + xFileNumber ), fsRAM_BUFFER_SIZE );

		/* Write the RAM buffer to the opened file a number of times.  The
		number of times the RAM buffer is written to the file depends on the
		file number, so the length of each created file will be different. */
		for( xWriteNumber = 0; xWriteNumber < xFileNumber; xWriteNumber++ )
		{
			lItemsWritten = ff_fwrite( pcRAMBuffer, fsRAM_BUFFER_SIZE, 1, pxFile );
			configASSERT( lItemsWritten == 1 );
		}

		/* Close the file so another file can be created. */
		ff_fclose( pxFile );
	}

	vPortFree( pcRAMBuffer );
	vPortFree( pcFileName );
}
/*-----------------------------------------------------------*/

static void prvVerifyDemoFileUsing_ff_fread( void )
{
	BaseType_t xFileNumber, xReadNumber;
	const BaseType_t xMaxFiles = 5;
	size_t xItemsRead, xChar;
	FF_FILE *pxFile;
	char *pcRAMBuffer, *pcFileName;

	/* Allocate buffers used to hold date written to/from the disk, and the
	file names. */
	pcRAMBuffer = ( char * ) pvPortMalloc( fsRAM_BUFFER_SIZE );
	pcFileName = ( char * ) pvPortMalloc( ffconfigMAX_FILENAME );
	configASSERT( pcRAMBuffer );
	configASSERT( pcFileName );

	/* Read back the files that were created by
	prvCreateDemoFilesUsing_ff_fwrite(). */
	for( xFileNumber = 1; xFileNumber <= xMaxFiles; xFileNumber++ )
	{
		/* Generate the file name. */
		snprintf( pcFileName, ffconfigMAX_FILENAME, "root%03d.txt", ( int ) xFileNumber );

		/* Obtain the current working directory and print out the file name and
		the	directory from which the file is being read. */
		ff_getcwd( pcRAMBuffer, fsRAM_BUFFER_SIZE );
		FF_PRINTF( "Reading file %s from %s\n", pcFileName, pcRAMBuffer );

		/* Open the file for reading. */
		pxFile = ff_fopen( pcFileName, "r" );
		configASSERT( pxFile );

		/* Read the file into the RAM buffer, checking the file contents are as
		expected.  The size of the file depends on the file number. */
		for( xReadNumber = 0; xReadNumber < xFileNumber; xReadNumber++ )
		{
			/* Start with the RAM buffer clear. */
			memset( pcRAMBuffer, 0x00, fsRAM_BUFFER_SIZE );

			xItemsRead = ff_fread( pcRAMBuffer, fsRAM_BUFFER_SIZE, 1, pxFile );
			configASSERT( xItemsRead == 1 );

			/* Check the RAM buffer is filled with the expected data.  Each
			file contains a different repeating ascii character that indicates
			the number of the file. */
			for( xChar = 0; xChar < fsRAM_BUFFER_SIZE; xChar++ )
			{
				configASSERT( pcRAMBuffer[ xChar ] == ( '0' + ( char ) xFileNumber ) );
			}
		}

		/* Close the file. */
		ff_fclose( pxFile );
	}

	vPortFree( pcRAMBuffer );
	vPortFree( pcFileName );

	/*_RB_ also test what happens when attempting to read using too large item
	sizes, etc. */
}
/*-----------------------------------------------------------*/

static void prvCreateDemoFileUsing_ff_fputc( const char *pcMountPath )
{
	int32_t iReturn, iByte, iReturned;
	FF_FILE *pxFile;
	char *pcRAMBuffer, *pcFileName;

	/* Allocate buffers used to hold date written to/from the disk, and the
	file names. */
	pcRAMBuffer = ( char * ) pvPortMalloc( fsRAM_BUFFER_SIZE );
	pcFileName = ( char * ) pvPortMalloc( ffconfigMAX_FILENAME );
	configASSERT( pcRAMBuffer );
	configASSERT( pcFileName );

	/* Obtain and print out the working directory. */
	ff_getcwd( pcRAMBuffer, fsRAM_BUFFER_SIZE );
	FF_PRINTF( "In directory %s\n", pcRAMBuffer );

	/* Create a sub directory. */
	iReturn = ff_mkdir( pcDirectory1 );
	configASSERT( iReturn == pdFREERTOS_ERRNO_NONE );

	/* Move into the created sub-directory. */
	iReturn = ff_chdir( pcDirectory1 );
	configASSERT( iReturn == pdFREERTOS_ERRNO_NONE );

	/* Obtain and print out the working directory. */
	ff_getcwd( pcRAMBuffer, fsRAM_BUFFER_SIZE );
	FF_PRINTF( "In directory %s\n", pcRAMBuffer );

	/* Create a subdirectory in the new directory. */
	iReturn = ff_mkdir( pcDirectory2 );
	configASSERT( iReturn == pdFREERTOS_ERRNO_NONE );

	/* Move into the directory just created - now two directories down from
	the root. */
	iReturn = ff_chdir( pcDirectory2 );
	configASSERT( iReturn == pdFREERTOS_ERRNO_NONE );

	/* Obtain and print out the working directory. */
	ff_getcwd( pcRAMBuffer, fsRAM_BUFFER_SIZE );
	FF_PRINTF( "In directory %s\n", pcRAMBuffer );
	snprintf( pcFileName, ffconfigMAX_FILENAME, "%s%s", pcMountPath, pcFullPath );
	configASSERT( strcmp( pcRAMBuffer, pcFileName ) == 0 );

	/* Generate the file name. */
	snprintf( pcFileName, ffconfigMAX_FILENAME, "%s.txt", pcDirectory2 );

	/* Print out the file name and the directory into which the file is being
	written. */
	FF_PRINTF( "Writing file %s in %s\n", pcFileName, pcRAMBuffer );

	pxFile = ff_fopen( pcFileName, "w" );
	configASSERT( pxFile );

	/* Create a file 1 byte at a time.  The file is filled with incrementing
	ascii characters starting from '0'. */
	for( iByte = 0; iByte < fsPUTC_FILE_SIZE; iByte++ )
	{
		iReturned = ff_fputc( ( ( int ) '0' + iByte ), pxFile );
		configASSERT( iReturned ==  ( ( int ) '0' + iByte ) );
	}

	/* Finished so close the file. */
	ff_fclose( pxFile );

	/* Move back to the root directory. */
	iReturned = ff_chdir( "../.." );
	configASSERT( iReturn == pdFREERTOS_ERRNO_NONE );

	/* Obtain and print out the working directory. */
	ff_getcwd( pcRAMBuffer, fsRAM_BUFFER_SIZE );
	FF_PRINTF( "Back in root directory %s\n", pcRAMBuffer );
	configASSERT( strcmp( pcRAMBuffer, pcMountPath ) == 0 );

	vPortFree( pcRAMBuffer );
	vPortFree( pcFileName );
}
/*-----------------------------------------------------------*/

static void prvVerifyDemoFileUsing_ff_fgetc( const char *pcMountPath )
{
	int iByte, iReturned;
	FF_FILE *pxFile;
	char *pcRAMBuffer, *pcFileName;

	/* Allocate buffers used to hold date written to/from the disk, and the
	file names. */
	pcRAMBuffer = ( char * ) pvPortMalloc( fsRAM_BUFFER_SIZE );
	pcFileName = ( char * ) pvPortMalloc( ffconfigMAX_FILENAME );
	configASSERT( pcRAMBuffer );
	configASSERT( pcFileName );

	/* Move into the directory in which the file was created. */
	snprintf( pcFileName, ffconfigMAX_FILENAME, "%s%s", pcMountPath, pcFullPath );
	iReturned = ff_chdir( pcFileName );
	configASSERT( iReturned == pdFREERTOS_ERRNO_NONE );

	/* Obtain and print out the working directory. */
	ff_getcwd( pcRAMBuffer, fsRAM_BUFFER_SIZE );
	FF_PRINTF( "Back in directory %s\n", pcRAMBuffer );
	configASSERT( strcmp( pcRAMBuffer, pcFileName ) == 0 );

	/* pcFileName is about to be overwritten - take a copy. */
	strcpy( pcRAMBuffer, pcFileName );

	/* Generate the file name. */
	sprintf( pcFileName, "%s.txt", pcDirectory2 );

	/* Print out the file name and the directory from which the file is being
	read. */
	FF_PRINTF( "Reading file %s in %s\n", pcFileName, pcRAMBuffer );

	/* This time the file is opened for reading. */
	pxFile = ff_fopen( pcFileName, "r" );

	/* Read the file 1 byte at a time. */
	for( iByte = 0; iByte < fsPUTC_FILE_SIZE; iByte++ )
	{
		iReturned = ff_fgetc( pxFile );
		configASSERT( iReturned ==  ( ( int ) '0' + iByte ) );
	}

	/* Should not be able to read another bytes. */
	iReturned = ff_fgetc( pxFile );
	configASSERT( iReturned ==  FF_EOF );

	/* Finished so close the file. */
	ff_fclose( pxFile );

	/* Move back to the root directory. */
	iReturned = ff_chdir( "../.." );

	/* Obtain and print out the working directory. */
	ff_getcwd( pcRAMBuffer, fsRAM_BUFFER_SIZE );
	FF_PRINTF( "Back in root directory %s\n", pcRAMBuffer );

	vPortFree( pcRAMBuffer );
	vPortFree( pcFileName );
}

