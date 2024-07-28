/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

/*
 * Adapted by Jason R. Thorpe for the PocketBASIC2040 SD card
 * driver.
 */

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

#include "sd.h"

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv	/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat = 0;
	int error;

	error = sd_stat();
	switch (error) {
	case SD_NOERR:
		break;

	case SD_EROFS:
		stat = STA_PROTECT;
		break;

	case SD_ENODEV:
		stat |= STA_NODISK;
		/* FALLTHROUGH */

	default:
		stat |= STA_NOINIT;
		break;
	}

	return stat;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv	/* Physical drive nmuber to identify the drive */
)
{
	sd_mount();
	return disk_status(pdrv);
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,	/* Physical drive nmuber to identify the drive */
	BYTE *buff,	/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count	/* Number of sectors to read */
)
{
	DRESULT res;
	int error;

	for (error = 0;
	     error == 0 && count != 0;
	     count--, buff += SD_SECSIZE, sector++) {
		error = sd_rdblk(sector, buff);
	}

	switch (error) {
	case 0:
		res = RES_OK;
		break;

	case SD_EINVAL:
		res = RES_PARERR;
		break;

	default:
		res = RES_ERROR;
		break;
	}

	return res;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,	 /* Physical drive nmuber to identify the drive */
	const BYTE *buff,/* Data to be written */
	LBA_t sector,	 /* Start sector in LBA */
	UINT count	 /* Number of sectors to write */
)
{
	DRESULT res;
	int error;

	for (error = 0;
	     error == 0 && count != 0;
	     count--, buff += SD_SECSIZE, sector++) {
		error = sd_wrblk(sector, buff);
	}

	switch (error) {
	case 0:
		res = RES_OK;
		break;

	case SD_EINVAL:
		res = RES_PARERR;
		break;

	case SD_EROFS:
		res = RES_WRPRT;
		break;

	default:
		res = RES_ERROR;
		break;
	}

	return res;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,	/* Physical drive nmuber (0..) */
	BYTE cmd,	/* Control code */
	void *buff	/* Buffer to send/receive control data */
)
{
	DRESULT res = RES_OK;
	uint32_t val32;

	switch (cmd) {
	case CTRL_SYNC:
		break;

	case GET_SECTOR_COUNT:
		if (sd_blkcnt(&val32) == SD_NOERR) {
			*(LBA_t *)buff = val32;
		} else {
			res = RES_NOTRDY;
		}
		break;

	case GET_SECTOR_SIZE:
		*(WORD *)buff = SD_SECSIZE;
		break;

	case GET_BLOCK_SIZE:
		if (sd_eblksz(&val32) == SD_NOERR) {
			*(DWORD *)buff = val32 / SD_SECSIZE;
		} else {
			res = RES_NOTRDY;
		}
		break;

	case CTRL_TRIM:
		break;

	default:
		res = RES_PARERR;
		break;
	}

	return res;
}
