#include "samsung_flash.h"
#include "FSR_SpinLock.h"
 
static UINT8 aMBuf[FSR_SECTOR_SIZE * FSR_MAX_PHY_SCTS];
//#if !defined (FSR_NBL2)
//static UINT8 bMBuf[FSR_SECTOR_SIZE * FSR_MAX_PHY_SCTS]; // For samsung_lld_write()
//#endif
 
typedef struct
{
    UINT32        nNumOfBadBlocks[FSR_DEVS_PER_VOL];
    UINT32        anRFSbn[FSR_DEVS_PER_VOL][FSR_MAX_BAD_BLKS];
}BackupDataInfo;
 
//static BackupDataInfo gstDataInfo[FSR_BML_MAX_PARTENTRY];
 
// The case using samsung_reset_fs_rebuild(), samsung_set_fs_rebuild() in single core
UINT32  nFSRebuildFlag  = 0;
 
// this code is used for dump operation.
#if !defined(FSR_NBL2)
static BOOL32   IsFirstDump = TRUE32;
#define DUMP_IMG_ADDR   (0x80000000)    /* This value can be changed */
#define DUMP_IMG_SIZE   (0x200000)    /* 2MB */
#endif
 
INT32
samsung_ftl_init(void)
{
    INT32   nRet;
    UINT32  nVol = 0;
#if defined (BUILD_JNAND)
    UINT32  nBytesReturned;
#endif
    static BOOL32 Ftl_Init_Flag = FALSE32;
 
#if defined (BUILD_JNAND)
    /* freeze watchdog */
    *((unsigned long *)0xB8000110) = 1;
#endif

    if (Ftl_Init_Flag == FALSE32)
    {
        nRet = FSR_BML_Init(FSR_BML_FLAG_NONE);
        if ((nRet != FSR_BML_SUCCESS)&&(nRet != FSR_BML_ALREADY_INITIALIZED))
        {
            return FS_DEVICE_FAIL;
        }
 
        nRet = FSR_BML_Open(nVol, FSR_BML_FLAG_NONE);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
 
#if defined (BUILD_JNAND)
        nRet = FSR_BML_IOCtl(nVol,
                             FSR_BML_IOCTL_UNLOCK_WHOLEAREA,
                             NULL,
                             0,
                             NULL,
                             0,
                             &nBytesReturned);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
#endif
 
        Ftl_Init_Flag = TRUE32;
    }
 
    return FS_DEVICE_OK;
}
 
INT32
samsung_bml_open(void)
{
        INT32   nRet;
        UINT32  nVol = 0;
 
        nRet = FSR_BML_Open(nVol, FSR_BML_FLAG_NONE);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
 
        return FS_DEVICE_OK;
 
}
 
INT32
samsung_bml_close(void)
{
#if !defined (FSR_NBL2)
 
        INT32   nRet;
        UINT32  nVol = 0;
 
        nRet = FSR_BML_Close(nVol, FSR_BML_FLAG_NONE);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
#endif
        return FS_DEVICE_OK;
 
}
 
INT32
samsung_bml_get_SLC_blocks(UINT32* nBlocks)
{
#if !defined (FSR_NBL2)
    INT32       nRet;
    UINT32      nVol = 0;
    UINT32	nLen = 0;
    UINT32	nPDev = 0;
    FSRSLCBoundary stSLCBoundary;
 
    nRet = FSR_BML_IOCtl(nVol, FSR_BML_IOCTL_GET_SLC_BOUNDARY,(UINT8 *) &nPDev, sizeof(nPDev),(UINT8 *)&stSLCBoundary, sizeof(FSRSLCBoundary), &nLen);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
    
    *nBlocks = stSLCBoundary.nLastSLCBlkNum[0];
 
#endif
    return FS_DEVICE_OK;
}

INT32
samsung_bml_format(FSRPartI* pstFSRPartI)
{
#if !defined (FSR_NBL2)
    INT32       nRet;
    UINT32      nVol = 0;
    FSRVolSpec  stVolSpec;
 
    nRet = FSR_BML_Init(FSR_BML_FLAG_NONE);
    if ((nRet != FSR_BML_SUCCESS) && (nRet != FSR_BML_ALREADY_INITIALIZED))
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Recalculate the last partition size */
    if (pstFSRPartI->stPEntry[pstFSRPartI->nNumOfPartEntry-1].nNumOfUnits == 0xFFFF)
    {
        /* Get volume information */
        nRet = FSR_BML_GetVolSpec(nVol,
                                  &stVolSpec,
                                  FSR_BML_FLAG_NONE);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
 
        pstFSRPartI->stPEntry[pstFSRPartI->nNumOfPartEntry-1].nNumOfUnits = \
        stVolSpec.nNumOfUsUnits - pstFSRPartI->stPEntry[pstFSRPartI->nNumOfPartEntry-1].n1stVun;
    }
 
    nRet = FSR_BML_Format(nVol,
                          pstFSRPartI,
                          (FSR_BML_INIT_FORMAT | FSR_BML_AUTO_ADJUST_PARTINFO));
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_stl_init(UINT32 nPartID)
{
#if defined (FSR_NW)
           INT32       nRet;
           UINT32      nVol = 0;
           FSRStlInfo  stSTLInfo;
    static BOOL32      Stl_Init_Flag = FALSE32;
 
    if (Stl_Init_Flag == FALSE32)
    {
        nRet = FSR_STL_Init();
        if ((nRet != FSR_STL_SUCCESS)&&(nRet != FSR_STL_ALREADY_INITIALIZED))
        {
            return FS_DEVICE_FAIL;
        }
 
        nRet = FSR_STL_Open(nVol,
                            nPartID,
                            &stSTLInfo,
                            FSR_STL_FLAG_DEFAULT);
        if (nRet != FSR_STL_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
 
        Stl_Init_Flag = TRUE32;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_stl_open(UINT32 nPartID)
{
#if defined (FSR_NW)
    INT32 nRet;
    UINT32 nVol = 0;
    FSRStlInfo  stSTLInfo;
 
    nRet = FSR_STL_Open(nVol,
                        nPartID,
                        &stSTLInfo,
                        FSR_STL_FLAG_DEFAULT);
    if (nRet != FSR_STL_SUCCESS && nRet != FSR_STL_PARTITION_ALREADY_OPENED)
    {  
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_stl_close(UINT32 nPartID)
{
#if defined (FSR_NW)
    INT32 nRet;
    UINT32 nVol = 0;
 
    nRet = FSR_STL_Close(nVol,
                         nPartID);
    if (nRet != FSR_STL_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_stl_format(UINT32 nPartID)
{
#if defined (FSR_NW)
    INT32           nRet;
    INT32           nTmpRet;
    UINT32          nVol = 0;
    FSRStlFmtInfo   stStlFmt;
    UINT32          n1stVpn;
    UINT32          nPgsPerUnit;
    FSRPartEntry    stPartEntry;
    FSRChangePA     stChangePA;
    UINT32          nByteRet;
      
    nRet = FSR_STL_Init();
    if ((nRet != FSR_STL_SUCCESS) && (nRet != FSR_STL_ALREADY_INITIALIZED))
    {
        return FS_DEVICE_FAIL;
    }
 
    /* BML_Open 
       BML_LoadPIEntry can be used only if BML_Open is called */
    nRet = samsung_bml_open();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
 
    do
    {
        /* Load partition information of nID */
        nRet = FSR_BML_LoadPIEntry(nVol,
                                   nPartID,
                                   &n1stVpn,
                                   &nPgsPerUnit,
                                   &stPartEntry);
        if (nRet != FSR_BML_SUCCESS)
        {
            nRet = FS_DEVICE_FAIL;
            break;
        }
 
        /* Unlock Unit for erase operation */
        nRet = samsung_bml_unlock_wholearea();
        if (nRet != FS_DEVICE_OK)
        {
            break;
        }

 #if defined (BUILD_JNAND)
    /* freeze watchdog */
    *((unsigned long *)0xB8000110) = 1;
#endif

        /* Erase STL Partition */
        nRet = samsung_bml_erase(stPartEntry.n1stVun, stPartEntry.nNumOfUnits);
        if (nRet != FS_DEVICE_OK)
        {
            break;
        }
 
    }while(0);
        
    stChangePA.nPartID  = nPartID;
    stChangePA.nNewAttr = FSR_BML_PI_ATTR_RW;
    if (FSR_BML_IOCtl(nVol, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) 
    {
        return FS_DEVICE_FAIL;
    }
      
    stStlFmt.nOpt = FSR_STL_FORMAT_NONE;
 
    nRet = FSR_STL_Format(nVol,
                          nPartID,
                          &stStlFmt);
    if ((nRet != FSR_STL_SUCCESS) && (nRet != FSR_STL_PROHIBIT_FORMAT_BY_GWL))
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_stl_read(UINT32 nLsn, UINT32 nImgSize, UINT8* pBuf, UINT32 nPartID)
{
#if defined (FSR_NW)
    INT32   nRet;
    UINT32  nVol = 0;
    UINT32  nNumOfScts;
 
    /* Calculate # of sectors to be written */
    nNumOfScts = (nImgSize % FSR_SECTOR_SIZE) ? (nImgSize / FSR_SECTOR_SIZE + 1) : (nImgSize / FSR_SECTOR_SIZE);
 
    nRet = FSR_STL_Read(nVol,
                        nPartID,
                        nLsn,
                        nNumOfScts,
                        pBuf,
                        FSR_STL_FLAG_DEFAULT);
    if (nRet != FSR_STL_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_stl_write(UINT32 nLsn, UINT32 nImgSize, UINT8* pBuf, UINT32 nPartID)
{
#if defined (FSR_NW)
    INT32   nRet;
    UINT32  nVol = 0;
    UINT32  nNumOfScts;
 
    /* Calculate # of sectors to be written */
    nNumOfScts = (nImgSize % FSR_SECTOR_SIZE) ? (nImgSize / FSR_SECTOR_SIZE + 1) : (nImgSize / FSR_SECTOR_SIZE);
 
    nRet = FSR_STL_Write(nVol,
                         nPartID,
                         nLsn,
                         nNumOfScts,
                         pBuf,
                         FSR_STL_FLAG_DEFAULT);
    if (nRet != FSR_STL_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_stl_delete(UINT32 nLsn, UINT32 nNumOfScts, UINT32 nPartID)
{
#if defined (FSR_NW)
    INT32   nRet;
    UINT32  nVol = 0;
 
    nRet = FSR_STL_Delete (nVol,
                           nPartID,
                           nLsn,
                           nNumOfScts,
                           FSR_STL_FLAG_DEFAULT);
    if (nRet != FSR_STL_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_stl_get_logscts(UINT32 nPartID)
{
    UINT32 nNumOfLogScts    = 0;
#if defined (FSR_NW)
    INT32 nRet;
    UINT32 nVol = 0;
    UINT32 nBytesReturned;
 
    nRet = FSR_STL_IOCtl(nVol,
                         nPartID,
                         FSR_STL_IOCTL_LOG_SECTS,
                         NULL,
                         0,
                         (VOID *) &nNumOfLogScts,
                         sizeof(nNumOfLogScts),
                         &nBytesReturned);
    if (nRet != FSR_STL_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
 
    return (INT32)nNumOfLogScts;
}
 
INT32
samsung_bml_write(UINT32 nStartVpn, UINT32 nNumOfPgs, UINT8* pMBuf, FSRSpareBuf *pSBuf)
{
#if !defined (FSR_NBL2)
    INT32             nRet;
    UINT32            nVol = 0;
    UINT32            nBMLFlag = FSR_BML_FLAG_NONE;
 
    if (pSBuf != NULL)
    {
        nBMLFlag = FSR_BML_FLAG_USE_SPAREBUF;
    }
 
    nRet = FSR_BML_Write(nVol,
                         nStartVpn,
                         nNumOfPgs,
                         pMBuf,
                         pSBuf,
                         (FSR_BML_FLAG_ECC_ON | nBMLFlag));
 
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_bml_write_ext(UINT32 nStartVpn, UINT32 nNumOfPgs, UINT8* pMBuf, UINT8* pSBuf)
{
#if !defined (FSR_NBL2)
#if defined(FSR_BML_RWEXT)
    INT32             nRet;
    UINT32            nVol = 0;
 
    nRet = FSR_BML_WriteExt(nVol,
                         nStartVpn,
                         nNumOfPgs,
                         pMBuf,
                         pSBuf,
                         FSR_BML_FLAG_NONE);
 
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_bml_erase(UINT32 nVun, UINT32 nNumOfUnits)
{
#if !defined (FSR_NBL2)
    INT32   nRet ;
    UINT32  nVol = 0;
    UINT32  nCnt = 0;
 
    for (nCnt = nVun; nCnt < nNumOfUnits+nVun; nCnt++)
    {
        nRet = FSR_BML_Erase(nVol, &nCnt, 1, FSR_BML_FLAG_NONE);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_is_page_erased(UINT32 nPage)
{
    INT32           nRet ;
    UINT32          nVol = 0;
    UINT32          nCnt = 0;
    FSRVolSpec      stVolSpec;
 
    /* Get volume information */
    nRet = FSR_BML_GetVolSpec(nVol,
                              &stVolSpec,
                              FSR_BML_FLAG_NONE);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FALSE32;
    }
 
    nRet = samsung_bml_read(nPage * stVolSpec.nSctsPerPg,
                            stVolSpec.nSctsPerPg,
                            (UINT8*)aMBuf,
                            NULL);
    if (nRet != FS_DEVICE_OK)
    {
        return FALSE32;
    }
 
    for (nCnt = 0; nCnt < FSR_SECTOR_SIZE * (UINT32)stVolSpec.nSctsPerPg; nCnt+=sizeof(UINT32))
    {
        if (*(UINT32*)(aMBuf + nCnt) != 0xFFFFFFFF)
        {
            return FALSE32;
        }
    }
 
    return TRUE32;
}
 
INT32
samsung_bml_read(UINT32 nStartScts, UINT32 nNumOfScts, UINT8* pMBuf, FSRSpareBuf *pSBuf)
{
    INT32       nRet       = FSR_BML_SUCCESS;
    UINT32      nVol       = 0;
    UINT32      nStartVpn  = 0;
    UINT32      n1stSecOff = 0;
    UINT32      nBMLFlag   = FSR_BML_FLAG_NONE;
    FSRVolSpec  stVolSpec;
 
    /* Get volume information */
    nRet = FSR_BML_GetVolSpec(nVol,
                              &stVolSpec,
                              FSR_BML_FLAG_NONE);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FALSE32;
    }
 
    /* Calculate nStartVpn, n1stSecOff */
    nStartVpn  = nStartScts / stVolSpec.nSctsPerPg;
    n1stSecOff = (nStartScts & (stVolSpec.nSctsPerPg-1));
 
    if (pSBuf != NULL)
    {
        nBMLFlag = FSR_BML_FLAG_USE_SPAREBUF;
    }
 
    if (((UINT32) pMBuf == 0x0) || ((UINT32) pMBuf == 0x1000) || ((UINT32) pMBuf == 0x2000))
    {
        if (nNumOfScts > FSR_MAX_PHY_SCTS)
        {
          //the size of aMBuf is only 4096
          return FS_DEVICE_FAIL;
        }
 
        nRet = FSR_BML_ReadScts(nVol,
                              nStartVpn,
                              n1stSecOff,
                              nNumOfScts,
                              aMBuf,
                              pSBuf,
                              FSR_BML_FLAG_ECC_ON | nBMLFlag);
 
        if (nRet != FSR_BML_SUCCESS)
        {
          return FS_DEVICE_FAIL;
        }
 
        FSR_OAM_MEMCPY(pMBuf, aMBuf, nNumOfScts * FSR_SECTOR_SIZE);
 
    }
    else
    {
        nRet = FSR_BML_ReadScts(nVol,
                              nStartVpn,
                              n1stSecOff,
                              nNumOfScts,
                              pMBuf,
                              pSBuf,
                              FSR_BML_FLAG_ECC_ON | nBMLFlag);
        if (nRet != FSR_BML_SUCCESS)
        {
          return FS_DEVICE_FAIL;
        }
    }
 
    return FS_DEVICE_OK;
}
 
INT32
samsung_bml_read_ext(UINT32 nStartVpn, UINT32 nNumOfPages, UINT8* pMBuf, UINT8* pSBuf)
{
#if !defined (FSR_NBL2)
#if defined(FSR_BML_RWEXT)
    INT32       nRet       = FSR_BML_SUCCESS;
    UINT32      nVol       = 0;
    FSRVolSpec  stVolSpec;
 
    /* Get volume information */
    nRet = FSR_BML_GetVolSpec(nVol,
                              &stVolSpec,
                              FSR_BML_FLAG_NONE);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    if (((UINT32) pMBuf == 0x0) || ((UINT32) pMBuf == 0x1000) || ((UINT32) pMBuf == 0x2000))
    {
        if (nNumOfPages > 1)
        {
          //the size of aMBuf is only 4096
          return FS_DEVICE_FAIL;
        }
 
        nRet = FSR_BML_ReadExt(nVol,
                              nStartVpn,
                              nNumOfPages,
                              aMBuf,
                              pSBuf,
                              FSR_BML_FLAG_NONE);
 
        if (nRet != FSR_BML_SUCCESS)
        {
          return FS_DEVICE_FAIL;
        }
 
        FSR_OAM_MEMCPY(pMBuf, aMBuf, stVolSpec.nSctsPerPg * FSR_SECTOR_SIZE);
 
    }
    else
    {
        nRet = FSR_BML_ReadExt(nVol,
                              nStartVpn,
                              nNumOfPages,
                              pMBuf,
                              pSBuf,
                              FSR_BML_FLAG_NONE);
        if (nRet != FSR_BML_SUCCESS)
        {
          return FS_DEVICE_FAIL;
        }
    }
#endif
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_bml_lock_op (UINT32 nStartUnit, UINT32 nNumOfUnits, UINT32 nOpcode)
{
#if !defined (FSR_NBL2)
    INT32            nRet;
    INT32            nErr = FS_DEVICE_OK;
    UINT32           nVol = 0;
    UINT32           nBytesReturned;
    UINT32           nDevIdx = 0;
    UINT32           n1stVpn;
    UINT32           nPgsPerUnit;
    UINT32           nRsrvBlk;
    UINT32           nCnt;
    UINT32           nDieIdx;
    UINT32           nBMFIdx;
    UINT32           nSbn;
    UINT32           nPbn;
    UINT32           nLockStat;
    UINT32           nLLDCode;
    UINT32           nErrPbn;
    FSRDevSpec       stDevSpec;
    FSRBMInfo        stBMInfo;
    BmlBMF           stBmf[FSR_MAX_DIES][FSR_MAX_BAD_BLKS];
    FSRLowFuncTbl    stLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   *pstLFT[FSR_MAX_VOLS];
    LLDProtectionArg stLLDIO;
 
    /* check input parameter */
    if ((nOpcode != FLASH_UNLOCK_OP)    ||
        (nOpcode != FLASH_LOCK_OP)      ||
        (nOpcode != FLASH_LOCKTIGHT_OP))
    {
        return FS_DEVICE_NOT_SUPPORTED;
    }
 
    /* Get BMI */
    FSR_OAM_MEMSET(&stBMInfo, 0x00, sizeof(stBMInfo));
    stBMInfo.pstBMF[0] = &stBmf[0][0];
    stBMInfo.pstBMF[1] = &stBmf[1][0];
 
    nRet = FSR_BML_IOCtl(nVol,
                         FSR_BML_IOCTL_GET_BMI,
                         (UINT8*)&nDevIdx,
                         sizeof(nDevIdx),
                         (UINT8*)&stBMInfo,
                         sizeof(stBMInfo),
                         &nBytesReturned);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Acquire SM */
    FSR_BML_AcquireSM(nVol);
 
    do
    {
        /* Register LLD Function Table */
        FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
        pstLFT[0] = &stLFT[0];
        pstLFT[1] = &stLFT[1];
 
        nRet = FSR_PAM_RegLFT(pstLFT);
        if (nRet != FSR_PAM_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        nRet = pstLFT[nVol]->LLD_GetDevSpec(nDevIdx,
                                            &stDevSpec,
                                            FSR_LLD_FLAG_NONE);
        if (nRet != FSR_LLD_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        /* Find the SLC/MLC attributes of units
          (Let's suppose all units have same attributes) */
        nRet = FSR_BML_GetVirUnitInfo(nVol,
                                      nStartUnit,
                                      &n1stVpn,
                                      &nPgsPerUnit);
        if (nRet != FSR_BML_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        nRsrvBlk = 0;
        if (nPgsPerUnit == (stDevSpec.nPgsPerBlkForMLC*stDevSpec.nNumOfDies))
        {
            nRsrvBlk = 30;
        }
 
        for (nCnt = nStartUnit; nCnt<(nStartUnit+nNumOfUnits); nCnt++)
        {
            for (nDieIdx = 0; stDevSpec.nNumOfDies; nDieIdx++)
            {
                /* Change the Vun to the Pbn */
                nSbn = nRsrvBlk + nCnt + nDieIdx*(stDevSpec.nNumOfBlks/stDevSpec.nNumOfDies);
                nPbn = nSbn;
                for (nBMFIdx = 0; nBMFIdx<stBMInfo.nNumOfBMFs[nDieIdx]; nBMFIdx++)
                {
                    if (nSbn == stBMInfo.pstBMF[nDieIdx]->nSbn)
                    {
                        nPbn = stBMInfo.pstBMF[nDieIdx]->nRbn;
                        break;
                    }
                    stBMInfo.pstBMF[nDieIdx]++;
                }
 
                /* check the block lock state */
                nRet = pstLFT[nVol]->LLD_IOCtl(nDevIdx,
                                               FSR_LLD_IOCTL_GET_LOCK_STAT,
                                               (UINT8*) &nPbn,
                                               sizeof(nPbn),
                                               (UINT8*) &nLockStat,
                                               sizeof(nLockStat),
                                               &nBytesReturned);
                if (nRet != FSR_LLD_SUCCESS)
                {
                    nErr = FS_DEVICE_FAIL;
                    break;
                }
 
                if ((nLockStat  == FSR_LLD_BLK_STAT_LOCKED_TIGHT) &&
                    (nOpcode    != FLASH_LOCKTIGHT_OP))
                {
                    nErr = FS_DEVICE_FAIL;
                    break;
                }
 
                if (nOpcode == FLASH_UNLOCK_OP)
                {
                    nLLDCode = FSR_LLD_IOCTL_UNLOCK_BLOCK;
                }
                else if (nOpcode == FLASH_LOCK_OP)
                {
                    nLLDCode = FSR_LLD_IOCTL_LOCK_BLOCK;
                }
                else if (nOpcode == FLASH_LOCKTIGHT_OP)
                {
                     nLLDCode = FSR_LLD_IOCTL_LOCK_TIGHT;
 
                    /* change the block state to lock state */
                    stLLDIO.nStartBlk = nPbn;
                    stLLDIO.nBlks     = 1;
 
                    nRet = pstLFT[nVol]->LLD_IOCtl(nDevIdx ,
                                                   FSR_LLD_IOCTL_LOCK_BLOCK,
                                                   (UINT8 *) &stLLDIO,
                                                   sizeof(stLLDIO),
                                                   (UINT8 *) &nErrPbn,
                                                   sizeof(nErrPbn),
                                                   &nBytesReturned);
                    if (nRet != FSR_LLD_SUCCESS)
                    {
                        nErr = FS_DEVICE_FAIL;
                        break;
                    }
                }
 
                /* change the block lock state according to nOpcode */
                stLLDIO.nStartBlk = nPbn;
                stLLDIO.nBlks     = 1;
                nRet = pstLFT[nVol]->LLD_IOCtl(nDevIdx,
                                               nLLDCode,
                                               (UINT8 *) &stLLDIO,
                                               sizeof(stLLDIO),
                                               (UINT8 *) &nErrPbn,
                                               sizeof(nErrPbn),
                                               &nBytesReturned);
                if (nRet != FSR_LLD_SUCCESS)
                {
                    nErr = FS_DEVICE_FAIL;
                    break;
                }
 
            } /* End of "for (nDieIdx = 0;...)" */
 
            if (nErr != FS_DEVICE_OK)
            {
                break;
            }
 
        } /* End of "for (nCnt = nStartUnit;...)" */
 
    } while(0);
 
    /* Release SM */
    FSR_BML_ReleaseSM(nVol);
 
    return nErr;
#else
    return FS_DEVICE_OK;
#endif
}
 
UINT32
samsung_retrieve_ID(void)
{
    INT32           nRet;
    UINT32          nVol = 0;
    UINT32          nDev = 0;
    UINT32          nDID = 0;
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    FSRDevSpec      stDevSpec;
 
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FALSE32;
    }
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FALSE32;
    }
 
    FSR_BML_AcquireSM(nVol);
 
    do
    {
        nRet = pstLFT[nVol]->LLD_GetDevSpec(nDev,
                                            &stDevSpec,
                                            FSR_LLD_FLAG_NONE);
 
        if (nRet == FSR_LLD_SUCCESS)
        {
            nDID = stDevSpec.nDID;
        }
    } while(0);
 
    FSR_BML_ReleaseSM(nVol);
 
    return nDID;
}
 
INT32
samsung_set_fs_rebuild(void)
{
#if !defined (FSR_NBL2)
    INT32   nRet;
    UINT32  nVol = 0;
    UINT32  nDev = 0;
    UINT32  nPbn;
    UINT32  nRebuilPage;
 
    FSRDevSpec        stDevSpec;
    FSRLowFuncTbl     stLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl    *pstLFT[FSR_MAX_VOLS];
    UINT32            nDevIdx = 0;
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return  FS_DEVICE_FAIL;
    }
 
    FSR_BML_AcquireSM(nVol);
    
    nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
    if (nRet != FSR_BML_SUCCESS)
    {
        return  FS_DEVICE_FAIL;
    }
        
    nRet = pstLFT[nVol]->LLD_GetDevSpec(nDevIdx,
                                        &stDevSpec,
                                        FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return  FS_DEVICE_FAIL;
    }
                           
    /* The case that fs is located SLC Unit */
    nPbn = FS_REBUILD_BLOCK;
    nRebuilPage = (FS_REBUILD_BLOCK) * stDevSpec.nPgsPerBlkForSLC + stDevSpec.nPgsPerBlkForSLC - 1;
 
    nRet = pstLFT[nVol]->LLD_Read(nDev,
                              nPbn,
                              nRebuilPage,
                              aMBuf,
                              NULL,
                              FSR_LLD_FLAG_NONE);
 
    FSR_BML_ReleaseSM(nVol);
    if (nRet == FSR_LLD_SUCCESS)
    {
        if((*(UINT32*) aMBuf     ) == FS_REBUILD_MAGIC1 &&
           (*(UINT32*)(aMBuf + 4)) == FS_REBUILD_MAGIC2)
        {
            /* Set rebuild flag in single core */
            nFSRebuildFlag = 1;
        }
        return FS_DEVICE_OK;
    } 
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_reset_fs_rebuild(void)
{
    INT32             nRet = FS_DEVICE_OK;
#if !defined (FSR_NBL2)
    UINT32            nVol = 0;
    UINT32            nDevIdx = 0;
    UINT32            nRetByte;
    UINT32            nRebuilPage;
    UINT32            nPbn;
    UINT32            nPageOffset;
    FSRDevSpec        stDevSpec;
    UINT32            nErrPbn;
    FSRLowFuncTbl     stLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl    *pstLFT[FSR_MAX_VOLS];
    LLDProtectionArg  stLLDIO;
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return  FS_DEVICE_FAIL;
    }
 
    FSR_BML_AcquireSM(nVol);
    
    nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
    if (nRet != FSR_BML_SUCCESS)
    {
        return  FS_DEVICE_FAIL;
    }
 
    nRet = pstLFT[nVol]->LLD_GetDevSpec(nDevIdx,
                                        &stDevSpec,
                                        FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return  FS_DEVICE_FAIL;
    }
   
    /* The case that fs is located SLC Unit */
    nPbn = FS_REBUILD_BLOCK;
    nRebuilPage = (FS_REBUILD_BLOCK) * stDevSpec.nPgsPerBlkForSLC + stDevSpec.nPgsPerBlkForSLC - 1;
   
    /* The case that fs is located SLC Unit */
    nRet = pstLFT[nVol]->LLD_Read(nDevIdx,
                              nPbn,
                              nRebuilPage,
                              aMBuf,
                              NULL,
                              FSR_LLD_FLAG_NONE);
 
    FSR_BML_ReleaseSM(nVol);
 
    if (nRet != FSR_LLD_SUCCESS)
    {
        /* after overprogramming the page which contains rebuild flag
           samsung_bml_read() returns read error
           so, ignore read error [namo.hwang 090202]
        */    
        return FS_DEVICE_OK;
    }
 
    if((*(UINT32*) aMBuf     ) != FS_REBUILD_MAGIC1 ||
       (*(UINT32*)(aMBuf + 4)) != FS_REBUILD_MAGIC2)
    {
        nFSRebuildFlag = 0;
        return FS_DEVICE_OK;
    }
 
    nFSRebuildFlag = 0;
 
    do
    {
        FSR_BML_AcquireSM(nVol);
 
        do
        {
            nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
            if (nRet != FSR_BML_SUCCESS)
            {
                nRet = FS_DEVICE_FAIL;
                break;
            }
 
            /* change the block state to unlock state */
            stLLDIO.nStartBlk = FS_REBUILD_BLOCK;
            stLLDIO.nBlks     = 1;
 
            nRet = pstLFT[nVol]->LLD_IOCtl(nDevIdx ,
                                           FSR_LLD_IOCTL_UNLOCK_BLOCK,
                                           (UINT8 *) &stLLDIO,
                                           sizeof(stLLDIO),
                                           (UINT8 *) &nErrPbn,
                                           sizeof(nErrPbn),
                                           &nRetByte);
            if (nRet != FSR_LLD_SUCCESS)
            {
                nRet = FS_DEVICE_FAIL;
                break;
            }
        } while(0);
 
        FSR_BML_ReleaseSM(nVol);
 
        if (nRet == FS_DEVICE_FAIL)
        {
            break;
        }
 
        FSR_OAM_MEMSET(aMBuf, 0x00, FSR_SECTOR_SIZE * stDevSpec.nSctsPerPG);
 
        nPageOffset = stDevSpec.nPgsPerBlkForSLC - 1;
 
        nRet = samsung_lld_write(stLLDIO.nStartBlk,
                                 nPageOffset,
                                 FSR_SECTOR_SIZE * stDevSpec.nSctsPerPG,
                                 aMBuf);
        if (nRet != FS_DEVICE_OK)
        {
            nRet = FS_DEVICE_FAIL;
            break;
        }
 
        FSR_BML_AcquireSM(nVol);
        do
        {
            nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
            if (nRet != FSR_BML_SUCCESS)
            {
                nRet = FS_DEVICE_FAIL;
                break;
            }
 
            nRet = pstLFT[nVol]->LLD_IOCtl(nDevIdx ,
                                           FSR_LLD_IOCTL_LOCK_BLOCK,
                                           (UINT8 *) &stLLDIO,
                                           sizeof(stLLDIO),
                                           (UINT8 *) &nErrPbn,
                                           sizeof(nErrPbn),
                                           &nRetByte);
            if (nRet != FSR_LLD_SUCCESS)
            {
                nRet = FS_DEVICE_FAIL;
                break;
            }

            nRet = FS_DEVICE_OK;
        }while (0);
 
        FSR_BML_ReleaseSM(nVol);
    } while(0); 
#endif

// namohwang
//    return FS_DEVICE_OK;
    return nRet;
}
 
INT32
samsung_init_fs(void)
{
#if !defined (FSR_NBL2)
    UINT32         nVol = 0;
    UINT32         nDevIdx = 0;
    INT32          nRet;
    UINT32         nPbn;
    UINT32         nPageOffset;
    FSRDevSpec     stDevSpec;
    FSRLowFuncTbl  stLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl *pstLFT[FSR_MAX_VOLS];
 
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
       return FALSE32;
    }
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return  FS_DEVICE_FAIL;
    }
 
    nRet = pstLFT[nVol]->LLD_GetDevSpec(nDevIdx,
                                        &stDevSpec,
                                        FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return  FS_DEVICE_FAIL;
    }
 
    nPbn = FS_REBUILD_BLOCK;
 
    nPageOffset = stDevSpec.nPgsPerBlkForSLC - 1;
 
    FSR_OAM_MEMSET(aMBuf, 0xFF, FSR_SECTOR_SIZE * stDevSpec.nSctsPerPG);
 
 
    *(UINT32*) aMBuf      = FS_REBUILD_MAGIC1;
    *(UINT32*)(aMBuf + 4) = FS_REBUILD_MAGIC2;
 
 
    nRet = samsung_lld_write(nPbn, nPageOffset, FSR_SECTOR_SIZE * stDevSpec.nSctsPerPG, aMBuf);
    if (nRet != FS_DEVICE_OK)
    {
        return  FS_DEVICE_FAIL;
    }
 
    return FS_DEVICE_OK;
#else
    return FS_DEVICE_OK;
#endif
}
 
 
INT32
samsung_lld_get_devspec(FSRDevSpec *pstDevSpec)
{
    INT32           nRet;
    UINT32          nVol = 0;
    UINT32          nDev = 0;
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Acquire SM */
    FSR_BML_AcquireSM(nVol);

    nRet = pstLFT[nVol]->LLD_GetDevSpec(nDev, pstDevSpec, FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Release SM */
    FSR_BML_ReleaseSM(nVol);

    return FS_DEVICE_OK;
}
 
INT32
samsung_lld_write(UINT32 nPbn, UINT32 nPageOffset, UINT32 nSize, UINT8* pMBuf)
{
#if !defined (FSR_NBL2)
    INT32           nRet;
    INT32           nErr = FS_DEVICE_OK;
    UINT32          nVol = 0;
    UINT32          nDev = 0;
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    FSRSpareBuf     stSpareBuf;
    FSRSpareBufBase stSpareBufBase;
    FSRSpareBufExt  stSpareBufExt[FSR_MAX_SPARE_BUF_EXT];
    FSRDevSpec      stDevSpec;
    UINT32          nSizeOfPage;
    UINT32          nNumOfPages;
    UINT32          nCnt;
 
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Acquire SM */
    FSR_BML_AcquireSM(nVol);
 
    do
    {
        nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
        if (nRet != FSR_BML_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        /* Register LLD Function Table */
        FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
        pstLFT[0] = &stLFT[0];
        pstLFT[1] = &stLFT[1];
 
        nRet = FSR_PAM_RegLFT(pstLFT);
        if (nRet != FSR_PAM_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        nRet = pstLFT[nVol]->LLD_GetDevSpec(nDev, &stDevSpec, FSR_LLD_FLAG_NONE);
        if (nRet != FSR_LLD_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        nSizeOfPage = stDevSpec.nSctsPerPG * FSR_SECTOR_SIZE;
        nNumOfPages = (nSize % nSizeOfPage) ? (nSize / nSizeOfPage + 1) : (nSize / nSizeOfPage);
 
        /* Set spare buffer */
        FSR_OAM_MEMSET(&stSpareBufBase, 0xFF, sizeof(stSpareBufBase));
        FSR_OAM_MEMSET(&stSpareBufExt[0], 0xFF, sizeof(stSpareBufExt));
 
        stSpareBuf.pstSpareBufBase  = &stSpareBufBase;
        stSpareBuf.nNumOfMetaExt    = 2;
        stSpareBuf.pstSTLMetaExt    = &stSpareBufExt[0];
 
        for (nCnt = 0; nCnt < nNumOfPages; nCnt++)
        {
            /* if the pMBuf is smaller than page size */
            if ((nCnt == nNumOfPages -1) &&
                ((nSize % nSizeOfPage) != 0))
            {
                FSR_OAM_MEMSET(aMBuf, 0xFF, sizeof(aMBuf));
                FSR_OAM_MEMCPY(aMBuf, pMBuf, nSize % nSizeOfPage);
                pMBuf = aMBuf;
            }
 
            nRet = pstLFT[nVol]->LLD_Write(nDev,
                                           nPbn,
                                           nPageOffset + nCnt,
                                           pMBuf,
                                           &stSpareBuf,
                                           (FSR_LLD_FLAG_1X_PROGRAM | FSR_LLD_FLAG_ECC_ON | FSR_LLD_FLAG_USE_SPAREBUF));
            if (nRet != FSR_LLD_SUCCESS)
            {
                nErr = FS_DEVICE_FAIL;
                break;
            }
            nRet = pstLFT[nVol]->LLD_FlushOp(nDev,
                                             0,
                                             FSR_LLD_FLAG_NONE);
            if (nRet != FSR_LLD_SUCCESS)
            {
                nErr = FS_DEVICE_FAIL;
                break;
            }
 
            pMBuf += nSizeOfPage;
        }
 
    } while (0);
 
    /* Release SM */
    FSR_BML_ReleaseSM(nVol);
 
    return nErr;
#else
    return FS_DEVICE_OK;
#endif
}
 
INT32
samsung_dump_bootimg(UINT32 nID, UINT8* pBuf, UINT32* pDumpSize)
{
#if defined (BUILD_JNAND)
    INT32   nRet;
    UINT32  nVol = 0;
    UINT32  n1stVpn;
    UINT32  nPgsPerUnit;
    FSRPartEntry stPartEntry;
    UINT32  nUnitCnt;
    UINT32  nDumpSize;
    UINT8*  pTmpBuf;
    FSRVolSpec  stVolSpec;
 
    pTmpBuf = pBuf;
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Get volume information */
    nRet = FSR_BML_GetVolSpec(nVol, &stVolSpec, FSR_BML_FLAG_NONE);
    if (nRet != FSR_BML_SUCCESS)
    {
         return FS_DEVICE_FAIL;
    }
 
    nRet = FSR_BML_LoadPIEntry(nVol, nID, &n1stVpn, &nPgsPerUnit,  &stPartEntry);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    for (nUnitCnt = 0; nUnitCnt < stPartEntry.n1stVun + stPartEntry.nNumOfUnits; nUnitCnt++)
    {
        nRet = FSR_BML_Read(nVol, nUnitCnt * nPgsPerUnit, nPgsPerUnit, pTmpBuf, NULL, FSR_BML_FLAG_NONE);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
 
        pTmpBuf +=  nPgsPerUnit * stVolSpec.nSctsPerPg * FSR_SECTOR_SIZE;
    }
 
 
    nDumpSize = nUnitCnt * nPgsPerUnit * stVolSpec.nSctsPerPg * FSR_SECTOR_SIZE;
    /* The original dump size is "nDumpSize
       But, this value has been modified for T32 */
    *pDumpSize = nDumpSize - 1;
 
#endif
    return FS_DEVICE_OK;
}
 
VOID
samsung_smem_init(void)
{
    static BOOL32 smem_Init_Flag = FALSE32;
 
    if(!smem_Init_Flag)
    {
        FSR_PAM_Init();
        FSR_OAM_Init();
#if defined(FSR_USE_DUAL_CORE)
        FSR_OAM_InitSharedMemory();
#endif
        smem_Init_Flag = TRUE32;
    }
}
 
INT32
samsung_block_count(void)
{
    INT32           nRet;
    FSRVolSpec  stVolSpec;
    UINT32          nVol = 0;
 
#if defined (BUILD_JNAND)
	nRet = FSR_BML_Init(FSR_BML_FLAG_NONE);
	if ((nRet != FSR_BML_SUCCESS)&&(nRet != FSR_BML_ALREADY_INITIALIZED))
	{
		return FS_DEVICE_FAIL;
	}

#else
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
#endif
 
    nRet = FSR_BML_GetVolSpec(nVol,
                              &stVolSpec,
                              FSR_BML_FLAG_NONE);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    return stVolSpec.nNumOfUsUnits;
}
 
INT32
samsung_block_size(void)
{
    INT32           nRet;
    FSRVolSpec  stVolSpec;
    UINT32          nVol = 0;
 
#if defined (BUILD_JNAND)
	nRet = FSR_BML_Init(FSR_BML_FLAG_NONE);
	if ((nRet != FSR_BML_SUCCESS)&&(nRet != FSR_BML_ALREADY_INITIALIZED))
	{
		return FS_DEVICE_FAIL;
	}

#else
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
#endif
 
    nRet = FSR_BML_GetVolSpec(nVol,
                              &stVolSpec,
                              FSR_BML_FLAG_NONE);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    return stVolSpec.nPgsPerSLCUnit;
}
 
INT32
samsung_page_size(void)
{
    INT32           nRet;
    FSRVolSpec  stVolSpec;
    UINT32          nVol = 0;
 
#if defined (BUILD_JNAND)
	nRet = FSR_BML_Init(FSR_BML_FLAG_NONE);
	if ((nRet != FSR_BML_SUCCESS)&&(nRet != FSR_BML_ALREADY_INITIALIZED))
	{
		return FS_DEVICE_FAIL;
	}

#else
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
#endif
 
    nRet = FSR_BML_GetVolSpec(nVol,
                              &stVolSpec,
                              FSR_BML_FLAG_NONE);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    return stVolSpec.nSctsPerPg;
}
 
INT32
samsung_lld_read(UINT32 nPbn, UINT32 nPageOffset, UINT32 nSize, UINT8* pMBuf)
{
#if !defined (FSR_NBL2)
    INT32           nRet;
    INT32           nErr = FS_DEVICE_OK;
    UINT32          nVol = 0;
    UINT32          nDev = 0;
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    FSRDevSpec      stDevSpec;
    UINT32          nSizeOfPage;
    UINT32          nNumOfPages;
    UINT32          nCnt;
 
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Acquire SM */
    FSR_BML_AcquireSM(nVol);
 
    do
    {
        nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
        if (nRet != FSR_BML_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        /* Register LLD Function Table */
        FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
        pstLFT[0] = &stLFT[0];
        pstLFT[1] = &stLFT[1];
 
        nRet = FSR_PAM_RegLFT(pstLFT);
        if (nRet != FSR_PAM_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        nRet = pstLFT[nVol]->LLD_GetDevSpec(nDev, &stDevSpec, FSR_LLD_FLAG_NONE);
        if (nRet != FSR_LLD_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        nSizeOfPage = stDevSpec.nSctsPerPG * FSR_SECTOR_SIZE;
        nNumOfPages = (nSize % nSizeOfPage) ? (nSize / nSizeOfPage + 1) : (nSize / nSizeOfPage);
 
        for (nCnt = 0; nCnt < nNumOfPages; nCnt++)
        {
            FSR_OAM_MEMSET(aMBuf, 0x00, sizeof(aMBuf));
 
            nRet = pstLFT[nVol]->LLD_Read(nDev,
                                          nPbn,
                                          nPageOffset + nCnt,
                                          aMBuf,
                                          NULL,
                                          FSR_LLD_FLAG_NONE);
            if (nRet != FSR_LLD_SUCCESS)
            {
                nErr = FS_DEVICE_FAIL;
                break;
            }
 
            FSR_OAM_MEMCPY(pMBuf, aMBuf, nSizeOfPage);
 
            pMBuf += nSizeOfPage;
        }
 
    } while (0);
 
    /* Release SM */
    FSR_BML_ReleaseSM(nVol);
 
    return nErr;
#else
    return FS_DEVICE_OK;
#endif
}

INT32
samsung_lld_chkbadblk(UINT32 nPbn)
{
#if !defined (FSR_NBL2)
    INT32           nRet;
    INT32           nErr = FS_DEVICE_OK;
    UINT32          nVol = 0;
    UINT32          nDev = 0;
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    FSRDevSpec      stDevSpec;
    UINT32          nSizeOfPage;
    UINT32          nNumOfPages;
    UINT32          nCnt;
 
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Acquire SM */
    FSR_BML_AcquireSM(nVol);
 
    do
    {
        nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
        if (nRet != FSR_BML_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        /* Register LLD Function Table */
        FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
        pstLFT[0] = &stLFT[0];
        pstLFT[1] = &stLFT[1];
 
        nRet = FSR_PAM_RegLFT(pstLFT);
        if (nRet != FSR_PAM_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        nErr = pstLFT[nVol]->LLD_ChkBadBlk(nDev, nPbn, FSR_LLD_FLAG_NONE);
    } while (0);
 
    /* Release SM */
    FSR_BML_ReleaseSM(nVol);
 
    return nErr;
#else
    return FS_DEVICE_OK;
#endif
}
 
INT32
samsung_bml_get_vir_unit_info(UINT32        nVun,
                              UINT32       *pn1stVpn,
                              UINT32       *pnPgsPerUnit)
{
    UINT32          nVol = 0;
    INT32           nRet;
 
    nRet = FSR_BML_GetVirUnitInfo(nVol, nVun, pn1stVpn, pnPgsPerUnit);
    if (nRet == FSR_BML_SUCCESS)
    {
      return FS_DEVICE_OK;
    }
    else
    {
      return FS_DEVICE_FAIL;
    }
}
 
VOID
samsung_otp_read(UINT32 page, UINT8 *page_buf)
{
#if !defined(FSR_NBL2)
    FSRSpareBuf     SBuf;
    UINT32          nSctsPerPg;
 
    nSctsPerPg = samsung_page_size();
 
    SBuf.pstSpareBufBase = (FSRSpareBufBase *)(page_buf + nSctsPerPg * FSR_SECTOR_SIZE);
    SBuf.pstSTLMetaExt = NULL;
    SBuf.nNumOfMetaExt = 0;
    
    FSR_BML_OTPRead(0, page, 1, page_buf, &SBuf, FSR_BML_FLAG_ECC_OFF|FSR_BML_FLAG_USE_SPAREBUF);
#endif
}

 

VOID
samsung_otp_write(UINT32 page, UINT8 *page_buf)
{
#if !defined(FSR_NBL2)
    FSRSpareBuf     SBuf;
    UINT32          nSctsPerPg;
 
    nSctsPerPg = samsung_page_size();
 
    SBuf.pstSpareBufBase = (FSRSpareBufBase *)(page_buf + nSctsPerPg * FSR_SECTOR_SIZE);
    SBuf.pstSTLMetaExt = NULL;
    SBuf.nNumOfMetaExt = 0;
 
    FSR_BML_OTPWrite(0, page, 1, page_buf, &SBuf, FSR_BML_FLAG_ECC_OFF|FSR_BML_FLAG_USE_SPAREBUF);
#endif
}

 
#if defined(FSR_USE_DUAL_CORE)
VOID
samsung_smem_flash_spin_lock(UINT32 nIdx, UINT32 nBaseAddr)
{
    smem_flash_spin_lock(nIdx, nBaseAddr);
}
#endif
 
#if defined(FSR_USE_DUAL_CORE)
VOID
samsung_smem_flash_spin_unlock(UINT32 nIdx, UINT32 nBaseAddr)
{
    smem_flash_spin_unlock(nIdx, nBaseAddr);
}
#endif
 
INT32
samsung_bml_unlock_wholearea(void)
{
#if !defined (FSR_NBL2)
    INT32   nRet = FS_DEVICE_OK;
    UINT32  nVol = 0;
    UINT32  nBytesReturned = 0;
    
    nRet = FSR_BML_IOCtl(nVol,
                         FSR_BML_IOCTL_UNLOCK_WHOLEAREA,
                         NULL,
                         0,
                         NULL,
                         0,
                         &nBytesReturned);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_lld_read_ext(UINT32 nDev, UINT32 nPbn, UINT32 nPageOffset, UINT32 nNumOfPgs, UINT8* pBuf)
{
#if !defined (FSR_NBL2)
    INT32           nRet;
    INT32           nErr    = FS_DEVICE_OK;
    UINT32          nVol    = 0;
    UINT32          nPgIdx  = 0;
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    FSRDevSpec      stDevSpec;
    UINT8           *pMBuf;
    UINT8           *pSBuf;
 
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Get device spec. */
    FSR_OAM_MEMSET(&stDevSpec, 0x00, sizeof(stDevSpec));
    nRet = pstLFT[nVol]->LLD_GetDevSpec(nDev,
                                        &stDevSpec,
                                        FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Acquire SM */
    FSR_BML_AcquireSM(nVol);
 
    do
    {
        nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
        if (nRet != FSR_BML_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        /* Set temporary buffer */
        pMBuf = pBuf;
        pSBuf = pMBuf + (stDevSpec.nSctsPerPG * FSR_SECTOR_SIZE);
 
        for (nPgIdx = nPageOffset; nPgIdx < nPageOffset + nNumOfPgs; nPgIdx++)
        {
            nRet = pstLFT[nVol]->LLD_Read(nDev,
                                          nPbn,
                                          nPgIdx,
                                          pMBuf,
                                          (FSRSpareBuf*) pSBuf,
                                          FSR_LLD_FLAG_USE_SPAREBUF |
                                          FSR_LLD_FLAG_ECC_ON       |
                                          FSR_LLD_FLAG_DUMP_ON);
 
            if (nRet != FSR_LLD_SUCCESS)
            {
                nErr = FS_DEVICE_FAIL;
                break;
            }
 
            /* Set pMBuf & pSBuf */
            pMBuf   +=  stDevSpec.nSctsPerPG * (stDevSpec.nSparePerSct + FSR_SECTOR_SIZE);
            pSBuf   =   pMBuf + (stDevSpec.nSctsPerPG * FSR_SECTOR_SIZE);
        } /* End of "for (nPgIdx = nPageOffset;...")*/
 
    } while (0);
 
    /* Release SM */
    FSR_BML_ReleaseSM(nVol);
 
    return nErr;
#else
    return FS_DEVICE_OK;
#endif
}
 
INT32
samsung_lld_write_ext(UINT32 nDev, UINT32 nPbn, UINT32 nPageOffset, UINT32 nNumOfPgs, UINT8* pBuf)
{
#if !defined (FSR_NBL2)
    INT32           nRet;
    INT32           nErr = FS_DEVICE_OK;
    UINT32          nVol = 0;
    UINT32          nDieIdx;
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    FSRDevSpec      stDevSpec;
    UINT32          nPgIdx;
    UINT8          *pMBuf;
    UINT8          *pSBuf;
 
    nRet = samsung_ftl_init();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Get device spec. */
    FSR_OAM_MEMSET(&stDevSpec, 0x00, sizeof(stDevSpec));
    nRet = pstLFT[nVol]->LLD_GetDevSpec(nDev,
                                        &stDevSpec,
                                        FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Acquire SM */
    FSR_BML_AcquireSM(nVol);
 
    do
    {
        nRet = FSR_BML_FlushOp(nVol, FSR_BML_FLAG_NO_SEMAPHORE);
        if (nRet != FSR_BML_SUCCESS)
        {
            nErr = FS_DEVICE_FAIL;
            break;
        }
 
        /* Set pMBuf & pSBuf */
        pMBuf = pBuf;
        pSBuf = pMBuf + (stDevSpec.nSctsPerPG * FSR_SECTOR_SIZE);
 
        /* Set nDieIdx for LLD_FlushOp() */
        nDieIdx = nPbn / (stDevSpec.nNumOfBlks/stDevSpec.nNumOfDies);
 
        for (nPgIdx = nPageOffset; nPgIdx < nPageOffset + nNumOfPgs; nPgIdx++)
        {
            nRet = pstLFT[nVol]->LLD_Write(nDev,
                                           nPbn,
                                           nPgIdx,
                                           pMBuf,
                                           (FSRSpareBuf*) pSBuf,
                                           FSR_LLD_FLAG_USE_SPAREBUF    |
                                           FSR_LLD_FLAG_ECC_ON          |
                                           FSR_LLD_FLAG_DUMP_ON         |
                                           FSR_LLD_FLAG_1X_PROGRAM);
 
            nRet = pstLFT[nVol]->LLD_FlushOp(nDev,
                                             nDieIdx,
                                             FSR_LLD_FLAG_NONE);
            if (nRet != FSR_LLD_SUCCESS)
            {
                nErr = FS_DEVICE_FAIL;
                break;
            }
 
            /* Increase pMBuf & pSBuf */
            pMBuf += stDevSpec.nSctsPerPG * (stDevSpec.nSparePerSct + FSR_SECTOR_SIZE);
            pSBuf = pMBuf + (stDevSpec.nSctsPerPG * FSR_SECTOR_SIZE);
 
        } /* End of "for (nPgIdx = nPageOffset;...)" */
 
    } while (0);
 
    /* Release SM */
    FSR_BML_ReleaseSM(nVol);
 
    return nErr;
#else
    return FS_DEVICE_OK;
#endif
}
 
INT32
samsung_backup_rfdata(UINT32 nID, UINT8* pBuf, UINT32 *pnSizeOfBuf)
{
#if 0 //!defined (FSR_NBL2)
    INT32       nRet        = 0;
    UINT32      nVol        = 0;
    UINT32      nDevIdx     = 0;
    UINT32      nBytesReturned = 0;
    UINT32      n1stVpn;
    UINT32      nPgsPerUnit;
    UINT32      nDieIdx;
    UINT32      nStartSBN;
    UINT32      nLastSBN;
    UINT32      nBMFIdx;
    UINT32      nNumOfPgs;
    UINT32      nMaxBadBlocksInDie;
    UINT32      nBadBlks;
    UINT8      *pTmpBuf;
 
    FSRPairedBMInfo stPairedBMFInfo;
    FSRDevSpec      stDevSpec;
    FsrVolParm      stPAM[FSR_MAX_VOLS];
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    BmlBMF         *pstBMF;
    FSRPartEntry    stPartEntry;
 
    /* Initialize *pnSizeOfBuf */
    *pnSizeOfBuf = 0;
 
    /* Get volume paramaters*/
    nRet  = FSR_PAM_GetPAParm(stPAM);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* BML_Open 
       BML_LoadPIEntry can be used only if BML_Open is called */
    nRet = samsung_bml_open();
    if (nRet != FS_DEVICE_OK)
    {
        return nRet;
    }
 
    /* Get device spec. */
    FSR_OAM_MEMSET(&stDevSpec, 0x00, sizeof(stDevSpec));
    nRet = pstLFT[nVol]->LLD_GetDevSpec(nVol * stPAM[nVol].nDevsInVol,
                                        &stDevSpec,
                                        FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Load partition information of nID */
    nRet = FSR_BML_LoadPIEntry(nVol,
                               nID,
                               &n1stVpn,
                               &nPgsPerUnit,
                               &stPartEntry);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Set Buffer pointer */
    pTmpBuf = pBuf;
 
    /* Initialize gstDataInfo array */
    FSR_OAM_MEMSET(&gstDataInfo[nID], 0x00, sizeof(gstDataInfo[nID]));
 
    /* Initialize stPairedBMFInfo */
    FSR_OAM_MEMSET(&stPairedBMFInfo, 0x00, sizeof(stPairedBMFInfo));
 
    nMaxBadBlocksInDie = (UINT32) stDevSpec.nRsvBlksInDev/ stDevSpec.nNumOfDies;
 
    /* Memory allocation/initialization: pstBMF, pstRCB */
    for (nDieIdx = 0; nDieIdx < stDevSpec.nNumOfDies; nDieIdx++)
    {
        stPairedBMFInfo.pstBMF[nDieIdx] = FSR_OAM_Malloc(sizeof(struct BMF_) * nMaxBadBlocksInDie);
        stPairedBMFInfo.pstRCB[nDieIdx] = FSR_OAM_Malloc(sizeof(UINT32) * nMaxBadBlocksInDie);
 
        FSR_OAM_MEMSET(stPairedBMFInfo.pstBMF[nDieIdx], 0x00, sizeof(struct BMF_) * nMaxBadBlocksInDie);
        FSR_OAM_MEMSET(stPairedBMFInfo.pstRCB[nDieIdx], 0x00, sizeof(UINT32) * nMaxBadBlocksInDie);
    }
 
    for (nDevIdx = 0; nDevIdx < stPAM[nVol].nDevsInVol ;nDevIdx++)
    {
        /* Initialize nBadBlks */
        nBadBlks = 0;
 
        /* STEP1. Get bad block mapping data        */
        nRet = FSR_BML_IOCtl(nVol,
                             FSR_BML_IOCTL_GET_PAIRED_BMI,
                             (UINT8*)&nDevIdx,
                             sizeof(nDevIdx),
                             (UINT8*)&stPairedBMFInfo,
                             sizeof(stPairedBMFInfo),
                             &nBytesReturned);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
 
        /* STEP2. Search replaced block of RF-Backup partition */
        for (nDieIdx = 0; nDieIdx < stDevSpec.nNumOfDies; nDieIdx++)
        {
            /* Calculate nStartSBN & nLastSBN */
            nStartSBN = (nDieIdx * (stDevSpec.nNumOfBlks/stDevSpec.nNumOfDies)) + 
                        (stPartEntry.n1stVun * stDevSpec.nNumOfPlanes);
            nNumOfPgs = stDevSpec.nPgsPerBlkForSLC;
 
            if (stPartEntry.nAttr & FSR_BML_PI_ATTR_MLC)
            {
                nStartSBN += (stDevSpec.nRsvBlksInDev/stDevSpec.nNumOfDies);
                nNumOfPgs = stDevSpec.nPgsPerBlkForMLC;
            }
 
            nLastSBN = nStartSBN + stPartEntry.nNumOfUnits * stDevSpec.nNumOfPlanes;
 
            /* Scan BMI */
            pstBMF      = stPairedBMFInfo.pstBMF[nDieIdx];
            for (nBMFIdx = 0; nBMFIdx < stPairedBMFInfo.nNumOfBMFs[nDieIdx]; nBMFIdx++)
            {
                if ((nStartSBN <= pstBMF->nSbn) && (nLastSBN >= pstBMF->nSbn))
                {
                    /* Store replaced SBN & nNumOfBadBlks */
                    gstDataInfo[nID].anRFSbn[nDevIdx][nBadBlks] = pstBMF->nSbn;
                    nBadBlks++;
 
                    /* Load RF data */
                    nRet = samsung_lld_read_ext(nDevIdx + (nVol * stPAM[nVol].nDevsInVol),
                                                pstBMF->nRbn,
                                                0,
                                                nNumOfPgs,
                                                pTmpBuf);
                    if (nRet != FS_DEVICE_OK)
                    {
                        return FS_DEVICE_FAIL;
                    }
 
                    /* Increase pBuf */
                    pTmpBuf += nNumOfPgs * stDevSpec.nSctsPerPG * (stDevSpec.nSparePerSct + FSR_SECTOR_SIZE);
                    *pnSizeOfBuf += nNumOfPgs * stDevSpec.nSctsPerPG * (stDevSpec.nSparePerSct + FSR_SECTOR_SIZE);
                }
                pstBMF++;
            } /* End of "for (nBMFIdx = 0;...)" */
 
        } /* End of "for (nDieIdx = 0;...)" */
 
        /* Store # of bad blocks per device */
        gstDataInfo[nID].nNumOfBadBlocks[nDevIdx] = nBadBlks;
 
    } /* End of "for (nDevIdx = 0;...)" */
 
    /* BML_Close 
       BML_Open & BML_Close should be paired */
    nRet = samsung_bml_close();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_restore_rfdata(UINT32 nID, UINT8* pBuf)
{
#if 0 //!defined (FSR_NBL2)
    INT32       nRet        = 0;
    UINT32      nVol        = 0;
    UINT32      nDevIdx     = 0;
    UINT32      nBytesReturned = 0;
    UINT32      n1stVpn;
    UINT32      nPgsPerUnit;
    UINT32      nDieIdx;
    UINT32      nBMFIdx;
    UINT32      nNumOfPgs;
    UINT32      nBlkIdx;
    UINT32      nMaxBadBlocksInDie;
    UINT8      *pTmpBuf;
 
    FSRPairedBMInfo stPairedBMFInfo;
    FSRDevSpec      stDevSpec;
    FsrVolParm      stPAM[FSR_MAX_VOLS];
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    BmlBMF         *pstBMF;
    FSRPartEntry    stPartEntry;
 
    /* Get volume paramaters*/
    nRet  = FSR_PAM_GetPAParm(stPAM);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* BML_Open 
       BML_LoadPIEntry can be used only if BML_Open is called */
    nRet = samsung_bml_open();
    if (nRet != FS_DEVICE_OK)
    {
        return nRet;
    }
 
    /* Get device spec. */
    FSR_OAM_MEMSET(&stDevSpec, 0x00, sizeof(stDevSpec));
    nRet = pstLFT[nVol]->LLD_GetDevSpec(nVol * stPAM[nVol].nDevsInVol,
                                        &stDevSpec,
                                        FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Load partition information of nID */
    nRet = FSR_BML_LoadPIEntry(nVol,
                               nID,
                               &n1stVpn,
                               &nPgsPerUnit,
                               &stPartEntry);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    nNumOfPgs = stDevSpec.nPgsPerBlkForSLC;
    if (stPartEntry.nAttr & FSR_BML_PI_ATTR_MLC)
    {
        nNumOfPgs = stDevSpec.nPgsPerBlkForMLC;
    }
 
    /* Set Buffer pointer */
    pTmpBuf = pBuf;
 
    /* Initialize stPairedBMFInfo */
    FSR_OAM_MEMSET(&stPairedBMFInfo, 0x00, sizeof(stPairedBMFInfo));
 
    nMaxBadBlocksInDie = (UINT32) stDevSpec.nRsvBlksInDev/ stDevSpec.nNumOfDies;
 
    /* Memory allocation/initialization: pstBMF, pstRCB */
    for (nDieIdx = 0; nDieIdx < stDevSpec.nNumOfDies; nDieIdx++)
    {
        stPairedBMFInfo.pstBMF[nDieIdx] = FSR_OAM_Malloc(sizeof(struct BMF_) * nMaxBadBlocksInDie);
        stPairedBMFInfo.pstRCB[nDieIdx] = FSR_OAM_Malloc(sizeof(UINT32) * nMaxBadBlocksInDie);
 
        FSR_OAM_MEMSET(stPairedBMFInfo.pstBMF[nDieIdx], 0x00, sizeof(struct BMF_) * nMaxBadBlocksInDie);
        FSR_OAM_MEMSET(stPairedBMFInfo.pstRCB[nDieIdx], 0x00, sizeof(UINT32) * nMaxBadBlocksInDie);
    }
 
    for (nDevIdx = 0; nDevIdx < stPAM[nVol].nDevsInVol ;nDevIdx++)
    {
        /* STEP1. Check anNumOfBadBlks[nDevIdx] 
                  if RF backup data does not exist in reservoir, this routine is skipped */
        if (gstDataInfo[nID].nNumOfBadBlocks[nDevIdx] == 0)
        {
            continue;
        }
 
        /* STEP2. Get bad block mapping data        */
        nRet = FSR_BML_IOCtl(nVol,
                             FSR_BML_IOCTL_GET_PAIRED_BMI,
                             (UINT8*)&nDevIdx,
                             sizeof(nDevIdx),
                             (UINT8*)&stPairedBMFInfo,
                             sizeof(stPairedBMFInfo),
                             &nBytesReturned);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
 
        /* STEP2. Search replaced block of RF-Backup partition */
        for (nDieIdx = 0; nDieIdx < stDevSpec.nNumOfDies; nDieIdx++)
        {
            for (nBlkIdx = 0; nBlkIdx < gstDataInfo[nID].nNumOfBadBlocks[nDevIdx]; nBlkIdx++)
            {
                /* Scan BMI */
                pstBMF      = stPairedBMFInfo.pstBMF[nDieIdx];
                for (nBMFIdx = 0; nBMFIdx < stPairedBMFInfo.nNumOfBMFs[nDieIdx]; nBMFIdx++)
                {
                    if (gstDataInfo[nID].anRFSbn[nDevIdx][nBlkIdx] == pstBMF->nSbn)
                    {
                        /* Re-write RF data */
                        nRet = samsung_lld_write_ext(nDevIdx + (nVol * stPAM[nVol].nDevsInVol),
                                                     pstBMF->nRbn,
                                                     0,
                                                     nNumOfPgs,
                                                     pTmpBuf);
                        if (nRet != FS_DEVICE_OK)
                        {
                            return FS_DEVICE_FAIL;
                        }
 
                        /* Increase pBuf */
                        pTmpBuf += nNumOfPgs * stDevSpec.nSctsPerPG * (stDevSpec.nSparePerSct + FSR_SECTOR_SIZE);
                    } /* End of "if (gstDataInfo[nID]...)" */
 
                    pstBMF++;
                } /* End of "for (nBMFIdx = 0;...)" */
            } /* End of "for (nBlkIdx = 0;...)" */
 
        } /* End of "for (nDieIdx = 0;...)" */
 
    } /* End of "for (nDevIdx = 0;...)" */
 
    /* BML_Close 
       BML_Open & BML_Close should be paired */
    nRet = samsung_bml_close();
    if (nRet != FS_DEVICE_OK)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32 samsung_bml_parti(UINT32 nPartID, UINT32 *n1stVpn, UINT32 *nPgsPerUnit, FSRPartEntry *pstPartEntry)
{
    UINT32 nVol = 0;
    UINT32 nRet;
 
    if(pstPartEntry == NULL)
    {
        FSRPartEntry    stPartEntry;
        nRet = FSR_BML_LoadPIEntry(nVol, nPartID, n1stVpn, nPgsPerUnit, &stPartEntry);
    }
    else
    {
        nRet = FSR_BML_LoadPIEntry(nVol, nPartID, n1stVpn, nPgsPerUnit, pstPartEntry);
    }
 
    if(nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
    
    return FS_DEVICE_OK;
}
 
INT32 samsung_get_full_bmlparti(FSRPartI *pstPartI)
{
    UINT32 nVol = 0;
    UINT32 nRet;
 
    if(pstPartI == NULL)
    {
        return FS_DEVICE_FAIL;
    }
    else
    {
        nRet = FSR_BML_GetFullPartI(nVol, pstPartI);
    }
 
    if(nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
    
    return FS_DEVICE_OK;
}
 
 
INT32
samsung_get_badblockinfo(BadBlkInfo* pstBadBlkInfo)
{
#if !defined (FSR_NBL2)
    UINT32  nVol            = 0;
    UINT32  nDevIdx         = 0;
    UINT32  nDieIdx         = 0;
    UINT32  nBytesReturned  = 0;
    UINT32  nBMFIdx         = 0;
    INT32   nRet;
 
    FSRBMInfo       stBMLInfo;
    FSRDevSpec      stDevSpec;
    FsrVolParm      stPAM[FSR_MAX_VOLS];
    FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS];
    FSRLowFuncTbl   stLFT[FSR_MAX_VOLS];
    BmlBMF         *pstBMF;
 
    /* Get volume paramaters*/
    nRet  = FSR_PAM_GetPAParm(stPAM);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    /* Register LLD Function Table */
    FSR_OAM_MEMSET(&stLFT[0], 0x00, sizeof(FSRLowFuncTbl));
    pstLFT[0] = &stLFT[0];
    pstLFT[1] = &stLFT[1];
 
    nRet = FSR_PAM_RegLFT(pstLFT);
    if (nRet != FSR_PAM_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    nRet = samsung_bml_open();
    if (nRet != FS_DEVICE_OK)
    {
        return nRet;
    }
 
    /* Get device spec. */
    FSR_OAM_MEMSET(&stDevSpec, 0x00, sizeof(stDevSpec));
    nRet = pstLFT[nVol]->LLD_GetDevSpec(nVol * stPAM[nVol].nDevsInVol,
                                        &stDevSpec,
                                        FSR_LLD_FLAG_NONE);
    if (nRet != FSR_LLD_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
 
    for (nDieIdx = 0; nDieIdx < stDevSpec.nNumOfDies; nDieIdx++)
    {
        stBMLInfo.pstBMF[nDieIdx] = FSR_OAM_Malloc(sizeof(struct BMF_) * (stDevSpec.nRsvBlksInDev/ stDevSpec.nNumOfDies));
    }
 
    for (nDevIdx = 0; nDevIdx < stPAM[nVol].nDevsInVol ;nDevIdx++)
    {
        /* Get bad block mapping data        */
        nRet = FSR_BML_IOCtl(nVol,
                             FSR_BML_IOCTL_GET_BMI,
                             (UINT8*)&nDevIdx,
                             sizeof(nDevIdx),
                             (UINT8*)&stBMLInfo,
                             sizeof(stBMLInfo),
                             &nBytesReturned);
        if (nRet != FSR_BML_SUCCESS)
        {
            return FS_DEVICE_FAIL;
        }
 
        pstBMF = pstBadBlkInfo->pstBMF[nDevIdx];
 
        for (nDieIdx = 0; nDieIdx < stBMLInfo.nNumOfDies; nDieIdx++)
        {
            for (nBMFIdx = 0; nBMFIdx <stBMLInfo.nNumOfBMFs[nDieIdx]; nBMFIdx++)
            {
                FSR_OAM_MEMCPY((UINT8 *)pstBMF,
                               (UINT8 *)&stBMLInfo.pstBMF[nDieIdx][nBMFIdx],
                               sizeof(struct BMF_));
                pstBMF++;
            }
 
            pstBadBlkInfo->nNumOfBadBlk[nDevIdx] += stBMLInfo.nNumOfBMFs[nDieIdx];
        }
 
    } /* End of "for (nDevIdx = 0; ..)" */
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_get_dumpsize(UINT32   *pnDumpSize)
{
#if !defined(FSR_NBL2)
    UINT32      nVol = 0;
    INT32       nRet;
 
    nRet = FSR_BML_GetDumpSize(nVol, pnDumpSize);
    if (nRet != FSR_BML_SUCCESS)
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}
 
INT32
samsung_dump_img(UINT32     *pnSizeOfDumpImage)
{
#if !defined(FSR_NBL2)
    UINT32      nVol = 0;
    UINT32      nDumpType = FSR_DUMP_VOLUME;
    UINT32      nDumpOrder;
    INT32       nRet;
 
    if (IsFirstDump == TRUE32)
    {
        nDumpOrder  = FSR_BML_FLAG_DUMP_FIRST;
        IsFirstDump = FALSE32;
    }
    else
    {
        nDumpOrder  = FSR_BML_FLAG_DUMP_CONTINUE;
    }
 
    /****************************************************/
    /* STEP1. Initialize pnSizeOfDumpImage              */
    /****************************************************/
    *pnSizeOfDumpImage = 0;
    FSR_OAM_MEMSET((UINT8*)DUMP_IMG_ADDR, 0x00, DUMP_IMG_SIZE);
 
    /****************************************************/
    /* STEP2. Call FSR_BML_Dump()                       */
    /****************************************************/
    nRet = FSR_BML_Dump(nVol,
                        nDumpType,
                        nDumpOrder,
                        NULL,
                        (UINT8*)DUMP_IMG_ADDR,
                        pnSizeOfDumpImage);
    if ((nRet != FSR_BML_DUMP_COMPLETE) &&
        (nRet != FSR_BML_DUMP_INCOMPLETE))
    {
        return FS_DEVICE_FAIL;
    }
#endif
    return FS_DEVICE_OK;
}

