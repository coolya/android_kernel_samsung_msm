/**
 *   @mainpage   Flex Sector Remapper
 *
 *   @section Intro
 *       Flash Translation Layer for Flex-OneNAND and OneNAND
 *
 *    @section  Copyright
 *            COPYRIGHT. 2003-2009 SAMSUNG ELECTRONICS CO., LTD.
 *                            ALL RIGHTS RESERVED
 *
 *     Permission is hereby granted to licensees of Samsung Electronics
 *     Co., Ltd. products to use or abstract this computer program for the
 *     sole purpose of implementing a product based on Samsung
 *     Electronics Co., Ltd. products. No other rights to reproduce, use,
 *     or disseminate this computer program, whether in part or in whole,
 *     are granted.
 *
 *     Samsung Electronics Co., Ltd. makes no representation or warranties
 *     with respect to the performance of this computer program, and
 *     specifically disclaims any responsibility for any damages,
 *     special or consequential, connected with the use of this program.
 *
 *     @section Description
 *
 */

/**
 * @file        FSR_PAM_MSM7k.c
 * @brief       This file contain the Platform Adaptation Modules for MSM7k
 * @author      NamOh Hwang
 * @date        28-JAN-2008
 * @remark
 * REVISION HISTORY
 * @n   28-JAN-2008 [NamOh Hwang] : first writing
 *
 */

#include "FSR.h"
#include "FSR_PAM_MSM7k.h"
#include "targtsncjnlya.h"
/*****************************************************************************/
/* [PAM customization]                                                       */
/* The following 3 parameters can be customized                              */
/*                                                                           */
/* - FSR_EBI2ND_REG_BASE                                                     */
/* - FSR_ENABLE_FLEXOND_LFT                                                  */
/* - FSR_ENABLE_ONENAND_LFT                                                  */
/*                                                                           */
/*****************************************************************************/
/**< if FSR_ENABLE_FLEXOND_LFT is defined,
     Low level function table is linked with Flex-OneNAND LLD */
#undef    FSR_ENABLE_4K_ONENAND_LFT

/**< if FSR_ENABLE_ONENAND_LFT is defined,
     Low level function table is linked with OneNAND LLD */
#undef FSR_ENABLE_ONENAND_LFT

#define   FSR_ENABLE_FLEXOND_LFT 

#define     MSM7200_SHARED_MEMORY_BASE          (SCL_SHARED_RAM_BASE+SCL_SHARED_RAM_SIZE-MSM7200_SHARED_MEMORY_SIZE)
#define     MSM7200_SHARED_MEMORY_SIZE          (0x5000)

#if defined (APPSBL)
#define FSR_DMA_MEMCPY
#endif
/*****************************************************************************/
/* Local #defines                                                            */
/*****************************************************************************/

#if defined (FSR_ENABLE_ONENAND_LFT)
    #include "FSR_LLD_OneNAND.h"
#elif defined (FSR_ENABLE_4K_ONENAND_LFT)
    #include "FSR_LLD_4K_OneNAND.h"
#elif defined (FSR_ENABLE_FLEXOND_LFT)
    #include "FSR_LLD_FlexOND.h"
#else
    #error Either FSR_ENABLE_FLEXOND_LFT or FSR_ENABLE_ONENAND_LFT should be defined
#endif

#if defined (FSR_DMA_MEMCPY)
#include "Dmov_7500.h"
#endif

#define DBG_PRINT(x)                                FSR_DBG_PRINT(x)
#define RTL_PRINT(x)                                FSR_RTL_PRINT(x)

#if defined (FSR_WINCE)
#define     MSM7200_SHARED_MEMORY_INIT_CYCLE        (0x5)
#elif defined (TINY_FSR)
#define     MSM7200_SHARED_MEMORY_INIT_CYCLE        (0x5)
#else    /* Normal case */
#define     MSM7200_SHARED_MEMORY_INIT_CYCLE        (0x4)
#endif

#define SYNC_DATA_READ_CMD                          (0x6)
#define SYNC_DATA_WRITE_CMD                         (0x7)
#define SYNC_REG_READ_CMD                           (0x2)
#define SYNC_REG_WRITE_CMD                          (0x3)

#if defined (FSR_DMA_MEMCPY)
UINT32 *m_DMAPhyAddr;
UINT32 *m_DMACmdPhyAddr;
UINT32 *m_DMACmdPtrPhyAddr;
UINT32  Vaddr_check;

#define domain 3
#define channel 7

/* Register addresses */
#if defined (APPSBL)
#define ADMH00_BASE                 0xa9400000
#define HI0_CHn_CMD_PTR             *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x000+4*channel+0x100400*domain))
#define HI0_CHn_RSLT                *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x040+4*channel+0x100400*domain))
#define HI0_CHn_FLUSH_STATE0        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x080+4*channel+0x100400*domain))
#define HI0_CHn_FLUSH_STATE1        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x0C0+4*channel+0x100400*domain))
#define HI0_CHn_FLUSH_STATE2        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x100+4*channel+0x100400*domain))
#define HI0_CHn_FLUSH_STATE3        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x140+4*channel+0x100400*domain))
#define HI0_CHn_FLUSH_STATE4        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x180+4*channel+0x100400*domain))
#define HI0_CHn_FLUSH_STATE5        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x1C0+4*channel+0x100400*domain))
#define HI0_CHn_STATUS              *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x200+4*channel+0x100400*domain))
#define HI0_CHn_RSLT_CONF           *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x300+4*channel+0x100400*domain))
#define HI0_CHn_DBG_0               *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x280+4*channel))
#define HI0_CIn_CONF                *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x390+4*channel))
#else /* #if defined (APPSBL) */
#define ADMH00_BASE                 0xC2D00000
#define HI0_CHn_CMD_PTR             *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x000+4*channel-0xFFC00*domain))
#define HI0_CHn_RSLT                *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x040+4*channel-0xFFC00*domain))
#define HI0_CHn_FLUSH_STATE0        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x080+4*channel-0xFFC00*domain))
#define HI0_CHn_FLUSH_STATE1        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x0C0+4*channel-0xFFC00*domain))
#define HI0_CHn_FLUSH_STATE2        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x100+4*channel-0xFFC00*domain))
#define HI0_CHn_FLUSH_STATE3        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x140+4*channel-0xFFC00*domain))
#define HI0_CHn_FLUSH_STATE4        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x180+4*channel-0xFFC00*domain))
#define HI0_CHn_FLUSH_STATE5        *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x1C0+4*channel-0xFFC00*domain))
#define HI0_CHn_STATUS              *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x200+4*channel-0xFFC00*domain))
#define HI0_CHn_RSLT_CONF           *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x300+4*channel-0xFFC00*domain))
#define HI0_CHn_DBG_0               *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x280+4*channel))
#define HI0_CIn_CONF                *((volatile UINT32*) ((UINT32)ADMH00_BASE+0x390+4*channel))
#endif /* #if defined (APPSBL)  */

struct DMOVCMDList
{
    UINT32 command;
    UINT32 src_addr;
    UINT32 dst_addr;
    UINT32 len;
};
#endif /* #if defined (FSR_DMA_MEMCPY) */

#define SFLASHC_CMD_ADDR_SET(nCmd, nNum)                                      \
{                                                                             \
    if (nCmd == SYNC_REG_READ_CMD || nCmd == SYNC_DATA_READ_CMD)              \
        gpEBI2NDReg->nSFLASHC_CMD_ADDR = (nCmd) | (nNum) | ASYNC_READ_MASK;   \
    else if (nCmd == SYNC_REG_WRITE_CMD || nCmd == SYNC_DATA_WRITE_CMD)       \
        gpEBI2NDReg->nSFLASHC_CMD_ADDR = (nCmd) | (nNum) | ASYNC_WRITE_MASK;  \
    else                                                                      \
        gpEBI2NDReg->nSFLASHC_CMD_ADDR = (nCmd) | (nNum);                     \
}

#define CMD_EXEC                                                              \
{                                                                             \
    while((gpEBI2NDReg->nSFLASHC_EXEC_CMD_ADDR & 0x1) != 0);                  \
    gpEBI2NDReg->nSFLASHC_EXEC_CMD_ADDR = 0x1;                                \
    while((gpEBI2NDReg->nSFLASHC_EXEC_CMD_ADDR & 0x1) != 0);                  \
}

/*****************************************************************************/
/* Local typedefs                                                            */
/*****************************************************************************/

/*****************************************************************************/
/* Static variables definitions                                              */
/*****************************************************************************/

PRIVATE FsrVolParm              gstFsrVolParm[FSR_MAX_VOLS];
PRIVATE BOOL32                  gbPAMInit                   = FALSE32;

/* MSM7200 does not use OneNAND controller. address of OneNANDReg should be 0 */
#if   defined (FSR_ENABLE_ONENAND_LFT)
//PRIVATE volatile OneNANDReg     *gpOneNANDReg = (volatile OneNANDReg     *) 0;
#elif defined (FSR_ENABLE_4K_ONENAND_LFT)
//PRIVATE volatile OneNAND4kReg   *gpOneNANDReg = (volatile OneNAND4kReg   *) 0;
#elif defined (FSR_ENABLE_FLEXOND_LFT)
//PRIVATE volatile FlexOneNANDReg *gpOneNANDReg = (volatile FlexOneNANDReg *) 0;
#else
#error  Either FSR_ENABLE_FLEXOND_LFT or FSR_ENABLE_ONENAND_LFT should be defined
#endif

PRIVATE volatile EBI2NDReg *gpEBI2NDReg = (volatile EBI2NDReg  *) 0;

/******************************************************************************/
/* Local function prototypes                                                  */
/******************************************************************************/
#if defined (FSR_DMA_MEMCPY)
PRIVATE VOID DMOV_Memcpy(VOID *pDst, VOID *pSrc, UINT32 nLen);
#endif

PRIVATE VOID FSR_PAM_Memcpy(VOID *pDst, VOID *pSrc, UINT32 nLen);

/*****************************************************************************/
/* Function Implementation                                                   */
/*****************************************************************************/

/**
 * @brief           This function initialize NAND Controller
 *
 * @param[in]       none
 *
 * @return          none
 *
 * @author          NamOh Hwang
 *
 * @version         1.0.0
 *
 */
PUBLIC VOID
FSR_PAM_InitNANDController(VOID)
{
    PRIVATE BOOL32    bInitFlag = FALSE32;   /* Flag to inform calling FSR_PAM_InitNANDController */

    if (!bInitFlag)
    {
#if defined(FSR_LINUX_OAM)
	    /* Translate physical address to virtual address for Linux OS */
	    gpEBI2NDReg = (volatile EBI2NDReg *) FSR_OAM_Pa2Va(FSR_EBI2ND_REG_BASE);
#else
	    gpEBI2NDReg = (volatile EBI2NDReg *) FSR_EBI2ND_REG_BASE;
#endif

	    /* initialize code for MSM NAND contoroller */
	    gpEBI2NDReg->nNAND_FLASH_CHIP_SELECT_ADDR = NAND_FLASH_CHIP_SELECT_ONE_NAND_ENABLE;

	    gpEBI2NDReg->nFLASH_DEV_CMD0_ADDR = 0;
	    gpEBI2NDReg->nFLASH_DEV_CMD1_ADDR = 0;
	    gpEBI2NDReg->nFLASH_DEV_CMD2_ADDR = 0;

// namohwang
      gpEBI2NDReg->nFLASH_XFR_STEP1_ADDR = 0x47804780;
      gpEBI2NDReg->nFLASH_XFR_STEP2_ADDR = 0x03A003A0;
      gpEBI2NDReg->nFLASH_XFR_STEP3_ADDR = 0x11B911B9;
      gpEBI2NDReg->nFLASH_XFR_STEP4_ADDR = 0x85A085A0;
      gpEBI2NDReg->nFLASH_XFR_STEP5_ADDR = 0xC020C020;
      gpEBI2NDReg->nFLASH_XFR_STEP6_ADDR = 0xC020C020;
      gpEBI2NDReg->nFLASH_XFR_STEP7_ADDR = 0xC020C020;

      gpEBI2NDReg->nSFLASHC_BURST_CFG_ADDR = 0x21100327;
//      gpEBI2NDReg->nSFLASHC_BURST_CFG_ADDR = 0x21100327;

	    gpEBI2NDReg->nNAND_ADDR0_ADDR = 0xF221; /* ONLD_REG_SYS_CONF1 */
	    gpEBI2NDReg->nNAND_GENP_REG0_ADDR = FlexOneNAND_SYSCONFIG_VAL;
#if defined (FSR_INIT_SET)
	    gpEBI2NDReg->nSFLASHC_CMD_ADDR = (SYNC_REG_WRITE_CMD | (1 << NAND_EXEC_CMD_NUM_XFRS_SHFT) | (NAND_EXEC_CMD_ASYNC_MODE << NAND_EXEC_CMD_ASYNC_MODE_SHFT) | ( 1 << 5));
#else
	    gpEBI2NDReg->nSFLASHC_CMD_ADDR = (SYNC_REG_WRITE_CMD | (1 << NAND_EXEC_CMD_NUM_XFRS_SHFT) |ASYNC_WRITE_MASK);
#endif
	    while((gpEBI2NDReg->nSFLASHC_EXEC_CMD_ADDR & 0x1) != 0)
	    {
	        /* empty statement */;
	    }
	    gpEBI2NDReg->nSFLASHC_EXEC_CMD_ADDR = 0x1;
	    while((gpEBI2NDReg->nSFLASHC_EXEC_CMD_ADDR & 0x1) != 0)
	    {
	        /* empty statement */;
	    }
	    while((gpEBI2NDReg->nSFLASHC_STATUS_ADDR & NAND_FLASH_STATUS_OPER_STATUS) != 0)
	    {
	        /* empty statement */;
	    }

	    bInitFlag = TRUE32;
    }
}

/**
 * @brief           This function initializes PAM
 *                  this function is called by FSR_BML_Init
 *
 * @return          FSR_PAM_SUCCESS
 *
 * @author          NamOh Hwang
 * @version         1.0.0
 *
 */
PUBLIC INT32
FSR_PAM_Init(VOID)
{
    INT32   nRe = FSR_PAM_SUCCESS;


    FSR_STACK_VAR;

    FSR_STACK_END;

    if (gbPAMInit == TRUE32)
    {
        return FSR_PAM_SUCCESS;
    }
    gbPAMInit     = TRUE32;


    RTL_PRINT((TEXT("[PAM:   ] ++%s\r\n"), __FSR_FUNC__));

    gstFsrVolParm[0].nBaseAddr[0]               = 0;
    gstFsrVolParm[0].nBaseAddr[1]               = FSR_PAM_NOT_MAPPED;
    gstFsrVolParm[0].nIntID[0]                  = FSR_INT_ID_NAND_0;
    gstFsrVolParm[0].nIntID[1]                  = FSR_INT_ID_NONE;
    gstFsrVolParm[0].nDevsInVol                 = 1;
    gstFsrVolParm[0].bProcessorSynchronization  = TRUE32;
#if defined (FSR_LINUX_OAM)
    gstFsrVolParm[0].nSharedMemoryBase          = FSR_OAM_Pa2Va(MSM7200_SHARED_MEMORY_BASE);
#else
    gstFsrVolParm[0].nSharedMemoryBase          = MSM7200_SHARED_MEMORY_BASE;
#endif
    gstFsrVolParm[0].nSharedMemorySize          = MSM7200_SHARED_MEMORY_SIZE;
    gstFsrVolParm[0].nSharedMemoryInitCycle     = MSM7200_SHARED_MEMORY_INIT_CYCLE;
    gstFsrVolParm[0].nMemoryChunkID             = 0;
#if defined (IMAGE_MODEM_PROC)
    gstFsrVolParm[0].nProcessorID               = FSR_PAM_PROCESSOR_ID0;
#elif defined (IMAGE_APPS_PROC)
    gstFsrVolParm[0].nProcessorID               = FSR_PAM_PROCESSOR_ID1;
#else
    gstFsrVolParm[0].nProcessorID               = 0;
#endif
    gstFsrVolParm[0].pExInfo                    = NULL;

    gstFsrVolParm[1].nBaseAddr[0]               = FSR_PAM_NOT_MAPPED;
    gstFsrVolParm[1].nBaseAddr[1]               = FSR_PAM_NOT_MAPPED;
    gstFsrVolParm[1].nIntID[0]                  = FSR_INT_ID_NONE;
    gstFsrVolParm[1].nIntID[1]                  = FSR_INT_ID_NONE;
    gstFsrVolParm[1].nDevsInVol                 = 0;
    gstFsrVolParm[1].bProcessorSynchronization  = TRUE32;
    gstFsrVolParm[1].nSharedMemoryBase          = FSR_PAM_NOT_MAPPED;
    gstFsrVolParm[1].nSharedMemorySize          = 0;
    gstFsrVolParm[1].nSharedMemoryInitCycle     = 0;
    gstFsrVolParm[1].nMemoryChunkID             = 1;
#if defined (IMAGE_MODEM_PROC)
    gstFsrVolParm[1].nProcessorID               = FSR_PAM_PROCESSOR_ID0;
#elif defined (IMAGE_APPS_PROC)
    gstFsrVolParm[1].nProcessorID               = FSR_PAM_PROCESSOR_ID1;
#else
    gstFsrVolParm[1].nProcessorID               = 0;
#endif
    gstFsrVolParm[1].pExInfo                    = NULL;

#if defined (FSR_DMA_MEMCPY)
#if defined (APPSBL)
    m_DMAPhyAddr        = (UINT32 *)0x01FFAE00;
    m_DMACmdPhyAddr     = (UINT32 *)0x01FFAC00;
    m_DMACmdPtrPhyAddr  = (UINT32 *)0x01FFAA00;
#else
    m_DMAPhyAddr 	    = (UINT32 *)pmem_malloc (512, PMEM_UNCACHED_ID );
    m_DMACmdPhyAddr     = (UINT32 *)pmem_malloc (512, PMEM_UNCACHED_ID );
    m_DMACmdPtrPhyAddr  = (UINT32 *)pmem_malloc (512, PMEM_UNCACHED_ID );
#endif
#endif

    RTL_PRINT((TEXT("[PAM:   ] --%s\r\n"), __FSR_FUNC__));

    return nRe;
}

/**
 * @brief           This function returns FSR volume parameter
 *                  this function is called by FSR_BML_Init
 *
 * @param[in]       stVolParm[FSR_MAX_VOLS] : FsrVolParm data structure array
 *
 * @return          FSR_PAM_SUCCESS
 * @return          FSR_PAM_NOT_INITIALIZED
 *
 * @author          NamOh Hwang
 * @version         1.0.0
 *
 */
PUBLIC INT32
FSR_PAM_GetPAParm(FsrVolParm stVolParm[FSR_MAX_VOLS])
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    if (gbPAMInit == FALSE32)
    {
        return FSR_PAM_NOT_INITIALIZED;
    }

    FSR_OAM_MEMCPY(&(stVolParm[0]), &gstFsrVolParm[0], sizeof(FsrVolParm));
    FSR_OAM_MEMCPY(&(stVolParm[1]), &gstFsrVolParm[1], sizeof(FsrVolParm));

    return FSR_PAM_SUCCESS;
}

/**
 * @brief           This function registers LLD function table
 *                  this function is called by FSR_BML_Open
 *
 * @param[in]      *pstLFT[FSR_MAX_VOLS] : pointer to FSRLowFuncTable data structure
 *
 * @return          none
 *
 * @author          NamOh Hwang
 * @version         1.0.0
 *
 */
PUBLIC INT32
FSR_PAM_RegLFT(FSRLowFuncTbl  *pstLFT[FSR_MAX_VOLS])
{
    UINT32  nVolIdx;
    FSR_STACK_VAR;

    FSR_STACK_END;

    if (gbPAMInit == FALSE32)
    {
        return FSR_PAM_NOT_INITIALIZED;
    }

    if (gstFsrVolParm[0].nDevsInVol > 0)
    {
        nVolIdx = 0;


#if defined (FSR_ENABLE_FLEXOND_LFT)
        pstLFT[nVolIdx]->LLD_Init               = FSR_FND_Init;
        pstLFT[nVolIdx]->LLD_Open               = FSR_FND_Open;
        pstLFT[nVolIdx]->LLD_Close              = FSR_FND_Close;
        pstLFT[nVolIdx]->LLD_Erase              = FSR_FND_Erase;
        pstLFT[nVolIdx]->LLD_ChkBadBlk          = FSR_FND_ChkBadBlk;
        pstLFT[nVolIdx]->LLD_FlushOp            = FSR_FND_FlushOp;
        pstLFT[nVolIdx]->LLD_GetDevSpec         = FSR_FND_GetDevSpec;
        pstLFT[nVolIdx]->LLD_Read               = FSR_FND_Read;
        pstLFT[nVolIdx]->LLD_ReadOptimal        = FSR_FND_ReadOptimal;
        pstLFT[nVolIdx]->LLD_Write              = FSR_FND_Write;
        pstLFT[nVolIdx]->LLD_CopyBack           = FSR_FND_CopyBack;
        pstLFT[nVolIdx]->LLD_GetPrevOpData      = FSR_FND_GetPrevOpData;
        pstLFT[nVolIdx]->LLD_IOCtl              = FSR_FND_IOCtl;
        pstLFT[nVolIdx]->LLD_InitLLDStat        = FSR_FND_InitLLDStat;
        pstLFT[nVolIdx]->LLD_GetStat            = FSR_FND_GetStat;
        pstLFT[nVolIdx]->LLD_GetBlockInfo       = FSR_FND_GetBlockInfo;
        pstLFT[nVolIdx]->LLD_GetNANDCtrllerInfo = FSR_FND_GetPlatformInfo;

#elif defined (FSR_ENABLE_ONENAND_LFT)
        pstLFT[nVolIdx]->LLD_Init               = FSR_OND_Init;
        pstLFT[nVolIdx]->LLD_Open               = FSR_OND_Open;
        pstLFT[nVolIdx]->LLD_Close              = FSR_OND_Close;
        pstLFT[nVolIdx]->LLD_Erase              = FSR_OND_Erase;
        pstLFT[nVolIdx]->LLD_ChkBadBlk          = FSR_OND_ChkBadBlk;
        pstLFT[nVolIdx]->LLD_FlushOp            = FSR_OND_FlushOp;
        pstLFT[nVolIdx]->LLD_GetDevSpec         = FSR_OND_GetDevSpec;
        pstLFT[nVolIdx]->LLD_Read               = FSR_OND_Read;
        pstLFT[nVolIdx]->LLD_ReadOptimal        = FSR_OND_ReadOptimal;
        pstLFT[nVolIdx]->LLD_Write              = FSR_OND_Write;
        pstLFT[nVolIdx]->LLD_CopyBack           = FSR_OND_CopyBack;
        pstLFT[nVolIdx]->LLD_GetPrevOpData      = FSR_OND_GetPrevOpData;
        pstLFT[nVolIdx]->LLD_IOCtl              = FSR_OND_IOCtl;
        pstLFT[nVolIdx]->LLD_InitLLDStat        = FSR_OND_InitLLDStat;
        pstLFT[nVolIdx]->LLD_GetStat            = FSR_OND_GetStat;
        pstLFT[nVolIdx]->LLD_GetBlockInfo       = FSR_OND_GetBlockInfo;
        pstLFT[nVolIdx]->LLD_GetNANDCtrllerInfo = FSR_OND_GetNANDCtrllerInfo;

#elif defined (FSR_ENABLE_4K_ONENAND_LFT)
        pstLFT[nVolIdx]->LLD_Init               = FSR_OND_4K_Init;
        pstLFT[nVolIdx]->LLD_Open               = FSR_OND_4K_Open;
        pstLFT[nVolIdx]->LLD_Close              = FSR_OND_4K_Close;
        pstLFT[nVolIdx]->LLD_Erase              = FSR_OND_4K_Erase;
        pstLFT[nVolIdx]->LLD_ChkBadBlk          = FSR_OND_4K_ChkBadBlk;
        pstLFT[nVolIdx]->LLD_FlushOp            = FSR_OND_4K_FlushOp;
        pstLFT[nVolIdx]->LLD_GetDevSpec         = FSR_OND_4K_GetDevSpec;
        pstLFT[nVolIdx]->LLD_Read               = FSR_OND_4K_Read;
        pstLFT[nVolIdx]->LLD_ReadOptimal        = FSR_OND_4K_ReadOptimal;
        pstLFT[nVolIdx]->LLD_Write              = FSR_OND_4K_Write;
        pstLFT[nVolIdx]->LLD_CopyBack           = FSR_OND_4K_CopyBack;
        pstLFT[nVolIdx]->LLD_GetPrevOpData      = FSR_OND_4K_GetPrevOpData;
        pstLFT[nVolIdx]->LLD_IOCtl              = FSR_OND_4K_IOCtl;
        pstLFT[nVolIdx]->LLD_InitLLDStat        = FSR_OND_4K_InitLLDStat;
        pstLFT[nVolIdx]->LLD_GetStat            = FSR_OND_4K_GetStat;
        pstLFT[nVolIdx]->LLD_GetBlockInfo       = FSR_OND_4K_GetBlockInfo;
        pstLFT[nVolIdx]->LLD_GetNANDCtrllerInfo = FSR_OND_4K_GetPlatformInfo;
#else
        RTL_PRINT((TEXT("[PAM:ERR] LowFuncTbl(FlexOneNAND) isn't linked : %s / line %d\r\n"), __FSR_FUNC__, __LINE__));
        return FSR_PAM_LFT_NOT_LINKED;
#endif

    }

    if (gstFsrVolParm[1].nDevsInVol > 0)
    {
        nVolIdx = 1;


#if defined (FSR_ENABLE_FLEXOND_LFT)
        pstLFT[nVolIdx]->LLD_Init               = FSR_FND_Init;
        pstLFT[nVolIdx]->LLD_Open               = FSR_FND_Open;
        pstLFT[nVolIdx]->LLD_Close              = FSR_FND_Close;
        pstLFT[nVolIdx]->LLD_Erase              = FSR_FND_Erase;
        pstLFT[nVolIdx]->LLD_ChkBadBlk          = FSR_FND_ChkBadBlk;
        pstLFT[nVolIdx]->LLD_FlushOp            = FSR_FND_FlushOp;
        pstLFT[nVolIdx]->LLD_GetDevSpec         = FSR_FND_GetDevSpec;
        pstLFT[nVolIdx]->LLD_Read               = FSR_FND_Read;
        pstLFT[nVolIdx]->LLD_ReadOptimal        = FSR_FND_ReadOptimal;
        pstLFT[nVolIdx]->LLD_Write              = FSR_FND_Write;
        pstLFT[nVolIdx]->LLD_CopyBack           = FSR_FND_CopyBack;
        pstLFT[nVolIdx]->LLD_GetPrevOpData      = FSR_FND_GetPrevOpData;
        pstLFT[nVolIdx]->LLD_IOCtl              = FSR_FND_IOCtl;
        pstLFT[nVolIdx]->LLD_InitLLDStat        = FSR_FND_InitLLDStat;
        pstLFT[nVolIdx]->LLD_GetStat            = FSR_FND_GetStat;
        pstLFT[nVolIdx]->LLD_GetBlockInfo       = FSR_FND_GetBlockInfo;
        pstLFT[nVolIdx]->LLD_GetNANDCtrllerInfo = FSR_FND_GetPlatformInfo;

#elif defined (FSR_ENABLE_ONENAND_LFT)
        pstLFT[nVolIdx]->LLD_Init               = FSR_OND_Init;
        pstLFT[nVolIdx]->LLD_Open               = FSR_OND_Open;
        pstLFT[nVolIdx]->LLD_Close              = FSR_OND_Close;
        pstLFT[nVolIdx]->LLD_Erase              = FSR_OND_Erase;
        pstLFT[nVolIdx]->LLD_ChkBadBlk          = FSR_OND_ChkBadBlk;
        pstLFT[nVolIdx]->LLD_FlushOp            = FSR_OND_FlushOp;
        pstLFT[nVolIdx]->LLD_GetDevSpec         = FSR_OND_GetDevSpec;
        pstLFT[nVolIdx]->LLD_Read               = FSR_OND_Read;
        pstLFT[nVolIdx]->LLD_ReadOptimal        = FSR_OND_ReadOptimal;
        pstLFT[nVolIdx]->LLD_Write              = FSR_OND_Write;
        pstLFT[nVolIdx]->LLD_CopyBack           = FSR_OND_CopyBack;
        pstLFT[nVolIdx]->LLD_GetPrevOpData      = FSR_OND_GetPrevOpData;
        pstLFT[nVolIdx]->LLD_IOCtl              = FSR_OND_IOCtl;
        pstLFT[nVolIdx]->LLD_InitLLDStat        = FSR_OND_InitLLDStat;
        pstLFT[nVolIdx]->LLD_GetStat            = FSR_OND_GetStat;
        pstLFT[nVolIdx]->LLD_GetBlockInfo       = FSR_OND_GetBlockInfo;
        pstLFT[nVolIdx]->LLD_GetNANDCtrllerInfo = FSR_OND_GetNANDCtrllerInfo;


#elif defined (FSR_ENABLE_4K_ONENAND_LFT)
        pstLFT[nVolIdx]->LLD_Init               = FSR_OND_4K_Init;
        pstLFT[nVolIdx]->LLD_Open               = FSR_OND_4K_Open;
        pstLFT[nVolIdx]->LLD_Close              = FSR_OND_4K_Close;
        pstLFT[nVolIdx]->LLD_Erase              = FSR_OND_4K_Erase;
        pstLFT[nVolIdx]->LLD_ChkBadBlk          = FSR_OND_4K_ChkBadBlk;
        pstLFT[nVolIdx]->LLD_FlushOp            = FSR_OND_4K_FlushOp;
        pstLFT[nVolIdx]->LLD_GetDevSpec         = FSR_OND_4K_GetDevSpec;
        pstLFT[nVolIdx]->LLD_Read               = FSR_OND_4K_Read;
        pstLFT[nVolIdx]->LLD_ReadOptimal        = FSR_OND_4K_ReadOptimal;
        pstLFT[nVolIdx]->LLD_Write              = FSR_OND_4K_Write;
        pstLFT[nVolIdx]->LLD_CopyBack           = FSR_OND_4K_CopyBack;
        pstLFT[nVolIdx]->LLD_GetPrevOpData      = FSR_OND_4K_GetPrevOpData;
        pstLFT[nVolIdx]->LLD_IOCtl              = FSR_OND_4K_IOCtl;
        pstLFT[nVolIdx]->LLD_InitLLDStat        = FSR_OND_4K_InitLLDStat;
        pstLFT[nVolIdx]->LLD_GetStat            = FSR_OND_4K_GetStat;
        pstLFT[nVolIdx]->LLD_GetBlockInfo       = FSR_OND_4K_GetBlockInfo;
        pstLFT[nVolIdx]->LLD_GetNANDCtrllerInfo = FSR_OND_4K_GetPlatformInfo;
#else
        RTL_PRINT((TEXT("[PAM:ERR] LowFuncTbl(FlexOneNAND) isn't linked : %s / line %d\r\n"), __FSR_FUNC__, __LINE__));
        return FSR_PAM_LFT_NOT_LINKED;
#endif
    }

    return FSR_PAM_SUCCESS;
}

/**
 * @brief           This function reads two bytes from Register of OneNAND
 *
 * @param[in]       nAddr : address of register
 *
 * @return          two byte value from nAddr
 *
 * @author          NamOh Hwang
 *
 * @version         1.0.0
 *
 */
PUBLIC UINT16
FSR_PAM_ReadOneNANDRegister(UINT32 nAddr)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    gpEBI2NDReg->nNAND_ADDR0_ADDR = (UINT32) nAddr >> 1;
    SFLASHC_CMD_ADDR_SET( SYNC_REG_READ_CMD,  (1 << NAND_EXEC_CMD_NUM_XFRS_SHFT));
    CMD_EXEC;
    while((gpEBI2NDReg->nSFLASHC_STATUS_ADDR & NAND_FLASH_STATUS_OPER_STATUS) != 0);
    return (UINT16) gpEBI2NDReg->nNAND_GENP_REG0_ADDR;
};

/**
 * @brief           This function writes two bytes to Register of OneNAND
 *
 * @param[in]       nAddr : address of register
 * @param[in]       nValue: two bytes data to write
 *
 * @return          none
 *
 * @author          NamOh Hwang
 *
 * @version         1.0.0
 *
 */
PUBLIC VOID
FSR_PAM_WriteToOneNANDRegister(UINT32 nAddr, UINT16 nValue)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    /* nAddr is a byte address, MSM NANDC takes word address, so half it */
    gpEBI2NDReg->nNAND_ADDR0_ADDR = (UINT32) nAddr >> 1;
    gpEBI2NDReg->nNAND_GENP_REG0_ADDR = nValue;
    SFLASHC_CMD_ADDR_SET(SYNC_REG_WRITE_CMD, (1 << NAND_EXEC_CMD_NUM_XFRS_SHFT));
    CMD_EXEC;
    while((gpEBI2NDReg->nSFLASHC_STATUS_ADDR & NAND_FLASH_STATUS_OPER_STATUS) != 0);
}

/**
 * @brief           This function reads two bytes from DataRAM of OneNAND
 *
 * @param[in]       nAddr : address of DataRAM
 *
 * @return          two byte value from nAddr
 *
 * @author          NamOh Hwang
 *
 * @version         1.0.0
 *
 */
PUBLIC UINT16
FSR_PAM_Read2BFromDataRAM(UINT32 nAddr)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    /* nAddr is a byte address to DataRAM */
    gpEBI2NDReg->nFLASH_MACRO1_REG_ADDR = (UINT32) nAddr >> 1;
    /* always load 4 bytes ? */
    SFLASHC_CMD_ADDR_SET(SYNC_DATA_READ_CMD, (2 << NAND_EXEC_CMD_NUM_XFRS_SHFT));
    CMD_EXEC;
    while ((gpEBI2NDReg->nSFLASHC_STATUS_ADDR & NAND_FLASH_STATUS_OPER_STATUS) != 0);

    return (UINT16) gpEBI2NDReg->nFLASH_BUFF_ACC_ADDR[0];
}

/**
 * @brief           This function writes two bytes to DataRAM of OneNAND
 *
 * @param[in]       nAddr : address of DataRAM
 * @param[in]       nValue: a value to write
 *
 * @return          none
 *
 * @author          NamOh Hwang
 *
 * @version         1.0.0
 *
 */
PUBLIC VOID
FSR_PAM_Write2BToDataRAM(UINT32 nAddr, UINT16 nValue)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    if ((nAddr & 0x3) != 0)
	{
		FSR_PAM_Read2BFromDataRAM(nAddr - 2);
		gpEBI2NDReg->nFLASH_BUFF_ACC_ADDR[0] = ((UINT32)nValue << 16) |( gpEBI2NDReg->nFLASH_BUFF_ACC_ADDR[0] & 0xFFFF);
		gpEBI2NDReg->nFLASH_MACRO1_REG_ADDR = (UINT32) (nAddr - 2) >> 1;
	}
	else
	{
		FSR_PAM_Read2BFromDataRAM(nAddr);
		gpEBI2NDReg->nFLASH_BUFF_ACC_ADDR[0] = (UINT32)nValue |( gpEBI2NDReg->nFLASH_BUFF_ACC_ADDR[0] & 0xFFFF0000);
		gpEBI2NDReg->nFLASH_MACRO1_REG_ADDR = (UINT32) nAddr >> 1;
	}
	SFLASHC_CMD_ADDR_SET(SYNC_DATA_WRITE_CMD, (2 << NAND_EXEC_CMD_NUM_XFRS_SHFT));
	CMD_EXEC;
	while((gpEBI2NDReg->nSFLASHC_STATUS_ADDR & NAND_FLASH_STATUS_OPER_STATUS) != 0);
}

/**
 * @brief           This function transfers data to NAND
 *
 * @param[in]      *pDst  : Destination array Pointer to be copied
 * @param[in]      *pSrc  : Source data allocated Pointer
 * @param[in]      *nSize : length to be transferred
 *
 * @return          none
 *
 * @author          NamOh Hwang
 * @version         1.0.0
 * @remark          pDst / pSrc address should be aligned by 4 bytes.
 */
PUBLIC VOID
FSR_PAM_TransToNAND(volatile VOID *pDst,
                    VOID          *pSrc,
                    UINT32        nSize)
{
    UINT32 nTrSize;

    FSR_STACK_VAR;

    FSR_STACK_END;

    FSR_ASSERT(((UINT32) pSrc & 0x03) == 0x00000000);
    FSR_ASSERT(((UINT32) pDst & 0x03) == 0x00000000);
    FSR_ASSERT(nSize > sizeof(UINT32));

    do
    {
        nTrSize = (nSize >= FSR_SECTOR_SIZE) ? FSR_SECTOR_SIZE : nSize;
        FSR_OAM_MEMCPY((void*) gpEBI2NDReg->nFLASH_BUFF_ACC_ADDR, pSrc, nTrSize);

        gpEBI2NDReg->nFLASH_MACRO1_REG_ADDR = ((UINT32) pDst) >> 1;

        SFLASHC_CMD_ADDR_SET(SYNC_DATA_WRITE_CMD, ((nTrSize >> 1) << NAND_EXEC_CMD_NUM_XFRS_SHFT));
        CMD_EXEC;
        while((gpEBI2NDReg->nSFLASHC_STATUS_ADDR & NAND_FLASH_STATUS_OPER_STATUS) != 0);

        nSize = nSize - nTrSize;

        pDst = (UINT8 *) pDst + nTrSize;
        pSrc = (UINT8 *) pSrc + nTrSize;

    } while (nSize > 0);
}

/**
 * @brief           This function transfers data from NAND
 *
 * @param[in]      *pDst  : Destination array Pointer to be copied
 * @param[in]      *pSrc  : Source data allocated Pointer
 * @param[in]      *nSize : length to be transferred
 *
 * @return          none
 *
 * @author          NamOh Hwang
 * @version         1.0.0
 * @remark          pDst / pSrc address should be aligned by 4 bytes
 */
PUBLIC VOID
FSR_PAM_TransFromNAND(VOID          *pDst,
                      volatile VOID *pSrc,
                      UINT32         nSize)
{
    UINT32 nTrSize;

    FSR_STACK_VAR;

    FSR_STACK_END;

    FSR_ASSERT(((UINT32) pSrc & 0x03) == 0x00000000);
    FSR_ASSERT(((UINT32) pDst & 0x03) == 0x00000000);
    FSR_ASSERT(nSize > sizeof(UINT32));

    do
    {
        nTrSize = (nSize >= FSR_SECTOR_SIZE) ? FSR_SECTOR_SIZE : nSize;

        gpEBI2NDReg->nFLASH_MACRO1_REG_ADDR = ((UINT32) pSrc) >> 1;

        SFLASHC_CMD_ADDR_SET(SYNC_DATA_READ_CMD, ((nTrSize >> 1) << NAND_EXEC_CMD_NUM_XFRS_SHFT));
        CMD_EXEC;
        while((gpEBI2NDReg->nSFLASHC_STATUS_ADDR & NAND_FLASH_STATUS_OPER_STATUS) != 0);

        FSR_PAM_Memcpy(pDst, (VOID *)gpEBI2NDReg->nFLASH_BUFF_ACC_ADDR, nTrSize);
        nSize = nSize - nTrSize;

        pSrc = (UINT8 *) pSrc + nTrSize;
        pDst = (UINT8 *) pDst + nTrSize;

    } while (nSize > 0);

}

/**
 * @brief           This function initializes the specified logical interrupt.
 *
 * @param[in]       nLogIntId : Logical interrupt id
 *
 * @return          FSR_PAM_SUCCESS
 *
 * @author          NamOh Hwang
 *
 * @version         1.0.0
 *
 * @remark          this function is used to support non-blocking I/O feature of FSR
 *
 */
PUBLIC INT32
FSR_PAM_InitInt(UINT32 nLogIntId)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    switch (nLogIntId)
    {
    case FSR_INT_ID_NAND_0:
        break;
    default:
        break;
    }

    return FSR_PAM_SUCCESS;
}

/**
 * @brief           This function deinitializes the specified logical interrupt.
 *
 * @param[in]       nLogIntId : Logical interrupt id
 *
 * @return          FSR_PAM_SUCCESS
 *
 * @author          NamOh Hwang
 *
 * @version         1.0.0
 *
 * @remark          this function is used to support non-blocking I/O feature of FSR
 *
 */
PUBLIC INT32
FSR_PAM_DeinitInt(UINT32 nLogIntId)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    switch (nLogIntId)
    {
    case FSR_INT_ID_NAND_0:
        break;
    default:
        break;
    }

    return FSR_PAM_SUCCESS;
}

/**
 * @brief           This function returns the physical interrupt ID from the logical interrupt ID
 *
 * @param[in]       nLogIntID : Logical interrupt id
 *
 * @return          physical interrupt ID
 *
 * @author          NamOh Hwang
 *
 * @version         1.0.0
 *
 */
PUBLIC UINT32
FSR_PAM_GetPhyIntID(UINT32  nLogIntID)
{
    UINT32 nPhyIntID = 0;
    FSR_STACK_VAR;

    FSR_STACK_END;

    switch (nLogIntID)
    {
    case FSR_INT_ID_NAND_0:
        break;
    default:
        break;
    }

    return nPhyIntID;
}

/**
 * @brief           This function creates spin lock for dual core.
 *
 * @param[out]     *pHandle : Handle of semaphore
 * @param[in]       nLayer  : 0 : FSR_OAM_SM_TYPE_BDD
 *                            0 : FSR_OAM_SM_TYPE_STL
 *                            1 : FSR_OAM_SM_TYPE_BML
 *                            2 : FSR_OAM_SM_TYPE_LLD
 *
 * @return          TRUE32   : this function creates spin lock successfully
 * @return          FALSE32  : fail
 *
 * @author          DongHoon Ham
 * @version         1.0.0
 * @remark          An initial count of spin lock object is 1.
 *                  An maximum count of spin lock object is 1.
 *
 */
PUBLIC BOOL32
FSR_PAM_CreateSL(UINT32* pHandle, UINT32 nLayer)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    return TRUE32;
}

/**
 * @brief          This function acquires spin lock for dual core.
 *
 * @param[in]       nHandle : Handle of semaphore to be acquired
 * @param[in]       nLayer  : 0 : FSR_OAM_SM_TYPE_BDD
 *                            0 : FSR_OAM_SM_TYPE_STL
 *                            1 : FSR_OAM_SM_TYPE_BML
 *                            2 : FSR_OAM_SM_TYPE_LLD
 *
 * @return          TRUE32   : this function acquires spin lock successfully
 * @return          FALSE32  : fail
 *
 * @author          DongHoon Ham
 * @version         1.0.0
 *
 */
PUBLIC BOOL32
FSR_PAM_AcquireSL(UINT32 pHandle, UINT32 nLayer)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    return FSR_OAM_AcquireSM(pHandle, nLayer);
}

/**
 * @brief           This function releases spin lock for dual core.
 *
 * @param[in]       nHandle : Handle of semaphore to be released
 * @param[in]       nLayer  : 0 : FSR_OAM_SM_TYPE_BDD
 *                            0 : FSR_OAM_SM_TYPE_STL
 *                            1 : FSR_OAM_SM_TYPE_BML
 *                            2 : FSR_OAM_SM_TYPE_LLD
 *
 * @return          TRUE32   : this function releases spin lock successfully
 * @return          FALSE32  : fail
 *
 * @author          DongHoon Ham
 * @version         1.0.0
 *
 */
PUBLIC BOOL32
FSR_PAM_ReleaseSL(UINT32 pHandle, UINT32 nLayer)
{
    FSR_STACK_VAR;

    FSR_STACK_END;

    return FSR_OAM_ReleaseSM(pHandle, nLayer);
}

#if defined (FSR_DMA_MEMCPY)
PRIVATE VOID
DMOV_Memcpy(VOID *pDst, VOID *pSrc, UINT32 nLen)
{

    struct DMOVCMDList *pstDMOVCMDList = (struct DMOVCMDList*) m_DMACmdPhyAddr
; // aligned?
    UINT32 rslt;

    FSR_STACK_VAR;

    FSR_STACK_END;

    if(Vaddr_check)
    {
        DMOV_INVAL_REGION((VOID *)pDst,nLen);
    }


    rslt = HI0_CHn_RSLT_CONF;
    HI0_CHn_RSLT_CONF = (rslt & 0xFFFFFFFE);

    *m_DMACmdPtrPhyAddr = ((UINT32) m_DMACmdPhyAddr)>>3 | 1UL <<31;
    pstDMOVCMDList->command = 0<<0 | 1<<31; // command + LC
    pstDMOVCMDList->src_addr = (UINT32) pSrc;
    pstDMOVCMDList->dst_addr = (UINT32) pDst;
    pstDMOVCMDList->len = nLen;

    HI0_CHn_CMD_PTR = ((UINT32)m_DMACmdPtrPhyAddr)>>3;
}
#endif /* #if defined (FSR_DMA_MEMCPY) */

PRIVATE VOID
FSR_PAM_Memcpy(VOID *pDst, VOID *pSrc, UINT32 nLen)
{
#if defined (FSR_DMA_MEMCPY)
    volatile UINT32 rslt;
    UINT32 timeout = 25000000;
    UINT32 status;
    UINT32 nAddrOffSet;

    FSR_STACK_VAR;

    FSR_STACK_END;

    if(((UINT32)pDst & 0xFFFFFFFF) >= 0x80000000)
    {
        Vaddr_check = 0;
        DMOV_Memcpy((VOID *)m_DMAPhyAddr, (unsigned int *)0xa0a00100, nLen);
    }
    else
    {
        Vaddr_check = 1;
        nAddrOffSet = 0x20-((UINT32)pDst & 0x1F);
        FSR_OAM_MEMCPY(pDst, pSrc, nAddrOffSet);
        DMOV_Memcpy((VOID *)((UINT32)pDst+nAddrOffSet),
            (unsigned int *)((UINT32)0xa0a00100+nAddrOffSet), (nLen-32));
    }

    do {
        rslt = HI0_CHn_RSLT;
        status = HI0_CHn_STATUS;
    } while ( timeout-- && ((status & 0x04000000) != 0) );

    if (Vaddr_check)
    {
        FSR_OAM_MEMCPY((VOID *)((UINT32)pDst+(nLen-32+nAddrOffSet)),
            (VOID *)((UINT32)pSrc+(nLen-32+nAddrOffSet)), (32-nAddrOffSet));
    }
    else
    {
        FSR_OAM_MEMCPY((VOID *)pDst, (VOID *)m_DMAPhyAddr, nLen);
    }
#else
    FSR_OAM_MEMCPY(pDst, pSrc, nLen);
#endif
}

