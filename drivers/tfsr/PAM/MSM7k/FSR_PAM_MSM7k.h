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
 * @file        FSR_PAM_MSM7k.h
 * @brief       This file contain the Platform Adaptation Modules for MSM7k
 * @author      NamOh Hwang
 * @date        28-JAN-2008
 * @remark
 * REVISION HISTORY
 * @n   18-NOV-2007 [Younwon Park]: first writing
 * @n   28-JAN-2008 [NamOh Hwang] : adaptation to FSR
 *
 */

#ifndef _FSR_PAM_MSM7K_H_
#define _FSR_PAM_MSM7K_H_

/**< MSM7200 NAND controller base address */
#if defined (FSR_NBL2)
#define     FSR_EBI2ND_REG_BASE                 (0xa0a00000)
#elif defined (FSR_NW)
#define     FSR_EBI2ND_REG_BASE                 (0xa0a00000)
#elif defined (FSR_LINUX_OAM)
#define     FSR_EBI2ND_REG_BASE                 (0xa0a00000)
#else
#define     FSR_EBI2ND_REG_BASE                 (0xc1300000)
#endif

/*****************************************************************************/
/* NAND Controller Register Command Definitions                              */
/*****************************************************************************/
#define SYNC_DATA_READ_CMD                          (0x6)
#define SYNC_DATA_WRITE_CMD                         (0x7)
#define SYNC_REG_READ_CMD                           (0x2)
#define SYNC_REG_WRITE_CMD                          (0x3)

/*****************************************************************************/
/* NAND Controller  Masking value Definitions                                */
/*****************************************************************************/
#define NAND_FLASH_CHIP_SELECT_ONE_NAND_ENABLE      (0x8)
#define NAND_FLASH_CHIP_SELECT_DM_ENABLE            (0x4)
#define NAND_FLASH_STATUS_READY_BUSY                (0x20)
#define NAND_FLASH_STATUS_OPER_STATUS               (0xF)
#define SFLASHC_STATUS_DEV_INTERRUPT_PIN            (0x20)
#define SFLASHC_STATUS_SFLASH_OPER_STATUS           (0xF)
#define NAND_DEVn_CFG0_NUM_ADDR_CYCLES_SHFT         (27)
#define NAND_DEVn_CFG0_NUM_ADDR_CYCLES_MASK         (7)
#define NAND_DEVn_CFG0_UD_SIZE_BYTES_SHFT           (9)
#define NAND_DEVn_CFG0_UD_SIZE_BYTES_MASK           (512)
#define NAND_EXEC_CMD_NUM_XFRS_SHFT                 (20)
#define NAND_EXEC_CMD_OFFSET_VAL_SHFT               (12)
#define NAND_EXEC_CMD_XFR_SHFT                      (5)
#define NAND_EXEC_CMD_ASYNC_MODE_SHFT               (4)
#define NAND_EXEC_CMD_ASYNC_MODE_MASK               (0x10)
#define NAND_EXEC_CMD_ASYNC_MODE                    (1)

/*****************************************************************************/
/* NAND Controller Register Address Definitions                              */
/*****************************************************************************/
typedef struct
{
    volatile UINT32  nNAND_FLASH_CMD_ADDR;             /* offset : 0x00000000)*/
    volatile UINT32  nNAND_ADDR0_ADDR;                 /* offset : 0x00000004)*/
    volatile UINT32  nNAND_ADDR1_ADDR;                 /* offset : 0x00000008)*/
    volatile UINT32  nNAND_FLASH_CHIP_SELECT_ADDR;     /* offset : 0x0000000c)*/
    volatile UINT32  nNANDC_EXEC_CMD_ADDR;             /* offset : 0x00000010)*/
    volatile UINT32  nNAND_FLASH_STATUS_ADDR;          /* offset : 0x00000014)*/
    volatile UINT32  nNANDC_BUFFER_STATUS_ADDR;        /* offset : 0x00000018)*/
    volatile UINT32  nSFLASHC_STATUS_ADDR;             /* offset : 0x0000001c)*/
    volatile UINT32  nNAND_DEV0_CFG0_ADDR;             /* offset : 0x00000020)*/
    volatile UINT32  nNAND_DEV0_CFG1_ADDR;             /* offset : 0x00000024)*/
    volatile UINT32  nRsv1[2];
    volatile UINT32  nNAND_DEV1_CFG0_ADDR;             /* offset : 0x00000030)*/
    volatile UINT32  nNAND_DEV1_CFG1_ADDR;             /* offset : 0x00000034)*/
    volatile UINT32  nSFLASHC_CMD_ADDR;                /* offset : 0x00000038)*/
    volatile UINT32  nSFLASHC_EXEC_CMD_ADDR;           /* offset : 0x0000003c)*/
    volatile UINT32  nNAND_FLASH_READ_ID_ADDR;         /* offset : 0x00000040)*/
    volatile UINT32  nNAND_FLASH_READ_STATUS_ADDR;     /* offset : 0x00000044)*/
    volatile UINT32  nRsv2[2];
    volatile UINT32  nNAND_FLASH_CONFIG_DATA_ADDR;     /* offset : 0x00000050)*/
    volatile UINT32  nNAND_FLASH_CONFIG_ADDR;          /* offset : 0x00000054)*/
    volatile UINT32  nNAND_FLASH_CONFIG_MODE_ADDR;     /* offset : 0x00000058)*/
    volatile UINT32  nRsv3;
    volatile UINT32  nNAND_FLASH_CONFIG_STATUS_ADDR;   /* offset : 0x00000060)*/
    volatile UINT32  nFLASH_MACRO1_REG_ADDR;           /* offset : 0x00000064)*/
    volatile UINT32  nRsv4[2];
    volatile UINT32  nFLASH_XFR_STEP1_ADDR;            /* offset : 0x00000070)*/
    volatile UINT32  nFLASH_XFR_STEP2_ADDR;            /* offset : 0x00000074)*/
    volatile UINT32  nFLASH_XFR_STEP3_ADDR;            /* offset : 0x00000078)*/
    volatile UINT32  nFLASH_XFR_STEP4_ADDR;            /* offset : 0x0000007c)*/
    volatile UINT32  nFLASH_XFR_STEP5_ADDR;            /* offset : 0x00000080)*/
    volatile UINT32  nFLASH_XFR_STEP6_ADDR;            /* offset : 0x00000084)*/
    volatile UINT32  nFLASH_XFR_STEP7_ADDR;            /* offset : 0x00000088)*/
    volatile UINT32  nRsv5;
    volatile UINT32  nNAND_GENP_REG0_ADDR;             /* offset : 0x00000090)*/
    volatile UINT32  nNAND_GENP_REG1_ADDR;             /* offset : 0x00000094)*/
    volatile UINT32  nNAND_GENP_REG2_ADDR;             /* offset : 0x00000098)*/
    volatile UINT32  nNAND_GENP_REG3_ADDR;             /* offset : 0x0000009c)*/
    volatile UINT32  nFLASH_DEV_CMD0_ADDR;             /* offset : 0x000000a0)*/
    volatile UINT32  nFLASH_DEV_CMD1_ADDR;             /* offset : 0x000000a4)*/
    volatile UINT32  nFLASH_DEV_CMD2_ADDR;             /* offset : 0x000000a8)*/
    volatile UINT32  nFLASH_DEV_CMD_VLD_ADDR;          /* offset : 0x000000ac)*/
    volatile UINT32  nEBI2_MISR_SIG_REG_ADDR;          /* offset : 0x000000b0)*/
    volatile UINT32  nRsv6[3];
    volatile UINT32  nNAND_ADDR2_ADDR;                 /* offset : 0x000000c0)*/
    volatile UINT32  nNAND_ADDR3_ADDR;                 /* offset : 0x000000c4)*/
    volatile UINT32  nNAND_ADDR4_ADDR;                 /* offset : 0x000000c8)*/
    volatile UINT32  nNAND_ADDR5_ADDR;                 /* offset : 0x000000cc)*/
    volatile UINT32  nFLASH_DEV_CMD3_ADDR;             /* offset : 0x000000d0)*/
    volatile UINT32  nFLASH_DEV_CMD4_ADDR;             /* offset : 0x000000d4)*/
    volatile UINT32  nFLASH_DEV_CMD5_ADDR;             /* offset : 0x000000d8)*/
    volatile UINT32  nFLASH_DEV_CMD6_ADDR;             /* offset : 0x000000dc)*/
    volatile UINT32  nSFLASHC_BURST_CFG_ADDR;          /* offset : 0x000000e0)*/
    volatile UINT32  nNAND_ADDR6_ADDR;                 /* offset : 0x000000e4)*/
    volatile UINT32  nRsv7[2];
    volatile UINT32  nEBI2_ECC_BUF_CFG_ADDR;           /* offset : 0x000000f0)*/
    volatile UINT32  nRsv8[3];
    volatile UINT32  nFLASH_BUFF_ACC_ADDR[0x80];       /* offset : 0x00000100)*/
} EBI2NDReg;
#endif /* #define _FSR_PAM_MSM7K_H_ */
