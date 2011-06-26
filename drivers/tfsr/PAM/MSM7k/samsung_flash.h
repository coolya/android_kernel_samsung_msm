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
#ifndef SAMSUNG_FLASH_H
#define SAMSUNG_FLASH_H
 
/*===========================================================================
INCLUDE FILES
===========================================================================*/
#include "FSR.h"
#include "targtsncjnlya.h"
/*===========================================================================
PUBLIC DATA DECLARATIONS
===========================================================================*/
#define FS_REBUILD_BLOCK                (0)
#define FS_REBUILD_MAGIC1               (0xA5A5A5A5)
#define FS_REBUILD_MAGIC2               (0x5A5A5A5A)
#define FS_REBUILD_FLAG                 (0xA5A55A5A)
#define FS_REBUILD_FLAG_RESET           (0x5A5A5A5A)
//#define FS_REBUILD_FLAG_POS             (0x01FFFFFC)
#define FS_REBUILD_FLAG_POS             (SCL_SHARED_RAM_BASE+SCL_SHARED_RAM_SIZE-0x4)


#define SPINLOCK_BOOT                   (1)
//#define SPINLOCK_BASEADDR               (0x01FFFFF0)
#define SPINLOCK_BASEADDR               ((SCL_SHARED_RAM_BASE+SCL_SHARED_RAM_SIZE-0x10))

#define FS_DEVICE_OK              0
#define FS_DEVICE_FAIL            (-1)
#define FS_DEVICE_NOT_SUPPORTED   (-15)
 
#define FLASH_UNLOCK_OP           0x1
#define FLASH_LOCK_OP             0x2
#define FLASH_LOCKTIGHT_OP        0x3
 
#define SLC_RESEVED_DEFUALT_UNITS	7

typedef struct
{
    UINT32        nNumOfBadBlk[FSR_DEVS_PER_VOL];
    BmlBMF       *pstBMF[FSR_DEVS_PER_VOL];
}BadBlkInfo;
 
/*===========================================================================
PUBLIC FUNCTION DECLARATIONS
===========================================================================*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
 
INT32
samsung_ftl_init(void);
 
INT32
samsung_bml_format(FSRPartI* pstFSRPartI);
 
INT32
samsung_bml_get_SLC_blocks(UINT32* nBlocks);

INT32
samsung_stl_init(UINT32 nPartID);
 
INT32
samsung_stl_open(UINT32 nPartID);
 
INT32
samsung_stl_close(UINT32 nPartID);
 
INT32
samsung_stl_format(UINT32 nPartID);
 
INT32
samsung_stl_write(UINT32 nLsn, UINT32 nImgSize, UINT8* pBuf, UINT32 nPartID);
 
INT32
samsung_stl_read(UINT32 nLsn, UINT32 nImgSize, UINT8* pBuf, UINT32 nPartID);
 
INT32
samsung_stl_delete(UINT32 nLsn, UINT32 nNumOfScts, UINT32 nPartID);
 
INT32
samsung_stl_get_logscts(UINT32 nPartID);
 
INT32
samsung_bml_write(UINT32 nStartSct, UINT32 nNumOfScts, UINT8* pMBuf, FSRSpareBuf *pSBuf);
 
INT32
samsung_bml_write_ext(UINT32 nStartSct, UINT32 nNumOfScts, UINT8* pMBuf, UINT8* pSBuf);
 
INT32
samsung_bml_erase(UINT32 nVUN, UINT32 nNumOfUnits);
 
INT32
samsung_is_page_erased(UINT32 nPage);
 
INT32
samsung_bml_read(UINT32 nStartScts, UINT32 nNumOfScts, UINT8* pMBuf, FSRSpareBuf *pSBuf);
 
INT32
samsung_bml_read_ext(UINT32 nStartVpn, UINT32 nNumOfPages, UINT8* pMBuf, UINT8* pSBuf);
 
INT32
samsung_bml_lock_op (UINT32 nStartUnit, UINT32 nNumOfUnits, UINT32 nOpcode);
 
UINT32
samsung_retrieve_ID(void);
 
INT32
samsung_set_fs_rebuild(void);
 
INT32
samsung_reset_fs_rebuild(void);
 
INT32
samsung_init_fs(void);
 
INT32
samsung_lld_write(UINT32 nPbn, UINT32 nPageOffset, UINT32 nSize, UINT8* pMBuf);
 
INT32
samsung_lld_read(UINT32 nPbn, UINT32 nPageOffset, UINT32 nSize, UINT8* pMBuf);

INT32
samsung_lld_chkbadblk(UINT32 nPbn);
 
INT32
samsung_bml_get_vir_unit_info(UINT32 nVun, UINT32 *pn1stVpn, UINT32 *pnPgsPerUnit);
 
INT32
samsung_dump_bootimg(UINT32 nID, UINT8* pBuf, UINT32* pDumpSize);
 
VOID
samsung_smem_init(void);
 
INT32
samsung_block_count(void);
 
INT32
samsung_block_size(void);
 
INT32
samsung_page_size(void);
 
VOID
samsung_otp_read(UINT32 page, UINT8 *page_buf);
  
VOID
samsung_otp_write(UINT32 page, UINT8 *page_buf);
 
 
#if defined(FSR_USE_DUAL_CORE)
VOID
samsung_smem_flash_spin_lock(UINT32 nIdx, UINT32 nBaseAddr);
#endif
 
#if defined(FSR_USE_DUAL_CORE)
VOID
samsung_smem_flash_spin_unlock(UINT32 nIdx, UINT32 nBaseAddr);
#endif
 
INT32
samsung_bml_unlock_wholearea(void);
 
INT32
samsung_bml_open(void);
 
INT32
samsung_bml_close(void);
 
INT32
samsung_lld_read_ext(UINT32 nDev, UINT32 nPbn, UINT32 nPageOffset, UINT32 nNumOfPgs, UINT8* pBuf);
 
INT32
samsung_lld_write_ext(UINT32 nDev, UINT32 nPbn, UINT32 nPageOffset, UINT32 nNumOfPgs, UINT8* pBuf);
 
INT32
samsung_backup_rfdata(UINT32 nID, UINT8* pBuf, UINT32 *pnSizeOfBuf);
 
INT32
samsung_restore_rfdata(UINT32 nID, UINT8* pBuf);
 
INT32
samsung_bml_parti(UINT32 nPartID, UINT32 *n1stVpn, UINT32 *nPgsPerUnit, FSRPartEntry *pstPartEntry);
 
INT32
samsung_lld_get_devspec(FSRDevSpec *pstDevSpec);
 
INT32 
samsung_get_full_bmlparti(FSRPartI *pstPartI);
 
INT32
samsung_get_badblockinfo(BadBlkInfo* pstBadBlkInfo);
 
INT32
samsung_get_dumpsize(UINT32   *pnDumpSize);
 
INT32
samsung_dump_img(UINT32     *pnSizeOfDumpImage);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* SAMSUNG_FLASH_H */

