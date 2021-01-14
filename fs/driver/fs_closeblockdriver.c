/****************************************************************************
 * fs/driver/fs_closeblockdriver.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in pathname and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of pathname code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "fs/fs.h"
#include "vfs_config.h"
#include "debug.h"
#include "errno.h"
#include "fs/vnode.h"
#include "disk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: close_blockdriver
 *
 * Description:
 *   Call the close method and release the vnode
 *
 * Input Parameters:
 *   vnode - reference to the vnode of a block driver opened by open_blockdriver
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   EINVAL  - vnode is NULL
 *   ENOTBLK - The vnode is not a block driver
 *
 ****************************************************************************/

int close_blockdriver(struct Vnode *vnode_ptr)
{
#ifdef VFS_IMPL_LATER
  int ret = 0; /* Assume success */
  los_part *part = NULL;
  los_disk *disk = NULL;

  /* Sanity checks */

  if (!vnode_ptr || !vnode_ptr->u.i_bops)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Verify that the vnode is a block driver. */

  if (!INODE_IS_BLOCK(vnode_ptr))
    {
      fdbg("vnode is not a block driver\n");
      ret = -ENOTBLK;
      goto errout;
    }

  part = los_part_find(vnode_ptr);
  if (part != NULL)
    {
      disk = get_disk(part->disk_id);
      if (disk == NULL)
        {
          ret = -EINVAL;
          goto errout_with_vnode;
        }

      if (pthread_mutex_lock(&disk->disk_mutex) != ENOERR)
        {
          PRINT_ERR("%s %d, mutex lock fail!\n", __FUNCTION__, __LINE__);
          vnode_release(vnode_ptr);
          return -1;
        }
      if (disk->disk_status == STAT_INUSED)
        {
          /* Close the block driver.  Not that no mutually exclusive access
          * to the driver is enforced here.  That must be done in the driver
          * if needed.
          */

          if (vnode_ptr->u.i_bops->close != NULL)
            {
              ret = vnode_ptr->u.i_bops->close(vnode_ptr);
            }
        }

      if (pthread_mutex_unlock(&disk->disk_mutex) != ENOERR)
        {
          PRINT_ERR("%s %d, mutex unlock fail!\n", __FUNCTION__, __LINE__);
          vnode_release(vnode_ptr);
          return -1;
        }

    }
  else
    {
      if ((vnode_ptr->i_flags & FSNODEFLAG_DELETED) == 0 && vnode_ptr->u.i_bops->close != NULL)
        {
          ret = vnode_ptr->u.i_bops->close(vnode_ptr);
        }
    }

errout_with_vnode:

  /* Then release the reference on the vnode */

  vnode_release(vnode_ptr);

errout:
  return ret;
#endif
  return 0;
}
