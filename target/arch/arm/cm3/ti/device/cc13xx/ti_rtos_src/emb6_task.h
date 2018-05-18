#ifndef __EMB6_TASK_H__
#define __EMB6_TASK_H__
#ifndef __DECL_EMB6_TASK_H__
#define __DECL_EMB6_TASK_H__ extern
#else
#define __DECL_EMB6_TASK_H__
#endif /* __EMB6_TASK_H__ */

/**
  @file       emb6_task.h
  @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
  @author     STACKFORCE
  @brief      Emb6 task module.

              Include before:
              - emb6_typedefs.h
              - <ti/sysbios/knl/Task.h>
*/


/*==============================================================================
                            DEFINES
==============================================================================*/

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*!
 * @brief  Creates a emb6 task for the specified function pointer
 *
 * @param fp_fxn  Function which sould be used as emb6 task
 * @param ps_eb   Pointer to the error handler
 */
/*============================================================================*/
void emb6_task_init(ti_sysbios_knl_Task_FuncPtr fp_fxn, Error_Block* ps_eb);

#endif /* __EMB6_TASK_H__ */
