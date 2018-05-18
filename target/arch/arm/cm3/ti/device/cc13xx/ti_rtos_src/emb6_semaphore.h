#ifndef __EMB6_SEMAPHORE_H__
#define __EMB6_SEMAPHORE_H__
#ifndef __DECL_EMB6_SEMAPHORE_H__
#define __DECL_EMB6_SEMAPHORE_H__ extern
#else
#define __DECL_EMB6_SEMAPHORE_H__
#endif /* __EMB6_SEMAPHORE_H__ */

/**
  @file       emb6_semaphore.h
  @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
  @author     STACKFORCE
  @brief      Emb6 semaphore module.

              Include before:
              - emb6_typedefs.h
              - <ti/sysbios/knl/Semaphore.h>
*/


/*==============================================================================
                            DEFINES
==============================================================================*/

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/*!
 * @brief  Creates a emb6 Semaphore
 *
 * @param  ps_eb Pointer to the error handler
 */
/*============================================================================*/
void semaphore_init(Error_Block* ps_eb);

/*============================================================================*/
/*!
 * @brief  Wait until resource is available
 *
 */
/*============================================================================*/
void semaphore_pend(void);

/*============================================================================*/
/*!
 * @brief  Make the resource available
 *
 */
/*============================================================================*/
void semaphore_post(void);

#endif /* __EMB6_SEMAPHORE_H__ */
