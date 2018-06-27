/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_mcu_aes.h
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      Header for the AES implementation.
*/
#ifndef __SF_MCU_AES_H__
#define __SF_MCU_AES_H__
#ifndef __DECL_SF_MCU_AES_H__
#define __DECL_SF_MCU_AES_H__ extern
#endif /* __DECL_SF_MCU_AES_H__ */

/******************************************************************************/
/*! @defgroup SF_MCU_AES STACKFORCE Example description
  @{  */

/******************************************************************************/
 /*! @defgroup SF_MCU_AES_API API
     @ingroup  SF_MCU_AES
   This section describes the API for the STACKFORCE Example implementation.
 */
/******************************************************************************/

/*!@} end of SF_MCU_AES */
/******************************************************************************/

/**
 * Structure of AES drivers.
 */
struct aes_128_driver {

  /**
   * \brief Sets the current key.
   */
  void (* set_key)( uint8_t *key);

  /**
   * \brief Encrypts.
   */
  void (* encrypt)(uint8_t *plaintext_and_result);
};


/*==============================================================================
                         FUNCTION PROTOTYPES OF THE API
==============================================================================*/
/******************************************************************************/
/*! \addtogroup SF_MCU_AES_API
 *  @{ */

/**
  @brief  Performs the initialization of the AES module.
          This function usually will be called by sf_mcu_init() and enables
          the AES module to initialize the hardware.
  @return On successful initialization the function should return @c true.
          Just in case the initialization wasn't possible for any reason, this
          function should return @c false.
*/
bool sf_mcu_aes_init(void);

/**
  @brief  Sets key for en-/decryption.
          Sets key to be used for any encryption or decryption procedure by the
          AES module. Basically the key will be maintained by the stack and
          given to the AES  odule for maintaing the underlying hardware.
          There is no need to call this function by user application.
  @param  pc_key    Pointer to the buffer which is holding the key.
  @return true, if key was set successfully, false otherwise
*/
bool sf_mcu_aes_setKey( uint8_t* pc_key);

/*!
  @brief  Encrypts a single block only.
          Encrypts a single block only, using the key that is currently stored
          within the  module. Block length for AES is fixed to 16 bytes.
  @param  pc_in     Pointer to the data to be encrypted.
  @param  pc_out    Pointer where to store the encrypted data.
  @param  length    Length of the message.
  @param  nonce     Pointer to 16 byte Nonce.
  @return Returns @c true if data has been encrypted successfully. In case of
          any problem (e.g. missing key) it'll return @c false.
*/
uint16_t sf_mcu_aes_cbc_encrypt( uint8_t* pc_in, uint8_t*  pc_out, uint16_t length, uint8_t* nonce);

/**
  @brief  Decrypts a single block only.
          Decrypts a single block only, using the key that is currently stored
          within the  module. Block length for AES is fixed to 16 bytes.
  @param  pc_in     Pointer containing data to be decrypted.
  @param  pc_out    Pointer where to store the decrypted data.
  @param  length    Length of the message.
  @param  nonce     Pointer to 16 byte Nonce.
  @return Returns @c true if data has been decrypted successfully. In case of
          any problem (e.g. missing key) it'll return @c false.
*/
uint16_t sf_mcu_aes_cbc_decrypt( uint8_t* pc_in, uint8_t* pc_out, uint16_t length, uint8_t* nonce);

/*!
  @brief  Encrypts a single block only.
          Encrypts a single block only, using the key that is currently stored
          within the  module. Block length for AES is fixed to 16 bytes.
  @param  pc_in     Pointer to the data to be encrypted.
  @param  pc_out    Pointer where to store the encrypted data.
  @return Returns @c true if data has been encrypted successfully. In case of
          any problem (e.g. missing key) it'll return @c false.
*/
bool sf_mcu_aes_ecb_encrypt( uint8_t* pc_in, uint8_t*  pc_out);

/**
  @brief  Decrypts a single block only.
          Decrypts a single block only, using the key that is currently stored
          within the  module. Block length for AES is fixed to 16 bytes.
  @param  pc_in     Pointer containing data to be decrypted.
  @param  pc_out    Pointer where to store the decrypted data.
  @return Returns @c true if data has been decrypted successfully. In case of
          any problem (e.g. missing key) it'll return @c false.
*/
bool sf_mcu_aes_ecb_decrypt( uint8_t* pc_in, uint8_t* pc_out);

 /*!@} end of SF_MCU_AES_API */
/******************************************************************************/

#endif /* __SF_MCU_AES_H__ */
