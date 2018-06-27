/**
* @code
*  ___ _____ _   ___ _  _____ ___  ___  ___ ___
* / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
* \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
* |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
* embedded.connectivity.solutions.==============
* @endcode
*
* @file       sf_mcu_aes.c
* @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
* @author     STACKFORCE
* @brief      AES de-/encryption.
*/
#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
                            INCLUDE FILES
==============================================================================*/
/* Standart libraries */
#include <stdint.h>
#include <stdbool.h>

#include "sf_mcu_aes.h"

#include "Board.h"
#include <ti/drivers/crypto/CryptoCC26XX.h>


/*==============================================================================
                          LOCAL FUNCTION PROTOTYPES
==============================================================================*/
static uint16_t loc_aes_cbc( uint8_t* pc_in, uint8_t* pc_out, uint16_t length, uint8_t* nonce, bool b_encrypt);
static bool loc_aes_ecb( uint8_t* pc_in, uint8_t* pc_out, bool b_encrypt);
static void setKey( uint8_t *key);
static void encrypt(uint8_t *pld);

/*==============================================================================
                            Global VARIABLES
==============================================================================*/
/** Module global variable for storing the encryption key. */
static uint8_t gac_key[KEY_BLENGTH];
CryptoCC26XX_Handle gs_cryptoHandle;

 struct aes_128_driver aes_cc1352_driver = {
    setKey,
    encrypt
};


/*==============================================================================
                          LOCAL FUNCTION
==============================================================================*/
/**
  @brief  Decrypts/Encrypts multiple blocks
          Decrypts/Encrypts a single block only, using the key that is currently
          stored within the aes module. Block length for AES is fixed to 16 bytes.
  @param  pc_in     Pointer containing data to be decrypted.
  @param  pc_out    Pointer where to store the decrypted data.
  @param  length    Length of the message.
  @param  nonce     Pointer to 16 byte Nonce.
  @param  b_encrypt Decidedes if the block should be decrypted or enrcypted
  @return Returns @c length of the ciphertext if data has been decrypted/encrypted successfully.
          In case of any problem (e.g. missing key) it'll return @c 0.
*/
static uint16_t loc_aes_cbc( uint8_t* pc_in, uint8_t* pc_out, uint16_t length, uint8_t* nonce, bool b_encrypt)
{
  uint16_t b_return = 0;
  int i_status;
  uint16_t cipher_len = 0;

  if ((length%KEY_BLENGTH) == 0)
    cipher_len = (length / KEY_BLENGTH) * KEY_BLENGTH;
  else
    cipher_len = ((length / KEY_BLENGTH)+1) * KEY_BLENGTH;

  uint8_t temp_buffer[cipher_len];
  CryptoCC26XX_AESCBC_Transaction trans;
  CryptoCC26XX_Operation c_cryptOperation;
  int i_keyIndex;

  if(b_encrypt == true)
  {
    c_cryptOperation = CRYPTOCC26XX_OP_AES_CBC_ENCRYPT;
  }/* if */
  else
  {
    c_cryptOperation = CRYPTOCC26XX_OP_AES_CBC_DECRYPT;
  }/* else */

  i_keyIndex = CryptoCC26XX_allocateKey(gs_cryptoHandle, CRYPTOCC26XX_KEY_0,
                                        ( uint32_t *) gac_key);
  if (i_keyIndex == CRYPTOCC26XX_STATUS_ERROR)
  {
    b_return = false;
  }/* if */
  else
  {
    /* Initialize transaction */
    CryptoCC26XX_Transac_init((CryptoCC26XX_Transaction *) &trans, c_cryptOperation);

    /* Setup transaction */
    trans.keyIndex        = i_keyIndex;
    trans.nonce           = nonce;
    trans.msgIn           = (uint8_t*) pc_in;
    trans.msgOut          = temp_buffer;
    trans.msgInLength     = length;


    /* Encrypt the plaintext with AES CCB */
    i_status = CryptoCC26XX_transact(gs_cryptoHandle, (CryptoCC26XX_Transaction *) &trans);

    if(i_status != CRYPTOCC26XX_STATUS_SUCCESS)
    {
      b_return = 0;
    }else
    {
        memcpy(pc_out ,temp_buffer, cipher_len);
        b_return = cipher_len;
    }

    CryptoCC26XX_releaseKey(gs_cryptoHandle, &i_keyIndex);
  }/* if */

  return b_return;
}/* loc_aes */

/**
  @brief  Decrypts/Encrypts a single block only.
          Decrypts/Encrypts a single block only, using the key that is currently
          stored within the aes module. Block length for AES is fixed to 16 bytes.
  @param  pc_in     Pointer containing data to be decrypted.
  @param  pc_out    Pointer where to store the decrypted data.
  @param  b_encrypt Decidedes if the block should be decrypted or enrcypted
  @return Returns @c true if data has been decrypted/encrypted successfully.
          In case of any problem (e.g. missing key) it'll return @c false.
*/
static bool loc_aes_ecb( uint8_t* pc_in, uint8_t* pc_out,  bool b_encrypt)
{
    bool b_return = true;
    int i_status;
    uint8_t pc_tmpData[AES_ECB_LENGTH];
    CryptoCC26XX_AESECB_Transaction trans;
    CryptoCC26XX_Operation c_cryptOperation;
    int i_keyIndex;

    if(b_encrypt == true)
    {
      c_cryptOperation = CRYPTOCC26XX_OP_AES_ECB_ENCRYPT;
    }/* if */
    else
    {
      c_cryptOperation = CRYPTOCC26XX_OP_AES_ECB_DECRYPT;
    }/* else */

    i_keyIndex = CryptoCC26XX_allocateKey(gs_cryptoHandle, CRYPTOCC26XX_KEY_0,
                                          ( uint32_t *) gac_key);
    if (i_keyIndex == CRYPTOCC26XX_STATUS_ERROR)
    {
      b_return = false;
    }/* if */
    else
    {
      /* Initialize transaction */
      CryptoCC26XX_Transac_init((CryptoCC26XX_Transaction *) &trans, c_cryptOperation);

      /* Setup transaction */
      trans.keyIndex         = i_keyIndex;
      trans.msgIn            = (uint32_t *) pc_in;
      trans.msgOut           = (uint32_t *) pc_tmpData;

      /* Encrypt the plaintext with AES ECB */
      i_status = CryptoCC26XX_transact(gs_cryptoHandle, (CryptoCC26XX_Transaction *) &trans);

      if(i_status != CRYPTOCC26XX_STATUS_SUCCESS)
      {
        b_return = false;
      }/* if */
      else
      {
        memcpy(pc_out, pc_tmpData, AES_ECB_LENGTH);
      }/* if...else */

      CryptoCC26XX_releaseKey(gs_cryptoHandle, &i_keyIndex);
    }/* if */

    return b_return;
}/* loc_aes_ecb */

static void encrypt(uint8_t *pld){
    loc_aes_ecb(pld, pld, true);
}/* ecb_encrypt */

static void setKey( uint8_t *key){
    sf_mcu_aes_setKey(key);
}/* setKey */

/*==============================================================================
                            FUNCTIONS
==============================================================================*/
/*============================================================================*/
/* sf_mcu_aes_init() */
/*============================================================================*/
bool sf_mcu_aes_init(void)
{
  bool b_return = true;
  /* Initialize Crypto driver */
  CryptoCC26XX_init();

  /* Attempt to open CryptoCC26XX. */
  gs_cryptoHandle = CryptoCC26XX_open(Board_CRYPTO0, false, NULL);
  if (!gs_cryptoHandle)
  {
    b_return = false;
  }/* if */

  return b_return;
} /* sf_mcu_aes_init */

/*============================================================================*/
/* sf_mcu_aes_setKey() */
/*============================================================================*/
bool sf_mcu_aes_setKey( uint8_t* pc_key)
{
  bool b_return = false;

  if(pc_key)
  {
    memcpy(gac_key, pc_key, KEY_BLENGTH);
    b_return = true;
  } /* if */

  return b_return;
} /* sf_mcu_aes_setKey */

/*============================================================================*/
/* sf_mcu_aes_cbc_encrypt() */
/*============================================================================*/
uint16_t sf_mcu_aes_cbc_encrypt( uint8_t* pc_in, uint8_t*  pc_out, uint16_t length, uint8_t* nonce)
{
    uint16_t b_return = 0;

  /* Call the function to encrypt a block */
  b_return = loc_aes_cbc(pc_in, pc_out, length, nonce, true);

  return b_return;
} /* sf_mcu_aes_cbc_encrypt */

/*============================================================================*/
/* sf_mcu_aes_cbc_decrypt() */
/*============================================================================*/
uint16_t sf_mcu_aes_cbc_decrypt( uint8_t* pc_in, uint8_t* pc_out, uint16_t length, uint8_t* nonce)
{
    uint16_t b_return = 0;

  /* Call the function to decrypt a block */
  b_return = loc_aes_cbc(pc_in, pc_out, length, nonce, false);

  return b_return;
} /* sf_mcu_aes_cbc_decrypt */

/*============================================================================*/
/* sf_mcu_aes_ecb_encrypt() */
/*============================================================================*/
bool sf_mcu_aes_ecb_encrypt( uint8_t* pc_in, uint8_t*  pc_out)
{
    bool b_return = false;

  /* Call the function to encrypt a block */
  b_return = loc_aes_ecb(pc_in, pc_out, true);

  return b_return;
} /* sf_mcu_aes_cbc_encrypt */

/*============================================================================*/
/* sf_mcu_aes_cbc_decrypt() */
/*============================================================================*/
bool sf_mcu_aes_ecb_decrypt( uint8_t* pc_in, uint8_t* pc_out)
{
    bool b_return = false;

  /* Call the function to decrypt a block */
  b_return = loc_aes_ecb(pc_in, pc_out, false);

  return b_return;
} /* sf_mcu_aes_ecb_decrypt */

#ifdef __cplusplus
}
#endif
