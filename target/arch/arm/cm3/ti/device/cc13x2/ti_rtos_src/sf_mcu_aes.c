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
#include <ti/drivers/AESECB.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>

/*==============================================================================
                          LOCAL FUNCTION PROTOTYPES
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
static bool loc_aes_cbc( uint8_t* pc_in, uint8_t* pc_out, uint16_t length, uint8_t* nonce, bool b_encrypt);

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
static bool loc_aes_ecb( uint8_t* pc_in, uint8_t* pc_out, bool b_encrypt);

/*!
 * @brief         XOR a block of data.
 * @param d       Pointer to destination.
 * @param s       Pointer to source.
 */
static void loc_xorBlock( uint8_t *d, const uint8_t *s );

static void setKey( uint8_t *key);
static void encrypt(uint8_t *pld);

/*==============================================================================
                            LOCAL VARIABLES
==============================================================================*/
/** Module global variable for storing the encryption key. */
static CryptoKey     cryptoKey;
static AESECB_Handle aesHandle;

/*==============================================================================
                            GLOBAL VARIABLES
==============================================================================*/
struct aes_128_driver aes_cc1352_driver = {
    setKey,
    encrypt
};


/*==============================================================================
                          LOCAL FUNCTION
==============================================================================*/

/*============================================================================*/
/* loc_xorBlock() */
/*============================================================================*/
static void loc_xorBlock( uint8_t *d, const uint8_t *s )
{
    ((uint8_t*)d)[ 0] ^= ((uint8_t*)s)[ 0];
    ((uint8_t*)d)[ 1] ^= ((uint8_t*)s)[ 1];
    ((uint8_t*)d)[ 2] ^= ((uint8_t*)s)[ 2];
    ((uint8_t*)d)[ 3] ^= ((uint8_t*)s)[ 3];
    ((uint8_t*)d)[ 4] ^= ((uint8_t*)s)[ 4];
    ((uint8_t*)d)[ 5] ^= ((uint8_t*)s)[ 5];
    ((uint8_t*)d)[ 6] ^= ((uint8_t*)s)[ 6];
    ((uint8_t*)d)[ 7] ^= ((uint8_t*)s)[ 7];
    ((uint8_t*)d)[ 8] ^= ((uint8_t*)s)[ 8];
    ((uint8_t*)d)[ 9] ^= ((uint8_t*)s)[ 9];
    ((uint8_t*)d)[10] ^= ((uint8_t*)s)[10];
    ((uint8_t*)d)[11] ^= ((uint8_t*)s)[11];
    ((uint8_t*)d)[12] ^= ((uint8_t*)s)[12];
    ((uint8_t*)d)[13] ^= ((uint8_t*)s)[13];
    ((uint8_t*)d)[14] ^= ((uint8_t*)s)[14];
    ((uint8_t*)d)[15] ^= ((uint8_t*)s)[15];
} /* loc_xorBlock() */

/*============================================================================*/
/* loc_aes_cbc() */
/*============================================================================*/
static bool loc_aes_cbc( uint8_t* pc_in, uint8_t* pc_out, uint16_t length, uint8_t* nonce, bool b_encrypt)
{
    int32_t result = AESECB_STATUS_ERROR;
    AESECB_Operation aes_operation;
    uint16_t len = length;
    uint8_t temp[AES_BLOCK_SIZE];
    uint8_t temp_pc_out[AES_BLOCK_SIZE];

    if ( ((length % AES_BLOCK_SIZE) != 0) || (pc_out == NULL || pc_in == NULL || nonce == NULL) )
        return false;

    memcpy(temp, nonce, AES_BLOCK_SIZE);

    if(b_encrypt)
    {
        while (len)
        {
            result = AESECB_STATUS_ERROR;

            loc_xorBlock(temp, pc_in + length - len);

            /* Perform a single step encrypt operation of the plain text */
            AESECB_Operation_init(&aes_operation);
            aes_operation.key            = &cryptoKey;
            aes_operation.input          = (uint8_t*) temp;
            aes_operation.output         = temp_pc_out + length - len;
            aes_operation.inputLength    = AES_BLOCK_SIZE;

            result = AESECB_oneStepEncrypt(aesHandle, &aes_operation);

            if (result != AESECB_STATUS_SUCCESS) {
                return false;
            }else
            {
                memcpy(temp, pc_out + length - len, AES_BLOCK_SIZE);
                len = len - AES_BLOCK_SIZE;
            }
        }
    }else
    {

        while (len)
        {
            result = AESECB_STATUS_ERROR;
            /* Perform a single step encrypt operation of the plain text */
            AESECB_Operation_init(&aes_operation);
            aes_operation.key            = &cryptoKey;
            aes_operation.input          = (uint8_t*) pc_in + length - len;
            aes_operation.output         = temp_pc_out;
            aes_operation.inputLength    = AES_BLOCK_SIZE;

            result = AESECB_oneStepDecrypt(aesHandle, &aes_operation);

            if (result != AESECB_STATUS_SUCCESS) {
                return false;
            }else
            {
                loc_xorBlock(temp_pc_out, temp);
                memcpy(temp, pc_in + length - len, AES_BLOCK_SIZE);
                memcpy(pc_out + length - len , temp_pc_out, AES_BLOCK_SIZE);
                len = len - AES_BLOCK_SIZE;
            }
        }
    }

    return true;

}/* loc_aes_cbc */

/*============================================================================*/
/* loc_aes_ecb() */
/*============================================================================*/
static bool loc_aes_ecb( uint8_t* pc_in, uint8_t* pc_out,  bool b_encrypt)
{
    int32_t result = AESECB_STATUS_ERROR;
    AESECB_Operation aes_operation;

    if ( pc_out == NULL || pc_in == NULL )
        return false;

    /* Perform a single step encrypt operation of the plain text */
    AESECB_Operation_init(&aes_operation);
    aes_operation.key            = &cryptoKey;
    aes_operation.input          = (uint8_t*) pc_in;
    aes_operation.output         = pc_out;
    aes_operation.inputLength    = AES_BLOCK_SIZE;

    if(b_encrypt)
        result = AESECB_oneStepEncrypt(aesHandle, &aes_operation);
    else
        result = AESECB_oneStepDecrypt(aesHandle, &aes_operation);

    if (result != AESECB_STATUS_SUCCESS) {
        /* Error while encrypting */
        return false;
    }
    else {
        /* Print prompt and the encrypted result */
        return true;
    }/* if */
}/* loc_aes_ecb */

/*============================================================================*/
/* encrypt() */
/*============================================================================*/
static void encrypt(uint8_t *pld){
    loc_aes_ecb(pld, pld, true);
}/* ecb_encrypt */

/*============================================================================*/
/* setKey() */
/*============================================================================*/
static void setKey( uint8_t *key){
    sf_mcu_aes_setKey(key, AES_BLOCK_SIZE);
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

  AESECB_init();
  /* Initialize Crypto driver */
  aesHandle = AESECB_open(0, NULL);
  if (!aesHandle) {
      /* Initialization of the Crypto driver failed */
      b_return = false;
  }

  return b_return;
} /* sf_mcu_aes_init */

/*============================================================================*/
/* sf_mcu_aes_setKey() */
/*============================================================================*/
bool sf_mcu_aes_setKey( uint8_t* pc_key, uint8_t key_length )
{
    /* Initialize the key structure */
    if (CryptoKeyPlaintext_initKey(&cryptoKey, (uint8_t*) pc_key, key_length) != CryptoKey_STATUS_SUCCESS)
        return false;

    return true;
} /* sf_mcu_aes_setKey */

/*============================================================================*/
/* sf_mcu_aes_cbc_encrypt() */
/*============================================================================*/
bool sf_mcu_aes_cbc_encrypt( uint8_t* pc_in, uint8_t*  pc_out, uint16_t length, uint8_t* nonce)
{
    bool b_return = false;

    /* Call the function to encrypt a block */
    b_return = loc_aes_cbc(pc_in, pc_out, length, nonce, true);

    return b_return;
} /* sf_mcu_aes_cbc_encrypt */

/*============================================================================*/
/* sf_mcu_aes_cbc_decrypt() */
/*============================================================================*/
bool sf_mcu_aes_cbc_decrypt( uint8_t* pc_in, uint8_t* pc_out, uint16_t length, uint8_t* nonce)
{
    bool b_return = 0;

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
