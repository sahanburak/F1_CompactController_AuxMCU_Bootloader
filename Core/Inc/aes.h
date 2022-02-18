/******************************************************************************
 *
 *   Copyright (C) Rota Teknik 2019 All Rights Reserved. Confidential
 *
 **************************************************************************//**
 * @file        aes.h
 * @brief       
 *
 * @author      Burak Sahan
 * @date        Dec 1, 2020
 *
 * @ingroup     F1_CompactController_IOExp_Bootloader
 * @{
 *****************************************************************************/
#ifndef INC_AES_H_
#define INC_AES_H_
/*============================================================================*/
/* Includes                                                                   */
/*============================================================================*/
#include <stdint.h>
/*============================================================================*/
/* Forward declarations                                                       */
/*============================================================================*/
extern  const uint8_t 	io_expander_key[];
#define AES_KEY			io_expander_key
/*============================================================================*/
/* Constants and macros                                                       */
/*============================================================================*/

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/

/*============================================================================*/
/* Global data                                                                */
/*============================================================================*/

/*============================================================================*/
/* Declarations                                                               */
/*============================================================================*/
// #define the macros below to 1/0 to enable/disable the mode of operation.
//
// CBC enables AES encryption in CBC-mode of operation.
// ECB enables the basic ECB 16-byte block algorithm. Both can be enabled simultaneously.

// The #ifndef-guard allows it to be configured before #include'ing or at compile time.
#ifndef CBC
  #define CBC 1
#endif

/*#ifndef ECB
  #define ECB 1
#endif
*/

#define AES128 1
//#define AES192 1
//#define AES256 1

#if defined(ECB) && (ECB == 1)

void AES_ECB_encrypt(const uint8_t* input, const uint8_t* key, uint8_t *output, const uint32_t length);
void AES_ECB_decrypt(const uint8_t* input, const uint8_t* key, uint8_t *output, const uint32_t length);

#endif // #if defined(ECB) && (ECB == !)


#if defined(CBC) && (CBC == 1)

void AES_CBC_encrypt_buffer(uint8_t* output, uint8_t* input, uint32_t length, const uint8_t* key, const uint8_t* iv);
void AES_CBC_decrypt_buffer(uint8_t* output, uint8_t* input, uint32_t length, const uint8_t* key, const uint8_t* iv);

#endif // #if defined(CBC) && (CBC == 1)

#endif /* INC_AES_H_ */
/**@}*/
/******************************************************************************/
/*   Copyright (C) Rota Teknik 2019,  All Rights Reserved. Confidential.      */
/******************************************************************************/
