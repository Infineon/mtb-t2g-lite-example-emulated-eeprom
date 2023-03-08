/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the <CE235226> XMC7000 MCU Emulated EEPROM
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_em_eeprom.h"
#include "cycfg_system.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Logical Size of Emulated EEPROM in bytes. */
#define LOGICAL_EEPROM_SIZE     (15u)
#define LOGICAL_EEPROM_START    (0u)

/* Location of reset counter in Em_EEPROM. */
#define RESET_COUNT_LOCATION    (13u)
/* Size of reset counter in bytes. */
#define RESET_COUNT_SIZE        (2u)

/* ASCII "9" */
#define ASCII_NINE              (0x39)

/* ASCII "0" */
#define ASCII_ZERO              (0x30)

/* ASCII "P" */
#define ASCII_P                 (0x50)

#define GPIO_LOW                (0u)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void HandleError(uint32_t status, char *message);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* EEPROM configuration and context structure. */
cy_stc_eeprom_config_t Em_EEPROM_config =
{
        .eepromSize = MY_EEPROM_SIZE,
        .simpleMode = MY_EEPROM_SIMPLEMODE,
        .blockingWrite = MY_EEPROM_BLOCKINGMODE,
        .redundantCopy = MY_EEPROM_REDUNDANT_COPY,
        .wearLevelingFactor = MY_EEPROM_WEARLEVELING_FACTOR,
        .userFlashStartAddr = CY_WFLASH_SM_SBM_BASE,      /*user can map the EEPROM to any Work Flash region*/
};

cy_stc_eeprom_context_t Em_EEPROM_context;

/* RAM arrays for holding EEPROM read and write data respectively. */
uint8_t eepromReadArray[LOGICAL_EEPROM_SIZE];
uint8_t eepromWriteArray[LOGICAL_EEPROM_SIZE] = { 0x50, 0x6F, 0x77, 0x65, 0x72, 0x20, 0x43, 0x79, 0x63, 0x6C, 0x65, 0x23, 0x20, 0x30, 0x30};
                                                /* P, o, w, e, r, , C, y, c, l, e, #, , 0, 0 */


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* System entry point. This function configures and initializes UART and
* Emulated EEPROM, reads the EEPROM content, increments it by one and writes the
* new content back to EEPROM.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    int count;
    /* Return status for EEPROM. */
    cy_en_em_eeprom_status_t eepromReturnValue;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG,CYBSP_LED_STATE_OFF);

    /* gpio init failed. Stop program execution */
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    printf("EmEEPROM demo \r\n");

    /*Initialize the EEPROM memory*/
    eepromReturnValue = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);
    HandleError(eepromReturnValue, "Emulated EEPROM Initialization Error \r\n");

    /* Read 15 bytes out of EEPROM memory. */
    eepromReturnValue = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START, eepromReadArray,
                                              LOGICAL_EEPROM_SIZE, &Em_EEPROM_context);
    HandleError(eepromReturnValue, "Emulated EEPROM Read failed \r\n");

    /* If first byte of EEPROM is not 'P', then write the data for initializing the EEPROM content.*/
    if(ASCII_P != eepromReadArray[0])
    {
        /* Write initial data to EEPROM. */
        eepromReturnValue = Cy_Em_EEPROM_Write(LOGICAL_EEPROM_START,
                                                   eepromWriteArray,
                                                   LOGICAL_EEPROM_SIZE,
                                                   &Em_EEPROM_context);
        HandleError(eepromReturnValue, "Emulated EEPROM Write failed \r\n");
    }

    else
    {
        /* The EEPROM content is valid. Increment Counter by 1. */
        eepromReadArray[RESET_COUNT_LOCATION+1]++;

        /* Counter is in ASCII, so handle overflow. */
        if(eepromReadArray[RESET_COUNT_LOCATION+1] > ASCII_NINE)
        {
            /* Set lower digit to zero. */
            eepromReadArray[RESET_COUNT_LOCATION+1] = ASCII_ZERO;
            /* Increment upper digit. */
            eepromReadArray[RESET_COUNT_LOCATION]++;

            /* only increment to 99. */
            if(eepromReadArray[RESET_COUNT_LOCATION] > ASCII_NINE)
            {
                eepromReadArray[RESET_COUNT_LOCATION] = ASCII_NINE;
                eepromReadArray[RESET_COUNT_LOCATION+1] = ASCII_NINE;
            }
        }

        /* Only update the two count values in the EEPROM. */
        eepromReturnValue = Cy_Em_EEPROM_Write(RESET_COUNT_LOCATION,
                                                   &eepromReadArray[RESET_COUNT_LOCATION],
                                                   RESET_COUNT_SIZE,
                                                   &Em_EEPROM_context);
        HandleError(eepromReturnValue, "Emulated EEPROM Write failed \r\n");
        }

    /* Read contents of EEPROM after write. */
    eepromReturnValue = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START,
                                          eepromReadArray, LOGICAL_EEPROM_SIZE,
                                          &Em_EEPROM_context);
    HandleError(eepromReturnValue, "Emulated EEPROM Read failed \r\n" );

    for(count = 0; count < LOGICAL_EEPROM_SIZE ; count++)
    {
        printf("%c",eepromReadArray[count]);
    }
    printf("\r\n");

    for (;;)
    {

    }
}

/*******************************************************************************
* Function Name: HandleError
********************************************************************************
*
* Summary:
* This function processes unrecoverable errors such as any component
* initialization errors etc. In case of such error the system will
* stay in the infinite loop of this function.
*
* Parameters:
* uint32_t status: contains the status.
* char* message: contains the message that is printed to the serial terminal.
*
* Note: If error occurs interrupts are disabled.
*
*******************************************************************************/
void HandleError(uint32_t status, char *message)
{

    if(CY_EM_EEPROM_SUCCESS != status)
    {
        if(CY_EM_EEPROM_REDUNDANT_COPY_USED != status)
        {
            cyhal_gpio_write(CYBSP_USER_LED, false);
            __disable_irq();

            if(NULL != message)
            {
                printf("%s",message);
            }

            while(1u);
        }
        else
        {
            printf("%s","Main copy is corrupted. Redundant copy in Emulated EEPROM is used \r\n");
        }

    }
}

/* [] END OF FILE */
