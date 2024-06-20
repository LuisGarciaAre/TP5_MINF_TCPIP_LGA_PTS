/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    appgen.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "appgen.h"
#include "Mc32DriverLcd.h"
#include "Mc32gestSpiDac.h"
#include "GesPec12.h"
#include "Generateur.h"
#include "MenuGen.h"
#include "DefMenuGen.h"
#include "Mc32gest_SerComm.h"
#include "Mc32gestI2cSeeprom.h"
#include "driver/tmr/drv_tmr_static.h"
#include "bsp.h"
#include "tcpip/tcpip.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_GEN_DATA appgenData;
S_ParamGen LocalParamGen;   // Declaration de variable de parametres generateur de signal local
S_ParamGen RemoteParamGen;  // Declaration de variable de parametres generateur de signal remote
bool saveParams = false;    // Declaration de variable qui indique la sauvegarde des parametres

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
/********************************************************************************************************
 * Nom de fonction: APPGEN_Callback_TMR1
 * Auteur: [JAR & LGA]
 * Param?tres: 
 *   Entree: Aucune
 *   Sortie: Aucune
 * 
 * Description: Cette fonction est le callback de l'interruption du timer 1. Elle g?re les actions 
 *              p?riodiques telles que le changement d'?tat d'une LED, le traitement antirebonds pour les 
 *              entr?es PEC12 et le bouton S9, et le passage ? l'?tat de service des t?ches de l'application 
 *              apr?s un d?lai de 10 ms.
 * 
 ********************************************************************************************************/
void APPGEN_Callback_TMR1(void)
{
    // Variables locales
    static uint16_t cntStart = 0;           // Variable 16 bits compteur de start
    static uint8_t cntCyclesProgram = 0;    // Variable 8 bits pour compteur de cycles de programme
    
    // Changement d'etat de led 1
    LED1_W = !LED1_R;
    
    // Realise traitement d'antirebonds pour PEC12
    ScanPec12(PEC12_A, PEC12_B, PEC12_PB);
    
    // Realise traitement d'antirebonds pour bouton S9
    scan_btnOk();
    
    // Attente de 3s 
    if(cntStart < VAL_WAIT_3S)
    {
        cntStart++;
    }
    else
    {
        // Realise le APP_STATE_SERVICE_TASKS chaque 10 cycles d'interruption (10 ms))
        if(cntCyclesProgram < VAL_WAIT_10ms)
        {
            cntCyclesProgram++;
        }
        else
        {
            cntCyclesProgram = 0;
            // Indique qu'un changement de etat de machine d'etat principale
            appgenData.intTMR1 = true;
        }
    }
}

/********************************************************************************************************
 * Nom de fonction: APPGEN_Callback_TMR3
 * Auteur: [JAR & LGA]
 * Param?tres: 
 *   Entree: Aucune
 *   Sortie: Aucune
 * 
 * Description: Cette fonction est le callback de l'interruption du timer TMR3. Elle allume une LED, 
 *              ex?cute une fonction d'envoi sur le DAC, puis ?teint la LED.
 * 
 ********************************************************************************************************/
void APPGEN_Callback_TMR3(void)
{
    // Allume led 0
    LED0_W  = 1;
    
    // Execute fonction d'envoi sur DAC
    GENSIG_Execute();
    
    // Etetint led 0
    LED0_W = 0;
}

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/********************************************************************************************************
 * Nom de fonction: APPGEN_GETSET_STR
 * Auteur: [JAR & LGA]
 * Parametres: 
 *   Entree: 
 *     - str : Pointeur vers la chaine de caracteres a traiter (uint8_t*)
 *     - tailleChaine : Taille de la chaine de caracteres (uint32_t)
 *   Sortie: Aucune
 * 
 * Description: Cette fonction copie une chaine de caracteres d'entree dans un buffer local, puis utilise 
 *              cette chaine pour mettre a jour des parametres a distance. Si les parametres sont mis a jour, 
 *              elle renvoie la chaine modifiee et active un drapeau pour mettre a jour l'affichage LCD.
 * 
 ********************************************************************************************************/
void APPGEN_GETSET_STR(uint8_t* str, uint32_t tailleChaine)
{
    char strMessage[APP_GEN_TAILLE_MAX_STR];    // Tableau de sauvegarde du message
    
    // Definie la taille maximale
    if(tailleChaine > APP_GEN_TAILLE_MAX_STR)
    {
        tailleChaine = APP_GEN_TAILLE_MAX_STR;
    }
    
    memcpy(strMessage, str, tailleChaine);  // Copie des carateres re?us sur tableau local
    strMessage[tailleChaine] = '\0';    // Indique la fin de la cahaine
    
    // Indique qu'il faut mettre ? jour si valeurs des parametres recuperes
    appgenData.updateParams = GetMessage((int8_t*)strMessage, &RemoteParamGen, &saveParams);
    
    // Si valeurs recuperes
    if(appgenData.updateParams == true)
    {
        // Modifie la chaine recus en ajoutnt l'quitance de recepetion
        SendMessage((int8_t*)strMessage, &RemoteParamGen, saveParams);
        // Rempli le buffer de l'USB avec la confirmation de r?ception
        memcpy(str, strMessage, strlen(strMessage));
        // Indique la mise a jour du LCD
        appgenData.flagUpdateAffichageLCD = true;
    }  
}

/********************************************************************************************************
 * Nom de fonction: All_leds
 * Auteur: [DBS & LGA]
 * Paramettres: 
 *   Entr?e: state -> variable 8 bits non sign?, ?tat des LEDs (1 pour allumer, 0 pour ?teindre)
 *   Sortie: Aucune
 * 
 * Description: Cette fonction contr?le l'?tat de toutes les LEDs sur la carte. Si l'?tat pass? en 
 *              param?tre est 1, toutes les LEDs sont allum?es. Sinon, si l'?tat est 0, toutes les 
 *              LEDs sont ?teintes.
 * 
 * Remarque: Cette fonction utilise des macros BSP_LEDOn et BSP_LEDOff pour contr?ler les LEDs.
 ********************************************************************************************************/
void All_ledsBSP(bool stateLeds)
{
    BSP_LED tabBSPLeds[8] = {   BSP_LED_0, BSP_LED_1, BSP_LED_2, BSP_LED_3, 
                                BSP_LED_4, BSP_LED_5, BSP_LED_6, BSP_LED_7  };
    uint8_t numLed = 0;
    
    if(stateLeds == false)
    {
        for(numLed = 0; numLed < 8; numLed++)
        {
            BSP_LEDOff(tabBSPLeds[numLed]);
        }
    } 
    else
    {
        for(numLed = 0; numLed < 8; numLed++)
        {
            BSP_LEDOn(tabBSPLeds[numLed]);
        }
    }

}/*Fin de fonction All_leds*/

/********************************************************************************************************
 * Nom de fonction: APPGEN_UPDATE_STATE_USB
 * Auteur: [JAR & LGA]
 * Parametres: 
 *   Entree: 
 *     - state : Etat de la connexion USB (bool)
 *   Sortie: Aucune
 * 
 * Description: Cette fonction met a jour l'etat de la connexion USB et active les flags pour mettre a jour 
 *              l'affichage LCD et les parametres pour le DAC.
 * 
 ********************************************************************************************************/
void APPGEN_UPDATE_STATE_USB(bool state)
{
    appgenData.clientIsConnected = state;
    appgenData.flagUpdateAffichageLCD = true;
    appgenData.updateParams = true;
}/*Fin de fonction APPGEN_UPDATE_STATE_USB*/


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APPGEN_Initialize ( void )

  Remarks:
    See prototype in appgen.h.
 */

void APPGEN_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appgenData.state = APP_GEN_STATE_INIT;
    appgenData.newIp = false;
    
    appgenData.flagUpdateAffichageLCD = true;
    appgenData.state = APP_GEN_STATE_INIT;
    appgenData.clientIsConnected = false;
    appgenData.flagAskToSave = false;
    appgenData.updateParams = false;
    appgenData.intTMR1 = false;
    appgenData.strReceived = false;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APPGEN_Tasks ( void )

  Remarks:
    See prototype in appgen.h.
 */

void APPGEN_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( appgenData.state )
    {
        /* Application's initial state. */
        case APP_GEN_STATE_INIT:
        {
            // Initialisation de l'ecran LCD
            lcd_init();
            
            // Allume le retroeclairage de l'ecran LCD
            lcd_bl_on();
            
            // Initialisation du BSP (Board Support Package)
            BSP_Initialize();

            // Init SPI DAC
            SPI_InitLTC2604();

            // Init I2C EEPROM
            I2C_InitMCP79411();
            
            // Initialisation PEC12
            Pec12Init();
            
            // Initialisation de bouton S9
            btnOKInit();
            
            // Test si USB est connnect? sur kit
            if(appgenData.clientIsConnected == true)
            {
                // Initialisation du generateur et menu en mode remote (usb connecte))
                GENSIG_Initialize(&RemoteParamGen);
                MENU_Initialize(&RemoteParamGen);
                
                // Synchronisation des parametres
                LocalParamGen = RemoteParamGen;
            }
            else
            {
                // Initialisation du generateur et mnue en mode local (usb non connecte))
                GENSIG_Initialize(&LocalParamGen);
                MENU_Initialize(&LocalParamGen);
                
                // Synchronisation des parametres
                RemoteParamGen = LocalParamGen;
            }
            
            // Eteint toutes les leds
            All_ledsBSP(false);
            
            // Active les timers 
            DRV_TMR0_Start();
            DRV_TMR1_Start();
            
            // Changement d'etat de machine d'etat
            appgenData.state = APP_STATE_WAIT;
            break;
        }

        case APP_GEN_STATE_SERVICE_TASKS:
        {
            if(appgenData.clientIsConnected == true)
            {
                MENU_Execute(&RemoteParamGen, true);
            }
            else
            {
                MENU_Execute(&LocalParamGen, false);
            }

            appgenData.state = APP_STATE_WAIT;
            break;
        }        
        case APP_STATE_WAIT:
        {
            if(appgenData.intTMR1 == true)
            {
                appgenData.state = APP_GEN_STATE_SERVICE_TASKS;
                appgenData.intTMR1 = false;
            }
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

//void APPGEN_GetNewAdrIP(IPV4_ADDR ipAddr)
//{
//    appgenData.newIp = true;
//    sprintf(appgenData.tabAdressIP, "IP: %d.%d.%d.%d", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
//}

void IPV4_ADDR_TRANSMIT(IPV4_ADDR ip)
{
    appgenData.newIp = true;
    sprintf(appgenData.str_IP, "IP : %d.%d.%d.%d", ip.v[0], ip.v[1], ip.v[2], ip.v[3]);
} 

// connecter = true 
// non conecter = false
void STATU_COM_SERVER(bool statu_Serveur)
{
    appgenData.clientIsConnected = statu_Serveur;
    appgenData.flagUpdateAffichageLCD = true;
} 

/*******************************************************************************
* 
* @brief Fonction de lecture du buffer USB et envoi des donn?es au LCD.
* @author: samuel pitton 
* @date: 23.05.2024
* 
* Cette fonction lit les donn?es du buffer USB et les envoie ? l'?cran LCD. Elle
* marque ?galement le fait que des donn?es ont ?t? re?ues. Les donn?es re?ues 
* sont copi?es dans une structure de donn?es de l'application et envoy?es au LCD.
* @param str Un pointeur vers le buffer contenant les donn?es ? envoyer au LCD.
* @param strLen La longueur des donn?es ? envoyer au LCD.
* @return void
*******************************************************************************/

void APP_GEN_READBUFFER_USB(uint8_t* str, uint8_t strLen)
{
    appgenData.strReceived = true;
    
    // cpopie l adreese du pointeur recu et sauvegarde dans un "l'app_gen"
    memcpy(appgenData.str_DATA, str,strLen); 
    
    appgenData.str_DATA[strLen] = 0;// met la fin de chaine C    
}


 

/*******************************************************************************
 End of File
 */
