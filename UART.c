// Bibliotheque UART

#include "LPC17xx.h"

// init UART 150 kbauds

void Init_UART()
{
LPC_PINCON->PINSEL4 = 0x000A0000; // Broche P2.8 pour TX2 et P2.9 pour RX2
LPC_SC->PCONP |= 0x01000000;
LPC_UART2->LCR |= 0x03;	// configuration


// R�glage de la vitesse de transmission
LPC_UART2->LCR |= 0x80; // For�age bit DLAB=1 (Demande autorisation de modification)
LPC_UART2->DLM = 0; // Pas de sur-division de PCLK
LPC_UART2->DLL = 10; // Division principale par 10 de PCLK
LPC_UART2->FDR = 0xE5; // Division fractionnaire par 1,35 (DIVADDVAL=5 et MULVAL=14)
LPC_UART2->LCR &= 0x7F; // For�age bit DLAB=0 (Fin d'autorisation de modification)
}


char UART_GetChar (void)
{
while ((LPC_UART2->LSR & 0x01)==0); // Attente caract�re re�u (attente tant que bit RDR=0)
																		// RDR : Receiver Data Ready (passe � 1 � chaque nouvelle r�ception)
return  LPC_UART2->RBR; 						// Lecture registre r�ception (remet automatiquement le bit RDR � 0)
}



void UART_PutChar (unsigned char caractere)
{
	while ((LPC_UART2->LSR & 0x20)==0); // Attente buffer libre (attente tant que bit THRE=0)
																			//(passe � 1 quand un buffer emission vide)
	// Envoi de la donn�e (valeur 0x75)
	LPC_UART2->THR = caractere;
}
