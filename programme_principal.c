#include "LPC17xx.h"
#include <stdio.h>
#include "GLCD.h"
#include "GPIO.h"
#include "cmsis_os.h"                       // CMSIS RTOS header file
#include "Driver_SPI.h"
#include "Driver_USART.h"               // ::CMSIS Driver:USART
#include "string.h"

	extern ARM_DRIVER_USART Driver_USART2;
																												//*******************PROTOTYPES**********************
	void Init_UART();																									//fonctions
	void PWM_Init();
	void PWM_Droite(int rapport_cyclique);
	void PWM_Gauche(int rapport_cyclique);
	void LidarInit();
	void affichage(int a);
	void GetBTString(char *ptab,char delimFin);
	void reception_gant(char v_moy,char delta);
	void reception_joystick(char v_moy,char delta);
	void sendCommand(char * command, int tempo_ms);
	void Init_BT(void);
		
	void automatique (void const *argument);                             // thread function
	void gant (void const *argument);                             
	void joystick (void const *argument);                             
	void reception (void const *argument); 
	                                                       //*******************ID**********************
	osThreadId tid_automatique, tid_gant, tid_joystick, tid_reception;   // thread id
	
	osMutexId ID_mut_MODE;
																												 //*******************DEFINITIONS**********************
	osThreadDef (automatique, osPriorityNormal, 1, 0);                   // thread object	
	osThreadDef (gant, osPriorityNormal, 1, 0);                   
	osThreadDef (joystick, osPriorityNormal, 1, 0);                   	
	osThreadDef (reception, osPriorityNormal, 1, 0); 
	
	osMutexDef(mut_MODE);																									//Mutex
	
	char tab[1], mode = 0;
																													//***********************INITIALISATIONS***********************
int main (void) 
{		
		char  qualite,  angle1,  angle2,  distance1 ,  distance2, trame[5], a, RxBt;// Chaine de caractères (53 max en petite police / 20 max en grande police)
		int somme, etat=0, k;
		long int i;
		short angle, distance;
		
		osKernelInitialize ();  
		
		Init_UART();
	  Initialise_GPIO(); // init GPIO
		PWM_Init();
		//LidarInit();
		GLCD_Init();						// Initialisation LCD
		GLCD_Clear(White);			// Fond d'ecran en blanc
		
		LPC_GPIO2->FIODIR1=0x01;
		
		//LPC_UART2->THR = 0xA5; // Envoi de la donnée (valeur 0xA5)
			
		//LPC_UART2->THR = 0x20; // Envoi de la donnée (valeur 0x20)
		
		LPC_GPIO2->FIODIR0|=0x04;
		
		//GLCD_DisplayString(1,0,0,"Recherche de connexion...");
		
		Driver_USART2.Receive(tab,1);
		while(Driver_USART2.GetRxCount()<1);
		
		//GLCD_DisplayString(1,0,0,"Connexion établie                ");
		
		if(tab[0] == '1') 
		{
			mode = 1; //mode auto
			//GLCD_DisplayString(1,0,0,"***Mode Automatique***                           ");
		}
		else if(tab[0] == '2') 
		{
			mode = 2; //mode gant
			//GLCD_DisplayString(1,0,0,"***Mode Force***                           ");
		}
		else if(tab[0] == '3')
		{
			mode = 3; //mode nunchuck
			//GLCD_DisplayString(1,0,0,"***Mode Wii***                           ");
		}
		
		ID_mut_MODE = osMutexCreate(osMutex(mut_MODE));
		
		tid_automatique = osThreadCreate (osThread(automatique), NULL);
		tid_gant = osThreadCreate (osThread(gant), NULL);
		tid_joystick = osThreadCreate (osThread(joystick), NULL);
		tid_reception = osThreadCreate (osThread(reception), NULL);
	
		osKernelStart ();                         // start thread execution 
		osDelay(osWaitForever);
}
																												//***********************MODE AUTOMATIQUE***********************
void automatique (void const *argument)
{
	int etat = 0;
	
		while(1)
		{
			if(mode != 1) 
			{
				osSignalWait(0x0001,osWaitForever);
				PWM_Droite(0);
				PWM_Gauche(0);
			}
			//GLCD_DisplayString(5,0,0,"               o                   ");
//			UART_PutChar(0xA5);
			 
			switch (etat)
		{
         case 0: 
					 PWM_Droite(70);
					 PWM_Gauche(70);
					 affichage(0);
					 osDelay(1600);
					 etat = 1;
					 
					 break;
				 
				 case 1: 
					 PWM_Droite(70);
					 PWM_Gauche(50);
				   affichage(1);
					 osDelay(700);
				   etat = 2;
					 
					 break;
				 
				 case 2: 
					 PWM_Droite(65);
					 PWM_Gauche(65);
				   affichage(2);
					 osDelay(3000);
				   etat = 3;
					 
				   break;
					 
				case 3: 
					 PWM_Droite(70);
					 PWM_Gauche(50);
				   affichage(3);
					 osDelay(600);
				   etat = 0;
				 
					 break;	 
    }
		}
}
																												//***********************MODE GANT***********************
void gant (void const *argument)
{
	char v_moy,delta,v_moteurD,v_moteurG,texte[50];
	
	while(1)
	{
	if(mode != 2) 
	{
		osSignalWait(0x0002,osWaitForever);
		PWM_Droite(0);
	  PWM_Gauche(0);
	}
	
	PWM_Droite(0);
	PWM_Gauche(0);
	
	osMutexWait(ID_mut_MODE, osWaitForever);
	reception_gant(v_moy,delta);
	osMutexRelease(ID_mut_MODE);
	
	v_moteurD = v_moy - delta;
	v_moteurG = v_moy + delta;
	
	sprintf(texte,"MoteurD = %c      MoteurG = %c",v_moteurD,v_moteurG);
	GLCD_DisplayString(5,0,0,texte);
	}
}
																												//***********************MODE JOYSTICK***********************
void joystick (void const *argument)
{
	char v_moy,delta,v_moteurD,v_moteurG,texte[50];
	
	while(1)
	{
	if(mode != 3) 
	{
		osSignalWait(0x0004,osWaitForever);
		PWM_Droite(0);
	  PWM_Gauche(0);
	}
	//GLCD_DisplayString(5,0,0,"(informations liees au joystick)                 ");
	PWM_Droite(0);
	PWM_Gauche(0);
	
	osMutexWait(ID_mut_MODE, osWaitForever);
	reception_gant(v_moy,delta);
	osMutexRelease(ID_mut_MODE);
	
	v_moteurD = v_moy - delta;
	v_moteurG = v_moy + delta;
	
	sprintf(texte,"MoteurD = %c      MoteurG = %c",v_moteurD,v_moteurG);
	GLCD_DisplayString(5,0,0,texte);
	}
}
																													//***********************CHANGEMENT DE MODE***********************
void reception (void const *argument)
{
	while(1)
	{
	Driver_USART2.Receive(tab,1);
		
	if(Driver_USART2.GetRxCount()<1)
	{
		
	osMutexWait(ID_mut_MODE, osWaitForever);
		if(tab[0] == '1') 
		{
			mode = 1; //mode auto
			//GLCD_DisplayString(1,0,0,"***Mode Automatique***                           ");
			osSignalSet(tid_automatique,0x0001);
		}
		else if(tab[0] == '2') 
		{
			mode = 2; //mode gant
			//GLCD_DisplayString(1,0,0,"***Mode Force***                           ");
			osSignalSet(tid_gant,0x0002);
		}
		else if(tab[0] == '3')
		{
			mode = 3; //mode nunchuck
			//GLCD_DisplayString(1,0,0,"***Mode Wii***                           ");
			osSignalSet(tid_joystick,0x0004);
		}
	 osMutexRelease(ID_mut_MODE);
		
	}
}
}
		/*while(1)
		{	
			
			
			a=UART_GetChar();
			LPC_GPIO2->FIOPIN0|=0x04;
			
		if(a == 0xA5)
		{	
			
			a=UART_GetChar();
			if(a == 0x5A)
			{
				for(k=0; k<5 ; k++)
				{
					trame[k] = UART_GetChar();
					
				}
			}
		}
		
			if(trame[0]!=0x00)
			{
				qualite=(trame[0] >> 2);
				angle1=(trame[1] >> 1);
				angle2=trame[2];
				distance1=trame[3];
				distance2=trame[4];
			
				angle=((angle2<<7) | angle1) / 64.00;
				distance=((distance2<<8) | distance1) /4.00;
			}
		
		}*/
																																//***********************FONCTIONS***********************
void PWM_Init()
{
	  LPC_PINCON->PINSEL4 |= 0x00000005;
		LPC_SC->PCONP = LPC_SC->PCONP | 0x00000040;	// enable PWM1
		
		LPC_PWM1->PR = 11;  // prescaler
		LPC_PWM1->MR0 = 99;
		
	  LPC_PWM1->MCR = LPC_PWM1->MCR | 0x00000002; // Timer relancé quand MR0 repasse à 0
		LPC_PWM1->LER = LPC_PWM1->LER | 0x0000000F;  // ceci donne le droit de modifier dynamiquement la valeur du rapport cyclique
	                                             // bit 0 = MR0    bit3 = MR3
		LPC_PWM1->PCR = LPC_PWM1->PCR | (7<<9);  // autorise les sorties PWM1.1 et PWM1.2  
		
		LPC_PWM1->MR1 = 0; //remise à 0 des compteurs
		LPC_PWM1->MR2 = 0;
		LPC_PWM1->MR3 = 39;
		
		LPC_PWM1->TCR = 1;  /*validation de timer 1 et reset counter */
}




void PWM_Droite(int rapport_cyclique)
{
		LPC_PWM1->MR1 = rapport_cyclique;    // ceci ajuste la duree de l'état haut pour le moteur droit
}

void PWM_Gauche(int rapport_cyclique)
{
		LPC_PWM1->MR2 = rapport_cyclique;    // ceci ajuste la duree de l'état haut pour le moteur gauche
}

void LidarInit(int rapport_cyclique)
{
		LPC_PINCON->PINSEL7 = LPC_PINCON->PINSEL7 | 0x00300000;
		LPC_PWM1->MR3 = rapport_cyclique;
}

void affichage(int a)
{
	char Chaine_char[53] = "";
	
	sprintf(Chaine_char, "Etat = %d", a);	// creation chaine texte
	//GLCD_DisplayString(2,0,1,Chaine_char);		// affichage chaine ligne 0, colonne 0, petite police
}

void Init_UART()
{
	Driver_USART2.Initialize(NULL);
	Driver_USART2.PowerControl(ARM_POWER_FULL);
	Driver_USART2.Control(	ARM_USART_MODE_ASYNCHRONOUS |
							ARM_USART_DATA_BITS_8		|
							ARM_USART_STOP_BITS_1		|
							ARM_USART_PARITY_NONE		|
							ARM_USART_FLOW_CONTROL_NONE,
							115200);
	Driver_USART2.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART2.Control(ARM_USART_CONTROL_RX,1);
}

void reception_gant(char v_moy,char delta)
{
	while(Driver_USART2.GetStatus().tx_busy==1);
	Driver_USART2.Send("0",1);
	
	Driver_USART2.Receive(&v_moy,1);
	while(Driver_USART2.GetRxCount()<1);
	
	while(Driver_USART2.GetStatus().tx_busy==1);
	Driver_USART2.Send("1",1);
	
	Driver_USART2.Receive(&delta,1);
	while(Driver_USART2.GetRxCount()<1);
}

void reception_joystick(char v_moy,char delta)
{
	while(Driver_USART2.GetStatus().tx_busy==1);
	Driver_USART2.Send("0",1);
	
	Driver_USART2.Receive(&v_moy,1);
	while(Driver_USART2.GetRxCount()<1);
	
	while(Driver_USART2.GetStatus().tx_busy==1);
	Driver_USART2.Send("1",1);
	
	Driver_USART2.Receive(&delta,1);
	while(Driver_USART2.GetRxCount()<1);
}
void sendCommand(char * command, int tempo_ms)
{
	int len;
	len = strlen (command);
	Driver_USART2.Send(command,len); // send the read character to the esp8266
	osSignalWait(0x02, osWaitForever);		// sommeil fin emission
	osDelay(tempo_ms);		// attente traitement retour
}
void Init_BT(void)
{
	// reset module
	sendCommand("AT+RST\r\n",7000); 
	  
  // configure as Station 
	sendCommand("AT+CWMODE=1\r\n",2000);
	
	// disconnect from any Access Point
	sendCommand("AT+CWQAP\r\n",2000); 

	//Connect to YOUR Access Point
	sendCommand("AT+CWJAP=\"AndroidAP\",\"mag990630\"\r\n",7000); 
}

