#include <stdbool.h>        //%%%%%%%%%%%%%%____MIA PROVA___%%%%%%%%
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "bsp.h"
#include "hardfault.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_pwr_mgmt.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_channel_config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"
#include "nrf_gpio.h" //potrebbe non servire
#include "nrf_drv_saadc.h"
//#include "nrf_drv_clock.h" //da qualche errore, rimosso il .c (da lucchesini posso comunque commentarlo e non da errore)
#include "nrf_nvmc.h"
#include "app_timer.h"

#include "nrf_drv_twi.h"
//includere tutte le librerie per i sensori
//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% CONSTANT DEFINITION %%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
#define TWI_ADDRESSES   127         //Number of possible TWI addresses.
#define APP_ANT_OBSERVER_PRIO   1   // Application's ANT observer priority. You shouldn't need to modify this value.
#define SAADC_BATTERY   0           //Canale tensione della batteria
#define TIMEOUT_VALUE   25   //1000       // 25 mseconds timer time-out value. Interrupt a 40Hz
#define START_ADDR  0x00011200      //indirizzo di partenza per salvataggio dati in memoria non volatile
#define LED             07

#define SECONDS_2           2       //interval of 2 seconds
#define SECONDS_20          20      //interval of 20 seconds
//=============================================================================================================================================================================

struct data{
    uint8_t giorno;
    uint8_t mese;
    uint8_t anno;
};

struct val_campionati{
    float Temp;
    float Hum;
    float Pres;
    float PM2p5;
    float PM1p0;    //controlla che sia corretto, scegliere quali valori guardare di PM
    float CO2;
    float CO;
    float NO2;
    float VOC;
};

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% GLOBAL VARIABLE DEFINITION %%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
volatile int count=0, stato=0, i=0;
const float deltat = 0.025;
int notshown = 1;           // per il log

int rtc_count = 0;

nrf_saadc_value_t sample;   //variabile per campionamento 
ret_code_t err_code;        //variabile per valore di ritorno

//PUò ESSERE CHE QUESTE DUE VARIABILI NON MI SERVANO POI (al momento serve per ant)
volatile int cal_rec = 0;   //impedisce che si faccia più di una calibrazione, se se ne vuole fare un'altra bisogna spegnere e riaccendere
volatile int flag_cal = 0;  //flag che definisce calibrazione in corso
//=============================================================================================================================================================================

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% TWI COMMUNICATION %%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
//TWI instance ID.
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

//TWI instance
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//TWI initialization
void twi_init (void)
{
    ret_code_t err_code;
    const nrf_drv_twi_config_t twi_config = {
       .scl                = 19,
       .sda                = 18,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}
//=============================================================================================================================================================================

APP_TIMER_DEF(m_repeated_timer_id);     /*Handler for repeated timer */

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% SAADC %%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)  //non serve
{
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    nrf_saadc_channel_config_t channel2_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(SAADC_BATTERY, &channel_config);
    APP_ERROR_CHECK(err_code);
	
    err_code = nrf_drv_saadc_channel_init(1, &channel2_config);
    APP_ERROR_CHECK(err_code); 
}
//=============================================================================================================================================================================

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void utils_setup(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_LEDS,
                        NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}
//=============================================================================================================================================================================

/*
*   Si può aggiungere funzione per lettura e scrittura
*   in memoria non volatile
*/

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% ANT COMMUNICATION %%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
void ant_send(int campione, int counter, int quat1, int quat2, int quat3, int quat4)
{
    uint8_t message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE];

    memset(message_payload, 0, ANT_STANDARD_DATA_PAYLOAD_SIZE);
    // Assign a new value to the broadcast data.
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 8] = 11; //DEVICENUMBER
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 7] = campione; 
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 6] = 0;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 5] = counter;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 4] = quat1;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 3] = quat2;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 2] = quat3;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = quat4;

    // Broadcast the data.
    ret_code_t err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,
                                                    ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                    message_payload);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Messaggio numero %d, ret code ant broadcast: %d", counter, err_code);
}

//Function for handling stack event
void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
    if (p_ant_evt->channel == BROADCAST_CHANNEL_NUMBER && flag_cal == 0)  //durante la calibrazione ignora tutti i messaggi che arrivano dal master
    {
        switch (p_ant_evt->event)
        {
            case EVENT_RX:
                if (p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_BROADCAST_DATA_ID)
                {
                    if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0x80 )   //se il primo byte del payload è zero e l'ultimo è 128
                    { 									
                        stato=0;	//ferma l'acquisizione												
                        nrf_gpio_pin_set(LED);
                        sd_ant_pending_transmit_clear (BROADCAST_CHANNEL_NUMBER, NULL); //svuota il buffer, utile per una seconda acquisizione
                        NRF_LOG_INFO("Ricevuto messaggio di stop acquisizione");
                          
                    }
                    if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0x00)   //se il primo byte del payload è zero avvia l'acquisizione
                    {
                        NRF_LOG_INFO("Inizio acquisizione");	
                        sd_ant_pending_transmit_clear (BROADCAST_CHANNEL_NUMBER, NULL); //svuota il buffer, utile per una seconda acquisizione
                        nrf_gpio_pin_clear(LED);
                        count=0;
                        i=0;
                        stato = 1;
                    }   
                    if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0xFF && cal_rec == 0)   
                    {       //messaggio di inizio calibrazione
                        cal_rec = 1;
                        stato = 0;
                        //calibrazione();
                        //salva_calib_flash();
                    }		
                }
                break;
            case EVENT_RX_SEARCH_TIMEOUT:	   //in caso di timeout riapre il canale
                err_code = sd_ant_channel_open(BROADCAST_CHANNEL_NUMBER);
                APP_ERROR_CHECK(err_code); 
                break;							

            default:
                break;
        }
    }
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);

//Function for ANT stack initialization.
static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);
}

// Function for setting up the ANT module to be ready for RX broadcast.
static void ant_channel_rx_broadcast_setup(void)
{
    ant_channel_config_t broadcast_channel_config =
    {
        .channel_number    = BROADCAST_CHANNEL_NUMBER,
        .channel_type      = CHANNEL_TYPE_SLAVE,
        .ext_assign        = 0x00,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .device_number     = CHAN_ID_DEV_NUM,
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_NETWORK_NUM,
    };

    ret_code_t err_code = ant_channel_init(&broadcast_channel_config);
    APP_ERROR_CHECK(err_code);

    // Open channel.
    err_code = sd_ant_channel_open(BROADCAST_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}
//=============================================================================================================================================================================

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% TIMER HANDLER %%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

static void repeated_timer_handler(void * p_context)  //app timer, faccio scattare ogni un secondo
{ 
    rtc_count ++;
    //controllo della batteria, ogni quanto? come il campionamento, e come fare controllo? voltage divider?
    err_code = nrf_drv_saadc_sample_convert(SAADC_BATTERY, &sample);   //lettura ADC
    APP_ERROR_CHECK(err_code);

    //1 sec
    //Lettura dati VOC
    
    //2 sec
    if ((rtc_count % SECONDS_2) == 0)
    {
        //invio ant
    }

    //20 sec
    if ((rtc_count % SECONDS_20) == 0)
    {
        //campiono tutti i valori, flag e si fa nel main, confronto con umidità
        //rtc_count = 0 se non mi serve intervallo più grande
    }
									                  
}
//=============================================================================================================================================================================


int main(void)
{
    nrf_gpio_cfg_output(LED);
    nrf_gpio_pin_set(LED);
    //Inizializzazione di tutte le componenti
    log_init();
    saadc_init();
    twi_init();
    softdevice_setup();
    ant_channel_rx_broadcast_setup();
    utils_setup();
    NRF_LOG_INFO("ANT Broadcast started.");


    sd_ant_channel_radio_tx_power_set(BROADCAST_CHANNEL_NUMBER, RADIO_TX_POWER_LVL_4, NULL); 	//potenza trasmissione
    uint8_t  message_addr[ANT_STANDARD_DATA_PAYLOAD_SIZE];
    memset(message_addr, 11, ANT_STANDARD_DATA_PAYLOAD_SIZE); //DEVICENUMBER

    //err_code = nrf_drv_saadc_sample_convert(SAADC_REFERENCE, &sample);  //campiona tensione alimentazione (1.8V)
    message_addr[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = sample;  //invia campione nell'ultimo byte del payload
		
    err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,         //invia messaggio di accensione
                                           ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                           message_addr);
    
    app_timer_init();
    err_code = app_timer_create(&m_repeated_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            repeated_timer_handler);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_clear(LED);
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(TIMEOUT_VALUE), NULL);
    APP_ERROR_CHECK(err_code);	

    /* 
     * Inizializzazione dei sensori e settaggio dei parametri con le funzioni apposite
     * Tenere traccia del giorno e vedere quanto è passato per fare pulizia della ventola
     * 
     */

    // Main loop.
    for (;;)
    {
        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
    }
}


/**
 *@}
 **/
